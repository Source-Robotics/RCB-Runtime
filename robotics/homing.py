"""
Homing routines for joint calibration.

Two homing strategies, selected per joint via the has_hall_sensor XML field:

  Hall-sensor joints (e.g. PAR6 J6, has_hall_sensor=1):
      Motor runs in HALL_INDEX mode (Send_data_pack_HALL).
      Endstop detected when Motor.HALL_trigger == 0 OR Motor.hall_index == 1.

  Endstop/stall joints (all others, has_hall_sensor=0):
      Motor runs in velocity mode (Send_data_pack_1 with Speed set).
      Endstop detected when position stops changing AND current exceeds
      homing_current_ma threshold for N consecutive ticks.

After detection both strategies (two-pass / single-pass):

  Two-pass mode (default, mirrors Prusa sensorless homing):
      1. Fast first approach  → endstop hit  → dwell (motor stops) → back off
         for backoff_s seconds → pause (motor stops again).
      2. Slow second approach → endstop hit  → settle.
      The dwell and pause phases ensure the motor is fully stationary before
      direction changes, preventing false triggers from momentum/current spikes.
      The second pass uses rehome_speed_factor × the original homing speed for a
      more repeatable trigger point.

  Single-pass mode (two_pass_homing[j] == False):
      Endstop detection → settle immediately (original behaviour).

  Release (optional, per joint, stall joints only):
    After the final endstop detection, command a controlled current (or zero)
    for a configurable duration.  This relieves gearbox preload so the joint
    relaxes to its true resting endstop position.  The encoder position is
    latched at a configurable point during the release window.
    Controlled by homing_release_current_ma, homing_release_duration_s,
    homing_release_sample_pct.  All NaN = skip (default).

  Settle: Send Idle for _SETTLE_TIME seconds, then update
    joint.master_position to the endstop encoder tick so that
    get_joint_position(endstop_tick) == home_offset[j] radians.

  Post-home move (optional, per joint):
    After the settle period a joint may be driven to a specific angle before the
    next joint starts homing.  Controlled by post_home_position[j] (radians).
    NaN = skip (default).  Speed is post_home_speeds[j].
    Useful when a downstream joint can only reach its endstop if the upstream
    joint is already in a known clear position.

Homing order
------------
  By default joints are homed in index order (0, 1, 2 …).  Override by setting
  home_sequence directly, or via the per-joint XML field <homing_order>N</homing_order>
  (1-based integer).  Joints without a value are appended last in index order.

  Parallel homing is supported via a homing sequence file (see parse_homing_sequence).
  Each step can home multiple joints simultaneously with optional pre-moves.

Integration with RTI.py
-----------------------
1.  Instantiate once after XML loading:
        homing_ctrl = HomingController.from_config(cfg, joint_num)

2.  On HOMING mode transition:
        homing_ctrl.enter(Motor, joint_num, load_motor_configs, motor_config)

3.  Each RT tick instead of send_motor_commands():
        result = homing_ctrl.step(Motor, Joint, shared_data)
        if result == 'done':   pending_mode_request = ENABLED
        if result == 'failed': pending_mode_request = ERROR

4.  On leaving HOMING mode:
        homing_ctrl.exit(Motor, joint_num, load_motor_configs, motor_config)
"""

import logging
import math
import numpy as np

# ---------------------------------------------------------------------------
# Homing status codes written to shared_data.rti["homing_status"][j]
# ---------------------------------------------------------------------------
STATUS_IDLE    = 0
STATUS_RUNNING = 1
STATUS_DONE    = 2
STATUS_FAILED  = 3


# ---------------------------------------------------------------------------
# Per-joint homing state machine
# ---------------------------------------------------------------------------

class JointHomingState:
    """
    Self-contained homing state machine for a single joint.

    Encapsulates all per-joint state: phase tracking, stall detection,
    backoff, settle, and post-home move logic.  Call tick() every RT cycle.
    """

    _HOMING    = 'homing'
    _DWELL     = 'dwell'       # brief stop after endstop hit, before backoff
    _BACKOFF   = 'backing_off'
    _PAUSE     = 'pause'       # brief stop after backoff, before pass 2
    _RELEASE   = 'releasing'   # current-command phase to release gearbox preload
    _SETTLE    = 'settling'
    _POST_MOVE = 'post_moving'
    _PRE_CLEAR = 'pre_clearing'  # Hall triggered too early — back off before real approach
    _DONE      = 'done'
    _FAILED    = 'failed'

    # --- Time-based constants (seconds) --- converted to ticks via dt
    _STALL_TIME            = 0.08   # position must not move for this long (secondary gate)
    _CURRENT_CONFIRM_TIME  = 0.08   # current must be above threshold for this long to trigger
    _SETTLE_TIME           = 0.08   # idle hold after endstop detection
    _POST_MOVE_STABLE_TIME = 0.08   # must hold within tolerance for this long
    _STARTUP_GUARD_TIME    = 0.15   # ignore stall triggers during startup (covers acceleration transients)
    _DWELL_TIME            = 0.08   # stop after endstop hit before reversing
    _PAUSE_TIME            = 0.15   # stop after backoff before starting pass 2
    # If a Hall trigger fires within this many ticks the motor was already sitting
    # on/near the sensor — back off and re-approach rather than accepting the position.
    _HALL_PRE_CLEAR_GUARD_S = 0.5  # 500 ms at 12000 ticks/s ≈ 6000 ticks of travel

    # The motor Ilim is set to homing_current_ma, so the motor saturates just below
    # that value. Detect at this fraction of Ilim so the threshold is always reachable.
    _CURRENT_DETECT_FRACTION = 0.70

    # Max allowed difference between pass 1 and pass 2 endstop positions (encoder ticks).
    # If the two passes disagree by more than this, homing is unreliable and fails.
    _TWO_PASS_MAX_DIFF_TICKS = 200

    # Position arrival tolerance for post-home move (encoder ticks, not time)
    _POST_MOVE_TOL_TICKS   = 50

    def __init__(self, j, homing_speed, homing_direction, homing_current_ma,
                 has_hall_sensor, dt, timeout_s, two_pass, backoff_s,
                 rehome_speed_factor, home_offset,
                 post_home_position=float('nan'), post_home_speed=None,
                 is_arm_joint=True,
                 normal_ilim=None, normal_velocity_limit=None,
                 release_current_ma=float('nan'), release_duration_s=float('nan'),
                 release_sample_pct=float('nan'),
                 two_pass_max_diff=float('nan')):
        """
        Args:
            j                    (int)   : joint index
            homing_speed         (float) : motor speed [ticks/s] during homing
            homing_direction     (int)   : 0 = positive motor dir, 1 = negative
            homing_current_ma    (float) : current threshold [mA] for stall detection
            has_hall_sensor      (int)   : 1 if joint uses Hall sensor, 0 for stall
            dt                   (float) : RT loop period in seconds
            timeout_s            (float) : max time allowed for homing [seconds]
            two_pass             (bool)  : True = Prusa-style back-off + slow second approach
            backoff_s            (float) : time to reverse after first hit [seconds]
            rehome_speed_factor  (float) : second-pass speed = first-pass × factor
            home_offset          (float) : joint angle [rad] at endstop
            post_home_position   (float) : angle [rad] to move to after homing (NaN = skip)
            post_home_speed      (float) : speed [ticks/s] for post-home move
            is_arm_joint         (bool)  : True for arm joints (have Joint object), False for gripper
            normal_ilim          (float) : normal operating current limit [mA] to restore after homing
            normal_velocity_limit(float) : normal velocity limit [ticks/s] to restore after homing
            release_current_ma   (float) : current [mA] to command during release phase (NaN = skip).
                                           Sign matters: positive/negative sets motor torque direction.
                                           0 = coast (release gearbox preload with zero torque).
            release_duration_s   (float) : how long to hold the release current [seconds] (NaN = skip)
            release_sample_pct   (float) : when to latch the endstop position during release,
                                           as a fraction 0.0–1.0 of release_duration_s.
                                           E.g. 0.5 = sample at the midpoint. (NaN = sample at end)
            two_pass_max_diff    (float) : max allowed encoder tick difference between pass 1
                                           and pass 2. Exceeding this fails homing. (NaN = 200)
        """
        self.j                   = int(j)
        self.homing_speed        = float(homing_speed)
        self.homing_direction    = int(homing_direction)
        self.homing_current_ma   = float(homing_current_ma)
        self._current_detect_ma  = float(homing_current_ma) * self._CURRENT_DETECT_FRACTION
        self.has_hall_sensor     = int(has_hall_sensor)
        self.dt                  = float(dt)
        self.two_pass            = bool(two_pass)
        self.rehome_speed_factor = float(rehome_speed_factor)
        self.home_offset         = float(home_offset)
        self.post_home_position  = float(post_home_position)
        self.post_home_speed     = float(post_home_speed if post_home_speed is not None
                                         else homing_speed)
        self.is_arm_joint        = bool(is_arm_joint)
        self.normal_ilim          = float(normal_ilim) if normal_ilim is not None else None
        self.normal_velocity_limit = float(normal_velocity_limit) if normal_velocity_limit is not None else None

        # Release phase parameters (NaN = skip)
        self.release_current_ma  = float(release_current_ma)
        self.release_duration_s  = float(release_duration_s)
        self.release_sample_pct  = float(release_sample_pct)
        # Whether to do the release phase: need valid duration and not a Hall joint
        self._release_enabled = (not math.isnan(self.release_duration_s)
                                 and self.release_duration_s > 0
                                 and not self.has_hall_sensor)
        if self._release_enabled:
            self._release_ticks = max(1, round(self.release_duration_s / self.dt))
            # If release_current_ma is NaN, treat as 0 (coast)
            if math.isnan(self.release_current_ma):
                self.release_current_ma = 0.0
            # Sample tick: default to end of release if pct is NaN
            pct = self.release_sample_pct if not math.isnan(self.release_sample_pct) else 1.0
            pct = max(0.0, min(1.0, pct))
            self._release_sample_tick = max(1, round(self._release_ticks * pct))
        else:
            self._release_ticks = 0
            self._release_sample_tick = 0

        # Two-pass max difference: per-joint value from XML, fallback to class default
        self._two_pass_max_diff = (int(two_pass_max_diff)
                                   if not math.isnan(two_pass_max_diff)
                                   else self._TWO_PASS_MAX_DIFF_TICKS)

        # Derived tick thresholds from time constants
        self._stall_window     = max(5, round(self._STALL_TIME / self.dt))
        self._current_threshold = max(2, round(self._CURRENT_CONFIRM_TIME / self.dt))
        self._settle_ticks      = max(1, round(self._SETTLE_TIME / self.dt))
        self._post_move_stable = max(1, round(self._POST_MOVE_STABLE_TIME / self.dt))
        self._timeout_ticks    = max(1, round(float(timeout_s) / self.dt))
        self._backoff_ticks    = max(1, round(float(backoff_s) / self.dt))
        self._startup_guard    = round(self._STARTUP_GUARD_TIME / self.dt)
        self._dwell_ticks      = max(1, round(self._DWELL_TIME / self.dt))
        self._pause_ticks      = max(1, round(self._PAUSE_TIME / self.dt))
        self._hall_pre_clear_guard = max(1, round(self._HALL_PRE_CLEAR_GUARD_S / self.dt))

        # CAN node ID — captured from motor object on first tick
        self._node_id          = None

        # Mutable state — reset on each start
        self._phase            = self._HOMING
        self._tick_count       = 0
        self._settle_count     = 0
        self._stall_ref_pos    = None   # position at start of stall window
        self._stall_count      = 0      # ticks since stall_ref_pos was set
        self._current_count    = 0      # ticks with current above threshold
        self._current_above    = 0      # total ticks above threshold in current window
        self._backoff_count    = 0
        self._dwell_count      = 0
        self._pause_count      = 0
        self._release_count    = 0
        self._pre_clear_count  = 0      # ticks spent backing off in PRE_CLEAR phase
        self._pass             = 1      # 1 = fast first approach, 2 = slow second approach
        self._last_pos         = None
        self._triggered_pos    = None
        self._pass1_pos        = None   # endstop position from pass 1 (for two-pass comparison)
        self._post_move_target = None
        self._post_move_ok     = 0
        self._limits_sent      = False

    # ------------------------------------------------------------------
    @property
    def phase(self) -> str:
        return self._phase

    @property
    def timeout_s(self) -> float:
        return self._timeout_ticks * self.dt

    @property
    def backoff_s(self) -> float:
        return self._backoff_ticks * self.dt

    # ------------------------------------------------------------------
    def _restore_limits(self, motor):
        """Send normal operating current/velocity limits back to this joint's motor driver."""
        if self.normal_ilim is not None and self.normal_velocity_limit is not None:
            for _ in range(4):
                motor.Send_Limits(
                    Velocity_limit=int(self.normal_velocity_limit),
                    Current_limit=int(self.normal_ilim),
                )
            logging.info(
                f"[Homing] {self._label} current limit restored to "
                f"{self.normal_ilim:.0f} mA (was homing limit)"
            )

    # ------------------------------------------------------------------
    def reset(self):
        """Reset state for a fresh homing attempt."""
        self._phase            = self._HOMING
        self._tick_count       = 0
        self._settle_count     = 0
        self._stall_ref_pos    = None
        self._stall_count      = 0
        self._current_count    = 0
        self._current_above    = 0
        self._backoff_count    = 0
        self._dwell_count      = 0
        self._pause_count      = 0
        self._release_count    = 0
        self._pre_clear_count  = 0
        self._pass             = 1
        self._last_pos         = None
        self._triggered_pos    = None
        self._pass1_pos        = None
        self._post_move_target = None
        self._post_move_ok     = 0
        self._limits_sent      = False

    @property
    def _label(self) -> str:
        """Human-readable label for log messages, e.g. 'J4 [CANID 3]' or 'Gripper [CANID 6]'."""
        canid = f" [CANID {self._node_id}]" if self._node_id is not None else ""
        if self.is_arm_joint:
            return f"J{self.j + 1}{canid}"
        return f"Gripper{canid}"

    # ------------------------------------------------------------------
    def tick(self, motor, joint_or_none, shared_data) -> str:
        """
        Advance one RT tick.

        Returns:
            'running' — in progress
            'done'    — homing complete for this joint
            'failed'  — homing failed for this joint
        """
        if self._phase == self._DONE:
            return 'done'
        if self._phase == self._FAILED:
            return 'failed'

        # Capture CAN node ID from the motor object on first tick
        if self._node_id is None:
            self._node_id = getattr(motor, 'node_id', None)

        # On the very first tick, set the motor's current limit to the homing
        # value so stall detection is reliable.  For arm joints this mirrors
        # what enter() already did via load_motor_configs_fn; for the gripper
        # motor it is the only place this limit is applied.
        if not self._limits_sent and self.normal_velocity_limit is not None:
            for _ in range(4):
                motor.Send_Limits(
                    Velocity_limit=int(self.normal_velocity_limit),
                    Current_limit=int(self.homing_current_ma),
                )
            logging.info(
                f"[Homing] {self._label} current limit set to "
                f"{self.homing_current_ma:.0f} mA for homing"
            )
            self._limits_sent = True

        if self._phase == self._HOMING:
            return self._do_homing(motor, shared_data)
        elif self._phase == self._PRE_CLEAR:
            return self._do_pre_clear(motor)
        elif self._phase == self._DWELL:
            return self._do_dwell(motor)
        elif self._phase == self._BACKOFF:
            return self._do_backoff(motor)
        elif self._phase == self._PAUSE:
            return self._do_pause(motor)
        elif self._phase == self._RELEASE:
            return self._do_release(motor)
        elif self._phase == self._POST_MOVE:
            return self._do_post_move(motor, joint_or_none, shared_data)
        else:
            return self._do_settle(motor, joint_or_none, shared_data)

    # ------------------------------------------------------------------
    def _signed_speed(self, factor: float = 1.0) -> int:
        spd = int(self.homing_speed * factor)
        return -spd if self.homing_direction == 1 else spd

    # ------------------------------------------------------------------
    def _do_homing(self, motor, shared_data) -> str:
        j = self.j
        self._tick_count += 1

        if self._tick_count > self._timeout_ticks:
            self._fail(shared_data,
                       f"timeout ({self.timeout_s:.1f} s / "
                       f"{self._timeout_ticks} ticks) — "
                       f"check homing_direction and wiring")
            return 'running'

        factor = self.rehome_speed_factor if self._pass == 2 else 1.0
        speed  = self._signed_speed(factor)

        if self._tick_count == 1:
            shared_data.rti["homing_status"][j] = STATUS_RUNNING
            motor.HALL_trigger = None
            motor.hall_index   = None

        endstop_hit     = False
        detection_method = ''

        if self.has_hall_sensor:
            # Hall-sensor joint (e.g. PAR6 J6)
            motor.Send_data_pack_HALL(Speed=speed, trigger_value=2)

            if motor.HALL_trigger is not None and motor.HALL_trigger == 0:
                endstop_hit      = True
                detection_method = "Hall trigger"
            elif motor.hall_index is not None and motor.hall_index == 1:
                endstop_hit      = True
                detection_method = "Hall edge"

        else:
            # Endstop/stall joint — velocity mode; detect stall + current spike
            motor.Send_data_pack_1(Position=None, Speed=speed, Current=0)

            curr_pos = motor.position

            # ── Windowed stall detection ──
            # Instead of comparing adjacent positions (noisy at low speed / high
            # loop rate), check total displacement over a _stall_window.
            # If the motor moved less than stall_distance_threshold over the
            # entire window, it is stalled.
            stall_distance_threshold = max(
                10, int(abs(speed) * self._STALL_TIME * 0.25)
            )

            if curr_pos is not None:
                if self._stall_ref_pos is None:
                    # First valid position reading — start the window
                    self._stall_ref_pos = int(curr_pos)
                    self._stall_count   = 0
                else:
                    displacement = abs(int(curr_pos) - self._stall_ref_pos)
                    if displacement < stall_distance_threshold:
                        # Not moving — count up toward the stall window
                        self._stall_count += 1
                    else:
                        # Clearly moving — reset the window from current position
                        self._stall_ref_pos = int(curr_pos)
                        self._stall_count   = 0
            self._last_pos = curr_pos

            # ── Current detection ──
            # Use a ratio-based approach: count how many of the last N ticks
            # had current above threshold.  Trigger when ≥ 60% of ticks in the
            # window are above threshold.  This handles noisy current that
            # oscillates around the threshold (common at low homing speeds).
            curr_current = motor.current if motor.current is not None else 0
            abs_current = abs(curr_current)
            if self._tick_count > self._startup_guard:
                if abs_current >= self._current_detect_ma:
                    self._current_count += 1
                    self._current_above += 1
                else:
                    self._current_count += 1
                    # Don't increment _current_above

                # Check if we have enough samples in the window
                current_window_ok = (
                    self._current_count >= self._current_threshold
                    and self._current_above >= int(self._current_threshold * 0.6)
                )
            else:
                current_window_ok = False

            # ── Trigger ──
            # Primary: current above threshold for enough of the window
            # Secondary gate: position must be stalling (not a transient spike)
            stall_ok = self._stall_count >= self._stall_window
            if current_window_ok and stall_ok:
                endstop_hit      = True
                detection_method = (
                    f"current+stall (I={abs_current:.0f}mA, threshold="
                    f"{self._current_detect_ma:.0f}mA, above={self._current_above}/"
                    f"{self._current_count} ticks, "
                    f"displacement={abs(int(curr_pos) - self._stall_ref_pos) if curr_pos is not None else '?'} "
                    f"< {stall_distance_threshold} for {self._stall_count} ticks)"
                )

        if endstop_hit:
            pass_label = f"pass {self._pass}"
            logging.info(
                f"[Homing] {self._label} endstop detected via {detection_method} "
                f"({pass_label}, tick={self._tick_count}, pos={motor.position})"
            )

            # Guard: if a Hall sensor fires within the pre-clear window the joint
            # was already sitting on or very near the sensor when homing started.
            # Accepting that position would set a wrong home reference.  Instead,
            # back off to clear the sensor and then re-approach from a known side.
            if self.has_hall_sensor and self._tick_count <= self._hall_pre_clear_guard:
                motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
                logging.warning(
                    f"[Homing] {self._label} Hall triggered too early "
                    f"(tick={self._tick_count} ≤ guard={self._hall_pre_clear_guard}) "
                    f"— joint likely started on/near sensor, backing off to clear"
                )
                self._phase           = self._PRE_CLEAR
                self._pre_clear_count = 0
                return 'running'

            if self.has_hall_sensor:
                # Hall joints: the Hall edge/trigger IS the reference — capture now.
                self._triggered_pos = motor.position
            # Stop the motor immediately
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)

            if self._pass == 1 and self.two_pass and not self.has_hall_sensor:
                # Save pass 1 endstop position for later comparison with pass 2
                self._pass1_pos = motor.position
                # Dwell briefly to let the motor actually stop before reversing.
                # Hall sensor joints skip two-pass: the digital edge IS the
                # reference, a second approach doesn't improve accuracy.
                self._phase       = self._DWELL
                self._dwell_count = 0
            elif self._release_enabled:
                # Release phase: command a controlled current to relieve gearbox
                # preload, then latch the true resting endstop position.
                self._phase         = self._RELEASE
                self._release_count = 0
                self._triggered_pos = None  # will be latched during release
                logging.info(
                    f"[Homing] {self._label} entering release phase: "
                    f"{self.release_current_ma:.0f} mA for "
                    f"{self.release_duration_s:.3f} s, "
                    f"sample at {self.release_sample_pct if not math.isnan(self.release_sample_pct) else 1.0:.0%}"
                )
            else:
                self._phase        = self._SETTLE
                self._settle_count = 0

        return 'running'

    # ------------------------------------------------------------------
    def _do_pre_clear(self, motor) -> str:
        """Back off in the reverse direction when Hall triggered too early.

        Moves opposite to the homing direction for backoff_s seconds, then
        pauses and re-enters _HOMING so the real approach starts from a
        position that is clearly away from the Hall sensor.
        """
        self._pre_clear_count += 1
        backoff_speed = -self._signed_speed()
        motor.Send_data_pack_1(Position=None, Speed=backoff_speed, Current=0)

        if self._pre_clear_count >= self._backoff_ticks:
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
            logging.info(
                f"[Homing] {self._label} pre-clear backoff complete "
                f"({self.backoff_s:.3f} s / {self._backoff_ticks} ticks), "
                f"restarting approach"
            )
            # Reset all homing state for a clean second attempt
            self._phase           = self._HOMING
            self._tick_count      = 0
            self._stall_ref_pos   = None
            self._stall_count     = 0
            self._current_count   = 0
            self._current_above   = 0
            self._last_pos        = None
            self._triggered_pos   = None
            self._pre_clear_count = 0
            # Clear stale Hall readings so the first tick of the new approach
            # doesn't immediately re-trigger on cached CAN data.
            motor.HALL_trigger    = None
            motor.hall_index      = None

        return 'running'

    # ------------------------------------------------------------------
    def _do_dwell(self, motor) -> str:
        """Brief stop after endstop hit to let motor fully decelerate before backoff."""
        motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
        self._dwell_count += 1
        if self._dwell_count >= self._dwell_ticks:
            self._phase         = self._BACKOFF
            self._backoff_count = 0
            logging.info(
                f"[Homing] {self._label} dwell complete, starting backoff"
            )
        return 'running'

    # ------------------------------------------------------------------
    def _do_backoff(self, motor) -> str:
        backoff_speed = -self._signed_speed()
        motor.Send_data_pack_1(Position=None, Speed=backoff_speed, Current=0)
        self._backoff_count += 1

        if self._backoff_count >= self._backoff_ticks:
            # Stop and pause before starting pass 2 — let momentum dissipate
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
            self._phase       = self._PAUSE
            self._pause_count = 0
            logging.info(
                f"[Homing] {self._label} backed off ({self.backoff_s:.3f} s / "
                f"{self._backoff_ticks} ticks), pausing before second approach"
            )
        return 'running'

    # ------------------------------------------------------------------
    def _do_pause(self, motor) -> str:
        """Brief stop after backoff to let motor settle before pass 2."""
        motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
        self._pause_count += 1
        if self._pause_count >= self._pause_ticks:
            self._pass          = 2
            self._phase         = self._HOMING
            self._tick_count    = 0
            self._stall_ref_pos = None
            self._stall_count   = 0
            self._current_count = 0
            self._current_above = 0
            self._last_pos      = None
            self._triggered_pos = None
            logging.info(
                f"[Homing] {self._label} pause complete, starting slow second approach "
                f"({self.rehome_speed_factor*100:.0f}% speed)"
            )
        return 'running'

    # ------------------------------------------------------------------
    def _do_release(self, motor) -> str:
        """Command a controlled current to release gearbox preload at endstop.

        The motor is driven in current-command mode (Position=None, Speed=None,
        Current=release_current_ma) for release_duration_s seconds.  The endstop
        encoder position is latched at the configured sample point within this
        window — after the gearbox has relaxed but before the joint drifts away.
        """
        self._release_count += 1
        current_cmd = int(self.release_current_ma)
        motor.Send_data_pack_1(Position=None, Speed=None, Current=current_cmd)

        # Latch position at the configured sample tick
        if self._release_count == self._release_sample_tick:
            self._triggered_pos = motor.position
            logging.info(
                f"[Homing] {self._label} release: latched position "
                f"{self._triggered_pos} at tick {self._release_count}/"
                f"{self._release_ticks}"
            )

        if self._release_count >= self._release_ticks:
            # If position wasn't captured at sample point (e.g. was None), grab it now
            if self._triggered_pos is None and motor.position is not None:
                self._triggered_pos = motor.position
                logging.warning(
                    f"[Homing] {self._label} release: sample at configured tick "
                    f"failed, using end-of-release position {self._triggered_pos}"
                )
            # Stop current command before moving to settle
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
            self._phase        = self._SETTLE
            self._settle_count = 0
            logging.info(
                f"[Homing] {self._label} release complete "
                f"({self.release_duration_s:.3f} s), pos={self._triggered_pos}"
            )
        return 'running'

    # ------------------------------------------------------------------
    def _do_settle(self, motor, joint_or_none, shared_data) -> str:
        j = self.j
        motor.Send_data_pack_1(Position=None, Speed=0, Current=0)

        self._settle_count += 1

        if self._settle_count < self._settle_ticks:
            return 'running'

        # Latch position now that the motor is fully stopped.
        # Skip if already set by: Hall detection (at trigger) or release phase (at sample point).
        # If motor.position is None (transient CAN read failure), keep waiting up to
        # an extra _settle_ticks worth of ticks rather than silently skipping calibration.
        if self._triggered_pos is None:
            if motor.position is not None:
                self._triggered_pos = motor.position
            else:
                if self._settle_count < self._settle_ticks * 2:
                    logging.debug(
                        f"[Homing] {self._label} settle: motor.position is None, retrying "
                        f"(tick {self._settle_count})"
                    )
                    return 'running'
                logging.error(
                    f"[Homing] {self._label} settle: motor.position never became valid — "
                    f"home reference NOT set, joint will be uncalibrated"
                )

        # Two-pass consistency check: if both passes recorded a position,
        # verify they agree within tolerance. A large difference means the
        # endstop trigger is unreliable (e.g. debris, loose mechanism).
        if (self._pass1_pos is not None
                and self._triggered_pos is not None):
            diff = abs(int(self._triggered_pos) - int(self._pass1_pos))
            logging.info(
                f"[Homing] {self._label} two-pass difference: "
                f"{diff} ticks (pass1={self._pass1_pos}, "
                f"pass2={self._triggered_pos}, "
                f"max={self._two_pass_max_diff})"
            )
            if diff > self._two_pass_max_diff:
                logging.error(
                    f"[Homing] {self._label} FAILED: two-pass difference "
                    f"{diff} ticks exceeds max {self._two_pass_max_diff} — "
                    f"endstop trigger is not repeatable"
                )
                shared_data.rti["homing_status"][j] = STATUS_FAILED
                self._restore_limits(motor)
                self._phase = self._FAILED
                return 'failed'

        # Update joint position reference at endstop
        if self._triggered_pos is not None:
            if self.is_arm_joint and joint_or_none is not None:
                _update_home_reference(joint_or_none, self._triggered_pos,
                                       self.home_offset)
                logging.info(
                    f"[Homing] {self._label} reference set: "
                    f"master_pos={self._triggered_pos}, "
                    f"home_offset={self.home_offset:.4f} rad"
                )
            else:
                logging.info(
                    f"[Homing] {self._label} homed at encoder pos "
                    f"{self._triggered_pos}"
                )

        shared_data.rti["homing_status"][j] = STATUS_DONE

        # Set per-joint/gripper homed flag if shared memory supports it
        if self.is_arm_joint:
            if "joint_homed" in shared_data.rti:
                shared_data.rti["joint_homed"][j] = 1
        else:
            if hasattr(shared_data, 'gripper') and "gripper_homed" in shared_data.gripper:
                shared_data.gripper["gripper_homed"][0] = 1

        # Restore normal current limit now that homing is done
        self._restore_limits(motor)

        # If a post-home position is configured, drive there before the next joint
        if (self.is_arm_joint and joint_or_none is not None
                and not math.isnan(self.post_home_position)):
            self._post_move_target = None
            self._post_move_ok     = 0
            self._tick_count       = 0
            self._phase            = self._POST_MOVE
            logging.info(
                f"[Homing] {self._label} moving to post-home position "
                f"{self.post_home_position:.4f} rad"
            )
            return 'running'

        self._phase = self._DONE
        return 'done'

    # ------------------------------------------------------------------
    def _do_post_move(self, motor, joint_or_none, shared_data) -> str:
        j = self.j
        self._tick_count += 1

        # Compute absolute motor-tick target once on first entry
        if self._post_move_target is None:
            self._post_move_target = int(
                joint_or_none.get_encoder_position(self.post_home_position)
            )
            logging.info(
                f"[Homing] {self._label} post-home target: "
                f"{self.post_home_position:.4f} rad "
                f"→ {self._post_move_target} ticks"
            )

        motor.Send_data_pack_1(
            Position=self._post_move_target,
            Speed=int(self.post_home_speed),
            Current=0
        )

        # Timeout — don't block the rest of the sequence if the move fails
        if self._tick_count > self._timeout_ticks:
            logging.warning(
                f"[Homing] {self._label} post-home move timed out ({self.timeout_s:.1f} s) "
                f"— continuing to next joint"
            )
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
            self._phase = self._DONE
            return 'done'

        # Arrival check: within tolerance for _post_move_stable consecutive ticks
        curr_pos = motor.position
        if curr_pos is not None:
            if abs(int(curr_pos) - self._post_move_target) < self._POST_MOVE_TOL_TICKS:
                self._post_move_ok += 1
            else:
                self._post_move_ok = 0

        if self._post_move_ok >= self._post_move_stable:
            logging.info(f"[Homing] {self._label} reached post-home position.")
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
            self._phase = self._DONE
            return 'done'

        return 'running'

    # ------------------------------------------------------------------
    def _fail(self, shared_data, reason: str):
        logging.error(f"[Homing] {self._label} FAILED: {reason}")
        shared_data.rti["homing_status"][self.j] = STATUS_FAILED
        self._phase = self._FAILED


# ---------------------------------------------------------------------------
# Gripper firmware homing state machine
# ---------------------------------------------------------------------------

class GripperHomingState:
    """
    Firmware-level gripper homing state machine.

    Sends Send_gripper_calibrate() once, then polls gripper_calibrated
    status each tick until it becomes 1 or timeout.
    """

    _CALIBRATING = 'calibrating'
    _DONE        = 'done'
    _FAILED      = 'failed'

    _CALIBRATE_TIMEOUT_TIME = 10.0  # seconds
    _MIN_WAIT_S             = 2.0   # minimum seconds to wait after sending command
                                    # before accepting g_cal=1, regardless of prior state

    def __init__(self, j, dt, timeout_s=None):
        self.j  = int(j)
        self.dt = float(dt)
        self._node_id = None
        timeout = float(timeout_s) if timeout_s is not None else self._CALIBRATE_TIMEOUT_TIME
        self._timeout_ticks  = max(1, round(timeout / self.dt))
        self._min_wait_ticks = max(1, round(self._MIN_WAIT_S / self.dt))
        self._phase      = self._CALIBRATING
        self._tick_count = 0
        self._sent_cmd   = False

    @property
    def _label(self) -> str:
        canid = f" [CANID {self._node_id}]" if self._node_id is not None else ""
        return f"Gripper{canid}"

    @property
    def phase(self) -> str:
        return self._phase

    @property
    def timeout_s(self) -> float:
        return self._timeout_ticks * self.dt

    def reset(self):
        self._phase      = self._CALIBRATING
        self._tick_count = 0
        self._sent_cmd   = False

    def tick(self, motor, joint_or_none, shared_data) -> str:
        """
        One RT tick. Returns 'running' | 'done' | 'failed'.

        Sends Send_gripper_calibrate() once, then waits a minimum of
        _MIN_WAIT_S seconds before accepting g_cal=1 as completion.
        The minimum wait ensures the gripper has actually run its calibration
        sequence even if g_cal was already 1 before the command was sent
        (some grippers never drop g_cal to 0 during recalibration).
        """
        if self._phase == self._DONE:
            return 'done'
        if self._phase == self._FAILED:
            return 'failed'

        if self._node_id is None:
            self._node_id = getattr(motor, 'node_id', None)

        self._tick_count += 1

        if self._tick_count == 1:
            shared_data.rti["homing_status"][self.j] = STATUS_RUNNING

        # Send calibrate command once
        if not self._sent_cmd:
            if hasattr(motor, 'Send_gripper_calibrate'):
                motor.Send_gripper_calibrate()
                logging.info(f"[Homing] {self._label} firmware calibrate sent")
            else:
                logging.error(
                    f"[Homing] {self._label} motor object has no "
                    f"Send_gripper_calibrate method"
                )
                shared_data.rti["homing_status"][self.j] = STATUS_FAILED
                self._phase = self._FAILED
                return 'failed'
            self._sent_cmd = True

        # Poll gripper status — send empty pack (all None) so the gripper
        # responds with its current state without altering any commands.
        motor.Send_gripper_data_pack()

        # Only accept g_cal=1 after the minimum wait has elapsed
        if self._tick_count >= self._min_wait_ticks:
            calibrated = getattr(motor, 'gripper_calibrated', None)
            if calibrated is not None and int(calibrated) == 1:
                logging.info(
                    f"[Homing] {self._label} firmware calibration done "
                    f"(tick={self._tick_count})"
                )
                shared_data.rti["homing_status"][self.j] = STATUS_DONE
                if hasattr(shared_data, 'gripper') and "gripper_homed" in shared_data.gripper:
                    shared_data.gripper["gripper_homed"][0] = 1
                self._phase = self._DONE
                return 'done'

        # Timeout
        if self._tick_count > self._timeout_ticks:
            logging.error(
                f"[Homing] {self._label} firmware calibration timed out "
                f"({self.timeout_s:.1f} s)"
            )
            shared_data.rti["homing_status"][self.j] = STATUS_FAILED
            self._phase = self._FAILED
            return 'failed'

        return 'running'


# ---------------------------------------------------------------------------
# Shared utility
# ---------------------------------------------------------------------------

def _update_home_reference(joint, motor_pos: int, home_offset: float):
    """Update joint reference so get_joint_position(motor_pos) == home_offset."""
    joint.master_position = int(motor_pos)
    joint.offset          = float(home_offset)
    if joint.dir == 0:
        joint.offset_ticks = joint.radians_to_ticks(joint.offset)
    else:
        joint.offset_ticks = joint.radians_to_ticks(2.0 * math.pi - joint.offset)


# ---------------------------------------------------------------------------
# Orchestrating controller
# ---------------------------------------------------------------------------

class HomingController:
    """
    Homing sequence controller — manages one or more joints.

    Supports two modes:
    1. Legacy sequential: one joint at a time, order from home_sequence list
    2. Parallel step-based: multiple joints per step, with pre-moves,
       controlled by a homing sequence XML file

    Call step() every RT tick while robot_mode == HOMING.
    Returns 'running', 'done', or 'failed'.
    """

    _IDLE      = 'idle'
    _PRE_MOVE  = 'pre_moving'
    _HOMING    = 'homing'
    _POST_MOVE = 'post_moving'
    _MOVE_TO   = 'move_to'
    _DONE      = 'done'
    _FAILED    = 'failed'

    # Time-based constants for pre/post moves (seconds)
    _MOVE_STABLE_TIME   = 0.05   # must hold within tolerance for this long
    _MOVE_TOL_TICKS     = 100    # encoder tick tolerance for arrival
    _MOVE_TIMEOUT_S     = 4.0    # max time for a pre/post move (short — joint may not be calibrated yet)

    def __init__(self, joint_num, home_offsets, homing_speeds, homing_directions,
                 homing_current_ma, has_hall_sensor, dt=0.001,
                 timeout_s=6.0, two_pass_homing=None, backoff_s=0.3,
                 rehome_speed_factor=0.3, home_sequence=None,
                 post_home_positions=None, post_home_speeds=None,
                 arm_count=None, homing_steps=None,
                 release_current_ma=None, release_duration_s=None,
                 release_sample_pct=None,
                 two_pass_max_diff=None):
        """
        Args:
            joint_num            (int)          : total number of actuators (arm + gripper)
            home_offsets         (np.array)     : joint angle [rad] at each endstop
            homing_speeds        (np.array)     : motor speed [ticks/s] during homing
            homing_directions    (np.array)     : 0 = positive motor dir, 1 = negative
            homing_current_ma    (np.array)     : current threshold [mA] for stall detection
            has_hall_sensor      (np.array)     : 1 if joint uses Hall sensor, 0 for stall
            dt                   (float)        : RT loop period in seconds
            timeout_s            (float/array)  : max time allowed per joint [seconds]
            two_pass_homing      (array-like)   : per-joint bool; True = Prusa-style
            backoff_s            (float)        : time to reverse after first hit [seconds]
            rehome_speed_factor  (float)        : second-pass speed = first-pass × factor
            home_sequence        (list[int])    : joint indices in homing order (legacy mode)
            post_home_positions  (array-like)   : per-joint angle [rad] to move to after homing
            post_home_speeds     (array-like)   : per-actuator speed [ticks/s] for post-home move
            arm_count            (int)          : number of arm joints (0..arm_count-1)
            homing_steps         (list[dict])   : parsed homing sequence steps (parallel mode)
            release_current_ma   (array-like)   : per-joint current [mA] for release phase (NaN=skip)
            release_duration_s   (array-like)   : per-joint release duration [seconds] (NaN=skip)
            release_sample_pct   (array-like)   : per-joint position sample point 0.0–1.0 (NaN=end)
            two_pass_max_diff    (array-like)   : per-joint max allowed tick difference between passes (NaN=200)
        """
        self.joint_num           = joint_num
        self._arm_count          = int(arm_count) if arm_count is not None else joint_num
        self.home_offsets        = np.asarray(home_offsets,      dtype=float)
        self.homing_speeds       = np.asarray(homing_speeds,     dtype=float)
        self.homing_directions   = np.asarray(homing_directions, dtype=int)
        self.homing_current_ma   = np.asarray(homing_current_ma, dtype=float)
        self.has_hall_sensor     = np.asarray(has_hall_sensor,   dtype=int)
        self.rehome_speed_factor = float(rehome_speed_factor)
        self.dt                  = float(dt)

        # Per-actuator timeout in seconds
        _ts = np.broadcast_to(np.asarray(timeout_s, dtype=float), (joint_num,)).copy()
        self._timeout_s = _ts
        self._timeout_ticks = np.round(_ts / self.dt).astype(int)

        self._backoff_s        = np.broadcast_to(np.asarray(backoff_s,        dtype=float), (joint_num,)).copy()

        if two_pass_homing is None:
            self.two_pass_homing = np.ones(joint_num, dtype=bool)
        else:
            self.two_pass_homing = np.asarray(two_pass_homing, dtype=bool)

        if post_home_positions is None:
            self.post_home_positions = np.full(joint_num, np.nan)
        else:
            self.post_home_positions = np.asarray(post_home_positions, dtype=float)

        if post_home_speeds is None:
            self.post_home_speeds = self.homing_speeds.copy()
        else:
            self.post_home_speeds = np.asarray(post_home_speeds, dtype=float)

        # Release phase arrays (NaN = skip)
        if release_current_ma is None:
            self.release_current_ma = np.full(joint_num, np.nan)
        else:
            self.release_current_ma = np.asarray(release_current_ma, dtype=float)
        if release_duration_s is None:
            self.release_duration_s = np.full(joint_num, np.nan)
        else:
            self.release_duration_s = np.asarray(release_duration_s, dtype=float)
        if release_sample_pct is None:
            self.release_sample_pct = np.full(joint_num, np.nan)
        else:
            self.release_sample_pct = np.asarray(release_sample_pct, dtype=float)

        # Two-pass max difference (NaN = default 200 ticks)
        if two_pass_max_diff is None:
            self.two_pass_max_diff = np.full(joint_num, np.nan)
        else:
            self.two_pass_max_diff = np.asarray(two_pass_max_diff, dtype=float)

        # Homing order (legacy mode)
        self.home_sequence: list = list(home_sequence) if home_sequence is not None \
                                   else list(range(joint_num))

        # Parallel homing steps (None = legacy sequential mode)
        self._homing_steps = homing_steps

        # Tick threshold for move arrival
        self._move_stable_ticks = max(1, round(self._MOVE_STABLE_TIME / self.dt))
        self._move_timeout_ticks = max(1, round(self._MOVE_TIMEOUT_S / self.dt))

        # Runtime state
        self._seq_phase = self._IDLE
        self._seq_idx   = 0
        self._active_j  = 0                              # legacy compat
        self._active_state: JointHomingState = None       # legacy single-joint
        self._active_states: dict = {}                    # parallel: {j: JointHomingState}
        self._homed_results: dict = {}                    # {j: triggered_pos} accumulated as joints finish
        self._pre_move_targets: dict = {}                 # {j: (target_ticks, speed)}
        self._pre_move_ok: dict = {}                      # {j: consecutive_ok_count}
        self._move_tick_count = 0
        self._post_move_list = []                         # sequence-level post moves
        self._move_to_targets: dict = {}                  # {j: {position_rad, duration_ticks, start_ticks, end_ticks, done}}
        self._move_to_tick = 0
        self._last_gripper_move: tuple = None             # last gripper_move params — replayed as idle hold
        self._Motor = None                                # stored for log labels (set in enter())

    # ------------------------------------------------------------------
    # Public time-based properties — backward compat
    # ------------------------------------------------------------------

    @property
    def timeout_s(self) -> np.ndarray:
        """Per-actuator homing timeout in seconds (read/write)."""
        return self._timeout_ticks * self.dt

    @timeout_s.setter
    def timeout_s(self, val):
        arr = np.broadcast_to(np.asarray(val, dtype=float), (self.joint_num,)).copy()
        self._timeout_ticks = np.round(arr / self.dt).astype(int)
        self._timeout_s = arr

    @property
    def backoff_s(self) -> np.ndarray:
        """Per-joint back-off duration in seconds (read/write)."""
        return self._backoff_s

    @backoff_s.setter
    def backoff_s(self, val):
        self._backoff_s = np.broadcast_to(np.asarray(val, dtype=float), (self.joint_num,)).copy()

    # ------------------------------------------------------------------
    @classmethod
    def from_config(cls, xml_config: dict, joint_num: int) -> 'HomingController':
        """
        Build from the dict returned by load_full_robot_config().
        NaN values (missing XML fields) fall back to safe defaults.

        All timeout/backoff values are read in **seconds** from XML fields:
          homing_timeout_s  (per-joint or global scalar, default 6.0 s)
          homing_backoff_s  (global scalar, default 0.3 s)

        If cfg contains 'gripper_CAN_gripper' == 1 and 'gripper_homing_speed'
        is set, the gripper is appended as an extra actuator at index joint_num.
        Its order in the homing sequence is controlled by 'gripper_homing_order'.
        """
        dt = float(xml_config.get('delta_t', 0.001))

        def _fill(key, default):
            arr = xml_config.get(key, np.full(joint_num, np.nan))
            return np.where(np.isnan(arr), default, np.asarray(arr, dtype=float))

        # --- Arm-joint arrays ---
        home_offsets      = _fill('home_offset',       math.pi / 2)
        homing_speeds     = _fill('homing_speed',      5000.0)
        homing_directions = _fill('homing_direction',  1.0)
        homing_current_ma = _fill('homing_current_ma', 600.0)
        has_hall_sensor   = _fill('has_hall_sensor',   0.0).astype(int)
        two_pass          = _fill('homing_two_pass',   1.0).astype(bool)
        timeout_s         = _fill('homing_timeout_s', 6.0)
        backoff_s         = _fill('homing_backoff_s', 0.3)
        rehome_factor     = float(xml_config.get('homing_rehome_factor', 0.3))

        # Release phase parameters (NaN = skip for that joint)
        release_current   = _fill('homing_release_current_ma', np.nan)
        release_duration  = _fill('homing_release_duration_s',  np.nan)
        release_sample    = _fill('homing_release_sample_pct',  np.nan)

        # Two-pass max difference (NaN = use default 200 ticks)
        two_pass_max_diff = _fill('homing_two_pass_max_diff', np.nan)

        # Post-home positions / speeds for arm joints
        post_pos_raw = xml_config.get('post_home_position', None)
        post_pos = (_fill('post_home_position', np.nan)
                    if post_pos_raw is not None
                    else np.full(joint_num, np.nan))
        post_spd_raw = xml_config.get('post_home_speed', None)
        if post_spd_raw is not None:
            raw_spd  = _fill('homing_speed', 5000.0)
            post_spd = np.where(np.isnan(_fill('post_home_speed', np.nan)),
                                raw_spd, _fill('post_home_speed', np.nan))
        else:
            post_spd = None

        # Arm-joint homing order: 1-based per-joint field → order_arr for sorting
        order_raw = xml_config.get('homing_order', None)
        if order_raw is not None:
            order_arr = np.asarray(order_raw, dtype=float)
            order_arr = np.where(np.isnan(order_arr),
                                 1e9 + np.arange(joint_num, dtype=float),
                                 order_arr)
        else:
            order_arr = 1e9 + np.arange(joint_num, dtype=float)

        # --- Optional CAN-gripper homing ---
        def _gget(key, default):
            v = xml_config.get(f'gripper_{key}', np.nan)
            if isinstance(v, float) and math.isnan(v):
                return float(default)
            try:
                return float(v)
            except (TypeError, ValueError):
                return float(default)

        can_gripper    = int(_gget('CAN_gripper', 0))
        gripper_speed  = xml_config.get('gripper_homing_speed', np.nan)
        do_gripper     = (can_gripper == 1
                          and not (isinstance(gripper_speed, float)
                                   and math.isnan(gripper_speed)))

        if do_gripper:
            total = joint_num + 1

            home_offsets      = np.append(home_offsets,      _gget('home_offset',      math.pi / 2))
            homing_speeds     = np.append(homing_speeds,     _gget('homing_speed',     5000.0))
            homing_directions = np.append(homing_directions, _gget('homing_direction', 1.0))
            homing_current_ma = np.append(homing_current_ma, _gget('homing_current_ma', 600.0))
            has_hall_sensor   = np.append(has_hall_sensor,   int(_gget('has_hall_sensor', 0)))
            two_pass          = np.append(two_pass,          bool(_gget('homing_two_pass', 1.0)))
            backoff_s         = np.append(backoff_s,         _gget('homing_backoff_s', 0.3))
            g_timeout         = _gget('homing_timeout_s',    6.0)
            timeout_s         = np.append(timeout_s,         g_timeout)
            post_pos          = np.append(post_pos,          np.nan)
            if post_spd is None:
                post_spd = np.append(homing_speeds[:joint_num],
                                     _gget('homing_speed', 5000.0))
            else:
                post_spd = np.append(post_spd, _gget('homing_speed', 5000.0))

            # Gripper doesn't use release phase
            release_current  = np.append(release_current,  np.nan)
            release_duration = np.append(release_duration, np.nan)
            release_sample   = np.append(release_sample,   np.nan)

            two_pass_max_diff = np.append(two_pass_max_diff, _gget('homing_two_pass_max_diff', np.nan))

            gripper_normal_ilim           = _gget('Ilim',           1000.0)
            gripper_normal_velocity_limit = _gget('Velocity_limit', 80000.0)

            g_order_raw = xml_config.get('gripper_homing_order', np.nan)
            g_order_val = (1e9 + joint_num
                           if (isinstance(g_order_raw, float) and math.isnan(g_order_raw))
                           else float(g_order_raw))
            order_arr = np.append(order_arr, g_order_val)
        else:
            total = joint_num

        home_seq = list(np.argsort(order_arr, kind='stable').astype(int))

        # Check for parsed homing sequence from XML file
        homing_steps = xml_config.get('homing_sequence', None)

        obj = cls(
            total,
            home_offsets        = home_offsets,
            homing_speeds       = homing_speeds,
            homing_directions   = homing_directions,
            homing_current_ma   = homing_current_ma,
            has_hall_sensor     = has_hall_sensor,
            dt                  = dt,
            timeout_s           = timeout_s,
            two_pass_homing     = two_pass,
            backoff_s           = backoff_s,
            rehome_speed_factor = rehome_factor,
            home_sequence       = home_seq,
            post_home_positions = post_pos,
            post_home_speeds    = post_spd,
            arm_count           = joint_num,
            homing_steps        = homing_steps,
            release_current_ma  = release_current,
            release_duration_s  = release_duration,
            release_sample_pct  = release_sample,
            two_pass_max_diff   = two_pass_max_diff,
        )
        if do_gripper:
            obj._gripper_normal_ilim           = gripper_normal_ilim
            obj._gripper_normal_velocity_limit = gripper_normal_velocity_limit
            obj._gripper_stroke_mm             = float(_gget('stroke', 0.0))
            obj._gripper_gear_r                = float(xml_config.get('gripper_gear_r', 0.0) or 0.0)
        return obj

    # ------------------------------------------------------------------
    def _make_joint_state(self, j: int) -> JointHomingState:
        """Create a JointHomingState for joint index j."""
        return JointHomingState(
            j                   = j,
            homing_speed        = float(self.homing_speeds[j]),
            homing_direction    = int(self.homing_directions[j]),
            homing_current_ma   = float(self.homing_current_ma[j]),
            has_hall_sensor     = int(self.has_hall_sensor[j]),
            dt                  = self.dt,
            timeout_s           = float(self._timeout_s[j]),
            two_pass            = bool(self.two_pass_homing[j]),
            backoff_s           = float(self._backoff_s[j]),
            rehome_speed_factor = self.rehome_speed_factor,
            home_offset         = float(self.home_offsets[j]),
            post_home_position  = float(self.post_home_positions[j]),
            post_home_speed     = float(self.post_home_speeds[j]),
            is_arm_joint        = (j < self._arm_count),
            normal_ilim          = (
                float(self._motor_config['Ilim'][j])
                if getattr(self, '_motor_config', None) is not None
                   and j < len(self._motor_config['Ilim'])
                else getattr(self, '_gripper_normal_ilim', None)
                     if j >= self._arm_count else None
            ),
            normal_velocity_limit = (
                float(self._motor_config['velocity_limit'][j])
                if getattr(self, '_motor_config', None) is not None
                   and j < len(self._motor_config['velocity_limit'])
                else getattr(self, '_gripper_normal_velocity_limit', None)
                     if j >= self._arm_count else None
            ),
            release_current_ma   = float(self.release_current_ma[j]),
            release_duration_s   = float(self.release_duration_s[j]),
            release_sample_pct   = float(self.release_sample_pct[j]),
            two_pass_max_diff    = float(self.two_pass_max_diff[j]),
        )

    # ------------------------------------------------------------------
    def enter(self, Motor, joint_num, load_motor_configs_fn, motor_config, shared_data=None):
        """
        Call once when entering HOMING mode.

        Loads reduced current limits for gentle endstop contact, then starts
        the homing sequence.  Keeps motor_config for exit() restore.

        Args:
            Motor                : list of motor objects
            joint_num            : number of arm joints
            load_motor_configs_fn: callable — load_motor_configs from can_message_handlers
            motor_config         : dict with keys: velocity_limit, Ilim, KPP, KD,
                                   KPIQ, KIIQ, KPV, KIV  (normal operating values)
            shared_data          : shared data object (optional, for clearing homed flags)
        """
        self._motor_config = motor_config
        self._load_motor_configs_fn = load_motor_configs_fn
        self._Motor = Motor  # stored for log labels

        load_motor_configs_fn(Motor,
            joint_num      = joint_num,
            watchdog_rate  = [5000] * joint_num,
            velocity_limit = motor_config['velocity_limit'],
            current_limit  = self.homing_current_ma,
            KP  = motor_config['KP'],   KD  = motor_config['KD'],
            KPIQ= motor_config['KPIQ'], KIIQ= motor_config['KIIQ'],
            KPV = motor_config['KPV'],  KIV = motor_config['KIV'],
            KPP = motor_config['KPP'],
            watchdog_action="Idle",
            num_repeats=4)

        self.start(shared_data)

    # ------------------------------------------------------------------
    def exit(self, Motor, joint_num):
        """
        Call once when leaving HOMING mode (done or failed).

        Restores normal motor current limits.
        """
        mc = getattr(self, '_motor_config', None)
        fn = getattr(self, '_load_motor_configs_fn', None)
        if mc is not None and fn is not None:
            fn(Motor,
                joint_num      = joint_num,
                watchdog_rate  = [5000] * joint_num,
                velocity_limit = mc['velocity_limit'],
                current_limit  = mc['Ilim'],
                KP  = mc['KP'],   KD  = mc['KD'],
                KPIQ= mc['KPIQ'], KIIQ= mc['KIIQ'],
                KPV = mc['KPV'],  KIV = mc['KIV'],
                KPP = mc['KPP'],
                watchdog_action="Idle",
                num_repeats=4)

    # ------------------------------------------------------------------
    def _send_idle(self, Motor, i):
        """Send an appropriate idle/poll command for actuator i.

        Arm joints  (i < _arm_count): Send_data_pack_1 with zero speed/current
        Gripper     (i >= _arm_count): replay the last gripper_move command if
            one was issued this homing session, otherwise send an empty poll.
            Replaying holds the gripper at its commanded position throughout
            subsequent homing phases without needing a new command each step.
        """
        if i >= self._arm_count:
            if self._last_gripper_move is not None:
                pos, spd, cur, act, act2, estop, rel = self._last_gripper_move
                Motor[i].Send_gripper_data_pack(
                    position_=pos, speed_=spd, current_=cur,
                    activate_=act, action_=act2,
                    ESTOP_=estop, Release_dir_=rel,
                )
            else:
                Motor[i].Send_gripper_data_pack()
        else:
            Motor[i].Send_data_pack_1(Position=None, Speed=0, Current=0)

    # ------------------------------------------------------------------
    def _jlabel(self, j, Motor) -> str:
        """Human-readable label for joint j, e.g. 'J4 [CANID 3]' or 'Gripper [CANID 6]'."""
        canid = ""
        if Motor is not None and j < len(Motor):
            nid = getattr(Motor[j], 'node_id', None)
            if nid is not None:
                canid = f" [CANID {nid}]"
        if j >= self._arm_count:
            return f"Gripper{canid}"
        return f"J{j + 1}{canid}"

    # ------------------------------------------------------------------
    def _build_steps(self):
        """
        Build the internal step list from either homing_steps (parallel mode)
        or home_sequence (legacy sequential mode).

        Each step dict has: pre_moves, home_joints, gripper_mode, post_moves (optional).
        In legacy mode, each step homes a single joint with no pre-moves.
        """
        if self._homing_steps is not None:
            # Parallel mode — resolve 'gripper' to actual index
            steps = []
            for raw_step in self._homing_steps:
                step = dict(raw_step)
                resolved = []
                for jref in step.get('home_joints', []):
                    if jref == 'gripper':
                        # gripper is at index _arm_count (= joint_num for legacy,
                        # or total-1 when gripper is appended)
                        resolved.append(self._arm_count)
                    else:
                        resolved.append(int(jref))
                step['home_joints'] = resolved
                steps.append(step)
            return steps

        # Legacy sequential mode — one joint per step
        steps = []
        for j in self.home_sequence:
            steps.append({
                'pre_moves': [],
                'home_joints': [j],
                'gripper_mode': None,
            })
        return steps

    # ------------------------------------------------------------------
    def start(self, shared_data=None):
        """Begin the homing sequence. Call once when entering HOMING mode."""
        self._steps = self._build_steps()
        self._seq_idx = 0
        self._active_states = {}
        self._homed_results = {}
        self._pre_move_targets = {}
        self._pre_move_ok = {}
        self._move_tick_count = 0
        self._post_move_list = []
        self._move_to_targets = {}
        self._move_to_tick = 0
        self._last_gripper_move = None

        # Legacy compat
        if self._steps and self._steps[0].get('home_joints'):
            self._active_j = self._steps[0]['home_joints'][0]
        else:
            self._active_j = 0
        self._active_state = None

        # Clear per-joint/gripper homed flags
        if shared_data is not None:
            if "joint_homed" in shared_data.rti:
                shared_data.rti["joint_homed"][:] = 0
            if hasattr(shared_data, 'gripper') and "gripper_homed" in shared_data.gripper:
                shared_data.gripper["gripper_homed"][0] = 0

        # Enter first step
        self._enter_step(0)

        mode_desc = "parallel" if self._homing_steps is not None else "sequential"
        logging.info(
            f"[Homing] Sequence started ({mode_desc}) — "
            f"{len(self._steps)} steps, {self.joint_num} actuators"
        )

    # ------------------------------------------------------------------
    def _enter_step(self, idx):
        """Initialize the state for step at index idx."""
        if idx >= len(self._steps):
            return

        self._seq_idx = idx
        step = self._steps[idx]

        # Check for move_to step (smooth interpolated joint-space moves)
        if step.get('move_to') and not step['home_joints']:
            self._move_to_targets = {}
            self._move_to_tick = 0
            for mt in step['move_to']:
                j = mt['joint']
                duration_ticks = max(1, round(mt['duration_s'] / self.dt))
                self._move_to_targets[j] = {
                    'position_rad':   mt['position_rad'],
                    'duration_ticks': duration_ticks,
                    'start_ticks':    None,
                    'end_ticks':      None,
                    'done':           False,
                }
            self._seq_phase = self._MOVE_TO
            logging.info(
                f"[Homing] Step {idx}: move_to for joints "
                f"{list(self._move_to_targets.keys())}"
            )
            return

        # Check for post_moves-only step (sequence-level post moves)
        if 'post_moves' in step and not step['home_joints']:
            self._post_move_list = step['post_moves']
            self._pre_move_targets = {}
            self._pre_move_ok = {}
            self._move_tick_count = 0
            for pm in self._post_move_list:
                j = pm['joint']
                self._pre_move_targets[j] = (None, pm['speed'])  # target computed at runtime
                self._pre_move_ok[j] = 0
            self._seq_phase = self._POST_MOVE
            return

        # Has pre-moves?
        if step['pre_moves']:
            self._pre_move_targets = {}
            self._pre_move_ok = {}
            self._move_tick_count = 0
            for pm in step['pre_moves']:
                j = pm['joint']
                # Resolve 'gripper' to the actual gripper motor index
                if j == 'gripper':
                    j = self._arm_count
                    if j >= self.joint_num:
                        # No CAN gripper configured — skip silently
                        continue
                if pm.get('type') == 'nudge':
                    duration_ticks = max(1, round(pm['duration_s'] / self.dt))
                    self._pre_move_targets[j] = ('nudge', pm['speed'], duration_ticks)
                elif pm.get('type') == 'idle':
                    duration_ticks = max(1, round(pm['duration_s'] / self.dt))
                    self._pre_move_targets[j] = ('idle', duration_ticks)
                elif pm.get('type') == 'gripper_move':
                    duration_ticks = max(1, round(pm['duration_s'] / self.dt))
                    self._pre_move_targets[j] = (
                        'gripper_move',
                        pm['position'], pm['speed'], pm['current'],
                        pm['activate'], pm['action'], pm['estop'], pm['release_dir'],
                        duration_ticks,
                    )
                else:
                    # position move — target ticks computed on first tick (needs Joint obj)
                    self._pre_move_targets[j] = (None, pm['speed'], pm['position_rad'])
                self._pre_move_ok[j] = 0
            # Ensure the pre-move timeout is at least as long as the longest
            # timed target (idle / nudge / gripper_move) so timed durations
            # are never cut short by the safety timeout.
            max_timed_ticks = 0
            for info in self._pre_move_targets.values():
                if info[0] == 'idle':
                    max_timed_ticks = max(max_timed_ticks, info[1])
                elif info[0] == 'nudge':
                    max_timed_ticks = max(max_timed_ticks, info[2])
                elif info[0] == 'gripper_move':
                    max_timed_ticks = max(max_timed_ticks, info[-1])
            self._pre_move_timeout_ticks = max(self._move_timeout_ticks, max_timed_ticks)
            self._seq_phase = self._PRE_MOVE
        else:
            self._enter_homing_phase(step)

    # ------------------------------------------------------------------
    def _enter_homing_phase(self, step):
        """Start homing the joints listed in this step."""
        home_joints = step['home_joints']
        gripper_mode = step.get('gripper_mode')
        self._active_states = {}
        for j in home_joints:
            # Use GripperHomingState for gripper firmware mode
            if gripper_mode == 'firmware' and j >= self._arm_count:
                if j >= self.joint_num:
                    # No CAN gripper selected — arrays were never extended, skip gracefully.
                    logging.warning(
                        f"[Homing] Skipping {self._jlabel(j, self._Motor)} in step {self._seq_idx + 1}: "
                        f"no CAN gripper configured (gripper_mode='firmware' — skipping)"
                    )
                    continue
                state = GripperHomingState(
                    j=j, dt=self.dt,
                    timeout_s=float(self._timeout_s[j]) if j < len(self._timeout_s)
                              else GripperHomingState._CALIBRATE_TIMEOUT_TIME
                )
            elif j >= self.joint_num:
                # Joint index is beyond the configured arrays — happens when a
                # homing sequence step references the gripper (mode='motor') but
                # the robot is using a non-CAN gripper (do_gripper=False), so the
                # arrays were never extended.  Skip gracefully.
                logging.warning(
                    f"[Homing] Skipping {self._jlabel(j, self._Motor)} in step {self._seq_idx + 1}: "
                    f"index out of range for homing arrays (non-CAN gripper, "
                    f"gripper_mode='{gripper_mode}' — motor homing not applicable)"
                )
                continue
            else:
                state = self._make_joint_state(j)
            self._active_states[j] = state
        # Legacy compat: set _active_j to first joint
        if home_joints:
            self._active_j = home_joints[0]
            self._active_state = self._active_states.get(self._active_j)
        self._seq_phase = self._HOMING
        if len(home_joints) > 1:
            labels = [self._jlabel(j, self._Motor) for j in home_joints]
            logging.info(f"[Homing] Parallel homing [{', '.join(labels)}]")
        elif home_joints:
            logging.info(
                f"[Homing] Starting {self._jlabel(home_joints[0], self._Motor)} "
                f"(step {self._seq_idx + 1}/{len(self._steps)})"
            )

    @property
    def firmware_gripper_homing_active(self) -> bool:
        """True when the current homing step is running firmware gripper calibration.
        RTI.py uses this to skip send_gripper_commands so the gripper gets only
        the empty-poll Send_gripper_data_pack() sent by GripperHomingState.tick()."""
        if self._seq_phase != self._HOMING:
            return False
        for state in self._active_states.values():
            if isinstance(state, GripperHomingState):
                return True
        return False

    @property
    def gripper_homing_active(self) -> bool:
        """True when the homing controller owns the gripper CAN slot.
        RTI.py must suppress send_gripper_commands whenever this is True.

        Covers three cases:
          - firmware/motor homing   (_HOMING phase, gripper in _active_states)
          - gripper_move pre_move   (_PRE_MOVE phase, gripper in _pre_move_targets)
          - holding last command    (_last_gripper_move set — keep suppressing so
                                    the replayed command in _send_idle is not
                                    overwritten by RTI's normal gripper loop)
        """
        if self._last_gripper_move is not None:
            return True
        if self._seq_phase == self._HOMING:
            for j in self._active_states:
                if j >= self._arm_count:
                    return True
        if self._seq_phase == self._PRE_MOVE:
            for j in self._pre_move_targets:
                if j >= self._arm_count:
                    return True
        return False

    # ------------------------------------------------------------------
    def step(self, Motor, Joint, shared_data) -> str:
        """
        Call every RT tick while in HOMING mode.

        Returns:
            'running'  — in progress
            'done'     — all joints homed; switch to ENABLED
            'failed'   — a joint failed; switch to ERROR
        """
        if self._seq_phase == self._DONE:
            return 'done'
        if self._seq_phase == self._FAILED:
            return 'failed'
        if self._seq_phase == self._IDLE:
            logging.warning("[Homing] step() called before start()")
            return 'running'

        if self._seq_phase == self._PRE_MOVE:
            return self._do_step_pre_move(Motor, Joint, shared_data)
        elif self._seq_phase == self._HOMING:
            return self._do_step_homing(Motor, Joint, shared_data)
        elif self._seq_phase == self._POST_MOVE:
            return self._do_step_post_move(Motor, Joint, shared_data)
        elif self._seq_phase == self._MOVE_TO:
            return self._do_step_move_to(Motor, Joint, shared_data)

        return 'running'

    # ------------------------------------------------------------------
    def _do_step_pre_move(self, Motor, Joint, shared_data) -> str:
        """Execute pre-moves for the current step."""
        self._move_tick_count += 1
        active_move_joints = set(self._pre_move_targets.keys())

        # Keep non-moving joints sending zero-torque commands so their
        # CAN data stays fresh and the staleness detector never fires.
        for i in range(self.joint_num):
            if i not in active_move_joints:
                self._send_idle(Motor, i)

        all_arrived = True
        for j, move_info in self._pre_move_targets.items():

            if move_info[0] == 'idle':
                # Put the motor driver into idle mode for a timed duration.
                # The driver does not respond to Send_Idle, so we send it at
                # least twice (first two ticks) to ensure it is received, then
                # follow up with Send_Respond_Encoder_data to keep getting
                # motor feedback for the remainder of the idle period.
                duration_ticks = move_info[1]
                if self._move_tick_count <= 2:
                    Motor[j].Send_Idle()
                else:
                    Motor[j].Send_Respond_Encoder_data()
                if self._move_tick_count < duration_ticks:
                    all_arrived = False
                continue

            if move_info[0] == 'nudge':
                # Velocity-based timed nudge — no position target, just run for duration_ticks.
                # After duration elapses, hold with zero velocity until other pre-moves finish.
                speed = int(move_info[1])
                duration_ticks = move_info[2]
                if self._move_tick_count < duration_ticks:
                    Motor[j].Send_data_pack_1(Position=None, Speed=speed, Current=0)
                    all_arrived = False
                else:
                    Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
                continue

            if move_info[0] == 'gripper_move':
                # Gripper position/speed/current command held for duration_ticks,
                # then replayed by _send_idle for the rest of homing.
                # position and speed are 0-255; current is mA; flags are 0/1.
                _, position, speed, current, activate, action, estop, release_dir, duration_ticks = move_info
                Motor[j].Send_gripper_data_pack(
                    position_=position, speed_=speed, current_=current,
                    activate_=activate, action_=action,
                    ESTOP_=estop, Release_dir_=release_dir,
                )
                self._last_gripper_move = (position, speed, current, activate, action, estop, release_dir)
                if self._move_tick_count < duration_ticks:
                    all_arrived = False
                continue

            # Compute position target once (needs Joint object)
            if move_info[0] is None:
                if j < self._arm_count and j < len(Joint):
                    target_ticks = int(Joint[j].get_encoder_position(move_info[2]))
                else:
                    target_ticks = 0
                self._pre_move_targets[j] = (target_ticks, move_info[1], move_info[2])
                move_info = self._pre_move_targets[j]

            target_ticks = move_info[0]
            speed = int(move_info[1])
                                        # Position=target_ticks
            Motor[j].Send_data_pack_1(Position=None, Speed=speed, Current=0)

            # Check arrival
            curr_pos = Motor[j].position
            if curr_pos is not None:
                if abs(int(curr_pos) - target_ticks) < self._MOVE_TOL_TICKS:
                    self._pre_move_ok[j] = self._pre_move_ok.get(j, 0) + 1
                else:
                    self._pre_move_ok[j] = 0

            if self._pre_move_ok.get(j, 0) < self._move_stable_ticks:
                all_arrived = False

        # Timeout — use per-step timeout that respects timed pre-move durations
        if self._move_tick_count > self._pre_move_timeout_ticks:
            logging.warning(
                f"[Homing] Pre-move timed out at step {self._seq_idx + 1} "
                f"— joint(s) {list(active_move_joints)} could not reach target "
                f"(robot may not be calibrated yet). Continuing to homing."
            )
            all_arrived = True

        if all_arrived:
            for j in active_move_joints:
                self._send_idle(Motor, j)
            step = self._steps[self._seq_idx]
            self._enter_homing_phase(step)

        return 'running'

    # ------------------------------------------------------------------
    def _do_step_homing(self, Motor, Joint, shared_data) -> str:
        """Tick all active joint homing state machines in parallel."""
        active_joints = set(self._active_states.keys())

        # Keep non-homing joints sending zero-torque commands so their
        # CAN data stays fresh and the staleness detector never fires.
        for i in range(self.joint_num):
            if i not in active_joints:
                self._send_idle(Motor, i)

        # Tick each active joint
        done_joints = []
        for j, state in list(self._active_states.items()):
            joint_obj = Joint[j] if j < self._arm_count else None
            result = state.tick(Motor[j], joint_obj, shared_data)

            if result == 'failed':
                self._seq_phase = self._FAILED
                return 'failed'
            elif result == 'done':
                done_joints.append(j)

        # Remove completed joints from active set
        for j in done_joints:
            state = self._active_states[j]
            if isinstance(state, GripperHomingState):
                # Firmware gripper homing has no triggered_pos — nothing to record
                pass
            elif state._triggered_pos is not None:
                self._homed_results[j] = state._triggered_pos
                # For gripper: compute and store ticks_per_meter conversion factor
                if j >= self._arm_count and hasattr(self, '_gripper_stroke_mm'):
                    stroke_m = self._gripper_stroke_mm / 1000.0
                    gear_r = getattr(self, '_gripper_gear_r', 0.0)
                    if hasattr(shared_data, 'gripper'):
                        shared_data.gripper['gripper_homed_endstop_ticks'][0] = state._triggered_pos
                        shared_data.gripper['gripper_stroke_m'][0] = stroke_m
                        # Compute ticks_per_meter from mechanical parameters.
                        # Gear_r is the pinion pitch radius.  One pinion drives both jaws,
                        # so total opening per motor revolution = 2 × 2π × Gear_r.
                        # ticks_per_meter_of_opening = encoder_counts / (4π × Gear_r)
                        if gear_r > 0:
                            _GRIPPER_ENCODER_BITS = 14  # All Spectral BLDC motors use 14-bit encoders
                            encoder_counts = 2 ** _GRIPPER_ENCODER_BITS
                            tpm = encoder_counts / (4.0 * math.pi * gear_r)
                            shared_data.gripper['gripper_ticks_per_meter'][0] = tpm
                            logging.info(
                                f"[Homing] Gripper ticks_per_meter = {tpm:.1f} "
                                f"(from Gear_r={gear_r}, encoder={_GRIPPER_ENCODER_BITS}bit, "
                                f"endstop={state._triggered_pos}, stroke={self._gripper_stroke_mm}mm)"
                            )
                        else:
                            # Fallback: estimate from homing travel (only correct if gripper was fully closed at power-on)
                            endstop_ticks = abs(state._triggered_pos)
                            if endstop_ticks > 0 and stroke_m > 0:
                                tpm = endstop_ticks / stroke_m
                                shared_data.gripper['gripper_ticks_per_meter'][0] = tpm
                                logging.warning(
                                    f"[Homing] Gripper Gear_r not set — estimating ticks_per_meter = {tpm:.1f} "
                                    f"from homing travel (may be inaccurate if gripper was not fully closed at power-on)"
                                )
                        if gear_r > 0:
                            shared_data.gripper['gripper_gear_r'][0] = gear_r
            else:
                logging.warning(
                    f"[Homing] {self._jlabel(j, Motor)} completed but _triggered_pos is None "
                    f"— home reference was not set (CAN read failure during settle)"
                )
            del self._active_states[j]

        # Update legacy compat pointers
        if self._active_states:
            first_active = next(iter(self._active_states))
            self._active_j = first_active
            self._active_state = self._active_states[first_active]

        # All done? Advance to next step
        if not self._active_states:
            return self._advance_sequence(shared_data)

        return 'running'

    # ------------------------------------------------------------------
    def _do_step_post_move(self, Motor, Joint, shared_data) -> str:
        """Execute sequence-level post-moves (park joints after all homing)."""
        self._move_tick_count += 1
        active_move_joints = set(self._pre_move_targets.keys())

        for i in range(self.joint_num):
            if i not in active_move_joints:
                self._send_idle(Motor, i)

        all_arrived = True
        for j, move_info in self._pre_move_targets.items():
            if move_info[0] is None:
                pm = [p for p in self._post_move_list if p['joint'] == j][0]
                if j < self._arm_count and j < len(Joint):
                    target_ticks = int(Joint[j].get_encoder_position(pm['position_rad']))
                else:
                    target_ticks = 0
                self._pre_move_targets[j] = (target_ticks, pm['speed'])
                move_info = self._pre_move_targets[j]

            target_ticks = move_info[0]
            speed = int(move_info[1])   
                                            #Position=target_ticks
            Motor[j].Send_data_pack_1(Position=None, Speed=speed, Current=0)

            curr_pos = Motor[j].position
            if curr_pos is not None:
                if abs(int(curr_pos) - target_ticks) < self._MOVE_TOL_TICKS:
                    self._pre_move_ok[j] = self._pre_move_ok.get(j, 0) + 1
                else:
                    self._pre_move_ok[j] = 0

            if self._pre_move_ok.get(j, 0) < self._move_stable_ticks:
                all_arrived = False

        if self._move_tick_count > self._move_timeout_ticks:
            logging.warning("[Homing] Post-move timed out — finishing")
            all_arrived = True

        if all_arrived:
            for j in active_move_joints:
                Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
            self._seq_phase = self._DONE
            shared_data.homed_state.set("HOMED")
            logging.info("[Homing] All joints homed successfully.")
            logging.info("[Homing] ── Home positions set per joint ─────────────")
            for j in sorted(self._homed_results):
                logging.info(f"[Homing]   {self._jlabel(j, self._Motor)}: endstop encoder pos = {self._homed_results[j]}")
            logging.info("[Homing] ─────────────────────────────────────────────")
            return 'done'

        return 'running'

    # ------------------------------------------------------------------
    def _do_step_move_to(self, Motor, Joint, shared_data) -> str:
        """Execute smooth interpolated move_to commands for the current step.

        Each joint moves from its current position to a target angle over a
        specified duration using cubic Hermite (smoothstep) interpolation.
        This gives zero velocity at both start and end for smooth motion.
        """
        self._move_to_tick += 1
        active_joints = set(self._move_to_targets.keys())

        # Keep non-moving joints alive (prevent CAN staleness)
        for i in range(self.joint_num):
            if i not in active_joints:
                self._send_idle(Motor, i)

        all_done = True
        for j, info in self._move_to_targets.items():
            # Capture start position and compute end position on first tick
            if info['start_ticks'] is None:
                curr_pos = Motor[j].position
                if curr_pos is None:
                    # Motor position not yet available — hold and retry next tick
                    Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
                    all_done = False
                    continue
                info['start_ticks'] = int(curr_pos)
                info['end_ticks'] = int(
                    Joint[j].get_encoder_position(info['position_rad'])
                )
                logging.info(
                    f"[Homing] move_to {self._jlabel(j, Motor)}: "
                    f"{info['start_ticks']} → {info['end_ticks']} ticks "
                    f"({info['position_rad']:.4f} rad) over "
                    f"{info['duration_ticks']} ticks "
                    f"({info['duration_ticks'] * self.dt:.2f} s)"
                )

            if info['done']:
                # Hold final position
                Motor[j].Send_data_pack_1(
                    Position=info['end_ticks'], Speed=0, Current=0
                )
                continue

            # Cubic Hermite (smoothstep): s(tau) = 3*tau^2 - 2*tau^3
            tau = min(1.0, self._move_to_tick / info['duration_ticks'])
            s = 3.0 * tau * tau - 2.0 * tau * tau * tau
            setpoint = int(
                info['start_ticks']
                + (info['end_ticks'] - info['start_ticks']) * s
            )

            Motor[j].Send_data_pack_1(Position=setpoint, Speed=0, Current=0)

            if tau >= 1.0:
                # Interpolation done — now verify motor actually reached target
                curr_pos = Motor[j].position
                if curr_pos is not None and \
                   abs(int(curr_pos) - info['end_ticks']) < self._MOVE_TOL_TICKS:
                    info['settle_ok'] = info.get('settle_ok', 0) + 1
                else:
                    info['settle_ok'] = 0

                if info.get('settle_ok', 0) >= self._move_stable_ticks:
                    info['done'] = True
                    logging.info(f"[Homing] move_to {self._jlabel(j, Motor)} arrived and settled.")
                else:
                    all_done = False
            else:
                all_done = False

        # Timeout safety: max duration + 2 seconds margin
        max_duration = max(
            info['duration_ticks'] for info in self._move_to_targets.values()
        )
        timeout_ticks = max_duration + max(1, round(2.0 / self.dt))
        if self._move_to_tick > timeout_ticks:
            logging.warning("[Homing] move_to timed out — advancing")
            for j in active_joints:
                Motor[j].Send_data_pack_1(Position=None, Speed=0, Current=0)
            return self._advance_sequence(shared_data)

        if all_done:
            return self._advance_sequence(shared_data)

        return 'running'

    # ------------------------------------------------------------------
    def _advance_sequence(self, shared_data) -> str:
        """Advance to the next step, or finish if all done."""
        next_idx = self._seq_idx + 1
        if next_idx >= len(self._steps):
            self._seq_phase = self._DONE
            shared_data.homed_state.set("HOMED")
            logging.info("[Homing] All joints homed successfully.")
            logging.info("[Homing] ── Home positions set per joint ─────────────")
            for j in sorted(self._homed_results):
                logging.info(f"[Homing]   {self._jlabel(j, self._Motor)}: endstop encoder pos = {self._homed_results[j]}")
            logging.info("[Homing] ─────────────────────────────────────────────")
            return 'done'

        self._enter_step(next_idx)
        return 'running'

    # ------------------------------------------------------------------
    # Backward-compat properties — tests inspect these on the controller
    # ------------------------------------------------------------------

    @property
    def _phase(self):
        """Returns the per-joint phase for backward compat with tests.

        Tests check ctrl._phase for values like 'homing', 'backing_off',
        'post_moving', etc. These are now per-joint phases on JointHomingState.
        """
        if self._active_state is not None:
            return self._active_state.phase
        return self._seq_phase

    @property
    def _pass(self):
        return self._active_state._pass if self._active_state else 1

    @property
    def _tick_count(self):
        return self._active_state._tick_count if self._active_state else 0

    @property
    def _post_move_target(self):
        return self._active_state._post_move_target if self._active_state else None

    # ------------------------------------------------------------------
    def _signed_speed(self, j: int, factor: float = 1.0) -> int:
        """Backward compat — delegates to active state or computes directly."""
        spd = int(self.homing_speeds[j] * factor)
        return -spd if int(self.homing_directions[j]) == 1 else spd

    def _fail(self, j: int, shared_data, reason: str):
        """Backward compat — fail the sequence."""
        logging.error(f"[Homing] {self._jlabel(j, self._Motor)} FAILED: {reason}")
        shared_data.rti["homing_status"][j] = STATUS_FAILED
        self._seq_phase = self._FAILED

def sector_home_example():
    None
    """
    # TODO this is temp calibration, will remove it later
    for i in range(joint_num):
    unwrapped_position_raw = Joint[i].unwrap_position(shared_data.motor['motor_position'][i])
    #[i] =  Joint[i].get_joint_position(motor_position[i])

    if sector_home_status[i] == 0:
    sector_home_status[i] = 1
    Joint[i].determine_sector(shared_data.motor['motor_position'][i])
    """