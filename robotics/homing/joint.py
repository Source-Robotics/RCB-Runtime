"""One joint (or the gripper motor) homing: approach and stall / Hall detection, dwell, back-off, pause,
second approach, release, settle (the home reference), the optional post-home move."""
# Split out of robotics/homing.py (2026-10-09, TODO/homing_plan.md H3) — moved verbatim; the recording
# (dummy_test_scripts/test_homing_golden.py) proves the behaviour unchanged.

import logging
import math

from robotics.homing.reference import _update_home_reference
from robotics.homing.status import STATUS_DONE, STATUS_FAILED, STATUS_RUNNING


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
    # The release position is the MEAN of the valid reads in this window up to the sample tick (2026-10-09, homing plan
    # H2.2): one read was None in ~1 of 4 real homings — Spectral_BLDC.UnpackData() wipes position for every reply it
    # decodes, the health polls' included — and the latch then fell back to the end of the release.
    _RELEASE_AVG_TIME      = 0.04
    # If a Hall trigger fires within this many ticks the motor was already sitting
    # on/near the sensor — back off and re-approach rather than accepting the position.
    _HALL_PRE_CLEAR_GUARD_S = 0.5  # 500 ms at 12000 ticks/s ≈ 6000 ticks of travel
    # 2026-10-09: the pre-clear backs off for the guard PLUS backoff_s, so the next approach needs longer than the guard to
    # reach the sensor again — with only backoff_s (0.3 s < the 0.5 s guard) a J6 that started ON its sensor re-triggered
    # "too early" on every approach, forever (each retry also restarted the timeout). After this many tries homing FAILS.
    _HALL_PRE_CLEAR_MAX     = 3

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
            two_pass             (bool)  : True = back-off + slow second approach (3D-printer-style)
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
            self._release_avg_ticks = max(1, min(self._release_sample_tick, round(self._RELEASE_AVG_TIME / self.dt)))
        else:
            self._release_ticks = 0
            self._release_sample_tick = 0
            self._release_avg_ticks = 0
        self._release_reads: list = []

        # Two-pass max difference: per-joint value from XML, fallback to class default
        self._two_pass_max_diff = (int(two_pass_max_diff)
                                   if not math.isnan(two_pass_max_diff)
                                   else self._TWO_PASS_MAX_DIFF_TICKS)
        # 2026-10-14 (user: a failing joint "flops first"): a joint that slides more than twice its two-pass tolerance
        # during its release is not resting on an endstop (a false stall) — it fails at once instead of finishing the
        # release (normal: J1 -12..-116, J2 +1064..+2144, J3 -744..-1125, J5 ~+10 ticks; seen failing: J2 up to
        # +21684, J3 up to -19506)
        self._release_slide_max = 2 * self._two_pass_max_diff

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
        self._pre_clear_ticks  = self._hall_pre_clear_guard + self._backoff_ticks
        self._pre_clear_tries  = 0

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
        # Pass 2's STALL position, taken before a release phase lets the joint relax (2026-10-09). The two-pass check
        # compares pass 1 with THIS: comparing with the released position counted the gearbox's play as "not
        # repeatable" — the black arm's J2 relaxed 2062 ticks (limit 2000) with its two stalls 45 ticks apart.
        self._pass2_stall_pos  = None
        self._two_pass_done    = False
        self._post_move_target = None
        self._post_move_ok     = 0
        self._limits_sent      = False
        # This homing's numbers for the arm table's history (robotics/homing_arms.py, 2026-10-09): set in _do_settle
        self.two_pass_diff     = None   # ticks between the two stalls
        self.release_relaxed   = None   # ticks the release let the joint move from its stall
        # 2026-10-14: how far the joint travelled from its start to pass 1's endstop (ticks / s) — measured, not yet
        # judged (user: "measure the travel first"); and why it failed, for the error the controller raises.
        self._start_pos        = None   # the first valid position read on pass 1's approach
        self._last_valid_pos   = None   # the newest valid position read on an approach (a read can come back None)
        self.travel_ticks      = None
        self.travel_s          = None
        self.fail_reason       = None

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
        self._pre_clear_tries  = 0
        self._pass             = 1
        self._last_pos         = None
        self._triggered_pos    = None
        self._pass1_pos        = None
        self._pass2_stall_pos  = None
        self._two_pass_done    = False
        self._post_move_target = None
        self._post_move_ok     = 0
        self._limits_sent      = False
        self.two_pass_diff     = None
        self.release_relaxed   = None
        self._start_pos        = None
        self._last_valid_pos   = None
        self.travel_ticks      = None
        self.travel_s          = None
        self.fail_reason       = None

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
            return self._do_release(motor, shared_data)
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

            endstop_hit, detection_method = self._detect_stall(motor, speed)

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
                self._pre_clear_tries += 1
                if self._pre_clear_tries > self._HALL_PRE_CLEAR_MAX:
                    self._fail(shared_data,
                               f"the Hall sensor still triggered right after backing off "
                               f"{self._HALL_PRE_CLEAR_MAX} times — check the sensor and its magnet")
                    return 'running'
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

            if not self.has_hall_sensor:
                # The position at the stall: this tick's read, else the newest valid one of the approach (the stall
                # window has just confirmed it moved less than the threshold since). A read can come back None (the
                # library wipes it for a health-poll reply); on 2026-10-14 pass 1's None made the two-pass check skip
                # a false stall.
                stall_pos = motor.position if motor.position is not None else self._last_valid_pos
                if self._pass == 1 and self._start_pos is not None and stall_pos is not None:
                    self.travel_ticks = int(stall_pos) - self._start_pos
                    self.travel_s = self._tick_count * self.dt
                    logging.info(
                        f"[Homing] {self._label} travel to the endstop (pass 1): {self.travel_ticks:+d} ticks in "
                        f"{self.travel_s:.3f} s (from {self._start_pos} to {stall_pos})")
            if self._pass == 1 and self.two_pass and not self.has_hall_sensor:
                # Save pass 1 endstop position for later comparison with pass 2
                self._pass1_pos = stall_pos
                if self._pass1_pos is None:
                    self._fail(shared_data, "no position was read on the way to the endstop (pass 1) — the two passes "
                                            "cannot be compared")
                    return 'running'
                # Dwell briefly to let the motor actually stop before reversing.
                # Hall sensor joints skip two-pass: the digital edge IS the
                # reference, a second approach doesn't improve accuracy.
                self._phase       = self._DWELL
                self._dwell_count = 0
            elif self._release_enabled:
                # Release phase: command a controlled current to relieve gearbox
                # preload, then latch the true resting endstop position.
                self._pass2_stall_pos = stall_pos if not self.has_hall_sensor else motor.position
                # 2026-10-14: the two passes compared HERE, before the release — two passes apart are a joint that
                # cannot hold itself at its homing current; the release (less current) let it fall further first
                if (self._pass1_pos is not None and self._pass2_stall_pos is not None
                        and not self._two_pass_check(motor, shared_data, self._pass2_stall_pos)):
                    return 'failed'
                self._phase         = self._RELEASE
                self._release_count = 0
                self._release_reads = []
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
    def _detect_stall(self, motor, speed):
        """A stall joint's endstop test for this tick: the position window, the current window, the trigger. Returns
        (endstop_hit, detection_method). Moved out of _do_homing verbatim (2026-10-14: its complexity, radon)."""
        endstop_hit     = False
        detection_method = ''
        curr_pos = motor.position
        if curr_pos is not None:
            self._last_valid_pos = int(curr_pos)
            if self._pass == 1 and self._start_pos is None:
                self._start_pos = int(curr_pos)

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
        return endstop_hit, detection_method

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

        if self._pre_clear_count >= self._pre_clear_ticks:
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
            logging.info(
                f"[Homing] {self._label} pre-clear backoff complete "
                f"({self._pre_clear_ticks * self.dt:.3f} s / {self._pre_clear_ticks} ticks), "
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
            self._pass2_stall_pos = None
            logging.info(
                f"[Homing] {self._label} pause complete, starting slow second approach "
                f"({self.rehome_speed_factor*100:.0f}% speed)"
            )
        return 'running'

    # ------------------------------------------------------------------
    def _two_pass_check(self, motor, shared_data, pass2_pos) -> bool:
        """Pass 2's stall against pass 1's (logged; once per homing). Too far apart: FAILED, the joint's normal limits
        restored — False. With a release this runs at pass 2's stall (2026-10-14), without one after the settle."""
        self._two_pass_done = True
        diff = abs(int(pass2_pos) - int(self._pass1_pos))
        self.two_pass_diff = diff
        logging.info(
            f"[Homing] {self._label} two-pass difference: "
            f"{diff} ticks (pass1={self._pass1_pos}, "
            f"pass2={pass2_pos}, "
            f"max={self._two_pass_max_diff})"
        )
        if diff > self._two_pass_max_diff:
            self.fail_reason = (f"the two passes stopped {diff} ticks apart (max {self._two_pass_max_diff}) — "
                                f"the endstop trigger is not repeatable (a false stall: the joint could not move, "
                                f"or something blocked it?)")
            logging.error(
                f"[Homing] {self._label} FAILED: two-pass difference "
                f"{diff} ticks exceeds max {self._two_pass_max_diff} — "
                f"endstop trigger is not repeatable"
            )
            shared_data.rti["homing_status"][self.j] = STATUS_FAILED
            self._restore_limits(motor)
            self._phase = self._FAILED
            return False
        return True

    # ------------------------------------------------------------------
    def _do_release(self, motor, shared_data=None) -> str:
        """Command a controlled current to release gearbox preload at endstop.

        The motor is driven in current-command mode (Position=None, Speed=None,
        Current=release_current_ma) for release_duration_s seconds.  The endstop
        encoder position is latched at the configured sample point within this
        window — after the gearbox has relaxed but before the joint drifts away.
        """
        self._release_count += 1
        pos = motor.position
        if (shared_data is not None and pos is not None and self._pass2_stall_pos is not None
                and abs(int(pos) - int(self._pass2_stall_pos)) > self._release_slide_max):
            # Sliding, not resting on an endstop: hold it (zero velocity) and give it its normal limits NOW (2026-10-14)
            motor.Send_data_pack_1(Position=None, Speed=0, Current=0)
            self._restore_limits(motor)
            self._fail(shared_data, f"it slid {int(pos) - int(self._pass2_stall_pos):+d} ticks during its release "
                                    f"(more than {self._release_slide_max}, twice its two-pass tolerance) — it was not "
                                    f"resting on an endstop: a false stall (could it carry the arm from where it was?)")
            return 'failed'
        current_cmd = int(self.release_current_ma)
        motor.Send_data_pack_1(Position=None, Speed=None, Current=current_cmd)

        # Latch the mean of the valid reads in the window up to the configured sample tick
        if self._release_sample_tick - self._release_avg_ticks < self._release_count <= self._release_sample_tick:
            if motor.position is not None:
                self._release_reads.append(int(motor.position))
        if self._release_count == self._release_sample_tick:
            reads = self._release_reads
            self._triggered_pos = int(round(sum(reads) / len(reads))) if reads else None
            logging.info(
                f"[Homing] {self._label} release: latched position "
                f"{self._triggered_pos} at tick {self._release_count}/"
                f"{self._release_ticks} (mean of {len(reads)} of the last {self._release_avg_ticks} reads)"
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
                # A joint without a reference is not homed (2026-10-14: it used to carry on, uncalibrated).
                self._fail(shared_data, "its position never read valid at the endstop — no home reference")
                self._restore_limits(motor)
                return 'failed'

        # Two-pass consistency check: if both passes recorded a position,
        # verify they agree within tolerance. A large difference means the
        # endstop trigger is unreliable (e.g. debris, loose mechanism).
        # With a release phase the two STALLS are compared (pass 2's was kept before the release moved the joint); the
        # release's relaxation is the drivetrain's play, logged on its own.
        pass2_pos = self._pass2_stall_pos if self._pass2_stall_pos is not None else self._triggered_pos
        if self._pass2_stall_pos is not None and self._triggered_pos is not None:
            self.release_relaxed = int(self._triggered_pos) - int(self._pass2_stall_pos)
            logging.info(
                f"[Homing] {self._label} release relaxed "
                f"{int(self._triggered_pos) - int(self._pass2_stall_pos):+d} ticks "
                f"(stall {self._pass2_stall_pos} -> latched {self._triggered_pos})"
            )
        if (self._pass1_pos is not None and pass2_pos is not None and not self._two_pass_done
                and not self._two_pass_check(motor, shared_data, pass2_pos)):
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
        self.fail_reason = reason
        logging.error(f"[Homing] {self._label} FAILED: {reason}")
        shared_data.rti["homing_status"][self.j] = STATUS_FAILED
        self._phase = self._FAILED
