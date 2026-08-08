"""JOG mode handler — velocity-ramped joint jogging with gravity compensation.

All internal state (position, velocity, limits, acceleration) is in joint space:
  position  — radians
  velocity  — rad/s
  limits    — radians

Motor-space conversion (ticks, ticks/s) and CAN sends happen inside dispatch
(utility/mode_dispatch.py::_to_motor_space + _send), not here. This handler
returns a ControlOutput describing the joint-space setpoint; dispatch owns
the rest of the pipeline.

set_jog() / stop_jog() are called from RTI.py's ZMQ command handler and write
into plain Python attributes — no shared memory, no locking needed since RTI.py
is single-threaded on the command-handling path.
"""

import numpy as np
from utility.tick_types import TickResult, ControlOutput
from utility.activity_log import get_rti_activity_logger, log_activity

# Extra margin applied to the kinematic stopping distance before triggering deceleration.
# Accounts for mechanical lag (motor current limiting, control loop latency, measurement noise).
# 1.5 = start decelerating 50 % earlier than the theoretical minimum stopping distance.
_LOOKAHEAD_SAFETY_FACTOR = 1.5

# ---------------------------------------------------------------------------
# Velocity ramp profile functions
#
# Signature: (vel, target_vel, accel_max, jerk_max, dt, current_accel) -> (new_vel, new_accel_state)
#
# current_accel / new_accel_state: per-joint acceleration tracking (S-curve jerk state).
# Trapezoidal ignores it and always returns 0.0.
# ---------------------------------------------------------------------------

def _ramp_trapezoidal(vel, target_vel, accel_max, jerk_max, dt, current_accel):
    error = target_vel - vel
    new_vel = vel + float(np.clip(error, -accel_max * dt, accel_max * dt))
    # Report the acceleration actually applied this tick. Trapezoidal does not NEED an accel
    # state (unlike s-curve, which limits its rate of change), but the handler publishes this
    # as joint_acceleration_commanded — returning a hardcoded 0.0 made that channel claim the
    # arm was never accelerating while the ramp was plainly running.
    return new_vel, (new_vel - vel) / dt if dt > 0 else 0.0


def _ramp_scurve(vel, target_vel, accel_max, jerk_max, dt, current_accel):
    """Jerk-limited velocity ramp. Tracks per-joint acceleration to limit its rate of change."""
    vel_error = target_vel - vel

    if abs(vel_error) > 1e-9:
        accel_sign = 1.0 if vel_error > 0 else -1.0
    else:
        accel_sign = 0.0

    # If accelerating toward target, check whether to start reducing acceleration to avoid
    # velocity overshoot. Velocity gained while reducing current_accel to 0 at max jerk: a² / (2·j).
    if jerk_max > 0 and accel_sign * current_accel > 0:
        v_coast = (current_accel * current_accel) / (2.0 * jerk_max)
        if abs(vel_error) <= v_coast:
            accel_sign = 0.0

    desired_accel = accel_sign * accel_max
    max_delta     = jerk_max * dt
    new_accel     = current_accel + float(np.clip(desired_accel - current_accel, -max_delta, max_delta))
    new_accel     = float(np.clip(new_accel, -accel_max, accel_max))

    # Snap to rest when both velocity error and remaining acceleration fit within one tick.
    if abs(vel_error) < accel_max * dt and abs(new_accel) < jerk_max * dt:
        return float(target_vel), 0.0

    return float(vel + new_accel * dt), new_accel


_PROFILE_MAP = {
    "trapezoidal": _ramp_trapezoidal,
    "scurve":      _ramp_scurve,
}

_PROFILE_TO_IDX = {"trapezoidal": 0, "scurve": 1}
_CONTROL_TO_IDX = {"pid": 0,         "pd":     1}


class JogHandler:

    def __init__(self, joint_num: int, max_vel_rad, delta_t: float,
                 limit_neg=None, limit_pos=None,
                 accel_time: float = 0.55, accel_limit=None,
                 profile: str = "scurve",
                 jerk_factor: float = 3.0,
                 control_mode: str = "pid",
                 speed_pct: float = 0.20,
                 shared_data=None):
        """
        Args:
            joint_num:     Number of arm joints.
            max_vel_rad:   Per-joint velocity limits in rad/s (from XML joint_velocity_limit).
            delta_t:       RT loop period in seconds.
            limit_neg:     Per-joint negative soft limits in radians.
            limit_pos:     Per-joint positive soft limits in radians.
            accel_time:    Time (s) to ramp from 0 to full speed.
            profile:       'trapezoidal' or 'scurve' velocity ramp.
            jerk_factor:   jerk = accel * factor (S-curve only).
            control_mode:  'pid' (Send_data_pack_1) or 'pd' (Send_data_pack_PD).
            speed_pct:     Initial slider value 0.0–1.0 (no effect until set_jog).
            shared_data:   Optional reference to data.shared_data module. When
                           provided, the handler mirrors its config to
                           shared_data.rti[] so new GUI tabs see live values.

        Defaults match the <jog_defaults> example in config/system.xml. RTI.py
        seeds the real defaults from system.xml at startup.
        """
        self.joint_num   = joint_num
        self._delta_t    = delta_t
        self._shared_data = shared_data

        self._max_vel_rad = np.asarray(max_vel_rad, dtype=np.float64)
        # The PHYSICAL ceiling from the robot XML (<joint_acceleration_limit>). accel_time is
        # an operator PREFERENCE — a GUI slider — and preferences live inside limits, they do
        # not override them. Before this clamp existed, dragging the slider to its 0.05 s floor
        # commanded 30 rad/s^2 on joint 1 against a declared limit of 3.0: ten times over.
        # +inf when the XML is silent, so a robot without the field behaves exactly as before.
        self._accel_limit = (np.full_like(self._max_vel_rad, np.inf) if accel_limit is None
                             else np.where(np.isnan(np.asarray(accel_limit, dtype=np.float64)),
                                           np.inf, np.asarray(accel_limit, dtype=np.float64)))
        self._accel_time  = float(max(0.05, accel_time))
        self._accel_rad   = self._compute_accel(self._accel_time)   # rad/s²

        self._limit_neg = np.asarray(limit_neg, dtype=np.float64) if limit_neg is not None else None
        self._limit_pos = np.asarray(limit_pos, dtype=np.float64) if limit_pos is not None else None

        # Pre-allocated buffers — never reallocated inside step()
        self._target_pos_rad  = np.zeros(joint_num, dtype=np.float64)  # rad
        self._target_vel_rad  = np.zeros(joint_num, dtype=np.float64)  # rad/s

        # Per-joint direction block: 0 = free, +1 = positive rad/s blocked, -1 = negative rad/s blocked.
        # Stored in joint space so comparisons against jog_direction_rad are consistent for all joints.
        # Set when lookahead or hard clamp fires. Cleared only when user presses the opposite direction.
        self._blocked_dir = np.zeros(joint_num, dtype=np.int8)

        # Velocity ramp profile
        self._jerk_factor = float(max(0.5, jerk_factor))
        self._jerk_rad    = self._accel_rad * self._jerk_factor  # rad/s³
        self._accel_state = np.zeros(joint_num, dtype=np.float64)  # per-joint current accel (S-curve state)

        if profile not in _PROFILE_MAP:
            profile = "scurve"
        self._profile_name = profile
        self._ramp_fn      = _PROFILE_MAP[profile]

        # Control mode: "pid" → Send_data_pack_1, "pd" → Send_data_pack_PD
        if control_mode not in ("pid", "pd"):
            control_mode = "pid"
        self._control_mode = control_mode

        # Jog input state — written by ZMQ command handler, read by step()
        self._jog_axis         = -1
        self._jog_direction    = 0    # -1 or +1 (joint space: positive = increasing joint angle)
        self._jog_speed_pct    = float(max(0.0, min(1.0, speed_pct)))

        # Activity log — session/limit edge tracking (see TODO/activity_log_design.md).
        self._activity_logger    = get_rti_activity_logger()
        self._jog_session_active = False

        # Mirror initial config to shared memory so the GUI sees XML defaults
        # immediately on the first page load.
        self._mirror_all_to_shared_memory()

    # -------------------------------------------------------------------------
    # Shared-memory mirroring — keeps rti[] in sync so new GUI tabs see live state
    # -------------------------------------------------------------------------

    def _mirror_all_to_shared_memory(self):
        if self._shared_data is None:
            return
        rti = self._shared_data.rti
        rti['jog_speed_pct'][0]    = self._jog_speed_pct
        rti['jog_accel_time'][0]   = self._accel_time
        rti['jog_jerk_factor'][0]  = self._jerk_factor
        rti['jog_profile_idx'][0]  = _PROFILE_TO_IDX[self._profile_name]
        rti['jog_control_mode'][0] = _CONTROL_TO_IDX[self._control_mode]

    # -------------------------------------------------------------------------
    # External interface — called from RTI.py ZMQ command handler
    # -------------------------------------------------------------------------

    def set_jog(self, axis: int, direction: int, speed_pct: float):
        # Switching to a different joint — clear the previous joint's block so its
        # indicator resets. The new joint's block state is unaffected.
        if axis != self._jog_axis and self._jog_axis >= 0:
            if self._blocked_dir[self._jog_axis] != 0:
                log_activity(self._activity_logger, "RTI", "info", "LIMIT_CLEARED",
                             message=f"axis={self._jog_axis}")
            self._blocked_dir[self._jog_axis] = 0
        # Pressing the opposite direction on the same joint unblocks it.
        # Pressing the same blocked direction keeps the block (no re-entry).
        if self._blocked_dir[axis] != 0 and self._blocked_dir[axis] != direction:
            log_activity(self._activity_logger, "RTI", "info", "LIMIT_CLEARED", message=f"axis={axis}")
            self._blocked_dir[axis] = 0
        self._jog_axis      = axis
        self._jog_direction = direction
        self._jog_speed_pct = max(0.0, min(1.0, speed_pct))
        if self._shared_data is not None:
            self._shared_data.rti['jog_speed_pct'][0] = self._jog_speed_pct

        if direction != 0 and not self._jog_session_active:
            self._jog_session_active = True
            log_activity(self._activity_logger, "RTI", "info", "JOG_STARTED",
                         message=f"axis={axis} direction={direction:+d} speed_pct={self._jog_speed_pct:.2f}")
        elif direction == 0 and self._jog_session_active:
            self._jog_session_active = False
            log_activity(self._activity_logger, "RTI", "info", "JOG_STOPPED", message=f"axis={axis}")

    def stop_jog(self):
        self._jog_direction = 0
        # Intentionally do NOT clear _blocked_dir — block persists across button releases
        # so the user cannot wiggle past the soft limit by rapidly tapping.
        if self._jog_session_active:
            self._jog_session_active = False
            log_activity(self._activity_logger, "RTI", "info", "JOG_STOPPED",
                         message=f"axis={self._jog_axis}")

    def _compute_accel(self, accel_time):
        """Ramp acceleration for a given accel_time, never above the XML limit."""
        return np.minimum(self._max_vel_rad / accel_time, self._accel_limit)

    def set_accel_time(self, accel_time: float):
        accel_time = max(0.05, accel_time)  # floor at 50 ms
        self._accel_time = float(accel_time)
        self._accel_rad  = self._compute_accel(accel_time)
        self._jerk_rad   = self._accel_rad * self._jerk_factor
        if self._shared_data is not None:
            self._shared_data.rti['jog_accel_time'][0] = self._accel_time

    def set_profile(self, name: str):
        """Switch velocity ramp profile. Options: 'trapezoidal', 'scurve'."""
        if name not in _PROFILE_MAP:
            raise ValueError(f"Unknown jog profile {name!r}. Choose from: {list(_PROFILE_MAP)}")
        self._profile_name   = name
        self._ramp_fn        = _PROFILE_MAP[name]
        self._accel_state[:] = 0.0
        if self._shared_data is not None:
            self._shared_data.rti['jog_profile_idx'][0] = _PROFILE_TO_IDX[name]

    def set_jerk_factor(self, factor: float):
        """Set jerk as a multiple of accel (default 3.0). Higher = snappier S-curve corners."""
        self._jerk_factor = max(0.5, factor)
        self._jerk_rad    = self._accel_rad * self._jerk_factor
        if self._shared_data is not None:
            self._shared_data.rti['jog_jerk_factor'][0] = self._jerk_factor

    def set_control_mode(self, mode: str):
        """Switch jog control mode. Options: 'pid' (Send_data_pack_1), 'pd' (Send_data_pack_PD)."""
        if mode not in ("pid", "pd"):
            raise ValueError(f"Unknown jog control mode {mode!r}. Choose from: 'pid', 'pd'")
        self._control_mode = mode
        if self._shared_data is not None:
            self._shared_data.rti['jog_control_mode'][0] = _CONTROL_TO_IDX[mode]

    # -------------------------------------------------------------------------
    # Mode handler interface — called by mode_dispatch
    # -------------------------------------------------------------------------

    def enter(self, ctx, **kwargs):
        """Capture current joint position as starting point; clear velocity."""
        self._target_pos_rad[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
        self._target_vel_rad[:] = 0.0
        self._accel_state[:]    = 0.0
        self._blocked_dir[:] = 0
        self._jog_axis      = -1
        self._jog_direction = 0
        self._jog_speed_pct = 0.0

    def step(self, ctx):
        dt        = self._delta_t
        joint_pos = ctx.shared_data.joint['joint_position']  # rad, read-only view

        for i in range(self.joint_num):
            pos   = joint_pos[i]
            vel_i = self._target_vel_rad[i]  # rad/s

            # ── target velocity (joint space) ─────────────────────────────────
            if i == self._jog_axis and self._jog_direction != 0:
                if self._blocked_dir[i] == self._jog_direction:
                    target_vel = 0.0  # blocked at limit — must press opposite to unblock
                else:
                    target_vel = self._jog_direction * self._max_vel_rad[i] * self._jog_speed_pct
            else:
                target_vel = 0.0

            # ── lookahead deceleration ────────────────────────────────────────
            # Everything is in joint space (rad, rad/s) — no dir confusion.
            # stopping_dist is the kinematic distance to bring vel_i down to 0
            # under the *active* ramp profile:
            #   trapezoidal: d = ½·v²/a            (instant accel snap to −a)
            #   s-curve:     d = ½·v²/a + v·a/(2j) (extra term for jerk ramp-in/out)
            # Without the s-curve term the lookahead under-estimates by a·v/(2j),
            # the joint overshoots the soft limit, and the hard clamp at the
            # bottom of this loop fires — snapping accel_state to 0 and giving
            # a noticeably abrupter stop than a normal jog-button release.
            if abs(vel_i) > 0:
                a = self._accel_rad[i]
                trap_dist = 0.5 * abs(vel_i) * (abs(vel_i) / a)
                if self._ramp_fn is _ramp_scurve and self._jerk_rad[i] > 0:
                    stopping_dist = (trap_dist + 0.5 * abs(vel_i) * (a / self._jerk_rad[i])) \
                                    * _LOOKAHEAD_SAFETY_FACTOR
                else:
                    stopping_dist = trap_dist * _LOOKAHEAD_SAFETY_FACTOR

                if vel_i > 0 and self._limit_pos is not None:
                    if (self._limit_pos[i] - pos) <= stopping_dist:
                        target_vel = 0.0
                        if self._blocked_dir[i] == 0:
                            log_activity(self._activity_logger, "RTI", "warning", "LIMIT_REACHED",
                                         message=f"axis={i} direction=+1")
                        self._blocked_dir[i] = 1
                elif vel_i < 0 and self._limit_neg is not None:
                    if (pos - self._limit_neg[i]) <= stopping_dist:
                        target_vel = 0.0
                        if self._blocked_dir[i] == 0:
                            log_activity(self._activity_logger, "RTI", "warning", "LIMIT_REACHED",
                                         message=f"axis={i} direction=-1")
                        self._blocked_dir[i] = -1

            # ── velocity ramp ─────────────────────────────────────────────────
            self._target_vel_rad[i], self._accel_state[i] = self._ramp_fn(
                vel_i, target_vel, self._accel_rad[i], self._jerk_rad[i], dt, self._accel_state[i]
            )

            # ── hard clamp (safety net) ───────────────────────────────────────
            # Fires if lookahead undershot due to measurement noise or integrator drift.
            if self._limit_neg is not None and pos <= self._limit_neg[i] and vel_i < 0:
                self._target_vel_rad[i] = 0.0
                self._accel_state[i]    = 0.0
                if self._blocked_dir[i] == 0:
                    log_activity(self._activity_logger, "RTI", "warning", "LIMIT_REACHED",
                                 message=f"axis={i} direction=-1 hard_clamp")
                self._blocked_dir[i]    = -1
            if self._limit_pos is not None and pos >= self._limit_pos[i] and vel_i > 0:
                self._target_vel_rad[i] = 0.0
                self._accel_state[i]    = 0.0
                if self._blocked_dir[i] == 0:
                    log_activity(self._activity_logger, "RTI", "warning", "LIMIT_REACHED",
                                 message=f"axis={i} direction=+1 hard_clamp")
                self._blocked_dir[i]    = 1

            # ── position integration ──────────────────────────────────────────
            self._target_pos_rad[i] += self._target_vel_rad[i] * dt

        ctx.shared_data.joint['jog_blocked'][:self.joint_num] = self._blocked_dir

        # Gravity-comp feedforward only when enabled. Dispatch's _to_motor_space
        # converts Nm → mA in one in-place multiply.
        grav_on = bool(ctx.shared_data.rti['gravity_comp_enabled'][0])
        trq = ctx.shared_data.joint['joint_gravity_torque'][:self.joint_num] if grav_on else None

        # target == commanded in JOG (handler doesn't separate the two — the
        # lookahead and hard clamp mutate _target_*_rad in place, so the buffers
        # already reflect post-clamp values). Setting both explicitly per the
        # "tight" definition in control_output_refactor_plan.md §7.5 — matches
        # the meaning EXEC / RTI will use ("target = next waypoint after any
        # planner-side intervention").
        # acc_rad is the ramp's current acceleration (rad/s^2). It is TELEMETRY only — no CAN
        # pack carries acceleration (data_pack_1/PD are Position/Speed/Current), so publishing
        # it changes nothing the arm does. It exists so JOG populates the same channels EXEC
        # already does instead of leaving joint_acceleration_commanded pinned at 0.
        return TickResult(), ControlOutput(
            pos_rad=self._target_pos_rad,
            vel_rad=self._target_vel_rad,
            acc_rad=self._accel_state,
            trq_nm=trq,
            mode=self._control_mode,
            target_pos_rad=self._target_pos_rad,
            target_vel_rad=self._target_vel_rad,
            target_acc_rad=self._accel_state,
        )

    def exit(self, ctx):
        if self._jog_session_active:
            self._jog_session_active = False
            log_activity(self._activity_logger, "RTI", "info", "JOG_STOPPED", message="mode exit")
        self._target_vel_rad[:] = 0.0
        self._accel_state[:]    = 0.0
        self._blocked_dir[:] = 0
        ctx.shared_data.joint['jog_blocked'][:self.joint_num] = 0
