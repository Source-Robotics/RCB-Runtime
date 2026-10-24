"""JOG mode handler — joint jogging along a direction in joint space, with PLANNED, exact stops at every limit.

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

HOW A JOG MOVES (2026-10-07; TODO/self_collision_plan.md S3e, docs/self_collision.md). A jog is a DIRECTION in joint
space — a unit vector; today's buttons and joysticks set one joint (set_jog), set_jog_vector() takes any mix of joints
(multi-joint / joystick jogging, the user's requirement) — and a speed along it. It moves along the ray
ref + s * direction (ref = the commanded pose when the jog session began) and BRAKES along the same ray, all moving
joints together, so where it can stop is always on that line. The motion along the ray is one-dimensional and
planned by Ruckig every tick (the jog's own acceleration and jerk, the speed slider as the speed): cruise, and brake
so as to ARRIVE at the nearer limit with zero speed —
  * the soft joint limits along the ray (exact, computed here once per session), and
  * the self-collision limit along the ray: the collision worker scans the same ray and publishes how far it is free
    (rti['sc_ray_free'] for rti['sc_ray_session'] — a verified frontier, then the exact entry into the jog margin by
    bisection). Until the worker has answered for THIS session the free length is 0: the jog waits (~10-20 ms).
So a jog comes to rest AT the limit at any speed and acceleration — the user's rule ("the stops and distance need
to be consistent and respected"). Before this, limit stops were TRIGGERED (stop now, then the fixed ramp) and the
rest point depended on the speed: soft limits early by up to half the stopping distance, self-collision 5-22 mm.
A new direction waits until the previous motion has braked to rest, then starts a new session from where it stopped.
"""

import logging
import math

import numpy as np
from ruckig import ControlInterface, InputParameter, OutputParameter, Result, Ruckig

from utility.activity_log import get_rti_activity_logger, log_activity
from utility.tick_types import ControlOutput, TickResult

_PROFILE_TO_IDX = {"trapezoidal": 0, "scurve": 1}   # trapezoidal = Ruckig with unlimited jerk
_CONTROL_TO_IDX = {"pid": 0, "pd": 1}
_REST = 1e-9            # rad/s along the ray: at rest
_ARRIVED = 1e-6         # rad along the ray: at the planned limit


def ray_soft_free(ref, direction, lo, hi):
    """rad: the path length from `ref` along the unit joint-space `direction` to the first soft limit (inf when no
    moving joint has one). A joint already outside its range gives 0 for a direction that takes it further out and a
    positive length for one that brings it back — never further out (the rule every motion source follows)."""
    s = math.inf
    for j in range(len(direction)):
        dj = direction[j]
        if dj > 1e-12 and hi is not None:
            s = min(s, (hi[j] - ref[j]) / dj)
        elif dj < -1e-12 and lo is not None:
            s = min(s, (lo[j] - ref[j]) / dj)
    return max(0.0, s)


def ray_limits(direction, vmax, amax, jmax):
    """(vel, accel, jerk) along the ray: every moving joint within its own limit (|dq_j/dt| = |ds/dt| * |d_j|)."""
    v = a = j = math.inf
    for k in range(len(direction)):
        dk = abs(direction[k])
        if dk > 1e-12:
            v, a, j = min(v, vmax[k] / dk), min(a, amax[k] / dk), min(j, jmax[k] / dk)
    return v, a, j


class JogHandler:

    def __init__(self, joint_num: int, max_vel_rad, delta_t: float,
                 limit_neg=None, limit_pos=None,
                 accel_time: float = 0.55, accel_limit=None,
                 profile: str = "scurve",
                 jerk_factor: float = 3.0,
                 control_mode: str = "pid",
                 speed_pct: float = 0.20,
                 shared_data=None,
                 self_collision: bool = False):
        """
        Args:
            joint_num:     Number of arm joints.
            max_vel_rad:   Per-joint velocity limits in rad/s (from XML joint_velocity_limit).
            delta_t:       RT loop period in seconds.
            limit_neg:     Per-joint negative soft limits in radians.
            limit_pos:     Per-joint positive soft limits in radians.
            accel_time:    Time (s) to ramp from 0 to full speed.
            profile:       'trapezoidal' or 'scurve' (jerk-limited) motion along the ray.
            jerk_factor:   jerk = accel * factor (S-curve only).
            control_mode:  'pid' (Send_data_pack_1) or 'pd' (Send_data_pack_PD).
            speed_pct:     Initial slider value 0.0–1.0 (no effect until set_jog).
            shared_data:   Optional reference to data.shared_data module. When
                           provided, the handler mirrors its config to
                           shared_data.rti[] so new GUI tabs see live values,
                           and publishes the jog session for the collision worker.
            self_collision: True when <self_collision><enabled> is on: the worker's free length along the ray
                           limits the jog (and no answer yet = wait). False: soft limits only.

        Defaults match the <jog_defaults> example in config/system.xml. RTI.py
        seeds the real defaults from system.xml at startup.
        """
        self.joint_num   = joint_num
        self._delta_t    = delta_t
        self._shared_data = shared_data
        self._sc_enabled = bool(self_collision) and shared_data is not None

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
        self._accel_state     = np.zeros(joint_num, dtype=np.float64)  # rad/s²: the ray's acceleration per joint (telemetry + the inertia feedforward)
        self._tmp             = np.zeros(joint_num, dtype=np.float64)

        # Per-joint direction block indicator for the GUI: 0 = free, +1 / -1 = that direction is at a limit.
        # Set when a jog arrives at its limit (soft or self-collision); cleared when another direction is pressed.
        self._blocked_dir = np.zeros(joint_num, dtype=np.int8)

        # Motion profile along the ray
        if profile not in _PROFILE_TO_IDX:
            profile = "scurve"
        self._profile_name = profile
        self._jerk_factor = float(max(0.5, jerk_factor))
        self._jerk_rad    = self._accel_rad * self._jerk_factor  # rad/s³

        # Control mode: "pid" → Send_data_pack_1, "pd" → Send_data_pack_PD
        if control_mode not in ("pid", "pd"):
            control_mode = "pid"
        self._control_mode = control_mode

        # The requested jog (written by set_jog / set_jog_vector / stop_jog)
        self._want_dir    = np.zeros(joint_num, dtype=np.float64)    # unit vector, or zeros = no jog
        self._want_on     = False
        self._want_is_session = False                                # the request is the running session's direction
        self._jog_axis    = -1                                       # the axis of a single-joint jog (logs, indicator)
        self._jog_direction = 0
        self._jog_speed_pct = float(max(0.0, min(1.0, speed_pct)))

        # The jog session: one ray. ref = the commanded pose it began at; s / sv / sa = the position, speed and
        # acceleration along it (Ruckig's state); s_soft = the soft limits' free length along it.
        self._dir     = np.zeros(joint_num, dtype=np.float64)
        self._ref     = np.zeros(joint_num, dtype=np.float64)
        self._session = 0
        self._s = self._sv = self._sa = 0.0
        self._s_soft  = 0.0
        self._has_session = False
        self._v_ray = self._a_ray = self._j_ray = 0.0
        self._limits_dirty = True
        self._limit_logged = False

        # Ruckig along the ray, 1 DOF. ALLOCATION NOTE: new_position / new_velocity / new_acceleration return fresh
        # one-element lists — the same deliberate exception to the hot-path rule as motion/rti_handler.py's Ruckig
        # (no zero-copy view in the binding); the inputs are pre-allocated arrays. ~2 us per update (measured).
        self._rk     = Ruckig(1, delta_t)
        self._rk_in  = InputParameter(1)
        self._rk_out = OutputParameter(1)
        self._rk_x   = np.zeros(1); self._rk_v = np.zeros(1); self._rk_a = np.zeros(1)
        self._rk_tx  = np.zeros(1); self._rk_zero = np.zeros(1)
        self._rk_vmax = np.zeros(1); self._rk_amax = np.zeros(1); self._rk_jmax = np.zeros(1)
        self._rk_in.target_velocity = self._rk_zero
        self._rk_in.target_acceleration = self._rk_zero

        # Activity log — session/limit edge tracking (see TODO/activity_log_design.md). Texts built once: nothing is
        # formatted on the tick.
        self._activity_logger    = get_rti_activity_logger()
        self._jog_session_active = False
        self._cmd_id = None              # the jog command's id for JOG_STARTED (set_jog)
        self._limit_msg = [(f"axis={i} direction=-1", f"axis={i} direction=+1") for i in range(joint_num)]
        self._sc_msg = [(f"axis={i} direction=-1 self-collision", f"axis={i} direction=+1 self-collision") for i in range(joint_num)]

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

    def set_jog(self, axis: int, direction: int, speed_pct: float, cmd_id=None):
        """Jog one joint: `direction` +1 / -1 (0 = stop), speed as a fraction of that joint's velocity limit.
        `cmd_id`: the jog command's id, for the activity log's JOG_STARTED / JOG_STOPPED (2026-10-15)."""
        self._cmd_id = cmd_id
        if not 0 <= int(axis) < self.joint_num:      # the executor refuses it; never an IndexError in the loop (2026-10-24)
            logging.getLogger("rti").warning("jog refused: no joint %s (joints count from 0)", axis)
            return
        if direction == 0:
            self.stop_jog(cmd_id)
            return
        vec = self._tmp
        vec[:] = 0.0
        vec[axis] = 1.0 if direction > 0 else -1.0
        # the indicator: pressing anything but the blocked direction clears it
        for i in range(self.joint_num):
            if self._blocked_dir[i] != 0 and not (i == axis and self._blocked_dir[i] == (1 if direction > 0 else -1)):
                log_activity(self._activity_logger, "RTI", "info", "LIMIT_CLEARED", message=f"axis={i}")
                self._blocked_dir[i] = 0
        self._jog_axis, self._jog_direction = axis, (1 if direction > 0 else -1)
        self._request(vec, speed_pct, f"axis={axis} direction={self._jog_direction:+d}")

    def set_jog_vector(self, direction, speed_pct: float, cmd_id=None):
        """Jog several joints at once: `direction` is any joint-space vector (normalised here); the speed is a fraction
        of the fastest speed along it that keeps every moving joint within its own velocity limit. Stops at the
        nearer of the soft limits and the self-collision margin along that ray, exactly like a single-joint jog."""
        self._cmd_id = cmd_id
        d = np.asarray(direction, dtype=np.float64)[:self.joint_num]
        norm = float(np.linalg.norm(d))
        if norm < 1e-9:
            self.stop_jog(cmd_id)
            return
        for i in range(self.joint_num):
            if self._blocked_dir[i] != 0:
                log_activity(self._activity_logger, "RTI", "info", "LIMIT_CLEARED", message=f"axis={i}")
                self._blocked_dir[i] = 0
        self._jog_axis, self._jog_direction = -1, 0
        self._request(d / norm, speed_pct, "vector=" + np.array2string(d / norm, precision=3, separator=","))

    def _request(self, unit_dir, speed_pct, what):
        self._want_is_session = self._has_session and bool(np.array_equal(unit_dir, self._dir))
        self._want_dir[:] = unit_dir
        self._want_on = True
        # NaN / infinity: no speed (2026-10-24: max(0, min(1, nan)) is 1 — a NaN speed ran the joint at FULL speed)
        self._jog_speed_pct = max(0.0, min(1.0, speed_pct)) if math.isfinite(speed_pct) else 0.0
        if self._shared_data is not None:
            self._shared_data.rti['jog_speed_pct'][0] = self._jog_speed_pct
        if not self._jog_session_active:
            self._jog_session_active = True
            log_activity(self._activity_logger, "RTI", "info", "JOG_STARTED", id=self._cmd_id,
                         message=f"{what} speed_pct={self._jog_speed_pct:.2f}")

    def stop_jog(self, cmd_id=None):
        self._want_on = False
        # The indicator stays (a release does not unblock: pressing the same direction again stays at the limit).
        if self._jog_session_active:
            self._jog_session_active = False
            log_activity(self._activity_logger, "RTI", "info", "JOG_STOPPED", id=cmd_id,
                         message=f"axis={self._jog_axis}")

    def _compute_accel(self, accel_time):
        """Ramp acceleration for a given accel_time, never above the XML limit."""
        return np.minimum(self._max_vel_rad / accel_time, self._accel_limit)

    def set_accel_time(self, accel_time: float):
        accel_time = max(0.05, accel_time)  # floor at 50 ms
        self._accel_time = float(accel_time)
        self._accel_rad  = self._compute_accel(accel_time)
        self._jerk_rad   = self._accel_rad * self._jerk_factor
        self._limits_dirty = True
        if self._shared_data is not None:
            self._shared_data.rti['jog_accel_time'][0] = self._accel_time

    def set_profile(self, name: str):
        """Switch the motion profile along the ray. Options: 'trapezoidal' (unlimited jerk), 'scurve' (jerk-limited)."""
        if name not in _PROFILE_TO_IDX:
            raise ValueError(f"Unknown jog profile {name!r}. Choose from: {list(_PROFILE_TO_IDX)}")
        self._profile_name = name
        self._limits_dirty = True
        if self._shared_data is not None:
            self._shared_data.rti['jog_profile_idx'][0] = _PROFILE_TO_IDX[name]

    def set_jerk_factor(self, factor: float):
        """Set jerk as a multiple of accel (default 3.0). Higher = snappier S-curve corners."""
        self._jerk_factor = max(0.5, factor)
        self._jerk_rad    = self._accel_rad * self._jerk_factor
        self._limits_dirty = True
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
    # The jog session (cold: once per new direction)
    # -------------------------------------------------------------------------

    def _start_session(self):
        """A new ray from where the jog stands: the commanded pose, the requested direction, its soft-limit free
        length and limits; published for the collision worker (ref and direction BEFORE the session number)."""
        self._ref[:] = self._target_pos_rad
        self._dir[:] = self._want_dir
        self._has_session = self._want_is_session = True
        self._session += 1
        self._s = self._sv = self._sa = 0.0
        self._s_soft = ray_soft_free(self._ref, self._dir, self._limit_neg, self._limit_pos)
        self._limits_dirty = True
        self._limit_logged = False
        if self._shared_data is not None:
            rti = self._shared_data.rti
            rti['jog_ref'][:self.joint_num] = self._ref
            rti['jog_dir'][:self.joint_num] = self._dir
            rti['jog_session'][0] = self._session

    def _refresh_limits(self):
        jerk = self._jerk_rad if self._profile_name == "scurve" else np.full(self.joint_num, np.inf)
        self._v_ray, self._a_ray, self._j_ray = ray_limits(self._dir, self._max_vel_rad, self._accel_rad, jerk)
        self._rk_amax[0], self._rk_jmax[0] = self._a_ray, self._j_ray
        self._limits_dirty = False

    # -------------------------------------------------------------------------
    # Mode handler interface — called by mode_dispatch
    # -------------------------------------------------------------------------

    def enter(self, ctx, **kwargs):
        """Capture current joint position as starting point; no motion, no session."""
        self._target_pos_rad[:] = ctx.shared_data.joint['joint_position'][:self.joint_num]
        self._target_vel_rad[:] = 0.0
        self._accel_state[:]    = 0.0
        self._blocked_dir[:] = 0
        self._want_on = self._want_is_session = self._has_session = False
        self._want_dir[:] = 0.0
        self._dir[:] = 0.0
        self._ref[:] = self._target_pos_rad                          # no session: the targets stay where the arm is
        self._s = self._sv = self._sa = 0.0
        self._jog_axis, self._jog_direction = -1, 0
        self._jog_speed_pct = 0.0
        if self._shared_data is not None:
            self._shared_data.rti['jog_dir'][:self.joint_num] = 0.0

    def step(self, ctx):
        rti = ctx.shared_data.rti

        # 1. the session: a new direction starts once the previous motion is at rest
        moving_same = self._want_on and self._want_is_session
        if self._want_on and not moving_same and abs(self._sv) <= _REST:
            self._start_session()
            moving_same = True
        if self._limits_dirty and self._has_session:
            self._refresh_limits()

        # 2. the free length along the ray: the nearer of the soft limits and the self-collision limit
        s_lim = self._s_soft
        sc_final = True
        if moving_same and self._sc_enabled:
            if rti['sc_ray_session'][0] == self._session:
                s_sc = rti['sc_ray_free'][0]
                sc_final = rti['sc_ray_done'][0] >= 0.5
            else:
                s_sc, sc_final = 0.0, False                  # the worker has not answered for this ray yet: wait
            if s_sc < s_lim:
                s_lim = s_sc

        # 3. Ruckig along the ray: to the limit with zero speed (jogging), or to rest (released / switching)
        inp = self._rk_in
        self._rk_x[0], self._rk_v[0], self._rk_a[0] = self._s, self._sv, self._sa
        inp.current_position, inp.current_velocity, inp.current_acceleration = self._rk_x, self._rk_v, self._rk_a
        if moving_same:
            inp.control_interface = ControlInterface.Position
            self._rk_tx[0] = s_lim
            inp.target_position = self._rk_tx
            self._rk_vmax[0] = max(self._v_ray * self._jog_speed_pct, 1e-9)
        else:
            inp.control_interface = ControlInterface.Velocity
            self._rk_vmax[0] = max(self._v_ray, abs(self._sv), 1e-9)
        inp.max_velocity, inp.max_acceleration, inp.max_jerk = self._rk_vmax, self._rk_amax, self._rk_jmax
        if self._has_session:
            res = self._rk.update(inp, self._rk_out)
            if res == Result.Working or res == Result.Finished:
                self._s = self._rk_out.new_position[0]
                self._sv = self._rk_out.new_velocity[0]
                self._sa = self._rk_out.new_acceleration[0]
            else:                                            # cold: Ruckig refused — hold where the command is
                self._sv = self._sa = 0.0

        # 4. the joint targets: on the ray
        np.multiply(self._dir, self._s, out=self._tmp)
        np.add(self._ref, self._tmp, out=self._target_pos_rad)
        np.multiply(self._dir, self._sv, out=self._target_vel_rad)
        np.multiply(self._dir, self._sa, out=self._accel_state)

        # 5. arrived at a final limit while the jog is still held: the GUI indicator and one activity-log line
        if (moving_same and sc_final and not self._limit_logged and abs(self._sv) <= _REST
                and self._s >= s_lim - _ARRIVED):
            self._limit_logged = True
            soft = s_lim >= self._s_soft - _ARRIVED
            if self._jog_axis >= 0:
                k = 1 if self._jog_direction > 0 else 0
                self._blocked_dir[self._jog_axis] = self._jog_direction
                log_activity(self._activity_logger, "RTI", "warning", "LIMIT_REACHED" if soft else "SELF_COLLISION_BLOCK",
                             message=self._limit_msg[self._jog_axis][k] if soft else self._sc_msg[self._jog_axis][k])
            else:
                for i in range(self.joint_num):
                    if self._dir[i] != 0.0:
                        self._blocked_dir[i] = 1 if self._dir[i] > 0 else -1
                log_activity(self._activity_logger, "RTI", "warning", "LIMIT_REACHED" if soft else "SELF_COLLISION_BLOCK",
                             message="vector jog")

        ctx.shared_data.joint['jog_blocked'][:self.joint_num] = self._blocked_dir

        # Gravity-comp feedforward only when enabled. Dispatch's _to_motor_space
        # converts Nm → mA in one in-place multiply.
        grav_on = bool(rti['gravity_comp_enabled'][0])
        trq = ctx.shared_data.joint['joint_gravity_torque'][:self.joint_num] if grav_on else None

        # target == commanded in JOG (the planned ray is already within every limit). acc_rad is the ray's current
        # acceleration (rad/s^2) per joint (direction x the 1-DOF profile's acceleration). No CAN pack carries an
        # acceleration, but since 2026-10-10 dispatch feeds it forward as inertia / Coriolis torque
        # (utility/mode_dispatch._add_dynamics_ff, behind the robot-wide switches) — it DOES change what the arm gets.
        # EXEC fills the same channel.
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
        self._want_on = self._want_is_session = self._has_session = False
        self._dir[:] = 0.0
        self._ref[:] = self._target_pos_rad
        self._s = self._sv = self._sa = 0.0
        self._target_vel_rad[:] = 0.0
        self._accel_state[:]    = 0.0
        self._blocked_dir[:] = 0
        ctx.shared_data.joint['jog_blocked'][:self.joint_num] = 0
        if self._shared_data is not None:
            self._shared_data.rti['jog_dir'][:self.joint_num] = 0.0
