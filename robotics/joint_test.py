"""A small powered test move of one joint, homed or not (2026-10-18; the first-run check's stage 3, TODO/first_run_plan.md
— user: "automatic move detection"; 5 deg at 5 deg/s, the current limit 1.5 x the joint's worst gravity torque, the
robot XML's <first_run_move>).

The joint moves `amplitude` away from where it stands and back, on a cosine ramp (peak speed `speed`), holding HOLD_S
at each end, in POSITION mode in the drive's own encoder frame (raw motor ticks: the homing approach's way — it needs
no reference, so it works unhomed), with its drive's current limit lowered to `current_ma` for the test. Every other
joint keeps its normal command. The tick conversion is the joint's own: Joint.get_encoder_speed(1 rad/s) ticks per rad
(gear, encoder resolution and <dir> included).

Stopped at once — the joint's live limits back, its normal command the same tick — when it falls ABORT_DEG behind the
commanded motion (blocked, an endstop, too little current, a motor that does not follow), on leaving IDLE / ENABLED,
on any pending mode change (homing, a jog, an error, the E-stop), by the operator. Never beside the Hall sensor check
or a freed joint (all three use the joint's current limit).

Published in rti['joint_test'] (RESULT_FIELDS): the joint (1-based, 0 = none), the state (IDLE / RUNNING / DONE /
FELL_BEHIND / STOPPED), the commanded amplitude, how far it got (the largest displacement the commanded way), the
largest tracking error, the peak motor current, where it ended against its start (deg), and a sequence number that
counts finished tests.
"""
from __future__ import annotations

import math

LIMIT_REPEATS = 2            # each limits frame sent twice (the CAN library swallows a dropped frame)
RESEND_TICKS = 125           # the test limit re-sent every ~0.5 s at 250 Hz (a reconnect reload would restore the limit)
HOLD_S = 0.5                 # still at each end of the move
ABORT_DEG = 2.0              # behind the commanded motion by more than this: stopped
_FELL_BEHIND_TEXT = f"more than {ABORT_DEG:g} deg behind the commanded motion"   # built once: tick builds no string
MAX_AMPLITUDE_DEG = 15.0     # the executor refuses more
MAX_SPEED_DPS = 10.0

IDLE, RUNNING, DONE, FELL_BEHIND, STOPPED = 0, 1, 2, 3, 4
STATE_NAMES = {IDLE: "idle", RUNNING: "running", DONE: "done", FELL_BEHIND: "fell_behind", STOPPED: "stopped"}
RESULT_FIELDS = ("joint", "state", "amplitude_deg", "moved_deg", "max_error_deg", "peak_current_ma", "end_error_deg",
                 "seq")
_I = {k: i for i, k in enumerate(RESULT_FIELDS)}


def profile(t, amplitude, ramp_s, hold_s=HOLD_S):
    """(offset, speed) at time t of the test move: a cosine ramp out (0 .. amplitude in ramp_s), hold, a cosine ramp
    back, hold; (None, 0) once finished."""
    if t < ramp_s:
        x = math.pi * t / ramp_s
        return 0.5 * amplitude * (1.0 - math.cos(x)), 0.5 * amplitude * math.pi / ramp_s * math.sin(x)
    t -= ramp_s
    if t < hold_s:
        return amplitude, 0.0
    t -= hold_s
    if t < ramp_s:
        x = math.pi * t / ramp_s
        return 0.5 * amplitude * (1.0 + math.cos(x)), -0.5 * amplitude * math.pi / ramp_s * math.sin(x)
    t -= ramp_s
    if t < hold_s:
        return 0.0, 0.0
    return None, 0.0


def ramp_seconds(amplitude, speed):
    """The cosine ramp whose peak speed is `speed` (rad, rad/s)."""
    return 0.5 * math.pi * abs(amplitude) / speed


class JointTest:
    """`motors` the CAN library's motor objects (Send_Limits, .position, .current), `joints` the toolbox Joint objects
    (get_encoder_speed), `velocity_limit` / `current_limit` the live per-joint limits (stop() restores them), `ctx` the
    dispatch context (joint_test_joint / _pos / _speed), `rti` the rti shared-memory dict, `dt` the tick (s)."""

    def __init__(self, motors, joints, joint_num, velocity_limit, current_limit, ctx, rti, dt, log=None, done=None,
                 q_meas=None, soft_lo=None, soft_hi=None):
        self.motors, self.joints, self.joint_num = motors, joints, int(joint_num)
        # On a homed arm the test's travel must stay inside the soft limits (2026-10-24, user): the measured joint
        # angles (rad) and the robot XML's soft limits (rad). Unhomed, the angles mean nothing: no check then.
        self.q_meas, self.soft_lo, self.soft_hi = q_meas, soft_lo, soft_hi
        self.velocity_limit, self.current_limit = velocity_limit, current_limit
        self.ctx, self.rti, self.dt, self.log, self.done = ctx, rti, float(dt), log, done
        self.joint = None
        self._reset_ctx()
        self.rti["joint_test"][:] = 0.0

    @property
    def active(self) -> bool:
        return self.joint is not None

    # ------------------------------------------------------------------------------------------------ cold path
    def _reset_ctx(self):
        self.ctx.joint_test_joint = -1
        self.ctx.joint_test_pos = None
        self.ctx.joint_test_speed = 0

    def _send_limits(self, j, current):
        for _ in range(LIMIT_REPEATS):
            self.motors[j].Send_Limits(Velocity_limit=int(self.velocity_limit[j]), Current_limit=int(current))

    def start(self, joint, amplitude_deg, speed_dps, current_ma, idle, enabled, mode_pending, booting,
              other_active=False, homed=False):
        """Start a test move of `joint` (0-based). Returns None, or why it was refused (nothing is sent then)."""
        j = int(joint)
        if not 0 <= j < self.joint_num:
            return f"no joint {j + 1} (this robot has J1-J{self.joint_num})"
        if self.active:
            return f"J{self.joint + 1}'s test move is still running"
        if not idle or mode_pending:
            return "only in IDLE (homed or not)"
        if not enabled:
            return "the arm is disabled: enable it first (a test move is powered motion)"
        if booting:
            return "RTI is still configuring the drives (the first ~1.2 s after its start)"
        if other_active:
            return "the Hall sensor check or a freed joint is active — stop it first"
        a, v, c = float(amplitude_deg), float(speed_dps), float(current_ma)
        if not (0.0 < abs(a) <= MAX_AMPLITUDE_DEG and 0.0 < v <= MAX_SPEED_DPS and 0.0 < c <= self.current_limit[j]):
            return (f"amplitude 0-{MAX_AMPLITUDE_DEG:g} deg, speed 0-{MAX_SPEED_DPS:g} deg/s, current up to the "
                    f"joint's limit ({int(self.current_limit[j])} mA)")
        if homed and self.q_meas is not None and self.soft_lo is not None and self.soft_hi is not None:
            q = float(self.q_meas[j])
            end = q + math.radians(a)
            lo, hi = float(self.soft_lo[j]), float(self.soft_hi[j])
            if not lo <= end <= hi:
                return (f"J{j + 1} would move to {math.degrees(end):.1f} deg, past its soft limits "
                        f"({math.degrees(lo):.1f} to {math.degrees(hi):.1f} deg) — move it away first")
        pos = self.motors[j].position
        if pos is None:
            return f"no position read from J{j + 1} yet — try again"
        self._k = float(self.joints[j].get_encoder_speed(1.0))          # ticks per rad, signed
        self._amp = math.radians(a)
        self._ramp = ramp_seconds(self._amp, math.radians(v))
        self._start_ticks = float(pos)
        self._t = 0.0
        self._ticks = 0
        self._moved = self._max_err = self._peak_i = self._end = 0.0
        self._test_ma = c
        self._send_limits(j, c)
        self.joint = j
        self.ctx.joint_test_joint = j
        self.ctx.joint_test_pos = int(round(self._start_ticks))
        self.ctx.joint_test_speed = 0
        r = self.rti["joint_test"]
        r[:_I["seq"]] = 0.0
        r[_I["joint"]], r[_I["state"]], r[_I["amplitude_deg"]] = j + 1, RUNNING, a
        if self.log:
            self.log(f"J{j + 1} test move: {a:+.1f} deg and back at {v:.1f} deg/s, current limit {c:.0f} mA")
        return None

    def stop(self, reason="stopped", state=STOPPED):
        if not self.active:
            return
        j = self.joint
        self._reset_ctx()                       # this tick's dispatch sends the joint its normal command again
        self._send_limits(j, self.current_limit[j])
        self.joint = None
        r = self.rti["joint_test"]
        r[_I["state"]], r[_I["moved_deg"]] = state, math.degrees(self._moved)
        r[_I["max_error_deg"]], r[_I["peak_current_ma"]] = math.degrees(self._max_err), self._peak_i
        r[_I["end_error_deg"]] = math.degrees(self._end)
        r[_I["seq"]] += 1
        text = (f"J{j + 1} test move {STATE_NAMES[state]} ({reason}): moved {math.degrees(self._moved):+.2f} deg, "
                f"behind at most {math.degrees(self._max_err):.2f} deg, peak {self._peak_i:.0f} mA, ended "
                f"{math.degrees(self._end):+.2f} deg from its start")
        if self.log:
            self.log(text + "; limits restored")
        if self.done:
            self.done(text)

    # ------------------------------------------------------------------------------------------------ per tick
    def tick(self, ok):
        """Every tick while active, BEFORE RTI's mode management and dispatch: sets this tick's command for the joint.
        `ok` = IDLE, ENABLED and no mode change pending. No allocation on the normal path."""
        if not ok:
            self.stop("the robot left IDLE / was disabled or a mode change was requested")
            return
        j = self.joint
        self._ticks += 1
        if self._ticks % RESEND_TICKS == 0:
            self.motors[j].Send_Limits(Velocity_limit=int(self.velocity_limit[j]), Current_limit=int(self._test_ma))
        pos, cur = self.motors[j].position, self.motors[j].current
        offset, speed = profile(self._t, self._amp, self._ramp)
        if pos is not None:
            moved = (float(pos) - self._start_ticks) / self._k
            if offset is not None:
                err = abs(moved - offset)
                if err > self._max_err:
                    self._max_err = err
            if moved * self._amp > self._moved * self._amp:      # the largest displacement the commanded way
                self._moved = moved
            self._end = moved
            if offset is not None and self._max_err > math.radians(ABORT_DEG):
                self.stop(_FELL_BEHIND_TEXT, FELL_BEHIND)
                return
        if cur is not None and abs(float(cur)) > self._peak_i:
            self._peak_i = abs(float(cur))
        if offset is None:
            self.stop("finished", DONE)
            return
        self.ctx.joint_test_pos = int(round(self._start_ticks + self._k * offset))
        self.ctx.joint_test_speed = int(self._k * speed)
        self._t += self.dt
