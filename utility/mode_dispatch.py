"""
Mode dispatch infrastructure.

Provides TickContext / TickResult / ControlOutput, simple function handlers
for stateless modes (BOOTING, IDLE, ACTIVE_ERROR, SAFETY_STOP), a dispatch
wrapper for the existing HomingController, and the build_mode_dispatch()
factory that wires everything into a MODE_HANDLERS lookup table.

Per-tick pipeline (dispatch_step):
  1. handler.step(ctx)                    → returns (TickResult, ControlOutput)
  2. _commit_joint_state(ctx, output)     → writes joint_*_commanded + joint_*_target
                                            with fallback (§5.1)
  3. _fk_commanded(ctx)                   → mirror measured TCP →
                                            TCP_pose_commanded / TCP_speed_commanded /
                                            TCP_pose_commanded_matrix
  4. _commit_tcp_target(ctx, output)      → writes TCP_*_target with fallback to
                                            TCP_*_commanded (§5.2). Must run AFTER
                                            _fk_commanded so fallback reads live values.
  5. _to_motor_space(ctx, output)         → rad → ticks, Nm → mA
  6. _send(ctx, motor_cmd, output.mode)   → one CAN send per tick

Handlers are pure setpoint computers in joint space. Commit, convert, and
send live in dispatch — handlers cannot forget them and never call them
directly.

HOMING is the one exception: it owns multi-phase per-joint CAN sends inside
robotics/homing.py. Its dispatch wrapper returns SELF_MANAGED and the
pipeline above is skipped.

Design notes (from mode_dispatch_plan_2.md and control_output_refactor_plan.md):
  - Function handlers for stateless modes; class handlers for stateful modes.
  - TickContext is built once before the RT loop; only rti_udp_data and
    current_tick are reassigned per tick (attribute swap, no allocation).
  - All numpy buffers are pre-allocated in build_mode_dispatch() — never
    inside step() or the dispatch helpers.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import logging
logger = logging.getLogger("rti.loop")  # docs/logging.md "Logger names"
import os
import time

import numpy as np

from utility.state_machine import RobotMode
from utility.tick_types import TickContext, TickResult, ControlOutput, SELF_MANAGED
from utility.kinematics_backend import should_use_pinocchio, write_rpy_xyz
from communication.can_message_handlers import (
    send_motor_commands,
    send_motor_commands_pd,
    write_motor_config_to_shared,
)
from robotics.dynamics import build_torque_to_ma_factor
from robotics.jog import JogHandler
from motion.exec_handler import ExecHandler


# ---------------------------------------------------------------------------
# Module-level pre-allocated buffers — set by build_mode_dispatch() before
# the RT loop starts. Reused every tick; never re-allocated.
# ---------------------------------------------------------------------------
_zeros_rad           = None  # np.zeros(joint_num, float64) — for vel_rad=0 handlers
_motor_pos_buf       = None  # float64 buffer for converted positions (ticks)
_motor_vel_buf       = None  # float64 buffer for converted speeds    (ticks/s)
_motor_curr_buf      = None  # float64 buffer for converted currents  (mA)
_torque_to_mA_factor = None  # per-joint scaling: trq_nm * factor → motor mA
_tick_ok             = TickResult()
# Preallocated per the hot-path allocation rule — FLASHING claims the gripper
# channel so RTI's gripper send block stays silent too.
_tick_flashing       = TickResult(gripper_owned=True)
_output_vel_zero: ControlOutput = None     # type: ignore[assignment]  # built by build_dispatch() before any tick
_output_torque_zero: ControlOutput = None  # type: ignore[assignment]  # (the same)
_output_idle_gravity = ControlOutput()
_idle_trq_buf: np.ndarray = None  # type: ignore[assignment]  # gravity (+ walls) + friction comp., preallocated in build

_MODE_PROFILE_ENABLED = os.environ.get("RTI_MODE_PROFILE", "0") == "1"
_MODE_PROFILE_INTERVAL = max(1, int(os.environ.get("RTI_MODE_PROFILE_INTERVAL", "500")))
_MODE_PROFILE_NAMES = ("handler", "commit_joint", "fk_commanded", "commit_tcp", "to_motor", "send")
_mode_profile_sum = {name: 0 for name in _MODE_PROFILE_NAMES}
_mode_profile_max = {name: 0 for name in _MODE_PROFILE_NAMES}
_mode_profile_count = 0


def _mode_profile_record(samples):
    global _mode_profile_count
    _mode_profile_count += 1
    for name, elapsed_ns in samples:
        _mode_profile_sum[name] += elapsed_ns
        if elapsed_ns > _mode_profile_max[name]:
            _mode_profile_max[name] = elapsed_ns

    if _mode_profile_count >= _MODE_PROFILE_INTERVAL:
        total_ns = sum(_mode_profile_sum.values())
        # One queued log record per report (RTI logs through a queue), not a blocking print in the RT thread.
        lines = [f"dispatch profile, last {_mode_profile_count} calls:"]
        for name in _MODE_PROFILE_NAMES:
            avg_ms = (_mode_profile_sum[name] / _mode_profile_count) / 1e6
            max_ms = _mode_profile_max[name] / 1e6
            share = (_mode_profile_sum[name] / total_ns * 100.0) if total_ns > 0 else 0.0
            lines.append(f"  {name:<14} avg={avg_ms:8.3f} ms  max={max_ms:8.3f} ms  share={share:5.1f}%")
            _mode_profile_sum[name] = 0
            _mode_profile_max[name] = 0
        _mode_profile_count = 0
        logger.info("\n".join(lines))


# ---------------------------------------------------------------------------
# Pipeline helpers — called by dispatch_step after the handler returns.
# ---------------------------------------------------------------------------

def _commit_joint_state(ctx, output):
    """Write joint-space _commanded and _target shared memory from ControlOutput.

    Fallback rules per docs/commanded_target_actual.md §5.1:

      commanded ← handler value, OR fallback to:
                    pos:  measured joint_position
                    vel:  measured joint_speed
                    acc:  0
                    trq:  0

      target    ← handler value, OR fallback to:
                    pos:  joint_position_commanded
                    vel:  joint_speed_commanded
                    acc:  joint_acceleration_commanded
                    trq:  joint_torque_commanded

    No None / NaN ever lands in shared memory.
    """
    n = ctx.joint_num
    j = ctx.shared_data.joint

    # _commanded (post-clamp) — fallback to measured / 0
    j['joint_position_commanded'][:n] = (
        output.pos_rad if output.pos_rad is not None else j['joint_position'][:n]
    )
    j['joint_speed_commanded'][:n] = (
        # Fallback uses FILTERED speed: raw joint_speed carries encoder-dither noise, and this
        # feeds joint_speed_commanded -> TCP_speed_commanded. Before the 2026-07-17 raw/filtered
        # split joint_speed was itself filtered, so this preserves that.
        output.vel_rad if output.vel_rad is not None else j['joint_speed_filtered'][:n]
    )
    j['joint_acceleration_commanded'][:n] = (
        output.acc_rad if output.acc_rad is not None else 0.0
    )
    j['joint_torque_commanded'][:n] = (
        output.trq_nm if output.trq_nm is not None else 0.0
    )

    # _target (upstream intent) — fallback to the _commanded values we just wrote
    j['joint_position_target'][:n] = (
        output.target_pos_rad if output.target_pos_rad is not None
        else j['joint_position_commanded'][:n]
    )
    j['joint_speed_target'][:n] = (
        output.target_vel_rad if output.target_vel_rad is not None
        else j['joint_speed_commanded'][:n]
    )
    j['joint_acceleration_target'][:n] = (
        output.target_acc_rad if output.target_acc_rad is not None
        else j['joint_acceleration_commanded'][:n]
    )
    j['joint_torque_target'][:n] = (
        output.target_trq_nm if output.target_trq_nm is not None
        else j['joint_torque_commanded'][:n]
    )


def _commit_tcp_target(ctx, output):
    """Write TCP-space _target shared memory from ControlOutput.

    Must run AFTER _fk_commanded so the fallback reads the live TCP commanded
    values (TCP_pose_commanded, TCP_pose_commanded_matrix, TCP_speed_commanded).

    Fallback rule per docs/commanded_target_actual.md §5.2:
      TCP_pose_target        ← handler value OR fallback to TCP_pose_commanded
      TCP_pose_target_matrix ← handler value OR fallback to TCP_pose_commanded_matrix
      TCP_speed_target       ← handler value OR fallback to TCP_speed_commanded

    In joint-space modes (everything we have today) the handler leaves all
    TCP fields None and we just mirror commanded. TCP-space modes (future)
    will provide both the 6-vec and the 16-vec — humans use one, math the other.
    """
    tcp = ctx.shared_data.tcp

    if output.target_TCP_pose is not None:
        tcp['TCP_pose_target'][:] = output.target_TCP_pose
    else:
        tcp['TCP_pose_target'][:] = tcp['TCP_pose_commanded']

    if output.target_TCP_pose_matrix is not None:
        tcp['TCP_pose_target_matrix'][:] = output.target_TCP_pose_matrix
    else:
        tcp['TCP_pose_target_matrix'][:] = tcp['TCP_pose_commanded_matrix']

    if output.target_TCP_speed is not None:
        tcp['TCP_speed_target'][:] = output.target_TCP_speed
    else:
        tcp['TCP_speed_target'][:] = tcp['TCP_speed_commanded']


def _to_motor_space(ctx, output):
    """Convert joint-space ControlOutput to motor-space buffers.

    Returns (pos_buf, vel_buf, curr_buf) — each is the pre-allocated module
    buffer if the corresponding ControlOutput field is set, or None if not.
    None propagates through to send_motor_commands which interprets it as
    "leave this channel unset" (Position=None / Speed=None / Current=0).

    Allocation-free: all writes go through np.multiply(..., out=) or per-joint
    scalar assignments into module-level buffers.
    """
    n = ctx.joint_num

    if output.pos_rad is not None:
        for i in range(n):
            _motor_pos_buf[i] = ctx.Joint[i].get_encoder_position(output.pos_rad[i])
        pos_out = _motor_pos_buf
    else:
        pos_out = None

    if output.vel_rad is not None:
        for i in range(n):
            _motor_vel_buf[i] = ctx.Joint[i].get_encoder_speed(output.vel_rad[i])
        vel_out = _motor_vel_buf
    else:
        vel_out = None

    if output.trq_nm is not None:
        np.multiply(output.trq_nm[:n], _torque_to_mA_factor[:n], out=_motor_curr_buf[:n])
        curr_out = _motor_curr_buf
    else:
        curr_out = None

    return pos_out, vel_out, curr_out


def _record_sent(ctx, pos_buf, vel_buf, curr_buf, mode):
    """Mirror what is about to go on the wire into motor_*_sent (motor space).

    Applies the SAME int() truncation the CAN senders do, so these are the literal values
    transmitted — not a reader's re-derivation, which could silently drift from the real
    conversion. NaN marks a channel data_pack_1 leaves unset (genuinely different from a
    commanded 0); data_pack_PD always sends all three.
    """
    n = ctx.joint_num
    m = ctx.shared_data.motor
    for i in range(n):
        m['motor_position_sent'][i] = float(int(pos_buf[i])) if pos_buf is not None else float('nan')
        m['motor_speed_sent'][i] = float(int(vel_buf[i])) if vel_buf is not None else float('nan')
        # currents=None means the senders substitute 0, so record the 0 that is actually sent.
        m['motor_current_sent'][i] = float(int(curr_buf[i])) if curr_buf is not None else 0.0
    ctx.shared_data.rti['motor_pack_mode_sent'][0] = 1.0 if mode == "pd" else 0.0


def _send(ctx, pos_buf, vel_buf, curr_buf, mode):
    """Dispatch the CAN send based on ControlOutput.mode."""
    _record_sent(ctx, pos_buf, vel_buf, curr_buf, mode)
    # hall_check_joint: the joint in the Hall sensor check (robotics/hall_check.py) gets its Hall-index command instead
    if mode == "pd":
        send_motor_commands_pd(
            ctx.Motor, ctx.joint_num,
            positions=pos_buf, speeds=vel_buf, currents=curr_buf, hall_joint=ctx.hall_check_joint,
        )
    else:
        send_motor_commands(
            ctx.Motor, ctx.joint_num,
            positions=pos_buf, speeds=vel_buf, currents=curr_buf, hall_joint=ctx.hall_check_joint,
        )


def _fk_commanded(ctx):
    """Populate TCP_pose_commanded / _matrix / _speed from joint_*_commanded.

    Per docs/commanded_target_actual.md §5.2:
      TCP_pose_commanded         = FK(joint_position_commanded)
      TCP_pose_commanded_matrix  = same FK, as flattened 4x4
      TCP_speed_commanded        = J(q_commanded) · joint_speed_commanded

    Single source of truth: commanded TCP is always derived from
    joint_position_commanded, never written directly by handlers. With the
    pinocchio backend this costs ~30 µs per tick on the Pi — small enough that
    keeping the canonical invariant is the right trade.

    Two TCP-pose representations are populated from the same FK pass:
      - TCP_pose_commanded         (6-vec [x, y, z, rx, ry, rz]) — for display
      - TCP_pose_commanded_matrix  (16-vec flattened 4x4)         — for math
    See docs/variable_reference.md §2.6 on which to use when.
    """
    n = ctx.joint_num
    j = ctx.shared_data.joint
    tcp = ctx.shared_data.tcp

    q_c  = j['joint_position_commanded'][:n]
    dq_c = j['joint_speed_commanded'][:n]

    if should_use_pinocchio(ctx.robot):
        pose_4x4, J = ctx.robot.pinocchio_fkine_and_jacob0(q_c)
        tcp['TCP_pose_commanded'][:3]       = pose_4x4[:3, 3]
        write_rpy_xyz(pose_4x4, tcp['TCP_pose_commanded'], start=3)
        tcp['TCP_pose_commanded_matrix'][:] = pose_4x4.reshape(16)
    else:
        T = ctx.robot.fkine(q_c)
        tcp['TCP_pose_commanded'][:3]       = T.t
        tcp['TCP_pose_commanded'][3:6]      = T.rpy(order='xyz')
        tcp['TCP_pose_commanded_matrix'][:] = T.A.flatten()
        J = ctx.robot.jacob0(q=q_c, T=T)

    tcp['TCP_speed_commanded'][:] = J @ dq_c


# ---------------------------------------------------------------------------
# Stateless function handlers — pure setpoint computers.
# Each returns (TickResult, ControlOutput).
# ---------------------------------------------------------------------------

def handle_booting(ctx):
    """BOOTING: velocity-hold at 0 rad/s, no torque feedforward."""
    return _tick_ok, _output_vel_zero


def handle_idle(ctx: TickContext) -> tuple[TickResult, ControlOutput]:
    """IDLE: gravity-comp torque hold if homed/enabled/grav-on — velocity-hold otherwise."""
    robot_homed     = bool(np.all(ctx.shared_data.rti['homing_status'][:ctx.joint_num] == 2))
    robot_enabled   = ctx.shared_data.robot_state.get() == "ENABLED"
    gravity_comp_on = bool(ctx.shared_data.rti['gravity_comp_enabled'][0])

    if robot_homed and robot_enabled and gravity_comp_on:
        gravity = ctx.shared_data.joint['joint_gravity_torque'][:ctx.joint_num]
        # Freedrive: the safety zones push back as haptic walls (utility/haptic_walls.py). None = no wall
        # touched this tick, so the arm is exactly as weightless as it always was.
        walls = ctx.haptic_walls
        trq = walls.step(ctx.current_tick, gravity) if walls is not None else None
        base = gravity if trq is None else trq
        # Friction compensation (utility/friction_comp.py): the operator switch, off at every start. Added into
        # our own buffer — `gravity` is a view of shared memory and must never be written.
        fc = getattr(ctx, "friction_comp", None)
        if fc is not None and ctx.shared_data.rti['friction_comp_enabled'][0] > 0.5:
            n = ctx.joint_num
            comp = fc.step(ctx.shared_data.joint['joint_speed_filtered'])
            np.add(base[:n], comp[:n], out=_idle_trq_buf[:n])
            ctx.shared_data.joint['joint_friction_comp_torque'][:n] = comp[:n]
            _output_idle_gravity.trq_nm = _idle_trq_buf
        else:
            _output_idle_gravity.trq_nm = base
        return _tick_ok, _output_idle_gravity
    return _tick_ok, _output_vel_zero


def handle_active_error(ctx):
    """ACTIVE_ERROR: velocity-hold at 0 rad/s until errors clear."""
    return _tick_ok, _output_vel_zero


def handle_safety_stop(ctx):
    """SAFETY_STOP: zero torque to all motors (limp)."""
    return _tick_ok, _output_torque_zero


def handle_flashing(ctx):
    """FLASHING: transmit NOTHING — an external flasher owns the bus.

    Returning SELF_MANAGED makes dispatch skip the commit/convert/send pipeline,
    exactly as HOMING does, except this mode sends nothing at all rather than
    doing its own sends. `gripper_owned=True` suppresses RTI's gripper send block
    for the same reason: during a firmware flash the ONLY thing allowed to put
    frames on can0 is the flasher process.

    The loop keeps ticking throughout, so E-stop, IO, shared memory and the GUI
    stay live — that is the entire point of doing this as a mode instead of
    stopping the runtime. The motors lose torque here (their watchdogs fire into
    shorted-phase brake), which is why entry is gated on the arm being parked.
    """
    return _tick_flashing, SELF_MANAGED


# ---------------------------------------------------------------------------
# Homing dispatch wrapper — adapts HomingController to the handler interface.
# HOMING owns its own per-joint multi-phase CAN sends inside robotics/homing.py,
# so step() returns SELF_MANAGED and dispatch skips the commit/convert/send
# pipeline.
# ---------------------------------------------------------------------------

class _HomingDispatch:

    def __init__(self, homing_ctrl, load_motor_configs_fn, motor_config):
        self._homing_ctrl = homing_ctrl
        self._load_motor_configs = load_motor_configs_fn
        self._motor_config = motor_config
        self._step_ilim = np.array(motor_config['Ilim'], dtype=float)

    def enter(self, ctx, **kwargs):
        self._homing_ctrl.enter(
            ctx.Motor, ctx.joint_num,
            self._load_motor_configs,
            motor_config=self._motor_config,
            shared_data=ctx.shared_data,
        )
        write_motor_config_to_shared(
            ctx.shared_data, ctx.joint_num,
            KPP=self._motor_config['KPP'],
            KPV=self._motor_config['KPV'],
            KIV=self._motor_config['KIV'],
            KPIQ=self._motor_config['KPIQ'],
            KIIQ=self._motor_config['KIIQ'],
            KP=self._motor_config['KP'],
            KD=self._motor_config['KD'],
            Ilim=self._homing_ctrl.homing_current_ma,
            velocity_limit=self._motor_config['velocity_limit'],
        )

    def step(self, ctx):
        result = self._homing_ctrl.step(ctx.Motor, ctx.Joint, ctx.shared_data)

        mc = self._motor_config
        homing_status = ctx.shared_data.rti['homing_status']
        for j in range(ctx.joint_num):
            self._step_ilim[j] = (
                mc['Ilim'][j] if homing_status[j] >= 2
                else self._homing_ctrl.homing_current_ma[j]
            )
        write_motor_config_to_shared(
            ctx.shared_data, ctx.joint_num,
            KPP=mc['KPP'], KPV=mc['KPV'], KIV=mc['KIV'],
            KPIQ=mc['KPIQ'], KIIQ=mc['KIIQ'],
            KP=mc['KP'], KD=mc['KD'],
            Ilim=self._step_ilim,
            velocity_limit=mc['velocity_limit'],
        )

        pending = None
        if result == 'done':
            pending = RobotMode.IDLE
        elif result == 'failed':
            ctx.shared_data.homed_state.set("NOT HOMED")
            ctx.shared_data.rti["homing_status"][:] = 0
            # A reported failure latches as a hard error in this tick's error pass (2026-10-14), which ends HOMING
            # itself (ACTIVE_ERROR, disabled); asking for IDLE as well would flip the mode to IDLE for a tick.
            pending = None if self._homing_ctrl.has_failures else RobotMode.IDLE
        return TickResult(
            pending_mode=pending,
            gripper_owned=self._homing_ctrl.gripper_homing_active,
        ), SELF_MANAGED

    def exit(self, ctx):
        self._homing_ctrl.exit(ctx.Motor, ctx.joint_num)
        write_motor_config_to_shared(
            ctx.shared_data, ctx.joint_num,
            KPP=self._motor_config['KPP'],
            KPV=self._motor_config['KPV'],
            KIV=self._motor_config['KIV'],
            KPIQ=self._motor_config['KPIQ'],
            KIIQ=self._motor_config['KIIQ'],
            KP=self._motor_config['KP'],
            KD=self._motor_config['KD'],
            Ilim=self._motor_config['Ilim'],
            velocity_limit=self._motor_config['velocity_limit'],
        )


# ---------------------------------------------------------------------------
# Dispatch — single per-tick entry point.
# ---------------------------------------------------------------------------

# Inertia + Coriolis feedforward (2026-10-10, TODO/dynamics_feedforward_plan.md). The sum is written into
# dispatch's OWN buffer — never into the handler's torque buffer, which a handler may hand back next tick
# without recomputing it. One buffer pair per joint count, allocated on the first call.
_dyn_bufs: dict = {}


def _dyn_buffers(n):
    """(sum, zeros) for n joints — allocated ONCE per joint count (build_mode_dispatch does it before the loop; a
    caller that skips the factory gets them on its first call), then only looked up."""
    bufs = _dyn_bufs.get(n)
    if bufs is None:
        bufs = _dyn_bufs[n] = (np.zeros(n, dtype=np.float64), np.zeros(n, dtype=np.float64))
    return bufs


def _add_dynamics_ff(ctx, output):
    """Add M(q)·qdd (inertia) and C(q, qd)·qd (Coriolis) to the tick's commanded torque, each behind its own switch
    (rti inertia_comp_enabled / coriolis_comp_enabled), and publish both in joint_inertia_torque /
    joint_coriolis_torque (Nm; dispatch_step clears them every tick).

    From the COMMANDED motion only (user decision 2026-10-10): the handler's acc_rad declares that it has one —
    EXEC (the planned velocity's change per tick), JOG (its profile). Without a commanded acceleration (freedrive,
    holds, error / stop outputs) nothing is added. NOT in RTI / SOFT_RTI (user decision 2026-10-11): their output
    carries the generator's acceleration with acc_feedforward=False — the planner's own state, switching between its
    limits as it chases each new target (simulated on PAR6: ±1.8 Nm on J1 at 7-100 Hz in SOFT_RTI, 0.9 Nm RMS in hard
    RTI, where the motion's own inertia torque was 0.06 Nm); a PC that wants dynamics sends torque. q = the commanded
    position, or the measured one where the mode commands none (velocity commands: M and C only need the
    configuration); qd = the commanded velocity (no Coriolis without one). Gravity stays the handlers' own, at the
    measured pose. A non-finite result adds nothing."""
    if output.acc_rad is None or not output.acc_feedforward:
        return
    rti = ctx.shared_data.rti
    use_i = rti['inertia_comp_enabled'][0] > 0.5
    use_c = rti['coriolis_comp_enabled'][0] > 0.5 and output.vel_rad is not None
    if not (use_i or use_c):
        return
    fn = getattr(ctx.robot, 'pinocchio_dynamics_ff', None)
    if fn is None:
        return
    n = ctx.joint_num
    total, zeros = _dyn_buffers(n)
    j = ctx.shared_data.joint
    q = output.pos_rad if output.pos_rad is not None else j['joint_position'][:n]
    v = output.vel_rad if output.vel_rad is not None else zeros
    tau_i, tau_c = fn(q, v, output.acc_rad, use_i, use_c)
    check = float(tau_i.sum()) + float(tau_c.sum())
    if check != check or check in (float('inf'), float('-inf')):
        return
    if output.trq_nm is None:
        total.fill(0.0)
    else:
        np.copyto(total, output.trq_nm)
    np.add(total, tau_i, out=total)
    np.add(total, tau_c, out=total)
    j['joint_inertia_torque'][:n] = tau_i
    j['joint_coriolis_torque'][:n] = tau_c
    output.trq_nm = total


def dispatch_step(handler: Any, ctx: TickContext) -> TickResult:
    """Run one tick of the active mode handler.

    Pipeline:
      1. handler.step(ctx) → (TickResult, ControlOutput)
      2. If output is SELF_MANAGED (HOMING): return result, skip pipeline.
      3. Otherwise: commit joint state, convert to motor space, send on CAN.

    Falls back to handle_idle if handler is None.
    """
    if handler is None:
        handler = handle_idle
    step_fn = handler.step if hasattr(handler, 'step') else handler
    # The friction compensation applied this tick: 0 unless freedrive (handle_idle) sets it — every mode, so a
    # value from the last freedrive tick never lingers.
    ctx.shared_data.joint['joint_friction_comp_torque'].fill(0.0)
    ctx.shared_data.joint['joint_inertia_torque'].fill(0.0)
    ctx.shared_data.joint['joint_coriolis_torque'].fill(0.0)

    if not _MODE_PROFILE_ENABLED:
        result, output = step_fn(ctx)

        if output is SELF_MANAGED:
            # Homing owns its own CAN sends, so the dispatch pipeline sent nothing this tick.
            # Mark it rather than leaving the previous tick's values looking current.
            ctx.shared_data.rti['motor_pack_mode_sent'][0] = float('nan')
            return result

        _add_dynamics_ff(ctx, output)
        _commit_joint_state(ctx, output)
        _fk_commanded(ctx)
        _commit_tcp_target(ctx, output)
        pos_buf, vel_buf, curr_buf = _to_motor_space(ctx, output)
        _send(ctx, pos_buf, vel_buf, curr_buf, output.mode)
        return result

    samples = []
    t0 = time.perf_counter_ns()
    result, output = step_fn(ctx)
    t1 = time.perf_counter_ns()
    samples.append(("handler", t1 - t0))

    if output is SELF_MANAGED:
        # As above: nothing went through the dispatch pipeline this tick.
        ctx.shared_data.rti['motor_pack_mode_sent'][0] = float('nan')
        _mode_profile_record(samples)
        return result

    _add_dynamics_ff(ctx, output)
    t1b = time.perf_counter_ns()
    samples.append(("dynamics_ff", t1b - t1))
    _commit_joint_state(ctx, output)
    t2 = time.perf_counter_ns()
    samples.append(("commit_joint", t2 - t1b))

    _fk_commanded(ctx)
    t3 = time.perf_counter_ns()
    samples.append(("fk_commanded", t3 - t2))

    _commit_tcp_target(ctx, output)
    t4 = time.perf_counter_ns()
    samples.append(("commit_tcp", t4 - t3))

    pos_buf, vel_buf, curr_buf = _to_motor_space(ctx, output)
    t5 = time.perf_counter_ns()
    samples.append(("to_motor", t5 - t4))

    _send(ctx, pos_buf, vel_buf, curr_buf, output.mode)
    t6 = time.perf_counter_ns()
    samples.append(("send", t6 - t5))

    _mode_profile_record(samples)
    return result


# ---------------------------------------------------------------------------
# Factory — call once before the main loop.
# ---------------------------------------------------------------------------

def build_mode_dispatch(joint_num: int, homing_ctrl: Any, load_motor_configs_fn: Callable,
                        motor_config: dict, jog_config: dict, motor_dynamics: Any,
                        shared_data: Any = None, exec_limits: dict | None = None) -> dict:
    """Initialise pre-allocated buffers and return the MODE_HANDLERS dict.

    Must be called before the RT loop starts.

    Args:
        joint_num:             Number of arm joints.
        homing_ctrl:           HomingController instance.
        load_motor_configs_fn: load_motor_configs callable.
        motor_config:          Dict with motor PID/limit settings.
        jog_config:            Dict with jog-handler init params. Required keys:
                               max_vel_rad, delta_t, limit_neg, limit_pos.
                               Optional keys (XML defaults from system.xml):
                               accel_time, accel_limit, profile, jerk_factor, control_mode,
                               speed_pct.
        motor_dynamics:        Dict with gear_ratio, gear_efficiency, motor_kt,
                               motor_signs — used by _to_motor_space() to
                               convert joint Nm → motor mA in one in-place
                               multiplication.
        shared_data:           Optional reference to the data.shared_data module.
                               Passed into JogHandler so it can mirror its config
                               to rti[] for GUI readback. Optional for backward
                               compat; if omitted, JOG works but new tabs will
                               see zeroed jog state until the user touches a
                               slider.
        exec_limits:           Required. The limits of EXEC's controlled pause / stop
                               (motion/exec_handler.py): stop_accel / stop_jerk = the
                               robot XML's hardware ceilings (rad/s^2, rad/s^3),
                               resume_accel = <exec_limits> acceleration.

    Returns:
        dict mapping RobotMode → handler (plain function or class instance).
    """
    _dyn_buffers(joint_num)                        # the inertia / Coriolis feedforward's buffers, before the loop
    global _zeros_rad, _motor_pos_buf, _motor_vel_buf, _motor_curr_buf, _torque_to_mA_factor
    global _output_vel_zero, _output_torque_zero, _idle_trq_buf

    _zeros_rad      = np.zeros(joint_num, dtype=np.float64)
    _idle_trq_buf   = np.zeros(joint_num, dtype=np.float64)
    _motor_pos_buf  = np.zeros(joint_num, dtype=np.float64)
    _motor_vel_buf  = np.zeros(joint_num, dtype=np.float64)
    _motor_curr_buf = np.zeros(joint_num, dtype=np.float64)
    _output_vel_zero    = ControlOutput(vel_rad=_zeros_rad)
    _output_torque_zero = ControlOutput(trq_nm=_zeros_rad)

    # Pre-compute the per-joint factor that turns joint torque (Nm) into motor
    # current (mA) in one multiply. Shared formula with the per-tick shared-
    # memory update in data/update_shared_data.py — both go through
    # robotics.dynamics.build_torque_to_ma_factor() so the formula has a single
    # source of truth.
    _torque_to_mA_factor = build_torque_to_ma_factor(
        gear_ratio      = motor_dynamics['gear_ratio'],
        gear_efficiency = motor_dynamics['gear_efficiency'],
        motor_kt        = motor_dynamics['motor_kt'],
        motor_signs     = motor_dynamics['motor_signs'],
    )

    _homing_dispatch = _HomingDispatch(homing_ctrl, load_motor_configs_fn, motor_config)
    _jog_handler     = JogHandler(
        joint_num,
        jog_config['max_vel_rad'],
        jog_config['delta_t'],
        limit_neg=jog_config['limit_neg'],
        limit_pos=jog_config['limit_pos'],
        accel_time   = jog_config.get('accel_time',   0.55),
        accel_limit  = jog_config.get('accel_limit'),
        profile      = jog_config.get('profile',      'scurve'),
        jerk_factor  = jog_config.get('jerk_factor',  3.0),
        control_mode = jog_config.get('control_mode', 'pid'),
        speed_pct    = jog_config.get('speed_pct',    0.20),
        shared_data  = shared_data,
        self_collision = bool(jog_config.get('self_collision', False)),
    )
    if exec_limits is None:
        raise ValueError("build_mode_dispatch: exec_limits (stop_accel, stop_jerk, resume_accel) is required — "
                         "EXEC's controlled pause / stop brakes at the robot XML's ceilings")
    _exec_handler    = ExecHandler(joint_num, shared_data=shared_data, **exec_limits)

    return {
        RobotMode.BOOTING:       handle_booting,
        RobotMode.IDLE:          handle_idle,
        RobotMode.ACTIVE_ERROR:  handle_active_error,
        RobotMode.HOMING:        _homing_dispatch,
        RobotMode.JOG:           _jog_handler,
        RobotMode.EXEC:          _exec_handler,      # Phase 3
        RobotMode.RTI:           handle_idle,        # Phase 4
        # SOFT_RTI is replaced by the same RTIModeHandler instance as RTI in RTI.py, for
        # the same reason RTI is: both contracts share one handler so no safety layer can
        # be applied to one and forgotten in the other.
        RobotMode.SOFT_RTI:      handle_idle,
        RobotMode.HAND_GUIDING:  handle_idle,        # Future
        RobotMode.IMPEDANCE:     handle_idle,        # Future
        RobotMode.SAFETY_STOP:   handle_safety_stop,
        RobotMode.FLASHING:      handle_flashing,
    }
