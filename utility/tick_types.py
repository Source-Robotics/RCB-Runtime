"""TickContext, TickResult, and ControlOutput — shared data structures for mode handlers.

Kept in a separate module to avoid circular imports between mode_dispatch.py
and individual handler modules (jog.py, exec_handler.py, rti_handler.py, etc.)
that need these types but are also imported by mode_dispatch.py.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class TickContext:
    """Read-only references passed to every handler each tick.

    Built once before the main loop. Only rti_udp_data and current_tick
    are updated per tick (attribute assignments, no allocation).
    All other fields point to existing objects — no copies.
    """
    # Hardware
    Motor: list
    Joint: list
    joint_num: int
    active_CAN_gripper: int

    # State
    shared_data: object
    robot: object
    delta_t: float

    # Communication (RTI mode only)
    rti_udp_data: object        # bytes | None — latest UDP packet this tick
    socket_sender: object       # UDP send socket for RTI state packets

    # Recipes / ring-buffers (RTI mode only)
    recipe: object              # active data recipe (for build_payload)
    Joint_data: object          # joint deque ring buffers
    Gripper_data: object        # gripper deque ring buffers

    # Current tick (updated by RTI.py each tick)
    current_tick: int

    # Per-node motor deque dicts, bundled for build_payload()'s 4th argument
    # ({"data", "temperature", "voltage", "errors", "config"} → per-node deques).
    # Only RTI mode needs it — it is the one handler that serialises state itself.
    # Defaulted so existing TickContext(...) construction stays valid.
    motor_buffers: object = None

    # How many UDP packets drain_rti_udp() pulled off the socket this tick, accepted or
    # not. Only the NEWEST can be applied, so anything above 1 was discarded — and that is
    # invisible in every other metric: success rate still reads 100%, misses still read 0.
    rti_udp_count: int = 0

    # Haptic zone walls (utility/haptic_walls.py) for IDLE gravity-comp freedrive. None = no walls.
    haptic_walls: object = None


@dataclass
class TickResult:
    """Value returned by every handler step()."""
    pending_mode: object = None   # RobotMode | None — request a mode transition
    gripper_owned: bool = False   # True = this handler sent gripper CAN commands


@dataclass
class ControlOutput:
    """Joint-space setpoint produced by a handler's step().

    Every handler returns (TickResult, ControlOutput). Dispatch translates
    ControlOutput into shared-memory writes (_commanded and _target fields)
    plus CAN sends.

    Three families of fields:

    1. Commanded / post-clamp (pos_rad, vel_rad, acc_rad, trq_nm)
       What gets converted to motor space and sent on CAN this tick.
       None means "not driving this channel": pos_rad=None / vel_rad=None →
       Position=None / Speed=None on CAN; trq_nm=None → Current=0.

    2. Target / raw upstream intent (target_pos_rad, target_vel_rad,
       target_acc_rad, target_trq_nm)
       What the upstream layer asked for, before clamps / ramps / safety.
       None falls back to the matching commanded value via the chain in
       docs/commanded_target_actual.md §5.1. In modes where the handler
       doesn't separate target from commanded (e.g. JOG today), set both
       to the same buffer.

    3. TCP-space target (target_TCP_pose, target_TCP_speed, target_TCP_pose_matrix)
       Only set in TCP-space modes (EXEC linear / cartesian RTI / TCP jog —
       all future). In joint-space modes these are None and fall back to
       TCP_*_commanded via §5.2. If target_TCP_pose_matrix is None it falls
       back to TCP_pose_commanded_matrix; future TCP-space handlers should
       set both forms to keep the 6-vec / matrix pair consistent.

    All numeric fields are joint space (rad, rad/s, rad/s^2, Nm) except the TCP target
    pair (TCP pose in m / rad, twist in m/s + rad/s; matrix is row-major 4x4
    flattened). Motor-space conversion happens in _to_motor_space().

    Shared-memory _target and _commanded fields are filled every tick via
    fallback — no None / NaN ever lands in shared memory.

    mode:
      "pid" → send_motor_commands       (Send_data_pack_1)
      "pd"  → send_motor_commands_pd    (Send_data_pack_PD — requires pos and vel)
      "self_managed" → reserved for SELF_MANAGED sentinel
    """
    # Commanded / post-clamp — what goes on CAN this tick
    pos_rad: Optional[np.ndarray] = None
    vel_rad: Optional[np.ndarray] = None
    acc_rad: Optional[np.ndarray] = None
    trq_nm:  Optional[np.ndarray] = None
    mode:    str = "pid"

    # Target / raw upstream intent — None falls back to matching commanded
    target_pos_rad: Optional[np.ndarray] = None
    target_vel_rad: Optional[np.ndarray] = None
    target_acc_rad: Optional[np.ndarray] = None
    target_trq_nm:  Optional[np.ndarray] = None

    # TCP-space target — only set in TCP-space modes; None falls back to TCP commanded
    target_TCP_pose:        Optional[np.ndarray] = None   # 6-vec [x, y, z, rx, ry, rz]
    target_TCP_speed:       Optional[np.ndarray] = None   # 6-vec twist [v, ω]
    target_TCP_pose_matrix: Optional[np.ndarray] = None   # 16-vec flattened 4x4 SE(3)

    # Whether acc_rad may become inertia / Coriolis FEEDFORWARD torque (utility/mode_dispatch._add_dynamics_ff).
    # False: acc_rad is still published (joint_acceleration_commanded, telemetry j_ac) but nothing is fed forward.
    # The RTI handler sets it False (2026-10-11): its generator's acceleration is the planner's own state, switching
    # between its limits as it chases each new target — fed forward it was up to ±1.8 Nm of 7-100 Hz torque on J1.
    acc_feedforward: bool = True


# Sentinel returned by handlers that own their own multi-phase CAN sends
# (HOMING). Dispatch sees `output is SELF_MANAGED` and skips the commit /
# convert / send pipeline for that tick.
SELF_MANAGED = ControlOutput(mode="self_managed")
