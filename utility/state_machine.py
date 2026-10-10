"""
Robot state machine — pure logic, no hardware access.

Four shared-memory variables govern the robot's state:
  robot_mode   — what the robot is actively doing (IDLE, HOMING, JOG, RTI, ...)
  robot_state  — are motors energised? (ENABLED / DISABLED) — set independently
                 by enable/disable commands, not derived from mode
  homed_state  — HOMED / NOT HOMED, written only by the homing routine
  error_state  — JSON list of active faults, written by error_checks.py

Enable/disable is handled separately from mode changes:
  - "enable"  command → robot_state = ENABLED  (blocked while errors are active)
  - "disable" command → robot_state = DISABLED, robot_mode forced to IDLE
  - Any error → robot_state = DISABLED, robot_mode forced to ACTIVE_ERROR (velocity-hold)
  - Errors cleared → robot_mode returns to IDLE automatically

Mode changes require robot_state == ENABLED (except IDLE which is always allowed).
"""

from __future__ import annotations

from enum import Enum


class RobotMode(str, Enum):
    BOOTING      = "BOOTING"
    IDLE         = "IDLE"
    ACTIVE_ERROR = "ACTIVE_ERROR"
    HOMING       = "HOMING"
    JOG          = "JOG"
    RTI          = "RTI"
    # Soft real-time streaming, the soft half of the hard / soft real-time split of industrial interfaces.
    # RTI: the PC closes the loop, one setpoint per 4 ms tick, hard real-time, UDP.
    # SOFT_RTI: the PC sends targets at ITS own rate (5-50 Hz) and the robot interpolates.
    #
    # A separate mode rather than a flag on RTI because the difference is not settings:
    # the transport can differ (UDP or TCP), and the link diagnostics mean different things
    # -- control_success_rate is packet loss when the PC sends once per tick and is
    # meaningless at 20 Hz, where a perfectly healthy link scores 8%. An unnamed contract
    # makes every metric need a mental asterisk.
    #
    # Both modes dispatch to the SAME RTIModeHandler, so every safety layer (soft joint
    # limits, brake-at-limits, the TCP velocity cap, gravity comp, start-pose validation)
    # is shared by construction and cannot be forgotten in one of them.
    SOFT_RTI     = "SOFT_RTI"
    EXEC         = "EXEC" # AUTO?
    HAND_GUIDING = "HAND_GUIDING"
    IMPEDANCE    = "IMPEDANCE"
    SAFETY_STOP  = "SAFETY_STOP"
    FLASHING     = "FLASHING"   # maintenance: RTI goes silent so a flasher owns the bus


# Modes that require the robot to be homed before entering
REQUIRES_HOMED = frozenset({
    RobotMode.JOG,
    RobotMode.RTI,
    RobotMode.SOFT_RTI,
    RobotMode.EXEC,
    RobotMode.HAND_GUIDING,
    RobotMode.IMPEDANCE,
})

# Maintenance modes — no motion, no motor commands, RTI transmits nothing.
#
# These deliberately bypass the enabled / errors-cleared / homed gates that
# every operational mode must pass. That is not a loophole, it is the point:
# you enter FLASHING precisely to service a robot that may be disabled, faulted,
# or missing a CAN node (a driver stuck in its bootloader after a failed flash
# raises a node-disconnect error, so requiring "no errors" would make the one
# situation you most need the flasher for unreachable).
#
# What they are NOT exempt from is `park_ok` — see request_mode_change(). Motors
# go limp in these modes, so the arm must be resting in a pose that survives
# losing torque.
MAINTENANCE_MODES = frozenset({
    RobotMode.FLASHING,
})

# Legal mode transitions — all operational modes require robot_state == ENABLED
VALID_TRANSITIONS = {
    RobotMode.BOOTING:      [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    RobotMode.IDLE:         [RobotMode.HOMING,  RobotMode.JOG,
                             RobotMode.RTI,     RobotMode.SOFT_RTI,
                             RobotMode.EXEC,
                             RobotMode.HAND_GUIDING, RobotMode.IMPEDANCE,
                             RobotMode.SAFETY_STOP, RobotMode.FLASHING],
    # ACTIVE_ERROR → FLASHING is the RECOVERY path: a driver left in its
    # bootloader by a failed flash shows up as a disconnected node, i.e. as an
    # active error. Without this edge the fix would be unreachable.
    # SAFETY_STOP belongs here for the same reason FLASHING does: a faulted
    # robot is precisely when you need to cut motor authority, and
    # ACTIVE_ERROR still holds velocity-0 — it is not a quiet state.
    RobotMode.ACTIVE_ERROR: [RobotMode.IDLE,    RobotMode.FLASHING,
                             RobotMode.SAFETY_STOP],
    RobotMode.HOMING:       [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    RobotMode.JOG:          [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    RobotMode.RTI:          [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    # Deliberately NOT RTI <-> SOFT_RTI directly. They are different contracts with
    # different timeouts and different transports; swapping between them under a live
    # claim would leave the PC streaming into a link whose rules changed underneath it.
    # Via IDLE means the claim is dropped and re-made, which is the honest sequence.
    RobotMode.SOFT_RTI:     [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    RobotMode.EXEC:         [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    RobotMode.HAND_GUIDING: [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    RobotMode.IMPEDANCE:    [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
    RobotMode.SAFETY_STOP:  [RobotMode.IDLE],
    RobotMode.FLASHING:     [RobotMode.IDLE,    RobotMode.SAFETY_STOP],
}


def request_enable(errors_active: bool, safety_active: bool) -> tuple[bool, str]:
    """
    Validate an enable request.

    Returns:
        (success: bool, reason: str)
    """
    if safety_active:
        return False, "Cannot enable: safety violation active"
    if errors_active:
        return False, "Cannot enable: active errors must be cleared first"
    return True, "OK"


def request_mode_change(current_mode: RobotMode | str, requested_mode: RobotMode | str, robot_enabled: bool, homed: bool,
                        errors_active: bool, park_ok: bool = True) -> tuple[bool, RobotMode, str]:
    """
    Validate and approve a mode transition.

    Returns:
        (success: bool, final_mode: RobotMode, reason: str)

    Pure function — does NOT touch shared_data.
    The caller is responsible for writing robot_mode.

    Args:
        current_mode:   current RobotMode (string or enum value)
        requested_mode: desired RobotMode (string or enum value)
        robot_enabled:  True if robot_state == "ENABLED"
        homed:          True if homed_state == "HOMED"
        errors_active:  True if error_active flag is set
        park_ok:        True if a HUMAN has asserted the arm can survive losing
                        torque. Only consulted for MAINTENANCE_MODES, where the
                        motors go limp; every other mode ignores it.

                        Deliberately an assertion, not a measurement. It was
                        once derived from joint_gravity_torque against a
                        threshold, which is the wrong question twice over:
                        gravity torque says how hard the motors are working, not
                        whether the arm is resting on something, and PAR6's own
                        standby pose measured 1.80 Nm against a 1.0 Nm limit —
                        so the arm's designated resting pose failed the gate.
                        The caller now supplies an explicit operator claim
                        (`set_mode:FLASHING:PARKED` or `:FORCE`).
    """
    try:
        current_mode   = RobotMode(current_mode)
        requested_mode = RobotMode(requested_mode)
    except ValueError as e:
        return False, RobotMode(current_mode) if isinstance(current_mode, RobotMode) else RobotMode.IDLE, f"Unknown mode: {e}"

    # Transition to IDLE is always allowed (disable / error path)
    if requested_mode != RobotMode.IDLE:
        allowed = VALID_TRANSITIONS.get(current_mode, [])
        if requested_mode not in allowed:
            return False, current_mode, f"Transition {current_mode} → {requested_mode} not allowed"

        # Maintenance modes command no motion and must stay reachable on a
        # disabled or faulted robot — that is what they are for. They answer to
        # park_ok instead. See MAINTENANCE_MODES above.
        if requested_mode in MAINTENANCE_MODES:
            if not park_ok:
                return False, current_mode, (
                    f"Cannot enter {requested_mode}: motors go limp in this mode, "
                    "and nobody has confirmed the arm can hold itself up.\n"
                    "Move the arm to its park pose and confirm, or override "
                    "explicitly if it is physically supported."
                )
        elif requested_mode == RobotMode.SAFETY_STOP:
            # Always reachable, from any state. SAFETY_STOP commands zero current
            # with position and speed unset — the motors are driven by nothing at
            # all, which is the SAFEST state the robot has. Gating it behind
            # "robot is enabled" would mean the one command guaranteed to stop
            # the machine acting could itself be refused. Nothing to check.
            pass
        else:
            # All operational modes require the robot to be enabled
            if not robot_enabled:
                return False, current_mode, f"Cannot enter {requested_mode}: robot is not enabled"

            # Cannot enter operational modes while errors are active
            if errors_active:
                return False, current_mode, f"Cannot enter {requested_mode}: active errors must be cleared first"

            # Operational modes (except HOMING) require homing
            if requested_mode in REQUIRES_HOMED and not homed:
                return False, current_mode, f"Cannot enter {requested_mode}: robot not homed"

    return True, requested_mode, "OK"
