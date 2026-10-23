"""This client's version and the protocol versions it speaks (2026-10-21, versioning V3).

The client ships with the runtime, so its version is the runtime's (the repository's VERSION file). It is written here,
not read from that file, because the client also runs on a PC that has only this package. The protocol numbers are this
client's copies of the robot's (communication/protocol.py PROTOCOL_VERSION, communication/rti_protocol.py
RTI_PROTOCOL_VERSION, utility/serialization.py TELEMETRY_FORMAT); dummy_test_scripts/test_protocol_versions.py fails when
any of them, or __version__, differs from the robot side.

A protocol number changes only on an incompatible change, so it alone decides compatibility: equal numbers work, whatever
the two versions are (a command newer than the robot is refused by the robot as UNKNOWN_CMD, which already says why).

Renamed commands (2026-10-23): a command's wire ID is its method's name, and 25 of them changed in the release named by
COMMANDS_RENAMED_IN. The robot accepts the old IDs for ever (communication/protocol.COMMAND_ALIASES), so an old client
works with a new robot. For a NEW client against an OLDER robot the client does the same the other way round: once it
knows the robot's runtime_version (at connect) it sends the IDs that robot knows, from WIRE_NAMES_BEFORE_RENAME. So the
protocol number stays, and a program runs on either robot; dummy_test_scripts/test_command_aliases.py keeps the table
equal to the robot's alias table. A command that did not exist at all on the older robot is still UNKNOWN_CMD there.
"""
from __future__ import annotations

import warnings

from .exceptions import VersionMismatch, VersionWarning

__version__ = "0.10.0"

PROTOCOL_VERSION = 1        # the command protocol (newline JSON over TCP to the executor)
RTI_PROTOCOL_VERSION = 1    # the RTI command packet
TELEMETRY_FORMAT = 1        # the state / telemetry packets ("tag:v,v,...")

# What a client checks at connect, by the robot's status key "protocols".
SPOKEN = {"commands": PROTOCOL_VERSION, "rti": RTI_PROTOCOL_VERSION, "telemetry": TELEMETRY_FORMAT}

# The first runtime whose executor knows the renamed command IDs; a robot older than it gets the IDs below
# (every ID the 0.9.0 release accepts). The two aliases set_default_profile / set_default_ik_solver are not here:
# that release accepted set_profile / set_ik_solver already.
COMMANDS_RENAMED_IN = "0.10.0"
WIRE_NAMES_BEFORE_RENAME = {
    "move_linear_rel_world": "move_linear_rel_wrf", "move_arc_rel_world": "move_arc_rel_wrf",
    "move_circle_rel_world": "move_circle_rel_wrf", "move_spline_rel_world": "move_spline_rel_wrf",
    "move_linear_rel_tool": "move_linear_rel_trf", "move_arc_rel_tool": "move_arc_rel_trf",
    "move_circle_rel_tool": "move_circle_rel_trf", "move_spline_rel_tool": "move_spline_rel_trf",
    "open_gripper": "gripper_open", "close_gripper": "gripper_close", "calibrate_gripper": "gripper_calibrate",
    "jog": "jog_set", "set_jog_accel": "jog_accel", "set_jog_profile": "jog_profile",
    "set_jog_jerk_factor": "jog_jerk_factor", "set_jog_control_mode": "jog_control_mode",
    "set_gravity_comp": "gravity_comp", "set_friction_comp": "friction_comp", "set_inertia_comp": "inertia_comp",
    "set_coriolis_comp": "coriolis_comp", "set_friction_ff": "friction_ff", "set_can_read_first": "can_read_first",
    "set_collision_detection": "collision_detection", "set_gripper_ctrl_mode": "gripper_ctrl_mode",
    "set_gripper": "gripper_set",
}


def version_tuple(text) -> tuple[int, ...] | None:
    """"0.9.0" -> (0, 9, 0); "0.10.0rc1" -> (0, 10, 0); None or anything else unparsable -> None."""
    if not isinstance(text, str):
        return None
    parts = []
    for piece in text.strip().split("."):
        digits = ""
        for ch in piece:
            if not ch.isdigit():
                break
            digits += ch
        if not digits:
            break
        parts.append(int(digits))
    return tuple(parts) if parts else None


def wire_names_for(runtime_version) -> dict[str, str]:
    """The command IDs to send to a robot that reports `runtime_version`: the pre-rename ones when it is older than
    COMMANDS_RENAMED_IN, reports none (a runtime from before 0.9.0) or reports something unparsable; else none."""
    theirs = version_tuple(runtime_version)
    since = version_tuple(COMMANDS_RENAMED_IN)
    if theirs is None or since is None or theirs < since:
        return dict(WIRE_NAMES_BEFORE_RENAME)
    return {}


def check_protocols(status: dict | None, which: tuple[str, ...], allow_mismatch: bool = False) -> bool:
    """Compare the robot's protocol versions (its status message's "protocols") with this client's, for the protocols
    in `which`. Returns True when the robot reported them all.

    - equal: nothing happens.
    - different: VersionMismatch (names each protocol, both numbers and both versions), unless allow_mismatch.
    - the robot reports none (its runtime predates versioning, before 0.9.0): a VersionWarning, unless allow_mismatch —
      it works as it did, but this client may use commands that robot does not have.
    """
    status = status or {}
    theirs = status.get("protocols")
    if not isinstance(theirs, dict) or not all(k in theirs for k in which):
        if not allow_mismatch:
            warnings.warn(f"the robot reports no protocol versions: its runtime predates versioning (before 0.9.0); "
                          f"this robotclient is {__version__}. It may use commands that robot does not have (refused as "
                          f"UNKNOWN_CMD). Update the robot's runtime, or connect with allow_mismatch=True to silence this.",
                          VersionWarning, stacklevel=3)
        return False
    wrong = [f"{k}: robot {theirs[k]}, this client {SPOKEN[k]}" for k in which if theirs[k] != SPOKEN[k]]
    if wrong and not allow_mismatch:
        raise VersionMismatch(
            f"incompatible protocol ({'; '.join(wrong)}). The robot runs runtime {status.get('runtime_version', '?')}, "
            f"this robotclient is {__version__}. Use the robotclient that came with the robot's runtime (its "
            f"repository's robotclient/ folder), or connect with allow_mismatch=True at your own risk.")
    return True
