"""This client's version and the protocol versions it speaks (2026-10-21, versioning V3).

The client ships with the runtime, so its version is the runtime's (the repository's VERSION file). It is written here,
not read from that file, because the client also runs on a PC that has only this package. The protocol numbers are this
client's copies of the robot's (communication/protocol.py PROTOCOL_VERSION, communication/rti_protocol.py
RTI_PROTOCOL_VERSION, utility/serialization.py TELEMETRY_FORMAT); dummy_test_scripts/test_protocol_versions.py fails when
any of them, or __version__, differs from the robot side.

A protocol number changes only on an incompatible change, so it alone decides compatibility: equal numbers work, whatever
the two versions are (a command newer than the robot is refused by the robot as UNKNOWN_CMD, which already says why).
"""
from __future__ import annotations

import warnings

from .exceptions import VersionMismatch, VersionWarning

__version__ = "0.9.0"

PROTOCOL_VERSION = 1        # the command protocol (newline JSON over TCP to the executor)
RTI_PROTOCOL_VERSION = 1    # the RTI command packet
TELEMETRY_FORMAT = 1        # the state / telemetry packets ("tag:v,v,...")

# What a client checks at connect, by the robot's status key "protocols".
SPOKEN = {"commands": PROTOCOL_VERSION, "rti": RTI_PROTOCOL_VERSION, "telemetry": TELEMETRY_FORMAT}


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
