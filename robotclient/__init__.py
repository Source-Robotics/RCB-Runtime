"""robotclient — Python client library for the PAR6 robot (EXEC mode + RTI streaming).

    from robotclient import RobotClient, RTIClient
    from robotclient.rti_client import JointPositionCommand, MotionFinished

Talks newline-JSON over TCP to command_executor (EXEC), UDP to RTI.
"""

from .client import RobotClient, GripperResult
# rti_client is standalone by design (stdlib only, no robot-side imports) so it is
# safe to pull in eagerly here.
from .rti_client import (
    RTIClient, JointPositionCommand, JointVelocityCommand, JointTorqueCommand,
    MotionFinished, RobotState, ControlLost,
)
from .exceptions import (
    RobotError, RobotConnectionError, RobotTimeout, CommandRejected, MotionFailed,
    GripFailed,
)
from .exceptions import HomingFailed as HomingFailed  # "as": an explicit re-export (2026-10-14)
# This client's version and the version check at connect (2026-10-21, versioning V3; robotclient/version.py).
from .exceptions import VersionMismatch, VersionWarning
from .version import __version__

# dry_run is the ONE module here that is not PC-portable: it imports the planner, the
# kinematics and pinocchio, i.e. the whole robot-side stack. Importing it eagerly made
# `from robotclient import RTIClient` require roboticstoolbox and pinocchio on the operator's
# machine — which quietly broke the entire point of keeping client.py and rti_client.py to
# stdlib + relative imports (found 2026-08-07 preparing the Stage 2 network test).
#
# PEP 562 module __getattr__: dry_run is loaded on FIRST USE instead. On the robot nothing
# changes; on a bare PC `RobotClient` and `RTIClient` now import with no third-party deps at
# all, and only touching DryRunRobotClient raises — with its real ImportError, naming what is
# missing.
_LAZY = {"DryRunPlan": "dry_run", "DryRunRobotClient": "dry_run"}


def __getattr__(name):
    if name in _LAZY:
        import importlib
        mod = importlib.import_module(f".{_LAZY[name]}", __name__)
        value = getattr(mod, name)
        globals()[name] = value          # cache, so the indirection costs one lookup ever
        return value
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
    return sorted(list(globals()) + list(_LAZY))

__all__ = [
    "RobotClient",
    "RTIClient", "JointPositionCommand", "JointVelocityCommand", "JointTorqueCommand",
    "MotionFinished", "RobotState", "ControlLost",
    "DryRunRobotClient", "DryRunPlan",
    "RobotError", "RobotConnectionError", "RobotTimeout", "CommandRejected",
    "MotionFailed",
    "VersionMismatch", "VersionWarning", "__version__",
]
