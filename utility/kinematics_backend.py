"""Shared FK/Jacobian backend selection helpers."""

import math

import numpy as np


_kinematics_backend = "auto"  # auto | pinocchio | rtb
_RPY_SINGULAR_TOL = 20.0 * np.finfo(float).eps


def set_kinematics_backend(backend):
    """Select the FK/Jacobian backend used by commanded and measured TCP paths.

    Valid values:
      - "auto":      use Pinocchio when attached, otherwise RTB fallback
      - "pinocchio": require the attached Pinocchio helpers
      - "rtb":       force robotics-toolbox fkine/jacob0
    """
    global _kinematics_backend

    normalized = str(backend).strip().lower()
    if normalized == "roboticstoolbox":
        normalized = "rtb"
    if normalized not in ("auto", "pinocchio", "rtb"):
        raise ValueError(
            f"Invalid kinematics backend {backend!r}; expected auto, pinocchio, or rtb"
        )
    _kinematics_backend = normalized


def get_kinematics_backend():
    return _kinematics_backend


def should_use_pinocchio(robot):
    """Return True when the selected backend should use attached Pinocchio helpers."""
    return (
        _kinematics_backend != "rtb"
        and hasattr(robot, "pinocchio_fkine_and_jacob0")
    )


def require_selected_backend_available(robot):
    """Raise if the selected backend is forced to Pinocchio but unavailable."""
    if _kinematics_backend == "pinocchio" and not should_use_pinocchio(robot):
        raise RuntimeError(
            "Kinematics backend is set to pinocchio, but this robot has no "
            "pinocchio_fkine_and_jacob0 helper attached"
        )


def write_quaternion_wxyz(R, out, start=0):
    """Write the rotation in R into out as a unit quaternion, SCALAR-FIRST (w, x, y, z).

    R may be a 3x3 rotation matrix or a full 4x4 homogeneous transform — only the
    top-left 3x3 block is read either way.

    ⚠️ COMPONENT ORDER IS (w, x, y, z) — scalar FIRST. This matches spatialmath,
    roboticstoolbox, Eigen/Pinocchio and MATLAB, i.e. every library already in this
    stack, so nothing here needs converting. ROS/tf2, SciPy and Bullet use the OPPOSITE
    order (x, y, z, w); feeding one to the other silently produces a plausible-looking
    WRONG rotation rather than an error. Consumers that need the ROS layout reorder it
    themselves — q[[1, 2, 3, 0]] — no conversion belongs here.

    Writes in place into `out` (no allocation) — this runs every RT tick.

    Uses Shepperd's method: pick the branch off the largest diagonal term so the square
    root is never taken of something near zero. The naive single-formula version loses
    precision near 180-degree rotations, which is a pose this arm can actually reach.

    Sign: q and -q encode the SAME rotation (the double cover). The branches would
    otherwise hand back either one depending on which diagonal term happened to win,
    so the result is canonicalized to w >= 0 to keep this channel deterministic —
    without it a consumer differencing tick-to-tick would see a full sign flip look
    like a violent jump while the arm barely moved. Note this only MOVES the
    discontinuity rather than removing it: it now sits at w = 0, i.e. a 180-degree
    rotation. Consumers doing SLERP or orientation error must handle the double cover
    themselves regardless.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        out[start]     = 0.25 * s
        out[start + 1] = (R[2, 1] - R[1, 2]) / s
        out[start + 2] = (R[0, 2] - R[2, 0]) / s
        out[start + 3] = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        out[start]     = (R[2, 1] - R[1, 2]) / s
        out[start + 1] = 0.25 * s
        out[start + 2] = (R[0, 1] + R[1, 0]) / s
        out[start + 3] = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        out[start]     = (R[0, 2] - R[2, 0]) / s
        out[start + 1] = (R[0, 1] + R[1, 0]) / s
        out[start + 2] = 0.25 * s
        out[start + 3] = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        out[start]     = (R[1, 0] - R[0, 1]) / s
        out[start + 1] = (R[0, 2] + R[2, 0]) / s
        out[start + 2] = (R[1, 2] + R[2, 1]) / s
        out[start + 3] = 0.25 * s

    if out[start] < 0.0:
        out[start]     = -out[start]
        out[start + 1] = -out[start + 1]
        out[start + 2] = -out[start + 2]
        out[start + 3] = -out[start + 3]


def write_rpy_xyz(R, out, start=0):
    """Write SpatialMath-compatible tr2rpy(..., order='xyz') into out."""
    if abs(abs(R[0, 2]) - 1.0) < _RPY_SINGULAR_TOL:
        out[start] = 0.0
        if R[0, 2] > 0.0:
            out[start + 2] = math.atan2(R[2, 1], R[1, 1])
        else:
            out[start + 2] = -math.atan2(R[1, 0], R[2, 0])
        out[start + 1] = math.asin(max(-1.0, min(1.0, R[0, 2])))
        return

    out[start] = -math.atan2(R[0, 1], R[0, 0])
    out[start + 2] = -math.atan2(R[1, 2], R[2, 2])

    a0 = abs(R[0, 0])
    a1 = abs(R[0, 1])
    a2 = abs(R[1, 2])
    a3 = abs(R[2, 2])
    if a0 >= a1 and a0 >= a2 and a0 >= a3:
        out[start + 1] = math.atan(R[0, 2] * math.cos(out[start]) / R[0, 0])
    elif a1 >= a2 and a1 >= a3:
        out[start + 1] = -math.atan(R[0, 2] * math.sin(out[start]) / R[0, 1])
    elif a2 >= a3:
        out[start + 1] = -math.atan(R[0, 2] * math.sin(out[start + 2]) / R[1, 2])
    else:
        out[start + 1] = math.atan(R[0, 2] * math.cos(out[start + 2]) / R[2, 2])
