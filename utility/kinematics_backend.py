"""Shared FK/Jacobian backend selection helpers."""

from __future__ import annotations

from typing import Any

import math

import numpy as np


_kinematics_backend = "auto"  # auto | pinocchio | rtb
_RPY_SINGULAR_TOL = 20.0 * np.finfo(float).eps


def set_kinematics_backend(backend: str) -> None:
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


def should_use_pinocchio(robot: Any) -> bool:
    """Return True when the selected backend should use attached Pinocchio helpers."""
    return (
        _kinematics_backend != "rtb"
        and hasattr(robot, "pinocchio_fkine_and_jacob0")
    )


def require_selected_backend_available(robot: Any) -> None:
    """Raise if the selected backend is forced to Pinocchio but unavailable."""
    if _kinematics_backend == "pinocchio" and not should_use_pinocchio(robot):
        raise RuntimeError(
            "Kinematics backend is set to pinocchio, but this robot has no "
            "pinocchio_fkine_and_jacob0 helper attached"
        )


def write_quaternion_wxyz(R: np.ndarray, out: np.ndarray, start: int = 0) -> None:
    """Write the rotation in R into out as a unit quaternion, SCALAR-FIRST (w, x, y, z).

    R may be a 3x3 rotation matrix or a full 4x4 homogeneous transform — only the
    top-left 3x3 block is read either way.

    ⚠️ COMPONENT ORDER IS (w, x, y, z) — scalar FIRST. This matches spatialmath,
    roboticstoolbox, Eigen/Pinocchio and MATLAB, i.e. every library already in this
    stack, so nothing here needs converting. many other libraries use the OPPOSITE
    order (x, y, z, w); feeding one to the other silently produces a plausible-looking
    WRONG rotation rather than an error. Consumers that need the (x, y, z, w) layout reorder it
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


# ─── The pose angles (2026-10-21, user) ────────────────────────────────────────────────────────────────────────────
# A TCP pose 6-vector is [x, y, z, rx, ry, rz]: metres, then three angles in radians with
#     R = Rx(rx) · Ry(ry) · Rz(rz)
# — turn about the base X axis by rx, then about the tool's NEW Y axis by ry, then about the tool's own Z axis by rz
# (mobile X, Y', Z''; the same rotation as turning about the fixed base axes Z by rz, then Y by ry, then X by rx).
# rz is always the spin about the tool's own axis, at any tilt. This is how a number of industrial arms define their
# rx, ry, rz (or alpha, beta, gamma). Gimbal lock at ry = ±90°: the tool's Z axis along base ±X — rx and rz then turn
# about the same axis; the readout reports rz = 0 and puts the turn in rx. Chosen over fixed X, Y, Z after measuring
# PAR6 (2026-10-21, user): around the poses the arm rests in after homing it never comes within 10° of lock (fixed
# X, Y, Z: 4-7 % of those poses; over the whole joint range the two are equal).
# ONE definition, two directions: angles -> rotation is motion.cartesian_paths.pose_from_xyz_rpy, rotation -> angles
# is write_pose_angles / pose_angles below; everything else works on the matrix. Before 2026-10-21 the code stored
# SpatialMath's order='xyz' angles, R = Rx(rz) Ry(ry) Rz(rx): the same convention with rx and rz swapped —
# blockly/programs.py swaps programs saved before (format 1).
POSE_CONVENTION = "xyz-mobile"     # get_status()["pose_angles"]; a block program refuses an executor without it


def write_pose_angles(R: np.ndarray, out: np.ndarray, start: int = 0) -> None:
    """Write the pose angles [rx, ry, rz] of rotation R (3x3, or the top-left of a 4x4) into out[start:start + 3]:
    R = Rx(rx) Ry(ry) Rz(rz). Exactly SpatialMath's tr2rpy(R, order='xyz') with its three values in reverse order —
    the same branches, the same pick of the best-conditioned element for ry. In place, no allocation: this runs every
    RT tick (the measured and the commanded pose)."""
    if abs(abs(R[0, 2]) - 1.0) < _RPY_SINGULAR_TOL:
        # gimbal lock (ry = ±90°): rx and rz turn about the same axis — rz = 0, rx carries the turn
        rz = 0.0
        if R[0, 2] > 0.0:
            rx = math.atan2(R[2, 1], R[1, 1])
        else:
            rx = -math.atan2(R[1, 0], R[2, 0])
        ry = math.asin(max(-1.0, min(1.0, R[0, 2])))
    else:
        rz = -math.atan2(R[0, 1], R[0, 0])
        rx = -math.atan2(R[1, 2], R[2, 2])
        a0 = abs(R[0, 0])
        a1 = abs(R[0, 1])
        a2 = abs(R[1, 2])
        a3 = abs(R[2, 2])
        if a0 >= a1 and a0 >= a2 and a0 >= a3:
            ry = math.atan(R[0, 2] * math.cos(rz) / R[0, 0])
        elif a1 >= a2 and a1 >= a3:
            ry = -math.atan(R[0, 2] * math.sin(rz) / R[0, 1])
        elif a2 >= a3:
            ry = -math.atan(R[0, 2] * math.sin(rx) / R[1, 2])
        else:
            ry = math.atan(R[0, 2] * math.cos(rx) / R[2, 2])
    out[start] = rx
    out[start + 1] = ry
    out[start + 2] = rz


def pose_angles(R: np.ndarray) -> np.ndarray:
    """The pose angles [rx, ry, rz] of rotation R as a new array — write_pose_angles for code off the RT path."""
    out = np.empty(3, dtype=np.float64)
    write_pose_angles(R, out)
    return out
