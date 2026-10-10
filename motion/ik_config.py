"""Single source of truth for IK tuning across the whole project.

Every place that solves IK — the seeded path solver (move_linear / zone chains),
move_pose's single-target solve, and any future RTI streaming lane — reads its
margins, retry limits, and thresholds from one IKConfig so they can't drift apart
(a scattered slimit=100 here / slimit=1 there is exactly the bug this removes).

This holds the TUNING, shared by both IK engines. Which engine runs is a per-solve
choice (SeededIKPathSolver's `solver` arg, from a command's ik_solver option):
  - "pinocchio"        -> DLS on Pinocchio's Jacobian (uses dls_* fields below)
  - "roboticstoolbox"  -> ik_LM (uses slimit/ilimit/lm_*), DLS fallback (dls_*)
Some fields therefore apply to only one engine (slimit/ilimit -> rtb; dls_* -> both).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class IKConfig:
    # --- roboticstoolbox ik_LM primary (only used by the "roboticstoolbox" engine) ---
    slimit: int = 1                 # ik_LM search attempts. 1 = seeded only. >1 reseeds from
                                    # RANDOM configs, which flips to other kinematic branches
                                    # (same TCP pose, so the FK check accepts it) and makes the
                                    # solver non-deterministic — measured 4/8 pi-flips at
                                    # slimit=100. Keep at 1 unless you have a reason and have
                                    # re-measured.
    ilimit: int = 100              # ik_LM iterations per attempt
    lm_method: str = "chan"
    lm_k: float = 0.1
    tol: float = 1e-5

    # --- continuity / validation thresholds ---
    # Reject a solution that jumped to a different kinematic branch between two path
    # samples. Both branches reach the same TCP pose, so the FK-residual check can
    # never catch a flip — this is the only defense. Measured over 225 paths: worst
    # legitimate per-sample step 17.7 deg, smallest genuine flip 180.2 deg (clean 10x
    # gap). 20 deg sits in the gap: 0 false rejects, 0 missed flips. Was 2 deg, which
    # false-rejected 37% of solvable paths (the step scales with sampling density).
    branch_jump_threshold_rad: float = np.deg2rad(20.0)
    # Jump from the arm's ACTUAL position to the first IK sample — a physical lurch,
    # not a branch flip, so it stays tight.
    start_jump_threshold_rad: float = np.deg2rad(2.0)
    pose_translation_tolerance_m: float = 2e-4
    pose_rotation_tolerance_rad: float = np.deg2rad(0.5)
    joint_margin_rad: float = 0.0

    # --- damped-least-squares fallback (seeded, deterministic) ---
    dls_max_step_rad: float = 0.02  # per-iteration joint step cap. Independent of the branch
                                    # threshold: a rejection criterion and a solver step size
                                    # are unrelated concerns.
    dls_iterations: int = 80
    dls_damping: float = 1e-3

    # --- singularity WARNING thresholds (emit IK_SINGULARITY_WARNING; do NOT reject) ---
    singularity_condition_warn: float = 1000.0
    singularity_sigma_warn: float = 1e-4

    def is_singularity_warning(self, condition, sigma_min):
        """Shared warn-threshold predicate — every warning site must call this rather
        than repeating the threshold literals (they drifted into three copies already;
        see TODO/singularity.md §3.2). Warning-only: never rejects a path. None inputs
        (metric unavailable) simply don't trigger."""
        return ((condition is not None and condition > self.singularity_condition_warn)
                or (sigma_min is not None and sigma_min < self.singularity_sigma_warn))

    def status_dict(self):
        """JSON-friendly view of the shared IK TUNING for get_status (angles in degrees).
        The active engine + FK/Jacobian backend are reported separately (planner status),
        since those are per-command, not part of this global tuning."""
        return {
            "lm_method": self.lm_method,          # ik_LM only (roboticstoolbox engine)
            "slimit": int(self.slimit),           # ik_LM only
            "ilimit": int(self.ilimit),           # ik_LM only
            "ik_tol": float(self.tol),
            "branch_jump_threshold_deg": round(float(np.rad2deg(self.branch_jump_threshold_rad)), 3),
            "start_jump_threshold_deg": round(float(np.rad2deg(self.start_jump_threshold_rad)), 3),
            "pose_translation_tolerance_m": float(self.pose_translation_tolerance_m),
            "pose_rotation_tolerance_deg": round(float(np.rad2deg(self.pose_rotation_tolerance_rad)), 3),
            "joint_margin_rad": float(self.joint_margin_rad),
            "dls_max_step_rad": float(self.dls_max_step_rad),   # both engines (pinocchio primary / rtb fallback)
            "dls_iterations": int(self.dls_iterations),
            "dls_damping": float(self.dls_damping),
            "singularity_condition_warn": float(self.singularity_condition_warn),
            "singularity_sigma_warn": float(self.singularity_sigma_warn),
        }


DEFAULT_IK_CONFIG = IKConfig()

# The Jacobian condition number is UNBOUNDED at an exact singularity (sigma_min -> 0),
# and we compute it as float("inf") there. That cannot go on the wire: JSON has no
# Infinity literal — Python's json emits a non-standard `Infinity` token that strict
# parsers (e.g. a JS/web GUI) reject outright. Reporting `null` instead would be worse:
# it is indistinguishable from "metric unavailable", so the single most important sample
# in a singular path would read as missing data.
#
# So on the wire: NaN (genuinely unavailable) -> null, +inf (exactly singular) -> this
# finite sentinel. It sits far above every threshold, so all warn/reject logic still
# fires. sigma_min remains the exact, bounded measure of how singular a pose is — the
# condition number is the scale-dependent one anyway (see TODO/singularity.md §2).
SINGULAR_CONDITION_REPORT_CAP = 1e12


def json_safe_metric(value: float | None, is_condition: bool = False) -> float | None:
    """Make one singularity metric safe to put in the status JSON.

      NaN  (metric unavailable)   -> None
      +inf (exactly singular)     -> SINGULAR_CONDITION_REPORT_CAP, for the condition
                                     number only (see the constant above)
      finite                      -> the value
    """
    if value is None:
        return None
    v = float(value)
    if np.isnan(v):
        return None
    if not np.isfinite(v):
        return float(SINGULAR_CONDITION_REPORT_CAP) if is_condition else None
    return v

# FK/Jacobian backend for the per-sample validation, singularity metrics, and the DLS
# math — INDEPENDENT of which IK engine is chosen. Pinocchio's C++ kinematics are
# ~100x faster than roboticstoolbox's pure-Python fkine/jacob0, and both give the same
# answer, so we default to pinocchio regardless of engine (falling back to rtb only if
# the pinocchio helpers aren't attached). Decoupling this from the engine is what makes
# the roboticstoolbox engine ~5x faster — its ik_LM solve was always fast; the slow part
# was doing all the check-up FK/Jacobian on rtb's Python backend.
DEFAULT_FK_JACOBIAN_BACKEND = "pinocchio"
