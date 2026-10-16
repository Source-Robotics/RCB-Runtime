"""Seeded IK-chain solving and diagnostics for Cartesian EXEC planning."""

import logging
from dataclasses import dataclass

import numpy as np

from motion.exec_commands import normalize_ik_solver_name
from motion.ik_config import (
    DEFAULT_IK_CONFIG, DEFAULT_FK_JACOBIAN_BACKEND, json_safe_metric,
)
from robotics.robot_kinematics import (
    calculate_forward_kinematics,
    calculate_inverse_kinematics,
    calculate_jacobian,
)
from utility.kinematics_backend import set_kinematics_backend

logger = logging.getLogger("kin")  # docs/logging.md "Logger names"


@dataclass(frozen=True)
class IKPathResult:
    success: bool
    positions: np.ndarray
    valid_mask: np.ndarray
    first_failure_index: object = None
    failure_code: object = None
    failure_message: object = None
    branch_jump_indices: tuple = ()
    min_manipulability: object = None
    max_jacobian_condition: object = None
    min_jacobian_sigma: object = None
    # Per-sample singularity profile (index-aligned with `positions`; NaN where a
    # metric couldn't be computed). worst_sample_index = most-singular sample
    # (smallest sigma). See TODO/singularity.md Phase 1.
    condition_profile: tuple = ()
    sigma_profile: tuple = ()
    manipulability_profile: tuple = ()
    worst_sample_index: object = None
    start_delta_max: object = None
    start_delta_norm: object = None
    start_jump_threshold: object = None
    max_translation_error: object = None
    max_rotation_error: object = None
    pose_translation_tolerance: object = None
    pose_rotation_tolerance: object = None

    def diagnostic(self):
        return {
            "success": bool(self.success),
            "num_samples": int(self.valid_mask.shape[0]),
            "valid_count": int(np.count_nonzero(self.valid_mask)),
            "valid_mask": [bool(v) for v in self.valid_mask.tolist()],
            "first_failure_index": self.first_failure_index,
            "failure_code": self.failure_code,
            "failure_message": self.failure_message,
            "branch_jump_indices": [int(i) for i in self.branch_jump_indices],
            "min_manipulability": _json_metric(self.min_manipulability),
            # Condition is +inf at an exact singularity -> reported as the finite sentinel
            # (JSON has no Infinity; see ik_config.SINGULAR_CONDITION_REPORT_CAP).
            "max_jacobian_condition": _json_metric(self.max_jacobian_condition, is_condition=True),
            "min_jacobian_sigma": _json_metric(self.min_jacobian_sigma),
            # Per-sample profile + worst point.
            "condition_profile": _json_metric_list(self.condition_profile, is_condition=True),
            "sigma_profile": _json_metric_list(self.sigma_profile),
            "manipulability_profile": _json_metric_list(self.manipulability_profile),
            "worst_sample_index": None if self.worst_sample_index is None else int(self.worst_sample_index),
            "worst_sample_condition": _profile_at(self.condition_profile, self.worst_sample_index, is_condition=True),
            "worst_sample_sigma": _profile_at(self.sigma_profile, self.worst_sample_index),
            "worst_sample_manipulability": _profile_at(self.manipulability_profile, self.worst_sample_index),
            "worst_sample_joints_deg": self._worst_sample_joints_deg(),
            "start_delta_max_rad": None if self.start_delta_max is None else float(self.start_delta_max),
            "start_delta_norm_rad": None if self.start_delta_norm is None else float(self.start_delta_norm),
            "start_jump_threshold_rad": None if self.start_jump_threshold is None else float(self.start_jump_threshold),
            "max_translation_error_m": None if self.max_translation_error is None else float(self.max_translation_error),
            "max_rotation_error_rad": None if self.max_rotation_error is None else float(self.max_rotation_error),
            "pose_translation_tolerance_m": None if self.pose_translation_tolerance is None else float(self.pose_translation_tolerance),
            "pose_rotation_tolerance_rad": None if self.pose_rotation_tolerance is None else float(self.pose_rotation_tolerance),
        }

    def _worst_sample_joints_deg(self):
        """Joint angles (deg) at the most-singular sample, for the diagnostic."""
        i = self.worst_sample_index
        if (i is None or self.positions is None
                or i < 0 or i >= self.positions.shape[0]):
            return None
        return np.rad2deg(self.positions[i]).tolist()


class SeededIKPathSolver:
    """Solve a sampled Cartesian path with previous-solution seeding.

    Two independent choices:
      - `solver` = the IK ENGINE (which algorithm finds the joint angles):
          "pinocchio"        -> DLS on Pinocchio's Jacobian (default; deterministic)
          "roboticstoolbox"  -> ik_LM primary + DLS fallback
      - `kinematics_backend` = which library computes the per-sample FK/Jacobian used
        for validation, singularity metrics, and the DLS math. Defaults to fast
        Pinocchio C++ REGARDLESS of engine (falls back to rtb if pinocchio isn't
        attached). These were coupled before, which needlessly made the
        roboticstoolbox engine slow — its solve was fast, the check-ups were not.

    All tuning (retry limits, thresholds, tolerances, margins, DLS params) comes from a
    single IKConfig — see motion/ik_config.py.
    """

    def __init__(self, robot, joint_num, solver="pinocchio", config=None,
                 kinematics_backend=None):
        self.robot = robot
        self.joint_num = int(joint_num)
        self.solver = normalize_ik_solver_name(solver or "pinocchio")
        self.kinematics_backend = kinematics_backend or DEFAULT_FK_JACOBIAN_BACKEND
        self.config = config or DEFAULT_IK_CONFIG
        c = self.config
        self.joint_margin = float(c.joint_margin_rad)
        self.branch_jump_threshold = float(c.branch_jump_threshold_rad)
        self.start_jump_threshold = float(c.start_jump_threshold_rad)
        self.ilimit = int(c.ilimit)
        self.slimit = int(c.slimit)
        self.tol = float(c.tol)
        self.pose_translation_tolerance = float(c.pose_translation_tolerance_m)
        self.pose_rotation_tolerance = float(c.pose_rotation_tolerance_rad)

    def _select_backend(self):
        if self.robot is None:
            raise NotImplementedError("Cartesian IK needs a robot model")
        # FK/Jacobian backend is independent of the IK engine. Prefer pinocchio; fall
        # back to rtb if its helpers aren't attached (every engine works on either).
        backend = self.kinematics_backend
        if backend == "pinocchio" and not hasattr(self.robot, "pinocchio_fkine_and_jacob0"):
            backend = "rtb"
        set_kinematics_backend(backend)

    def _limits(self):
        qlim = np.asarray(getattr(self.robot, "qlim", []), dtype=np.float64)
        if qlim.shape == (2, self.joint_num):
            low, high = qlim
        elif qlim.shape == (self.joint_num, 2):
            low, high = qlim[:, 0], qlim[:, 1]
        else:
            return None, None
        return low + self.joint_margin, high - self.joint_margin

    def _check_limits(self, q, low, high):
        if low is None:
            return True
        return bool(np.all(q >= low - 1e-9) and np.all(q <= high + 1e-9))

    def _singularity_metrics(self, q):
        jacobian = calculate_jacobian(self.robot, q)
        singular_values = np.linalg.svd(jacobian, compute_uv=False)
        if singular_values.size == 0:
            return None, None, None
        sigma_min = float(np.min(singular_values))
        sigma_max = float(np.max(singular_values))
        condition = float("inf") if sigma_min <= 1e-12 else sigma_max / sigma_min
        manipulability = float(np.prod(singular_values))
        return manipulability, condition, sigma_min

    def _pose_error(self, q, target_pose):
        actual = calculate_forward_kinematics(self.robot, q)
        translation_error = float(np.linalg.norm(actual.t - target_pose.t))
        rel = actual.A[:3, :3].T @ target_pose.A[:3, :3]
        c = np.clip((float(np.trace(rel)) - 1.0) * 0.5, -1.0, 1.0)
        rotation_error = float(np.arccos(c))
        return translation_error, rotation_error

    def _rotation_error_vector(self, actual_R, target_R):
        rel = target_R @ actual_R.T
        c = np.clip((float(np.trace(rel)) - 1.0) * 0.5, -1.0, 1.0)
        angle = float(np.arccos(c))
        skew = np.array([
            rel[2, 1] - rel[1, 2],
            rel[0, 2] - rel[2, 0],
            rel[1, 0] - rel[0, 1],
        ], dtype=np.float64)
        if angle < 1e-9:
            return 0.5 * skew
        s = float(np.sin(angle))
        if abs(s) < 1e-9:
            return np.zeros(3, dtype=np.float64)
        return (angle / (2.0 * s)) * skew

    def _cartesian_error_vector(self, q, target_pose):
        actual = calculate_forward_kinematics(self.robot, q)
        translation_error = np.asarray(target_pose.t - actual.t, dtype=np.float64)
        rotation_error = self._rotation_error_vector(actual.A[:3, :3], target_pose.A[:3, :3])
        return np.concatenate((translation_error, rotation_error))

    def _solve_dls_sample(self, target_pose, q0, low, high):
        q = np.asarray(q0, dtype=np.float64)[:self.joint_num].copy()
        damping = self.config.dls_damping
        max_step = self.config.dls_max_step_rad

        for _ in range(self.config.dls_iterations):
            err = self._cartesian_error_vector(q, target_pose)
            if (np.linalg.norm(err[:3]) <= self.pose_translation_tolerance * 0.25
                    and np.linalg.norm(err[3:]) <= self.pose_rotation_tolerance * 0.25):
                return q

            jacobian = np.asarray(calculate_jacobian(self.robot, q), dtype=np.float64)[:6, :self.joint_num]
            if jacobian.shape != (6, self.joint_num) or not np.all(np.isfinite(jacobian)):
                return None

            lhs = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
            try:
                dq = jacobian.T @ np.linalg.solve(lhs, err)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(jacobian) @ err

            if dq.shape[0] != self.joint_num or not np.all(np.isfinite(dq)):
                return None
            peak = float(np.max(np.abs(dq))) if dq.size else 0.0
            if peak > max_step:
                dq *= max_step / peak
            if peak < 1e-12:
                return q

            q = q + dq
            if low is not None:
                q = np.minimum(np.maximum(q, low), high)

        return q

    def _solve_sample_ik(self, target_pose, q_seed, low, high):
        # Engine dispatch:
        #   "roboticstoolbox" -> ik_LM primary; on a tolerance miss, DLS fallback.
        #   "pinocchio"       -> DLS on Pinocchio's Jacobian directly (no ik_LM). The
        #                        DLS is seeded + deterministic, so this engine never
        #                        does random restarts; measured as accurate as ik_LM
        #                        (0.0149mm) and faster on the same path.
        # _select_backend() has already pointed calculate_* at the matching backend
        # (pinocchio for the pinocchio engine, rtb for the roboticstoolbox engine).
        if self.solver == "roboticstoolbox":
            q = calculate_inverse_kinematics(
                self.robot, target_pose, q0=q_seed, method=self.config.lm_method,
                ilimit=self.ilimit, slimit=self.slimit, k=self.config.lm_k, tol=self.tol)
            if q is not None:
                try:
                    translation_error, rotation_error = self._pose_error(q, target_pose)
                    if (translation_error <= self.pose_translation_tolerance
                            and rotation_error <= self.pose_rotation_tolerance):
                        return q
                except Exception:
                    # A broken candidate (a NaN pose, a shape error) must not abort the path — the seeded DLS
                    # below is the fallback either way — but it is no longer silent (Phase 5, 2026-10-03).
                    logger.debug("ik_LM candidate rejected by the pose check; DLS fallback", exc_info=True)
        return self._solve_dls_sample(target_pose, q_seed, low, high)

    def solve_pose(self, target_pose, seed):
        """Solve IK for a SINGLE target pose (move_pose), sharing the exact engine,
        config, and fallback as the path solver: ik_LM primary → seeded DLS fallback.

        Returns a best-effort q (rad) — NOT tolerance-gated; the caller validates
        limits/residual and produces its own diagnostics. This is why move_pose is
        far more robust than a bare ik_LM call: the DLS fallback recovers targets
        ik_LM alone misses (measured: a 30mm target ik_LM missed by >400mm at every
        slimit, solved to 0.009mm once the DLS fallback runs). No continuity/branch
        check here — a single target legitimately involves whatever joint motion it
        takes to reach the pose.
        """
        self._select_backend()
        q_seed = np.asarray(seed, dtype=np.float64)[:self.joint_num].copy()
        low, high = self._limits()
        return self._solve_sample_ik(target_pose, q_seed, low, high)

    @staticmethod
    def _worst_sample(sigmas):
        """Index of the most-singular sample (smallest sigma). None if no finite value."""
        if sigmas is None or len(sigmas) == 0:
            return None
        arr = np.asarray(sigmas, dtype=np.float64)
        return None if bool(np.all(np.isnan(arr))) else int(np.nanargmin(arr))

    def _finish_failure(self, positions, valid_mask, first_failure_index, code, message,
                        branch_jumps, manipulabilities, conditions, sigmas,
                        start_delta_max=None, start_delta_norm=None,
                        translation_errors=None, rotation_errors=None):
        return IKPathResult(
            success=False,
            positions=np.asarray(positions, dtype=np.float64),
            valid_mask=valid_mask,
            first_failure_index=int(first_failure_index),
            failure_code=code,
            failure_message=str(message),
            branch_jump_indices=tuple(branch_jumps),
            min_manipulability=_safe_min(manipulabilities),
            max_jacobian_condition=_safe_max(conditions),
            min_jacobian_sigma=_safe_min(sigmas),
            condition_profile=tuple(conditions),
            sigma_profile=tuple(sigmas),
            manipulability_profile=tuple(manipulabilities),
            worst_sample_index=self._worst_sample(sigmas),
            start_delta_max=start_delta_max,
            start_delta_norm=start_delta_norm,
            start_jump_threshold=self.start_jump_threshold,
            max_translation_error=_safe_max(translation_errors or []),
            max_rotation_error=_safe_max(rotation_errors or []),
            pose_translation_tolerance=self.pose_translation_tolerance,
            pose_rotation_tolerance=self.pose_rotation_tolerance,
        )

    def solve(self, cartesian_path, seed):
        # Engine ("pinocchio" DLS / "roboticstoolbox" ik_LM) is validated at
        # construction and dispatched inside _solve_sample_ik.
        self._select_backend()
        q_seed = np.asarray(seed, dtype=np.float64)[:self.joint_num].copy()
        initial_seed = q_seed.copy()
        low, high = self._limits()

        positions = []
        valid_mask = np.zeros(cartesian_path.num_samples, dtype=bool)
        branch_jumps = []
        manipulabilities = []
        conditions = []
        sigmas = []
        translation_errors = []
        rotation_errors = []

        previous = initial_seed.copy()
        start_delta_max = 0.0
        start_delta_norm = 0.0
        for i, pose in enumerate(cartesian_path.poses):
            if i == 0:
                # The first Cartesian sample is the current FK pose by construction.
                # Solving IK for it can pick an equivalent but distant branch; anchor
                # the path to the measured joint state instead.
                q = initial_seed.copy()
            else:
                q = self._solve_sample_ik(pose, q_seed, low, high)
                if q is None:
                    code = "IK_TARGET_UNREACHABLE" if i == cartesian_path.num_samples - 1 else "IK_PARTIAL_PATH"
                    return self._finish_failure(
                        positions, valid_mask, i, code, "IK failed for Cartesian sample",
                        branch_jumps, manipulabilities, conditions, sigmas,
                        start_delta_max=start_delta_max, start_delta_norm=start_delta_norm)

                q = np.asarray(q, dtype=np.float64)[:self.joint_num]
                if q.shape[0] != self.joint_num or not np.all(np.isfinite(q)):
                    return self._finish_failure(
                        positions, valid_mask, i, "IK_PARTIAL_PATH", "IK returned an invalid joint solution",
                        branch_jumps, manipulabilities, conditions, sigmas,
                        start_delta_max=start_delta_max, start_delta_norm=start_delta_norm)

            if not self._check_limits(q, low, high):
                return self._finish_failure(
                    positions, valid_mask, i, "IK_MARGIN_VIOLATION",
                    "IK solution violates joint limits or margins", branch_jumps,
                    manipulabilities, conditions, sigmas,
                    start_delta_max=start_delta_max, start_delta_norm=start_delta_norm)

            delta_from_previous = q - previous
            max_jump = float(np.max(np.abs(delta_from_previous)))
            norm_jump = float(np.linalg.norm(delta_from_previous))
            if i == 0:
                start_delta_max = max_jump
                start_delta_norm = norm_jump
            elif max_jump > self.branch_jump_threshold:
                branch_jumps.append(i)
                return self._finish_failure(
                    positions, valid_mask, i, "IK_BRANCH_JUMP",
                    f"IK branch jump {max_jump:.3f} rad exceeds threshold",
                    branch_jumps, manipulabilities, conditions, sigmas,
                    start_delta_max=start_delta_max, start_delta_norm=start_delta_norm,
                    translation_errors=translation_errors, rotation_errors=rotation_errors)

            try:
                translation_error, rotation_error = self._pose_error(q, pose)
            except Exception as exc:
                return self._finish_failure(
                    positions, valid_mask, i, "IK_POSE_ERROR",
                    f"could not validate IK FK residual: {exc}",
                    branch_jumps, manipulabilities, conditions, sigmas,
                    start_delta_max=start_delta_max, start_delta_norm=start_delta_norm,
                    translation_errors=translation_errors, rotation_errors=rotation_errors)

            translation_errors.append(translation_error)
            rotation_errors.append(rotation_error)
            if (translation_error > self.pose_translation_tolerance
                    or rotation_error > self.pose_rotation_tolerance):
                return self._finish_failure(
                    positions, valid_mask, i, "IK_POSE_ERROR",
                    f"IK FK residual exceeds tolerance: translation={translation_error:.6f} m, "
                    f"rotation={rotation_error:.6f} rad",
                    branch_jumps, manipulabilities, conditions, sigmas,
                    start_delta_max=start_delta_max, start_delta_norm=start_delta_norm,
                    translation_errors=translation_errors, rotation_errors=rotation_errors)

            try:
                manipulability, condition, sigma_min = self._singularity_metrics(q)
            except Exception:
                manipulability = condition = sigma_min = None
            # Always append one value per sample (NaN when unavailable) so the arrays
            # stay index-aligned with `positions` — needed for "worst sample #i".
            manipulabilities.append(np.nan if manipulability is None else manipulability)
            conditions.append(np.nan if condition is None else condition)
            sigmas.append(np.nan if sigma_min is None else sigma_min)

            positions.append(q)
            valid_mask[i] = True
            previous = q
            q_seed = q

        return IKPathResult(
            success=True,
            positions=np.asarray(positions, dtype=np.float64),
            valid_mask=valid_mask,
            branch_jump_indices=tuple(branch_jumps),
            min_manipulability=_safe_min(manipulabilities),
            max_jacobian_condition=_safe_max(conditions),
            min_jacobian_sigma=_safe_min(sigmas),
            condition_profile=tuple(conditions),
            sigma_profile=tuple(sigmas),
            manipulability_profile=tuple(manipulabilities),
            worst_sample_index=self._worst_sample(sigmas),
            start_delta_max=start_delta_max,
            start_delta_norm=start_delta_norm,
            start_jump_threshold=self.start_jump_threshold,
            max_translation_error=_safe_max(translation_errors),
            max_rotation_error=_safe_max(rotation_errors),
            pose_translation_tolerance=self.pose_translation_tolerance,
            pose_rotation_tolerance=self.pose_rotation_tolerance,
        )


_json_metric = json_safe_metric  # single implementation lives in motion/ik_config.py


def _json_metric_list(values, is_condition=False):
    """JSON-safe per-sample array (see _json_metric)."""
    return [_json_metric(v, is_condition) for v in values]


def _profile_at(profile, index, is_condition=False):
    """One profile value by index, JSON-safe. None if out of range."""
    if index is None or index < 0 or index >= len(profile):
        return None
    return _json_metric(profile[index], is_condition)


def _safe_min(values):
    if values is None or len(values) == 0:
        return None
    arr = np.asarray(values, dtype=np.float64)
    return None if bool(np.all(np.isnan(arr))) else float(np.nanmin(arr))


def _safe_max(values):
    if values is None or len(values) == 0:
        return None
    arr = np.asarray(values, dtype=np.float64)
    return None if bool(np.all(np.isnan(arr))) else float(np.nanmax(arr))
