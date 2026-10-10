"""Can the arm reach this TCP pose at all? — the global search behind `is_reachable`.

The planner's IK (motion/ik_path.py) is LOCAL by design: it follows paths, so its damped fallback moves
at most 0.02 rad per iteration and ik_LM runs seeded with no restarts (IKConfig.slimit = 1, because a
restart flips branches mid-path). Measured 2026-10-01 on PAR6: from a random start it solved 0 of 40
reachable poses, and even 65 starts missed 29 of 100. A "no" from it means "not from this start", not
"out of reach" — fine for a path, wrong for a reachability question.

This answers the global question, the way an industrial controller's "does an IK solution exist" query does:
  1. ik_LM from `near` first, then from a fixed spread of starts over the joint range (deterministic:
     the same pose always gets the same answer);
  2. each answer is moved by whole turns into the joint range — PAR6's J3 runs 114..376 deg and J6
     -49..409 deg, so a correct solution often comes back 360 deg off (without this, 39 of 100 solved);
  3. accepted only inside the software joint limits and within the planner's own pose tolerances.
Measured: 300 of 300 reachable poses found, none needing more than 32 starts, median 0.2 ms. A pose
that is out of reach costs every start (~45 ms at 128).

Self-collision (2026-10-07): find_reachable_clear() takes the arm's convex pieces (utility/self_collision.py)
and skips a solution that brings two links within the PLANNER margin — the margin the planner would refuse
move_joints(joints) with — and keeps searching the other branches (elbow up / down, wrist flip, J1 +- pi). Only
when every solution found self-collides does it return the closest one, with the pair, so the caller can say
why. It checks the END pose only; the planner still checks the path to it.

NOT thread-safe per robot model: pinocchio keeps one scratch buffer per model, and two threads sharing
one returned wrong poses on 3880 of 80000 calls. Each concurrent user needs its own model.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from motion.ik_config import DEFAULT_IK_CONFIG

REACH_STARTS = 128      # 4x the most any of 300 reachable poses needed
_START_SEED = 2026
# ik_LM's tol bounds E = 0.5*e'e, not a distance; the planner's 1e-5 allows ~4.5 mm, which the pose
# check below would then reject as a false "no". 1e-12 is what the 300/300 measurement used.
_LM_TOL = 1e-12
_LIMIT_SLACK = 1e-6     # same slack move_pose's limit check uses


def make_starts(low: np.ndarray, high: np.ndarray, count: int = REACH_STARTS, seed: int = _START_SEED) -> np.ndarray:
    """Fixed IK start points spread uniformly over [low, high] (rad), shape (count, joints)."""
    low = np.asarray(low, dtype=np.float64)
    high = np.asarray(high, dtype=np.float64)
    return low + np.random.default_rng(seed).random((count, low.shape[0])) * (high - low)


def _wrap_into_limits(q, near, low, high):
    """Move each joint by whole turns: nearest to `near` first, then any turn that fits the range."""
    q = q + 2.0 * np.pi * np.round((near - q) / (2.0 * np.pi))
    for j in range(q.shape[0]):
        if low[j] <= q[j] <= high[j]:
            continue
        for turns in (1, -1, 2, -2):
            candidate = q[j] + 2.0 * np.pi * turns
            if low[j] <= candidate <= high[j]:
                q[j] = candidate
                break
    return q


def _fk(robot, q):
    """4x4 TCP pose from the robot's OWN model — never the module-global backend switch the planner
    thread flips (utility/kinematics_backend.py)."""
    if hasattr(robot, "pinocchio_fkine"):
        return robot.pinocchio_fkine(q)
    return np.asarray(robot.fkine(q).A, dtype=np.float64)


def _pose_ok(robot, q, target):
    actual = _fk(robot, q)
    if np.linalg.norm(actual[:3, 3] - target[:3, 3]) > DEFAULT_IK_CONFIG.pose_translation_tolerance_m:
        return False
    c = (np.trace(actual[:3, :3].T @ target[:3, :3]) - 1.0) * 0.5
    return float(np.arccos(np.clip(c, -1.0, 1.0))) <= DEFAULT_IK_CONFIG.pose_rotation_tolerance_rad


def find_reachable(robot: Any, target: np.ndarray, near: np.ndarray, low: np.ndarray, high: np.ndarray,
                   starts: np.ndarray, accept: Callable[[np.ndarray], bool] | None = None) -> np.ndarray | None:
    """Joint solution (rad) for the 4x4 TCP pose `target` inside [low, high], or None if none exists.

    Of the solutions found, returns the one closest to `near` (largest single-joint difference): the
    answer from `near`'s own start if that one succeeds, else the closest over all `starts`.
    accept: optional test of a solution (e.g. clear of self-collision); one it refuses is skipped as if
    the start had failed, so the search goes on to the other branches.
    """
    target = np.asarray(target, dtype=np.float64)
    near = np.asarray(near, dtype=np.float64)
    low = np.asarray(low, dtype=np.float64)
    high = np.asarray(high, dtype=np.float64)
    near_ref = np.clip(near, low, high)
    best, best_dist = None, np.inf
    for i in range(starts.shape[0] + 1):
        # rtb's Robot.ik_LM returns a plain tuple (q, success, iterations, searches, residual).
        sol = robot.ik_LM(target, q0=near if i == 0 else starts[i - 1],
                          ilimit=DEFAULT_IK_CONFIG.ilimit, slimit=1, tol=_LM_TOL, joint_limits=False,
                          k=DEFAULT_IK_CONFIG.lm_k, method=DEFAULT_IK_CONFIG.lm_method)
        if not sol[1]:
            continue
        q = _wrap_into_limits(np.asarray(sol[0], dtype=np.float64), near_ref, low, high)
        if np.any(q < low - _LIMIT_SLACK) or np.any(q > high + _LIMIT_SLACK) or not _pose_ok(robot, q, target):
            continue
        if accept is not None and not accept(q):
            continue
        if i == 0:
            return q
        dist = float(np.max(np.abs(q - near)))
        if dist < best_dist:
            best, best_dist = q, dist
    return best


def find_reachable_clear(robot: Any, target: np.ndarray, near: np.ndarray, low: np.ndarray, high: np.ndarray,
                         starts: np.ndarray, sc_model: Any, which: str = "planner"
                         ) -> tuple[np.ndarray | None, tuple[int, int] | None]:
    """find_reachable, preferring solutions clear of self-collision. sc_model: a
    utility.self_collision.SelfCollisionModel of THIS caller's own (its Pinocchio data is a scratch buffer —
    the same thread rule as the robot model), or None for no check.

    Returns (q, None) — the closest solution whose links are all outside the `which` margin; (q, (a, b)) — every
    solution found brings two links within it: the closest of those and its first flagged pair (frame indices);
    (None, None) — no solution inside the limits at all."""
    if sc_model is None:
        return find_reachable(robot, target, near, low, high, starts), None
    refused: list[tuple[np.ndarray, tuple[int, int]]] = []

    def clear(q: np.ndarray) -> bool:
        hits = sc_model.check(q, which)
        if hits:
            refused.append((q.copy(), hits[0]))
        return not hits

    q = find_reachable(robot, target, near, low, high, starts, accept=clear)
    if q is not None or not refused:
        return q, None
    near = np.asarray(near, dtype=np.float64)
    q, pair = min(refused, key=lambda r: float(np.max(np.abs(r[0] - near))))
    return q, pair
