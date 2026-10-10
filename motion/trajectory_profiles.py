"""Trajectory profile generators for the EXEC planner (Phase C).

Pure math, no hardware, no ZMQ — trivially unit-testable (plot position/velocity/
acceleration to eyeball a profile before running it on the arm).

Design: a single normalized path parameter ``s(tau)`` in [0, 1] drives ALL joints, so
they are automatically time-synchronized and follow a straight line in joint space
(each joint scales the same s-curve by its own travel distance). This is the classic
path-parameterization approach — synchronization falls out for free.

Profiles are pluggable via the ``TrajectoryProfile`` base class:
  - TrapezoidalProfile — trapezoidal velocity, blended accel/decel.
  - QuinticProfile — smooth 5th-order time scaling, zero endpoint velocity/accel.
  - SCurveProfile — 7th-order S-curve time scaling, zero endpoint jerk too.

The planner calls ``plan_joint_move()`` which picks a total duration T that respects
every joint's velocity and acceleration limit, then samples at the RT period ``dt``.

Blending (velocity-continuous, straight joint-space segments — see
TODO/executive_mode_plan.md §3.9): ``plan_joint_move`` optionally takes nonzero
``start_vel``/``end_vel`` boundary velocities so consecutive moves don't have to stop
at the shared corner. ``compute_corner_speed`` decides how much speed is safe to
carry through that corner. See the module-level note above ``compute_corner_speed``
for the derivation.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np


class TrajectoryProfile(ABC):
    """Normalized path parameter s(tau), tau in [0, 1], s(0)=0 s(1)=1.

    Returns (s, s_dot_norm) where s_dot_norm is ds/dtau (per unit normalized time).
    Real joint velocity = distance * s_dot_norm / T.
    """

    @abstractmethod
    def sample(self, tau: np.ndarray):
        """Return (s, s_dot_norm) evaluated at each tau in [0, 1]."""
        ...

    @property
    @abstractmethod
    def peak_vel_norm(self) -> float:
        """Peak of ds/dtau over the profile (used to size T against vel limits)."""
        ...

    @property
    @abstractmethod
    def peak_acc_norm(self) -> float:
        """Peak of d2s/dtau2 over the profile (used to size T against accel limits)."""
        ...


class QuinticProfile(TrajectoryProfile):
    """5th-order polynomial time scaling: s=10t^3-15t^4+6t^5.

    Smooth endpoint behavior: s'(0)=s'(1)=0 and s''(0)=s''(1)=0.
    The peak normalized velocity is 15/8. The peak normalized acceleration is
    10/sqrt(3), at tau=(3±sqrt(3))/6.
    """

    @property
    def peak_vel_norm(self):
        return 15.0 / 8.0

    @property
    def peak_acc_norm(self):
        return 10.0 / np.sqrt(3.0)

    def sample(self, tau):
        tau = np.asarray(tau, dtype=np.float64)
        t2 = tau * tau
        t3 = t2 * tau
        t4 = t3 * tau
        t5 = t4 * tau
        s = 10.0 * t3 - 15.0 * t4 + 6.0 * t5
        sd = 30.0 * t2 - 60.0 * t3 + 30.0 * t4
        return np.clip(s, 0.0, 1.0), sd


class SCurveProfile(TrajectoryProfile):
    """7th-order S-curve time scaling: s=35t^4-84t^5+70t^6-20t^7.

    This is the classic smootherstep profile. It has zero endpoint velocity,
    acceleration, and jerk. It is not TOPPRA/Ruckig; it is a simple normalized
    stop-to-stop profile that fits the existing pure-numpy path timing lane.
    """

    @property
    def peak_vel_norm(self):
        return 35.0 / 16.0

    @property
    def peak_acc_norm(self):
        return 84.0 * np.sqrt(5.0) / 25.0

    def sample(self, tau):
        tau = np.asarray(tau, dtype=np.float64)
        t2 = tau * tau
        t3 = t2 * tau
        t4 = t3 * tau
        t5 = t4 * tau
        t6 = t5 * tau
        t7 = t6 * tau
        s = 35.0 * t4 - 84.0 * t5 + 70.0 * t6 - 20.0 * t7
        sd = 140.0 * t3 - 420.0 * t4 + 420.0 * t5 - 140.0 * t6
        return np.clip(s, 0.0, 1.0), sd


class TrapezoidalProfile(TrajectoryProfile):
    """Trapezoidal velocity in normalized time.

    ``blend`` is the fraction of the move spent accelerating (and again decelerating).
    blend=0.5 → triangular (no cruise); blend→0 → almost constant velocity.
    """

    def __init__(self, blend: float = 0.25):
        if not (0.0 < blend <= 0.5):
            raise ValueError(f"blend must be in (0, 0.5], got {blend}")
        self._b = float(blend)
        self._v_pk = 1.0 / (1.0 - self._b)        # peak normalized velocity
        self._a_norm = self._v_pk / self._b       # normalized acceleration magnitude

    @property
    def peak_vel_norm(self):
        return self._v_pk

    @property
    def peak_acc_norm(self):
        return self._a_norm

    def sample(self, tau):
        tau = np.asarray(tau, dtype=np.float64)
        b, v_pk, a = self._b, self._v_pk, self._a_norm
        s = np.empty_like(tau)
        sd = np.empty_like(tau)

        accel = tau < b
        decel = tau > (1.0 - b)
        cruise = ~(accel | decel)

        # Accelerate: s = ½ a τ²,  s' = a τ
        s[accel] = 0.5 * a * tau[accel] ** 2
        sd[accel] = a * tau[accel]

        # Cruise: s = ½ v_pk b + v_pk (τ - b),  s' = v_pk
        s[cruise] = 0.5 * v_pk * b + v_pk * (tau[cruise] - b)
        sd[cruise] = v_pk

        # Decelerate (mirror of accel): s = 1 - ½ a (1-τ)²,  s' = a (1-τ)
        u = 1.0 - tau[decel]
        s[decel] = 1.0 - 0.5 * a * u ** 2
        sd[decel] = a * u

        # Clamp endpoints exactly (guard against float drift at tau=0/1).
        s = np.clip(s, 0.0, 1.0)
        return s, sd


def _solve_duration(dist, max_vel, max_accel, dt, speed_scale, profile):
    """Minimum move duration (s) so no joint exceeds max_vel/max_accel, given the
    normalized profile's peak velocity/acceleration (extracted from plan_joint_move
    so compute_corner_speed can size a *candidate* neighbor move's duration without
    generating its full waypoint arrays).

    Returns None if dist is ~zero (no motion — caller should short-circuit; there's
    no meaningful duration for a move that doesn't go anywhere).
    """
    adist = np.abs(dist)
    moving = adist > 1e-9
    if not np.any(moving):
        return None
    v_pk = profile.peak_vel_norm
    a_pk = profile.peak_acc_norm
    t_vel = np.max((adist[moving] * v_pk) / np.maximum(max_vel[moving], 1e-9))
    t_acc = np.max(np.sqrt((adist[moving] * a_pk) / np.maximum(max_accel[moving], 1e-9)))
    return max(t_vel, t_acc) / speed_scale


def _sample_boundary_trapezoid(tau: np.ndarray, w0: float, w1: float, alpha: float) -> tuple[np.ndarray, np.ndarray]:
    """Asymmetric trapezoid in normalized time: s(0)=0, s(1)=1, sd(0)=w0, sd(1)=w1,
    constant-magnitude acceleration ``alpha`` during the ramp phases. Reduces exactly
    to TrapezoidalProfile's own accel/cruise/decel shape when w0=w1=0.

    Derivation (normalized units, tau in [0,1]): with cruise speed wc unknown, the
    accel phase runs tau in [0, tau1] with tau1=(wc-w0)/alpha, the decel phase runs
    tau in [tau2, 1] with tau2=1-(wc-w1)/alpha, and the total distance covered must
    equal 1. That gives a quadratic in wc:

        wc^2 - wc*(alpha+w0+w1) + alpha + (w0^2+w1^2)/2 = 0

    The smaller root is the physically meaningful one (the larger root corresponds to
    "overshoot past 1 and come back," not a valid forward-only trapezoid) — verified
    against the w0=w1=0 case, where it reduces exactly to TrapezoidalProfile's own
    v_pk.

    Raises ValueError if no valid trapezoid exists (w0/w1 too large for the available
    accel budget within distance 1 — would need a triangular profile instead). Callers
    should treat this as "can't blend this pair," not attempt to recover a partial
    profile — see trajectory_planner.py's fallback to a normal (non-blended) plan.
    """
    tau = np.asarray(tau, dtype=np.float64)
    b = alpha + w0 + w1
    disc = b * b - 4.0 * (alpha + 0.5 * (w0 * w0 + w1 * w1))
    if disc < 0.0:
        raise ValueError(f"boundary trapezoid infeasible (no real solution): "
                         f"w0={w0}, w1={w1}, alpha={alpha}")
    wc = (b - np.sqrt(disc)) / 2.0
    if wc < max(w0, w1) - 1e-9:
        raise ValueError(f"boundary trapezoid infeasible: wc={wc} < max(w0,w1)={max(w0, w1)}")

    tau1 = (wc - w0) / alpha
    tau2 = 1.0 - (wc - w1) / alpha
    if tau1 > tau2 + 1e-9:
        raise ValueError(f"boundary trapezoid infeasible: accel/decel phases overlap "
                         f"(tau1={tau1:.4f}, tau2={tau2:.4f})")
    tau2 = max(tau1, tau2)  # guard float drift when the cruise phase is ~0-length

    s = np.empty_like(tau)
    sd = np.empty_like(tau)

    accel = tau < tau1
    decel = tau > tau2
    cruise = ~(accel | decel)

    s[accel] = w0 * tau[accel] + 0.5 * alpha * tau[accel] ** 2
    sd[accel] = w0 + alpha * tau[accel]

    s_at_tau1 = w0 * tau1 + 0.5 * alpha * tau1 ** 2
    s[cruise] = s_at_tau1 + wc * (tau[cruise] - tau1)
    sd[cruise] = wc

    u = 1.0 - tau[decel]
    s[decel] = 1.0 - (w1 * u + 0.5 * alpha * u ** 2)
    sd[decel] = w1 + alpha * u

    s = np.clip(s, 0.0, 1.0)
    return s, sd


def compute_corner_speed(dist_a, dist_b, max_vel, max_accel, dt, blend_scale,
                         speed_scale=1.0, profile: TrajectoryProfile = None):
    """Scalar corner speed (rad-generalized/s) safe to carry through the A->B
    junction without stopping.

    The two segments travel in different joint-space directions in general, so
    there's no single velocity *vector* that's simultaneously the natural
    continuation of A's own path and the natural start of B's own path. Instead A's
    last waypoint carries velocity ``v_c * unit(dist_a)`` and B's first waypoint
    carries ``v_c * unit(dist_b)`` — same magnitude, different direction — and the
    direction change happens across the one tick between them. ``v_c`` is capped so
    that implied one-tick direction change never asks any joint for more than
    max_accel*dt (conservative — this is the same "junction velocity" approximation
    CNC and 3D-printer firmware use for cornering, sized against our own RT tick instead of
    an idealized instant).

    Further capped by ``blend_scale`` (0..1, from set_blending()) and by how sharp
    the corner is: cos(angle between directions), floored at 0 — a 90-degree corner
    or sharper gets no speed preserved (matches physical intuition: you can't cruise
    through a near-reversal without exceeding the accel budget above).

    Returns 0.0 for a degenerate pair (no motion on either side, blend_scale<=0, or
    corner too sharp) — callers should treat that as "plan this pair normally."

    Note: v_cap here can still land on a value _sample_boundary_trapezoid() can't
    actually satisfy. A move's duration T is fixed by the ordinary 0-to-0 sizing
    (see plan_joint_move), and asking that same fixed-T move to exit at its own
    full cruise peak is analytically infeasible — for the default profile shape
    (blend=0.25) the max single-sided boundary speed starting from rest is only
    ~84% of peak, not 100%. This is most visible at blend_scale=100 on an
    equal-distance, dead-straight pair (the degenerate worst case, not a typical
    one — see dummy_test_scripts/test_blend_integration.py's "fallback safety
    net" case). compute_corner_speed does not try to predict this — it's cheaper
    and just as safe to let plan_joint_move raise and have the caller
    (trajectory_planner._plan_move_joints) fall back to planning that pair
    without a boundary velocity.
    """
    profile = profile or TrapezoidalProfile()
    dist_a = np.asarray(dist_a, dtype=np.float64)
    dist_b = np.asarray(dist_b, dtype=np.float64)
    if blend_scale <= 0.0:
        return 0.0

    norm_a = float(np.linalg.norm(dist_a))
    norm_b = float(np.linalg.norm(dist_b))
    if norm_a < 1e-9 or norm_b < 1e-9:
        return 0.0

    u_a = dist_a / norm_a
    u_b = dist_b / norm_b
    cos_theta = float(np.clip(np.dot(u_a, u_b), -1.0, 1.0))
    if cos_theta <= 0.0:
        return 0.0

    max_vel = np.asarray(max_vel, dtype=np.float64)
    max_accel = np.asarray(max_accel, dtype=np.float64)

    t_a = _solve_duration(dist_a, max_vel, max_accel, dt, speed_scale, profile)
    t_b = _solve_duration(dist_b, max_vel, max_accel, dt, speed_scale, profile)
    if t_a is None or t_b is None:
        return 0.0
    peak_speed_a = norm_a * profile.peak_vel_norm / t_a
    peak_speed_b = norm_b * profile.peak_vel_norm / t_b

    delta_u = u_b - u_a
    significant = np.abs(delta_u) > 1e-9
    if np.any(significant):
        accel_cap = float(np.min(max_accel[significant] * dt / np.abs(delta_u[significant])))
    else:
        accel_cap = np.inf   # dead-straight corner — no direction-change constraint

    v_cap = blend_scale * min(peak_speed_a, peak_speed_b)
    return max(0.0, cos_theta * min(accel_cap, v_cap))


def plan_joint_move(start, end, max_vel, max_accel, dt,
                    speed_scale=1.0, profile: TrajectoryProfile = None,
                    start_vel=None, end_vel=None):
    """Time-synchronized joint-space move under per-joint vel/accel limits.

    Args:
        start:       (J,) start joint angles (rad).
        end:         (J,) target joint angles (rad).
        max_vel:     (J,) per-joint velocity limit (rad/s).
        max_accel:   (J,) per-joint acceleration limit (rad/s²).
        dt:          RT sample period (s) — one waypoint per dt.
        speed_scale: 0<scale<=1, fraction of full speed (from set_joint_speed).
        profile:     TrajectoryProfile (default TrapezoidalProfile()).
        start_vel:   optional (J,) rad/s boundary velocity, must be parallel to
                     (end-start) — i.e. speed * unit(end-start) for some speed>=0.
                     None (default) = start from rest, byte-for-byte the original
                     behavior. Used for blending into the end of a previous move —
                     see compute_corner_speed().
        end_vel:     optional (J,) rad/s boundary velocity, must be parallel to
                     (end-start). None (default) = stop at the end, original
                     behavior. Used for blending into the start of the next move.

    Returns:
        (positions (N, J) float64, velocities (N, J) float64, N).
        N >= 2. If start == end, returns two identical hold waypoints.

    Raises:
        ValueError if start_vel/end_vel are given but too large for the move's own
        distance/accel budget to reach in a valid trapezoid (see
        _sample_boundary_trapezoid) — callers should fall back to planning this move
        with start_vel=end_vel=None rather than propagate a broken profile.
    """
    profile = profile or TrapezoidalProfile()
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    max_vel = np.asarray(max_vel, dtype=np.float64)
    max_accel = np.asarray(max_accel, dtype=np.float64)
    speed_scale = float(np.clip(speed_scale, 1e-3, 1.0))

    dist = end - start
    J = dist.shape[0]

    T = _solve_duration(dist, max_vel, max_accel, dt, speed_scale, profile)
    if T is None:
        # No motion — return a minimal hold pair (planner still checkpoints it).
        pos = np.repeat(start[None, :], 2, axis=0)
        return pos, np.zeros((2, J)), 2

    n = max(2, int(round(T / dt)) + 1)
    tau = np.linspace(0.0, 1.0, n)

    if start_vel is None and end_vel is None:
        s, sd = profile.sample(tau)
    else:
        D = np.linalg.norm(dist)
        w0 = float(np.linalg.norm(start_vel)) * T / D if start_vel is not None else 0.0
        w1 = float(np.linalg.norm(end_vel)) * T / D if end_vel is not None else 0.0
        s, sd = _sample_boundary_trapezoid(tau, w0, w1, profile.peak_acc_norm)

    positions = start[None, :] + np.outer(s, dist)          # (N, J)
    velocities = np.outer(sd / T, dist)                     # (N, J) = dist * s'(tau)/T
    return positions, velocities, n
