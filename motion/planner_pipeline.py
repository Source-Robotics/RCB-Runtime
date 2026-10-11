"""Shared EXEC planner pipeline stages.

G3 introduces explicit stage names without changing the current joint-space math:

    PathBuilder -> TimeParameterizer -> Sampler -> ExecHandler batch payload

The first implementation is intentionally thin.  It wraps the existing
``trajectory_profiles.plan_joint_move()`` behavior so move_joints remains
byte-for-byte compatible while future Cartesian/IK/profile work has stable seams.
"""

from __future__ import annotations

from typing import Any

from dataclasses import dataclass

import numpy as np

from motion.exec_commands import normalize_profile_name
from motion.joint_paths import CORNER_DEVIATION_COEFF
from motion.trajectory_profiles import (
    plan_joint_move, TrapezoidalProfile, QuinticProfile, SCurveProfile,
)


@dataclass(frozen=True)
class JointPath:
    """Joint-space path geometry before timing."""

    start: np.ndarray
    target: np.ndarray

    @property
    def delta(self):
        return self.target - self.start


@dataclass(frozen=True)
class TimedJointPath:
    """Joint path sampled in time but before EXEC feedforward packaging."""

    positions: np.ndarray
    velocities: np.ndarray
    num_points: int


@dataclass(frozen=True)
class ExecTrajectorySample:
    """Sampler output compatible with communication.exec_channels.pack_batch()."""

    positions: np.ndarray
    velocities: np.ndarray
    torque_ff: np.ndarray

    @property
    def num_points(self):
        return int(self.positions.shape[0])


class PathBuilder:
    """Build path geometry for EXEC commands."""

    def __init__(self, joint_num):
        self.joint_num = int(joint_num)

    def joint_line_from_degrees(self, start, args):
        if len(args) < self.joint_num:
            raise ValueError(f"move_joints needs {self.joint_num} angles, got {len(args)}")
        start = np.asarray(start, dtype=np.float64)[:self.joint_num]
        target = np.deg2rad(np.asarray(args[:self.joint_num], dtype=np.float64))
        return JointPath(start=start, target=target)

    def joint_line_from_radians(self, start, target):
        start = np.asarray(start, dtype=np.float64)[:self.joint_num]
        target = np.asarray(target, dtype=np.float64)[:self.joint_num]
        if target.shape[0] < self.joint_num:
            raise ValueError(f"joint target needs {self.joint_num} angles, got {target.shape[0]}")
        return JointPath(start=start, target=target)


class TimeParameterizer:
    """Base interface for path timing stages."""

    def parameterize(self, path, max_vel, max_accel, dt, speed_scale=1.0,
                     start_vel=None, end_vel=None):
        raise NotImplementedError


class IKPathSolver:
    """Base interface for Cartesian/pose path conversion.

    Pure joint-space paths skip this stage. Cartesian support will implement seeded
    IK here in G5/G6.
    """

    def solve(self, cartesian_path, seed):
        raise NotImplementedError


class ProfileTimeParameterizer(TimeParameterizer):
    """Generic normalized-profile time parameterizer."""

    supports_boundary_velocity = False

    def __init__(self, profile):
        self.profile = profile

    def parameterize(self, path, max_vel, max_accel, dt, speed_scale=1.0,
                     start_vel=None, end_vel=None):
        if not self.supports_boundary_velocity:
            start_vel = None
            end_vel = None
        pos, vel, n = plan_joint_move(
            path.start, path.target, max_vel, max_accel, dt, profile=self.profile,
            speed_scale=speed_scale, start_vel=start_vel, end_vel=end_vel)
        return TimedJointPath(positions=pos, velocities=vel, num_points=n)


class TrapezoidTimeParameterizer(ProfileTimeParameterizer):
    """LSPB (linear segment with parabolic blend), wrapped as a pipeline stage.

    `ramp_fraction` is the textbook LSPB blend: the fraction of the move spent
    ACCELERATING (and again decelerating). It fixes the whole shape:

        peak velocity     = 1 / (1 - r)      x the average velocity
        peak acceleration = peak_vel / r

        r = 1/3   -> 1/3 accel, 1/3 cruise, 1/3 decel.  peak vel 1.5x, peak accel 4.5.
                     THE "1/3 RULE" — Peter Corke's lspb() default (V = 1.5*(qf-q0)/T).
        r = 0.25  -> 1/4 accel, 1/2 cruise, 1/4 decel.  peak vel 1.333x, peak accel 5.333.
                     Our default: a longer cruise with sharper ramps.
        r -> 0.5  -> triangular, no cruise at all.

    Neither is "more correct" — they trade peak velocity against peak acceleration for the
    same duration. Exposed so a caller can ask for the 1/3 shape explicitly.
    """

    supports_boundary_velocity = True

    def __init__(self, ramp_fraction=0.25):
        super().__init__(TrapezoidalProfile(blend=float(ramp_fraction)))


class QuinticTimeParameterizer(ProfileTimeParameterizer):
    """Quintic timing behavior for smooth stop-to-stop joint moves."""

    def __init__(self):
        super().__init__(QuinticProfile())


class SCurveTimeParameterizer(ProfileTimeParameterizer):
    """7th-order S-curve timing behavior for smooth stop-to-stop joint moves."""

    def __init__(self):
        super().__init__(SCurveProfile())


class RuckigUnavailable(RuntimeError):
    """profile='ruckig' was requested but the ruckig package is not importable."""


class RuckigInfeasible(RuntimeError):
    """Ruckig could not produce a trajectory for the requested move/limits."""


class TOPPRAUnavailable(RuntimeError):
    """The toppra package is not importable. Specific subclass (not a plain
    RuntimeError) so trajectory_planner.py can translate it to a real
    PlannerError code (TOPPRA_UNAVAILABLE) instead of the generic PLAN_FAIL
    fallback — see TODO/activity_log_design.md §9."""


class TOPPRAInfeasible(ValueError):
    """TOPPRA could not compute a valid trajectory for the given constraints.
    Specific subclass (not a plain ValueError) for the same reason as
    TOPPRAUnavailable above — and so it doesn't get swept into
    trajectory_planner.py's generic `except ValueError` blend-infeasible
    fallback in _plan_move_joints(), which predates this exception and is
    unrelated to it."""


def path_parameter(waypoints):
    """The spline's knot vector: cumulative chord length, normalized to [0, 1].

    Shared by build_geometric_path (the curve) and geometric_gridpoints (where TOPPRA
    evaluates its constraints). These MUST agree — see geometric_gridpoints for what happens
    when they don't.
    """
    waypoints = np.asarray(waypoints, dtype=np.float64)
    d = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    total = float(d.sum())
    if total < 1e-12:
        return np.linspace(0.0, 1.0, waypoints.shape[0])
    ss = np.maximum.accumulate(np.concatenate(([0.0], np.cumsum(d))) / total)
    ss[0], ss[-1] = 0.0, 1.0
    return ss


def geometric_gridpoints(waypoints, min_points=100):
    """Where TOPPRA evaluates its constraints — the path's OWN knots, refined.

    THIS IS A SAFETY-CRITICAL FUNCTION. TOPPRA only enforces its acceleration limit AT the
    gridpoints. `linspace(0, 1, N)` spreads them EVENLY IN s — which is fine only while the
    samples are evenly spaced in arc length too.

    Once s is cumulative chord length (which is what makes a densely-sampled blend zone
    well-posed — see build_geometric_path), a TIGHT corner occupies a tiny slice of s. With
    uniform gridpoints, NO GRIDPOINT LANDS INSIDE THE CORNER. TOPPRA then never sees the
    curvature there, does not brake, and happily emits a trajectory that violates the joint
    acceleration limit: measured 17.3 rad/s^2 against a 2.0 limit (8.6x) on a 0.05mm
    tolerance — fast, smooth-looking, and it would have faulted the drivers.

    So the gridpoints are built FROM the path's own knots, then bisected until there are
    enough. That keeps them dense exactly where the samples are dense — i.e. inside the
    corners, which is the only place the acceleration constraint actually bites.
    """
    grid = path_parameter(waypoints)
    while grid.size < int(min_points):
        grid = np.unique(np.concatenate([grid, 0.5 * (grid[:-1] + grid[1:])]))
    return grid


def blend_distance_for_tolerance(p_prev: np.ndarray, p: np.ndarray, p_next: np.ndarray, tol: float,
                                 continuity: str = "c1") -> float:
    """`G64 P<tol>` — the blend distance that keeps a corner within `tol` of the vertex.

    The G-code knob. A slicer emits no blend information at all: G-code is a bare list of
    moves, and the corner policy is entirely the CONTROLLER's. The established answer is to
    take a TOLERANCE (`G64 P0.05`) and round each corner as hard as you like so long as the
    path never strays further than `tol` from the programmed one. That composes across thousands
    of segments of wildly varying length; a fixed `blend=<mm>` distance does not — which is
    why every corner of a dense mesh would otherwise just pin to the half-segment clamp.

    Our corner is a QUADRATIC BEZIER with control point at the vertex `p` and endpoints a
    distance `d` back along each leg. Its closest approach to the vertex is at t=0.5:

        B(0.5) = (P0 + 2p + P2) / 4
        |p - B(0.5)| = |2p - P0 - P2| / 4 = d * |u_in - u_out| / 4 = d * sin(phi/2) / 2

    where `phi` is the TURN angle (0 = straight on, pi = full reversal). Setting that equal
    to `tol` and inverting:

        d = 2 * tol / sin(phi/2)

    Exactly invertible — no iteration — and verified against the sampled curve to 1e-6 over
    10..179 degrees. It also behaves correctly at both ends: a shallow 10-degree corner gets
    a generous blend (little deviation per unit of rounding), a 179-degree reversal gets a
    tight one. Sharp corners are rounded cautiously, gentle ones freely, which is precisely
    what a deviation budget should buy.

    NOTE this is a *distance*, unclamped. The caller still applies the half-segment clamp —
    a near-straight corner returns a huge d, which the clamp turns into a Bezier through
    almost-collinear points, i.e. a straight line. Harmless by construction.
    """
    a = np.asarray(p_prev, dtype=np.float64) - np.asarray(p, dtype=np.float64)
    b = np.asarray(p_next, dtype=np.float64) - np.asarray(p, dtype=np.float64)
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-9 or nb < 1e-9 or tol <= 0.0:
        return 0.0                      # degenerate corner (or blending switched off)
    # Angle BETWEEN the legs (pi = straight through, 0 = full reversal), so the turn angle
    # is phi = pi - theta and sin(phi/2) = cos(theta/2).
    cos_theta = float(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))
    half = 0.5 * float(np.arccos(cos_theta))
    sin_half_phi = float(np.cos(half))
    if sin_half_phi < 1e-9:
        return float("inf")             # dead straight: no corner to cut, clamp handles it
    coeff = CORNER_DEVIATION_COEFF.get(continuity, CORNER_DEVIATION_COEFF["c1"])
    return float(tol) / (coeff * sin_half_phi)


def build_geometric_path(waypoints: np.ndarray) -> Any:
    """The TOPPRA geometric path for a joint waypoint array (s in [0, 1]).

    Shared so the timing solve and the sd (path-velocity) conversion below always
    describe the SAME curve — a tangent taken from a different spline than the one
    TOPPRA times would silently corrupt a rolling-buffer splice.
    """
    import toppra as ta
    waypoints = np.asarray(waypoints, dtype=np.float64)
    # PARAMETERIZE BY CUMULATIVE CHORD LENGTH, not by index.
    #
    # `linspace` says "every sample is an equal step in s". That is only true when the
    # samples are equally spaced in ARC LENGTH — which is exactly what §9.4's fix arranged,
    # and why linspace worked. But a blend zone is sampled with a floor of 3 points
    # (joint_paths/cartesian_paths `n = max(3, ...)`), so a TIGHT tolerance packs 3 samples
    # into a sliver of path while the legs stay coarse. Under linspace those 3 samples get a
    # full 2/(N-1) of the s-axis, so |dq/ds| COLLAPSES inside the zone and the spline reads
    # it as enormous curvature. TOPPRA then brakes for a corner far sharper than the real
    # one: measured 2.06s -> 3.14s (a 50% slowdown) as the tolerance went 2.0mm -> 0.2mm,
    # with the corner geometry unchanged. That is §9.4 again, arriving through a new door.
    #
    # Chord length makes |dq/ds| constant (= the path's total length) no matter how the
    # samples are distributed, so the timing is DENSITY-INDEPENDENT. For the uniform paths
    # that ship today, cumulative chord length IS linspace to within the chord-vs-arc error,
    # so this is a no-op for them — and it is what lets a zone be sampled densely without
    # manufacturing curvature.
    ss = path_parameter(waypoints)
    # SplineInterpolator needs STRICTLY increasing knots. Duplicate samples (a zero-length
    # chord) otherwise raise deep inside toppra with an unhelpful message.
    dup = np.flatnonzero(np.diff(ss) <= 0.0)
    if dup.size:
        keep = np.ones(ss.shape[0], dtype=bool)
        keep[dup + 1] = False
        ss, waypoints = ss[keep], waypoints[keep]
        ss = ss.copy()
        ss[0], ss[-1] = 0.0, 1.0
    return ta.SplineInterpolator(ss, waypoints)


# NOTE (2026-07-11): a resample_uniform_arclength() helper used to live here, to
# even out the ||dq/ds|| swing on a composite path before a splice. It has been
# deleted: it resampled an already-sampled POLYLINE by linear interpolation, so
# the new points landed on the CHORDS rather than on the true curve, turning every
# original sample into a small kink. The spline then read those kinks as curvature
# (|q''| ~ 1/ds), so refining the sampling made the trajectory slower without
# limit — a blended corner measured 0.95 rad/s at N=100 and 0.24 rad/s at N=1600,
# never converging. The ||dq/ds|| swing it was meant to paper over is now fixed at
# the source: motion/joint_paths.py samples every piece from its ANALYTIC curve at
# a uniform arc-length spacing. Never reintroduce polyline resampling here.


def path_velocity_from_joint_velocity(waypoints: np.ndarray, joint_velocity: np.ndarray, s: float = 0.0) -> float:
    """Convert a JOINT velocity into TOPPRA's PATH velocity sd (ds/dt) at path
    coordinate `s` — the mapping the rolling buffer needs to splice a continuation
    onto an already-moving body (TODO/blending_design.md §6).

    Along a path q(s):   q̇ = (dq/ds) · sd

    sd is recovered by PROJECTING q̇ onto the tangent:

        sd = (q̇ · dq/ds) / ‖dq/ds‖²

    not by ‖q̇‖ / ‖dq/ds‖. The two agree exactly when q̇ is parallel to the tangent —
    which is the case on the joint lane, where the committed body and the continuation
    are the same straight line. They do NOT agree on the Cartesian lane: the
    continuation's tangent comes from a fresh IK solve, so it sits a hair off the
    direction the arm is actually travelling, and the norm form then feeds the FULL
    magnitude of q̇ into a slightly different direction — overshooting. Measured: the
    norm form left a 1.2e-3 .. 3.0e-3 rad/s velocity step at a Cartesian seam (enough to
    trip the splice's own exactness check); projection cuts it to the residual
    perpendicular component, which is all that is physically unavoidable.

    Returns 0.0 for a degenerate tangent (a stationary path).
    """
    tangent = np.asarray(build_geometric_path(waypoints)(float(s), 1), dtype=np.float64)
    tangent_sq = float(np.dot(tangent, tangent))
    if tangent_sq < 1e-18:
        return 0.0
    qd = np.asarray(joint_velocity, dtype=np.float64)
    return float(np.dot(qd, tangent) / tangent_sq)


# FALLBACK ONLY, since 2026-08-07. The robot XML now states EXEC's jerk budget outright
# (<exec_limits><jerk>), and that is what the planner passes in as `max_jerk`. This factor is
# used only when a caller supplies no jerk limit — i.e. a robot XML predating the field.
#
# It used to be the sole source, deriving jerk as `acceleration * 3.0` in consumer code. That
# is a limit computed from another limit, the exact pattern CLAUDE.md forbids, and the same
# shape as the old `MAX_VEL / ACCEL_TIME` bug: the XML gained a <joint_jerk_limit> on
# 2026-08-02 and this kept ignoring it, so EXEC ran at 28.8 rad/s^3 on joint 0 while the XML
# declared 16000 and nothing pointed at the disagreement.
#
# 3.0 means "reach full acceleration in 1/3 of a second".
RUCKIG_JERK_FACTOR = 3.0


class RuckigTimeParameterizer(TimeParameterizer):
    """Ruckig — jerk-limited (3rd-order) point-to-point timing.

    OPT-IN (`profile="ruckig"`); trapezoid remains the default. Genuinely better than
    trapezoid/quintic/s_curve for a plain move: those bound velocity and acceleration but
    let jerk be unbounded (trapezoid steps acceleration instantaneously), while Ruckig
    bounds all three and still lands the endpoint exactly. Measured on the Pi: 0.023 ms to
    solve a 6-DOF move, jerk held exactly at the limit, endpoint error 1e-16.

    POINT-TO-POINT ONLY — it can never be a zone-blend engine, and not because it is
    "online only" (that framing was wrong; it computes offline trajectories fine, which is
    what this class does). The reason is that Ruckig is TARGET-following, not PATH-following:
    hand it a Bezier corner and it discards your geometry and invents its own smooth route to
    the endpoint. Zone blending needs a path follower. That is TOPPRA's job and stays TOPPRA's
    job (blending_design.md 10).
    """

    supports_boundary_velocity = True
    # No fixed normalized shape: an exact_velocity blend corner (`blend` > 0) is sized by
    # compute_corner_speed with its default, the trapezoid's estimate. Without the attribute
    # every profile="ruckig" move with blend > 0 failed PLAN_FAIL (found 2026-10-11).
    profile = None

    def __init__(self, jerk_factor=RUCKIG_JERK_FACTOR, max_jerk=None):
        """max_jerk: per-joint rad/s^3 from <exec_limits><jerk>, ALREADY scaled by the
        command's `jerk` option (like max_accel, which arrives pre-scaled by `accel`).
        None falls back to `acceleration * jerk_factor` — the pre-2026-08-07 derivation —
        so a robot XML without the field behaves exactly as it always did."""
        self.jerk_factor = float(jerk_factor)
        self.max_jerk = None if max_jerk is None else np.asarray(max_jerk, dtype=np.float64)

    def parameterize(self, path, max_vel, max_accel, dt, speed_scale=1.0,
                     start_vel=None, end_vel=None):
        try:
            from ruckig import Ruckig, InputParameter, Trajectory, Result, Synchronization
        except Exception as exc:
            raise RuckigUnavailable(
                "profile='ruckig' requested but the ruckig package is not importable") from exc

        start = np.asarray(path.start, dtype=np.float64)
        target = np.asarray(path.target, dtype=np.float64)
        dof = int(start.shape[0])

        # Same time-scaling convention TOPPRA uses above: slowing time by `scale` scales
        # velocity by scale, acceleration by scale^2 — and therefore jerk by scale^3. Using
        # anything else here would make `speed` mean two different things depending on the
        # profile, which is exactly the kind of quiet inconsistency that costs a day later.
        scale = float(np.clip(speed_scale, 1e-3, 1.0))
        vmax = np.asarray(max_vel, dtype=np.float64) * scale
        amax = np.asarray(max_accel, dtype=np.float64) * scale * scale
        jmax = ((self.max_jerk if self.max_jerk is not None
                 else np.asarray(max_accel, dtype=np.float64) * self.jerk_factor)
                * scale ** 3)

        inp = InputParameter(dof)
        inp.current_position = start.tolist()
        inp.current_velocity = (np.zeros(dof) if start_vel is None
                                else np.asarray(start_vel, dtype=np.float64)).tolist()
        inp.current_acceleration = np.zeros(dof).tolist()
        inp.target_position = target.tolist()
        inp.target_velocity = (np.zeros(dof) if end_vel is None
                               else np.asarray(end_vel, dtype=np.float64)).tolist()
        inp.target_acceleration = np.zeros(dof).tolist()
        inp.max_velocity = vmax.tolist()
        inp.max_acceleration = amax.tolist()
        inp.max_jerk = jmax.tolist()
        # PHASE synchronization: every joint follows a scaled copy of one profile, so the move stays on the straight
        # start -> target line in joint space, as the trapezoid's does. Ruckig's default (time synchronization) only
        # makes the joints FINISH together: measured 2026-10-11, a 6-joint move bent up to 2.3 deg off that line
        # (2-joint moves 0.5-0.7) for the SAME duration, and the joints not setting the pace moved more abruptly (J1's
        # 6 Hz excitation in J1+J6: 0.15 -> 0.01 deg). Exact-velocity blend corners keep it too: their boundary
        # velocities lie along the line. Where phase sync is impossible Ruckig falls back to time sync itself.
        inp.synchronization = Synchronization.Phase

        traj = Trajectory(dof)
        result = Ruckig(dof).calculate(inp, traj)
        if result not in (Result.Working, Result.Finished):
            raise RuckigInfeasible(
                f"ruckig could not time this move (result={result}) — check that the "
                f"velocity/acceleration/jerk limits are all positive")

        duration = float(traj.duration)
        n = max(2, int(np.ceil(duration / float(dt))) + 1)
        pos = np.zeros((n, dof), dtype=np.float64)
        vel = np.zeros((n, dof), dtype=np.float64)
        for i in range(n):
            q, qd, _ = traj.at_time(min(i * float(dt), duration))
            pos[i] = q
            vel[i] = qd
        # Land the endpoint EXACTLY. dt rarely divides the duration, so the last sample is
        # taken at `duration` itself; without this the final row would sit a fraction of a
        # tick short and every move would end with a small standing position error.
        pos[-1] = target
        vel[-1] = np.zeros(dof) if end_vel is None else np.asarray(end_vel, dtype=np.float64)
        return TimedJointPath(positions=pos, velocities=vel, num_points=n)


class TOPPRATimeParameterizer(TimeParameterizer):
    """TOPPRA timing for joint paths, imported lazily so the planner has a fallback.

    parameterize() handles the 2-point joint-line path shape used by move_joints.
    parameterize_path() (TODO/blending_design.md §5.3/§9 phase 2) handles an
    arbitrary N-waypoint path — the dependency-backed lane for composite
    zone-blended chains. Both delegate to the same _solve() core.
    """

    supports_boundary_velocity = False

    def parameterize(self, path, max_vel, max_accel, dt, speed_scale=1.0,
                     start_vel=None, end_vel=None):
        waypoints = np.vstack((path.start, path.target))
        return self._solve(waypoints, max_vel, max_accel, dt, speed_scale)

    def parameterize_path(self, waypoints, max_vel, max_accel, dt, speed_scale=1.0,
                          sd_start=0.0, sd_end=0.0):
        """Time-parameterize an arbitrary N-waypoint joint-space path.

        Intended for composite zone-blended chains (motion/joint_paths.py,
        motion/cartesian_paths.py's build_composite_*_path output, IK-solved
        into joint space for the Cartesian case).

        sd_start / sd_end are TOPPRA *path* velocities (ds/dt), not joint
        velocities — the rolling buffer (TODO/blending_design.md §6) needs a
        continuation to begin at the speed the already-executing body ended at,
        instead of from rest. Use path_velocity_from_joint_velocity() to convert.
        Both default to 0 (rest-to-rest), which is every pre-phase-5 caller.

        Raises TOPPRAInfeasible if the requested boundary velocity is too fast for
        the path's own constraints — TOPPRA returns None, which is exactly the
        "can't splice this fast, commit the stopping tail instead" signal.
        """
        waypoints = np.asarray(waypoints, dtype=np.float64)
        if waypoints.shape[0] < 2:
            raise ValueError("TOPPRA path needs at least 2 waypoints")
        return self._solve(waypoints, max_vel, max_accel, dt, speed_scale,
                           sd_start=sd_start, sd_end=sd_end)

    def _solve(self, waypoints, max_vel, max_accel, dt, speed_scale,
               sd_start=0.0, sd_end=0.0):
        try:
            import toppra as ta
            from toppra import algorithm, constraint
        except Exception as exc:
            raise TOPPRAUnavailable("TOPPRA profile requested but toppra is not available") from exc

        # Everything from here down is TOPPRA doing real solving work — wrap it all,
        # not just the `traj is None` outcome. TOPPRA's own constraint construction
        # can raise plain ValueErrors for bad inputs (e.g. zero vel/accel limits),
        # and those need to become TOPPRAInfeasible too, not leak out as a plain
        # ValueError that trajectory_planner.py's blend-fallback would misread as
        # "blend infeasible, retry without blend" instead of a real TOPPRA failure.
        try:
            n_points = waypoints.shape[0]
            ss = np.linspace(0.0, 1.0, n_points)
            # A cubic spline through the path samples. The path's second derivative
            # q''(s) is what lets TOPPRA account for CENTRIPETAL acceleration
            # (qdd = q'(s)*sdd + q''(s)*sd^2) and slow down through curves. A
            # piecewise-linear path has q'' == 0 identically, which silently
            # disables that whole half of JointAccelerationConstraint — measured
            # 2026-07-10 at up to 103 rad/s^2 against a 2.46 rad/s^2 limit on a
            # blended rectangle. Keep this a spline.
            geom_path = build_geometric_path(waypoints)
            scale = float(np.clip(speed_scale, 1e-3, 1.0))
            constraints = [
                constraint.JointVelocityConstraint(np.asarray(max_vel, dtype=np.float64) * scale),
                constraint.JointAccelerationConstraint(np.asarray(max_accel, dtype=np.float64) * scale * scale),
            ]
            # Evenly-spaced explicit gridpoints, not TOPPRA's auto-selection —
            # auto-selection can cluster near discontinuities and mis-time the
            # result.
            # Floored so short (2-point) paths keep the density auto would pick.
            gridpoints = geometric_gridpoints(waypoints, min_points=max(n_points * 3, 100))
            instance = algorithm.TOPPRA(constraints, geom_path, gridpoints=gridpoints, solver_wrapper="seidel")
            # sd_start/sd_end are PATH velocities (ds/dt). Nonzero sd_start is what
            # lets a rolling-buffer continuation pick up at the speed the executing
            # body ended at instead of from rest (§6). Verified exact:
            # q̇(0) == (dq/ds)|_0 * sd_start. TOPPRA returns None if the boundary
            # velocity is infeasible for this path's constraints — that becomes
            # TOPPRAInfeasible below, i.e. "commit the stopping tail instead".
            traj = instance.compute_trajectory(float(sd_start), float(sd_end))
            if traj is None:
                raise TOPPRAInfeasible("TOPPRA could not compute a valid trajectory")

            # REFINE THE GRID TO THE OUTPUT'S OWN SAMPLE RATE.
            #
            # TOPPRA enforces its limits AT THE GRIDPOINTS. The trajectory it hands back is a
            # spline through them, and between gridpoints that spline BULGES — so the samples
            # we actually send to RTI can exceed the acceleration limit even though the solve
            # was perfectly legal. Measured on a plain move: +5.4% over the limit at full
            # accel, and +18.4% when the limit was scaled down (a lower limit = a longer, more
            # finely-sampled move = more gridpoint gaps to bulge through).
            #
            # It is pure discretization: at 600 gridpoints the same move came out at +0.0%.
            # The rule that makes it vanish is simply "at least one gridpoint per output
            # sample" — the spline cannot bulge between points that are one tick apart.
            #
            # We cannot know the duration before solving, so: solve once cheaply, read the
            # duration, and re-solve on a grid sized to it. One extra solve (~10-30ms), only
            # when the first grid was too coarse. This is what makes the ACCELERATION LIMIT
            # actually a limit rather than a suggestion.
            # VERIFY THE SAMPLED OUTPUT, DO NOT TRUST A PROXY FOR IT.
            #
            # TOPPRA enforces its limits AT THE GRIDPOINTS. What it returns is a spline through
            # them, and between gridpoints that spline BULGES — so the samples we actually send
            # to RTI can exceed the acceleration limit even though the solve was legal.
            #
            # The first fix here used a COUNT criterion ("at least one gridpoint per output
            # sample"). It cured plain point-to-point (which lands exactly on the limit) but NOT
            # blended chains, which still ran 2.5-6.2% over. Why: the gridpoints follow the
            # path's chord-length knots, so they cluster in the CORNERS and thin out on the
            # LEGS. The global count passes while the legs still have gaps to bulge through. A
            # proxy for the property is not the property.
            #
            # So check the thing we actually care about — the acceleration of the SAMPLES we
            # will send — and refine until it holds. Bounded at 3 tries; each doubles the grid.
            # REFINE THE GRID UNTIL THE SAMPLED OUTPUT IS LEGAL. No fudge factors.
            #
            # TOPPRA enforces its limits AT THE GRIDPOINTS. What it returns is a spline through
            # them, and BETWEEN gridpoints that spline BULGES — so the samples we actually send
            # to RTI can exceed the acceleration limit even though the solve was legal
            # (measured +5.4% on a plain move, +2.5-6.2% on blended chains). It is pure
            # DISCRETIZATION, so we discretize until it holds. We check the property we care
            # about — the acceleration of the samples we will actually SEND — not a proxy.
            #
            # DO NOT "FIX" THIS BY DERATING THE LIMIT. That is cheaper (one solve) and it works,
            # but the derate factor DEPENDS ON THE SAMPLING, and it destroyed two invariants
            # everything else rests on: density independence (a corner's timing must not depend
            # on how finely we sample it — blending_design.md 9.4; measured 7.3% drift) and
            # duration ∝ 1/speed_scale (what makes `duration` land on t and lets every profile
            # time-scale uniformly; duration=3.0s came out at 3.20s). A data-dependent fudge
            # factor has no place in the timing law.
            #
            # This costs real time — a 100-command splice goes ~72ms -> ~318ms — and
            # SPLICE_SAFETY_MARGIN_S is sized to pay for it. That is the right trade: an arm
            # that stops between two SHORT moves when the command source is slow is a nuisance;
            # an arm that quietly exceeds its acceleration limit is a defect.
            a_lim = np.asarray(max_accel, dtype=np.float64) * scale * scale

            def _worst_sampled_accel(tr):
                d = float(tr.duration)
                if d <= 0.0:
                    return 0.0
                nn = max(3, int(round(d / float(dt))) + 1)
                vv = np.asarray(tr(np.linspace(0.0, d, nn), 1), dtype=np.float64)
                aa = np.diff(vv, axis=0) / float(dt)
                return float(np.max(np.abs(aa) / (a_lim[None, :] + 1e-12)))

            for _ in range(4):
                if _worst_sampled_accel(traj) <= 1.02:
                    break
                nxt = geometric_gridpoints(
                    waypoints,
                    min_points=min(max(gridpoints.size * 2,
                                       int(np.ceil(float(traj.duration) / float(dt))) + 1),
                                   8000))
                if nxt.size <= gridpoints.size:
                    break
                gridpoints = nxt
                inst2 = algorithm.TOPPRA(constraints, geom_path, gridpoints=gridpoints,
                                         solver_wrapper="seidel")
                traj2 = inst2.compute_trajectory(float(sd_start), float(sd_end))
                if traj2 is None:
                    break
                traj = traj2
        except TOPPRAInfeasible:
            raise
        except Exception as exc:
            raise TOPPRAInfeasible(f"TOPPRA failed to compute a trajectory: {exc}") from exc

        duration = float(traj.duration)
        if duration <= 0.0:
            pos = np.repeat(waypoints[:1], 2, axis=0)
            return TimedJointPath(positions=pos, velocities=np.zeros_like(pos), num_points=2)
        n = max(2, int(round(duration / dt)) + 1)
        ts = np.linspace(0.0, duration, n)
        pos = np.asarray(traj(ts), dtype=np.float64)
        vel = np.asarray(traj(ts, 1), dtype=np.float64)
        return TimedJointPath(positions=pos, velocities=vel, num_points=n)


def make_time_parameterizer(profile_name: str, ramp_fraction: float | None = None, jerk_scale: float | None = None,
                            max_jerk: np.ndarray | None = None) -> TimeParameterizer:
    profile = normalize_profile_name(profile_name or "trapezoid")
    if jerk_scale is not None and profile != "ruckig":
        # Ruckig is the ONLY profile that can honour a jerk limit. trapezoid and toppra STEP
        # acceleration (jerk bounded only by the tick); quintic and s_curve are fixed
        # polynomial shapes whose jerk falls out of the duration and cannot be dialled.
        # Silently ignoring `jerk` on them is the bug class that hid blend_continuity, accel,
        # duration and ramp_fraction. Say no instead.
        raise ValueError(
            f"jerk only applies to profile='ruckig' — it is the only jerk-limited "
            f"parameterizer. profile={profile!r} does not bound jerk at all "
            f"(trapezoid/toppra step acceleration; quintic/s_curve have a fixed shape).")
    if profile == "ruckig":
        factor = RUCKIG_JERK_FACTOR if jerk_scale is None else RUCKIG_JERK_FACTOR * jerk_scale
        # `jerk` stays a PERCENTAGE of the budget whichever source supplies it, so the option
        # means the same thing before and after the XML gained <exec_limits><jerk>: jerk=100
        # is the full budget, jerk=25 is a quarter of it. Only the budget's origin changed.
        scaled = (None if max_jerk is None
                  else np.asarray(max_jerk, dtype=np.float64)
                  * (1.0 if jerk_scale is None else float(jerk_scale)))
        return RuckigTimeParameterizer(jerk_factor=factor, max_jerk=scaled)
    if profile == "trapezoid":
        return (TrapezoidTimeParameterizer() if ramp_fraction is None
                else TrapezoidTimeParameterizer(ramp_fraction))
    if ramp_fraction is not None:
        # Only the trapezoid HAS a ramp/cruise split. Silently ignoring it on a quintic or a
        # TOPPRA solve is the exact bug class that hid blend_continuity, accel and duration.
        raise ValueError(
            f"ramp_fraction only applies to profile='trapezoid' (it is the LSPB blend "
            f"fraction); profile={profile!r} has no accel/cruise/decel split")
    if profile == "quintic":
        return QuinticTimeParameterizer()
    if profile == "s_curve":
        return SCurveTimeParameterizer()
    if profile == "toppra":
        return TOPPRATimeParameterizer()
    raise ValueError(f"unsupported motion profile '{profile_name}'")


class ExecBatchSampler:
    """Convert a timed joint path to the arrays expected by pack_batch()."""

    def __init__(self, joint_num):
        self.joint_num = int(joint_num)

    def sample(self, timed_path):
        pos = np.ascontiguousarray(timed_path.positions, dtype=np.float64)
        vel = np.ascontiguousarray(timed_path.velocities, dtype=np.float64)
        if pos.ndim != 2 or pos.shape != vel.shape or pos.shape[1] != self.joint_num:
            raise ValueError(
                f"sample shape mismatch: pos={pos.shape}, vel={vel.shape}, "
                f"joint_num={self.joint_num}")
        torque_ff = np.zeros(pos.shape, dtype=np.float32)
        return ExecTrajectorySample(positions=pos, velocities=vel, torque_ff=torque_ff)
