"""Cartesian path geometry helpers for EXEC planning.

This module is deliberately geometry-only: it creates sampled SE3 paths, but it does
not solve IK, time-parameterize, or send anything to RTI.  The planner can use these
samples for dry-run validation first and later for real Cartesian execution.
"""

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation, Slerp
from spatialmath import SE3


@dataclass(frozen=True)
class CartesianPath:
    """Sampled Cartesian path before IK conversion."""

    poses: tuple
    s: np.ndarray
    translation_length: float
    rotation_angle: float

    @property
    def num_samples(self):
        return len(self.poses)


def pose_from_xyz_rpy(args, label="pose"):
    """Build SE3 from [x, y, z, rx, ry, rz], using meters and radians."""
    if len(args) < 6:
        raise ValueError(f"{label} needs [x, y, z, rx, ry, rz], got {len(args)} values")
    x, y, z, rx, ry, rz = [float(v) for v in args[:6]]
    return SE3(x, y, z) * SE3.RPY([rx, ry, rz], order="xyz")


def _rotation_angle(start_pose, target_pose):
    rel = start_pose.A[:3, :3].T @ target_pose.A[:3, :3]
    trace = float(np.trace(rel))
    c = np.clip((trace - 1.0) * 0.5, -1.0, 1.0)
    return float(np.arccos(c))


def sample_count_for_line(start_pose, target_pose, max_translation_step=0.01,
                          max_rotation_step=np.deg2rad(2.0), max_samples=500):
    """Choose a bounded diagnostic sample count for a Cartesian line.

    Defaults are intentionally modest: this is a dry-run path validator, not the final
    1 kHz hardware path generator.  Later G6 chunked execution can use denser timing.
    """
    translation = float(np.linalg.norm(target_pose.t - start_pose.t))
    rotation = _rotation_angle(start_pose, target_pose)
    by_translation = int(np.ceil(translation / max(float(max_translation_step), 1e-9))) + 1
    by_rotation = int(np.ceil(rotation / max(float(max_rotation_step), 1e-9))) + 1
    return int(np.clip(max(2, by_translation, by_rotation), 2, int(max_samples)))


def cartesian_line(start_pose, target_pose, samples=None, max_translation_step=0.01,
                   max_rotation_step=np.deg2rad(2.0), max_samples=500):
    """Sample a straight Cartesian line with spherical orientation interpolation."""
    if samples is None:
        samples = sample_count_for_line(
            start_pose, target_pose, max_translation_step=max_translation_step,
            max_rotation_step=max_rotation_step, max_samples=max_samples)
    samples = int(samples)
    if samples < 2:
        raise ValueError("Cartesian line needs at least 2 samples")

    poses = _linear_segment_poses(start_pose, target_pose, samples)
    return CartesianPath(
        poses=tuple(poses),
        s=np.linspace(0.0, 1.0, samples),
        translation_length=float(np.linalg.norm(target_pose.t - start_pose.t)),
        rotation_angle=_rotation_angle(start_pose, target_pose),
    )


# ---------------------------------------------------------------------------
# Composite path + Bezier zone blending (TODO/blending_design.md §4)
#
# Pose representation is spatialmath SE3 + scipy Slerp rather than pinokin's
# batch_se3_interp/se3_interp: the planner already carries SE3 everywhere else, and one
# representation end to end is worth more here than the faster batch path.
# Planning-time code only (runs once per queued
# command in the planner thread) — plain Python lists, not pre-allocated
# buffers; the hot-path allocation rules in CLAUDE.md don't apply here.
# ---------------------------------------------------------------------------

def _pose_from_R_t(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return SE3(T, check=False)


def _linear_segment_poses(start_pose, end_pose, samples, s_start=0.0, s_end=1.0, skip=0):
    """Sample a linear Cartesian segment over sub-range [s_start, s_end]."""
    n_total = samples + skip
    s = np.linspace(s_start, s_end, n_total)[skip:]
    translations = (1.0 - s)[:, None] * start_pose.t + s[:, None] * end_pose.t
    rotations = Rotation.from_matrix([start_pose.A[:3, :3], end_pose.A[:3, :3]])
    interp_rotations = Slerp([0.0, 1.0], rotations)(s).as_matrix()
    return [_pose_from_R_t(R, t) for R, t in zip(interp_rotations, translations)]


def interp_pose(start_pose, end_pose, s):
    """Single-point SE3 interpolation at fraction s — used for blend-zone endpoints."""
    return _linear_segment_poses(start_pose, end_pose, 1, s, s)[0]

# ---------------------------------------------------------------------------
# Standalone curved Cartesian paths. These deliberately return the same
# CartesianPath used by move_linear, so curve commands reuse the existing seeded
# IK, profile timing, joint-limit, and FK path-fidelity machinery.
# ---------------------------------------------------------------------------

def _curve_path(poses):
    """Build CartesianPath metadata from densely sampled curve poses."""
    poses = tuple(poses)
    if len(poses) < 2:
        raise ValueError("Cartesian curve needs at least 2 samples")
    translation_steps = np.array([
        np.linalg.norm(poses[i + 1].t - poses[i].t)
        for i in range(len(poses) - 1)
    ], dtype=np.float64)
    rotation_steps = np.array([
        _rotation_angle(poses[i], poses[i + 1])
        for i in range(len(poses) - 1)
    ], dtype=np.float64)
    metric_steps = _pose_metric_step(translation_steps, rotation_steps)
    metric = np.concatenate(([0.0], np.cumsum(metric_steps)))
    if metric[-1] <= 1e-12:
        s = np.linspace(0.0, 1.0, len(poses))
    else:
        s = metric / metric[-1]
    return CartesianPath(
        poses=poses,
        s=s,
        translation_length=float(translation_steps.sum()),
        rotation_angle=float(rotation_steps.sum()),
    )


def _curve_sample_count(translation_length, rotation_angle, samples, minimum):
    """Resolve explicit curve samples or a safe uniform path-spacing density."""
    if samples is not None:
        return max(int(minimum), int(samples))
    metric_length = float(np.hypot(
        float(translation_length), ROTATION_RADIUS_M * float(rotation_angle)))
    return int(np.clip(
        np.ceil(metric_length / DEFAULT_ARCLEN_SPACING_M) + 1,
        minimum,
        MAX_PATH_SAMPLES,
    ))


def _circle_from_3_points(start_xyz, via_xyz, end_xyz):
    """Return the unique circle center, radius, and oriented normal through 3 points."""
    start_xyz = np.asarray(start_xyz, dtype=np.float64)
    via_xyz = np.asarray(via_xyz, dtype=np.float64)
    end_xyz = np.asarray(end_xyz, dtype=np.float64)
    a = via_xyz - start_xyz
    b = end_xyz - start_xyz
    normal = np.cross(a, b)
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm < 1e-12:
        raise ValueError("arc start, via, and end positions are collinear or coincident")
    normal /= normal_norm

    aa = float(np.dot(a, a))
    bb = float(np.dot(b, b))
    ab = float(np.dot(a, b))
    determinant = aa * bb - ab * ab
    if abs(determinant) < 1e-20:
        raise ValueError("arc circle is degenerate")
    s = (bb * aa - ab * bb) / (2.0 * determinant)
    t = (aa * bb - ab * aa) / (2.0 * determinant)
    center = start_xyz + s * a + t * b
    radius = float(np.linalg.norm(start_xyz - center))
    if radius < 1e-9:
        raise ValueError("arc radius is too small")
    return center, radius, normal


def _positive_angle(start_vector, end_vector, normal):
    """Angle from start to end in [0, 2*pi), viewed along normal."""
    angle = float(np.arctan2(
        np.dot(normal, np.cross(start_vector, end_vector)),
        np.dot(start_vector, end_vector),
    ))
    return angle if angle >= 0.0 else angle + 2.0 * np.pi


def cartesian_arc_3point(start_pose, via_pose, end_pose, samples=None):
    """Sample the unique circular arc passing through start, via, and end.

    Translation follows the arc. Orientation uses SLERP from start to end; the
    via pose contributes its POSITION only — its orientation is ignored. A three-point
    arc is defined by position alone, so honouring the via orientation as well would
    over-constrain the interpolation and make the result depend on how the caller happened
    to orient a waypoint it only meant as a location.
    """
    center, radius, normal = _circle_from_3_points(
        start_pose.t, via_pose.t, end_pose.t)
    radial_start = (start_pose.t - center) / radius
    radial_via = (via_pose.t - center) / radius
    radial_end = (end_pose.t - center) / radius
    via_angle = _positive_angle(radial_start, radial_via, normal)
    end_angle = _positive_angle(radial_start, radial_end, normal)
    if via_angle <= 1e-10 or end_angle <= 1e-10:
        raise ValueError("arc via and end positions must differ from the start")
    if via_angle >= end_angle:
        end_angle += 2.0 * np.pi
    if via_angle >= end_angle - 1e-10:
        raise ValueError("arc sweep cannot pass through the requested via position")

    count = _curve_sample_count(
        radius * end_angle, _rotation_angle(start_pose, end_pose), samples, minimum=3)
    fractions = np.linspace(0.0, 1.0, count)
    rotations = Rotation.from_rotvec(np.outer(fractions * end_angle, normal))
    positions = center + rotations.apply(start_pose.t - center)
    orient = _slerp_matrices(start_pose, end_pose, fractions)
    return _curve_path([_pose_from_R_t(R, p) for R, p in zip(orient, positions)])


def cartesian_circle(start_pose, center, normal, turns=1.0, samples=None):
    """Sample a full circular TCP path around an explicit center and plane normal.

    The TCP orientation stays fixed. A negative turn count reverses direction.
    """
    center = np.asarray(center, dtype=np.float64)
    normal = np.asarray(normal, dtype=np.float64)
    if center.shape != (3,) or normal.shape != (3,):
        raise ValueError("circle center and normal each need exactly 3 values")
    normal_norm = float(np.linalg.norm(normal))
    if normal_norm < 1e-12:
        raise ValueError("circle normal must be non-zero")
    normal = normal / normal_norm
    radial = start_pose.t - center
    plane_offset = float(np.dot(radial, normal))
    # The client can build a center from a TCP query while the planner starts from
    # a fresh measured joint state a few milliseconds later. Snap only this tiny
    # measurement mismatch onto the true start plane; reject a real bad geometry.
    plane_snap_tolerance = 2e-4
    if abs(plane_offset) > plane_snap_tolerance:
        raise ValueError(
            "circle center must lie in the plane through the start position "
            f"(normal offset {plane_offset * 1000.0:.3f} mm)")
    if abs(plane_offset) > 1e-12:
        center = center + plane_offset * normal
        radial = start_pose.t - center
    radius = float(np.linalg.norm(radial))
    if radius < 1e-9:
        raise ValueError("circle start position must differ from center")
    turns = float(turns)
    if not np.isfinite(turns) or abs(turns) < 1e-12:
        raise ValueError("circle turns must be finite and non-zero")

    total_angle = 2.0 * np.pi * turns
    count = _curve_sample_count(radius * abs(total_angle), 0.0, samples, minimum=4)
    fractions = np.linspace(0.0, 1.0, count)
    rotations = Rotation.from_rotvec(np.outer(fractions * total_angle, normal))
    positions = center + rotations.apply(radial)
    fixed_R = start_pose.A[:3, :3]
    return _curve_path([_pose_from_R_t(fixed_R, p) for p in positions])


def cartesian_spline(start_pose, waypoint_poses, samples=None):
    """Sample an interpolating cubic position spline through Cartesian waypoints.

    Positions use a chord-length parameterization and pass through every waypoint.
    Orientations use SLERP over the same parameter. Dense samples are subsequently
    IK-validated, so an overshooting or unreachable spline is rejected before RTI.
    """
    poses = [start_pose, *tuple(waypoint_poses)]
    if len(poses) < 3:
        raise ValueError("spline needs at least two waypoint poses after the start")

    kept = [poses[0]]
    for pose in poses[1:]:
        translation = float(np.linalg.norm(pose.t - kept[-1].t))
        rotation = _rotation_angle(kept[-1], pose)
        if np.hypot(translation, ROTATION_RADIUS_M * rotation) > 1e-10:
            kept.append(pose)
    if len(kept) < 3:
        raise ValueError("spline needs at least two distinct waypoint poses")

    translations = np.array([pose.t for pose in kept], dtype=np.float64)
    key_rotations = Rotation.from_matrix([pose.A[:3, :3] for pose in kept])
    translation_steps = np.linalg.norm(np.diff(translations, axis=0), axis=1)
    rotation_steps = np.array([
        _rotation_angle(kept[i], kept[i + 1]) for i in range(len(kept) - 1)
    ], dtype=np.float64)
    parameter_steps = _pose_metric_step(translation_steps, rotation_steps)
    parameter = np.concatenate(([0.0], np.cumsum(parameter_steps)))
    count = _curve_sample_count(
        float(translation_steps.sum()), float(rotation_steps.sum()), samples, minimum=3)
    position_spline = CubicSpline(parameter, translations, axis=0, bc_type="natural")
    orientation_spline = Slerp(parameter, key_rotations)

    # Sample a dense curve, invert its metric arc length, and retain every key
    # parameter. This keeps TOPPRA's geometric parameterization smooth while
    # guaranteeing an interpolating spline really includes its programmed poses.
    dense_count = max(512, count * 8, len(parameter) * 8)
    parameter_dense = np.unique(np.concatenate((
        np.linspace(0.0, parameter[-1], dense_count), parameter)))
    positions_dense = np.asarray(position_spline(parameter_dense), dtype=np.float64)
    orientations_dense = orientation_spline(parameter_dense).as_matrix()
    dense_translation = np.linalg.norm(np.diff(positions_dense, axis=0), axis=1)
    dense_rotation = np.array([
        _rotation_angle(_pose_from_R_t(orientations_dense[i], positions_dense[i]),
                        _pose_from_R_t(orientations_dense[i + 1], positions_dense[i + 1]))
        for i in range(len(parameter_dense) - 1)
    ], dtype=np.float64)
    metric_dense = np.concatenate((
        [0.0], np.cumsum(_pose_metric_step(dense_translation, dense_rotation))))
    metric_targets = np.linspace(0.0, metric_dense[-1], count)
    key_metric = np.interp(parameter, parameter_dense, metric_dense)
    metric_targets = np.unique(np.concatenate((metric_targets, key_metric)))
    parameter_eval = np.interp(metric_targets, metric_dense, parameter_dense)
    positions = np.asarray(position_spline(parameter_eval), dtype=np.float64)
    orient = orientation_spline(parameter_eval).as_matrix()
    return _curve_path([_pose_from_R_t(R, p) for R, p in zip(orient, positions)])


# ---------------------------------------------------------------------------
# Uniform-arc-length sampling — see motion/joint_paths.py's module docstring for
# the full derivation. Short version: TOPPRA reads its centripetal acceleration
# limit off q''(s) of a spline fitted through these samples (after IK). If the
# samples are spaced by *index* rather than by path length, |dq/ds| jumps at
# every line->Bezier junction whenever the two blend fractions differ — which is
# whenever the adjacent segments have different lengths, i.e. almost always on a
# real toolpath. The spline reads that parameterization jump as curvature and
# the planner brakes for a corner that isn't there, by up to 27%. Sampling every
# piece from its analytic curve at one uniform spacing removes it.
#
# "Path length" for a pose has to fold in rotation. We use a Euclidean blend of
# translation and rotation, with ROTATION_RADIUS_M as the lever arm that converts
# radians to metres — i.e. 1 rad of TCP rotation costs the same path length as
# moving ROTATION_RADIUS_M in translation. Its exact value is not critical; it
# only sets the relative weighting. What correctness depends on is that ONE
# metric is used uniformly across every piece of the path.
# ---------------------------------------------------------------------------

ROTATION_RADIUS_M = 0.15

# 2 mm of TCP path per sample. Chosen so that no move gets COARSER than under the
# old index-based sampling, which gave ~1.6 mm on a short segment but ~4 mm on a
# long one (that variation was the bug). Cartesian samples each cost an IK solve
# and seeded IK tracks better when consecutive targets are close, so erring fine
# is the safe direction. Timing is converged well before this (<1% drift).
DEFAULT_ARCLEN_SPACING_M = 0.002
MAX_PATH_SAMPLES = 3000
_BEZIER_QUADRATURE = 512


def _pose_metric_step(translations, rotation_angles):
    """Per-step path length combining translation (m) and rotation (rad)."""
    return np.sqrt(translations ** 2 + (ROTATION_RADIUS_M * rotation_angles) ** 2)


def _slerp_matrices(start_pose, end_pose, t):
    rotations = Rotation.from_matrix([start_pose.A[:3, :3], end_pose.A[:3, :3]])
    return Slerp([0.0, 1.0], rotations)(t).as_matrix()


def _bezier_positions(entry_pose, waypoint_pose, exit_pose, t, continuity="c1"):
    """The corner curve, evaluated analytically at arbitrary t.

    "c1" (default, what shipped): quadratic Bezier — tangent-continuous, but CURVATURE
    STEPS at both ends (a straight leg has zero curvature; this curve enters at a finite
    nonzero one). Curvature drives centripetal acceleration, so that step is an instant
    acceleration change — infinite jerk twice per corner, in the GEOMETRY, unfixable by any
    profile.

    "c2": quintic Bezier with P0=E, P1=(V+E)/2, P2=V, P3=V, P4=(V+X)/2, P5=X, giving
    B''(0)=B''(1)=0 exactly, so curvature starts and ends at zero and matches the legs.
    Costs corner speed: peak curvature is higher (4.34 vs 1.41 for the same d), so TOPPRA
    brakes harder at the apex. See motion/joint_paths.py::_bezier for the full derivation.
    """
    if continuity == "c2":
        E, V, X = entry_pose.t, waypoint_pose.t, exit_pose.t
        p1, p4 = 0.5 * (V + E), 0.5 * (V + X)
        u = 1.0 - t
        return (np.outer(u**5, E) + np.outer(5.0 * u**4 * t, p1)
                + np.outer(10.0 * u**3 * t**2, V) + np.outer(10.0 * u**2 * t**3, V)
                + np.outer(5.0 * u * t**4, p4) + np.outer(t**5, X))
    omt = 1.0 - t
    return (np.outer(omt * omt, entry_pose.t) + np.outer(2.0 * omt * t, waypoint_pose.t)
            + np.outer(t * t, exit_pose.t))


def blend_zone_poses(entry_pose, waypoint_pose, exit_pose, spacing, continuity="c1"):
    """Sample a quadratic-Bezier corner-blend zone at uniform path length.

    Position follows P(t) = (1-t)^2*E + 2t(1-t)*W + t^2*X — tangent to the
    incoming segment at t=0, tangent to the outgoing segment at t=1, so the
    corner is rounded with C1 (velocity) continuity rather than passed through
    exactly. Orientation is geodesic (SLERP) from entry to exit.

    The Bezier's speed |dP/dt| varies along t, so uniform t is NOT uniform path
    length. We walk a fine quadrature grid on the true curve, accumulate path
    length, then invert it to find the t values that sit at equal steps.
    """
    t_fine = np.linspace(0.0, 1.0, _BEZIER_QUADRATURE)
    p_fine = _bezier_positions(entry_pose, waypoint_pose, exit_pose, t_fine, continuity)
    total_rot = _rotation_angle(entry_pose, exit_pose)

    step_t = np.linalg.norm(np.diff(p_fine, axis=0), axis=1)
    step_r = np.full(step_t.shape, total_rot / (len(t_fine) - 1))   # SLERP is constant-rate in t
    cum = np.concatenate(([0.0], np.cumsum(_pose_metric_step(step_t, step_r))))
    total = float(cum[-1])
    if total < 1e-12:
        return [entry_pose], 0.0

    n = max(3, int(round(total / spacing)) + 1)
    t_at = np.interp(np.linspace(0.0, total, n), cum, t_fine)

    positions = _bezier_positions(entry_pose, waypoint_pose, exit_pose, t_at, continuity)
    rotations = _slerp_matrices(entry_pose, exit_pose, t_at)
    return [_pose_from_R_t(R, p) for R, p in zip(rotations, positions)], total


def linear_segment_arclen(start_pose, end_pose, spacing, s_start=0.0, s_end=1.0):
    """Sample a (possibly trimmed) straight Cartesian segment at uniform path length.

    Both the position lerp and the orientation SLERP are constant-rate in s, so
    the path-length metric is constant-rate too and uniform s IS uniform path
    length here — no inversion needed.
    """
    span = float(s_end - s_start)
    full_t = float(np.linalg.norm(end_pose.t - start_pose.t))
    full_r = _rotation_angle(start_pose, end_pose)
    total = span * float(_pose_metric_step(np.array([full_t]), np.array([full_r]))[0])
    if total < 1e-12:
        return [interp_pose(start_pose, end_pose, s_start)], 0.0

    n = max(2, int(round(total / spacing)) + 1)
    return _linear_segment_poses(start_pose, end_pose, n, s_start, s_end), total


def _capped_spacing(spacing, waypoints):
    """Grow the spacing if this path would otherwise exceed MAX_PATH_SAMPLES.

    Uses the straight-line waypoint polygon (a cheap upper bound on the true
    path length), so a very long chain degrades to a coarser — but still
    perfectly UNIFORM — path rather than an enormous one. Every Cartesian sample
    costs an IK solve, so this bound is about planning time, not correctness:
    uniformity is what correctness depends on, not the absolute spacing.
    """
    if len(waypoints) < 2:
        return spacing
    approx = 0.0
    for i in range(len(waypoints) - 1):
        step_t = float(np.linalg.norm(waypoints[i + 1].t - waypoints[i].t))
        step_r = _rotation_angle(waypoints[i], waypoints[i + 1])
        approx += float(_pose_metric_step(np.array([step_t]), np.array([step_r]))[0])
    if approx <= 0.0:
        return spacing
    return max(spacing, approx / MAX_PATH_SAMPLES)


def build_composite_cartesian_path(waypoints, blend_radii_mm, arclen_spacing=None,
                                   samples_per_segment=None, start_trim=0.0,
                                   continuity="c1", return_blend_zones=False):
    """Build a composite Cartesian path with Bezier blend zones at intermediate waypoints.

    Concatenates linear Cartesian segments connected by quadratic-Bezier blend
    zones. Zone-overlap clamping: if two adjacent blend zones would
    overlap, both radii are proportionally reduced so neither exceeds half its
    adjacent segment's length.

    Every piece — straight segments and blend zones alike — is sampled from its
    analytic curve at the SAME path-length spacing, so the parameterization has
    no jump at the junctions. See the comment block above for why.

    Args:
        waypoints: List of SE3 poses defining the path corners (>= 2).
        blend_radii_mm: Blend radius in mm for each intermediate waypoint.
            Length must be len(waypoints) - 2. 0 means stop at that waypoint
            (no blending there).
        arclen_spacing: Target spacing between samples, as a path length in
            metres (rotation folded in via ROTATION_RADIUS_M). Defaults to
            DEFAULT_ARCLEN_SPACING_M.
        samples_per_segment: Deprecated and ignored — sample density is now set
            by arclen_spacing. Accepted so existing callers keep working.
        start_trim: Fraction of the FIRST segment to skip. The rolling buffer needs this:
            a batch spliced into a move already running starts at the EXIT of the Bezier
            rounding the corner into it — part-way along the first segment, not at the
            waypoint. Without it the builder treats that exit as a fresh waypoint and
            recomputes every blend radius against a shortened segment, relocating corners.

    Returns:
        (CartesianPath, boundaries):
          - CartesianPath — poses form the complete blended path.
          - boundaries: list of length N-1 (one per non-start waypoint / one
            per original queued command); boundaries[i] is the index into
            CartesianPath.poses where waypoint i+1 has been fully handed off
            (end of its incoming segment plus any trailing blend zone that
            rounds it) — where a per-command checkpoint should fire when
            sending a blended chain. See TODO/blending_design.md §5.4.
            boundaries[-1] always equals len(poses).

    Raises:
        ValueError: if inputs are inconsistent.
    """
    n = len(waypoints)
    if n < 2:
        raise ValueError("Need at least 2 waypoints")
    if len(blend_radii_mm) != max(0, n - 2):
        raise ValueError(f"Expected {max(0, n - 2)} blend radii, got {len(blend_radii_mm)}")

    spacing = (DEFAULT_ARCLEN_SPACING_M if arclen_spacing is None
               else float(arclen_spacing))
    if not spacing > 0.0:
        raise ValueError(f"arclen_spacing must be positive, got {spacing}")
    spacing = _capped_spacing(spacing, waypoints)

    trim0 = float(np.clip(start_trim, 0.0, 0.95))
    if n == 2:
        poses, _ = linear_segment_arclen(waypoints[0], waypoints[1], spacing,
                                         s_start=trim0)
        path = CartesianPath(
            poses=tuple(poses), s=np.linspace(0.0, 1.0, len(poses)),
            translation_length=float(np.linalg.norm(waypoints[1].t - waypoints[0].t)),
            rotation_angle=_rotation_angle(waypoints[0], waypoints[1]))
        return (path, [len(poses)], ()) if return_blend_zones else (path, [len(poses)])

    seg_lengths_mm = [
        float(np.linalg.norm(waypoints[i + 1].t - waypoints[i].t)) * 1000.0
        for i in range(n - 1)
    ]

    # Clamp blend radii (zone overlap prevention) — never eat more than half
    # of either adjacent segment.
    clamped = list(blend_radii_mm)
    for i in range(len(clamped)):
        clamped[i] = min(clamped[i], seg_lengths_mm[i] / 2.0, seg_lengths_mm[i + 1] / 2.0)
    # Adjacent zones that would themselves overlap get scaled down together.
    for i in range(len(clamped) - 1):
        seg_len = seg_lengths_mm[i + 1]
        total = clamped[i] + clamped[i + 1]
        if total > seg_len and total > 0:
            scale = seg_len / total
            clamped[i] *= scale
            clamped[i + 1] *= scale

    seg_exit_frac = [0.0] * (n - 1)
    seg_entry_frac = [0.0] * (n - 1)
    seg_entry_frac[0] = trim0
    for i in range(len(clamped)):
        if clamped[i] > 0:
            if seg_lengths_mm[i] > 0:
                seg_exit_frac[i] = clamped[i] / seg_lengths_mm[i]
            if seg_lengths_mm[i + 1] > 0:
                seg_entry_frac[i + 1] = clamped[i] / seg_lengths_mm[i + 1]

    poses = []
    boundaries = []
    blend_zones = []
    for seg_idx in range(n - 1):
        start = waypoints[seg_idx]
        end = waypoints[seg_idx + 1]
        s_start = seg_entry_frac[seg_idx]
        s_end = 1.0 - seg_exit_frac[seg_idx]

        if s_end > s_start + 1e-9:
            seg_poses, _ = linear_segment_arclen(start, end, spacing, s_start, s_end)
            poses.extend(seg_poses[1:] if poses else seg_poses)

        if seg_idx < len(clamped) and clamped[seg_idx] > 0:
            entry_pose = interp_pose(start, end, 1.0 - seg_exit_frac[seg_idx])
            corner_pose = end
            next_end = waypoints[seg_idx + 2]
            exit_pose = interp_pose(end, next_end, seg_entry_frac[seg_idx + 1])
            zone_start = len(poses) - 1 if poses else 0
            zone_poses, _ = blend_zone_poses(entry_pose, corner_pose, exit_pose, spacing,
                                             continuity)
            poses.extend(zone_poses[1:] if poses else zone_poses)
            blend_zones.append({"corner_index": int(seg_idx + 1), "start_index": int(zone_start),
                                "end_index": int(len(poses) - 1)})

        boundaries.append(len(poses))

    if not poses:
        raise ValueError("composite path produced no samples")

    total_translation = float(sum(
        np.linalg.norm(poses[i + 1].t - poses[i].t) for i in range(len(poses) - 1)))
    total_rotation = float(sum(
        _rotation_angle(poses[i], poses[i + 1]) for i in range(len(poses) - 1)))

    path = CartesianPath(
        poses=tuple(poses), s=np.linspace(0.0, 1.0, len(poses)),
        translation_length=total_translation, rotation_angle=total_rotation)
    return (path, boundaries, tuple(blend_zones)) if return_blend_zones else (path, boundaries)
