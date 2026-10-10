"""Joint-space composite path geometry with Bezier zone blending.

Mirrors motion/cartesian_paths.py's composite-path builder but operates
entirely in joint space (radians), with no IK or orientation involved.
See TODO/blending_design.md §4.

Planning-time code only (runs once per queued command in the planner
thread) — plain numpy array building, not pre-allocated buffers; the
hot-path allocation rules in CLAUDE.md don't apply here.

--------------------------------------------------------------------------
Why this path is sampled by ARC LENGTH, not by sample index
--------------------------------------------------------------------------
TOPPRA fits a cubic spline through these samples and reads the centripetal
acceleration limit off q''(s):

    q̈ = q'(s)·s̈ + q''(s)·ṡ²

so anything that inflates q''(s) makes the planner brake, whether or not the
path is actually curved there.

The original builder spaced samples by *index*: each piece got a sample count
derived from `samples_per_segment`. That makes |dq/ds| piecewise-constant but
DISCONTINUOUS at every line->Bezier junction. With blend fractions (f1, f2) on
segments of length (L1, L2), the old scheme gave

    straight segment i : |dq/ds| = L_i · N / sps
    Bezier at entry    : |dq/ds| = L1  · N / sps · 2·f1/(f1+f2)
    Bezier at exit     : |dq/ds| = L2  · N / sps · 2·f2/(f1+f2)

which match the adjacent straight segments ONLY when f1 == f2. The planner
computes f1 = r/seg_before and f2 = r/seg_after, so they are equal only when
the two adjacent segments happen to be the same length — i.e. almost never on
a real toolpath. Otherwise |dq/ds| jumps at the junction, the spline reads that
parameterization jump as curvature, and the planner brakes for a corner that
does not exist. The apparent curvature scales as ~1/ds, so the path never
converges: refining the sampling makes the move slower without limit.

Measured 2026-07-11 (joint rectangle, PAR6): blend fracs (0.20, 0.45) ran 27%
slow at the old default density and kept degrading (+44%) as density was
refined; (0.40, 0.30) drifted 28.6% over a 16x density sweep. Sampling every
piece from its ANALYTIC curve at a uniform arc-length spacing makes |dq/ds|
constant across the whole path, so q''(s) carries only true geometric
curvature. Same sweep after the fix: <1% drift, for both symmetric and
asymmetric fractions.

Corollary: never build one of these paths by resampling an already-sampled
polyline. Linear interpolation between existing samples lands the new points on
the CHORDS, turning every original sample into a tiny kink — the same
1/ds curvature blow-up by another route.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

# Bounds on the sampled path. The spacing itself is derived from the execution
# limits by arclen_spacing_for(); these only keep pathological inputs (a 1-urad
# move, a 50-rad chain) from producing a degenerate or enormous path.
MIN_PIECE_SAMPLES = 2
MAX_PATH_SAMPLES = 4000
DEFAULT_ARCLEN_SPACING_RAD = 0.01

# Bezier arc length is computed by dense quadrature on the true curve and then
# inverted to get t at uniform arc length. 512 points puts the arc-length error
# well below a micro-radian for any corner we can actually execute.
_BEZIER_QUADRATURE = 512


def arclen_spacing_for(max_vel: np.ndarray | float, dt: float) -> float:
    """Path resolution derived from the execution limits.

    One sample per control tick at full joint speed: the geometric path is
    never coarser than the trajectory that will be sampled off it. The absolute
    value is not critical (the path converges for anything below ~0.03 rad) —
    what matters for correctness is that the spacing is UNIFORM. This just picks
    a value that is comfortably inside the converged regime without generating
    more samples than TOPPRA needs.
    """
    speed = float(np.linalg.norm(np.asarray(max_vel, dtype=np.float64)))
    spacing = 0.5 * speed * float(dt)
    if not np.isfinite(spacing) or spacing <= 0.0:
        return DEFAULT_ARCLEN_SPACING_RAD
    return spacing


# Deviation of the corner curve from the vertex, as a multiple of d*sin(phi/2), where d is
# the blend distance and phi the turn angle. Both verified against the sampled curve to 1e-6.
# The C2 curve hugs the legs longer, so for the SAME d it deviates less (7/32 vs 1/2) — and
# therefore for the same TOLERANCE it needs a LARGER d.
CORNER_DEVIATION_COEFF = {"c1": 0.5, "c2": 7.0 / 32.0}


def _bezier(entry_q, corner_q, exit_q, t, continuity="c1"):
    """The corner curve, evaluated analytically at arbitrary t (never interpolated).

    continuity="c1" (default, and what shipped): QUADRATIC Bezier
        q(t) = (1-t)^2 E + 2t(1-t) W + t^2 X
    Tangent to the incoming leg at t=0 and the outgoing leg at t=1, so the corner is rounded
    with C1 (velocity) continuity. But CURVATURE STEPS at both ends: a straight leg has zero
    curvature and this curve enters at d(phi)/... a finite nonzero value (measured 0.5 for a
    90-degree corner with d=1). Curvature drives centripetal acceleration, so that step is an
    instantaneous acceleration change — infinite jerk, twice per corner, NO MATTER which
    profile times the path. TOPPRA cannot fix it; it is in the geometry.

    continuity="c2": QUINTIC Bezier with control points

        P0 = E,  P1 = (V+E)/2,  P2 = V,  P3 = V,  P4 = (V+X)/2,  P5 = X

    chosen so B''(0) = B''(1) = 0 exactly (20*(P2 - 2*P1 + P0) = 0 by construction, and it
    holds for ASYMMETRIC entry/exit distances too). Curvature therefore starts and ends at
    zero and matches the straight legs: no step, no jerk spike.

    THE TRADE, measured: the C2 curve is smoother at the joins but TIGHTER at the apex —
    peak curvature 4.34 vs 1.41 for the same d. TOPPRA brakes harder there. C2 buys
    continuity and pays in corner speed, which is exactly why it is selectable rather than
    simply "the better one".
    """
    if continuity == "c2":
        E = np.asarray(entry_q, dtype=np.float64)
        V = np.asarray(corner_q, dtype=np.float64)
        X = np.asarray(exit_q, dtype=np.float64)
        ctrl = (E, 0.5 * (V + E), V, V, 0.5 * (V + X), X)
        t = np.asarray(t, dtype=np.float64)[:, None]
        u = 1.0 - t
        # Bernstein basis, degree 5.
        return (u**5 * ctrl[0] + 5.0 * u**4 * t * ctrl[1] + 10.0 * u**3 * t**2 * ctrl[2]
                + 10.0 * u**2 * t**3 * ctrl[3] + 5.0 * u * t**4 * ctrl[4] + t**5 * ctrl[5])

    t = np.asarray(t, dtype=np.float64)[:, None]
    omt = 1.0 - t
    return (omt * omt) * entry_q + (2.0 * omt * t) * corner_q + (t * t) * exit_q


def blend_joint_zone(entry_q: np.ndarray, waypoint_q: np.ndarray, exit_q: np.ndarray, spacing: float,
                     continuity: str = "c1") -> tuple[np.ndarray, float]:
    """Sample a quadratic-Bezier joint-space corner at uniform ARC LENGTH.

    Walks a fine quadrature grid along the true curve, accumulates arc length,
    then inverts it to find the t values that sit at equal arc-length steps and
    evaluates the Bezier there. The returned samples therefore lie exactly on
    the curve AND are equally spaced along it.

    Returns (samples, arc_length).
    """
    entry_q = np.asarray(entry_q, dtype=np.float64)
    waypoint_q = np.asarray(waypoint_q, dtype=np.float64)
    exit_q = np.asarray(exit_q, dtype=np.float64)

    t_fine = np.linspace(0.0, 1.0, _BEZIER_QUADRATURE)
    p_fine = _bezier(entry_q, waypoint_q, exit_q, t_fine, continuity)
    cum = np.concatenate(
        ([0.0], np.cumsum(np.linalg.norm(np.diff(p_fine, axis=0), axis=1))))
    total = float(cum[-1])
    if total < 1e-12:
        return entry_q[None, :].copy(), 0.0

    n = max(3, int(round(total / spacing)) + 1)
    t_at = np.interp(np.linspace(0.0, total, n), cum, t_fine)
    return _bezier(entry_q, waypoint_q, exit_q, t_at, continuity), total


def linear_joint_segment(start, end, spacing):
    """Sample a straight joint-space segment at uniform arc length.

    Returns (samples, arc_length).
    """
    start = np.asarray(start, dtype=np.float64)
    end = np.asarray(end, dtype=np.float64)
    length = float(np.linalg.norm(end - start))
    if length < 1e-12:
        return start[None, :].copy(), 0.0

    n = max(MIN_PIECE_SAMPLES, int(round(length / spacing)) + 1)
    t = np.linspace(0.0, 1.0, n)[:, None]
    return start[None, :] + t * (end - start)[None, :], length


def build_composite_joint_path(waypoints: np.ndarray, blend_fracs: Sequence[float], arclen_spacing: float | None = None,
                               samples_per_segment: int | None = None, start_trim: float = 0.0,
                               continuity: str = "c1", return_blend_zones: bool = False) -> tuple:
    """Build a composite joint-space path with Bezier blend zones.

    Blend-zone sizes are expressed as fractions of each adjacent segment
    (caller pre-converts a physical blend radius to a fraction, e.g. via
    FK-based mm -> fraction conversion).

    Every piece — straight segments and blend zones alike — is sampled from its
    analytic curve at the SAME arc-length spacing, so |dq/ds| is constant across
    the whole path and q''(s) carries only real curvature. See the module
    docstring for why this matters.

    Args:
        waypoints: Joint-angle arrays (radians), length >= 2.
        blend_fracs: For each of the N-2 intermediate waypoints, a
            (frac_before, frac_after) tuple giving the fraction of the
            incoming/outgoing segment consumed by the blend zone. Clamped
            internally to [0, 0.5] and to prevent overlap.
        arclen_spacing: Target spacing between samples, in radians of
            joint-space path length. Defaults to DEFAULT_ARCLEN_SPACING_RAD;
            callers with access to the execution limits should pass
            arclen_spacing_for(max_vel, dt).
        samples_per_segment: Deprecated and ignored — sample density is now set
            by arclen_spacing. Accepted so existing callers keep working.
        start_trim: Fraction of the FIRST segment to skip. The rolling buffer needs this:
            when a batch of commands is spliced into a move already running, the new path
            starts at the EXIT of the Bezier that rounds the corner into it — part-way
            along the first segment — not at the waypoint. Without it the builder would
            treat that exit point as a fresh waypoint and recompute every blend fraction
            against the shortened segment, relocating the corners.

    Returns:
        (positions, boundaries):
          - positions: (M, ndof) ndarray of joint positions along the composite path.
          - boundaries: list of length N-1 (one per non-start waypoint / one per
            original queued command); boundaries[i] is the row index in
            `positions` where waypoint i+1 has been fully handed off (i.e. the
            end of its incoming segment plus any trailing blend zone that
            rounds it). This is where a per-command checkpoint should fire
            when sending a blended chain — see TODO/blending_design.md §5.4.
            boundaries[-1] always equals len(positions).

    Raises:
        ValueError: if inputs are inconsistent.
    """
    n = len(waypoints)
    if n < 2:
        raise ValueError("Need at least 2 waypoints")
    if len(blend_fracs) != max(0, n - 2):
        raise ValueError(f"Expected {max(0, n - 2)} blend_fracs, got {len(blend_fracs)}")

    waypoints = [np.asarray(w, dtype=np.float64) for w in waypoints]
    spacing = (DEFAULT_ARCLEN_SPACING_RAD if arclen_spacing is None
               else float(arclen_spacing))
    if not spacing > 0.0:
        raise ValueError(f"arclen_spacing must be positive, got {spacing}")

    if n == 2:
        # start_trim matters here too: a spliced continuation's last leg begins at the
        # Bezier EXIT, part-way along the segment, not at the waypoint. Missing this made
        # the path jump BACKWARDS from the exit to the waypoint — a doubling-back that
        # showed up as a path LONGER than the straight line (0.579 rad vs 0.494) and a 2.5x
        # spacing jump, which the spline then read as curvature.
        a = waypoints[0] + float(np.clip(start_trim, 0.0, 0.95)) * (waypoints[1] - waypoints[0])
        pos, _ = linear_joint_segment(a, waypoints[1], _capped_spacing(spacing, waypoints))
        return (pos, [pos.shape[0]], ()) if return_blend_zones else (pos, [pos.shape[0]])

    exit_frac = [min(max(f[0], 0.0), 0.5) for f in blend_fracs]
    entry_frac = [min(max(f[1], 0.0), 0.5) for f in blend_fracs]

    seg_start_trim = [0.0] * (n - 1)
    seg_end_trim = [0.0] * (n - 1)
    seg_start_trim[0] = float(np.clip(start_trim, 0.0, 0.95))
    for i in range(len(blend_fracs)):
        wp_idx = i + 1
        seg_end_trim[wp_idx - 1] = exit_frac[i]
        seg_start_trim[wp_idx] = entry_frac[i]

    # Clamp overlapping trims on any single segment (entry trim from the
    # previous corner + exit trim into the next corner can't exceed the
    # whole segment).
    for s in range(n - 1):
        total = seg_start_trim[s] + seg_end_trim[s]
        if total > 1.0:
            scale = 1.0 / total
            seg_start_trim[s] *= scale
            seg_end_trim[s] *= scale

    # Resolve every piece's endpoints first, so the total arc length is known
    # before any sampling happens — the spacing may need capping for a very long
    # chain, and it must be the SAME spacing on every piece or the junction
    # kinks come straight back.
    pieces = []          # line | (bezier, corner index, entry, corner, exit) | boundary
    control_points = []  # for the length estimate that caps the spacing
    for seg_idx in range(n - 1):
        start = waypoints[seg_idx]
        end = waypoints[seg_idx + 1]
        s_start = seg_start_trim[seg_idx]
        s_end = 1.0 - seg_end_trim[seg_idx]

        if s_end > s_start + 1e-9:
            lin_start = start + s_start * (end - start)
            lin_end = start + s_end * (end - start)
            pieces.append(("line", lin_start, lin_end))
            control_points += [lin_start, lin_end]

        if seg_idx < len(blend_fracs) and (exit_frac[seg_idx] > 0 or entry_frac[seg_idx] > 0):
            entry_q = start + (1.0 - seg_end_trim[seg_idx]) * (end - start)
            corner_q = end
            exit_q = end + seg_start_trim[seg_idx + 1] * (waypoints[seg_idx + 2] - end)
            pieces.append(("bezier", seg_idx + 1, entry_q, corner_q, exit_q))
            control_points += [entry_q, corner_q, exit_q]

        pieces.append(("boundary",))

    spacing = _capped_spacing(spacing, control_points)

    rows = []
    row_count = 0
    boundaries = []
    blend_zones = []
    for piece in pieces:
        if piece[0] == "boundary":
            boundaries.append(row_count)
            continue
        is_blend = piece[0] == "bezier"
        if piece[0] == "line":
            seg_rows, _ = linear_joint_segment(piece[1], piece[2], spacing)
        else:
            seg_rows, _ = blend_joint_zone(piece[2], piece[3], piece[4], spacing, continuity)
        zone_start = row_count - 1 if is_blend and rows else row_count
        if rows:
            seg_rows = seg_rows[1:]   # drop the duplicate point at the junction
        if seg_rows.shape[0] == 0:
            continue
        rows.append(seg_rows)
        row_count += seg_rows.shape[0]
        if is_blend:
            blend_zones.append({"corner_index": int(piece[1]), "start_index": int(zone_start),
                                "end_index": int(row_count - 1)})

    if not rows:
        raise ValueError("composite path produced no samples")
    positions = np.concatenate(rows, axis=0)
    return (positions, boundaries, tuple(blend_zones)) if return_blend_zones else (positions, boundaries)


def _capped_spacing(spacing, points):
    """Grow the spacing if this path would otherwise exceed MAX_PATH_SAMPLES.

    Uses the control polygon (a cheap upper bound on the true arc length), so a
    very long chain degrades to a coarser — but still perfectly UNIFORM — path
    rather than an enormous one. Uniformity is what correctness depends on; the
    absolute spacing only sets resolution.
    """
    if len(points) < 2:
        return spacing
    pts = np.asarray(points, dtype=np.float64)
    approx_len = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
    if approx_len <= 0.0:
        return spacing
    return max(spacing, approx_len / MAX_PATH_SAMPLES)
