"""Safety zones — allowed / forbidden regions of the CELL, checked against spheres on the robot.

Plan and vendor survey: TODO/safety_zones_plan.md. User guide: docs/safety_zones.md.

A zone describes the cell (a table, a fixture, a wall), not the arm, so zones live in a cell file
(config/cells/<name>.xml, chosen by <active_cell> in config/system.xml). No cell = no zones.

Coordinates: the ROBOT BASE frame, metres — the frame get_pose() reports.

The robot side (v1, decided 2026-10-01): sphere 0 is the TCP point (radius 0); after it come the
active gripper's tool spheres, given in the TCP frame in the gripper XML. A zone with checks="tcp"
looks at sphere 0 only, checks="tool" at all of them. The arm links are NOT checked yet.

Five shapes. signed_distance() is < 0 inside the shape, > 0 outside, and its size is the exact
distance to the surface (exact for all five):

  box       axis-aligned, min / max corners
  prism     a polygon of 3-16 XY vertices, extruded from z_min to z_max (as industrial zone tools offer)
  cylinder  axis p1 -> p2 (any direction), radius; flat ends, or rounded=1 for a capsule
  sphere    centre, radius
  plane     a point and a normal; "inside" is the side the normal points to (the usual plane-arrow convention)

penetration() is how far one robot sphere violates a zone, in metres; > 0 means violating:

  allowed    the sphere must be inside by the margin:  pen = sd + r + margin
  forbidden  the sphere must be outside by the margin: pen = r + margin - sd

Scalar math only (the math module, no numpy temporaries), so the RT safety monitor can call it every
tick without allocating, and the planner calls the very same functions per trajectory sample — one
implementation, not a fast copy and a slow copy that can drift apart.
"""
from __future__ import annotations

from collections.abc import Sequence

import math
import re
from typing import TYPE_CHECKING

if TYPE_CHECKING:            # hints only — this module stays numpy-free at run time (pure-math hot path)
    import numpy as np

SHAPES = ("box", "prism", "cylinder", "sphere", "plane")
KINDS = ("allowed", "forbidden")
REACTIONS = ("stop", "warn")
CHECKS = ("tcp", "tool")
FREEDRIVE_REACTIONS = ("warn", "stop")   # pushed through a zone in freedrive (IDLE): warn by default
MAX_VERTICES = 16          # bounds the per-tick cost of one prism
MAX_MARGIN_M = 0.5

ZONE_VIOLATION = "ZONE_VIOLATION"
ZONE_WARNING = "ZONE_WARNING"

_NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,32}$")
_BOX, _PRISM, _CYL, _CAPSULE, _SPHERE, _PLANE = range(6)
TCP_SPHERE = (0.0, 0.0, 0.0, 0.0)


def zone_key(zone: dict, suffix: str) -> str:
    """Error key for a zone, e.g. 'Z_fixture:ZONE_VIOLATION'. Starts with 'Z_' (never 'J', which
    error_checks reads as a joint key) and keeps the ':SUFFIX' form every other key has."""
    return f"Z_{zone['name']}:{suffix}"


# --------------------------------------------------------------------------------------- building
def _vec(value: Sequence[float], n: int, what: str) -> tuple[float, ...]:
    try:
        v = tuple(float(x) for x in value)
    except (TypeError, ValueError):
        raise ValueError(f"{what} must be {n} numbers, got {value!r}") from None
    if len(v) != n or not all(math.isfinite(x) for x in v):
        raise ValueError(f"{what} must be {n} finite numbers, got {value!r}")
    return v


def _positive(value: float, what: str) -> float:
    try:
        v = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{what} must be a number, got {value!r}") from None
    if not (math.isfinite(v) and v > 0.0):
        raise ValueError(f"{what} must be > 0, got {value!r}")
    return v


def _segments_cross(a, b, c, d):
    """True if segments ab and cd properly intersect (shared endpoints excluded)."""
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)


def _check_polygon(verts: Sequence[Sequence[float]], what: str) -> None:
    n = len(verts)
    if not (3 <= n <= MAX_VERTICES):
        raise ValueError(f"{what}: a prism needs 3-{MAX_VERTICES} vertices, got {n}")
    for i in range(n):
        if math.dist(verts[i], verts[(i + 1) % n]) < 1e-9:
            raise ValueError(f"{what}: vertices {i} and {(i + 1) % n} are the same point")
    area2 = sum(verts[i][0] * verts[(i + 1) % n][1] - verts[(i + 1) % n][0] * verts[i][1] for i in range(n))
    if abs(area2) < 1e-10:
        raise ValueError(f"{what}: the polygon has no area")
    for i in range(n):
        for j in range(i + 1, n):
            if abs(i - j) in (1, n - 1):
                continue                                 # neighbouring edges share a vertex
            if _segments_cross(verts[i], verts[(i + 1) % n], verts[j], verts[(j + 1) % n]):
                raise ValueError(f"{what}: the polygon's edges {i}-{(i + 1) % n} and {j}-{(j + 1) % n} cross")


def make_zone(name: str, shape: str, kind: str, reaction: str = "stop", checks: str = "tool", margin: float = 0.0,
              enabled: bool = True, haptic: bool = True, freedrive: str = "warn", **geometry: object) -> dict:
    """Validate one zone and precompute its geometry. Raises ValueError with a readable message.

    haptic:    in IDLE gravity-comp freedrive, render the zone as a virtual wall you feel (utility/haptic_walls.py).
    freedrive: what the safety monitor does when the zone is pushed THROUGH in freedrive (IDLE / ACTIVE_ERROR)
               — "warn" (default) or "stop". Active modes always use `reaction`.

    geometry per shape (metres, base frame):
      box: min=(x,y,z), max=(x,y,z)          prism: vertices=[(x,y), ...], z_min, z_max
      cylinder: p1, p2, radius, rounded=False  sphere: center, radius     plane: point, normal
    """
    what = f"zone '{name}'"
    if not isinstance(name, str) or not _NAME_RE.match(name):
        raise ValueError(f"zone name {name!r}: use 1-32 letters, digits, '_' or '-'")
    if shape not in SHAPES:
        raise ValueError(f"{what}: shape must be one of {SHAPES}, got {shape!r}")
    if kind not in KINDS:
        raise ValueError(f"{what}: kind must be one of {KINDS}, got {kind!r}")
    if reaction not in REACTIONS:
        raise ValueError(f"{what}: reaction must be one of {REACTIONS}, got {reaction!r}")
    if checks not in CHECKS:
        raise ValueError(f"{what}: checks must be one of {CHECKS}, got {checks!r}")
    if freedrive not in FREEDRIVE_REACTIONS:
        raise ValueError(f"{what}: freedrive must be one of {FREEDRIVE_REACTIONS}, got {freedrive!r}")
    if haptic not in (True, False, 0, 1, "0", "1"):
        raise ValueError(f"{what}: haptic must be 0 or 1, got {haptic!r}")
    haptic = haptic in (True, 1, "1")
    try:
        margin = float(margin)
    except (TypeError, ValueError):
        raise ValueError(f"{what}: margin must be a number, got {margin!r}") from None
    if not (math.isfinite(margin) and 0.0 <= margin <= MAX_MARGIN_M):
        raise ValueError(f"{what}: margin must be 0..{MAX_MARGIN_M} m, got {margin!r}")

    g = dict(geometry)
    expected = {"box": {"min", "max"}, "prism": {"vertices", "z_min", "z_max"},
                "cylinder": {"p1", "p2", "radius", "rounded"}, "sphere": {"center", "radius"},
                "plane": {"point", "normal"}}[shape]
    unknown = set(g) - expected
    if unknown:
        raise ValueError(f"{what}: unknown {shape} parameter(s) {sorted(unknown)}; expected {sorted(expected)}")

    if shape == "box":
        lo, hi = _vec(g.get("min"), 3, f"{what} min"), _vec(g.get("max"), 3, f"{what} max")
        if not all(lo[i] < hi[i] for i in range(3)):
            raise ValueError(f"{what}: every min must be below its max, got min={lo} max={hi}")
        public = {"min": list(lo), "max": list(hi)}
        compiled = (_BOX, (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2,
                    (hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2, (hi[2] - lo[2]) / 2)
    elif shape == "prism":
        raw = g.get("vertices")
        if raw is None or isinstance(raw, (str, bytes)):
            raise ValueError(f"{what}: vertices must be a list of (x, y) pairs")
        verts = [_vec(v, 2, f"{what} vertex {i}") for i, v in enumerate(raw)]
        _check_polygon(verts, what)
        z0 = _vec([g.get("z_min")], 1, f"{what} z_min")[0]
        z1 = _vec([g.get("z_max")], 1, f"{what} z_max")[0]
        if not z0 < z1:
            raise ValueError(f"{what}: z_min must be below z_max, got {z0} >= {z1}")
        public = {"vertices": [list(v) for v in verts], "z_min": z0, "z_max": z1}
        edges = []
        for i in range(len(verts)):
            (ax, ay), (bx, by) = verts[i - 1], verts[i]
            ex, ey = bx - ax, by - ay
            edges.append((ax, ay, bx, by, ex, ey, 1.0 / (ex * ex + ey * ey)))
        compiled = (_PRISM, tuple(edges), (z0 + z1) / 2, (z1 - z0) / 2)
    elif shape == "cylinder":
        p1, p2 = _vec(g.get("p1"), 3, f"{what} p1"), _vec(g.get("p2"), 3, f"{what} p2")
        r = _positive(g.get("radius"), f"{what} radius")
        length = math.dist(p1, p2)
        if length < 1e-6:
            raise ValueError(f"{what}: p1 and p2 are the same point")
        u = tuple((p2[i] - p1[i]) / length for i in range(3))
        try:
            rounded = bool(int(g.get("rounded", 0)))
        except (TypeError, ValueError):
            raise ValueError(f"{what}: rounded must be 0 or 1, got {g.get('rounded')!r}") from None
        public = {"p1": list(p1), "p2": list(p2), "radius": r, "rounded": rounded}
        compiled = (_CAPSULE if rounded else _CYL, p1[0], p1[1], p1[2], u[0], u[1], u[2], length, r)
    elif shape == "sphere":
        c = _vec(g.get("center"), 3, f"{what} center")
        r = _positive(g.get("radius"), f"{what} radius")
        public = {"center": list(c), "radius": r}
        compiled = (_SPHERE, c[0], c[1], c[2], r)
    else:
        p = _vec(g.get("point"), 3, f"{what} point")
        n = _vec(g.get("normal"), 3, f"{what} normal")
        nn = math.sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2])
        if nn < 1e-9:
            raise ValueError(f"{what}: the normal is zero")
        n = (n[0] / nn, n[1] / nn, n[2] / nn)
        public = {"point": list(p), "normal": list(n)}
        compiled = (_PLANE, p[0], p[1], p[2], n[0], n[1], n[2])

    if shape == "box":
        bound = (compiled[1], compiled[2], compiled[3], math.sqrt(compiled[4] ** 2 + compiled[5] ** 2 + compiled[6] ** 2))
    elif shape == "prism":
        xs, ys = [v[0] for v in verts], [v[1] for v in verts]
        bx, by = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        bound = (bx, by, compiled[2], math.sqrt(max((x - bx) ** 2 + (y - by) ** 2 for x, y in verts) + compiled[3] ** 2))
    elif shape == "cylinder":
        mid = tuple(p1[i] + 0.5 * length * u[i] for i in range(3))
        reach = 0.5 * length + r if rounded else math.sqrt((0.5 * length) ** 2 + r * r)
        bound = (mid[0], mid[1], mid[2], reach)
    elif shape == "sphere":
        bound = (compiled[1], compiled[2], compiled[3], compiled[4])
    else:
        bound = None
    return {"name": name, "shape": shape, "kind": kind, "reaction": reaction, "checks": checks,
            "margin": margin, "enabled": bool(enabled), "haptic": haptic, "freedrive": freedrive,
            "geometry": public, "_c": compiled, "_allowed": kind == "allowed", "_bound": bound}


def public_zone(zone: dict) -> dict:
    """JSON-safe description of a zone (for get_zones / the GUI)."""
    return {k: zone[k] for k in ("name", "shape", "kind", "reaction", "checks", "margin", "enabled", "haptic",
                                 "freedrive", "geometry")}


def check_unique_names(zones: Sequence[dict], where: str) -> None:
    seen = set()
    for z in zones:
        if z["name"] in seen:
            raise ValueError(f"{where}: two zones are named '{z['name']}'")
        seen.add(z["name"])


def robot_spheres(tool_spheres: Sequence[Sequence[float]]) -> tuple[tuple[float, ...], ...]:
    """Sphere 0 = the TCP point, then the tool spheres (x, y, z, r) in the TCP frame."""
    out = [TCP_SPHERE]
    for i, s in enumerate(tool_spheres or ()):
        x, y, z, r = _vec(s, 4, f"tool sphere {i}")
        if r < 0.0:
            raise ValueError(f"tool sphere {i}: radius must be >= 0, got {r}")
        out.append((x, y, z, r))
    return tuple(out)


# ------------------------------------------------------------------------------- per-tick geometry
def _polygon_sd(edges, x, y):
    """Signed distance to a simple polygon in the plane (< 0 inside). Exact. edges are precomputed
    (ax, ay, bx, by, ex, ey, 1/|e|^2) from make_zone."""
    best = math.inf
    inside = False
    for ax, ay, bx, by, ex, ey, inv in edges:
        wx, wy = x - ax, y - ay
        t = (wx * ex + wy * ey) * inv
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        dx, dy = wx - t * ex, wy - t * ey
        d = dx * dx + dy * dy
        if d < best:
            best = d
        if (ay > y) != (by > y) and x < ax + (y - ay) * ex / ey:
            inside = not inside
    d = math.sqrt(best)
    return -d if inside else d


def _sd(c, x, y, z):
    code = c[0]
    if code == _BOX:
        _, cx, cy, cz, hx, hy, hz = c
        qx, qy, qz = abs(x - cx) - hx, abs(y - cy) - hy, abs(z - cz) - hz
        ox = qx if qx > 0.0 else 0.0
        oy = qy if qy > 0.0 else 0.0
        oz = qz if qz > 0.0 else 0.0
        m = qx if qx > qy else qy
        if qz > m:
            m = qz
        return math.sqrt(ox * ox + oy * oy + oz * oz) + (m if m < 0.0 else 0.0)
    if code == _SPHERE:
        _, cx, cy, cz, r = c
        return math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2) - r
    if code == _PLANE:
        _, px, py, pz, nx, ny, nz = c
        return -((x - px) * nx + (y - py) * ny + (z - pz) * nz)
    if code == _CYL or code == _CAPSULE:
        _, ax, ay, az, ux, uy, uz, length, r = c
        dx, dy, dz = x - ax, y - ay, z - az
        t = dx * ux + dy * uy + dz * uz
        if code == _CAPSULE:
            tc = 0.0 if t < 0.0 else (length if t > length else t)
            ex, ey, ez = dx - tc * ux, dy - tc * uy, dz - tc * uz
            return math.sqrt(ex * ex + ey * ey + ez * ez) - r
        rx, ry, rz = dx - t * ux, dy - t * uy, dz - t * uz
        dr = math.sqrt(rx * rx + ry * ry + rz * rz) - r
        da = abs(t - 0.5 * length) - 0.5 * length
        m = dr if dr > da else da
        odr = dr if dr > 0.0 else 0.0
        oda = da if da > 0.0 else 0.0
        return (m if m < 0.0 else 0.0) + math.sqrt(odr * odr + oda * oda)
    _, edges, zm, hz = c                                    # prism
    d2 = _polygon_sd(edges, x, y)
    dz = abs(z - zm) - hz
    m = d2 if d2 > dz else dz
    od2 = d2 if d2 > 0.0 else 0.0
    odz = dz if dz > 0.0 else 0.0
    return (m if m < 0.0 else 0.0) + math.sqrt(od2 * od2 + odz * odz)


_GRAD_H = 1e-5        # m, central-difference step for the wall's direction


def push_direction(zone: dict, x: float, y: float, z: float) -> tuple[float, float, float] | None:
    """Unit vector along which a point's penetration of `zone` GROWS (the wall pushes the opposite way),
    or None where it is undefined (the centre of a sphere, the axis of a cylinder). Central differences of
    the same signed distance everything else uses — one geometry, not a second analytic copy that could
    disagree with it at an edge. Scalar math, no allocation."""
    c, h = zone["_c"], _GRAD_H
    gx = _sd(c, x + h, y, z) - _sd(c, x - h, y, z)
    gy = _sd(c, x, y + h, z) - _sd(c, x, y - h, z)
    gz = _sd(c, x, y, z + h) - _sd(c, x, y, z - h)
    n = math.sqrt(gx * gx + gy * gy + gz * gz)
    if n < 1e-3 * h:                                     # |grad sd| ~ 1 almost everywhere: ~0 = undefined
        return None
    s = (1.0 if zone["_allowed"] else -1.0) / n          # forbidden: penetration grows INWARD (-grad sd)
    return gx * s, gy * s, gz * s


def signed_distance(zone: dict, x: float, y: float, z: float) -> float:
    """< 0 inside the zone's shape, > 0 outside; the size is the distance to its surface (m)."""
    return _sd(zone["_c"], x, y, z)


def penetration(zone: dict, x: float, y: float, z: float, radius: float = 0.0, margin: float | None = None) -> float:
    """How far a sphere at (x, y, z) violates the zone (m); > 0 = violating. margin None = the zone's."""
    m = zone["margin"] if margin is None else margin
    sd = _sd(zone["_c"], x, y, z)
    return sd + radius + m if zone["_allowed"] else radius + m - sd


def sphere_centres(m: np.ndarray, spheres: Sequence[Sequence[float]], cx: np.ndarray, cy: np.ndarray, cz: np.ndarray) -> None:
    """World centres of the robot spheres for TCP pose `m` (a flat 16-value row-major 4x4 — a
    shared-memory view or a list), written INTO the lists cx, cy, cz (len >= len(spheres)). Reads each
    pose element once as a Python float: arithmetic on numpy scalars is several times slower."""
    m0, m1, m2, m3 = float(m[0]), float(m[1]), float(m[2]), float(m[3])
    m4, m5, m6, m7 = float(m[4]), float(m[5]), float(m[6]), float(m[7])
    m8, m9, m10, m11 = float(m[8]), float(m[9]), float(m[10]), float(m[11])
    for k in range(len(spheres)):
        sx, sy, sz, _ = spheres[k]
        cx[k] = m0 * sx + m1 * sy + m2 * sz + m3
        cy[k] = m4 * sx + m5 * sy + m6 * sz + m7
        cz[k] = m8 * sx + m9 * sy + m10 * sz + m11


def penetration_at(zone: dict, cx: np.ndarray, cy: np.ndarray, cz: np.ndarray, spheres: Sequence[Sequence[float]], n: int,
                   margin: float | None) -> tuple[float, int]:
    """Worst penetration of `zone` over robot spheres 0..n-1 whose centres are in cx, cy, cz.
    Returns (pen, sphere_index). For a FORBIDDEN zone a sphere far outside its bounding sphere is not
    evaluated exactly: its penetration is replaced by an upper bound that is still < 0 — so every
    "clear" stays clear and every violation is exact."""
    c = zone["_c"]
    allowed = zone["_allowed"]
    bound = None if allowed else zone["_bound"]
    worst, which = -math.inf, -1
    for k in range(n):
        x, y, z, sr = cx[k], cy[k], cz[k], spheres[k][3]
        if bound is not None:
            dx, dy, dz = x - bound[0], y - bound[1], z - bound[2]
            reach = bound[3] + sr + margin
            d2 = dx * dx + dy * dy + dz * dz
            if d2 > reach * reach:
                pen = reach - math.sqrt(d2)                 # >= the true value, and < 0
                if pen > worst:
                    worst, which = pen, k
                continue
        sd = _sd(c, x, y, z)
        pen = sd + sr + margin if allowed else sr + margin - sd
        if pen > worst:
            worst, which = pen, k
    return worst, which


def zone_penetration(zone: dict, m: np.ndarray, spheres: Sequence[Sequence[float]], use_margin: bool = True) -> tuple[float, int]:
    """Worst penetration of `zone` over the robot spheres it checks, for TCP pose `m` (flat 16-value
    row-major 4x4). Returns (pen, sphere_index). Cold-path convenience (allocates three lists); the
    monitor uses sphere_centres + penetration_at with preallocated ones."""
    n = len(spheres)
    cx, cy, cz = [0.0] * n, [0.0] * n, [0.0] * n
    sphere_centres(m, spheres, cx, cy, cz)
    return penetration_at(zone, cx, cy, cz, spheres, 1 if zone["checks"] == "tcp" else n,
                          zone["margin"] if use_margin else 0.0)


def pose_violations(zones: Sequence[dict], m: np.ndarray, spheres: Sequence[Sequence[float]],
                    use_margin: bool = True) -> list[tuple[dict, float, int]]:
    """[(zone, pen, sphere_index)] for every enabled zone this TCP pose violates (cold path)."""
    out = []
    for zone in zones:
        if zone["enabled"]:
            pen, k = zone_penetration(zone, m, spheres, use_margin)
            if pen > 0.0:
                out.append((zone, pen, k))
    return out


def first_path_violation(zones: Sequence[dict], poses: Sequence[np.ndarray], spheres: Sequence[Sequence[float]],
                         start_pose: np.ndarray, reaction: str = "stop") -> tuple[dict, int, float, int] | None:
    """First sample of a path (a list of flat TCP poses) that violates an enabled zone of `reaction`.

    RECOVERY, as for the joint limits: a robot that STARTS in violation of a zone may stay as deep
    as it already is, never deeper — so a move back out is accepted and nothing drives it further in.
    Returns None, or (zone, sample_index, pen, sphere_index) for the earliest offending sample.
    """
    best = None
    for zone in zones:
        if not zone["enabled"] or zone["reaction"] != reaction:
            continue
        start_pen, _ = zone_penetration(zone, start_pose, spheres)
        allow = (start_pen if start_pen > 0.0 else 0.0) + 1e-9
        for i, m in enumerate(poses):
            if best is not None and i >= best[1]:
                break
            pen, k = zone_penetration(zone, m, spheres)
            if pen > allow:
                best = (zone, i, pen, k)
                break
    return best


# ------------------------------------------------------------------------------ query replies (cold)
def zones_summary(setup: dict) -> dict:
    """get_zones / reload_zones reply for a zone setup {"cell", "zones", "tool_spheres"}."""
    return {"cell": setup["cell"], "zones": [public_zone(z) for z in setup["zones"]],
            "tool_spheres": [list(s) for s in setup["tool_spheres"]],
            "haptic_walls": dict(setup.get("haptic") or {})}


def reach_reply(joints_deg: list[float] | None, pose: np.ndarray, setup: dict,
                self_collision: str | None = None) -> dict:
    """is_reachable's reply. joints_deg: the solution (list, degrees) or None; pose: the target as a
    flat row-major 4x4. `allowed` = reachable AND no enabled stop-zone violated (margin included, as
    the planner checks) AND the solution clear of self-collision. With the TCP and tool spheres only (v1)
    the zones depend on the POSE alone, never on which joint solution reaches it; self-collision depends on
    the solution — the search prefers one that is clear, and `self_collision` names the two links
    ("upper arm and tool") only when every solution it found brings two links within the planner's margin
    (2026-10-07)."""
    blocked = None
    if joints_deg is not None:
        stops = [z for z in setup["zones"] if z["reaction"] == "stop"]
        hits = pose_violations(stops, pose, robot_spheres(setup["tool_spheres"]))
        blocked = hits[0][0]["name"] if hits else None
    return {"reachable": joints_deg is not None, "joints": joints_deg,
            "allowed": joints_deg is not None and blocked is None and self_collision is None, "zone": blocked,
            "self_collision": self_collision}
