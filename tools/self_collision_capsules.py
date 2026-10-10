#!/usr/bin/env python3
"""Self-collision geometry for the arm: convex pieces + capsules fitted to the link meshes, PROVEN against them, and
a map of where the arm hits itself.

TODO/self_collision_plan.md S1/S2 (2026-10-06). Everything here is OFFLINE (meshes): the robot will carry only
what this tool writes. Run it with the robot DISABLED — it is CPU-heavy.

THE MODEL (measured 2026-10-06, see the plan §3.1): a few CONVEX PIECES per link — the convex hulls of groups of
the link's triangles — checked with coal's GJK distance (coal ships with Pinocchio, which RTI already loads).
Against the real meshes the pieces read the gap to within ~0-8 mm (never optimistic) at ~6 µs per piece pair; a
bounding-sphere pre-filter in numpy leaves only the piece pairs that are actually near for GJK, so a quiet tick
costs tens of µs and a tight fold a few hundred. Capsules (one to eight per link, also fitted here) are coarser
by 40-60 mm — kept as the visual / coarse layer, not as the check.

    .venv/bin/python tools/self_collision_capsules.py fit                 # meshes -> capsules (several per link)
    .venv/bin/python tools/self_collision_capsules.py prove --poses 1000  # capsule gap <= mesh gap, every pose + pair
    .venv/bin/python tools/self_collision_capsules.py map --poses 20000   # which pairs collide / come close, how often
    .venv/bin/python tools/self_collision_capsules.py all                 # the three in a row
    .venv/bin/python tools/self_collision_capsules.py export --gripper msg --gripper-name MSG   # -> robots/, grippers/

fit    Per link (the PAR6 description package's URDF by default): points are sampled evenly over the mesh SURFACE (the CAD
       vertices sit only on edges and fillets) and grouped by k-means. CONVEX PIECES: --pieces groups per link;
       every triangle joins the nearest group with all three vertices, the piece is the convex hull of its group's
       vertices — so the union of the pieces contains the whole surface by construction. CAPSULES: ~24 groups,
       each its smallest capsule, merged bottom-up (always the merge adding the least volume — tubes merge along
       their length, a housing or an ear refuses to melt into the tube) down to the fewest capsules within
       --within x the finest fit's volume, at most --max-caps; then every vertex of the mesh refined to --max-edge
       is assigned to the nearest capsule, which grows to contain it, and every radius is inflated by
       max-edge / sqrt(3) — the capsules contain the surface too, coarsely. Both are checked here, vertex by
       vertex. Writes temp_data/self_collision/<robot>_<gripper>_collision.json — per link: our DH frame index,
       the fixed URDF->DH transform, the pieces (hull vertices + faces) and the capsules, in BOTH frames — plus the
       capsules-only XML snippet and the fit report.
prove  N uniform poses inside the soft limits: for every non-adjacent link pair, pieces gap <= mesh distance
       (coal on the full meshes, Pinocchio's own geometry model) and capsule gap <= pieces gap, within 0.1 mm.
       Any pose where not is a containment failure and is printed. Reported: the slack of pieces and capsules
       (median / p90 per pair), the false-near rate at --margin, and the cost of the pieces check with its
       pre-filter per pose (and how many GJK calls it took: median and worst).
map    The pieces model (exact to the mm), thousands of poses: per pair, how often the gap is < 0 (collision) and
       < the margin, an example pose in OUR degrees, and the pairs within the margin in (almost) every pose —
       close BY DESIGN: the report suggests their own smaller margin or an exclusion.

export Writes what the robot carries: robots/<robot>_collision.json (links 0-5: the pieces and capsules in OUR DH
       frames) and grippers/<gripper-name>_collision.json (the tool's, jaws folded, in the J6 joint frame).
       utility/self_collision.py loads them; the gripper XML names its file in <collision file="...">.

Frames. The URDF's link frames are not our DH frames: the fixed transform between them is found numerically (it
must be constant over random poses — checked) and the files carry the geometry in OUR frames: DH frame i for
link i (= RTI's Pinocchio joint i placed by the link's own Tz(d)Tx(a)Rx(alpha); the base in the world) and, for the
tool, the J6 JOINT frame (Pinocchio's oMi[6], before any gripper's link-6 transform), so the runtime never reads a
URDF. Joint angles: urdf = sign * ours + offset (see URDF_SIGN / URDF_OFFSET_DEG).

Source (2026-10-07, the user: "use stuff from that folder from now on for everything"): Source Robotics' PAR6 description
package 2.0, `~/Desktop/PAR6 Description/urdf/par6_description` (par6_flange / par6_msg / par6_ssg48.urdf). Its DH.md is
authoritative: TH1-TH6 ARE the real robot's joint coordinates and positive simulated motion is positive hardware motion,
theta2 = TH2 - 90 deg, theta3 = TH3 + 180, theta6 = TH6 + 180, wrist_3_link IS DH frame 6. Our DH (robots/PAR6.py) has no
angle offsets (theta = our angle), so TH = ours - [0, -90, 180, 0, 0, 180] (our home is TH = 0): sign +1, offset -home.
Checked: J1-J5 link frames identical to the old PAR6 model's (1e-15); our DH tool frame equals the package's `tcp`
to 0.004 mm / 0.001 deg (MSG). The OLD mapping, found by matching joint axes, had J6's offset as a free guess (0) and put the
MSG gripper turned 180 deg about its own axis — the bug this package fixed. Meshes are byte-identical to the old ones.

The jaws (prismatic, MSG 52.5 mm / SSG48 23.5 mm of stroke each) are fitted over their WHOLE stroke: each piece is the hull
of itself at both ends of the travel, which contains every position in between — never optimistic for an open gripper.
Since 2026-10-07 each jaw is ALSO written unswept, at its OPEN end, with the direction it closes along and its stroke
(the "jaws" of the gripper file): the collision worker can place it where the gripper reports it is (TODO/
self_collision_plan.md S3f). Which end is open is found from the meshes (the end where the two jaws are farthest
apart): URDF 0 for the MSG and the SSG48 — 104 mm / 47 mm between the jaws — and the far limit closed. `prove-jaws`
checks the unswept jaws, placed along that direction, against the meshes with the jaws at several openings.

Pieces per link: --pieces (6) for every link, --pieces-for LINK=K,... per link. Each link draws its random numbers from
its own seed (the run's seed + the link's name), so changing one link leaves every other link's pieces as they were;
--seeds (3) seeds are tried per link and the smallest total volume kept. --piece-edge (20 mm): triangles longer than
this are split before the grouping (2026-10-07). Why: the folded-elbow pair (upper arm / wrist 1) read up to 24 mm
closer than the meshes, and more pieces (8, 10, 12) changed nothing — a CAD triangle spanning a whole face made its
group's hull swallow the face; split first, the same 6 pieces read it within ~0-7 mm.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import struct
import sys
import time
import xml.etree.ElementTree as ET
import zlib

import numpy as np

try:
    import coal                                                   # ships with Pinocchio (cmeel)
except ImportError:                                               # pragma: no cover
    coal = None

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

from project_paths import PAR6_DESCRIPTION_DIR                                 # noqa: E402
REF = str(PAR6_DESCRIPTION_DIR)                                                # Source Robotics' PAR6 package 2.0
URDF = {"flange": (f"{REF}/par6_flange.urdf", REF),
        "msg": (f"{REF}/par6_msg.urdf", REF),
        "ssg48": (f"{REF}/par6_ssg48.urdf", REF)}
GRIPPER_FILE = {"flange": "Flange", "msg": "MSG", "ssg48": "SSG48"}                # grippers/<name>_collision.json
OUT_DIR = os.path.join(_ROOT, "temp_data", "self_collision")
# robot degrees -> URDF degrees: the package's TH = ours - our home pose (DH.md; see the module docstring)
URDF_SIGN = np.array([1, 1, 1, 1, 1, 1.0])
URDF_OFFSET_DEG = np.array([0.0, 90.0, -180.0, 0.0, 0.0, -180.0])
# our DH (robots/PAR6.py, standard DH): checked against the file's literals at start-up
DH_A = [0.1105, 0.02342, 0.180, 0.0435, 0.177]
DH_ALPHA = [-math.pi / 2, math.pi, math.pi / 2, -math.pi / 2, math.pi / 2, 0.0]
# (d, a) per link. "Frame 6" is the J6 JOINT frame — Pinocchio's oMi[6], BEFORE the gripper XML's own link-6 transform —
# so the tool's geometry does not depend on which gripper's <kinematics> is active (2026-10-06).
DH = [(DH_A[0], DH_A[1]), (0.0, DH_A[2]), (0.0, -DH_A[3]), (-DH_A[4], 0.0), (0.0, 0.0), (0.0, 0.0)]
COVER_TOL = 1e-4          # m: the covering proof's tolerance (float noise in coal's distances)
JSON_ROUND_TOL = 1e-5     # m: what rounding the stored pieces to 0.01 mm can cost (<= ~9 um per vertex; piece_json)


# ----------------------------------------------------------------------------- meshes
def read_stl(path):
    """Binary STL -> (n, 3, 3) triangle vertices, metres."""
    d = open(path, "rb").read()
    n = struct.unpack("<I", d[80:84])[0]
    if 84 + 50 * n != len(d):
        raise ValueError(f"{path}: not a binary STL (or truncated)")
    a = np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("p", "<u2")]))
    return np.ascontiguousarray(a["v"]).reshape(-1, 3, 3).astype(np.float64)


def link_meshes(urdf, pkg):
    """[(link name, triangles in the link frame)] for every link with collision geometry, in URDF order."""
    root = ET.parse(urdf).getroot()
    out = []
    for link in root.findall("link"):
        c = link.find("collision")
        if c is None or c.find("geometry/mesh") is None:
            continue
        fn = c.find("geometry/mesh").get("filename")
        path = fn.replace("package://", pkg + "/") if fn.startswith("package://") else os.path.join(os.path.dirname(urdf), fn)
        tris = read_stl(path)
        o = c.find("origin")
        if o is not None and (o.get("xyz", "0 0 0").split() != ["0", "0", "0"] or o.get("rpy", "0 0 0").split() != ["0", "0", "0"]):
            xyz = np.array([float(v) for v in o.get("xyz", "0 0 0").split()])
            R = rpy_matrix([float(v) for v in o.get("rpy", "0 0 0").split()])
            tris = tris @ R.T + xyz
        scale = c.find("geometry/mesh").get("scale")
        if scale:
            tris = tris * np.array([float(v) for v in scale.split()])
        out.append((link.get("name"), tris))
    return out


def rpy_matrix(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


# ----------------------------------------------------------------------------- capsule fitting
def capsule_for(V, axis):
    """The capsule along `axis` (unit) through V's centroid that contains every point of V (n, 3)."""
    c = V.mean(0)
    s = (V - c) @ axis
    perp = np.linalg.norm((V - c) - np.outer(s, axis), axis=1)
    p0, p1 = c + s.min() * axis, c + s.max() * axis
    return p0, p1, float(perp.max())


def capsule_volume(p0, p1, r):
    return math.pi * r * r * float(np.linalg.norm(p1 - p0)) + 4.0 / 3.0 * math.pi * r ** 3


def principal_axes(V):
    c = V.mean(0)
    return np.linalg.svd(V - c, full_matrices=False)[2], c


def mesh_volume(tris):
    """Enclosed volume of a closed mesh (divergence theorem), m^3."""
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    return abs(float(np.einsum("ij,ij->i", v0, np.cross(v1, v2)).sum())) / 6.0


def kmeans(points, k, rng, iters=40):
    """Plain k-means (k-means++ seeding). Returns the label of every point."""
    centres = [points[rng.integers(len(points))]]
    for _ in range(1, k):
        d2 = np.min(((points[:, None, :] - np.array(centres)[None]) ** 2).sum(-1), axis=1)
        centres.append(points[rng.choice(len(points), p=d2 / d2.sum())])
    C = np.array(centres)
    lab = np.zeros(len(points), dtype=int)
    for _ in range(iters):
        lab = np.argmin(((points[:, None, :] - C[None]) ** 2).sum(-1), axis=1)
        newC = np.array([points[lab == j].mean(0) if (lab == j).any() else C[j] for j in range(k)])
        if np.allclose(newC, C):
            break
        C = newC
    return lab


def subdivide(tris, max_edge=0.01):
    """Split triangles until no edge is longer than `max_edge` — the same surface, finer. The CAD meshes have single
    triangles spanning a whole face (150 mm on the base), and a covering rule stated per triangle would otherwise
    force one capsule to swallow the whole face."""
    tris = np.asarray(tris, float)
    while True:
        e = np.stack([np.linalg.norm(tris[:, 1] - tris[:, 0], axis=1), np.linalg.norm(tris[:, 2] - tris[:, 1], axis=1),
                      np.linalg.norm(tris[:, 0] - tris[:, 2], axis=1)], axis=1).max(axis=1)
        big = e > max_edge
        if not big.any():
            return tris
        T = tris[big]
        m01, m12, m20 = 0.5 * (T[:, 0] + T[:, 1]), 0.5 * (T[:, 1] + T[:, 2]), 0.5 * (T[:, 2] + T[:, 0])
        parts = [np.stack([T[:, 0], m01, m20], 1), np.stack([m01, T[:, 1], m12], 1),
                 np.stack([m20, m12, T[:, 2]], 1), np.stack([m01, m12, m20], 1)]
        tris = np.concatenate([tris[~big]] + parts, axis=0)


def surface_points(tris, n, rng):
    """n points sampled evenly (by area) over the triangles. The CAD vertices alone sit on edges and fillets."""
    area = 0.5 * np.linalg.norm(np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0]), axis=1)
    idx = rng.choice(len(tris), size=n, p=area / area.sum())
    u, v = rng.random(n), rng.random(n)
    flip = u + v > 1
    u[flip], v[flip] = 1 - u[flip], 1 - v[flip]
    t = tris[idx]
    return t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0])


def best_capsule(V):
    """The smallest-volume capsule around V among the three principal axes (PCA's first axis is not always it)."""
    axes, _ = principal_axes(V)
    return min((capsule_for(V, ax) for ax in axes), key=lambda c: capsule_volume(*c))


def capsule_gap(c1, c2):
    """Signed gap between two capsules (m): < 0 = they overlap."""
    W0 = np.array([c1[0], c2[0]]); W1 = np.array([c1[1], c2[1]])
    out = np.zeros(1)
    _segment_gap_py(W0, W1, np.array([0]), np.array([1]), np.array([c1[2] + c2[2]]), out)
    return float(out[0])


def merge_fit(pts, k0=24, max_caps=6, within=1.3, rng=None, verbose=False, hull_volume=None):
    """Bottom-up: k0 small groups of surface points, each its smallest capsule; merge the two NEIGHBOURING
    capsules whose union adds the least volume, until one capsule is left; keep the whole sequence and pick the
    fewest capsules (<= max_caps) whose total volume is within `within` x the finest fit's. Returns [(points of
    the group, capsule)] for the chosen level."""
    rng = rng or np.random.default_rng(0)
    lab = kmeans(pts, k0, rng)
    groups = [pts[lab == j] for j in range(k0) if (lab == j).any()]
    caps = [best_capsule(g) for g in groups]
    levels = {len(caps): (list(groups), list(caps), sum(capsule_volume(*c) for c in caps))}
    while len(caps) > 1:
        best = None
        for i in range(len(caps)):
            for j in range(i + 1, len(caps)):
                if capsule_gap(caps[i], caps[j]) > 0.01:        # only neighbours (within 10 mm) may merge
                    continue
                merged = best_capsule(np.vstack([groups[i], groups[j]]))
                added = capsule_volume(*merged) - capsule_volume(*caps[i]) - capsule_volume(*caps[j])
                if best is None or added < best[0]:
                    best = (added, i, j, merged)
        if best is None:                                        # no neighbours left: merge the two closest anyway
            pairs = [(capsule_gap(caps[i], caps[j]), i, j) for i in range(len(caps)) for j in range(i + 1, len(caps))]
            _, i, j = min(pairs)
            best = (0.0, i, j, best_capsule(np.vstack([groups[i], groups[j]])))
        _, i, j, merged = best
        groups = [g for k, g in enumerate(groups) if k not in (i, j)] + [np.vstack([groups[i], groups[j]])]
        caps = [c for k, c in enumerate(caps) if k not in (i, j)] + [merged]
        levels[len(caps)] = (list(groups), list(caps), sum(capsule_volume(*c) for c in caps))
    finest = min(v for _, _, v in levels.values())
    ok = [k for k, (_, _, v) in levels.items() if k <= max_caps and v <= within * finest]
    k = min(ok) if ok else max_caps
    if verbose:
        ref = hull_volume or finest
        print("      capsule volume / convex hull per count: " + ", ".join(
            f"{kk}:{levels[kk][2] / ref:.1f}x" for kk in sorted(levels) if kk <= max(max_caps, 8)))
    return list(zip(levels[k][0], levels[k][1]))


def grow_to_cover(V, cap):
    """The capsule along `cap`'s axis, through its centre line, that contains every point of V (n, 3)."""
    p0, p1, r = cap
    axis = p1 - p0
    axis = axis / np.linalg.norm(axis) if np.linalg.norm(axis) > 1e-9 else np.array([0.0, 0.0, 1.0])
    c = 0.5 * (p0 + p1)
    s = (V - c) @ axis
    perp = np.linalg.norm((V - c) - np.outer(s, axis), axis=1)
    half = 0.5 * float(np.linalg.norm(p1 - p0))
    return c + min(s.min(), -half) * axis, c + max(s.max(), half) * axis, max(r, float(perp.max()))


def fit_link(tris, max_caps=6, within=1.3, seed=0, verbose=False, n_points=4000, max_edge=0.005):
    """Capsules that contain the whole surface of `tris`: the merge fit on surface samples, then every vertex of
    the refined mesh assigned to its nearest capsule, that capsule grown to hold them, every radius inflated by
    max_edge / sqrt(3). Returns (capsules, total volume, hull volume, refined triangles)."""
    rng = np.random.default_rng(seed)
    pts = surface_points(tris, n_points, rng)
    from scipy.spatial import ConvexHull
    hull_vol = float(ConvexHull(tris.reshape(-1, 3)).volume)
    fitted = merge_fit(pts, max_caps=max_caps, within=within, rng=rng, verbose=verbose, hull_volume=hull_vol)
    fine = subdivide(tris, max_edge)
    V = np.unique(fine.reshape(-1, 3), axis=0)
    dist = np.stack([point_segment_distance(V, c[0], c[1]) - c[2] for _, c in fitted], axis=1)
    lab = np.argmin(dist, axis=1)
    caps = []
    for j, (_, c) in enumerate(fitted):
        sel = lab == j
        if sel.any():
            p0, p1, r = grow_to_cover(V[sel], c)
            caps.append((p0, p1, r + max_edge / math.sqrt(3.0)))
    return caps, sum(capsule_volume(*c) for c in caps), hull_vol, fine


def covers(tris, caps, tol=1e-9):
    """Is every vertex inside some capsule? With edges <= max_edge and radii inflated by max_edge / sqrt(3), that
    puts every point of every triangle inside the union. Returns (ok, vertices outside)."""
    V = tris.reshape(-1, 3)
    inside = np.zeros(len(V), dtype=bool)
    for p0, p1, r in caps:
        inside |= point_segment_distance(V, p0, p1) <= r + tol
    return bool(inside.all()), int((~inside).sum())


def point_segment_distance(P, a, b):
    ab = b - a
    t = np.clip(((P - a) @ ab) / max(float(ab @ ab), 1e-18), 0.0, 1.0)
    return np.linalg.norm(P - (a + np.outer(t, ab)), axis=1)


# ----------------------------------------------------------------------------- capsule distance (the robot's check)
def _segment_gap_py(W0, W1, IA, IB, RSUM, out):
    for p in range(IA.shape[0]):
        a_, b_ = IA[p], IB[p]
        d1x, d1y, d1z = W1[a_, 0] - W0[a_, 0], W1[a_, 1] - W0[a_, 1], W1[a_, 2] - W0[a_, 2]
        d2x, d2y, d2z = W1[b_, 0] - W0[b_, 0], W1[b_, 1] - W0[b_, 1], W1[b_, 2] - W0[b_, 2]
        rx, ry, rz = W0[a_, 0] - W0[b_, 0], W0[a_, 1] - W0[b_, 1], W0[a_, 2] - W0[b_, 2]
        a = d1x * d1x + d1y * d1y + d1z * d1z
        e = d2x * d2x + d2y * d2y + d2z * d2z
        f = d2x * rx + d2y * ry + d2z * rz
        c = d1x * rx + d1y * ry + d1z * rz
        b = d1x * d2x + d1y * d2y + d1z * d2z
        denom = a * e - b * b
        s = 0.0
        if denom > 1e-14:
            s = min(max((b * f - c * e) / denom, 0.0), 1.0)
        t = (b * s + f) / e if e > 1e-14 else 0.0
        if t < 0.0:
            t = 0.0
            s = min(max(-c / a, 0.0), 1.0) if a > 1e-14 else 0.0
        elif t > 1.0:
            t = 1.0
            s = min(max((b - c) / a, 0.0), 1.0) if a > 1e-14 else 0.0
        px = W0[a_, 0] + s * d1x - (W0[b_, 0] + t * d2x)
        py = W0[a_, 1] + s * d1y - (W0[b_, 1] + t * d2y)
        pz = W0[a_, 2] + s * d1z - (W0[b_, 2] + t * d2z)
        out[p] = math.sqrt(px * px + py * py + pz * pz) - RSUM[p]


try:
    import numba
    segment_gaps = numba.njit(cache=True, fastmath=True)(_segment_gap_py)
except Exception:                                      # numba missing: the pure-Python loop (slow, same numbers)
    segment_gaps = _segment_gap_py


class CapsuleModel:
    """Capsules per link (link frame) + the non-adjacent pair list; gaps for a set of link frames."""

    def __init__(self, links, caps_per_link, adjacency):
        # links: [name]; caps_per_link: {name: [(p0, p1, r)]}; adjacency: {name: joint index} (consecutive = adjacent)
        self.links = list(links)
        self.caps = []
        for li, name in enumerate(self.links):
            for p0, p1, r in caps_per_link.get(name, []):
                self.caps.append((li, np.asarray(p0, float), np.asarray(p1, float), float(r)))
        self.cap_link = np.array([c[0] for c in self.caps], dtype=np.int64)
        self.P0 = np.array([c[1] for c in self.caps]); self.P1 = np.array([c[2] for c in self.caps])
        self.R = np.array([c[3] for c in self.caps])
        ia, ib, pair_of = [], [], []
        self.link_pairs = [(a, b) for a in range(len(self.links)) for b in range(a + 1, len(self.links))
                           if abs(adjacency[self.links[a]] - adjacency[self.links[b]]) >= 2]
        pair_index = {p: i for i, p in enumerate(self.link_pairs)}
        for i in range(len(self.caps)):
            for j in range(i + 1, len(self.caps)):
                key = (min(self.cap_link[i], self.cap_link[j]), max(self.cap_link[i], self.cap_link[j]))
                if key in pair_index:
                    ia.append(i); ib.append(j); pair_of.append(pair_index[key])
        self.IA, self.IB = np.array(ia, dtype=np.int64), np.array(ib, dtype=np.int64)
        self.pair_of = np.array(pair_of, dtype=np.int64)
        self.RSUM = self.R[self.IA] + self.R[self.IB]
        self._W0, self._W1 = np.zeros_like(self.P0), np.zeros_like(self.P1)
        self._gaps = np.zeros(len(self.IA))
        self._pair_gap = np.zeros(len(self.link_pairs))

    def gaps(self, Rw, tw):
        """Per LINK pair: the smallest capsule gap (m), given each link's rotation (n,3,3) and translation (n,3)."""
        np.einsum("kij,kj->ki", Rw[self.cap_link], self.P0, out=self._W0); np.add(self._W0, tw[self.cap_link], out=self._W0)
        np.einsum("kij,kj->ki", Rw[self.cap_link], self.P1, out=self._W1); np.add(self._W1, tw[self.cap_link], out=self._W1)
        segment_gaps(self._W0, self._W1, self.IA, self.IB, self.RSUM, self._gaps)
        self._pair_gap.fill(np.inf)
        np.minimum.at(self._pair_gap, self.pair_of, self._gaps)
        return self._pair_gap


# ----------------------------------------------------------------------------- convex pieces (the robot's check)
def convex_pieces(tris, k, rng, n_points=4000, max_edge=None):
    """k convex pieces of a mesh: k-means groups of surface samples; every triangle joins the nearest group with
    all three vertices; piece = (hull vertices (m,3), hull faces (f,3) into them). The union contains the surface.
    max_edge (m): split the triangles longer than this FIRST (2026-10-07) — a CAD triangle can span a whole face, and
    the group it joins then swallows the face: measured on the upper arm / wrist 1 pair, splitting at 20 mm took the
    slack at the folded elbow from 6-10 mm median / 18-24 mm worst to ~0 / 3.5-7 mm, with the same 6 pieces, where 8,
    10 or 12 pieces without it changed nothing."""
    if max_edge:
        tris = subdivide(tris, max_edge)
    pts = surface_points(tris, n_points, rng)
    lab = kmeans(pts, k, rng) if k > 1 else np.zeros(len(pts), dtype=int)
    centres = np.array([pts[lab == j].mean(0) for j in range(k) if (lab == j).any()])
    tl = np.argmin(((tris.mean(1)[:, None, :] - centres[None]) ** 2).sum(-1), axis=1)
    from scipy.spatial import ConvexHull
    out = []
    for j in range(len(centres)):
        V = np.unique(tris[tl == j].reshape(-1, 3), axis=0)
        if len(V) < 4:
            continue
        hull = ConvexHull(V)
        remap = {old: new for new, old in enumerate(hull.vertices)}
        faces = np.array([[remap[a], remap[b], remap[c]] for a, b, c in hull.simplices], dtype=np.int64)
        out.append((V[hull.vertices], faces))
    return out


def points_in_hull(P, verts, faces, tol=1e-7):
    """Which points lie inside the convex hull (verts, faces)? Via the hull's face planes (outward normals)."""
    c = verts.mean(0)
    A, B, Cc = verts[faces[:, 0]], verts[faces[:, 1]], verts[faces[:, 2]]
    n = np.cross(B - A, Cc - A); n /= np.maximum(np.linalg.norm(n, axis=1), 1e-18)[:, None]
    flip = np.einsum("ij,ij->i", n, A - c) < 0
    n[flip] *= -1
    d = np.einsum("ij,ij->i", n, A)
    return (P @ n.T - d <= tol).all(axis=1)


def convex_volume(verts, faces):
    """Volume of a convex hull: tetrahedra from an interior point to every face (orientation-free)."""
    c = verts.mean(0)
    A, B, Cc = verts[faces[:, 0]] - c, verts[faces[:, 1]] - c, verts[faces[:, 2]] - c
    return float(np.abs(np.einsum("ij,ij->i", A, np.cross(B, Cc))).sum()) / 6.0


def coal_convex(verts, faces):
    """A coal Convex from hull vertices and faces."""
    pts = coal.StdVec_Vec3s(); pts.extend([np.asarray(v, float) for v in verts])
    tl = coal.StdVec_Triangle(); tl.extend([coal.Triangle(int(a), int(b), int(c)) for a, b, c in faces])
    return coal.Convex(pts, tl)


class ConvexModel:
    """The pieces check as the robot would run it: every piece has a covering CAPSULE (it contains the piece, so
    the capsule gap is a lower bound on the piece gap); all piece pairs' capsule gaps in one numba call, then
    coal's GJK distance only for the pairs whose bound is under `margin`. gaps() returns, per non-adjacent LINK
    pair, the exact gap when it is under the margin and a lower bound (>= margin) otherwise."""

    def __init__(self, links, pieces_per_link, adjacency):
        self.links = list(links)
        self.pieces, self.shapes, piece_link, P0, P1, R = [], [], [], [], [], []
        for li, name in enumerate(self.links):
            for verts, faces in pieces_per_link.get(name, []):
                p0, p1, r = best_capsule(verts)                   # contains the hull's vertices, so the whole piece
                self.pieces.append((li, verts, faces)); self.shapes.append(coal_convex(verts, faces))
                piece_link.append(li); P0.append(p0); P1.append(p1); R.append(r)
        self.piece_link = np.array(piece_link, dtype=np.int64); self.P0 = np.array(P0); self.P1 = np.array(P1); self.R = np.array(R)
        self.link_pairs = [(a, b) for a in range(len(self.links)) for b in range(a + 1, len(self.links))
                           if abs(adjacency[self.links[a]] - adjacency[self.links[b]]) >= 2]
        idx = {p: i for i, p in enumerate(self.link_pairs)}
        ia, ib, pof = [], [], []
        for i in range(len(self.pieces)):
            for j in range(i + 1, len(self.pieces)):
                key = (min(self.piece_link[i], self.piece_link[j]), max(self.piece_link[i], self.piece_link[j]))
                if key in idx:
                    ia.append(i); ib.append(j); pof.append(idx[key])
        order = np.argsort(pof, kind="stable")                   # grouped by link pair, for reduceat
        self.IA, self.IB, self.pair_of = np.array(ia)[order], np.array(ib)[order], np.array(pof)[order]
        self._starts = np.concatenate([[0], np.flatnonzero(np.diff(self.pair_of)) + 1])
        self.RSUM = self.R[self.IA] + self.R[self.IB]
        self._W0, self._W1 = np.zeros_like(self.P0), np.zeros_like(self.P1)
        self._lb = np.zeros(len(self.IA)); self._gap = np.zeros(len(self.link_pairs))
        self._req, self._res = coal.DistanceRequest(), coal.DistanceResult()
        self.gjk_calls = 0

    def gaps(self, Rw, tw, margin):
        np.einsum("kij,kj->ki", Rw[self.piece_link], self.P0, out=self._W0); np.add(self._W0, tw[self.piece_link], out=self._W0)
        np.einsum("kij,kj->ki", Rw[self.piece_link], self.P1, out=self._W1); np.add(self._W1, tw[self.piece_link], out=self._W1)
        segment_gaps(self._W0, self._W1, self.IA, self.IB, self.RSUM, self._lb)   # lower bound per piece pair
        self._gap[:] = np.minimum.reduceat(self._lb, self._starts)              # per link pair: the smallest bound
        near = np.flatnonzero(self._lb < margin)
        self.gjk_calls = len(near)
        if len(near):
            exact = np.full(len(self.link_pairs), np.inf)
            for k in near:
                i, j = self.IA[k], self.IB[k]
                Ti = coal.Transform3s(Rw[self.piece_link[i]], tw[self.piece_link[i]])
                Tj = coal.Transform3s(Rw[self.piece_link[j]], tw[self.piece_link[j]])
                self._res.clear(); coal.distance(self.shapes[i], Ti, self.shapes[j], Tj, self._req, self._res)
                g = self._res.min_distance
                if g < exact[self.pair_of[k]]:
                    exact[self.pair_of[k]] = g
            for pi in np.unique(self.pair_of[near]):
                # the exact gap if it is under the margin; else at least the margin (the piece pairs not queried are too)
                self._gap[pi] = exact[pi] if exact[pi] < margin else max(margin, self._gap[pi])
        return self._gap


# ----------------------------------------------------------------------------- the URDF model (Pinocchio + coal)
def _prismatic_axis(urdf, joint_name):
    """The <axis> of a prismatic joint in the URDF (unit vector in the child link's frame), None otherwise."""
    for jt in ET.parse(urdf).getroot().findall("joint"):
        if jt.get("name") == joint_name and jt.get("type") == "prismatic":
            a = np.array([float(v) for v in (jt.find("axis").get("xyz") if jt.find("axis") is not None else "1 0 0").split()])
            return a / np.linalg.norm(a)
    return None


def swept_piece(verts, travel):
    """A convex piece swept along a straight travel: the hull of the piece at both ends (contains every position)."""
    from scipy.spatial import ConvexHull
    P = np.vstack([verts, np.asarray(verts) + travel])
    hull = ConvexHull(P)
    remap = {old: new for new, old in enumerate(hull.vertices)}
    return P[hull.vertices], np.array([[remap[a], remap[b], remap[c]] for a, b, c in hull.simplices], dtype=np.int64)


def piece_json(P, decimals=5):
    """A piece for the JSON files: its vertices rounded (to 0.01 mm) and its faces REBUILT as the hull of the rounded
    vertices. Rounding moves each vertex by at most ~9 um, so the rounded hull lies within that of the exact one; but the
    exact hull's faces, kept with rounded vertices, are not quite convex any more (a sliver face tilts — tests on its
    planes then read up to 0.3 mm "outside", 2026-10-07)."""
    from scipy.spatial import ConvexHull
    V = np.unique(np.round(np.asarray(P, float), decimals), axis=0)
    hull = ConvexHull(V)
    remap = {old: new for new, old in enumerate(hull.vertices)}
    return {"verts": V[hull.vertices].tolist(), "faces": [[remap[a], remap[b], remap[c]] for a, b, c in hull.simplices]}


def link_rng(seed, name):
    """The random numbers for one link's fit: the run's seed + the link's name, so one link's change leaves the rest."""
    return np.random.default_rng([int(seed), zlib.crc32(name.encode())])


def parse_pieces_for(text):
    """"upper_arm_link=8,wrist_1_link=8" -> {name: 8}."""
    out = {}
    for item in filter(None, (t.strip() for t in (text or "").split(","))):
        name, _, k = item.partition("=")
        out[name.strip()] = int(k)
    return out


def jaw_open_ends(um, jaw_tris):
    """For a gripper's jaws: which end of the travel is OPEN. jaw_tris: {jaw link: its triangles (link frame)}.
    The jaws close toward each other, so the open end is where their centroids are farthest apart (in the tool
    link's frame). Returns {jaw link: (open_at "zero" | "far", close_dir in the TOOL LINK frame (unit), stroke m)}
    and the two centroid distances (m) at URDF 0 and at the far end."""
    cs, ds = {}, {}
    for name, tris in jaw_tris.items():
        j = um.link_joint[name]
        F = um.tool_fold[j]
        cs[name] = F[:3, :3] @ tris.reshape(-1, 3).mean(0) + F[:3, 3]
        ds[name] = F[:3, :3] @ um.jaw_travel[j]
    names = list(jaw_tris)
    if len(names) != 2:
        far_closes = True                                  # one jaw: assume the stroke ends closed (logged by the caller)
        d0 = d1 = float("nan")
    else:
        a, b = names
        d0 = float(np.linalg.norm(cs[a] - cs[b]))
        d1 = float(np.linalg.norm((cs[a] + ds[a]) - (cs[b] + ds[b])))
        far_closes = d1 < d0
    out = {}
    for name in names:
        stroke = float(np.linalg.norm(ds[name]))
        u = ds[name] / stroke
        out[name] = ("zero", u, stroke) if far_closes else ("far", -u, stroke)
    return out, d0, d1


class UrdfModel:
    def __init__(self, urdf, pkg):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.geom = pin.buildGeomFromUrdf(self.model, urdf, pin.GeometryType.COLLISION, package_dirs=[pkg])
        # a joint beyond the six arm joints (a gripper's prismatic jaws) stays at 0: its link is part of the tool.
        # tool_fold[j] = the placement of joint j's link in the TOOL LINK's frame (4x4) — the link with geometry on
        # joint 6, which in package 2.0 hangs on fixed joints (wrist_3_link -> flange -> gripper, a 180 deg turn about
        # y), so it is NOT joint 6's own frame. (Collision origins are identity in these URDFs: link frame = geometry.)
        tool_geo = next((g for g in self.geom.geometryObjects if g.parentJoint == 6), None)
        H = tool_geo.placement if tool_geo is not None else pin.SE3.Identity()
        self.tool_fold = {}
        for j in range(7, self.model.njoints):
            M = pin.SE3.Identity(); jj = j
            while jj > 6:                                      # walk up to joint 6 composing the (zero) placements
                M = self.model.jointPlacements[jj] * M; jj = self.model.parents[jj]
            self.tool_fold[j] = (H.inverse() * M).homogeneous
        # each jaw's stroke: the travel (m, in the jaw link's frame) from 0 to its far limit
        self.jaw_travel = {}
        for j in range(7, self.model.njoints):
            lo_, hi_ = float(self.model.lowerPositionLimit[self.model.joints[j].idx_q]), float(self.model.upperPositionLimit[self.model.joints[j].idx_q])
            far = lo_ if abs(lo_) > abs(hi_) else hi_
            ax = _prismatic_axis(urdf, self.model.names[j])
            self.jaw_travel[j] = None if ax is None or not np.isfinite(far) else ax * far
        self.geom.addAllCollisionPairs()
        # link name per geometry, joint index per link; drop adjacent pairs
        self.geo_link = [g.name.rsplit("_", 1)[0] for g in self.geom.geometryObjects]
        self.link_joint = {self.geo_link[i]: g.parentJoint for i, g in enumerate(self.geom.geometryObjects)}
        for cp in list(self.geom.collisionPairs):
            a, b = self.geom.geometryObjects[cp.first].parentJoint, self.geom.geometryObjects[cp.second].parentJoint
            if abs(a - b) < 2:
                self.geom.removeCollisionPair(cp)
        self.gdata = pin.GeometryData(self.geom)
        self.pairs = [(self.geo_link[cp.first], self.geo_link[cp.second]) for cp in self.geom.collisionPairs]
        self.nq = self.model.nq
        # HOSTS: the links of OUR model. A jaw's geometry is folded into the tool link (joint 6), so the fitted
        # model has one entry per host; the mesh truth keeps the jaws apart and is reduced per host pair.
        self.host_of = {n: (next(h for h in self.geo_link if self.link_joint[h] == 6) if self.link_joint[n] > 6 else n)
                        for n in self.geo_link}
        self.hosts = list(dict.fromkeys(self.host_of[n] for n in self.geo_link))
        self.host_rows = [self.geo_link.index(h) for h in self.hosts]
        self.host_adjacency = {h: self.link_joint[h] for h in self.hosts}
        self.host_pairs = [(a, b) for a in range(len(self.hosts)) for b in range(a + 1, len(self.hosts))
                           if abs(self.link_joint[self.hosts[a]] - self.link_joint[self.hosts[b]]) >= 2]
        hp_index = {p: i for i, p in enumerate(self.host_pairs)}
        # mesh pair -> host pair index (None for a pair inside one host, e.g. jaw vs jaw)
        self.mesh_to_host = []
        for a, b in self.pairs:
            ha, hb = self.hosts.index(self.host_of[a]), self.hosts.index(self.host_of[b])
            self.mesh_to_host.append(hp_index.get((min(ha, hb), max(ha, hb))))

    def q_from_ours(self, ours_deg, jaw_far=0.0):
        """Our six angles (deg) -> a Pinocchio configuration; jaws (if any) at `jaw_far` x their far limit (0 = URDF 0,
        1 = the far end of the stroke). A `continuous` URDF joint is two numbers (cos, sin) in Pinocchio, a revolute /
        prismatic one is one."""
        th = np.radians(URDF_SIGN * np.asarray(ours_deg, float) + URDF_OFFSET_DEG)
        q = self.pin.neutral(self.model)
        for j in range(1, self.model.njoints):
            far = 0.0
            if j > 6 and self.jaw_travel.get(j) is not None:
                lo_, hi_ = float(self.model.lowerPositionLimit[self.model.joints[j].idx_q]), float(self.model.upperPositionLimit[self.model.joints[j].idx_q])
                far = lo_ if abs(lo_) > abs(hi_) else hi_
            jm, v = self.model.joints[j], (th[j - 1] if j <= 6 else jaw_far * far)
            if jm.nq == 2:
                q[jm.idx_q:jm.idx_q + 2] = (math.cos(v), math.sin(v))
            else:
                q[jm.idx_q] = v
        return q

    def tool_frame_of(self, name):
        """(frame index for OUR model, 4x4 fold into that frame): a jaw folds into link 6; an arm link is itself."""
        j = self.link_joint[name]
        if j > 6:
            return 6, self.tool_fold[j]
        return j, np.eye(4)

    def frames(self, q):
        """Rotation (n,3,3) and translation (n,3) of every link with geometry, in geometry order."""
        self.pin.forwardKinematics(self.model, self.data, q)
        n = len(self.geo_link)
        Rw, tw = np.zeros((n, 3, 3)), np.zeros((n, 3))
        for i, g in enumerate(self.geom.geometryObjects):
            M = self.data.oMi[g.parentJoint] * g.placement if g.parentJoint > 0 else g.placement
            Rw[i], tw[i] = M.rotation, M.translation
        return Rw, tw

    def mesh_distances(self, q):
        """Per non-adjacent pair: coal's exact mesh distance (m), in self.pairs order."""
        self.pin.computeDistances(self.model, self.data, self.geom, self.gdata, q)
        return np.array([self.gdata.distanceResults[k].min_distance for k in range(len(self.pairs))])

    def host_frames(self, q):
        """frames() reduced to the hosts, in self.hosts order."""
        Rw, tw = self.frames(q)
        return Rw[self.host_rows], tw[self.host_rows]

    def host_mesh_distances(self, q):
        """The mesh truth per HOST pair (self.host_pairs order): the smallest distance over its mesh pairs."""
        dm = self.mesh_distances(q)
        out = np.full(len(self.host_pairs), np.inf)
        for i, hp in enumerate(self.mesh_to_host):
            if hp is not None and dm[i] < out[hp]:
                out[hp] = dm[i]
        return out


def pinocchio_native_cost(um, pieces_per_link, margin, Q):
    """Seconds per pose for the pieces check done entirely in C++: Pinocchio's computeCollisions over a GeometryModel
    of the pieces with `margin` as coal's security margin — plain (all pairs) and with the AABB-tree broadphase."""
    pin = um.pin
    gm = pin.GeometryModel(); joints = []
    for name in um.hosts:
        for k, (verts, faces) in enumerate(pieces_per_link.get(name, [])):
            gm.addGeometryObject(pin.GeometryObject(f"{name}_p{k}", um.link_joint[name], pin.SE3.Identity(), coal_convex(verts, faces)))
            joints.append(um.link_joint[name])
    for i in range(gm.ngeoms):
        for j in range(i + 1, gm.ngeoms):
            if abs(joints[i] - joints[j]) >= 2:
                gm.addCollisionPair(pin.CollisionPair(i, j))
    gd = pin.GeometryData(gm)
    for k in range(len(gm.collisionPairs)):
        gd.collisionRequests[k].security_margin = margin
    data = um.model.createData()
    pin.computeCollisions(um.model, data, gm, gd, Q[0], False)
    t0 = time.perf_counter()
    for q in Q:
        pin.computeCollisions(um.model, data, gm, gd, q, False)
    t_plain = (time.perf_counter() - t0) / len(Q)
    mgr = pin.TreeBroadPhaseManager_DynamicAABBTreeCollisionManager(um.model, gm, gd)
    cb = pin.CollisionCallBackDefault(gm, gd)
    def bp(q):
        pin.forwardKinematics(um.model, data, q); pin.updateGeometryPlacements(um.model, data, gm, gd)
        mgr.update(); pin.computeCollisions(mgr, cb)
    bp(Q[0]); t0 = time.perf_counter()
    for q in Q:
        bp(q)
    return t_plain, (time.perf_counter() - t0) / len(Q)


def soft_limits_deg():
    """Our soft limits from robots/PAR6.xml (degrees)."""
    from project_paths import ROBOTS_DIR
    root = ET.parse(os.path.join(ROBOTS_DIR, "PAR6.xml")).getroot()
    lo = [math.degrees(float(j.find("software_joint_limit_negative").text)) for j in root.iter("joint") if j.find("software_joint_limit_negative") is not None]
    hi = [math.degrees(float(j.find("software_joint_limit_positive").text)) for j in root.iter("joint") if j.find("software_joint_limit_positive") is not None]
    return np.array(lo[:6]), np.array(hi[:6])


def check_dh_literals():
    """Warn if robots/PAR6.py's a1..a5 differ from the constants here."""
    src = open(os.path.join(_ROOT, "robots", "PAR6.py")).read()
    vals = {k: float(v) / 1000 for k, v in re.findall(r"\n\s*(a[1-5])\s*=\s*([\d.]+)\s*/\s*1000", src)}
    mine = dict(zip(["a1", "a2", "a3", "a4", "a5"], DH_A))
    bad = [k for k in mine if k in vals and abs(vals[k] - mine[k]) > 1e-9]
    if bad:
        print(f"WARNING: robots/PAR6.py has other link lengths for {bad}: {[vals[k] for k in bad]} — update DH_A here")


def dh_frames(ours_deg):
    """Our DH frames: [world->frame i] for i = 0 (base) .. 6, standard DH as robots/PAR6.py builds it."""
    q = np.radians(ours_deg)
    M = np.eye(4); out = [M.copy()]
    for i in range(6):
        d, a = DH[i]; al = DH_ALPHA[i]
        ct, st, ca, sa = math.cos(q[i]), math.sin(q[i]), math.cos(al), math.sin(al)
        M = M @ np.array([[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0, sa, ca, d], [0, 0, 0, 1]])
        out.append(M.copy())
    return out


def urdf_to_dh_transforms(um: UrdfModel, rng):
    """Per link: the FIXED transform T with  (URDF link frame) = (our DH frame) @ T  — checked constant over poses."""
    lo, hi = soft_limits_deg()
    T_all = {name: [] for name in um.geo_link}
    for _ in range(12):
        ours = rng.uniform(lo, hi)
        Rw, tw = um.frames(um.q_from_ours(ours))
        dh = dh_frames(ours)
        for i, name in enumerate(um.geo_link):
            j = um.link_joint[name]                       # link after joint j -> our DH frame j (base: 0)
            if j > 6:
                continue                                  # the MSG jaws ride on link 6 — handled below
            Tu = np.eye(4); Tu[:3, :3], Tu[:3, 3] = Rw[i], tw[i]
            T_all[name].append(np.linalg.inv(dh[j]) @ Tu)
    out, worst = {}, 0.0
    for name, Ts in T_all.items():
        if not Ts:
            continue
        T = np.mean(Ts, axis=0)
        dev = max(np.abs(Tk - T).max() for Tk in Ts)
        worst = max(worst, dev)
        out[name] = (T, um.link_joint[name])
    return out, worst


# ----------------------------------------------------------------------------- commands
def cmd_fit(args):
    os.makedirs(OUT_DIR, exist_ok=True)
    urdf, pkg = URDF[args.gripper]
    meshes = link_meshes(urdf, pkg)
    report, caps_per_link, pieces_per_link = [], {}, {}
    pieces_for = parse_pieces_for(args.pieces_for)
    chosen_seed = {}
    unknown = set(pieces_for) - {n for n, _ in meshes}
    if unknown:
        print(f"--pieces-for names no link of this URDF: {sorted(unknown)} (links: {[n for n, _ in meshes]})"); return 1
    for name, tris in meshes:
        caps, vol, hull, fine = fit_link(tris, args.max_caps, within=args.within, verbose=args.verbose,
                                         max_edge=args.max_edge / 1000.0)
        ok, missing = covers(fine, caps)
        caps_per_link[name] = caps
        k = pieces_for.get(name, 1 if len(tris) < 1500 else args.pieces)
        # k-means depends on its random start: try a few seeds and keep the smallest total volume (the least conservative
        # model; deterministic for a given --seed)
        best = None
        for i in range(max(1, args.seeds)):
            cand = convex_pieces(tris, k, link_rng(args.seed + i, name), max_edge=args.piece_edge / 1000.0 if args.piece_edge > 0 else None)
            vol_c = sum(convex_volume(v, f) for v, f in cand)
            if best is None or vol_c < best[0]:
                best = (vol_c, cand, args.seed + i)
        pcs = best[1]
        chosen_seed[name] = best[2]
        pieces_per_link[name] = pcs
        V = np.unique(fine.reshape(-1, 3), axis=0)
        inside = np.zeros(len(V), dtype=bool)
        for verts, faces in pcs:
            inside |= points_in_hull(V, verts, faces)
        pv = sum(convex_volume(v, f) for v, f in pcs)
        vm = mesh_volume(tris)
        report.append(f"  {name:12s} {len(tris):6d} tris -> {len(pcs)} convex piece(s) (seed {chosen_seed[name]}, {sum(len(v) for v, _ in pcs)} hull vertices, "
                      f"volume {pv / vm:.1f}x the mesh, {pv / hull:.2f}x its convex hull; contain every vertex: "
                      f"{'YES' if inside.all() else f'NO ({int((~inside).sum())} outside)'}) | {len(caps)} capsule(s): "
                      + ", ".join(f"L {1e3 * np.linalg.norm(p1 - p0):.0f} r {1e3 * r:.0f}" for p0, p1, r in caps)
                      + f" mm (volume {vol / hull:.2f}x the hull; contain every vertex: {'YES' if ok else f'NO ({missing} outside)'})")
    print(f"fit ({args.gripper}):\n" + "\n".join(report))
    if not all(ln.count("YES") == 2 for ln in report):
        print("A link is NOT contained — the fit is wrong; nothing written."); return 1
    um = UrdfModel(urdf, pkg)
    swept, jaw_info, open_pieces = [], {}, {}
    jaw_tris = {name: tris for name, tris in meshes if um.link_joint[name] > 6 and um.jaw_travel.get(um.link_joint[name]) is not None}
    if jaw_tris:
        jaw_info, d0, d1 = jaw_open_ends(um, jaw_tris)
        print(f"jaws: centroids {1e3 * d0:.1f} mm apart at URDF 0, {1e3 * d1:.1f} mm at the far end -> OPEN at "
              f"{'URDF 0' if d1 < d0 else 'the far end'}" + ("" if len(jaw_tris) == 2 else " (not two jaws: assumed open at URDF 0)"))
    for name in list(pieces_per_link):
        j = um.link_joint[name]
        if j > 6 and um.jaw_travel.get(j) is not None:
            open_at, _, _ = jaw_info[name]
            shift = um.jaw_travel[j] if open_at == "far" else np.zeros(3)
            open_pieces[name] = [(v + shift, f) for v, f in pieces_per_link[name]]          # unswept, at the open end
            pieces_per_link[name] = [swept_piece(v, um.jaw_travel[j]) for v, _ in pieces_per_link[name]]
            swept.append(f"{name} {1e3 * np.linalg.norm(um.jaw_travel[j]):.1f} mm")
    if swept:
        print("jaw pieces swept over the whole stroke: " + ", ".join(swept) + "; also kept unswept at the open end")
    T_fix, worst = urdf_to_dh_transforms(um, np.random.default_rng(0))
    print(f"URDF link frame -> our DH frame: fixed transforms found, worst variation over 12 poses {1e3 * worst:.3f} mm"
          + ("" if worst < 1e-4 else "  <-- NOT constant: the joint mapping or the DH table is wrong")
          + ("  (the URDF's rounded angles, 1.5708 for pi/2)" if 1e-6 <= worst < 1e-4 else ""))

    def to_dh(name, P):
        T, _ = T_fix[name]
        return np.asarray(P, float) @ T[:3, :3].T + T[:3, 3]
    model = {"source": urdf, "generated": time.strftime("%Y-%m-%d %H:%M"), "frames": "URDF link frames (_urdf) and our DH frames (_dh)",
             "joint_mapping": {"urdf = sign * ours + offset": True, "sign": URDF_SIGN.tolist(), "offset_deg": URDF_OFFSET_DEG.tolist()},
             "jaws": ("swept over their whole stroke in pieces_*; unswept at the OPEN end in jaw.pieces_open_*, with the "
                      "direction they close along and the stroke") if swept else "none",
             "pieces_for": {name: len(p) for name, p in pieces_per_link.items()}, "seed": args.seed, "seeds_tried": args.seeds,
             "chosen_seed": chosen_seed, "piece_edge_mm": args.piece_edge,
             "links": {}}
    tool_link = next(n for n in um.geo_link if um.link_joint[n] == 6)
    for name, _ in meshes:
        frame, fold = um.tool_frame_of(name)
        T, j = T_fix[tool_link if frame == 6 else name]
        fold_pts = lambda P: np.asarray(P, float) @ fold[:3, :3].T + fold[:3, 3]      # a jaw -> the tool link's frame
        host = tool_link if frame == 6 else name
        link = {"dh_frame": j, "folded_into": None if host == name else host, "T_urdf_to_dh": np.round(T, 7).tolist(),
                "pieces_urdf": [piece_json(fold_pts(v)) for v, f in pieces_per_link[name]],
                "capsules_urdf": [{"p0": np.round(fold_pts([p0])[0], 5).tolist(), "p1": np.round(fold_pts([p1])[0], 5).tolist(), "r": round(r, 5)}
                                  for p0, p1, r in caps_per_link[name]],
                "pieces_dh": [piece_json(to_dh(host, fold_pts(v))) for v, f in pieces_per_link[name]],
                "capsules_dh": [{"p0": np.round(to_dh(host, fold_pts([p0]))[0], 5).tolist(), "p1": np.round(to_dh(host, fold_pts([p1]))[0], 5).tolist(),
                                 "r": round(r, 5)} for p0, p1, r in caps_per_link[name]]}
        if name in jaw_info:
            open_at, u_tool, stroke = jaw_info[name]
            Tr = T_fix[host][0][:3, :3]
            link["jaw"] = {"open_at": open_at, "stroke_m": round(stroke, 6),
                           "close_dir_urdf": np.round(u_tool, 7).tolist(), "close_dir_dh": np.round(Tr @ u_tool, 7).tolist(),
                           "pieces_open_urdf": [piece_json(fold_pts(v)) for v, f in open_pieces[name]],
                           "pieces_open_dh": [piece_json(to_dh(host, fold_pts(v))) for v, f in open_pieces[name]]}
        model["links"][name] = link
    json.dump(model, open(os.path.join(OUT_DIR, f"PAR6_{args.gripper}_collision.json"), "w"))
    lines = ["<!-- self-collision CAPSULES (the coarse layer), in OUR joint frames (frame i = after joint i; base = world).",
             f"     Generated by tools/self_collision_capsules.py fit --gripper {args.gripper} on {time.strftime('%Y-%m-%d')} from",
             f"     {os.path.basename(urdf)}; the convex pieces (the check itself) are in PAR6_{args.gripper}_collision.json. -->",
             "<self_collision>"]
    for name, caps in caps_per_link.items():
        frame, fold = um.tool_frame_of(name)
        T, j = T_fix[tool_link if frame == 6 else name]
        lines.append(f'  <link frame="{j}" source="{name}">')
        for p0, p1, r in caps:
            p0 = fold[:3, :3] @ p0 + fold[:3, 3]; p1 = fold[:3, :3] @ p1 + fold[:3, 3]
            q0 = T[:3, :3] @ p0 + T[:3, 3]; q1 = T[:3, :3] @ p1 + T[:3, 3]
            lines.append(f'    <capsule p0="{q0[0]:.5f} {q0[1]:.5f} {q0[2]:.5f}" p1="{q1[0]:.5f} {q1[1]:.5f} {q1[2]:.5f}" r="{r:.5f}"/>')
        lines.append("  </link>")
    lines.append("</self_collision>")
    open(os.path.join(OUT_DIR, f"PAR6_{args.gripper}_capsules.xml"), "w").write("\n".join(lines) + "\n")
    open(os.path.join(OUT_DIR, f"PAR6_{args.gripper}_fit_report.txt"), "w").write("\n".join(report) + "\n")
    print(f"wrote {os.path.relpath(OUT_DIR, _ROOT)}/PAR6_{args.gripper}_collision.json (pieces + capsules, both frames), "
          f"PAR6_{args.gripper}_capsules.xml, the fit report")
    return 0


def load_model(gripper):
    p = os.path.join(OUT_DIR, f"PAR6_{gripper}_collision.json")
    if not os.path.exists(p):
        raise SystemExit(f"{p} missing — run `fit` first")
    m = json.load(open(p))
    caps, pcs = {}, {}
    for n, l in m["links"].items():
        host = l.get("folded_into") or n                      # a jaw's geometry lives in the tool link's frame
        caps.setdefault(host, []).extend((np.array(c["p0"]), np.array(c["p1"]), c["r"]) for c in l["capsules_urdf"])
        pcs.setdefault(host, []).extend((np.array(pc["verts"]), np.array(pc["faces"], dtype=np.int64)) for pc in l["pieces_urdf"])
    return caps, pcs

def cmd_prove(args):
    urdf, pkg = URDF[args.gripper]
    um = UrdfModel(urdf, pkg)
    caps, pcs = load_model(args.gripper)
    cm = CapsuleModel(um.hosts, caps, um.host_adjacency)
    xm = ConvexModel(um.hosts, pcs, um.host_adjacency)
    assert cm.link_pairs == um.host_pairs == xm.link_pairs
    pairs = [(um.hosts[a], um.hosts[b]) for a, b in um.host_pairs]
    lo, hi = soft_limits_deg()
    rng = np.random.default_rng(args.seed)
    margin = args.margin / 1000.0
    n = len(pairs)
    fails, slack_p, slack_c = [], [[] for _ in range(n)], [[] for _ in range(n)]
    false_near, mesh_near, n_hit_mesh = np.zeros(n, int), np.zeros(n, int), 0
    t_pieces, t_mesh, calls = 0.0, 0.0, []
    for k in range(args.poses):
        ours = rng.uniform(lo, hi)
        q = um.q_from_ours(ours)
        Rw, tw = um.host_frames(q)
        gc = cm.gaps(Rw, tw).copy()
        t0 = time.perf_counter(); gp = xm.gaps(Rw, tw, margin).copy(); t1 = time.perf_counter()
        calls.append(xm.gjk_calls)
        dm = um.host_mesh_distances(q); t2 = time.perf_counter()
        t_pieces += t1 - t0; t_mesh += t2 - t1
        n_hit_mesh += int((dm <= 0).any())
        for i, (a_, b_) in enumerate(pairs):
            if gp[i] < margin and gp[i] > dm[i] + COVER_TOL:
                fails.append((k, a_, b_, "pieces", 1e3 * gp[i], 1e3 * dm[i], np.round(ours, 1).tolist()))
            if gc[i] > dm[i] + COVER_TOL:
                fails.append((k, a_, b_, "capsules", 1e3 * gc[i], 1e3 * dm[i], np.round(ours, 1).tolist()))
            if gp[i] < margin:
                slack_p[i].append(max(dm[i], 0.0) - gp[i])
                false_near[i] += dm[i] >= margin
            mesh_near[i] += dm[i] < margin
            slack_c[i].append(max(dm[i], 0.0) - gc[i])
    t_plain, t_bp = pinocchio_native_cost(um, pcs, margin, [um.q_from_ours(rng.uniform(lo, hi)) for _ in range(200)])
    print(f"prove ({args.gripper}, {args.poses} poses, margin {args.margin:g} mm): {len(xm.pieces)} pieces, {len(xm.IA)} piece pairs over "
          f"{n} link pairs; as a Pinocchio geometry model (one C++ call per pose, 'within the margin'): all pairs {1e6 * t_plain:.0f} us, "
          f"with coal's AABB-tree broadphase {1e6 * t_bp:.0f} us — the per-tick cost RTI would pay; via Python per pair: GJK calls "
          f"median {int(np.median(calls))} / worst {max(calls)}, {1e6 * t_pieces / args.poses:.0f} us/pose; the full meshes "
          f"{1e3 * t_mesh / args.poses:.1f} ms/pose; mesh collisions in {n_hit_mesh} poses")
    if fails:
        print(f"  CONTAINMENT FAILED in {len(fails)} pose/pair cases (a model gap larger than the mesh distance):")
        for f in fails[:10]:
            print(f"    pose {f[0]} {f[1]} vs {f[2]} ({f[3]}): model {f[4]:.1f} mm, mesh {f[5]:.1f} mm, ours {f[6]}")
    else:
        print("  containment holds: pieces gap <= mesh distance and capsule gap <= mesh distance, every pair, every pose")
    print("  pair                        pieces slack med/p90 (mm, poses < margin)   capsules slack med/p90   mesh<margin  pieces near, mesh not")
    for i, (a_, b_) in enumerate(pairs):
        sp = 1e3 * np.array(slack_p[i]) if slack_p[i] else None; sc_ = 1e3 * np.array(slack_c[i])
        sp_txt = f"{np.median(sp):6.1f} / {np.percentile(sp, 90):6.1f}" if sp is not None else "   -   /    -  "
        print(f"  {a_:>12s} vs {b_:<12s}   {sp_txt}                      {np.median(sc_):6.1f} / {np.percentile(sc_, 90):6.1f}"
              f"        {mesh_near[i]:5d}        {false_near[i]:5d}")
    return 1 if fails else 0

def cmd_prove_jaws(args):
    """The UNSWEPT jaws of the fit (jaw.pieces_open_*), placed at a closing fraction c along close_dir, against the URDF's
    jaws at that opening (2026-10-07, S3f): (1) every vertex of each jaw's mesh lies inside its pieces — exact, and it is
    what proves the open end and the direction the collision worker places them by; (2) poses: the pieces' gap never
    exceeds the mesh distance (coal), every pair within the margin, at --jaw-steps openings."""
    urdf, pkg = URDF[args.gripper]
    um = UrdfModel(urdf, pkg)
    p = os.path.join(OUT_DIR, f"PAR6_{args.gripper}_collision.json")
    if not os.path.exists(p):
        raise SystemExit(f"{p} missing — run `fit` first")
    m = json.load(open(p))
    jaws = {n: l for n, l in m["links"].items() if "jaw" in l}
    if not jaws:
        print(f"prove-jaws ({args.gripper}): this model has no jaws"); return 0
    meshes = dict(link_meshes(urdf, pkg))
    lo, hi = soft_limits_deg()
    rng = np.random.default_rng(args.seed)
    margin = args.margin / 1000.0
    pairs = [(um.hosts[a], um.hosts[b]) for a, b in um.host_pairs]
    bad_vertices, fails, near = 0, [], 0
    steps = max(2, args.jaw_steps)
    for step in range(steps):
        c = step / (steps - 1)                                          # 0 = open .. 1 = closed
        pcs = {}
        for n, l in m["links"].items():
            host = l.get("folded_into") or n
            if n in jaws:
                J = l["jaw"]
                shift = c * J["stroke_m"] * np.array(J["close_dir_urdf"])
                placed = [(np.array(pc["verts"]) + shift, np.array(pc["faces"], dtype=np.int64)) for pc in J["pieces_open_urdf"]]
                pcs.setdefault(host, []).extend(placed)
                # (1) the jaw's mesh at this opening, in the tool link's frame, inside its placed pieces
                j = um.link_joint[n]
                jaw_far = c if J["open_at"] == "zero" else 1.0 - c
                V = np.unique(meshes[n].reshape(-1, 3), axis=0) + jaw_far * um.jaw_travel[j]
                F = um.tool_fold[j]
                V = V @ F[:3, :3].T + F[:3, 3]
                inside = np.zeros(len(V), dtype=bool)
                for verts, faces in placed:
                    inside |= points_in_hull(V, verts, faces, tol=JSON_ROUND_TOL)
                bad_vertices += int((~inside).sum())
                if not inside.all():
                    print(f"  {n} at closing {c:.2f}: {int((~inside).sum())} of {len(V)} mesh vertices OUTSIDE its pieces")
            else:
                pcs.setdefault(host, []).extend((np.array(pc["verts"]), np.array(pc["faces"], dtype=np.int64)) for pc in l["pieces_urdf"])
        xm = ConvexModel(um.hosts, pcs, um.host_adjacency)
        jaw_far = c if next(iter(jaws.values()))["jaw"]["open_at"] == "zero" else 1.0 - c
        for _ in range(args.poses):
            ours = rng.uniform(lo, hi)
            q = um.q_from_ours(ours, jaw_far)
            Rw, tw = um.host_frames(q)
            gp = xm.gaps(Rw, tw, margin).copy()
            dm = um.host_mesh_distances(q)
            for i, (a_, b_) in enumerate(pairs):
                if gp[i] < margin:
                    near += 1
                    if gp[i] > dm[i] + COVER_TOL:
                        fails.append((c, a_, b_, 1e3 * gp[i], 1e3 * dm[i], np.round(ours, 1).tolist()))
    J0 = next(iter(jaws.values()))["jaw"]
    print(f"prove-jaws ({args.gripper}): {len(jaws)} jaws, open at {'URDF 0' if J0['open_at'] == 'zero' else 'the far end'}, "
          f"stroke {1e3 * J0['stroke_m']:.1f} mm each; {steps} openings x {args.poses} poses, margin {args.margin:g} mm")
    print(f"  (1) every jaw mesh vertex inside its unswept pieces at every opening: {'YES' if bad_vertices == 0 else f'NO ({bad_vertices})'}")
    print(f"  (2) pieces gap <= mesh distance: {'holds' if not fails else f'FAILED in {len(fails)} cases'} "
          f"({near} pair-poses within the margin checked)")
    for f in fails[:10]:
        print(f"    closing {f[0]:.2f} {f[1]} vs {f[2]}: model {f[3]:.1f} mm, mesh {f[4]:.1f} mm, ours {f[5]}")
    return 1 if (bad_vertices or fails) else 0


def cmd_map(args):
    urdf, pkg = URDF[args.gripper]
    um = UrdfModel(urdf, pkg)
    caps, pcs = load_model(args.gripper)
    cm = ConvexModel(um.hosts, pcs, um.host_adjacency)
    lo, hi = soft_limits_deg()
    rng = np.random.default_rng(args.seed)
    margin = args.margin / 1000.0
    n = len(cm.link_pairs)
    hits, nears, example, gmin = np.zeros(n, int), np.zeros(n, int), [None] * n, np.full(n, np.inf)
    t0 = time.perf_counter()
    for _ in range(args.poses):
        ours = rng.uniform(lo, hi)
        Rw, tw = um.host_frames(um.q_from_ours(ours))
        g = cm.gaps(Rw, tw, margin)
        np.minimum(gmin, g, out=gmin)
        for i in np.flatnonzero(g < margin):
            nears[i] += 1
            if g[i] < 0:
                hits[i] += 1
                if example[i] is None:
                    example[i] = np.round(ours, 1).tolist()
    dt = time.perf_counter() - t0
    print(f"map ({args.gripper}, {args.poses} poses inside the soft limits, margin {args.margin:g} mm, the pieces model, "
          f"{1e6 * dt / args.poses:.0f} us/pose incl. the Python-side FK):")
    print("  pair                           collide   <margin   min gap (mm)   note / example pose (ours, deg)")
    design = []
    for i in np.argsort(-nears):
        a, b = cm.links[cm.link_pairs[i][0]], cm.links[cm.link_pairs[i][1]]
        frac = nears[i] / args.poses
        note = ""
        if frac > 0.98:
            note = "CLOSE BY DESIGN -> own margin or excluded"; design.append((a, b, gmin[i]))
        elif hits[i]:
            note = f"e.g. {example[i]}"
        print(f"  {a:>12s} vs {b:<12s}   {hits[i]:6d}   {nears[i]:7d}   {1e3 * gmin[i]:9.1f}      {note}")
    if design:
        print("  pairs close by design:")
        for a, b, gm in design:
            if gm < 0:
                print(f'    <pair a="{a}" b="{b}" check="off"/>   <!-- the pieces overlap in every pose ({1e3 * gm:.0f} mm) -->')
            else:
                print(f'    <pair a="{a}" b="{b}" margin_mm="{1e3 * gm / 2:.1f}"/>   <!-- half the smallest gap seen -->')
    return 0

def cmd_export(args):
    """temp_data model -> robots/PAR6_collision.json + grippers/<name>_collision.json (the files the runtime loads)."""
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR
    src = os.path.join(OUT_DIR, f"PAR6_{args.gripper}_collision.json")
    if not os.path.exists(src):
        raise SystemExit(f"{src} missing — run `fit --gripper {args.gripper}` first")
    m = json.load(open(src))
    arm = {str(k): {"pieces": [], "capsules": []} for k in range(6)}
    tool = {"pieces": [], "capsules": [], "jaws": []}
    for name, l in m["links"].items():
        target = tool if l["dh_frame"] == 6 else arm[str(l["dh_frame"])]
        if "jaw" in l:                                # a jaw: its swept pieces carry its index; it is also listed unswept
            k = len(tool["jaws"])
            target["pieces"].extend({**pc, "jaw": k} for pc in l["pieces_dh"])
            tool["jaws"].append({"name": name, "stroke_m": l["jaw"]["stroke_m"], "close_dir": l["jaw"]["close_dir_dh"],
                                 "pieces": l["jaw"]["pieces_open_dh"]})
        else:
            target["pieces"].extend(l["pieces_dh"])
        target["capsules"].extend(l["capsules_dh"])
    if not tool["jaws"]:
        del tool["jaws"]
    stamp = time.strftime("%Y-%m-%d")
    robot_out = {"robot": "PAR6", "generated": stamp, "by": f"tools/self_collision_capsules.py export --gripper {args.gripper}",
                 "source": os.path.basename(m["source"]),
                 "frames": "link i (1-5) in DH frame i = RTI's Pinocchio joint i with placement Tz(d_i)Tx(a_i)Rx(alpha_i); link 0 in the world",
                 "pieces": "convex hulls: verts (m), faces into verts; their union contains the link's mesh surface",
                 "links": arm}
    tool_out = {"gripper": args.gripper_name, "generated": stamp, "by": robot_out["by"], "source": robot_out["source"],
                "frame": "the J6 JOINT frame (Pinocchio oMi[6]): joint 6's frame before any gripper's link-6 DH transform",
                "pieces": robot_out["pieces"],
                **({"jaws_note": "pieces with a 'jaw' index are that jaw swept over its whole stroke; 'jaws' lists each jaw "
                                 "UNSWEPT at its OPEN end, the unit direction it moves along as it closes (J6 joint frame) and "
                                 "its stroke: at a closing fraction c it sits at open + c * stroke_m * close_dir"} if "jaws" in tool else {}),
                **tool}
    rp = os.path.join(ROBOTS_DIR, "PAR6_collision.json"); gp = os.path.join(GRIPPERS_DIR, f"{args.gripper_name}_collision.json")
    if not args.tool_only:
        json.dump(robot_out, open(rp, "w")); print(f"wrote {os.path.relpath(rp, _ROOT)}: {sum(len(v['pieces']) for v in arm.values())} pieces, "
                                                  f"{sum(len(v['capsules']) for v in arm.values())} capsules over links 0-5")
    json.dump(tool_out, open(gp, "w")); print(f"wrote {os.path.relpath(gp, _ROOT)}: {len(tool['pieces'])} pieces, {len(tool['capsules'])} capsules (J6 joint frame)")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["fit", "prove", "prove-jaws", "map", "all", "export"])
    ap.add_argument("--gripper-name", default=None, help="export: the gripper file name (default Flange / MSG / SSG48)")
    ap.add_argument("--tool-only", action="store_true", help="export: only the gripper file (the arm is the same)")
    ap.add_argument("--gripper", choices=list(URDF), default="flange")
    ap.add_argument("--poses", type=int, default=None)
    ap.add_argument("--margin", type=float, default=15.0, help="mm")
    ap.add_argument("--pieces", type=int, default=6, help="convex pieces per link (1 for a small link)")
    ap.add_argument("--pieces-for", default="", help="fit: per-link counts, e.g. upper_arm_link=8,wrist_1_link=8")
    ap.add_argument("--piece-edge", type=float, default=20.0, help="fit: split triangles longer than this (mm) before grouping; 0 = off")
    ap.add_argument("--seeds", type=int, default=3, help="fit: k-means seeds tried per link; the smallest total volume is kept")
    ap.add_argument("--jaw-steps", type=int, default=5, help="prove-jaws: openings tried, closed to open")
    ap.add_argument("--max-caps", type=int, default=6, help="capsules per link, at most")
    ap.add_argument("--max-edge", type=float, default=5.0, help="fit: refine triangles to this edge length (mm); radii inflated by edge/sqrt(3)")
    ap.add_argument("--within", type=float, default=1.3, help="fit: accept the fewest capsules within this x the finest fit's volume")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--verbose", action="store_true", help="fit: the capsule volume for every k tried")
    args = ap.parse_args()
    check_dh_literals()
    if args.command == "export":
        args.gripper_name = args.gripper_name or GRIPPER_FILE[args.gripper]
        return cmd_export(args)
    if args.command == "fit":
        return cmd_fit(args)
    if args.command == "prove":
        args.poses = args.poses or 1000
        return cmd_prove(args)
    if args.command == "prove-jaws":
        args.poses = args.poses or 200
        return cmd_prove_jaws(args)
    if args.command == "map":
        args.poses = args.poses or 5000
        return cmd_map(args)
    rc = cmd_fit(args)
    if rc:
        return rc
    args.poses = args.poses or 1000
    rc = cmd_prove(args)
    args.poses = 5000
    return rc or cmd_map(args)


if __name__ == "__main__":
    sys.exit(main())
