"""Self-collision check: convex pieces of the arm and the tool, tested with coal through Pinocchio.

TODO/self_collision_plan.md (2026-10-06), docs/self_collision.md. The geometry is fitted offline by
tools/self_collision_capsules.py and PROVEN against the CAD meshes to be never optimistic: a few convex hulls per
link whose union contains the link's surface. Here they become Pinocchio GeometryObjects on RTI's own FK model
(utility/pinocchio_attach._build_pinocchio_model) and coal decides, per non-adjacent pair, whether the two are
within a margin (coal's security margin) — one C++ call per pose, with an AABB-tree broadphase.

Who uses it:
  * the EXEC planner (command_executor): check_path() on every planned trajectory before it runs -> PlannerError
    SELF_COLLISION naming the pair (the PLANNER margin, the larger one);
  * the collision worker process (self_collision_worker.py): check() on the MEASURED pose (the MONITOR margin, the
    smallest) and ray_start() / ray_advance() along the JOG's ray (the JOG margin: RTI's jog plans its stop to
    arrive exactly where that margin begins), at ~100 Hz on another core, into shared memory;
  * never RTI's tick itself: a folded pose costs 1-2.5 ms (measured) — RTI only reads the worker's fields.

Frames (what the JSON files mean, see tools/self_collision_capsules.py `export`): link i (1-5) in DH frame i, which
is RTI's Pinocchio joint i placed by the link's own Tz(d)Tx(a)Rx(alpha); link 0 in the world; the tool in the J6
JOINT frame (Pinocchio's oMi[6], before any gripper's link-6 DH transform), so a gripper's pieces are independent of
its <kinematics> d.

The jaws (2026-10-07, S3f). The gripper file's pieces hold each jaw SWEPT over its whole stroke (right for any opening —
the planner's model and the default). jaws="measured" builds the model with each jaw UNSWEPT instead, on a prismatic
joint of its own under J6 that moves it along the direction it closes; set_jaws(closing fraction) places them for the
checks that follow, each widened by jaw_band_m along its travel. The collision worker uses it when the gripper reports a
trustworthy position (self_collision_worker.jaw_closed_fraction), and the swept model otherwise.
"""
from __future__ import annotations

import json
import logging
logger = logging.getLogger("sc")  # docs/logging.md "Logger names"
import math
import os

import numpy as np

FRAME_NAMES = ("base", "link 1", "link 2", "link 3", "link 4", "link 5", "tool")
# PAR6 words for the log lines (the link each joint drives, as Source Robotics' PAR6 description package 2.0 names them:
# base_link, shoulder_link, upper_arm_link, forearm_link, wrist_1_link, wrist_2_link, the tool); another robot gets the
# generic names above
PAR6_NAMES = ("base", "shoulder", "upper arm", "forearm", "wrist 1", "wrist 2", "tool")
N_FRAMES = 7
# one planned sample may move no joint more than this between two checks (rad): 1.4 deg ~ 7 mm at the arm's reach,
# half of the 15 mm planner margin
PATH_STEP_RAD = math.radians(1.4)
# The ESCAPE rule (2026-10-07, user decision; a common rule in arm path guards): a path or a jog that STARTS
# with two links already within the margin (the arm pushed there in freedrive, a gripper swap) is allowed as long as
# no other pair comes within the margin and no flagged pair gets closer than at the start (by more than this — a
# numerical tolerance: planned samples have no noise). The monitor's version, on the measured pose, allows 2 mm.
ESCAPE_TOL_M = 1e-4
# while escaping, the path is sampled finer: 0.35 deg ~ 1.7 mm at the arm's reach, under the monitor's 2 mm
ESCAPE_STEP_RAD = math.radians(0.35)
# the jog's ray scan: a check every 0.5 deg (~1.3-2.6 mm at the wrist / the arm's reach), a hit bisected to 0.02 deg
# (~0.1 mm), at most 20 checks per worker cycle (10 deg per 10 ms: twice the fastest jog, 440 deg/s)
RAY_STEP_RAD = math.radians(0.5)
RAY_TOL_RAD = math.radians(0.02)
RAY_BUDGET = 20


def load_geometry(robots_dir, grippers_dir, robot_cfg: dict) -> tuple[dict, dict | None]:
    """The arm's and the gripper's collision files named in the robot / gripper XML -> (robot geometry, tool geometry
    or None). A named file that is missing or unreadable raises: when self-collision checking is ON, no geometry is an
    ERROR, never a silent 'no check' (the cell-file rule)."""
    rf = robot_cfg.get("self_collision", {}).get("file")
    if not rf:
        raise FileNotFoundError("the robot XML names no <self_collision file=...> — no self-collision geometry")
    robot_geo = json.load(open(os.path.join(robots_dir, rf)))
    if set(robot_geo.get("links", {})) != {str(i) for i in range(6)}:
        raise ValueError(f"{rf}: expected links 0-5, got {sorted(robot_geo.get('links', {}))}")
    gf = robot_cfg.get("gripper_collision", {}).get("file")
    tool_geo = json.load(open(os.path.join(grippers_dir, gf))) if gf else None
    if tool_geo is None:
        logger.warning(f"the gripper XML ({robot_cfg.get('gripper_name', '?')}) names no <collision file=...>: "
                        f"the TOOL is not checked against the arm — only the arm's own links are")
    return robot_geo, tool_geo


def _banded(verts, faces, axis, band_m: float):
    """A convex piece widened by band_m both ways along `axis` (unit): the hull of the piece at -band and +band — it
    contains the piece at every position in between."""
    if band_m <= 0.0:
        return verts, faces
    from scipy.spatial import ConvexHull
    V = np.asarray(verts, dtype=np.float64)
    P = np.vstack([V - band_m * axis, V + band_m * axis])
    hull = ConvexHull(P)
    remap = {old: new for new, old in enumerate(hull.vertices)}
    return P[hull.vertices], np.array([[remap[a], remap[b], remap[c]] for a, b, c in hull.simplices], dtype=np.int64)


def _convex(pin, verts, faces):
    """A coal Convex from hull vertices and faces (coal ships with Pinocchio)."""
    import coal
    pts = coal.StdVec_Vec3s(); pts.extend([np.asarray(v, float) for v in verts])
    tl = coal.StdVec_Triangle(); tl.extend([coal.Triangle(int(a), int(b), int(c)) for a, b, c in faces])
    return coal.Convex(pts, tl)


class SelfCollisionModel:
    """The pieces on RTI's FK model, two margin sets (planner / monitor), one broadphase check per pose.

    rtb_robot:   the DHRobot (robots/PAR6.py) — its links give the DH placements and the Pinocchio FK model.
    robot_geo:   robots/<robot>_collision.json as loaded; tool_geo: grippers/<gripper>_collision.json or None.
    policy:      system.xml <self_collision> (planner_margin_mm, monitor_margin_mm, ...).
    pairs:       the per-pair overrides from the robot XML + the gripper XML ({a, b, planner_mm, monitor_mm, check}).
    jaws:        "swept" (default: each jaw over its whole stroke) or "measured" (each jaw on its own prismatic joint,
                 placed by set_jaws(); the gripper file must list its "jaws" — re-exported 2026-10-07 — else ValueError).
    jaw_band_m:  measured jaws only: each jaw piece widened this much both ways along its travel.
    """

    def __init__(self, rtb_robot, robot_geo: dict, tool_geo: dict | None, policy: dict, pairs=(), names=PAR6_NAMES,
                 jaws: str = "swept", jaw_band_m: float = 0.0):
        import pinocchio as pin
        from utility.pinocchio_attach import _build_pinocchio_model, _link_transform
        self.pin = pin
        self.names = tuple(names) if names and len(names) == N_FRAMES else FRAME_NAMES
        self.model, self.data, _ = _build_pinocchio_model(rtb_robot)
        self.joint_num = self.model.nq                     # the ARM's joints; measured jaws add theirs after these
        # measured jaws: one prismatic joint per jaw under J6, along the direction it closes; q = how far it has closed (m)
        jaw_list = list((tool_geo or {}).get("jaws", [])) if jaws == "measured" else []
        if jaws == "measured" and not jaw_list:
            raise ValueError("jaws='measured' but the gripper's collision file lists no 'jaws' — re-export it "
                             "(tools/self_collision_capsules.py fit + export, 2026-10-07)")
        if jaws not in ("swept", "measured"):
            raise ValueError(f"jaws must be 'swept' or 'measured', got {jaws!r}")
        self.jaws = "measured" if jaw_list else "swept"
        self.jaw_strokes = np.array([float(j["stroke_m"]) for j in jaw_list], dtype=np.float64)
        self.jaw_band_m = float(jaw_band_m) if jaw_list else 0.0
        self.jaw_closed = 0.0 if jaw_list else None         # the closing fraction the jaws are placed at
        jaw_axes, jaw_joints = [], []
        for k, jw in enumerate(jaw_list):
            axis = np.asarray(jw["close_dir"], dtype=np.float64)
            axis = axis / np.linalg.norm(axis)
            jaw_axes.append(axis)
            jaw_joints.append(self.model.addJoint(6, pin.JointModelPrismaticUnaligned(axis), pin.SE3.Identity(), f"jaw_{k}"))
        if jaw_list:
            self.data = self.model.createData()
        # the geometry: frame k's pieces on joint k; placement = the link's own DH transform (1-5), identity (0, 6)
        self.geom = pin.GeometryModel()
        self.piece_frame = []
        placements = [np.eye(4)] + [_link_transform(l.d, l.a, l.alpha) for l in rtb_robot.links[:5]] + [np.eye(4)]
        for k in range(N_FRAMES):
            pieces = (tool_geo or {}).get("pieces", []) if k == 6 else robot_geo["links"][str(k)]["pieces"]
            if k == 6 and jaw_list:
                pieces = [pc for pc in pieces if "jaw" not in pc]    # the body; the jaws go on their own joints below
            for n, pc in enumerate(pieces):
                go = pin.GeometryObject(f"f{k}_p{n}", k, pin.SE3(placements[k]), _convex(pin, pc["verts"], pc["faces"]))
                self.geom.addGeometryObject(go); self.piece_frame.append(k)
        for k, jw in enumerate(jaw_list):                  # each jaw unswept at its OPEN end, widened by the band
            for n, pc in enumerate(jw["pieces"]):
                verts, faces = _banded(pc["verts"], pc["faces"], jaw_axes[k], self.jaw_band_m)
                go = pin.GeometryObject(f"f6_jaw{k}_p{n}", jaw_joints[k], pin.SE3.Identity(), _convex(pin, verts, faces))
                self.geom.addGeometryObject(go); self.piece_frame.append(6)
        self.piece_frame = np.array(self.piece_frame, dtype=np.int64)
        # the pairs: non-adjacent frames, minus the ones switched off; margins per pair (mm -> m). The JOG margin per pair
        # is the policy's, clamped between that pair's own monitor and planner margins (a pair close by design keeps its
        # tighter numbers; a jog never probes past the planner's nor stops inside the monitor's).
        pm, mm = float(policy["planner_margin_mm"]) / 1000.0, float(policy["monitor_margin_mm"]) / 1000.0
        jm = float(policy.get("jog_margin_mm", policy["planner_margin_mm"])) / 1000.0
        override = {(p["a"], p["b"]): p for p in pairs}
        self.pair_frames = []                       # per collision pair: (frame a, frame b)
        planner_m, monitor_m, jog_m = [], [], []
        for i in range(self.geom.ngeoms):
            for j in range(i + 1, self.geom.ngeoms):
                fa, fb = int(self.piece_frame[i]), int(self.piece_frame[j])
                if abs(fa - fb) < 2:
                    continue
                o = override.get((min(fa, fb), max(fa, fb)))
                if o is not None and not o["check"]:
                    continue
                self.geom.addCollisionPair(pin.CollisionPair(i, j))
                self.pair_frames.append((min(fa, fb), max(fa, fb)))
                planner_m.append(pm if o is None or o["planner_mm"] is None else o["planner_mm"] / 1000.0)
                monitor_m.append(mm if o is None or o["monitor_mm"] is None else o["monitor_mm"] / 1000.0)
                jog_m.append(min(planner_m[-1], max(monitor_m[-1], jm)))
        # the dashboard's readout (2026-10-07, S3g): every pair up to readout_mm above its own jog margin
        ro = float(policy.get("readout_mm", 30.0)) / 1000.0
        self.margins = {"planner": np.array(planner_m), "monitor": np.array(monitor_m), "jog": np.array(jog_m),
                        "readout": np.array(jog_m) + ro}
        self.jog_margin_of = {pf: float(m) for pf, m in zip(self.pair_frames, jog_m)}   # per link pair
        self._gd, self._mgr, self._cb = {}, {}, {}
        for which, margins in self.margins.items():
            gd = pin.GeometryData(self.geom)
            for k, m in enumerate(margins):
                gd.collisionRequests[k].security_margin = float(m)
            self._gd[which] = gd
            self._mgr[which] = pin.TreeBroadPhaseManager_DynamicAABBTreeCollisionManager(self.model, self.geom, gd)
            self._cb[which] = pin.CollisionCallBackDefault(self.geom, gd)
        self.n_pairs = len(self.pair_frames)
        self.disabled_pairs = sorted((a, b) for (a, b), o in override.items() if not o["check"])
        self._q = np.zeros(self.model.nq)                  # the arm's joints, then the jaws' (measured jaws only)
        # The broadphase refreshes only the pairs it visits: a pair that collided in an EARLIER pose and is far apart
        # now would keep reading "collision". So the pairs flagged last time are cleared before every call (few), and
        # the results are scanned only when the callback saw any collision (rare) — the common tick reads one flag.
        self._last_hit_idx = {which: [] for which in self.margins}
        logger.info(f"{self.geom.ngeoms} convex pieces ({'with' if tool_geo else 'without'} the tool), "
                     f"{self.n_pairs} piece pairs over {len(set(self.pair_frames))} link pairs; margins planner "
                     f"{1e3 * pm:g} mm / jog {1e3 * min(pm, max(mm, jm)):g} mm / monitor {1e3 * mm:g} mm"
                     + (f"; jaws MEASURED ({len(jaw_list)}, stroke {1e3 * self.jaw_strokes.max():.1f} mm, band {1e3 * self.jaw_band_m:g} mm)"
                        if jaw_list else ("; jaws swept over their stroke" if any("jaw" in pc for pc in (tool_geo or {}).get("pieces", [])) else ""))
                     + (f"; per-pair: {[(self.names[a], self.names[b]) for (a, b) in override]}" if override else "")
                     + (f"; not checked: {[(self.names[a], self.names[b]) for a, b in self.disabled_pairs]}" if self.disabled_pairs else ""))

    # ------------------------------------------------------------------ one pose
    def check(self, q, which: str = "monitor") -> list[tuple[int, int]]:
        """The link pairs within the `which` margin at joint angles q (rad) — [] when clear. One C++ call."""
        pin = self.pin
        self._q[:self.joint_num] = q[:self.joint_num]
        gd, mgr, cb = self._gd[which], self._mgr[which], self._cb[which]
        for k in self._last_hit_idx[which]:
            gd.collisionResults[k].clear()
        self._last_hit_idx[which] = []
        pin.forwardKinematics(self.model, self.data, self._q)
        pin.updateGeometryPlacements(self.model, self.data, self.geom, gd)
        mgr.update()
        pin.computeCollisions(mgr, cb)
        if not cb.collision:
            return []
        hits = []
        for k in range(self.n_pairs):
            if gd.collisionResults[k].isCollision():
                self._last_hit_idx[which].append(k)
                pf = self.pair_frames[k]
                if pf not in hits:
                    hits.append(pf)
        return hits

    def hit_gaps(self, which: str = "monitor") -> dict[tuple[int, int], float]:
        """The exact gap (m, negative = the pieces overlap) of every link pair flagged by the LAST check(q, which) —
        computed only over that call's flagged piece pairs, at its placements: a link pair's smallest gap is always
        one of them (it is within the margin, so its closest piece pair is too). Cheap where gap() is not: a pose
        in contact flags a handful of the ~100 piece pairs of a link pair."""
        pin, gd = self.pin, self._gd[which]
        out = {}
        for k in self._last_hit_idx[which]:
            d = pin.computeDistance(self.geom, gd, k).min_distance
            pf = self.pair_frames[k]
            if d < out.get(pf, math.inf):
                out[pf] = d
        return out

    def set_jaws(self, closed: float) -> None:
        """Measured jaws: place them at a closing fraction (0 = open .. 1 = closed) for the checks that follow."""
        if self.jaws != "measured":
            return
        c = min(max(float(closed), 0.0), 1.0)
        np.multiply(self.jaw_strokes, c, out=self._q[self.joint_num:])
        self.jaw_closed = c

    def readout(self, q):
        """The pair nearest its own JOG margin at q, if it is within the readout distance above it: (frame a, frame b,
        gap m, its jog margin m), else None. For the dashboard (the worker runs it every 20th cycle, 5 Hz)."""
        if not self.check(q, "readout"):
            return None
        best = None
        for pf, g in self.hit_gaps("readout").items():
            jm = self.jog_margin_of[pf]
            if best is None or g - jm < best[2] - best[3]:
                best = (pf[0], pf[1], g, jm)
        return best

    def gap(self, q, frame_a: int, frame_b: int) -> float:
        """Exact smallest distance (m) between two links' pieces at q (slower: a distance query per piece pair)."""
        pin = self.pin
        self._q[:self.joint_num] = q[:self.joint_num]
        gd = self._gd["monitor"]
        pin.forwardKinematics(self.model, self.data, self._q)
        pin.updateGeometryPlacements(self.model, self.data, self.geom, gd)
        best = math.inf
        for k, pf in enumerate(self.pair_frames):
            if pf == (min(frame_a, frame_b), max(frame_a, frame_b)):
                d = pin.computeDistance(self.geom, gd, k).min_distance
                if d < best:
                    best = d
        return best

    # ------------------------------------------------------------------ a planned path
    def check_path(self, positions, which: str = "planner"):
        """Samples of a planned path (N, nq), rad. Checks the first and last sample and every sample at which some
        joint has moved PATH_STEP_RAD since the last checked one. Returns None when clear, else
        (sample index, frame a, frame b, gap in m, why) of the first refused sample, why =
          "within" — the path brings two links within the margin;
          "closer" — it STARTS within the margin (the escape rule) and a flagged pair gets closer than at the start;
          "new"    — it starts within the margin and another pair comes within it.
        A path that starts within the margin is sampled every ESCAPE_STEP_RAD instead."""
        pos = np.atleast_2d(np.asarray(positions, dtype=np.float64))
        n = pos.shape[0]
        start = None                                          # escape: {pair: gap at the start}
        last = None
        for k in range(n):
            step = ESCAPE_STEP_RAD if start else PATH_STEP_RAD
            if last is not None and k != n - 1 and np.abs(pos[k, :self.joint_num] - last).max() < step:
                continue
            last = pos[k, :self.joint_num]
            hits = self.check(pos[k], which)
            if not hits:
                continue
            if k == 0:
                start = self.hit_gaps(which)
                continue
            if start is None:
                a, b = hits[0]
                return k, a, b, min(self.hit_gaps(which).values()), "within"
            for (a, b), g in sorted(self.hit_gaps(which).items(), key=lambda kv: kv[1]):
                if (a, b) not in start:
                    return k, a, b, g, "new"
                if g < start[(a, b)] - ESCAPE_TOL_M:
                    return k, a, b, g, "closer"
        return None

    def describe(self, a: int, b: int) -> str:
        return f"{self.names[a]} and {self.names[b]}"

    # ------------------------------------------------------------------ the jog's look-ahead
    # ------------------------------------------------------------------ the jog's ray (RTI's jog, 2026-10-07)
    def ray_start(self, q_ref, direction, horizon_rad: float) -> dict:
        """A scan of the jog's ray q_ref + s * direction (direction: a unit joint-space vector, any mix of joints),
        s from 0 to `horizon_rad` (the soft limits along it). The rule, with the baseline taken HERE: from a start
        clear of the jog margin, a pose is blocked when it is within it (the jog would enter it); from a start
        already inside, only when it brings a flagged pair closer (by ESCAPE_TOL_M) or a new pair in (the escape
        rule — moving apart is always allowed). Advance it with ray_advance()."""
        q0 = np.array(q_ref[:self.joint_num], dtype=np.float64)
        now = self.hit_gaps("jog") if self.check(q0, "jog") else {}
        # measured jaws: the whole scan uses the jaws where they are NOW (a gripper moving during a jog is the 2 mm
        # monitor's to catch)
        return {"q_ref": q0, "dir": np.array(direction[:self.joint_num], dtype=np.float64), "now": now,
                "s_free": 0.0, "done": horizon_rad <= 0.0, "hit": {}, "horizon": max(0.0, float(horizon_rad)),
                "probe": q0.copy(), "inside": bool(now), "jaw_q": self._q[self.joint_num:].copy()}

    def _ray_blocked(self, scan, s: float) -> dict:
        """{pair: gap} that blocks the pose at s along the ray, or {} when it is free (the rule in ray_start)."""
        probe = scan["probe"]
        np.multiply(scan["dir"], s, out=probe)
        probe += scan["q_ref"]
        jaw_q = scan.get("jaw_q")
        if jaw_q is not None and jaw_q.size:
            self._q[self.joint_num:] = jaw_q
        if not self.check(probe, "jog"):
            return {}
        at = self.hit_gaps("jog")
        now = scan["now"]
        if not now or any(p not in now or g < now[p] - ESCAPE_TOL_M for p, g in at.items()):
            return at
        return {}

    def ray_advance(self, scan: dict, budget: int = RAY_BUDGET, step_rad: float = RAY_STEP_RAD, tol_rad: float = RAY_TOL_RAD) -> None:
        """Extend the scan by at most `budget` checks: every `step_rad` past the verified free length; the first
        blocked pose is bisected to `tol_rad` and the scan ends with s_free = the last free parameter (the margin's
        entry, from below) and scan["hit"] = the pairs that block there. s_free never decreases — any value the jog
        has seen stays a valid limit. Reaching the horizon also ends it (nothing blocks before the soft limits)."""
        n = 0
        while n < budget and not scan["done"]:
            s_next = min(scan["s_free"] + step_rad, scan["horizon"])
            n += 1
            hit = self._ray_blocked(scan, s_next)
            if hit:
                lo, hi = scan["s_free"], s_next
                while hi - lo > tol_rad:
                    mid = 0.5 * (lo + hi)
                    n += 1
                    h = self._ray_blocked(scan, mid)
                    if h:
                        hi, hit = mid, h
                    else:
                        lo = mid
                scan["s_free"], scan["done"], scan["hit"] = lo, True, hit
                return
            scan["s_free"] = s_next
            if s_next >= scan["horizon"]:
                scan["done"] = True
