#!/usr/bin/env python3
"""61: do jogs come to rest at the SAME distance at every speed? (2026-10-07, the exact planned jog stops.) Moves NOTHING.

You jog from the dashboard (any joint, any speed, into a self-collision or toward a soft limit); this script watches.
Every time the arm comes to rest in JOG after moving, it prints:
  * the jog (joint and direction, the speed slider),
  * the closest pair of links (the convex pieces, exact gap) and that pair's JOG margin — at a self-collision stop the
    gap should equal the margin (5 mm as shipped) within ~0.5 mm at ANY speed,
  * how far the soft limits are along the jog's direction — at a soft-limit stop: ~0 deg at any speed,
  * the collision worker's answer for that jog (how far it found the ray free, and whether it was final).
Ctrl-C prints a summary per stop kind.

Reads shared memory as FILES (/dev/shm/<name>, never a live segment); builds the same self-collision model the worker
uses (Pinocchio, a few seconds at start). Nothing is commanded.

--cad (2026-10-08): at every stop also the CAD MESH distance of that pair (the PAR6 description package's meshes, the jaws
where the gripper reports them — firmware 0-255 or motor mode; fully open when the report is not usable). The model must
never read MORE than the CAD (it is proven never optimistic); if the real gripper touches where the CAD says 5 mm, the real
gripper is not the CAD one (other fingers, another rail, mounted turned). Loads the meshes at start (~5 s, ~200 MB).

    .venv/bin/python api_tests/61_jog_stops.py [--cad]
"""
import importlib
import math
import os
import struct
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

D = math.degrees


def f64(name, n=1):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.frombuffer(fh.read(8 * n), dtype=np.float64).copy()


def text(name):
    with open(f"/dev/shm/{name}", "rb") as fh:
        b = fh.read(260)
    n = struct.unpack("<I", b[:4])[0]
    return b[4:4 + n].decode(errors="replace")


def i32(name, n=1):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.frombuffer(fh.read(4 * n), dtype=np.int32).copy()


def stop_kind(has_dir, soft_ahead, answered, done, free, travelled):
    """What ended the motion (pure, angles in rad). "no jog": the arm settled with no jog direction set (e.g. as JOG was
    entered at a pose left by freedrive) — not a stop. "self-collision": the jog went EXACTLY the worker's final free
    length — the planned stop; judged on the worker's answer, not on the gap (2026-10-09: a settle 2.8 mm inside the
    margin read as a stop and the summary called six exact stops "SPREAD over 1 mm")."""
    if not has_dir:
        return "no jog"
    if soft_ahead < math.radians(0.2):
        return "soft"
    if answered and done >= 0.5 and abs(travelled - free) < math.radians(0.05):
        return "self-collision"
    return "released"


def cad_setup(cfg):
    """(UrdfModel, {(frame a, frame b): host-pair index}, jaws-open-at-URDF-0) for the active gripper, or None."""
    sys.path.insert(0, os.path.join(_ROOT, "tools"))
    import self_collision_capsules as sc
    variant = {"MSG_collision.json": "msg", "SSG48_collision.json": "ssg48", "Flange_collision.json": "flange"}.get(
        (cfg.get("gripper_collision") or {}).get("file"))
    if variant is None:
        print("--cad: no CAD model for this gripper — CAD column off")
        return None
    t0 = time.monotonic()
    um = sc.UrdfModel(*sc.URDF[variant])
    idx = {}
    for i, (a, b) in enumerate(um.host_pairs):
        fa, fb = um.link_joint[um.hosts[a]], um.link_joint[um.hosts[b]]
        idx[(min(fa, fb), max(fa, fb))] = i
    print(f"--cad: the {variant} meshes loaded in {time.monotonic() - t0:.1f} s")
    return um, idx


def main():
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from utility.self_collision import SelfCollisionModel, load_geometry
    from robotics.jog import ray_soft_free
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=sysc["active_gripper"])
    n = int(cfg["joint_num"])
    par6 = importlib.import_module(f"robots.{sysc['active_robot']}")
    robot = next(getattr(par6, a) for a in dir(par6) if a.upper() == sysc["active_robot"].upper() and isinstance(getattr(par6, a), type))()
    rg, tg = load_geometry(ROBOTS_DIR, GRIPPERS_DIR, cfg)
    m = SelfCollisionModel(robot, rg, tg, sysc["self_collision"], cfg["self_collision"]["pairs"] + cfg["gripper_collision"]["pairs"])
    # the WORKER's model (2026-10-08): with <jaws>measured</jaws> it places the jaws where the gripper reports them — the gap
    # column must too, or a closed gripper reads the swept stroke (-17 mm where the CAD said 7.8)
    from self_collision_worker import measured_jaws_refusal
    m_meas = None
    if sysc["self_collision"].get("jaws") == "measured" and measured_jaws_refusal(cfg, tg) is None:
        m_meas = SelfCollisionModel(robot, rg, tg, sysc["self_collision"], cfg["self_collision"]["pairs"] + cfg["gripper_collision"]["pairs"],
                                    jaws="measured", jaw_band_m=float(sysc["self_collision"]["jaw_band_mm"]) / 1000.0)
    m_swept = m
    pairs = sorted(set(m.pair_frames))
    jog_m = {pf: float(v) for pf, v in zip(m.pair_frames, m.margins["jog"])}
    lo = np.asarray(cfg["software_joint_limit_negative"], float)[:n]
    hi = np.asarray(cfg["software_joint_limit_positive"], float)[:n]
    pol = sysc["self_collision"]
    cad = cad_setup(cfg) if "--cad" in sys.argv else None
    from self_collision_worker import jaw_closed_fraction
    hd = cfg.get("gripper_homing_direction")
    motor_ok = hd is not None and hd == hd and int(hd) == 0
    print(("The gap column: the worker's model — " + (f"measured jaws (band {sysc['self_collision']['jaw_band_mm']:g} mm), swept when the "
           f"report is not usable" if m_meas is not None else "swept jaws")) + " (the jaws as they are NOW: a jog keeps the jaws of its start)")
    print(f"Watching (nothing moves). Margins: jog {pol['jog_margin_mm']:g} mm, monitor {pol['monitor_margin_mm']:g} mm. "
          f"Jog from the dashboard; Ctrl-C for the summary.\n")
    print(f"{'#':>3} {'jog':9} {'speed':>6}  {'closest pair':24} {'gap':>8} {'its jog margin':>14} {'gap - margin':>12}  "
          f"{'soft limit ahead':>16}  worker" + ("   | CAD mesh gap (jaws)" if cad else ""))
    stops = []
    q_prev = f64("joint_position", n)
    moved, still_since = False, None
    try:
        while True:
            time.sleep(0.05)
            q = f64("joint_position", n)
            mode = text("robot_mode")
            dq = float(np.max(np.abs(q - q_prev)))
            q_prev = q
            if mode != "JOG":
                moved, still_since = False, None
                continue
            if dq > math.radians(0.02):
                moved, still_since = True, None
                continue
            if not moved:
                continue
            if still_since is None:
                still_since = time.monotonic()
                continue
            if time.monotonic() - still_since < 0.4:
                continue
            moved, still_since = False, None                  # one report per stop
            d = f64("jog_dir", n)
            ref = f64("jog_ref", n)
            sess, sc_sess = f64("jog_session")[0], f64("sc_ray_session")[0]
            free, done = f64("sc_ray_free")[0], f64("sc_ray_done")[0]
            pct = f64("jog_speed_pct")[0]
            nz = np.flatnonzero(np.abs(d) > 1e-9)
            jog = (f"J{nz[0] + 1}{'+' if d[nz[0]] > 0 else '-'}" if len(nz) == 1 else "vector") if len(nz) else "?"
            rti_v = {"gripper_ctrl_mode": i32("gripper_ctrl_mode"), "old_data_warning": i32("old_data_warning", 16)}
            grip_v = {"gripper_calibrated": i32("gripper_calibrated"), "gripper_position": i32("gripper_position"),
                      "gripper_homed": i32("gripper_homed"), "gripper_joint_position": f64("gripper_joint_position"),
                      "gripper_stroke_m": f64("gripper_stroke_m"), "gripper_ticks_per_meter": f64("gripper_ticks_per_meter")}
            c = jaw_closed_fraction(rti_v, grip_v, n, text("homed_state"), motor_ok)
            m = m_swept                                       # what the worker uses this cycle
            if m_meas is not None and c is not None:
                m_meas.set_jaws(c)
                m = m_meas
            gaps = {p: m.gap(q, *p) for p in pairs}
            p_min = min(gaps, key=lambda p: gaps[p] - jog_m[p])
            dev = gaps[p_min] - jog_m[p_min]
            travelled = float(np.dot(q - ref, d)) if len(nz) else 0.0
            soft_ahead = ray_soft_free(q, d, lo, hi) if len(nz) else float("nan")
            kind = stop_kind(len(nz) > 0, soft_ahead, sc_sess == sess, done, free, travelled)
            worker = (f"free {D(free):6.2f} deg {'final' if done >= 0.5 else 'frontier'}, travelled {D(travelled):6.2f}"
                      if sc_sess == sess else "no answer for this jog")
            soft_txt = f"{D(soft_ahead):13.3f}deg" if len(nz) else f"{'-':>16}"
            stops.append((kind, jog, pct, dev, soft_ahead))
            cad_txt = ""
            if cad:
                um, cidx = cad
                # the URDF's jaws are OPEN at 0 and closed at their far limit (MSG / SSG48; the fit net pins it)
                dm = um.host_mesh_distances(um.q_from_ours(np.degrees(q), 0.0 if c is None else c))
                hp = cidx.get(p_min)
                cad_txt = (f"   | CAD {1e3 * max(dm[hp], 0.0):6.2f}mm (jaws " + ("open: no usable report" if c is None else f"{100 * c:.0f} % closed")
                           + (")" if dm[hp] + 1e-4 >= gaps[p_min] else ") <-- MODEL READS MORE THAN THE CAD: report this")) if hp is not None else ""
            print(f"{len(stops):3d} {jog:9} {100 * pct:5.0f}%  {m.describe(*p_min):24} {1e3 * gaps[p_min]:6.2f}mm {1e3 * jog_m[p_min]:12.1f}mm "
                  f"{1e3 * dev:+10.2f}mm  {soft_txt}  {worker}  [{kind}]{cad_txt}")
    except KeyboardInterrupt:
        pass
    print("\nSummary")
    for kind in ("self-collision", "soft"):
        rows = [s for s in stops if s[0] == kind]
        if not rows:
            continue
        if kind == "self-collision":
            dv = 1e3 * np.array([r[3] for r in rows])
            print(f"  self-collision stops: {len(rows)}; gap minus the jog margin: min {dv.min():+.2f} / max {dv.max():+.2f} mm "
                  f"(speeds {sorted({round(100 * r[2]) for r in rows})} %) — {'CONSISTENT' if dv.max() - dv.min() < 1.0 else 'SPREAD over 1 mm'}")
        else:
            sa = np.array([D(r[4]) for r in rows])
            print(f"  soft-limit stops: {len(rows)}; left to the limit: max {sa.max():.3f} deg (speeds {sorted({round(100 * r[2]) for r in rows})} %)")
    print(f"  released before any limit: {sum(1 for s in stops if s[0] == 'released')}")
    settled = sum(1 for s in stops if s[0] == "no jog")
    if settled:
        print(f"  rests with no jog running (the arm settled, e.g. as JOG was entered): {settled} — not counted above")
    return 0


if __name__ == "__main__":
    sys.exit(main())
