#!/usr/bin/env python3
"""63: where does the REAL arm touch itself, by the CAD? (2026-10-08) Moves NOTHING.

Why: on 2026-10-08 the user could "slam the gripper into the shoulder with J5" while every jog stop, checked against the
PAR6 CAD meshes at the MEASURED joint angles, left the gripper 6.5-9.2 mm from the shoulder (and the runs to J5's soft
limit passed at 8-12 mm). The collision check only knows the measured angles and the CAD; if the real gripper touches
where the CAD says 8 mm, the real arm is not where those angles + the CAD put it — a joint's zero (homing offset), backlash,
or a part shaped / mounted differently from the CAD. This measures that discrepancy.

How: put the arm in freedrive (dashboard: IDLE with gravity compensation — self-collision does nothing there, by design)
and push the gripper GENTLY onto the shoulder (or wherever it hits), hold it there, press Enter. It prints, at the measured
pose, the CAD gap of the closest link pairs (the jaws where the gripper reports them) and the collision model's gap. A real
contact should read ~0 mm in the CAD; X mm means the real parts are X mm closer than the model thinks, at that pose. Take
3-5 touches at different J3 / J5 / J6 (open and closed jaws); Ctrl-C (or q) saves them to temp_data/contact_probe.json.

    .venv/bin/python api_tests/63_contact_probe.py
"""
import json
import os
import struct
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "tools"))


def f64(name, n=1):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.frombuffer(fh.read(8 * n), dtype=np.float64).copy()


def i32(name, n=1):
    with open(f"/dev/shm/{name}", "rb") as fh:
        return np.frombuffer(fh.read(4 * n), dtype=np.int32).copy()


def text(name):
    with open(f"/dev/shm/{name}", "rb") as fh:
        b = fh.read(260)
    k = struct.unpack("<I", b[:4])[0]
    return b[4:4 + k].decode(errors="replace")


def main():
    import importlib
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML, TEMP_DATA_DIR
    from utility.self_collision import SelfCollisionModel, load_geometry
    from self_collision_worker import jaw_closed_fraction, measured_jaws_refusal
    import self_collision_capsules as sc
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=sysc["active_gripper"])
    pol = sysc["self_collision"]
    n = int(cfg["joint_num"])
    variant = {"MSG_collision.json": "msg", "SSG48_collision.json": "ssg48", "Flange_collision.json": "flange"}.get(
        (cfg.get("gripper_collision") or {}).get("file"))
    if variant is None:
        print("No CAD for this gripper — nothing to compare."); return 2
    PAR6 = importlib.import_module(f"robots.{sysc['active_robot']}")
    robot = next(getattr(PAR6, a) for a in dir(PAR6) if a.upper() == sysc["active_robot"].upper() and isinstance(getattr(PAR6, a), type))()
    rg, tg = load_geometry(ROBOTS_DIR, GRIPPERS_DIR, cfg)
    pairs = cfg["self_collision"]["pairs"] + cfg["gripper_collision"]["pairs"]
    m = SelfCollisionModel(robot, rg, tg, pol, pairs, jaws="measured", jaw_band_m=0.0) if measured_jaws_refusal(cfg, tg) is None \
        else SelfCollisionModel(robot, rg, tg, pol, pairs)
    um = sc.UrdfModel(*sc.URDF[variant])
    frame_pair = {}
    for i, (a, b) in enumerate(um.host_pairs):
        frame_pair[i] = tuple(sorted((um.link_joint[um.hosts[a]], um.link_joint[um.hosts[b]])))
    hd = cfg.get("gripper_homing_direction")
    motor_ok = hd is not None and hd == hd and int(hd) == 0
    print(f"Contact probe ({variant}). Freedrive the gripper gently onto the arm, hold, press Enter (q + Enter to finish).\n")
    samples = []
    while True:
        try:
            ans = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break
        if ans == "q":
            break
        q = f64("joint_position", n)
        rti_v = {"gripper_ctrl_mode": i32("gripper_ctrl_mode"), "old_data_warning": i32("old_data_warning", 16)}
        grip_v = {"gripper_calibrated": i32("gripper_calibrated"), "gripper_position": i32("gripper_position"),
                  "gripper_homed": i32("gripper_homed"), "gripper_joint_position": f64("gripper_joint_position"),
                  "gripper_stroke_m": f64("gripper_stroke_m"), "gripper_ticks_per_meter": f64("gripper_ticks_per_meter")}
        c = jaw_closed_fraction(rti_v, grip_v, n, text("homed_state"), motor_ok)
        cj = 0.0 if c is None else c
        if m.jaws == "measured":
            m.set_jaws(cj)
        dm = um.host_mesh_distances(um.q_from_ours(np.degrees(q), cj))
        order = np.argsort(dm)[:3]
        rows = []
        for i in order:
            pf = frame_pair[i]
            rows.append({"pair": m.describe(*pf), "frames": list(pf), "cad_mm": round(1e3 * float(dm[i]), 2),
                         "model_mm": round(1e3 * float(m.gap(q, *pf)), 2) if np.isfinite(m.gap(q, *pf)) else float("inf")})
        samples.append({"utc": time.strftime("%H:%M:%S", time.gmtime()), "q_deg": np.round(np.degrees(q), 3).tolist(),
                        "jaws_closed": None if c is None else round(cj, 3), "closest": rows})
        print(f"  #{len(samples)} q = {np.round(np.degrees(q), 2).tolist()} deg, jaws "
              + ("open (report not usable)" if c is None else f"{100 * cj:.0f} % closed"))
        for r in rows:
            model_txt = "not checked (close by design)" if not np.isfinite(r["model_mm"]) else f"{r['model_mm']:7.2f} mm"
            print(f"     {r['pair']:24s} CAD {r['cad_mm']:7.2f} mm   model {model_txt}")
        print("     (touching for real: the CAD should read ~0 — what it reads is how much closer the real parts are)")
    if samples:
        out = os.path.join(str(TEMP_DATA_DIR), "contact_probe.json")
        json.dump(samples, open(out, "w"), indent=1)
        print(f"\n{len(samples)} touch(es) saved to {os.path.relpath(out, _ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
