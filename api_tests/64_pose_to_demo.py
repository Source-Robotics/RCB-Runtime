#!/usr/bin/env python3
"""64: the arm's pose and the gripper's jaws as ONE line to paste into the self-collision demo. Moves NOTHING.

Drag the arm in freedrive (dashboard: IDLE with gravity compensation); each time you hold it still for about half a second
after moving it — or press Enter — this prints

    #3  paste:  89.29, -134.25, 134.70, 6.45, -83.24, 175.63, 99

the six joint angles in the robot's own degrees (as the dashboard shows them) and the jaws in % closed (0 open .. 100
closed, from the gripper's report exactly as the collision worker reads it). Paste the line into the self-collision demo's "Compare
with the real arm" box and press Show: the model takes that pose and that
opening. Look at the real arm from the same side (the demo's View from buttons) — anything that sits differently is
where the model and the arm disagree. The demo's Distances button off makes it draw only (fast).

Reads shared memory as FILES (/dev/shm/<name>), never a live segment; no model is loaded — it costs nothing while you work.
--cad also prints the closest link pair by the CAD meshes at each pose (loads the meshes at start, ~2 s).

    .venv/bin/python api_tests/64_pose_to_demo.py [--cad]
"""
import os
import select
import struct
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

MOVED_DEG = 0.3          # a pose counts as new once some joint has moved this much since the last line
STILL_DEG = 0.02         # ... and is held still (every joint within this) for STILL_S
STILL_S = 0.5


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


def paste_line(q_deg, closed) -> str:
    """The line the demo's paste box reads: six angles (degrees, 2 decimals) and, when the report is usable, the jaws in %."""
    parts = [f"{v:.2f}" for v in q_deg]
    if closed is not None:
        parts.append(f"{100.0 * closed:.0f}")
    return ", ".join(parts)


def main():
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from self_collision_worker import jaw_closed_fraction
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR, active_gripper=sysc["active_gripper"])
    n = int(cfg["joint_num"])
    hd = cfg.get("gripper_homing_direction")
    motor_ok = hd is not None and hd == hd and int(hd) == 0
    cad = None
    if "--cad" in sys.argv:
        sys.path.insert(0, os.path.join(_ROOT, "tools"))
        import self_collision_capsules as sc
        variant = {"MSG_collision.json": "msg", "SSG48_collision.json": "ssg48", "Flange_collision.json": "flange"}.get(
            (cfg.get("gripper_collision") or {}).get("file"))
        if variant:
            um = sc.UrdfModel(*sc.URDF[variant])
            cad = (um, [(um.hosts[a][:-5], um.hosts[b][:-5]) for a, b in um.host_pairs])
    print("Drag the arm in freedrive; hold it still ~0.5 s (or press Enter) for a line to paste into the demo. Ctrl-C to stop.\n")
    last_q, still_since, n_lines, stdin_open = None, None, 0, True
    prev = f64("joint_position", n)
    try:
        while True:
            time.sleep(0.05)
            q = f64("joint_position", n)
            moving = float(np.max(np.abs(np.degrees(q - prev)))) > STILL_DEG
            prev = q
            enter = False
            if stdin_open and select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline() == "":              # end of input (Ctrl-D, a closed pipe): stop listening for Enter
                    stdin_open = False
                else:
                    enter = True
            now = time.monotonic()
            if moving:
                still_since = None
            elif still_since is None:
                still_since = now
            moved = last_q is None or float(np.max(np.abs(np.degrees(q - last_q)))) > MOVED_DEG
            if not (enter or (moved and still_since is not None and now - still_since >= STILL_S)):
                continue
            last_q = q.copy()
            rti_v = {"gripper_ctrl_mode": i32("gripper_ctrl_mode"), "old_data_warning": i32("old_data_warning", 16)}
            grip_v = {"gripper_calibrated": i32("gripper_calibrated"), "gripper_position": i32("gripper_position"),
                      "gripper_homed": i32("gripper_homed"), "gripper_joint_position": f64("gripper_joint_position"),
                      "gripper_stroke_m": f64("gripper_stroke_m"), "gripper_ticks_per_meter": f64("gripper_ticks_per_meter")}
            c = jaw_closed_fraction(rti_v, grip_v, n, text("homed_state"), motor_ok)
            n_lines += 1
            line = f"#{n_lines:<3d} paste:  {paste_line(np.degrees(q), c)}"
            if c is None:
                line += "   (the gripper's report is not usable now — no jaws value)"
            if cad is not None:
                um, names = cad
                dm = um.host_mesh_distances(um.q_from_ours(np.degrees(q), 0.0 if c is None else c))
                i = int(np.argmin(dm))
                line += f"   | CAD closest: {names[i][0]} - {names[i][1]} {1e3 * max(dm[i], 0.0):.1f} mm"
            print(line, flush=True)
    except KeyboardInterrupt:
        pass
    print(f"\n{n_lines} pose(s) printed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
