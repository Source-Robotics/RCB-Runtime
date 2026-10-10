#!/usr/bin/env python3
"""55: DEMO — a virtual box in freedrive. Move the arm by hand; the TCP stays inside a box you can feel.

The box is an ALLOWED box zone with a haptic wall (utility/haptic_walls.py), centred on where the TCP is now (or
--center x y z, metres, robot base frame), --size 0.2 (a cube, m) or --size X Y Z. In freedrive (IDLE, gravity
comp ON) the arm moves freely inside; past a face the wall pushes the TCP back — a Cartesian spring-damper at the
TCP, sent to the motors as joint torques (tau = J^T F). Its stiffness follows the arm's effective mass along the
push (zone_mass_scaled, Z13): the same natural frequency (default 7.1 Hz, damping ratio 0.67) in every direction
and pose, so it is several times stiffer where the arm is heavy. --flat uses the fixed 200 N/m instead, to compare.
Push THROUGH a face (further than --breakthrough, default 30 mm) and you get a live ZONE_WARNING; nothing stops.

What it is not: only the TCP point is held — not the wrist, not the arm's links — and its orientation is free. The
push is capped (--max-force, default 15 N), so you can always push through. Let go outside the box and the wall
moves the arm back only while its push beats the joints' friction (api_tests/54: J5 ~0.08 Nm, J4/J6 0.1-0.2 Nm).

While it runs the box is a real zone of the active cell: an EXEC or JOG move that would leave it is refused or
stopped. Ctrl-C (or --seconds) removes it and restores config/system.xml; if the script is killed hard, put
<active_cell> back yourself and delete config/cells/demo_box.xml.

    .venv/bin/python api_tests/55_virtual_box.py                       # dry run: the box and its walls, nothing changes
    .venv/bin/python api_tests/55_virtual_box.py --go
    .venv/bin/python api_tests/55_virtual_box.py --go --size 0.3 0.2 0.15 --hz 9 --ratio 0.6
    .venv/bin/python api_tests/55_virtual_box.py --go --flat           # the old fixed-N/m wall, to feel the difference
    .venv/bin/python api_tests/55_virtual_box.py --go --filter 8       # more smoothing of the speed behind the damping

It records the robot's 250 Hz telemetry and, when you stop, lists WHERE it shook: the 1 s stretches with the most
motion above 5 Hz, whether that motion has a real peak (a ringing wall or joint) or just falls off from your hand's
motion, which joint shook most, how close the arm was to a singularity (the Jacobian's smallest singular value), how
stiff the box wall was there and whether a joint wall was pushing too. Saved to temp_data/virtual_box/<time>.npz.
"""
import argparse
import os
import re
import signal
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

CELL = "demo_box"
RECIPE = "haptic_wall"                 # TCP_p, TCP_s, j_p, j_s, j_t, j_tc, tick, mode
SHAKE_MM = 0.05                        # less motion above 5 Hz than this on an axis is noise, not a shake
WARN = f"Z_{CELL}:ZONE_WARNING"
AXES = ("x", "y", "z")


def set_haptic(xml, key, val):
    """Set <haptic_walls><key> in the system.xml text (added if missing)."""
    m = re.search(r"<haptic_walls>.*?</haptic_walls>", xml, flags=re.S)
    block = m.group(0)
    if re.search(rf"<{key}>[^<]*</{key}>", block):
        block = re.sub(rf"<{key}>[^<]*</{key}>", f"<{key}>{val}</{key}>", block, count=1)
    else:
        block = block.replace("</haptic_walls>", f"    <{key}>{val}</{key}>\n    </haptic_walls>")
    return xml[:m.start()] + block + xml[m.end():]


def nearest_face(p, lo, hi):
    """(signed distance to the nearest face — > 0 inside —, its label like '+x')."""
    best, label = None, ""
    for i, ax in enumerate(AXES):
        for d, sgn in ((p[i] - lo[i], "-"), (hi[i] - p[i], "+")):
            if best is None or d < best:
                best, label = d, sgn + ax
    return best, label


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--size", type=float, nargs="+", default=[0.2], help="m: one value (cube) or X Y Z")
    ap.add_argument("--center", type=float, nargs=3, help="m, robot base frame (default: the TCP now)")
    ap.add_argument("--hz", type=float, help="the wall's natural frequency (default: the policy's, 7.1 Hz)")
    ap.add_argument("--ratio", type=float, help="its damping ratio (default: the policy's, 0.67)")
    ap.add_argument("--max-force", type=float, help="N, the push cap (default: the policy's, 15)")
    ap.add_argument("--breakthrough", type=float, help="m past a face = pushed through (default: the policy's, 0.03)")
    ap.add_argument("--flat", action="store_true", help="the fixed N/m wall (zone_mass_scaled 0), to compare")
    ap.add_argument("--filter", type=float, help="ms, the walls' speed filter (default: the policy's, 4; 0-8)")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (default: until Ctrl-C)")
    ap.add_argument("--go", action="store_true")
    args = ap.parse_args()
    size = np.array(args.size * 3 if len(args.size) == 1 else args.size, dtype=float)
    if size.shape != (3,) or (size < 0.04).any() or (size > 1.0).any():
        print("--size: one value or three, each 0.04-1.0 m.")
        return 2

    import config.xml_parser as xp
    from project_paths import CELLS_DIR, GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from robotclient import RobotClient
    from utility import zones as zn
    from utility.haptic_walls import make_inverse_mass
    from utility.pinocchio_attach import attach_pinocchio
    from utility.robot_factory import create_robot
    base = __import__("46_monitor_baseline")
    w51 = __import__("51_haptic_wall")
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    model = create_robot(sysc["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    attach_pinocchio(model)
    minv_fn = make_inverse_mass(model)
    policy = dict(xp.load_zone_setup(SYSTEM_XML, CELLS_DIR, None)["haptic"])

    # The wall settings: Hz / ratio at the 0.1 kg reference -> the stiffness / damping the policy stores.
    M0 = xp.HAPTIC_REFERENCE_MASS
    f0 = float(np.sqrt(policy["stiffness"] / M0) / (2 * np.pi))
    z0 = float(policy["damping"] / (2 * np.sqrt(policy["stiffness"] * M0)))
    f = args.hz if args.hz is not None else f0
    z = args.ratio if args.ratio is not None else z0
    if args.filter is not None:
        if not 0.0 <= args.filter <= xp.WALL_FILTER_MAX_MS:
            print(f"--filter must be 0-{xp.WALL_FILTER_MAX_MS:g} ms (the stability table's columns).")
            return 2
        policy["velocity_tau"] = args.filter / 1000.0
    om = 2 * np.pi * f
    k, d = om * om * M0, 2 * z * om * M0
    win = xp.haptic_damping_window(k, policy["velocity_tau"])
    lo_b, hi_b = xp._HAPTIC_BOUNDS["stiffness"]
    if not (lo_b < k <= hi_b) or win is None or not (win[0] <= d <= win[1]):
        from utility.haptic_walls import wall_dynamics_limits
        f_max = wall_dynamics_limits(policy)[2]
        ok = (f"; up to {f_max:.1f} Hz" if win is None else
              f"; at {f:.2f} Hz the damping ratio may be {win[0] / (2 * om * M0):.2f}-{win[1] / (2 * om * M0):.2f}")
        print(f"{f:.2f} Hz / damping ratio {z:.2f} is outside the stability table (filter "
              f"{1000 * policy['velocity_tau']:g} ms){ok}. Nothing changed.")
        return 2
    want = {"stiffness": k, "damping": d, "zone_mass_scaled": 0 if args.flat else 1,
            "velocity_tau": policy["velocity_tau"]}
    if args.max_force is not None:
        want["max_force"] = args.max_force
    if args.breakthrough is not None:
        want["breakthrough"] = args.breakthrough

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    original_xml = SYSTEM_XML.read_text()
    cell_path = CELLS_DIR / f"{CELL}.xml"
    claimed = switched = False
    capture = prev_recipe = None
    try:
        tcp = np.asarray(robot.get_pose()[:3], dtype=float)
        q_deg = np.asarray(robot.get_joints(), dtype=float)
        centre = np.array(args.center, dtype=float) if args.center else tcp
        lo, hi = centre - size / 2, centre + size / 2
        q = np.radians(q_deg)
        print(f"Box {1000 * size[0]:.0f} x {1000 * size[1]:.0f} x {1000 * size[2]:.0f} mm around "
              f"({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:+.3f}) m; the TCP is at ({tcp[0]:+.3f}, {tcp[1]:+.3f}, "
              f"{tcp[2]:+.3f}).")
        print(f"Wall: {f:.2f} Hz, damping ratio {z:.2f} (= {k:.0f} N/m, {d:.1f} N.s/m at 0.1 kg), filter "
              f"{1000 * policy['velocity_tau']:g} ms, max {want.get('max_force', policy['max_force']):g} N, pushed "
              f"through {1000 * want.get('breakthrough', policy['breakthrough']):.0f} mm past a face.")
        pol = dict(policy, **want)
        print("Stiffness of each face at the pose now" + (" (FIXED — --flat):" if args.flat else
                                                         " (x the arm's effective mass along it / 0.1 kg):"))
        for i, ax in enumerate(AXES):
            n = np.zeros(3)
            n[i] = 1.0
            sc = w51.zone_scale(model, minv_fn, q, n, pol)
            print(f"   +/-{ax}: {k * sc:6.0f} N/m  (x{sc:.2f})")
        inside, _ = nearest_face(tcp, lo, hi)
        if inside <= 0:
            print("NOTE: the TCP is outside the box now — its wall does nothing until the TCP has been inside once.")
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing changed.")
            return 0
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") != "IDLE":
            print(f"Robot must be ENABLED, homed and in IDLE (got state={st.get('state')} homed={st.get('homed')} "
                  f"mode={st.get('mode')}).")
            return 2

        cell_path.write_text(
            '<?xml version="1.0" encoding="utf-8"?>\n<!-- written by api_tests/55_virtual_box.py; deleted when it ends -->\n'
            f'<cell>\n  <zone name="{CELL}" shape="box" kind="allowed" checks="tcp" haptic="1" freedrive="warn">\n'
            f'    <min>{lo[0]:.6f} {lo[1]:.6f} {lo[2]:.6f}</min> <max>{hi[0]:.6f} {hi[1]:.6f} {hi[2]:.6f}</max>\n'
            f'  </zone>\n</cell>\n')
        text = re.sub(r"<active_cell>.*?</active_cell>", f"<active_cell>{CELL}</active_cell>", original_xml)
        for key, val in want.items():
            text = set_haptic(text, key, f"{val:g}" if isinstance(val, float) else str(val))
        SYSTEM_XML.write_text(text)
        switched = True
        base.claim_control(robot)
        claimed = True
        summary = robot.reload_zones()
        applied = summary.get("haptic_walls") or {}
        if summary.get("cell") != CELL or len(summary.get("zones") or []) != 1:
            print(f"reload_zones did not apply the box: {summary}")
            return 1
        if abs(applied.get("stiffness", -1) - k) > 0.01 or abs(applied.get("damping", -1) - d) > 0.01:
            print(f"The robot refused the wall settings (applied {applied.get('stiffness')} N/m, "
                  f"{applied.get('damping')} N.s/m) — see RTI.log. Restoring.")
            return 1
        robot.set_gravity_comp(True)
        walls = robot.get_joint_walls()["walls"]
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        capture = base.Capture()
        robot.release_control()
        claimed = False
        zone = zn.make_zone(CELL, "box", "allowed", checks="tcp", min=tuple(lo), max=tuple(hi))
        print("\nFREEDRIVE: move the arm by hand. Ctrl-C ends the demo and removes the box.\n")
        t_end = time.monotonic() + args.seconds if args.seconds > 0 else None
        while t_end is None or time.monotonic() < t_end:
            p = np.asarray(robot.get_pose()[:3], dtype=float)
            st = robot.status()
            gap, face = nearest_face(p, lo, hi)
            rel = 1000 * (p - centre)
            pos = f"TCP {rel[0]:+5.0f} {rel[1]:+5.0f} {rel[2]:+5.0f} mm"
            if gap > 0:
                where = f"inside, {1000 * gap:5.1f} mm to {face}"
            else:
                depth = zn.signed_distance(zone, *p)
                n = zn.push_direction(zone, *p)
                sc = 1.0 if n is None else w51.zone_scale(model, minv_fn, np.radians(robot.get_joints()), np.array(n), pol)
                push = min(k * sc * depth, pol["max_force"])
                where = f"{1000 * depth:5.1f} mm past {face}: ~{push:4.1f} N"
            warn = "  THROUGH!" if WARN in (st.get("errors") or []) else ""
            other = st.get("mode") != "IDLE" or st.get("state") != "ENABLED"
            print(f"   {pos}   {where}{warn}" + (f"   [{st.get('state')} {st.get('mode')}]" if other else "") + " " * 6,
                  end="\r")
            time.sleep(0.1)
        print()
    except KeyboardInterrupt:
        print()
    finally:
        # From here on a second Ctrl-C is IGNORED until the box is removed and the recording saved. 2026-10-02:
        # three runs saved nothing — most likely a second Ctrl-C during the few seconds of parsing; the same
        # Ctrl-C during reload_zones would leave the box active in RTI.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if switched:
            print("Removing the box and saving the recording — a few seconds; Ctrl-C is ignored until done.")
            SYSTEM_XML.write_text(original_xml)
            try:
                if not claimed:
                    base.claim_control(robot)
                    claimed = True
                after = robot.reload_zones()
                print(f"Box removed: config/system.xml restored, zones back to cell '{after['cell']}' "
                      f"({len(after['zones'])} zone(s)).")
            except Exception as e:
                print(f"!! system.xml restored, but reload_zones failed ({e}) — run robot.reload_zones() or restart.")
        if capture is not None:
            capture.stop()
            if prev_recipe:
                try:
                    robot.set_recipe(prev_recipe)
                except Exception:
                    pass
        if cell_path.exists():
            cell_path.unlink()
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()
    try:
        if capture is not None and capture.packets:
            where_it_shook(w51.to_arrays(capture.packets), lo, hi, model, minv_fn, pol, walls, k)
        elif capture is not None:
            print("!! no telemetry was received during the demo — nothing to analyse")
    finally:
        signal.signal(signal.SIGINT, signal.default_int_handler)
    return 0


def where_it_shook(d, lo, hi, model, minv_fn, pol, walls, k, top=6):
    """Save the recording and list the 1 s stretches (IDLE only) with the most TCP motion above 5 Hz."""
    import json
    from project_paths import TEMP_DATA_DIR
    from utility import zones as zn
    w51 = __import__("51_haptic_wall")
    out = TEMP_DATA_DIR / "virtual_box"
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d_%H%M%S')}.npz"
    np.savez(path, meta=json.dumps({"lo": lo.tolist(), "hi": hi.tolist(), "policy": pol}, default=str), **d)
    print(f"\nrecorded {d['tick'].size / w51.FS:.0f} s -> {path.relative_to(_ROOT)}")
    idle = d["mode"] == "IDLE"
    p = d["TCP_p"][:, :3] * 1000.0
    zone = zn.make_zone(CELL, "box", "allowed", checks="tcp", min=tuple(lo), max=tuple(hi))
    wl = np.radians([[w["lo_deg"], w["hi_deg"]] if w["enabled"] else [-1e9, 1e9] for w in walls])
    win, rows = int(w51.FS), []
    for a in range(0, len(p) - win, win // 2):
        b = a + win
        if not idle[a:b].all():
            continue
        hp = [w51.highpass(p[a:b, i]) for i in range(3)]
        rms3 = [float(np.sqrt(np.mean(x ** 2))) for x, _, _ in hp]
        ax, peak = int(np.argmax(rms3)), None
        for i in range(3):                       # every axis: a ring on one can hide under the hand's motion on another
            pk = w51.resonance(hp[i][1], hp[i][2]) if rms3[i] >= SHAKE_MM else None
            if pk is not None and (peak is None or pk[1] > peak[1]):
                ax, peak = i, pk
        jshake = [float(np.sqrt(np.mean(w51.highpass(np.degrees(d["j_s"][a:b, j]))[0] ** 2))) for j in range(6)]
        mid = (a + b) // 2
        q = d["j_p"][mid]
        sv = float(np.linalg.svd(np.asarray(model.pinocchio_jacob0(q)), compute_uv=False)[-1])
        sd = max(zn.signed_distance(zone, *(p[i] / 1000.0)) for i in range(a, b, 5))
        n = zn.push_direction(zone, *(p[mid] / 1000.0)) if sd > 0 else None
        sc = w51.zone_scale(model, minv_fn, q, np.array(n), pol) if n is not None else None
        jw = [j + 1 for j in range(6) if (d["j_p"][a:b, j] < wl[j, 0]).any() or (d["j_p"][a:b, j] > wl[j, 1]).any()]
        rows.append((float(np.sqrt(sum(r * r for r in rms3))), a / w51.FS, peak, "xyz"[ax], int(np.argmax(jshake)) + 1,
                    max(jshake), sv, np.degrees(q[4]), sd, sc, jw))
    if not rows:
        print("no 1 s stretch in freedrive to analyse")
        return
    real = [r for r in rows if r[2] is not None]
    print(f"{len(rows)} one-second stretches in freedrive; {len(real)} with a REAL peak above 5 Hz (a ringing wall or joint)"
          f" — the rest is your hand's motion falling off. The shakiest:")
    print("     t    TCP>5Hz   peak        joint shaking most    J smallest sv   J5      box wall                joint walls")
    for rms, t, peak, axn, jm, jv, sv, j5, sd, sc, jw in sorted(rows, key=lambda r: (r[2] is not None, r[0]), reverse=True)[:top]:
        pk = f"{peak[0]:4.1f} Hz x{peak[1]:.1f}" if peak else "none (hand)"
        box = (f"{1000 * sd:5.1f} mm out, {k * sc:5.0f} N/m" if sc is not None else
               f"{1000 * sd:5.1f} mm out" if sd > 0 else "inside")
        print(f"   {t:5.1f}s  {rms:5.2f} mm  {pk:12s}  J{jm} {jv:6.1f} deg/s ({axn})    {sv:7.3f}      {j5:+6.1f}  "
              f"{box:22s}  {', '.join(f'J{j}' for j in jw) or '-'}")
    print("   (smallest singular value of the Jacobian near 0 = near a singularity; J5 near 0 = the wrist one)")


if __name__ == "__main__":
    sys.exit(main())
