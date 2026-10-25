#!/usr/bin/env python3
"""53: a safety zone in every mode that moves the arm — what stops it, and HOW FAR PAST THE SURFACE it ends up.
Only J5 moves, slowly, a few degrees. Nothing else.

dummy_test_scripts/test_zone_modes.py covers the software offline (every mode, detection depth vs speed); this
measures the real overshoot, braking included. Needs RTI AND command_executor running and the robot ENABLED and
homed. Like api_tests/50 it writes its own temporary cell (an ALLOWED PLANE the TCP reaches when J5 turns 4 deg)
and RESTORES config/system.xml, the zones and the telemetry recipe when it ends (also on Ctrl-C). If it is killed
hard: put <active_cell> back in config/system.xml and delete config/cells/api_test_53.xml.

  1. EXEC   move_joints into the zone -> refused (ZONE_LIMIT, prevention); J5 does not move.
  2. JOG    jog J5 towards it (~9 deg/s) -> the monitor stops it (ZONE_VIOLATION; JOG has no zone prevention yet).
  3. RTI    stream J5 into it at 250 Hz (8 deg/s ramp) -> the monitor stops it (RTI is monitor-only, v1).
  4. SOFT_RTI  the same at 50 Hz.
After each stop: Clear Errors, Enable, an EXEC move back out (always allowed), and the numbers — the TCP's depth
past the surface when the robot stopped itself, the deepest it got, and the difference (the braking overshoot).
From 250 Hz telemetry (recipe haptic_wall), saved to temp_data/haptic_wall/.

    .venv/bin/python api_tests/53_zone_modes.py          # dry run: the geometry
    .venv/bin/python api_tests/53_zone_modes.py --go
"""
import argparse
import os
import re
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robotclient import RobotClient, ControlLost                      # noqa: E402
from robotclient.exceptions import CommandRejected, MotionFailed      # noqa: E402
from _rti_session import rti_session                                  # noqa: E402

J = 4                                  # J5, 0-based
WALL_DEG, PAST_DEG = 4.0, 9.0          # the surface at +4 deg; never command further than +9 deg
CELL = "api_test_53"
KEY = "Z_api_modes:ZONE_VIOLATION"
RECIPE = "haptic_wall"
FS = 250.0
_ok, _bad = 0, []


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
    else:
        _bad.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--jog", type=float, default=2.0, help="jog speed, percent of the jog limit (default 2)")
    ap.add_argument("--rate", type=float, default=8.0, help="RTI ramp speed, deg/s (default 8)")
    args = ap.parse_args()

    import config.xml_parser as xp
    from project_paths import CELLS_DIR, GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from utility.pinocchio_attach import attach_pinocchio
    from utility.robot_factory import create_robot
    w51 = __import__("51_haptic_wall")
    base = __import__("46_monitor_baseline")
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    model = create_robot(sysc["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    attach_pinocchio(model)
    lo = float(np.degrees(cfg["software_joint_limit_negative"][J]))
    hi = float(np.degrees(cfg["software_joint_limit_positive"][J]))

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    original_xml = SYSTEM_XML.read_text()
    cell_path = CELLS_DIR / f"{CELL}.xml"
    claimed = switched = False
    capture = prev_recipe = None
    results = []
    try:
        q0 = np.asarray(robot.get_joints(), dtype=float)
        sign = 1.0 if (hi - q0[J]) >= (q0[J] - lo) else -1.0
        if (hi - q0[J] if sign > 0 else q0[J] - lo) < PAST_DEG + 3.0:
            print(f"J5 at {q0[J]:.1f} deg has no {PAST_DEG + 3:.0f} deg of room either way [{lo:.1f}, {hi:.1f}].")
            return 2

        def tcp(dj5):
            q = np.radians(q0.copy())
            q[J] += np.radians(sign * dj5)
            return np.asarray(model.pinocchio_fkine(q))[:3, 3]

        p_now, p_wall = tcp(0.0), tcp(WALL_DEG)
        n = p_now - p_wall
        n /= np.linalg.norm(n)
        clear_now = float(np.dot(p_now - p_wall, n)) * 1000
        beyond = float(np.dot(tcp(PAST_DEG) - p_wall, n)) * 1000
        live = np.asarray(robot.get_pose()[:3], dtype=float)
        print(f"J5 now {q0[J]:.1f} deg, turning {'+' if sign > 0 else '-'}. Wall: an allowed plane at J5 "
              f"{sign * WALL_DEG:+.0f} deg — the TCP {clear_now:.1f} mm before it now, {-beyond:.1f} mm past it at the "
              f"furthest command ({sign * PAST_DEG:+.0f} deg).")
        if np.linalg.norm(p_now - live) > 0.002 or clear_now < 6.0 or beyond > -6.0:
            print("Geometry not usable (model and robot disagree, or J5 barely moves the TCP here). Nothing moved.")
            return 2
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing moved, no file changed.")
            return 0
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True:
            print(f"Robot must be ENABLED and homed (got state={st.get('state')} homed={st.get('homed')}).")
            return 2

        cell_path.write_text(
            '<?xml version="1.0" encoding="utf-8"?>\n<!-- written by api_tests/53_zone_modes.py; deleted when it ends -->\n'
            f'<cell>\n  <zone name="api_modes" shape="plane" kind="allowed" haptic="0" freedrive="stop" checks="tcp">\n'
            f'    <point>{p_wall[0]:.6f} {p_wall[1]:.6f} {p_wall[2]:.6f}</point>\n'
            f'    <normal>{n[0]:.6f} {n[1]:.6f} {n[2]:.6f}</normal>\n  </zone>\n</cell>\n')
        SYSTEM_XML.write_text(re.sub(r"<active_cell>.*?</active_cell>", f"<active_cell>{CELL}</active_cell>", original_xml))
        switched = True
        base.claim_control(robot)
        claimed = True
        summary = robot.reload_zones()
        check("reload_zones applied the test cell", summary["cell"] == CELL, summary["cell"])
        capture = base.Capture()
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)

        def depth_of(d):
            return -((d["TCP_p"][:, :3] - p_wall) @ n) * 1000.0      # mm past the surface (> 0 = in it)

        def recover(label):
            """Clear the trip, enable, and move J5 back out to where it started (EXEC; always allowed). Returns
            whether the arm is OUT of the zone again — the next step must never start inside it (2026-10-02: a
            failed recovery left it 5 mm in, and the RTI step then drove it deeper)."""
            robot.clear_errors(wait=True)
            robot.enable(wait=True)
            if robot.status().get("mode") != "EXEC":
                if robot.status().get("mode") != "IDLE":
                    robot.enter_idle_mode(wait=True)
                robot.enter_exec_mode(wait=True)
            target = robot.get_joints()
            target[J] = float(q0[J])
            robot.move_joints(*[float(x) for x in target], speed=10)
            fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
            out = float(depth_now()) < -1.0
            check(f"{label}: moved back out of the zone", not fails and out and KEY not in (robot.status().get("errors") or []),
                  f"{depth_now():.1f} mm deep; {str(fails)[:100]}")
            robot.enter_idle_mode(wait=True)
            return out and not fails

        def depth_now():
            return -float(np.dot(np.asarray(robot.get_pose()[:3], dtype=float) - p_wall, n)) * 1000

        def measure(label, start_idx):
            d = w51.to_arrays(capture.packets[start_idx:])
            path = w51.save_recording(d, f"53 {label}", p_wall=p_wall.tolist(), n=n.tolist(), sign=sign)
            if d["tick"].size == 0:
                check(f"{label}: telemetry recorded", False)
                return
            depth = depth_of(d)
            stopped = np.flatnonzero(d["mode"] == "ACTIVE_ERROR")
            if stopped.size == 0:
                check(f"{label}: the robot stopped itself (ACTIVE_ERROR in telemetry)", False)
                return
            i = stopped[0]
            at_trip, deepest = float(depth[max(0, i - 1)]), float(depth[i:].max())
            # The AVERAGE over the last 30 ticks: per-tick differences carry J5's stepper ripple, and the median of
            # their absolute values read 105 mm/s for a 20 mm/s approach (2026-10-02).
            a = max(0, i - 31)
            speed = (depth[i - 1] - depth[a]) / max(1.0, d["tick"][i - 1] - d["tick"][a]) * FS if i > 2 else float("nan")
            results.append((label, speed, at_trip, deepest))
            print(f"   {label}: approached at {speed:.0f} mm/s; stopped itself {at_trip:.1f} mm past the surface, "
                  f"deepest {deepest:.1f} mm -> braking overshoot {deepest - at_trip:.1f} mm"
                  + ("" if path is None else f"   (saved {path.relative_to(_ROOT)})"))
            check(f"{label}: tripped past the surface + 5 mm tolerance, not before", at_trip >= 4.0, f"{at_trip:.1f} mm")

        def wait_trip(seconds, tick_fn=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if tick_fn is not None and tick_fn() is False:
                    break
                if KEY in (robot.status().get("errors") or []):
                    return True
                time.sleep(0.02)
            return KEY in (robot.status().get("errors") or [])

        # ------------------------------------------------------------------ 1. EXEC
        print("\n1. EXEC: the planner refuses a move into the zone")
        if st.get("mode") != "EXEC":
            if st.get("mode") != "IDLE":
                robot.enter_idle_mode(wait=True)
            robot.enter_exec_mode(wait=True)
        target = q0.copy()
        target[J] += sign * PAST_DEG
        time.sleep(0.3)                                                # EXEC holds the pose it entered at
        j5_before = float(robot.get_joints()[J])     # NOT q0: the arm floats in IDLE gravity comp before EXEC
        try:
            robot.move_joints(*[float(x) for x in target], speed=10)
            fails = robot.wait_idle(timeout=30.0, raise_on_error=False)
        except (CommandRejected, MotionFailed) as e:
            fails = [str(e)]
        check("move_joints into the zone -> ZONE_LIMIT", any("ZONE_LIMIT" in str(f) for f in (fails or [])), str(fails)[:120])
        moved = abs(float(robot.get_joints()[J]) - j5_before)
        check("J5 did not move", moved < 0.05, f"{moved:.3f} deg")
        robot.clear_errors(wait=True)                                  # clears the program abort latch
        robot.enter_idle_mode(wait=True)

        # ------------------------------------------------------------------ 2. JOG
        print(f"\n2. JOG: jog J5 towards the zone at {args.jog:g} % of the jog limit — no prevention, the monitor stops it")
        robot.enter_jog_mode(wait=True)
        start = len(capture.packets)
        robot.jog(J, int(sign), args.jog)
        far = lambda: sign * (float(robot.get_joints()[J]) - q0[J]) < PAST_DEG     # never past +9 deg
        tripped = wait_trip(20.0, far)
        robot.jog_stop()
        time.sleep(0.5)
        check("JOG: ZONE_VIOLATION stopped it", tripped, "" if tripped else "no trip before +9 deg")
        if tripped:
            measure("JOG", start)
        if not recover("JOG"):
            print("   Stopping here: the arm is not back out of the zone, and the next steps would start inside it.")
            return 1

        # ------------------------------------------------------------------ 3/4. RTI, SOFT_RTI
        for label, rate in (("RTI", None), ("SOFT_RTI", 50.0)):
            if depth_now() > -1.0:
                check(f"{label}: starts outside the zone", False, f"{depth_now():.1f} mm deep")
                return 1
            print(f"\n{3 if rate is None else 4}. {label}: stream J5 into the zone at {args.rate:g} deg/s"
                  f"{'' if rate is None else f', {rate:g} Hz'} — monitor-only, it stops")
            with rti_session(robot, rate_hz=rate, stop=False) as (rti, first):
                qs = first["j_p"]
                start = len(capture.packets)
                period = 1.0 / (rate or FS)
                t0, nxt, lost, tripped = time.monotonic(), time.monotonic(), None, False
                try:
                    while True:
                        t = time.monotonic() - t0
                        off = min(np.radians(args.rate) * t, np.radians(PAST_DEG))
                        q = list(qs)
                        q[J] = qs[J] + sign * off
                        rti.send_joint_position(q)
                        if int(t * 25) != int((t - period) * 25) and KEY in (robot.status().get("errors") or []):
                            tripped = True
                            break
                        if t > PAST_DEG / args.rate + 3.0:
                            break
                        nxt += period
                        slp = nxt - time.monotonic()
                        if slp > 0:
                            time.sleep(slp)
                except ControlLost as e:
                    lost = e
                    tripped = KEY in (robot.status().get("errors") or [])
            time.sleep(0.5)
            check(f"{label}: ZONE_VIOLATION stopped it", tripped, "" if tripped else f"no trip (lost: {lost})")
            if tripped:
                measure(label, start)
            if not recover(label):
                print("   Stopping here: the arm is not back out of the zone.")
                return 1

        if results:
            print("\n   mode       approach   stopped itself at   deepest    braking overshoot")
            for label, speed, at_trip, deepest in results:
                print(f"   {label:9s} {speed:6.0f} mm/s   {at_trip:8.1f} mm      {deepest:6.1f} mm   {deepest - at_trip:6.1f} mm")
    finally:
        if switched:
            SYSTEM_XML.write_text(original_xml)
            try:
                if not claimed:
                    base.claim_control(robot)
                    claimed = True
                if prev_recipe:
                    robot.set_recipe(prev_recipe)
                after = robot.reload_zones()
                print(f"\nRestored config/system.xml; zones back to cell '{after['cell']}' ({len(after['zones'])} zone(s)).")
            except Exception as e:
                print(f"\n!! system.xml restored, but reload_zones failed ({e}) — run robot.reload_zones() or restart.")
        if capture is not None:
            capture.stop()
        if cell_path.exists():
            cell_path.unlink()
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()
    print(f"\n{_ok} passed, {len(_bad)} failed" + (f": {_bad}" if _bad else ""))
    return 1 if _bad else 0


if __name__ == "__main__":
    sys.exit(main())
