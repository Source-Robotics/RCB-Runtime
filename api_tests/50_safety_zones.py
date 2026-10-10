#!/usr/bin/env python3
"""50: safety zones on the arm — the planner refuses a move into a zone, and the monitor stops a HAND
pushing the TCP into one. Only J5 moves.

Needs RTI AND command_executor restarted with the zones code (RTI.log: "[SAFETY] monitor ON: ... zones"),
and the robot re-homed. The script makes its own temporary cell and RESTORES config/system.xml and the
original zones when it ends (also on Ctrl-C). If it is killed hard, put <active_cell> back yourself and
delete config/cells/api_test_50.xml.

The zone: an ALLOWED PLANE through the point the TCP reaches when J5 turns 4 deg, facing back towards
where it is now — "stay on this side". A plane cannot be slipped past the way a small sphere could. It is
declared haptic="0" freedrive="stop" so step 3 tests the monitor STOPPING a hand in freedrive (the selectable
option); the default — a haptic wall that only warns when pushed through — is api_tests/51_haptic_wall.py.

  1. is_reachable: the pose at J5 + 8 deg is reachable but NOT allowed (zone 'api_wall').
  2. Planner: move_joints to J5 + 10 deg at 10 % -> refused ZONE_LIMIT; J5 does not move.
  3. Monitor: IDLE with gravity comp; YOU rotate J5 by hand past the wall. PASS if the robot disables
     itself with Z_api_wall:ZONE_VIOLATION after the wall, not before.
  4. Recovery: Clear Errors, enable, EXEC, J5 back to where it started (moving OUT of a violation is
     allowed). PASS if nothing re-trips.

    python3 api_tests/50_safety_zones.py          # dry run: prints the plan and the geometry
    python3 api_tests/50_safety_zones.py --go
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
from robotclient import RobotClient                                   # noqa: E402
from robotclient.exceptions import CommandRejected, MotionFailed      # noqa: E402

J = 4                                  # J5, 0-based
WALL_DEG, PAST_DEG = 4.0, 10.0
KEY = "Z_api_wall:ZONE_VIOLATION"
CELL = "api_test_50"
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
    ap.add_argument("--wait", type=float, default=90.0, help="seconds you get to push J5 past the wall")
    args = ap.parse_args()

    import config.xml_parser as xp
    from project_paths import CELLS_DIR, GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    from utility.pinocchio_attach import attach_pinocchio
    from utility.robot_factory import create_robot
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    model = create_robot(sysc["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    attach_pinocchio(model)
    lo = float(np.degrees(cfg["software_joint_limit_negative"][J]))
    hi = float(np.degrees(cfg["software_joint_limit_positive"][J]))

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    base = __import__("46_monitor_baseline")
    original_xml = SYSTEM_XML.read_text()
    cell_path = CELLS_DIR / f"{CELL}.xml"
    claimed = switched = False
    try:
        q0 = np.asarray(robot.get_joints(), dtype=float)
        sign = 1.0 if (hi - q0[J]) >= (q0[J] - lo) else -1.0          # turn towards the roomier side
        if (hi - q0[J] if sign > 0 else q0[J] - lo) < PAST_DEG + 3.0:
            print(f"J5 at {q0[J]:.1f} deg has no {PAST_DEG + 3:.0f} deg of room either way [{lo:.1f}, {hi:.1f}]. Nothing moved.")
            return 2

        def tcp(dj5):
            q = np.radians(q0.copy())
            q[J] += np.radians(sign * dj5)
            return np.asarray(model.pinocchio_fkine(q))[:3, 3]

        p_now, p_wall, p_past = tcp(0.0), tcp(WALL_DEG), tcp(PAST_DEG)
        live = np.asarray(robot.get_pose()[:3], dtype=float)
        n = p_now - p_wall
        n /= np.linalg.norm(n)
        clear_now = float(np.dot(p_now - p_wall, n)) * 1000
        beyond = float(np.dot(p_past - p_wall, n)) * 1000
        print(f"J5 now {q0[J]:.1f} deg, turning {'+' if sign > 0 else '-'}; TCP {np.round(p_now, 4).tolist()} m "
              f"(robot says {np.round(live, 4).tolist()})")
        print(f"Wall: an allowed plane at J5 {sign * WALL_DEG:+.0f} deg — the TCP is {clear_now:.1f} mm on the "
              f"allowed side now, {-beyond:.1f} mm past it at {sign * PAST_DEG:+.0f} deg.")
        if np.linalg.norm(p_now - live) > 0.002 or clear_now < 6.0 or beyond > -6.0:
            print("Geometry not usable (model and robot disagree, or J5 barely moves the TCP here). Nothing moved.")
            return 2
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing moved, no file changed.")
            return 0

        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True:
            print(f"Robot must be ENABLED and homed (got state={st.get('state')} homed={st.get('homed')}). Nothing moved.")
            return 2
        cell_path.write_text(
            '<?xml version="1.0" encoding="utf-8"?>\n<!-- written by api_tests/50_safety_zones.py; deleted when it ends -->\n'
            f'<cell>\n  <zone name="api_wall" shape="plane" kind="allowed" haptic="0" freedrive="stop">\n'
            f'    <point>{p_wall[0]:.6f} {p_wall[1]:.6f} {p_wall[2]:.6f}</point>\n'
            f'    <normal>{n[0]:.6f} {n[1]:.6f} {n[2]:.6f}</normal>\n  </zone>\n</cell>\n')
        SYSTEM_XML.write_text(re.sub(r"<active_cell>.*?</active_cell>", f"<active_cell>{CELL}</active_cell>", original_xml))
        switched = True
        base.claim_control(robot)
        claimed = True
        summary = robot.reload_zones()
        check("reload_zones applied the test cell", summary["cell"] == CELL, summary["cell"])
        time.sleep(0.3)

        print("\n1. is_reachable")
        q_past = q0.copy()
        q_past[J] += sign * 8.0
        pose_past = [float(v) for v in np.concatenate([tcp(8.0), model.fkine(np.radians(q_past)).rpy(order="xyz")])]
        r = robot.is_reachable(*pose_past, near=q_past.tolist())
        check("the pose at J5 +8 deg is reachable but NOT allowed, zone 'api_wall'",
              r["reachable"] and r["allowed"] is False and r["zone"] == "api_wall", str(r)[:120])

        print("\n2. The planner refuses a move through the wall")
        if st.get("mode") != "EXEC":
            if st.get("mode") != "IDLE":
                robot.enter_idle_mode(wait=True)
            robot.enter_exec_mode(wait=True)
        target = q0.copy()
        target[J] += sign * PAST_DEG
        try:
            robot.move_joints(*[float(x) for x in target], speed=10)
            fails = robot.wait_idle(timeout=30.0, raise_on_error=False)
        except (CommandRejected, MotionFailed) as e:
            fails = [str(e)]
        check("move_joints through the wall -> ZONE_LIMIT", any("ZONE_LIMIT" in str(f) for f in (fails or [])), str(fails)[:160])
        moved = abs(float(robot.get_joints()[J]) - q0[J])
        check("J5 did not move", moved < 0.05, f"{moved:.3f} deg")
        robot.clear_errors(wait=True)                                  # clears the program abort latch

        print("\n3. The monitor stops a hand")
        robot.enter_idle_mode(wait=True)
        robot.set_gravity_comp(True)
        robot.release_control()
        claimed = False
        print(f"   Rotate J5 BY HAND {'+' if sign > 0 else '-'}wards past {q0[J] + sign * WALL_DEG:.1f} deg now "
              f"({args.wait:.0f} s), slowly, a few degrees past.")
        tripped_at = None
        end = time.monotonic() + args.wait
        last = 0.0
        while time.monotonic() < end:
            qj = float(robot.get_joints()[J])
            if KEY in (robot.status().get("errors") or []):
                tripped_at = qj
                break
            if time.monotonic() - last > 0.5:
                print(f"   J5 {qj:7.2f} deg   (wall at {q0[J] + sign * WALL_DEG:.1f})", end="\r")
                last = time.monotonic()
            time.sleep(0.02)
        print()
        if not check(f"the monitor tripped with {KEY}", tripped_at is not None, "" if tripped_at is not None else "no trip in time"):
            return 1
        past = sign * (tripped_at - q0[J])
        check("it tripped AFTER the wall (+ the 5 mm tolerance), not before", past > WALL_DEG,
              f"J5 {past:+.2f} deg from the start at the trip (poll; RTI.log has the exact depth)")
        time.sleep(0.3)
        s = robot.status()
        check("the robot disabled itself (ACTIVE_ERROR)", s.get("state") == "DISABLED" and s.get("mode") == "ACTIVE_ERROR",
              f"state={s.get('state')} mode={s.get('mode')}")
        check("status reports a safety violation", s.get("safety") == 1, s.get("safety"))

        print("\n4. Recovery: back out of the violation")
        base.claim_control(robot)
        claimed = True
        robot.clear_errors(wait=True)
        robot.enable(wait=True)
        robot.enter_exec_mode(wait=True)
        target = robot.get_joints()
        target[J] = float(q0[J])
        robot.move_joints(*[float(x) for x in target], speed=10)
        fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
        check("the move back OUT of the violation ran", not fails, str(fails)[:120])
        s = robot.status()
        check("nothing re-tripped on the way out", KEY not in (s.get("errors") or []), s.get("errors"))
    finally:
        if switched:
            SYSTEM_XML.write_text(original_xml)
            try:
                if not claimed:
                    base.claim_control(robot)
                    claimed = True
                after = robot.reload_zones()
                print(f"\nRestored config/system.xml; zones back to cell '{after['cell']}' "
                      f"({len(after['zones'])} zone(s)).")
            except Exception as e:
                print(f"\n!! system.xml restored, but reload_zones failed ({e}) — run robot.reload_zones() or restart.")
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
