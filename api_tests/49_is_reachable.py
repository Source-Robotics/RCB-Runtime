#!/usr/bin/env python3
"""49: is_reachable on the live robot — a pure query, NOTHING MOVES, no control needed.

Needs command_executor restarted with the is_reachable change (RTI does not need a restart).

  1. The pose the arm is at now is reachable, and with near = the current joints the answer IS the
     current joints (when the arm is inside its soft limits — otherwise "no" is the right answer).
  2. A pose 2 m away is not reachable.
  3. Wrong arguments are refused with BAD_ARGS.
  4. Timing: a nearby answer should take a few ms, a "no" up to ~50 ms (every start is tried).

    python3 api_tests/49_is_reachable.py
"""
import argparse
import os
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
from robotclient import RobotClient                        # noqa: E402
from robotclient.exceptions import CommandRejected         # noqa: E402

_ok, _bad = 0, []


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
    else:
        _bad.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


def timed(fn, *a, **k):
    t = time.perf_counter()
    r = fn(*a, **k)
    return r, 1e3 * (time.perf_counter() - t)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    args = ap.parse_args()

    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    n = int(cfg["joint_num"])
    lo = np.degrees(np.asarray(cfg["software_joint_limit_negative"], dtype=float)[:n])
    hi = np.degrees(np.asarray(cfg["software_joint_limit_positive"], dtype=float)[:n])

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    try:
        joints = np.asarray(robot.get_joints(), dtype=float)
        pose = [float(v) for v in robot.get_pose()]
        inside = bool(np.all(joints >= lo) and np.all(joints <= hi))
        print(f"Arm at {np.round(joints, 2).tolist()} deg ({'inside' if inside else 'OUTSIDE'} its soft limits)")
        print(f"TCP   {np.round(pose, 4).tolist()} [m, rad]\n")

        try:
            r, ms = timed(robot.is_reachable, *pose, near=joints.tolist())
        except CommandRejected as exc:
            check("is_reachable is known to the executor", False,
                  f"{exc} — restart command_executor so it loads the new command")
            return 1
        if inside:
            check("the current pose is reachable", r["reachable"] is True, f"{ms:.1f} ms")
            if r["reachable"]:
                d = float(np.max(np.abs(np.asarray(r["joints"]) - joints)))
                check("with near = the current joints, the answer IS the current joints", d < 0.05,
                      f"largest difference {d:.4f} deg")
        else:
            print(f"  (arm outside its limits: answered reachable={r['reachable']} in {ms:.1f} ms — not judged)")

        r, ms = timed(robot.is_reachable, *pose)
        check("near omitted (the measured joints) gives the same answer",
              inside is False or r["reachable"] is True, f"{ms:.1f} ms")

        r, ms = timed(robot.is_reachable, 2.0, 0.0, 0.3, 0.0, 0.0, 0.0)
        check("a pose 2 m away is NOT reachable", r.get("reachable") is False and r.get("joints") is None, f"{ms:.1f} ms")
        check("a 'no' costs well under a second", ms < 500.0, f"{ms:.1f} ms")

        for bad in ([0.1, 0.2, 0.3], pose + [0.0]):
            try:
                robot._immediate("is_reachable", bad)
                check(f"{len(bad)} values refused", False, "accepted")
            except CommandRejected as exc:
                check(f"{len(bad)} values refused with BAD_ARGS", getattr(exc, "code", None) == "BAD_ARGS",
                      str(exc)[:80])
        after = np.asarray(robot.get_joints(), dtype=float)
        check("nothing moved", float(np.max(np.abs(after - joints))) < 0.05,
              f"largest change {float(np.max(np.abs(after - joints))):.4f} deg")
    finally:
        robot.disconnect()
    print(f"\n{_ok} passed, {len(_bad)} failed" + (f": {_bad}" if _bad else ""))
    return 1 if _bad else 0


if __name__ == "__main__":
    sys.exit(main())
