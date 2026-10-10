#!/usr/bin/env python3
"""48: soft joint limits in FREEDRIVE (IDLE gravity comp) — a warning, never an error. YOU push J5 by hand.

Decided 2026-10-01: in passive modes (IDLE, ACTIVE_ERROR) the robot commands no motion, so a soft limit
passed there was passed by a hand — the safety monitor shows J5:SOFT_LIMIT_WARNING (keys are 0-based:
J4 = the 5th joint) and does NOT stop the robot. Active modes keep their rules. Needs RTI restarted with
the change (RTI.log: "[SAFETY] monitor ON: out-of-range True (warning only in IDLE/ACTIVE_ERROR)") and
the robot re-homed.

  1. IDLE + gravity comp. You rotate J5 BY HAND past its soft limit + the 2 deg tolerance.
     PASS if the warning appears and the robot stays ENABLED in IDLE (no error, no stiffening).
  2. You bring it back inside: the warning clears.
  3. You push it past again and LEAVE it there. The script enters EXEC: PASS if nothing trips (it did
     not cross in an active mode), a move 3 deg FURTHER out is refused (JOINT_LIMIT, J5 does not move),
     and a move back to where it started runs and clears the warning.

Motion by the robot: only step 3's moves, J5 alone, 10 %.

    python3 api_tests/48_safety_monitor.py          # dry run: prints the plan
    python3 api_tests/48_safety_monitor.py --go
"""
import argparse
import os
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from robotclient import RobotClient                                   # noqa: E402
from robotclient.exceptions import CommandRejected, MotionFailed       # noqa: E402

J = 4                     # J5, 0-based
WARN = "J5:SOFT_LIMIT_WARNING"
TRIP = "J5:OUT_OF_RANGE"
_ok, _bad = 0, []


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
    else:
        _bad.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


def monitor_live():
    """True if RTI's log shows the monitor constructed since RTI last started."""
    for name in ("RTI.log", "RTI.out.log"):
        path = os.path.join(_ROOT, "logs", name)
        try:
            with open(path, errors="replace") as fh:
                tail = fh.read()[-400000:]
        except OSError:
            continue
        if "[SAFETY] monitor ON" in tail:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--wait", type=float, default=90.0, help="seconds you get for each hand step")
    args = ap.parse_args()

    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    lo = float(np.degrees(cfg["software_joint_limit_negative"][J]))
    hi = float(np.degrees(cfg["software_joint_limit_positive"][J]))
    tol = float(cfg["safety"]["joint_limit_tolerance_deg"])

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    base = __import__("46_monitor_baseline")
    claimed = False

    def push_until(pred, what):
        end = time.monotonic() + args.wait
        last = 0.0
        while time.monotonic() < end:
            q = float(robot.get_joints()[J])
            s = robot.status()
            if pred(q, s):
                print()
                return q, s
            if time.monotonic() - last > 0.5:
                print(f"   J5 {q:7.2f} deg   ({what})", end="\r")
                last = time.monotonic()
            time.sleep(0.02)
        print()
        return None, robot.status()

    try:
        j0 = float(robot.get_joints()[J])
        near_hi = (hi - j0) <= (j0 - lo)
        sign = 1.0 if near_hi else -1.0
        limit = hi if near_hi else lo
        beyond = limit + sign * (tol + 1.0)
        print(f"J5 now {j0:.1f} deg; soft limits [{lo:.1f}, {hi:.1f}] deg, tolerance {tol:.1f} deg.")
        print(f"Plan: IDLE + gravity comp — you push J5 past {beyond:.1f} deg — the robot must only WARN; back "
              f"inside; out again; then EXEC refuses a move further out and runs one back to {j0:.1f} deg.")
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing moved.")
            return 0
        if not monitor_live():
            print("RTI.log has no '[SAFETY] monitor ON' line: RTI is still running without the monitor. Restart RTI "
                  "(and re-home), then re-run. Nothing moved.")
            return 2
        st = robot.status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True:
            print(f"Robot must be ENABLED and homed (got state={st.get('state')} homed={st.get('homed')}). Nothing moved.")
            return 2

        base.claim_control(robot)
        claimed = True
        if st.get("mode") == "EXEC":
            robot.enter_idle_mode(wait=True)
        elif st.get("mode") != "IDLE":
            print(f"Robot is in {st.get('mode')}; put it in IDLE or EXEC first. Nothing moved.")
            return 2
        robot.set_gravity_comp(True)
        robot.release_control()          # freedrive needs nobody commanding
        claimed = False
        time.sleep(0.5)

        print(f"\n1. Rotate J5 BY HAND past {beyond:.1f} deg ({args.wait:.0f} s), and hold it there.")
        q, s = push_until(lambda q, s: sign * (q - beyond) > 0 and WARN in (s.get("errors") or []), f"past {beyond:.1f}")
        if not check(f"the warning {WARN} appeared", q is not None, "" if q is not None else "not seen in time"):
            return 1
        time.sleep(0.5)
        s = robot.status()
        check("the robot stayed ENABLED in IDLE (no error, no stiffening)",
              s.get("state") == "ENABLED" and s.get("mode") == "IDLE", f"state={s.get('state')} mode={s.get('mode')}")
        check(f"no {TRIP}, safety flag 0", TRIP not in (s.get("errors") or []) and s.get("safety") == 0,
              f"errors={s.get('errors')} safety={s.get('safety')}")

        print(f"\n2. Bring it back inside the limit ({limit:.1f} deg).")
        q, s = push_until(lambda q, s: WARN not in (s.get("errors") or []), "back inside")
        check("the warning cleared once it was back inside", q is not None and sign * (q - limit) < 0,
              "still warned" if q is None else f"J5 {q:.2f} deg")

        print(f"\n3. Push it past {beyond:.1f} deg again and LET GO there.")
        q_out, s = push_until(lambda q, s: sign * (q - beyond) > 0 and WARN in (s.get("errors") or []),
                              f"past {beyond:.1f}")
        if not check("outside again, warned", q_out is not None):
            return 1
        input("   Let go of the arm, then press Enter...")
        q_out = float(robot.get_joints()[J])
        base.claim_control(robot)
        claimed = True
        robot.enter_exec_mode(wait=True)
        time.sleep(0.5)
        s = robot.status()
        check("entering EXEC with J5 outside: nothing trips", TRIP not in (s.get("errors") or [])
              and s.get("mode") == "EXEC", f"errors={s.get('errors')} mode={s.get('mode')}")
        target = robot.get_joints()
        target[J] = q_out + sign * 3.0
        try:
            robot.move_joints(*[float(x) for x in target], speed=10)
            fails = robot.wait_idle(timeout=30.0, raise_on_error=False)
        except (CommandRejected, MotionFailed) as e:
            fails = [str(e)]
        check("an EXEC move 3 deg FURTHER out is refused (JOINT_LIMIT)", any("JOINT_LIMIT" in str(f) for f in (fails or [])),
              str(fails)[:140])
        moved = abs(float(robot.get_joints()[J]) - q_out)
        check("J5 did not move", moved < 0.1, f"{moved:.3f} deg")
        robot.clear_errors(wait=True)                    # clears the program abort latch
        robot.enter_exec_mode(wait=True)
        target = robot.get_joints()
        target[J] = j0
        robot.move_joints(*[float(x) for x in target], speed=10)
        fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
        check("a move back to where it started runs", not fails, str(fails)[:120])
        time.sleep(0.3)
        s = robot.status()
        check("back inside: no warning, no error", WARN not in (s.get("errors") or []) and TRIP not in (s.get("errors") or []),
              str(s.get("errors")))
    finally:
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
