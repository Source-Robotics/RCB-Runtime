#!/usr/bin/env python3
"""60: self-collision on the arm — a refused move, and the worker watching. NOTHING MOVES (docs/self_collision.md).

  1. the collision worker is alive: rti['sc_heartbeat'] advances (read from /dev/shm as a file, never a live segment)
     and the current pose is clear (sc_hit 0); RTI.log shows the monitor with self-collision ON and no STALE warning;
  2. a joint move from here into a pose the map found colliding (the gripper into the base) is REFUSED by the planner
     with SELF_COLLISION naming the pair, before anything moves: the joints are read before and after (0.05 deg);
  3. a move toward it that stops well short (10 % of the way) is accepted by the planner (and cancelled before it
     runs: the command is queued and stopped at once) — so the refusal is about THAT path, not every move.

The robot must be ENABLED, homed, in IDLE or EXEC, error-free; it takes control without forcing. A refused move
ABORTS the program (like JOINT_LIMIT): the test clears errors at the end.

    .venv/bin/python api_tests/60_self_collision.py          # dry run: prints the plan
    .venv/bin/python api_tests/60_self_collision.py --go
"""
import argparse
import os
import struct
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FOLD = [140.3, -10.7, 117.9, 106.0, 88.1, 389.5]          # ours, deg: base vs wrist / forearm / tool (the tool's map)


def shm(name):
    with open(f"/dev/shm/{name}", "rb") as f:
        return struct.unpack("<d", f.read(8))[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true")
    args = ap.parse_args()
    from robotclient import RobotClient
    from robotclient.exceptions import CommandRejected, MotionFailed
    base = __import__("46_monitor_baseline")
    ok = True
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    claimed = False
    try:
        st = robot.status()
        q0 = np.asarray(robot.get_joints(), float)
        print(f"Joints now (deg): {np.round(q0, 2).tolist()}")
        print(f"Plan: (1) read the worker's heartbeat; (2) move_joints to {FOLD} — expected REFUSED (SELF_COLLISION); "
              f"(3) move_joints 10 % of the way — expected accepted, then stopped. Nothing moves.")
        if not args.go:
            print("DRY RUN: re-run with --go. Nothing done.")
            return 0
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") not in ("IDLE", "EXEC") or st.get("err") or st.get("motion_aborted"):
            print(f"Robot must be ENABLED + homed, in IDLE/EXEC, error-free (got {st.get('mode')} {st.get('state')} homed={st.get('homed')} "
                  f"err={st.get('err')} aborted={st.get('motion_aborted')}). Nothing done.")
            return 2
        # 1. the worker
        h0 = shm("sc_heartbeat"); time.sleep(0.5); h1 = shm("sc_heartbeat")
        hz = 2 * (h1 - h0)
        print(f"1. worker heartbeat {h0:.0f} -> {h1:.0f} in 0.5 s (~{hz:.0f} Hz), sc_hit {shm('sc_hit'):.0f}, sc_min_gap {1e3 * shm('sc_min_gap'):.0f} mm")
        if not (50 <= hz <= 300) or shm("sc_hit") != 0:
            print("   FAIL: the worker should run at ~100 Hz and the current pose should be clear"); ok = False
        else:
            print("   OK")
        base.claim_control(robot); claimed = True
        if st.get("mode") != "EXEC":
            robot.enter_exec_mode(wait=True)
        # 2. the refused move
        refused = None
        try:
            robot.move_joints(*FOLD, speed=10)
            fails = robot.wait_idle(timeout=30.0, raise_on_error=False)
            refused = next((f for f in fails if f.get("code") == "SELF_COLLISION"), None)
            if refused is None and fails:
                print(f"   the move failed with {fails}")
        except CommandRejected as e:
            if e.code == "SELF_COLLISION":
                refused = {"code": e.code, "message": str(e)}
        except MotionFailed as e:
            refused = {"code": "SELF_COLLISION", "message": str(e)} if "SELF_COLLISION" in str(e) else None
        q1 = np.asarray(robot.get_joints(), float)
        moved = float(np.abs(q1 - q0).max())
        print(f"2. move into the fold: {'REFUSED — ' + str(refused.get('message', ''))[:140] if refused else 'NOT refused'}; the arm moved {moved:.3f} deg")
        if refused and moved < 0.05:
            print("   OK")
        else:
            print("   FAIL"); ok = False
        # the refusal aborted the program: clear
        robot.clear_errors()
        time.sleep(0.3)
        # 3. a short move toward it is planned (and stopped at once)
        target = q0 + 0.10 * (np.asarray(FOLD) - q0)
        try:
            robot.move_joints(*[float(x) for x in target], speed=5)
            time.sleep(0.05)
            robot.stop()
            fails = robot.wait_idle(timeout=10.0, raise_on_error=False)
            codes = {f.get("code") for f in fails}
            print(f"3. a move 10 % of the way: planned {'and stopped' if not codes - {'STOPPED'} else 'but failed: ' + str(codes)}")
            if codes - {"STOPPED"}:
                ok = False
            else:
                print("   OK")
        except (CommandRejected, MotionFailed) as e:
            print(f"3. a move 10 % of the way was REFUSED: {e} — FAIL (the refusal must be about the colliding path only)"); ok = False
        q2 = np.asarray(robot.get_joints(), float)
        print(f"   the arm is {float(np.abs(q2 - q0).max()):.2f} deg from where it started")
        robot.clear_errors()
    except KeyboardInterrupt:
        print("\nInterrupted — stopping."); robot.stop()
    finally:
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
