#!/usr/bin/env python3
"""79: the dry_run fix and the command names — ON THE ARM (2026-10-23, CLAUDE.md latest+184 / latest+185).

Checks through the running executor what changed on 2026-10-23:

  1. The executor runs today's code: it accepts the NEW wire ID set_collision_detection (an executor from before
     refuses it as UNKNOWN_CMD). Checked FIRST — an old executor would really move the arm in step 2.
  2. move_joints(..., dry_run=True) and delay(..., dry_run=True) send nothing: no joint moves. (Before the fix the
     "dry" joint move MOVED the arm.)
  3. Old and new command IDs do the same: the tool goes 5 mm up along base Z with the OLD wire ID
     (move_linear_rel_wrf) and back down with the new method (move_linear_rel_world) — it ends where it started.
  4. Two settings re-sent at their CURRENT value under the old and the new ID (collision_detection /
     set_collision_detection, gravity_comp / set_gravity_comp): all accepted, nothing changes.
  5. With --gripper: the gripper opens with the OLD ID (gripper_open), closes with the new method (close_gripper)
     and opens again with the old Python name (gripper_open()); each completes.
  6. latest_status() and its old name status() are the same method.

SAFETY
  * Nothing is enabled or homed by this script: the robot must be ENABLED and homed (the dashboard). From IDLE it
    enters EXEC itself (no motion). It takes control WITHOUT forcing (it waits for the dashboard's 5 s to pass)
    and gives it back at the end.
  * Moves: the start pose first at --start-speed % (skip with --no-start-pose), then the TCP 5 mm up and back at
    5 %. If the dry-run check failed, J5 would move 3 degrees (at 5 %) — the script then stops.
  * The gripper moves only with --gripper — keep fingers out of the jaws.

    .venv/bin/python api_tests/79_names_and_dry_run.py              # dry run: prints the plan, moves nothing
    .venv/bin/python api_tests/79_names_and_dry_run.py --go         # run it
    .venv/bin/python api_tests/79_names_and_dry_run.py --go --gripper
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from api_tests._common import START_POSE_DEG, connect, goto_start, parser  # noqa: E402
from robotclient import CommandRejected, RobotClient  # noqa: E402

UP_M = 0.005          # the relative move, along base Z
POSE_TOL_M = 0.0003   # where the tool must end, and how far the first move must go (0.3 mm)
STILL_DEG = 0.05      # a dry run may move no joint more than this (the encoders' noise is ~0.01)

results = []


def check(name, ok, detail=""):
    results.append((name, bool(ok)))
    print(f"  {'PASS' if ok else 'FAIL'}  {name}" + (f"   [{detail}]" if detail and not ok else ""))
    return ok


def claim_control(robot, seconds=15.0):
    """Take control WITHOUT forcing; the dashboard keeps it for 5 s after its last click."""
    end, told = time.monotonic() + seconds, False
    while True:
        try:
            robot.take_control()
            return
        except CommandRejected as exc:
            if exc.code != "CONTROL_HELD" or time.monotonic() > end:
                raise
            if not told:
                print(f"Control is held by {robot.control_holder()} — waiting up to {seconds:.0f} s ...")
                told = True
            time.sleep(1.0)


def joints(robot):
    return [float(v) for v in robot.get_joints()]


def biggest_move(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def main():
    p = parser(__doc__.splitlines()[0])
    p.add_argument("--gripper", action="store_true", help="also open / close the gripper under both names")
    args = p.parse_args()

    print(__doc__.split("SAFETY")[0])
    if not args.go:
        print("DRY RUN: nothing moves. With --go: the start pose", START_POSE_DEG, "(unless --no-start-pose), a dry-run"
              " J5 +3 deg move that must NOT move, the TCP 5 mm up and back at 5 %, two settings re-sent unchanged"
              + (", the gripper open / close / open." if args.gripper else "."))
        return 0

    robot = connect(args)
    # This test checks the WIRE: the executor must take the new IDs itself. The client would send the old ones to a
    # robot that reports a runtime older than the rename (robotclient/version.wire_names_for) — switched off here.
    robot._conn.wire_names = {}
    try:
        st = robot.get_status()
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") not in ("IDLE", "EXEC"):
            print(f"Not ready (mode={st.get('mode')} state={st.get('state')} homed={st.get('homed')}): Clear Errors,"
                  f" Enable (and home) in the dashboard first. Nothing moved.")
            return 2
        claim_control(robot)

        print("\n1. The executor runs today's code")
        cd_now = robot.latest_status().get("collision_detection")
        if cd_now not in ("off", "warn", "stop"):
            print(f"  status() has no collision_detection ({cd_now!r}) — cannot re-send it unchanged. Nothing moved.")
            return 2
        try:
            robot.set_collision_detection(cd_now)            # sends the NEW wire ID set_collision_detection
            new_ok, why = True, ""
        except CommandRejected as exc:
            new_ok, why = False, f"{exc.code}: {exc.message}"
        if not check("the executor accepts the new wire ID set_collision_detection", new_ok, why):
            print("  The running executor is from before today's change — restart it first:\n"
                  "    cd ~/Desktop/RCB-Runtime && tools/restart_gui_and_executor.sh\n  Nothing moved.")
            return 2

        if st.get("motion_aborted"):
            robot.clear_errors(wait=True)
        if st.get("mode") != "EXEC":
            print("Entering EXEC (no motion).")
            robot.enter_exec_mode(wait=True)
        goto_start(robot, args)

        print("\n2. dry_run=True sends nothing")
        q0 = joints(robot)
        target = list(q0)
        target[4] += 3.0
        robot.move_joints(*target, speed=5, dry_run=True, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)
        time.sleep(0.5)
        q1 = joints(robot)
        if not check("move_joints(J5 +3 deg, dry_run=True): no joint moved", biggest_move(q0, q1) < STILL_DEG,
                     f"moved up to {biggest_move(q0, q1):.3f} deg"):
            print("  STOPPING: the dry-run fix is not active on this executor.")
            return 1
        t0 = time.monotonic()
        robot.delay(0.5, dry_run=True)
        robot.wait_idle(timeout=args.timeout)
        q2 = joints(robot)
        check("delay(0.5, dry_run=True): completes, no joint moved", biggest_move(q0, q2) < STILL_DEG,
              f"moved up to {biggest_move(q0, q2):.3f} deg after {time.monotonic() - t0:.2f} s")

        print("\n3. Old and new IDs: the same relative move")
        p0 = robot.get_pose()
        robot._queued("move_linear_rel_wrf", [0.0, 0.0, UP_M, 0.0, 0.0, 0.0], args.timeout, {"speed": 5})
        robot.wait_idle(timeout=args.timeout)
        p1 = robot.get_pose()
        check("the OLD wire ID move_linear_rel_wrf: the tool went 5 mm up", abs((p1[2] - p0[2]) - UP_M) < POSE_TOL_M
              and abs(p1[0] - p0[0]) < POSE_TOL_M and abs(p1[1] - p0[1]) < POSE_TOL_M,
              f"dx {1000 * (p1[0] - p0[0]):+.2f} dy {1000 * (p1[1] - p0[1]):+.2f} dz {1000 * (p1[2] - p0[2]):+.2f} mm")
        robot.move_linear_rel_world(0.0, 0.0, -UP_M, speed=5)
        robot.wait_idle(timeout=args.timeout)
        p2 = robot.get_pose()
        err = max(abs(a - b) for a, b in zip(p2[:3], p0[:3]))
        check("the new move_linear_rel_world: back where it started", err < POSE_TOL_M, f"{1000 * err:.2f} mm off")

        print("\n4. Settings re-sent at their current value, old and new ID")
        s = robot.latest_status()
        gc_now = bool(s.get("gravity_comp"))
        for label, call in (("collision_detection (old ID)", lambda: robot._immediate("collision_detection", [cd_now])),
                            ("set_collision_detection (new)", lambda: robot.set_collision_detection(cd_now)),
                            ("gravity_comp (old ID)", lambda: robot._immediate("gravity_comp", [1 if gc_now else 0])),
                            ("set_gravity_comp (new)", lambda: robot.set_gravity_comp(gc_now))):
            try:
                call()
                check(f"{label} accepted", True)
            except CommandRejected as exc:
                check(f"{label} accepted", False, f"{exc.code}: {exc.message}")
        time.sleep(0.3)
        s = robot.latest_status()
        check("nothing changed (collision detection, gravity compensation)",
              s.get("collision_detection") == cd_now and bool(s.get("gravity_comp")) == gc_now,
              f"{s.get('collision_detection')} / {s.get('gravity_comp')}")

        if args.gripper:
            print("\n5. The gripper under both names")
            for label, call in (("the OLD wire ID gripper_open", lambda: robot._queued_gripper("gripper_open", [],
                                                                                              args.timeout, {})),
                                ("the new close_gripper()", lambda: robot.close_gripper()),
                                ("the old Python name gripper_open()", lambda: robot.gripper_open())):
                try:
                    call()
                    robot.wait_idle(timeout=args.timeout)
                    check(f"{label} completes", True)
                except CommandRejected as exc:
                    check(f"{label} completes", False, f"{exc.code}: {exc.message}")

        print("\n6. The client's names")
        check("latest_status() and status() are the same method", RobotClient.status is RobotClient.latest_status)
    finally:
        try:
            robot.release_control()
        except Exception:
            pass
        robot.disconnect()

    failed = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(failed)}/{len(results)} passed" + (f" — FAILED: {failed}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
