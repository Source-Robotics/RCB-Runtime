#!/usr/bin/env python3
"""45: EXEC joint limits, abort-on-failure and start re-sync — ON THE ARM, J5 ONLY.

Verifies step 1 of TODO/safety_stack_research.md §8 through the real TCP path:

  1. A move_joints target past J5's software limit is refused (JOINT_LIMIT) and J5 does NOT move.
  2. After a failed move the program is ABORTED: the next queued command is refused
     (MOTION_ABORTED) until clear_errors.
  3. An async program [legal, illegal, legal]: the first runs, the second is JOINT_LIMIT, the third
     is ABORTED and never runs.
  4. After jogging, the next EXEC move starts from where the arm IS, not from where the last EXEC
     move ended (before this, J5 would first snap back to the old endpoint).
  5. Leaving EXEC mid-move (enter_idle_mode) fails the pending command at once with ABORTED —
     before this, the client waited out its 120 s timeout for a move RTI had thrown away.

SAFETY
  * Only J5 moves, at --speed percent (default 10). Nothing is homed or enabled by this script:
    the robot must already be ENABLED and homed (use the dashboard). If it is in IDLE, the script
    enters EXEC itself — that moves nothing. It takes control WITHOUT forcing: if the dashboard
    holds it (it does for 5 s after a click) the script waits for the release. It gives control
    back at the end.
  * The refusal test first moves J5 to --margin degrees inside its nearer software limit (an
    ordinary legal pose), then asks for 3 degrees PAST the limit. Worst case, if the check were
    somehow broken, J5 moves margin + 3 degrees and stops 3 degrees past its soft limit — still
    far inside the hard limit. Make sure the tool/cables allow that wrist angle.
  * Refuses to run unless command_executor is running the new code (its status must carry the
    `motion_aborted` field). An old executor would really drive J5 past its limit.
  * Leaves J5 back where it started.

    python3 api_tests/45_exec_abort.py          # dry run: prints the plan, moves nothing
    python3 api_tests/45_exec_abort.py --go     # run it
"""
import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from robotclient import RobotClient                                   # noqa: E402
from robotclient.exceptions import CommandRejected                    # noqa: E402
import config.xml_parser as xp                                        # noqa: E402

J = 4                     # J5, 0-based
_ok, _bad = 0, []


def check(name, cond, detail=""):
    global _ok
    if cond:
        _ok += 1
    else:
        _bad.append(name)
    print(f"  {'PASS' if cond else 'FAIL'}  {name}" + (f"  [{detail}]" if detail else ""))
    return cond


def joints(robot):
    return np.asarray(robot.get_joints(), dtype=float)


def with_j5(q, deg):
    t = np.array(q, dtype=float)
    t[J] = deg
    return t


def move(robot, target, speed, timeout):
    """Queue one move_joints and wait for it. Returns the failure list ([] = success)."""
    robot.move_joints(*[float(x) for x in target], speed=speed)
    return robot.wait_idle(timeout=timeout, raise_on_error=False)


def codes(failures):
    return [f.get("code") for f in failures or []]


def claim_control(robot, seconds=15.0):
    """Take control WITHOUT forcing. The dashboard keeps it for 5 s after its last click, so wait
    for that release instead of seizing it from whoever holds it."""
    end = time.monotonic() + seconds
    told = False
    while True:
        try:
            robot.take_control()
            return
        except CommandRejected as exc:
            if exc.code != "CONTROL_HELD" or time.monotonic() > end:
                raise
            if not told:
                print(f"Control is held by {robot.control_holder()} — waiting up to {seconds:.0f} s "
                      f"for it to be released (the dashboard lets go 5 s after its last click)...")
                told = True
            time.sleep(1.0)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=29000)
    p.add_argument("--go", action="store_true", help="actually move J5")
    p.add_argument("--speed", type=float, default=10.0, help="joint speed percent")
    p.add_argument("--margin", type=float, default=5.0, help="deg inside the soft limit for the approach pose")
    p.add_argument("--timeout", type=float, default=60.0)
    args = p.parse_args()

    lo, hi = xp.load_software_joint_limits(os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "robots", "PAR6.xml"))
    lo_d, hi_d = float(np.degrees(lo[J])), float(np.degrees(hi[J]))

    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    try:
        st = robot.status()
        if "motion_aborted" not in st:
            print("command_executor is running OLD code (no `motion_aborted` in its status).\n"
                  "Restart it first: tools/restart_gui_and_executor.sh — then re-run. Nothing moved.")
            return 2
        q0 = joints(robot)
        j0 = float(q0[J])
        upper = (hi_d - j0) <= (j0 - lo_d)
        limit = hi_d if upper else lo_d
        sign = 1.0 if upper else -1.0
        near = limit - sign * args.margin
        past = limit + sign * 3.0
        print(f"J5 now {j0:.2f} deg, software limits [{lo_d:.2f}, {hi_d:.2f}] deg.")
        print(f"Plan: J5 -> {near:.2f} (legal), request {past:.2f} (must be REFUSED), back to {j0:.2f}; "
              f"then a jog of ~3 deg and a short move; then an interrupted move. Speed {args.speed:.0f}%.")
        if not args.go:
            print("DRY RUN: re-run with --go to execute. Nothing moved.")
            return 0
        if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("mode") not in ("IDLE", "EXEC"):
            print(f"Robot must be ENABLED + homed, in IDLE or EXEC (got mode={st.get('mode')} "
                  f"state={st.get('state')} homed={st.get('homed')}). Prepare it from the dashboard. "
                  f"Nothing moved.")
            return 2
        claim_control(robot)
        if st.get("motion_aborted"):
            robot.clear_errors(wait=True)
        if st.get("mode") != "EXEC":
            print("Entering EXEC (no motion).")
            robot.enter_exec_mode(wait=True)

        print("\n1. A target past the software limit is refused and J5 does not move")
        f = move(robot, with_j5(q0, near), args.speed, args.timeout)
        check("approach to the legal pose completes", not f, str(codes(f)))
        at_near = float(joints(robot)[J])
        f = move(robot, with_j5(q0, past), args.speed, args.timeout)
        check("J5 past its limit -> JOINT_LIMIT", codes(f) == ["JOINT_LIMIT"], str(f))
        time.sleep(0.5)
        moved = abs(float(joints(robot)[J]) - at_near)
        check("J5 did not move", moved < 0.5, f"moved {moved:.2f} deg")

        print("\n2. The program is aborted until clear_errors")
        try:
            robot.move_joints(*[float(x) for x in with_j5(q0, j0)], speed=args.speed)
            refused = None
        except CommandRejected as exc:
            refused = exc.code
        check("the next move is refused with MOTION_ABORTED", refused == "MOTION_ABORTED", str(refused))
        check("status reports the abort", bool(robot.status().get("motion_aborted")))
        robot.clear_errors(wait=True)
        time.sleep(0.2)
        check("clear_errors clears it", not robot.status().get("motion_aborted"), str(robot.status().get("motion_aborted")))
        f = move(robot, q0, args.speed, args.timeout)
        check("motion works again (back to start)", not f, str(codes(f)))

        print("\n3. Async program [legal, illegal, legal]: stops at the failure")
        robot.move_joints(*[float(x) for x in with_j5(q0, near)], speed=args.speed)
        # The planner can refuse a JOINT_LIMIT target within a millisecond of picking it up (command_executor
        # registers the id BEFORE queueing for exactly that reason). Whether the refusal arrives with the ack —
        # CommandRejected here — or as a failure event later is a race the executor does not hide, and this
        # program must stop at the failure either way (seen both ways on the arm, 2026-10-03).
        sync_reject = None
        try:
            robot.move_joints(*[float(x) for x in with_j5(q0, past)], speed=args.speed)
        except CommandRejected as e:
            sync_reject = e
        late_abort = None
        if sync_reject is None:
            # ... and the third outcome (seen in the simulator, 2026-10-13): the refusal lands between the two acks,
            # so the abort refuses the next command AT ITS ACK — the program stops at the failure all the same.
            try:
                robot.move_joints(*[float(x) for x in q0], speed=args.speed)
            except CommandRejected as e:
                late_abort = e
        f = robot.wait_idle(timeout=args.timeout, raise_on_error=False)
        got = (["JOINT_LIMIT"] if sync_reject is not None else []) + codes(f) \
            + (["ABORTED"] if late_abort is not None and late_abort.code == "ABORTED" else [])
        check("failures are [JOINT_LIMIT, ABORTED] (or JOINT_LIMIT at the ack, program not continued)",
              got in (["JOINT_LIMIT", "ABORTED"], ["JOINT_LIMIT"]),
              f"{got}" + (" — rejected at the ack" if sync_reject is not None else ""))
        stay = abs(float(joints(robot)[J]) - near)
        check("the move after the failure never ran (J5 still at the legal pose)", stay < 0.5, f"{stay:.2f} deg off")
        robot.clear_errors(wait=True)
        move(robot, q0, args.speed, args.timeout)

        print("\n4. After jogging, the next EXEC move starts where the arm is")
        a = j0 + 4.0 * (-sign)              # away from the near limit
        f = move(robot, with_j5(q0, a), args.speed, args.timeout)
        check("move to A completes", not f, str(codes(f)))
        # EXEC and JOG only connect through IDLE (utility/state_machine.py VALID_TRANSITIONS):
        # asked for directly, RTI refuses the mode change and the client just times out.
        robot.enter_idle_mode(wait=True)
        robot.enter_jog_mode(wait=True)
        direction = int(-sign)
        robot.jog(J, direction, 0.03)
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end and abs(float(joints(robot)[J]) - a) < 3.0:
            time.sleep(0.02)
        robot.jog_stop()
        time.sleep(0.8)
        b = float(joints(robot)[J])
        check("jogged about 3 deg away from A", abs(b - a) > 1.5, f"A={a:.2f} B={b:.2f}")
        robot.enter_idle_mode(wait=True)
        robot.enter_exec_mode(wait=True)
        c = b + 2.0 * np.sign(b - a)
        robot.move_joints(*[float(x) for x in with_j5(joints(robot), c)], speed=args.speed)
        toward_a = 0.0
        t_end = time.monotonic() + args.timeout
        while time.monotonic() < t_end:
            now = float(joints(robot)[J])
            toward_a = max(toward_a, (b - now) * np.sign(b - a))     # >0 = moved back toward A
            st = robot.status().get("planner") or {}
            if not st.get("active_command") and not st.get("queue_depth") and abs(now - c) < 0.3:
                break
            time.sleep(0.02)
        robot.wait_idle(timeout=args.timeout, raise_on_error=False)
        check("J5 never snapped back toward the old endpoint A", toward_a < 0.5, f"max {toward_a:.2f} deg toward A")
        move(robot, q0, args.speed, args.timeout)

        print("\n5. Leaving EXEC mid-move fails the pending command at once")
        robot.move_joints(*[float(x) for x in with_j5(q0, j0 + 15.0 * (-sign))], speed=max(2.0, args.speed / 2))
        time.sleep(0.6)
        t0 = time.monotonic()
        robot.enter_idle_mode(wait=True)
        f = robot.wait_idle(timeout=30.0, raise_on_error=False)
        dt = time.monotonic() - t0
        check("the interrupted move fails ABORTED", codes(f) == ["ABORTED"], str(codes(f)))
        check("...within seconds, not the 120 s timeout", dt < 5.0, f"{dt:.1f} s")
        check("status reports the abort", bool(robot.status().get("motion_aborted")))
        robot.clear_errors(wait=True)
        robot.enter_exec_mode(wait=True)
        f = move(robot, q0, args.speed, args.timeout)
        check("back to the start pose", not f, str(codes(f)))
    finally:
        try:
            robot.jog_stop()
        except Exception:
            pass
        try:
            robot.release_control()          # hand the arm back to the dashboard
        except Exception:
            pass
        robot.disconnect()

    print(f"\n{_ok} passed, {len(_bad)} failed" + (f": {_bad}" if _bad else ""))
    return 1 if _bad else 0


if __name__ == "__main__":
    sys.exit(main())
