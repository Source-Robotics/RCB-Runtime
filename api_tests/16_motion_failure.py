#!/usr/bin/env python3
"""16: MotionFailed surfacing — a partially-executed motion must NOT look like success.

Reproduces the exact bug found on hardware 2026-07-10: a burst of queued moves where
one fails mid-sequence. The others execute, so the arm really does move — but the
failed one used to vanish silently (in async mode _queued() returns at the "queued"
ack, so nothing was waiting on that command, and wait_idle() only waits for commands
to stop being outstanding — which a FAILED command also does). Result: 3 of 4 legs
ran and the script reported success.

Now wait_idle() raises MotionFailed listing every queued command that failed.

Sequence: +dx (good, executes) -> +far (BAD: fails IK) -> -dx (good, executes).
The bad move fails in the PLANNER, before any trajectory is sent, so it produces NO
motion — the arm just does +dx then -dx and ends where it started.

PASS = wait_idle() raises MotionFailed naming the bad command.
FAIL = wait_idle() returns cleanly (the original bug is back).
"""
from _common import parser, connect, require_go, prepare_exec, show, finish, main_guard
from robotclient import MotionFailed


def run():
    p = parser("16: MotionFailed surfacing (partial execution must not look like success)")
    p.add_argument("--dx-mm", type=float, default=30.0,
                   help="size of the two GOOD moves, in mm (small; +dx then -dx, net zero)")
    p.add_argument("--far-mm", type=float, default=500.0,
                   help="size of the BAD move, in mm. Must be far enough that IK fails in the "
                        "planner — it then sends NO trajectory, so this produces no motion.")
    args = p.parse_args()

    if args.sync:
        raise SystemExit("ERROR: this test needs async mode (the default). --sync makes each "
                         "move block and raise on its own, which is not the bug being tested.")

    robot = connect(args)
    if require_go(args, "queue a good/bad/good burst and check the failure is reported"):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")

        seen = []
        robot.on_motion_error(lambda f: seen.append(f))

        dx = args.dx_mm / 1000.0
        far = args.far_mm / 1000.0
        print(f"\nQueueing: +{args.dx_mm:.0f}mm (good) -> +{args.far_mm:.0f}mm (BAD, no motion) "
              f"-> -{args.dx_mm:.0f}mm (good)")
        print("The arm should move out and back; the middle command should FAIL.\n")

        robot.move_linear_rel_world(dx, 0.0, 0.0, profile=args.profile, speed=args.speed)
        robot.move_linear_rel_world(far, 0.0, 0.0, profile=args.profile, speed=args.speed)
        robot.move_linear_rel_world(-dx, 0.0, 0.0, profile=args.profile, speed=args.speed)

        failed = None
        try:
            robot.wait_idle(timeout=args.timeout)
        except MotionFailed as exc:
            failed = exc

        print("=" * 68)
        if failed is None:
            print("RESULT: *** FAIL *** — wait_idle() returned cleanly even though a queued")
            print("        command failed. That is the original bug: a partially-executed")
            print("        motion is being reported as success.")
        else:
            print("RESULT: *** PASS *** — wait_idle() raised MotionFailed, as it must.")
            print(f"        {failed}")
            for f in failed.failures:
                print(f"          - id={f['id']} code={f['code']} message={f['message']}")
            if seen:
                print(f"        on_motion_error callback also fired {len(seen)} time(s) "
                      f"(immediate notification works).")
        print("=" * 68)

        show("final status", robot.get_status())

    finish(robot)


if __name__ == "__main__":
    main_guard(run)
