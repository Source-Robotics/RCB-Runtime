#!/usr/bin/env python3
"""24: EXEC control-link watchdog (GUI migration Phase 5) — HARDWARE test.

Starts a long, continuous EXEC motion so you have a wide window to KILL command_executor
mid-move in another terminal. What you are testing is the ARM, not this script:

    if command_executor dies while EXEC motion is in flight, RTI must trip EXEC_LINK_LOST
    on its own and stop the arm (disabled + hold), within ~0.5 s.

Killing the executor also drops THIS script's connection — that is expected; the script catches
it and tells you what to watch. Recover afterwards by relaunching command_executor and clicking
Clear Errors (or sending clear_errors) — EXEC_LINK_LOST is a latching error, like an E-STOP.

Usage:
    python3 api_tests/24_exec_link_watchdog.py --go            # sweeps J1, ~25 s window
    python3 api_tests/24_exec_link_watchdog.py --go --span 40 --sweeps 2 --speed 8

Then, WHILE the arm is moving, in another terminal:
    pkill -f command_executor.py       # or: kill <pid from `pgrep -f command_executor.py`>

Sanity pass (do this too): run it and DON'T kill anything — the sweep must finish with NO trip.
"""
from _common import parser, connect, require_go, prepare_exec, goto_start, finish, main_guard
from robotclient import RobotError, RobotConnectionError, RobotTimeout


def run():
    p = parser("24: EXEC-link watchdog — kill command_executor mid-move")
    p.add_argument("--span", type=float, default=55.0,
                   help="J1 sweep amplitude in degrees from the start pose (stays within headroom)")
    p.add_argument("--sweeps", type=int, default=3, help="number of out-and-back J1 sweeps")
    args = p.parse_args()
    robot = connect(args)

    if not require_go(args, "run a long EXEC sweep so you can kill command_executor mid-move"):
        finish(robot)
        return

    prepare_exec(robot, args)
    goto_start(robot, args)                       # J1 starts at 90 deg (START_POSE_DEG)

    start = [float(v) for v in robot.get_joints()]
    j1 = start[0]
    near, far = list(start), list(start)
    far[0] = j1 + args.span                        # +span is within J1's ~+74 deg headroom

    # Queue the whole sweep up front (async): a back-and-forth on J1 so the arm is MOVING
    # (exec_samples_remaining > 0) throughout your kill window. Each leg is many seconds of
    # continuous motion at low speed — kill the executor while J1 is visibly sweeping.
    print(f"\nQueuing {args.sweeps} J1 sweep(s): {j1:.0f} <-> {j1 + args.span:.0f} deg "
          f"at {args.speed:.0f}% speed.")
    for _ in range(args.sweeps):
        robot.move_joints(far,  profile=args.profile, speed=args.speed, timeout=args.timeout)
        robot.move_joints(near, profile=args.profile, speed=args.speed, timeout=args.timeout)

    print("\n" + "=" * 70)
    print("  ARM IS NOW SWEEPING.  While it moves, in ANOTHER terminal run:")
    print("      pkill -f command_executor.py")
    print("  EXPECT: within ~0.5 s the arm STOPS and RTI latches EXEC_LINK_LOST")
    print("          (Errors tab / RTI log). Recover: relaunch command_executor,")
    print("          then Clear Errors.")
    print("  (If you kill nothing, the sweep just finishes — that's the no-false-trip check.)")
    print("=" * 70 + "\n")

    try:
        robot.wait_idle(timeout=args.timeout * (args.sweeps * 2 + 2))
        print("\nSweep finished with NO trip. If you did NOT kill the executor, that's the "
              "expected 'no false trip' pass. If you DID kill it and the arm kept going, "
              "the watchdog FAILED — check the heartbeat wiring.")
    except (RobotConnectionError, RobotTimeout) as exc:
        print(f"\nExecutor link dropped ({type(exc).__name__}: {exc}).")
        print(">>> This is expected if you just killed command_executor. NOW WATCH THE ARM:")
        print(">>>   it should already be stopped (RTI tripped EXEC_LINK_LOST and held).")
        print(">>>   Confirm in the RTI log / GUI Errors tab, then relaunch the executor")
        print(">>>   and Clear Errors to recover.")
    except RobotError as exc:
        print(f"\nMotion error: {type(exc).__name__}: {exc}")

    finish(robot)


if __name__ == "__main__":
    main_guard(run)
