#!/usr/bin/env python3
from _common import parser, connect, require_go, prepare_exec, finish, main_guard


def run():
    p = parser("02: tiny joint nudge, delay, return")
    p.add_argument("--deg", type=float, default=1.0, help="J1 nudge in degrees")
    args = p.parse_args()
    robot = connect(args)
    if require_go(args, f"move J1 by {args.deg} degrees and return"):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")
        start = [float(v) for v in robot.get_joints()]
        target = list(start)
        target[0] += args.deg
        print(f"Moving J1 relative: {start[0]:.3f} deg -> {target[0]:.3f} deg")
        robot.move_joints(target, profile=args.profile, speed=args.speed, timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        robot.delay(0.5, timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        print("Returning to starting joint position")
        robot.move_joints(start, profile=args.profile, speed=args.speed, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
