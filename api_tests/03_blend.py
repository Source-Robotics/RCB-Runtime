#!/usr/bin/env python3
from _common import parser, connect, require_go, prepare_exec, finish, main_guard


def run():
    p = parser("03: tiny velocity-continuous joint blend sequence")
    p.add_argument("--blend", type=float, default=25.0)
    p.add_argument("--deg", type=float, default=1.0)
    args = p.parse_args()
    robot = connect(args)
    if require_go(args, "run a small blended joint sequence"):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")
        robot.set_blending(args.blend)
        start = [float(v) for v in robot.get_joints()]
        p1 = list(start); p1[0] += args.deg
        p2 = list(p1); p2[1] += args.deg
        print("Running small relative blend path:")
        print("  start ->", [round(v, 3) for v in start])
        print("  p1    ->", [round(v, 3) for v in p1])
        print("  p2    ->", [round(v, 3) for v in p2])
        robot.move_joints(p1, profile="trapezoid", speed=args.speed)
        robot.move_joints(p2, profile="trapezoid", speed=args.speed)
        robot.move_joints(start, profile="trapezoid", speed=args.speed)
        robot.wait_idle(timeout=args.timeout)
        robot.set_blending(0)
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
