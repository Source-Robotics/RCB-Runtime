#!/usr/bin/env python3
from _common import parser, connect, require_go, prepare_exec, show, finish, main_guard


def run():
    p = parser("04: Cartesian IK dry-run checks; no trajectory should move")
    p.add_argument("--dx", type=float, default=0.001, help="small Cartesian offset in meters")
    args = p.parse_args()
    robot = connect(args)
    if require_go(args, "prepare EXEC and run Cartesian dry-run IK checks"):
        prepare_exec(robot, args)
        pose = robot.get_pose()
        robot.move_linear(*pose, dry_run=True, ik_solver=args.ik, samples=4, timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        robot.move_linear_rel_world(args.dx, 0, 0, dry_run=True, ik_solver=args.ik, samples=4, timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        robot.move_linear_rel_tool(args.dx, 0, 0, dry_run=True, ik_solver=args.ik, samples=4, timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        show("planner diagnostics", robot.get_status().get("planner"))
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
