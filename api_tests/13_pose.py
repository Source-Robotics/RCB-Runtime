#!/usr/bin/env python3
from _common import parser, connect, require_go, prepare_exec, show, finish, main_guard


def run():
    p = parser("13: endpoint move_pose current/near-current validation")
    p.add_argument("--pose-hardware-ack", action="store_true",
                   help="required before sending a real move_pose hardware trajectory")
    p.add_argument("--dx", type=float, default=0.0,
                   help="optional absolute target offset in world X meters from current pose")
    args = p.parse_args()

    robot = connect(args)
    if require_go(args, "prepare EXEC and validate move_pose endpoint IK"):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")
        pose = [float(v) for v in robot.get_pose()]
        target = list(pose)
        target[0] += args.dx

        show("current pose", pose)
        show("target pose", target)

        print("Preflight dry-run move_pose")
        robot.move_pose(*target, dry_run=True, ik_solver=args.ik, profile=args.profile,
                        speed=args.speed, timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        planner = robot.get_status().get("planner") or {}
        show("planner diagnostics after dry-run", planner)
        diag = planner.get("last_diagnostic") or {}
        if diag.get("command") != "move_pose" or diag.get("dry_run") is not True:
            raise RuntimeError(
                "move_pose dry-run diagnostics are missing. Restart command_executor.py "
                "so the patched planner with move_pose dry-run support is loaded before hardware testing.")
        if diag.get("hardware_enabled") is not False:
            raise RuntimeError("move_pose dry-run did not report hardware_enabled=false")

        if not args.pose_hardware_ack:
            print("DRY RUN ONLY: add --pose-hardware-ack to send the hardware move_pose trajectory.")
        else:
            print("Hardware move_pose")
            robot.move_pose(*target, ik_solver=args.ik, profile=args.profile,
                            speed=args.speed, timeout=args.timeout)
            robot.wait_idle(timeout=args.timeout)
            show("planner diagnostics after hardware", robot.get_status().get("planner"))
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
