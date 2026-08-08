#!/usr/bin/env python3
from _common import parser, connect, require_go, prepare_exec, finish, main_guard


def run():
    p = parser("05: tiny real Cartesian relative moves")
    p.add_argument("--dx", type=float, default=0.001, help="small Cartesian offset in meters")
    p.add_argument(
        "--one",
        choices=["wrf_x", "wrf_neg_x", "trf_x", "trf_neg_x", "all"],
        default="wrf_x",
        help="which Cartesian hardware move to run; default is one WRF +X move")
    p.add_argument(
        "--cartesian-hardware-ack", action="store_true",
        help="required in addition to --go for real Cartesian hardware motion")
    args = p.parse_args()
    robot = connect(args)
    if require_go(args, f"move TCP by +/-{args.dx} m in WRF and TRF"):
        if not args.cartesian_hardware_ack:
            print("SAFETY BLOCK: Cartesian hardware motion needs --cartesian-hardware-ack.")
            print("Run api_tests/04_cart_dry.py first and confirm IK diagnostics are branch-continuous.")
            raise SystemExit(2)
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")

        move_map = {
            "wrf_x": (robot.move_linear_rel_world, (args.dx, 0, 0), "WRF +X"),
            "wrf_neg_x": (robot.move_linear_rel_world, (-args.dx, 0, 0), "WRF -X"),
            "trf_x": (robot.move_linear_rel_tool, (args.dx, 0, 0), "TRF +X"),
            "trf_neg_x": (robot.move_linear_rel_tool, (-args.dx, 0, 0), "TRF -X"),
        }
        if args.one == "all":
            moves = tuple(move_map.values())
        else:
            moves = (move_map[args.one],)

        for fn, move_args, label in moves:
            print(f"Preflight dry-run {label}")
            fn(*move_args, dry_run=True, ik_solver=args.ik, samples=4, speed=args.speed, timeout=args.timeout)
            robot.wait_motion_complete(timeout=args.timeout)
            print(f"Hardware move {label}")
            fn(*move_args, ik_solver=args.ik, samples=4, speed=args.speed, timeout=args.timeout)
            robot.wait_motion_complete(timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
