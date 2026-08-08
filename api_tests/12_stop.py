#!/usr/bin/env python3
import time

from _common import parser, connect, require_go, prepare_exec, finish, show, main_guard
from robotclient import CommandRejected


def run():
    p = parser("12: stop during a slow tiny joint move")
    p.add_argument("--deg", type=float, default=5.0, help="J1 move size in degrees")
    p.add_argument("--stop-after", type=float, default=0.5, help="seconds after queueing move before stop")
    p.add_argument("--no-return", dest="return_after_stop", action="store_false",
                   help="do not re-enter EXEC and return to start after stop")
    p.set_defaults(return_after_stop=True)
    args = p.parse_args()

    if args.sync:
        raise RuntimeError("stop test must run async; omit --sync")

    robot = connect(args)
    if require_go(args, f"stop during J1 {args.deg} degree move"):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")
        robot.set_blending(0)

        start = [float(v) for v in robot.get_joints()]
        target = list(start)
        target[0] += args.deg

        print(f"Starting slow J1 move: {start[0]:.3f} deg -> {target[0]:.3f} deg")
        move_id = robot.move_joints(target, profile=args.profile, speed=args.speed, timeout=args.timeout)
        print(f"queued move id={move_id}; stopping after {args.stop_after:.2f}s")
        time.sleep(max(0.0, args.stop_after))

        print("Stopping")
        robot.stop(wait=True)
        stopped_status = robot.get_status()
        show("after stop", stopped_status)
        if stopped_status.get("mode") != "IDLE":
            raise RuntimeError(f"expected mode IDLE after stop, got {stopped_status.get('mode')!r}")

        stopped_error = None
        try:
            robot.wait_motion_complete(timeout=1.0)
        except CommandRejected as exc:
            stopped_error = {"code": exc.code, "message": exc.message}
            print(f"pending move rejected after stop: code={exc.code} message={exc.message}")

        stopped_joints = [float(v) for v in robot.get_joints()]
        print("stopped joints:", [round(v, 3) for v in stopped_joints])

        if args.return_after_stop:
            if stopped_status.get("state") != "ENABLED" or stopped_status.get("homed") is not True:
                print("Not returning: robot is not ENABLED+homed after stop.")
            else:
                print("Re-entering EXEC mode")
                robot.enter_exec_mode(wait=True)
                print("Returning to starting joint position")
                robot.move_joints(start, profile=args.profile, speed=args.speed, timeout=args.timeout)
                robot.wait_idle(timeout=args.timeout)

        show("stop summary", {
            "move_id": move_id,
            "stop_after_s": args.stop_after,
            "start_j1_deg": start[0],
            "target_j1_deg": target[0],
            "stopped_j1_deg": stopped_joints[0],
            "pending_move_error": stopped_error,
            "returned_to_start": bool(args.return_after_stop),
        })
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
