#!/usr/bin/env python3
import time

from _common import parser, connect, require_go, prepare_exec, finish, show, main_guard


def _assert_paused(robot, expected, timeout, label):
    robot.wait(lambda s: bool(s.get("exec_paused")) is expected, timeout=timeout, what=label)
    status = robot.get_status()
    show(label, status)
    if bool(status.get("exec_paused")) is not expected:
        raise RuntimeError(f"expected exec_paused={expected}, got {status.get('exec_paused')!r}")
    return status


def run():
    p = parser("11: pause/resume during a slow tiny joint move")
    p.add_argument("--deg", type=float, default=5.0, help="J1 move size in degrees")
    p.add_argument("--pause-after", type=float, default=0.5, help="seconds after queueing move before pause")
    p.add_argument("--hold", type=float, default=1.0, help="seconds to hold paused before resume")
    args = p.parse_args()

    if args.sync:
        raise RuntimeError("pause/resume test must run async; omit --sync")

    robot = connect(args)
    if require_go(args, f"pause/resume during J1 {args.deg} degree move"):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")
        robot.set_blending(0)

        start = [float(v) for v in robot.get_joints()]
        target = list(start)
        target[0] += args.deg

        print(f"Starting slow J1 move: {start[0]:.3f} deg -> {target[0]:.3f} deg")
        move_id = robot.move_joints(target, profile=args.profile, speed=args.speed, timeout=args.timeout)
        print(f"queued move id={move_id}; pausing after {args.pause_after:.2f}s")
        time.sleep(max(0.0, args.pause_after))

        print("Pausing")
        robot.pause(wait=True)
        paused_status = _assert_paused(robot, True, args.timeout, "paused status")
        paused_joints = [float(v) for v in robot.get_joints()]
        print("paused joints:", [round(v, 3) for v in paused_joints])

        time.sleep(max(0.0, args.hold))
        held_joints = [float(v) for v in robot.get_joints()]
        drift = max(abs(a - b) for a, b in zip(paused_joints, held_joints))
        print(f"max joint drift while paused: {drift:.4f} deg")

        print("Resuming")
        robot.resume(wait=True)
        _assert_paused(robot, False, args.timeout, "resumed status")
        robot.wait_motion_complete(timeout=args.timeout)
        after_target = [float(v) for v in robot.get_joints()]
        print("after target:", [round(v, 3) for v in after_target])

        print("Returning to starting joint position")
        robot.move_joints(start, profile=args.profile, speed=args.speed, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)

        show("pause/resume summary", {
            "move_id": move_id,
            "pause_after_s": args.pause_after,
            "hold_s": args.hold,
            "max_pause_drift_deg": drift,
            "start_j1_deg": start[0],
            "target_j1_deg": target[0],
            "paused_j1_deg": paused_joints[0],
            "after_target_j1_deg": after_target[0],
        })
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
