#!/usr/bin/env python3
from _common import parser, connect, require_go, prepare_exec, finish, show, main_guard
from robotclient import CommandRejected


def run():
    p = parser("08: callback and rejected-command path checks")
    p.add_argument("--deg", type=float, default=1.0, help="J1 nudge in degrees")
    args = p.parse_args()

    robot = connect(args)
    status_events = []
    motion_complete_ids = []
    error_events = []

    def on_status(status):
        status_events.append({
            "mode": status.get("mode"),
            "state": status.get("state"),
            "homed": status.get("homed"),
            "executing_seq": status.get("executing_seq"),
            "completed_seq": status.get("completed_seq"),
            "active_command": status.get("active_command"),
        })

    def on_motion_complete(command_id):
        motion_complete_ids.append(command_id)
        print(f"motion callback: complete id={command_id}")

    def on_error(msg):
        error_events.append(msg)
        print(f"error callback: {msg}")

    robot.on_status_change(on_status)
    robot.on_motion_complete(on_motion_complete)
    robot.on_error(on_error)

    if require_go(args, f"move J1 by {args.deg} degrees, return, and test rejection path"):
        prepare_exec(robot, args)
        robot.set_completion_policy("settled")

        start = [float(v) for v in robot.get_joints()]
        target = list(start)
        target[0] += args.deg

        print(f"Moving J1 relative: {start[0]:.3f} deg -> {target[0]:.3f} deg")
        robot.move_joints(target, profile=args.profile, speed=args.speed, timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)

        print("Returning to starting joint position")
        robot.move_joints(start, profile=args.profile, speed=args.speed, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)

        rejected = []
        # `accel` and `blend_mode` WERE here — both are supported per-command options now (motion/exec_commands.py);
        # the simulated suite found the stale cases 2026-10-13. This is the option move_joints still refuses.
        rejection_cases = [
            ("completion_policy", {"completion_policy": "strict"}),
        ]
        for label, options in rejection_cases:
            try:
                robot.move_joints(start, profile=args.profile, speed=args.speed,
                                  timeout=args.timeout, **options)
            except CommandRejected as exc:
                item = {"option": label, "code": exc.code, "message": exc.message}
                rejected.append(item)
                print(f"expected rejection ({label}): code={exc.code} message={exc.message}")

        show("callback summary", {
            "status_event_count": len(status_events),
            "last_status_event": status_events[-1] if status_events else None,
            "motion_complete_count": len(motion_complete_ids),
            "motion_complete_ids": motion_complete_ids,
            "unsolicited_error_event_count": len(error_events),
            "expected_rejections": rejected,
        })

        if len(motion_complete_ids) < 2:
            raise RuntimeError("expected at least two motion-complete callbacks")
        if not status_events:
            raise RuntimeError("expected at least one status callback")
        rejected_by_option = {item["option"]: item for item in rejected}
        for label, _options in rejection_cases:
            item = rejected_by_option.get(label)
            if item is None or item.get("code") != "BAD_OPTIONS":
                raise RuntimeError(f"expected BAD_OPTIONS rejection for unsupported {label} option")

    finish(robot)


if __name__ == "__main__":
    main_guard(run)
