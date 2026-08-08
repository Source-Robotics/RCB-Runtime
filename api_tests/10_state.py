#!/usr/bin/env python3
from _common import parser, connect, require_go, finish, show, main_guard


def _require_ready_status(robot):
    status = robot.get_status()
    show("current status", status)
    problems = []
    if status.get("state") != "ENABLED":
        problems.append(f"state={status.get('state')!r}")
    if status.get("mode") != "EXEC":
        problems.append(f"mode={status.get('mode')!r}")
    if status.get("homed") is not True:
        problems.append(f"homed={status.get('homed')!r}")
    if problems:
        raise RuntimeError("disable-cycle starts from EXEC + ENABLED + homed; got " + ", ".join(problems))
    return status


def _disable_cycle(robot, args):
    if not args.disable_hardware_ack:
        print("DRY RUN: disable-cycle needs --disable-hardware-ack because disabling may remove motor torque.")
        print("Re-run only when the arm is physically safe/supported.")
        return

    _require_ready_status(robot)
    print("Disabling robot state")
    robot.disable(wait=True)
    disabled = robot.get_status()
    show("after disable", disabled)
    if disabled.get("state") != "DISABLED":
        raise RuntimeError(f"expected DISABLED state, got {disabled.get('state')!r}")

    if not args.recover:
        print("Leaving robot disabled because --no-recover was requested.")
        return

    print("Recovering: enable")
    robot.enable(wait=True)
    enabled = robot.get_status()
    show("after enable", enabled)
    if enabled.get("state") != "ENABLED":
        raise RuntimeError(f"expected ENABLED state after recovery, got {enabled.get('state')!r}")

    if enabled.get("homed") is True:
        print("Recovering: enter EXEC mode")
        robot.enter_exec_mode(wait=True)
        show("after enter_exec_mode", robot.get_status())
    else:
        print("Robot is enabled but not homed after disable; not homing automatically.")


def run():
    p = parser("10: guarded state-control checks")
    p.add_argument("--disable-cycle", action="store_true",
                   help="disable, verify DISABLED, then recover to ENABLED/EXEC if still homed")
    p.add_argument("--disable-hardware-ack", action="store_true",
                   help="required with --disable-cycle because disabling may remove motor torque")
    p.add_argument("--no-recover", dest="recover", action="store_false",
                   help="leave robot disabled after --disable-cycle")
    p.set_defaults(recover=True)
    args = p.parse_args()

    robot = connect(args)
    if args.disable_cycle:
        if require_go(args, "run guarded disable/re-enable cycle"):
            _disable_cycle(robot, args)
    else:
        show("status", robot.get_status())
        print("No state action requested. Use --disable-cycle with --go --disable-hardware-ack to test disable.")
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
