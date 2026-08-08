#!/usr/bin/env python3
from _common import parser, connect, require_go, prepare_exec, show_basic, finish, main_guard


def run():
    args = parser("01: clear errors, enable, home, enter EXEC").parse_args()
    robot = connect(args)
    show_basic(robot)
    if require_go(args, "clear errors, enable, home, and enter EXEC mode"):
        prepare_exec(robot, args)
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
