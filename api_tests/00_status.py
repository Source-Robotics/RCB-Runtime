#!/usr/bin/env python3
from _common import parser, connect, show_basic, finish, main_guard


def run():
    args = parser("00: connect and read status/joints/pose").parse_args()
    robot = connect(args)
    show_basic(robot)
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
