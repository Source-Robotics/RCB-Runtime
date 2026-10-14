"""Client exceptions."""


class RobotError(Exception):
    """Base for all robotclient errors."""


class RobotConnectionError(RobotError):
    """TCP connection could not be established or was lost."""


class RobotTimeout(RobotError):
    """A command did not receive its expected reply in time."""


class CommandRejected(RobotError):
    """The robot rejected or failed a command (status == 'error')."""

    def __init__(self, code, message):
        self.code = code
        self.message = message
        super().__init__(f"[{code}] {message}")


class MotionFailed(RobotError):
    """One or more already-queued motion commands failed while executing.

    Distinct from CommandRejected, which is a validation-time refusal returned
    before the command was ever accepted. These failures arrive asynchronously,
    after the "queued" ack, so the caller has already moved on — they surface at
    the next wait_idle().

    `failures` is a list of {"id", "code", "message"} dicts, in arrival order.
    """

    def __init__(self, failures):
        self.failures = list(failures)
        n = len(self.failures)
        detail = "; ".join(f"{f['id']}: [{f['code']}] {f['message']}" for f in self.failures)
        noun = "command" if n == 1 else "commands"
        super().__init__(f"{n} queued motion {noun} failed — {detail}")


class GripFailed(RobotError):
    """A gripper command completed, but not with the outcome that was awaited.

    Deliberately NOT a MotionFailed: nothing went wrong with the command. The
    jaws moved and settled exactly as asked — they just closed on nothing, or
    opened while still holding something. That is a process failure (the part
    was not where it should be), not a robot failure, and the two want different
    handling: retry the pick versus stop the machine.

    Raised by wait_holding_part() / wait_released_part() so a missed grip cannot
    be silently ignored the way an unchecked `if` can be.

    `result` is the GripperResult that was actually observed.
    """

    def __init__(self, expected, result):
        self.expected = expected
        self.result = result
        got = result.name if result is not None else "no result"
        super().__init__(f"expected the gripper to be {expected}, but it reported: {got}")


class HomingFailed(RobotError):
    """home(wait=True) / wait_homed(): the homing ended in an error instead of homed (2026-10-14). The robot is
    disabled with the error latched — J{n}:HOMING_FAILED names the joint, its reason is in the dashboard's error list
    and the activity log; Clear Errors, Enable and home again. Until then the wait ran into its 180 s timeout and
    reported only "timed out".

    `errors` is the list of error keys the robot reported when the homing ended.
    """

    def __init__(self, errors):
        self.errors = list(errors)
        listed = ", ".join(self.errors) or "none reported"
        super().__init__(f"homing failed — errors: {listed} (the reason: the dashboard's error list or "
                         f"logs/activity_rti.log)")
