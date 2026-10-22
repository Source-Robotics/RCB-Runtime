"""Client exceptions."""


class RobotError(Exception):
    """Base for all robotclient errors."""


class RobotConnectionError(RobotError):
    """TCP connection could not be established or was lost."""


class VersionMismatch(RobotConnectionError):
    """The robot speaks another version of a protocol this client uses (2026-10-21, versioning V3): raised by connect()
    before anything is sent; connect(..., allow_mismatch=True) skips the check. robotclient/version.py."""


class VersionWarning(UserWarning):
    """The robot reports no protocol versions (a runtime from before versioning): connect() goes on and warns once
    (Python's warnings module; filter it like any warning)."""


class RobotTimeout(RobotError):
    """A command did not receive its expected reply in time."""


class CommandRejected(RobotError):
    """The robot rejected or failed a command (status == 'error').

    `code` and `message` as before; since 2026-10-15 also the catalog's `number`, `title` and `fix` when the robot
    sent them (docs/error_codes.md): `str(exc)` reads "[6003 BAD_ARGS] Invalid arguments: <message> Fix: ...".
    """

    def __init__(self, code, message, number=None, title=None, fix=None):
        self.code = code
        self.message = message
        self.number = number
        self.title = title
        self.fix = fix
        super().__init__(format_error(code, message, number, title, fix))


def format_error(code, message, number=None, title=None, fix=None):
    """One line for a person: "[6003 BAD_ARGS] Invalid arguments: the message. Fix: ...". Without the catalog
    fields (an older robot): "[BAD_ARGS] the message"."""
    head = f"[{number} {code}]" if number is not None else f"[{code}]"
    body = str(message or "")
    if title and title.lower() not in body.lower():
        body = f"{title}: {body}" if body else title
    text = f"{head} {body}".rstrip()
    if fix:
        text += (" " if text.endswith((".", "!", "?")) else ". ") + f"Fix: {fix}"
    return text


class MotionFailed(RobotError):
    """One or more already-queued motion commands failed while executing.

    Distinct from CommandRejected, which is a validation-time refusal returned
    before the command was ever accepted. These failures arrive asynchronously,
    after the "queued" ack, so the caller has already moved on — they surface at
    the next wait_idle().

    `failures` is a list of {"id", "code", "message"} dicts, in arrival order — with "number", "title" and "fix"
    when the robot sent them (2026-10-15).
    """

    def __init__(self, failures):
        self.failures = list(failures)
        n = len(self.failures)
        detail = "; ".join(f"{f['id']}: " + format_error(f.get("code"), f.get("message"), f.get("number"),
                                                        f.get("title"), f.get("fix")) for f in self.failures)
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
