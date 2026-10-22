"""Which drive firmware this runtime supports (2026-10-21, versioning step V4).

    SUPPORTED[driver_type] = (oldest, newest)   newest None = no upper bound
    check(driver_type, version)   -> None | "ok" | "too_old" | "too_new" | "undeclared"
    range_text(driver_type)       -> "103 or newer" | "103-109" | "not declared"

A property of THE RUNTIME, not of an arm: it is what this runtime's CAN code works with, the same for every robot
that uses these drives — so it lives here, changes with a runtime release, and goes into the compatibility table; the
robot XML says only which driver each joint has (<driver_type>). The user, 2026-10-21: "103 and forward are
supported" (every joint and the gripper ran 103 then; the flasher's newest cached release was 106).

A drive outside the range is a WARNING at run time (the executor logs it, the Versions card marks it) and a FAIL in
the first-run check (tools/first_run.py) — never a stop: an older firmware may well run, it is just not what this
runtime was made for. A version <= 0 is one the drive has not reported yet (shared memory starts at 0): no verdict.
"""
from __future__ import annotations

SUPPORTED: dict[str, tuple[int, int | None]] = {
    "stepfoc": (103, None),          # the joints' drives (and the MSG gripper's)
    "spectral-bldc": (103, None),    # the SSG48 gripper's drive
}


def check(driver_type, version) -> str | None:
    """The verdict on one drive's firmware: None when it has not reported one, "undeclared" when the robot / gripper
    XML names no driver type this runtime knows, else "ok" / "too_old" / "too_new"."""
    try:
        v = int(version)
    except (TypeError, ValueError):
        return None
    if v <= 0:
        return None
    if driver_type not in SUPPORTED:
        return "undeclared"
    oldest, newest = SUPPORTED[driver_type]
    if v < oldest:
        return "too_old"
    if newest is not None and v > newest:
        return "too_new"
    return "ok"


def range_text(driver_type) -> str:
    """The supported range for people: "103 or newer", "103-109", or "not declared"."""
    if driver_type not in SUPPORTED:
        return "not declared"
    oldest, newest = SUPPORTED[driver_type]
    return f"{oldest} or newer" if newest is None else f"{oldest}-{newest}"
