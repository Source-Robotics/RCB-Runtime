"""The restore of a joint's live limits, sent again over a second (review D14, 2026-11-20).

The Hall sensor check, a freed joint and a joint test each lower one joint's current limit (0 mA, or the test
current) and restore the live limits when they end — two back-to-back frames, no answer. The CAN library swallows a
frame it could not queue (a full send queue: a node missing, a bus fault), so both could be lost and the joint would
stay limp (0 mA) or weak with nothing noticing until the next reconnect reload. Here the restore is sent RESENDS more
times, every EVERY_TICKS, while the robot stays in IDLE.

Only in IDLE with no mode change pending: homing sends its own (lower) current limits on entry, and a resend after
that would replace them. Leaving IDLE, or the next start of any of the three, ends it (they never run together; RTI
gives all three ONE RestoreResend). The values are read from the live limit arrays at each resend, so a
set_pid_gains in between is honoured.
"""
from __future__ import annotations

EVERY_TICKS = 50             # 0.2 s at 250 Hz
RESENDS = 5                  # five more frames over ~1 s after the restore


class RestoreResend:
    """`motors` the CAN library's motor objects (Send_Limits), `velocity_limit` / `current_limit` the live per-joint
    limits (the arrays the three tests restore from)."""

    __slots__ = ("motors", "velocity_limit", "current_limit", "joint", "left", "_ticks")

    def __init__(self, motors, velocity_limit, current_limit):
        self.motors = motors
        self.velocity_limit = velocity_limit
        self.current_limit = current_limit
        self.joint = None                       # 0-based while resends are pending
        self.left = 0
        self._ticks = 0

    @property
    def pending(self) -> bool:
        return self.joint is not None

    def arm(self, joint):
        """A joint's limits were just restored: send them RESENDS more times over the next second."""
        self.joint, self.left, self._ticks = int(joint), RESENDS, 0

    def cancel(self):
        self.joint = None

    def tick(self, idle_and_no_mode_change):
        """Every tick while pending. No allocation on the normal path."""
        if not idle_and_no_mode_change:
            self.joint = None
            return
        self._ticks += 1
        if self._ticks % EVERY_TICKS:
            return
        j = self.joint
        self.motors[j].Send_Limits(Velocity_limit=int(self.velocity_limit[j]), Current_limit=int(self.current_limit[j]))
        self.left -= 1
        if self.left <= 0:
            self.joint = None
