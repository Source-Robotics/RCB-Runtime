"""Free one joint so it can be turned by hand (2026-10-18; the first-run check's S3, TODO/first_run_plan.md — user:
"prompt the user to move the robot by hand ... just motors off and guide the user how to spin the stuff").

The joint's drive gets current limit 0 (the Hall sensor check's mechanism, robotics/hall_check.py, verified on J6):
whatever its mode — IDLE's speed-0 hold, or the gravity-compensation current of a homed freedrive — the drive clamps
its current, and its speed loop's integral, to that limit, so the joint makes no torque. Its encoder keeps counting,
so `joint_position` follows the hand. Every OTHER joint keeps its normal command and limits: the arm holds its shape
(a joint carrying the arm's weight — J2, J3 — falls when freed unless someone supports it; the check says so first).

RTI calls start() / stop() from its `free_joint:<n>` command (n = 1..joint_num, 0 = stop) and tick() every tick while
active, BEFORE its mode management: a pending mode change (homing, a jog, an error, the E-stop) or leaving IDLE stops
it and puts the joint's live limits back before the new mode's first command. Only one joint at a time, never beside
the Hall sensor check (both use the joint's current limit). A joint left free TIMEOUT_TICKS (2 min) without the same
command again is held again by RTI itself — a client that dies mid-check (killed, a lost connection) must not leave
J2 limp for good.
"""
from __future__ import annotations

LIMIT_REPEATS = 2            # each limits frame sent twice (the CAN library swallows a dropped frame)
RESEND_TICKS = 125           # the zero limit re-sent every ~0.5 s at 250 Hz (a reconnect reload would restore the limit)
TIMEOUT_TICKS = 30000        # 2 min at 250 Hz: held again unless free_joint:<n> came again (each renews it)


class FreeJoint:
    """One joint limp for hand turning. `motors` are the CAN library's motor objects (Send_Limits), `velocity_limit` /
    `current_limit` the live per-joint limits (what stop() restores; PID apply edits these arrays in place), `rti` the
    rti shared-memory dict (`free_joint`: the freed joint's number, 0 = none)."""

    def __init__(self, motors, joint_num, velocity_limit, current_limit, rti, log=None):
        self.motors = motors
        self.joint_num = int(joint_num)
        self.velocity_limit = velocity_limit
        self.current_limit = current_limit
        self.rti = rti
        self.log = log
        self.joint = None                       # 0-based while a joint is free
        self._ticks = 0
        self.rti["free_joint"][0] = 0

    @property
    def active(self) -> bool:
        return self.joint is not None

    # ------------------------------------------------------------------------------------------------ cold path
    def _send_limits(self, j, current):
        for _ in range(LIMIT_REPEATS):
            self.motors[j].Send_Limits(Velocity_limit=int(self.velocity_limit[j]), Current_limit=int(current))

    def start(self, joint, idle, mode_pending, booting, hall_check_active=False):
        """Free `joint` (0-based). Returns None, or why it was refused (nothing is sent then). Freeing another joint
        while one is free gives the first its limits back first."""
        if not 0 <= int(joint) < self.joint_num:
            return f"no joint {int(joint) + 1} (this robot has J1-J{self.joint_num})"
        if not idle or mode_pending:
            return "only in IDLE (enabled or not, homed or not)"
        if booting:
            return "RTI is still configuring the drives (the first ~1.2 s after its start)"
        if hall_check_active:
            return "the Hall sensor check is running — stop it first"
        joint = int(joint)
        if self.joint == joint:
            self._ticks = 0                     # renewed: another TIMEOUT_TICKS
            return None
        if self.active:
            self.stop(f"J{joint + 1} freed instead")
        self._send_limits(joint, 0)
        self.joint = joint
        self._ticks = 0
        self.rti["free_joint"][0] = joint + 1
        if self.log:
            self.log(f"J{joint + 1} free (current limit 0) — it can be turned by hand")
        return None

    def stop(self, reason="stopped"):
        if not self.active:
            return
        j = self.joint
        self._send_limits(j, self.current_limit[j])
        self.joint = None
        self.rti["free_joint"][0] = 0
        if self.log:
            self.log(f"J{j + 1} held again ({reason}); its limits restored ({int(self.velocity_limit[j])} / "
                     f"{int(self.current_limit[j])} mA)")

    # ------------------------------------------------------------------------------------------------ per tick
    def tick(self, idle_and_no_mode_change):
        """Every tick while active, before RTI's mode management. No allocation on the normal path."""
        if not idle_and_no_mode_change:
            self.stop("the robot left IDLE or a mode change was requested")
            return
        self._ticks += 1
        if self._ticks >= TIMEOUT_TICKS:
            self.stop("2 minutes without a renewal")
            return
        if self._ticks % RESEND_TICKS == 0:
            self.motors[self.joint].Send_Limits(Velocity_limit=int(self.velocity_limit[self.joint]), Current_limit=0)
