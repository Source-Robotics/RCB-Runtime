"""The Hall sensor check (2026-10-13, user): the joint homed on a Hall sensor (J6 on PAR6) goes limp, the operator
turns it by hand, and the dashboard's I/O tab shows whether the magnet triggers the sensor — in IDLE, homed or not,
enabled or not.

How, read in the joint firmware (V106 source; the arm ran V103 when this was written — the first run on the arm is the
check): the drive reads its Hall pin only in its Hall-index mode (IN_HALL_INDEX, controller mode 8) and reports it only
in the reply to that command (OUT_DATA_HALL: the position and, in bit 1, the pin — `additional_pin2` in the CAN
library). That mode runs the drive's speed or position loop, so for the check the drive's CURRENT LIMIT is set to 0:
both loops clamp their current command (and the speed loop its integral) to that limit, so the joint makes no torque
whatever its gains or leftover state. The trigger value 3 matches neither pin level (0 / 1 = a level, 2 = an edge),
so the drive never switches to holding the position it latched. Dispatch sends the joint this command INSTEAD of its
data pack every tick (`TickContext.hall_check_joint`, communication/can_message_handlers.send_motor_commands); the
joint's other drives' commands are unchanged. Stop: the joint's limits go back to the robot XML's (RTI's start
reloads every drive's config anyway).

RTI calls start() / stop() from its `hall_check:1|0` command and tick() every tick while active — BEFORE its mode
management, so a pending mode change (homing, a jog, an error) stops the check and restores the limits before the new
mode's first command (homing sends its own limits on entry).
"""
from __future__ import annotations

import math

HALL_CHECK_TRIGGER = 3       # matches neither pin level: the drive stays in its speed loop and never latches
LIMIT_REPEATS = 2            # each limits frame sent twice (the CAN library swallows a dropped frame)
RESEND_TICKS = 125           # the zero limit re-sent every ~0.5 s at 250 Hz (a reconnect reload would restore the limit)
SETTLE_TICKS = 3             # the first replies carry the pin from the drive's LAST Hall session: ignore them
# 2 min at 250 Hz without a renewal (hall_check:1 again): the joint is held again (2026-10-24, as free_joint's — a
# client that died mid-check left J6 limp until a mode change).
TIMEOUT_TICKS = 30000


def hall_joint_index(has_hall_sensor, joint_num):
    """The first arm joint whose robot XML says <has_hall_sensor>1, or None."""
    for i in range(joint_num):
        try:
            if int(float(has_hall_sensor[i])) == 1:
                return i
        except (TypeError, ValueError, IndexError):
            return None
    return None


class HallCheck:
    """One joint limp + its Hall pin read every tick. `motors` are the CAN library's motor objects (Send_Limits,
    additional_pin2), `ctx` the dispatch context (hall_check_joint), `rti` the rti shared-memory dict."""

    def __init__(self, motors, joint, velocity_limit, current_limit, ctx, rti, joint_position, log=None):
        self.motors = motors
        self.joint = joint                      # None: this robot has no Hall-sensor joint
        self.velocity_limit = velocity_limit    # per joint, the robot XML's (what stop() restores)
        self.current_limit = current_limit
        self.ctx = ctx
        self.rti = rti
        self.joint_position = joint_position    # shared_data.joint['joint_position'] (rad)
        self.log = log
        self.active = False
        self._ticks = 0
        self._since_renewal = 0
        self._prev_pin = -1
        ctx.hall_check_joint = -1
        self._publish_off()

    # ------------------------------------------------------------------------------------------------ cold path
    def _send_limits(self, current):
        j = self.joint
        for _ in range(LIMIT_REPEATS):
            self.motors[j].Send_Limits(Velocity_limit=int(self.velocity_limit[j]), Current_limit=int(current))

    def _publish_off(self):
        self.rti["hall_check_joint"][0] = 0
        self.rti["hall_check_pin"][0] = -1
        self.rti["hall_check_changes"][0] = 0
        self.rti["hall_check_edge_deg"][:] = math.nan

    def start(self, idle, mode_pending, booting):
        """Start the check. Returns None, or why it was refused (nothing is sent then)."""
        if self.joint is None:
            return "this robot has no joint with <has_hall_sensor>1"
        if self.active:
            self._since_renewal = 0             # renewed: another TIMEOUT_TICKS
            return None
        if not idle or mode_pending:
            return "only in IDLE (enabled or not, homed or not)"
        if booting:
            return "RTI is still configuring the drives (the first ~1.2 s after its start)"
        self._send_limits(0)                    # BEFORE the first Hall command (this tick's dispatch)
        self.motors[self.joint].additional_pin2 = None
        self._ticks = 0
        self._since_renewal = 0
        self._prev_pin = -1
        self._publish_off()
        self.rti["hall_check_joint"][0] = self.joint + 1
        self.ctx.hall_check_joint = self.joint
        self.active = True
        if self.log:
            self.log(f"Hall check J{self.joint + 1} limp (current limit 0) — turn it by hand; its Hall pin is read "
                     f"every tick")
        return None

    def stop(self, reason="stopped"):
        if not self.active:
            return
        self.ctx.hall_check_joint = -1          # this tick's dispatch sends the joint its normal command again
        self._send_limits(self.current_limit[self.joint])
        self.active = False
        changes = int(self.rti["hall_check_changes"][0])
        self._publish_off()
        if self.log:
            self.log(f"Hall check J{self.joint + 1} stopped ({reason}); {changes} pin change(s) seen; its limits "
                     f"restored ({int(self.velocity_limit[self.joint])} / {int(self.current_limit[self.joint])} mA)")

    # ------------------------------------------------------------------------------------------------ per tick
    def tick(self, idle_and_no_mode_change):
        """Every tick while active, before RTI's mode management. No allocation on the normal path."""
        if not idle_and_no_mode_change:
            self.stop("the robot left IDLE or a mode change was requested")
            return
        self._ticks += 1
        self._since_renewal += 1
        if self._since_renewal >= TIMEOUT_TICKS:
            self.stop("2 minutes without a renewal")
            return
        if self._ticks % RESEND_TICKS == 0:
            self.motors[self.joint].Send_Limits(Velocity_limit=int(self.velocity_limit[self.joint]), Current_limit=0)
        if self._ticks <= SETTLE_TICKS:
            return
        pin = self.motors[self.joint].additional_pin2
        if pin is None:
            return
        pin = 1 if pin else 0
        if pin != self._prev_pin:
            if self._prev_pin != -1:
                self.rti["hall_check_changes"][0] += 1
                self.rti["hall_check_edge_deg"][pin] = math.degrees(self.joint_position[self.joint])
            self._prev_pin = pin
            self.rti["hall_check_pin"][0] = pin
