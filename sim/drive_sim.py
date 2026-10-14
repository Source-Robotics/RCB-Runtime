"""One simulated drive: it answers RTI's CAN frames the way the joint / gripper firmware does, byte for byte (read in
the joint firmware's communication_CAN.cpp, V106). The CAN library's own decoder is the contract:
dummy_test_scripts/test_sim_drives.py decodes every reply built here with Spectral_BLDC.UnpackData.

The model ("simple first"): a joint follows its command like an ideal motor — position mode moves toward the target
at up to the drive's velocity limit, speed mode at the commanded speed (clamped to it), current mode and idle hold
still (nothing pushes the joint: no gravity, no hand). A current limit of 0 holds it still in every mode (the Hall
sensor check relies on that). S2 (2026-10-13): a one-sided endstop on the homing side — the joint stops there and,
pushed into it, reports its current limit (homing's stall test: the limit is homing_current_ma, the threshold 70 % of
it); J6's magnet holds its Hall pin LOW within HALL_HALF_WIDTH ticks, with the firmware's edge / level detection and
latch (Hall_Indexing_mode). The drive's CAN watchdog is the firmware's: data packs,
Hall / gripper packs and polls restart its timer, IN_WATCHDOG_TIMEOUT only sets the time, a clear removes a trip but
does not restart the timer, and a tripped (or E-stopped) drive goes idle.

S6 (2026-10-13) — DISTURBANCES, each set by the control channel's "physics" command until a time: what a real arm does
that the ideal joint does not, so the safety monitor's checks can trip in simulation: a hand turning a limp joint
(push), the joint moving on its own whatever it is told (runaway), a drive following its position target with a lag
(lag), and the measured current swinging while the joint holds (buzz). 2026-10-14: an obstacle (block) — a powered
joint cannot pass the position it was at when the block began, on one side (the endstop's unless told), and pushed
into it reports its current limit, exactly as at an endstop: a load the homing current cannot lift, or something in
the way — homing sees a stall where there is no endstop, and both passes repeat it.
"""
from __future__ import annotations

import math
import struct

# Command ids (the CAN library's / the firmware's numbers)
IN_ESTOP, IN_CLEAR, IN_DATA_PACK_1, OUT_DATA_PACK_1, IN_DATA_PACK_PD = 0, 1, 2, 3, 4
OUT_IN_PING, IN_CAN_ID, IN_IDLE, IN_SAVE, IN_RESET, IN_WATCHDOG = 10, 11, 12, 13, 14, 15
IN_KP_KD, IN_KIIQ_KPIQ, IN_KPV_KIV, IN_KPP, IN_LIMITS, IN_IQ, IN_KT = 16, 17, 18, 19, 20, 21, 22
OUT_IN_TEMPERATURE, OUT_IN_VOLTAGE, OUT_IN_DEVICE_INFO, OUT_IN_ERRORS, OUT_IN_IQ, OUT_IN_ENCODER = 23, 24, 25, 26, 27, 28
IN_HALL, OUT_HALL, OUT_IN_KT, IN_VLIM = 31, 32, 33, 34
OUT_GRIPPER, IN_GRIPPER, IN_GRIPPER_CALIB = 60, 61, 62

FAULTS = ("temperature", "encoder", "vbus", "driver", "velocity", "current")   # the errors poll's flags, S4

POLLS = (OUT_IN_PING, OUT_IN_TEMPERATURE, OUT_IN_VOLTAGE, OUT_IN_DEVICE_INFO, OUT_IN_ERRORS, OUT_IN_IQ,
         OUT_IN_ENCODER, OUT_IN_KT)
CONFIG = (IN_KP_KD, IN_KIIQ_KPIQ, IN_KPV_KIV, IN_KPP, IN_IQ, IN_KT, IN_VLIM)

# Controller modes (the firmware's Controller_mode)
IDLE, POSITION, SPEED, CURRENT, PD, HALL = 0, 1, 2, 3, 4, 8

SPEED_TAU_S = 0.004             # the reported speed's low-pass (the joint jumps to each 4 ms target in one step)
GRIPPER_COUNTS_PER_S = 300.0     # the simulated jaws: the full 0-255 stroke in under a second


def can_id(node, cmd, err=0):
    """The 11-bit id: node (4 bits) | command (6 bits) | error bit."""
    return ((node & 0xF) << 7) | ((cmd & 0x3F) << 1) | (1 if err else 0)


def split_id(arb):
    return (arb >> 7) & 0xF, (arb >> 1) & 0x3F, arb & 1


def _bits(flags):
    """The firmware's bitsToByte: the FIRST flag is bit 7 (the library's split_2_bitfield reads it back so)."""
    v = 0
    for f in flags:
        v = (v << 1) | (1 if f else 0)
    return v


def _s(value, n):
    """A signed int as n big-endian bytes (two's complement, wrapped like the firmware's shifts)."""
    return (int(value) & ((1 << (8 * n)) - 1)).to_bytes(n, "big")


def _from(b):
    """Signed big-endian bytes -> int."""
    return int.from_bytes(b, "big", signed=True)


HALL_HALF_WIDTH = 1000         # ticks: the magnet holds the Hall pin LOW within this of its centre (the homing net's)
FREE_MA = 40                   # mA a free joint draws moving in speed mode (below every homing stall threshold)


class SimDrive:
    def __init__(self, node, gripper=False, kt=0.1, position=0.0, temperature=38, vbus_mv=23500, sw_version=103,
                 hw_version=1, batch=1, serial=1, stop=None, stop_dir=1, hall=None):
        """stop: the endstop's position (ticks) on the `stop_dir` (+1 / -1) side — homing pushes into it; the joint is
        free the other way (S2: one-sided, as the homing net's arm). hall: the magnet's centre (ticks), J6."""
        self.node = node
        self.gripper = gripper
        self.stop = None if stop is None else float(stop)
        self.stop_dir = 1 if stop_dir >= 0 else -1
        self.hall = None if hall is None else float(hall)
        self.pushing = False
        self._hall_prev = None                  # the firmware's additional2_var_prev (edge mode)
        self._hall_needs_init = True
        self.kt = float(kt)
        self.position = float(position)        # motor ticks
        self.speed = 0.0                        # ticks / s, measured
        self.current = 0.0                      # mA, measured
        self.mode = IDLE
        self.target = self.position
        self.target_speed = 0.0
        self.ff_current = 0.0
        self.iq_setpoint = 0.0
        self.velocity_limit = 1.0e9             # until RTI's config arrives
        self.current_limit = 1.0e9
        self.config = {}                        # the gains etc., as received (raw bytes per command id)
        self.temperature = temperature
        self.vbus_mv = vbus_mv
        self.info = (hw_version, batch, sw_version, serial)
        self.estop_error = False
        self.watchdog_error = False
        self.fault = None                       # S4: a lasting drive fault (FAULTS) — a CAN clear does not remove it
        self.last_cmd_ms = None                 # S4: the last frame that restarted the watchdog (data / Hall / gripper / poll)
        self.trip_ms = None                     # S4: when the watchdog last tripped from a clear state
        self.wd_time_ms = 0                     # 0 = off (the firmware's EEPROM default)
        self.wd_action = 0
        self._wd_last = 0
        self._wd_reset = True                   # common.h: watchdog_reset = 1 at power-up
        self.hall_pin = 1                       # HIGH = no magnet (S2: a magnet window)
        self.hall_trigger = 1
        self.hall_index = 0
        self.hall_trigger_value = 0
        self.g_position = 0                     # gripper: 0-255
        self._g_pos = 0.0
        self.g_target = 0
        self.g_activated = 0
        self.g_action = 0
        self.g_calibrated = 1
        self.g_object = 0
        self.frames = 0
        # S6 disturbances, each until its *_until_ms (the control channel's "physics" command)
        self.push_speed, self.push_until_ms = 0.0, -1          # ticks/s: a hand turning a LIMP joint (idle / current)
        self.runaway_speed, self.runaway_until_ms = 0.0, -1    # ticks/s: the joint moving on its own (a powered drive)
        self.lag_tau, self.lag_until_ms = 0.0, -1              # s: position / PD mode follows its target with this lag
        self.buzz_ma, self.buzz_hz, self.buzz_until_ms = 0.0, 0.0, -1   # the measured current swinging
        self.block_at, self.block_dir, self.block_until_ms = 0.0, self.stop_dir, -1   # an obstacle: block_at, one side
        self.push_dir = self.stop_dir                          # the side a saturated current points to

    @property
    def error(self):
        return self.estop_error or self.watchdog_error or self.fault is not None

    def reboot(self):
        """S4: the drive lost power and came back: its RAM config is gone (the watchdog off — the EEPROM default —, no
        limits from RTI, the gains), its errors cleared, idle. The position is kept (S4 does not model the encoder)."""
        self.mode = IDLE
        self.velocity_limit = self.current_limit = 1.0e9
        self.config = {}
        self.wd_time_ms, self.wd_action, self._wd_reset = 0, 0, True
        self.estop_error = self.watchdog_error = False
        self.fault = None
        self.trip_ms = None

    # ------------------------------------------------------------------------------------------------ the watchdog
    def _watchdog(self, ms):
        """communication_CAN.cpp:892-907, one CAN_watchdog(ms) call."""
        if self.wd_time_ms != 0:
            if self._wd_reset:
                self._wd_last = ms
                self._wd_reset = False
            if ms - self._wd_last >= self.wd_time_ms:
                if not self.watchdog_error:
                    self.trip_ms = ms
                self.watchdog_error = True
                self._wd_last = ms

    # ------------------------------------------------------------------------------------------------ replies
    def _reply(self, cmd, data=b""):
        return (can_id(self.node, cmd, self.error), bytes(data))

    def _data_pack_reply(self):
        return self._reply(OUT_DATA_PACK_1, _s(round(self.position), 3) + _s(round(self.speed), 3)
                           + _s(round(self.current), 2))

    def _poll_reply(self, cmd):
        if cmd == OUT_IN_PING:
            return self._reply(OUT_IN_PING)
        if cmd == OUT_IN_TEMPERATURE:
            return self._reply(cmd, _s(self.temperature, 2))
        if cmd == OUT_IN_VOLTAGE:
            return self._reply(cmd, _s(self.vbus_mv, 2))
        if cmd == OUT_IN_DEVICE_INFO:
            hw, batch, sw, serial = self.info
            return self._reply(cmd, bytes([hw & 0xFF, batch & 0xFF, sw & 0xFF]) + _s(serial, 4))
        if cmd == OUT_IN_ERRORS:
            f = self.fault
            return self._reply(cmd, bytes([
                _bits([self.error, f == "temperature", f == "encoder", f == "vbus", f == "driver", f == "velocity",
                       f == "current", self.estop_error]),
                _bits([1, 1, self.watchdog_error, 0, 0, 0, 0, 0])]))        # calibrated, activated, watchdog
        if cmd == OUT_IN_IQ:
            return self._reply(cmd, _s(round(self.current), 2))
        if cmd == OUT_IN_ENCODER:
            return self._reply(cmd, _s(round(self.position), 4) + _s(round(self.speed), 4))
        return self._reply(OUT_IN_KT, struct.pack(">f", self.kt))

    def _hall_reply(self):
        return self._reply(OUT_HALL, _s(round(self.position), 3)
                           + bytes([_bits([self.hall_trigger, self.hall_pin, self.hall_index, 0, 0, 0, 0, 0])]))

    def _gripper_reply(self):
        obj = max(0, min(3, int(self.g_object)))
        return self._reply(OUT_GRIPPER, bytes([self.g_position & 0xFF]) + _s(round(self.current), 2) + bytes([_bits([
            self.g_activated, self.g_action, obj & 1, (obj >> 1) & 1, 0, 0, self.estop_error, self.g_calibrated])]))

    # ------------------------------------------------------------------------------------------------ a frame
    def handle(self, cmd, data, remote, ms):
        """One frame from RTI. Returns the replies [(arbitration id, data)], in the firmware's order."""
        self.frames += 1
        data = bytes(data)
        out = []
        resets = False
        if remote:
            if cmd in POLLS:
                self._wd_reset = resets = True
                out.append(self._poll_reply(cmd))
        elif cmd == IN_DATA_PACK_1:
            if len(data) == 8:
                self.target, self.target_speed = float(_from(data[0:3])), float(_from(data[3:6]))
                self.ff_current = float(_from(data[6:8]))
                self.mode = POSITION
            elif len(data) == 5:
                self.target_speed, self.ff_current = float(_from(data[0:3])), float(_from(data[3:5]))
                self.mode = SPEED
            elif len(data) == 2:
                self.iq_setpoint = float(_from(data[0:2]))
                self.mode = CURRENT
            self._wd_reset = resets = True
            out.append(self._data_pack_reply())
        elif cmd == IN_DATA_PACK_PD:
            if len(data) == 8:
                self.target, self.target_speed = float(_from(data[0:3])), float(_from(data[3:6]))
                self.ff_current = float(_from(data[6:8]))
                self.mode = PD
            self._wd_reset = resets = True
            out.append(self._data_pack_reply())
        elif cmd == IN_HALL:
            if len(data) == 4:
                self.target_speed = float(_from(data[0:3]))
                self.hall_trigger_value = data[3]
                if self.mode != HALL:                   # re-armed only when entering the mode
                    self.hall_trigger, self.hall_index = 1, 0
                    self._hall_needs_init = True
                self.mode = HALL
                self._wd_reset = resets = True
            out.append(self._hall_reply())
        elif cmd == IN_GRIPPER:
            if len(data) == 5:
                self.g_target = data[0]
                flags = [(data[4] >> i) & 1 for i in range(7, -1, -1)]     # activate, action, estop, release_dir
                self.g_activated, self.g_action = flags[0], flags[1]
            self._wd_reset = resets = True
            out.append(self._gripper_reply())
        elif cmd == IN_GRIPPER_CALIB:
            self.g_calibrated = 1
        elif cmd == IN_WATCHDOG and len(data) >= 5:
            self.wd_time_ms = _from(data[0:4])
            self.wd_action = data[4]
        elif cmd == IN_LIMITS and len(data) == 8:
            self.velocity_limit = struct.unpack(">f", data[0:4])[0]
            self.current_limit = struct.unpack(">f", data[4:8])[0]
        elif cmd == IN_CLEAR:
            self.estop_error = self.watchdog_error = False
            self.mode = IDLE
        elif cmd == IN_ESTOP:
            self.estop_error = True
        elif cmd == IN_IDLE:
            self.mode = IDLE
        elif cmd in CONFIG:
            self.config[cmd] = data
        if resets:
            self.last_cmd_ms = ms
        self._watchdog(ms)
        return out

    # ------------------------------------------------------------------------------------------------ the model
    def _hall_pin(self):
        return 0 if self.hall is not None and abs(self.position - self.hall) <= HALL_HALF_WIDTH else 1

    def _hall_mode(self):
        """motor_control.cpp Hall_Indexing_mode(): the pin read, then edge (trigger 2) or level (0 / 1) detection; a
        detection latches the position (the drive then holds it) — any other trigger value (3: the Hall sensor check)
        never matches, so the speed loop runs on. Returns the mode the loop runs this step: SPEED or POSITION."""
        self.hall_pin = self._hall_pin()
        if self.hall_trigger_value == 2:
            if self._hall_needs_init:
                self._hall_prev = self.hall_pin
                self._hall_needs_init = False
            elif self.hall_trigger == 1:
                if self.hall_pin != self._hall_prev:
                    self.target = self.position
                    self.hall_trigger, self.hall_index = 0, 1
                self._hall_prev = self.hall_pin
            return POSITION if self.hall_trigger == 0 else SPEED
        if self.hall_pin == self.hall_trigger_value:
            if self.hall_trigger == 1:
                self.target = self.position
                self.hall_trigger, self.hall_index = 0, 1
            return POSITION
        self.hall_trigger = 1
        return SPEED

    def _clamp_stop(self, x):
        """The endstop: the joint cannot pass it on its side. Returns (position, pushed against it)."""
        if self.stop is None:
            return x, False
        if self.stop_dir > 0 and x > self.stop:
            return self.stop, True
        if self.stop_dir < 0 and x < self.stop:
            return self.stop, True
        return x, False

    def _powered_position(self, run, vmax, dt, ms):
        """Where a POWERED joint goes in dt: its runaway (S6), else its mode's motion toward its command."""
        if ms < self.runaway_until_ms:                   # S6: moving on its own — its command is ignored
            return self.position + self.runaway_speed * dt
        if run in (POSITION, PD):
            to_go = self.target - self.position
            if ms < self.lag_until_ms and self.lag_tau > 0:              # S6: a lagging drive
                to_go *= dt / (self.lag_tau + dt)
            return self.position + max(-vmax * dt, min(vmax * dt, to_go))
        if run == SPEED:
            return self.position + max(-vmax, min(vmax, self.target_speed)) * dt
        return self.position

    def _measured_current(self, run, ms):
        """The current the drive reports (mA): its mode's, saturated on an endstop, plus a buzz (S6)."""
        if self.error or self.mode == IDLE:
            cur = 0.0
        elif self.pushing:                               # leaning on the endstop / the obstacle: at its limit
            cur = self.push_dir * self.current_limit
        elif self.mode == CURRENT:
            cur = max(-self.current_limit, min(self.current_limit, self.iq_setpoint))
        elif run == SPEED and self.target_speed != 0:
            cur = max(-self.current_limit, min(self.current_limit,
                                               self.ff_current + (FREE_MA if self.target_speed > 0 else -FREE_MA)))
        else:
            cur = max(-self.current_limit, min(self.current_limit, self.ff_current))
        if ms < self.buzz_until_ms and self.buzz_ma and not self.error:   # S6: a buzzing hold (the drive's Iq swings)
            cur += self.buzz_ma * math.sin(2.0 * math.pi * self.buzz_hz * ms / 1000.0)
        return cur

    def step(self, dt, ms):
        """Advance the joint by dt seconds (and run the watchdog at ms)."""
        self._watchdog(ms)
        prev = self.position
        vmax = abs(self.velocity_limit)
        limp = self.error or self.current_limit == 0 or self.mode in (IDLE, CURRENT)
        run = self.mode
        if self.mode == HALL:
            run = self._hall_mode()
        self.pushing = False
        self.push_dir = self.stop_dir
        if not limp and dt > 0:
            self.position, self.pushing = self._clamp_stop(self._powered_position(run, vmax, dt, ms))
            if ms < self.block_until_ms and (self.position - self.block_at) * self.block_dir > 0:   # S6: the obstacle
                self.position, self.pushing, self.push_dir = self.block_at, True, self.block_dir
        if dt > 0 and limp and not self.error and ms < self.push_until_ms:     # S6: a hand on a limp joint
            self.position, _ = self._clamp_stop(self.position + self.push_speed * dt)
        if dt > 0:                                       # the firmware reports a filtered speed (Velocity_Filter)
            self.speed += ((self.position - prev) / dt - self.speed) * (dt / (SPEED_TAU_S + dt))
        self.current = self._measured_current(run, ms)
        if self.gripper and self.g_activated and self.g_action and not self.error and dt > 0:
            d = self.g_target - self._g_pos
            self._g_pos += max(-GRIPPER_COUNTS_PER_S * dt, min(GRIPPER_COUNTS_PER_S * dt, d))
            self.g_position = int(round(self._g_pos))
