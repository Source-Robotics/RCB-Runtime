#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: error_checks.py
Author: Petar Crnjak
Company: Source Robotics
Description:
    RTI error checking — called once per loop tick.

    Uses latched error tracking: once an error appears it stays in the set until
    the user explicitly calls clear_errors(). Logging is edge-triggered on
    appearance only — no log spam while the error persists.

    Usage in the RTI superloop:

        # Before the loop:
        latched_errors = set()

        # Inside the loop — clear if requested, then check:
        if shared_data.clear_error_signal.get() == "1":
            latched_errors = clear_errors(latched_errors)
            shared_data.clear_error_signal.set("0")
        error_active, latched_errors = check_errors(
            shared_data, joint_num, latched_errors, estop_active=check_estop_active()
        )
        shared_data.rti["error_active"][0] = 1.0 if error_active else 0.0

    Architecture — two separate safety layers
    -----------------------------------------
    Layer               Handles                                    Behaviour                          Recovery
    ----------------    ---------------------------------------    --------------------------------   ----------------------------------
    safety_checks.py    Joint limits, singularity, velocity,       Blocks/scales motion this tick,    Auto-resolves when you move away.
                        collision detection.                        does NOT latch.
    error_checks.py     Motor faults (watchdog, temperature,       Latches until user explicitly       Requires CAN clear command,
                        encoder, ...), CAN lost, ESTOP.            clears via GUI or ZMQ.              settle period, then user action.

    Rule of thumb: if the robot motors are healthy and the condition goes away
    by itself when you move — it belongs in safety_checks.py.  If the motor has
    entered a fault state and needs a CAN clear command to recover — it belongs
    here in error_checks.py.
"""

import logging
import time

from utility.state_machine import RobotMode, MAINTENANCE_MODES
from communication.can_message_handlers import write_motor_config_to_shared
from utility.activity_log import get_rti_activity_logger, log_activity, RateLimiter


# Debounce guard for activity-log emission of hardware faults — see
# RateLimiter's docstring / activity_log_design.md §4.
_activity_rate_limiter = RateLimiter()
_ACTIVITY_MIN_INTERVAL = 1.0  # seconds


# ---------------------------------------------------------------------------
# Tuning constants — shared by the RTI superloop and process_errors_tick().
# ---------------------------------------------------------------------------
# The settle must outlast the round-robin error poll, or the latch gets wiped before the
# evidence it exists to wait for can possibly arrive. handle_polling_and_overrides() reaches a
# given node's STATE_OF_ERRORS once per (n_nodes * 3) ticks — 21 ticks on a 6-joint arm plus
# gripper — plus up to 7 more when the device-info sweep preempts telemetry, plus ~2 ticks for
# the node to act on the clear: 30 ticks worst case. At the old value of 10, roughly half the
# nodes had not been re-polled by wipe time, so check_errors() re-latched their pre-clear
# readings. That was the "Clear Errors needs two presses" bug, and why the affected joints
# looked arbitrary — it depended on where the round-robin happened to be standing.
#
# NOT a guarantee: an active override queue skips polling for an unbounded number of ticks.
# The robust fix is to gate on evidence (deque entry["tick"] > the tick the clear was sent)
# instead of on elapsed time — see currently_doing.md.
CLEAR_ERROR_SETTLE_TICKS = 38   # ticks (~152 ms @ 4 ms) to wait after a CAN clear before wiping the latch
CLEAR_ERROR_SEND_REPEATS = 3    # how many times Send_Clear_Error() is sent per motor

# RT loop degradation bands — FRACTIONS of delta_t, never absolute times. delta_t is
# per-robot (PAR6 4 ms / 250 Hz; INDIGO and SKAR 10 ms / 100 Hz), so an absolute band
# would read a perfectly healthy 100 Hz robot as catastrophically slow and disable it
# on boot.
#
# Sized off measured PAR6 behaviour (2026-07-17, clean post-boot stats): p95 4.020 ms,
# p99 4.025 ms — 99% of ticks land within 0.64% of nominal. 1.05 is ~8x that deviation:
# wide enough that normal operation never trips it, tight enough to catch real trouble.
# The band is one-sided by construction: RTI sleeps delta_t - exec (a relative sleep with
# no catch-up), so the loop can overshoot its period but can never run FAST.
LOOP_WARN_PERIOD_FACTOR  = 1.05   # p99 > +5% of nominal  -> warning (4.20 ms / 238 Hz on PAR6)
LOOP_ERROR_PERIOD_FACTOR = 1.10   # p99 > +10% of nominal -> error   (4.40 ms / 227 Hz on PAR6)
LOOP_ERROR_SUSTAIN_S     = 1.0    # ...and held that long before the error latches

# Keys (or key suffixes) that are warnings only — they do not set error_active.
_WARNING_SUFFIXES = ("CAN_STALE", "HOMING_FAILED", "NOT_HOMED", "LOOP_DEGRADED")


# ---------------------------------------------------------------------------
# Cause/fix descriptions for every possible error key, also the RTI-hardware
# slice of the activity-log catalog (see utility/activity_log.py). The key is
# what goes into the set; "cause" is what gets printed by check_errors()/
# process_errors_tick(), "fix" is only surfaced in the activity log.
# ---------------------------------------------------------------------------
ERROR_DESCRIPTIONS = {
    # E-stop
    "ESTOP":                    {"cause": "E-STOP is active — robot is halted.", "fix": "release the E-STOP, then clear errors to resume."},
    "SW_ESTOP":                 {"cause": "Software E-STOP is active — robot is halted.", "fix": "clear the software E-STOP, then clear errors to resume."},
    "EXEC_LINK_LOST":           {"cause": "command_executor went silent during EXEC motion — the control link is lost.", "fix": "restart command_executor, then clear errors to resume."},
    "RTI_LINK_LOST":            {"cause": "the external PC stopped sending setpoints while it held RTI control — the robot stopped itself.", "fix": "check the network and the controlling PC, then clear errors and re-claim control."},

    # RT loop health (see LoopDegradationMonitor)
    "LOOP_DEGRADED":            {"cause": "RT loop is running slower than its configured rate.", "fix": "check CPU load, thermal throttling, or a silent CAN node; clears itself once the loop recovers."},
    "LOOP_CRITICAL":            {"cause": "RT loop has been critically slow for over a second — real-time guarantees are not being met.", "fix": "check CPU load/thermals and the timing stats in the GUI's RT Diagnostics tab, then clear errors to resume."},

    # Per-joint motor errors  (formatted as "J{i}:KEY" at runtime)
    "TEMPERATURE":              {"cause": "motor is overheating.", "fix": "let the motor cool down before re-enabling."},
    "ENCODER":                  {"cause": "encoder error — position signal is lost or corrupted.", "fix": "check the encoder cable/connector on this joint."},
    "VBUS":                     {"cause": "bus voltage error — supply voltage is out of range.", "fix": "check the power supply and bus wiring."},
    "DRIVER":                   {"cause": "motor driver fault — internal hardware error.", "fix": "power-cycle the joint driver; replace if the fault recurs."},
    "VELOCITY":                 {"cause": "velocity error — motor exceeded its velocity limit.", "fix": "check for an unexpected collision/obstruction, then re-enable."},
    "CURRENT":                  {"cause": "current error — overcurrent detected.", "fix": "check for a mechanical jam on this joint before re-enabling."},
    "ESTOP_MOTOR":              {"cause": "e-stop error reported by the motor driver.", "fix": "check the motor driver's own e-stop input wiring."},
    "WATCHDOG":                 {"cause": "watchdog error — motor did not receive a command in time.", "fix": "check CAN wiring/power to the joint; re-enable once the fault clears."},

    # Per-joint CAN communication errors
    "CAN_LOST":                 {"cause": "CAN connection lost — no data received for too long.", "fix": "check CAN bus wiring/termination and joint power."},
    "CAN_STALE":                {"cause": "CAN data is getting stale — motor may be slow to respond.", "fix": "check CAN bus load/wiring; usually resolves on its own."},

    # Homing
    "NOT_HOMED":                {"cause": "robot is not homed — home the robot before operating.", "fix": "run the home command."},
    "HOMING_FAILED":            {"cause": "homing failed — could not find the endstop.", "fix": "check the endstop/limit switch wiring for this joint, then retry homing."},

    # Gripper errors
    "GRIPPER_DISCONNECTED":     {"cause": "gripper is not connected — no CAN reply from the gripper.", "fix": "check the gripper's CAN wiring/power."},
    "GRIPPER_TIMEOUT":          {"cause": "gripper timeout error — gripper did not respond in time.", "fix": "check gripper CAN wiring/power."},
    "GRIPPER_ESTOP":            {"cause": "gripper e-stop error — gripper received an e-stop signal.", "fix": "clear the gripper's e-stop condition."},
    "GRIPPER_ERROR":            {"cause": "gripper reported a general fault.", "fix": "check logs/high_level.log for detail; power-cycle the gripper if it recurs."},

    # Gripper motor-level errors (same faults as arm motors, reported by the gripper's motor driver)
    "GRIPPER_MOTOR_TEMPERATURE": {"cause": "gripper motor is overheating.", "fix": "let the gripper motor cool down before re-enabling."},
    "GRIPPER_MOTOR_ENCODER":     {"cause": "gripper motor encoder error — position signal is lost or corrupted.", "fix": "check the gripper's encoder cable/connector."},
    "GRIPPER_MOTOR_VBUS":        {"cause": "gripper motor bus voltage error — supply voltage is out of range.", "fix": "check the power supply and bus wiring to the gripper."},
    "GRIPPER_MOTOR_DRIVER":      {"cause": "gripper motor driver fault — internal hardware error.", "fix": "power-cycle the gripper driver; replace if the fault recurs."},
    "GRIPPER_MOTOR_VELOCITY":    {"cause": "gripper motor velocity error — exceeded velocity limit.", "fix": "check for an obstruction in the gripper before re-enabling."},
    "GRIPPER_MOTOR_CURRENT":     {"cause": "gripper motor current error — overcurrent detected.", "fix": "check for a mechanical jam in the gripper before re-enabling."},
    "GRIPPER_MOTOR_ESTOP":       {"cause": "gripper motor e-stop error reported by the motor driver.", "fix": "check the gripper motor driver's own e-stop input wiring."},
    "GRIPPER_MOTOR_WATCHDOG":    {"cause": "gripper motor watchdog error — motor did not receive a command in time.", "fix": "check CAN wiring/power to the gripper; re-enable once the fault clears."},
}


def _cause_text(short_key: str, fallback: str) -> str:
    """Cause string for log text — short_keys not in the catalog fall back to the raw key."""
    entry = ERROR_DESCRIPTIONS.get(short_key)
    return entry["cause"] if entry else fallback


# ---------------------------------------------------------------------------
# Motor error field names — single source of truth used by both the deque
# zeroing (Motor_errors_data entries) and the shared-memory zeroing.
# ---------------------------------------------------------------------------
_MOTOR_ERROR_DEQUE_KEYS = (
    "temperature_error", "encoder_error", "vbus_error",  "driver_error",
    "velocity_error",    "current_error", "estop_error", "watchdog_error",
)
# Corresponding shared_data.motor keys (same names with "motor_" prefix).
_MOTOR_ERROR_SHARED_KEYS = tuple(f"motor_{k}" for k in _MOTOR_ERROR_DEQUE_KEYS)

# Gripper error fields in Gripper_data deques → shared_data.gripper keys.
_GRIPPER_ERROR_DEQUE_KEYS  = ("g_tim_err", "g_estop_err", "g_err")
_GRIPPER_ERROR_SHARED_KEYS = ("gripper_timeout_error", "gripper_estop_error", "gripper_error")

# Gripper motor-level error fields in shared_data.gripper (same faults as arm motors).
# Note: temp and encoder use shortened names (temp_error, enc_error) in shared memory.
_GRIPPER_MOTOR_ERROR_SHARED_KEYS = (
    "gripper_motor_temp_error",     "gripper_motor_enc_error",
    "gripper_motor_vbus_error",     "gripper_motor_driver_error",
    "gripper_motor_velocity_error", "gripper_motor_current_error",
    "gripper_motor_estop_error",    "gripper_motor_watchdog_error",
)


def _joint_key(joint_index: int, short_key: str) -> str:
    """Build a unique error key for a per-joint error, e.g. 'J2:TEMPERATURE'."""
    return f"J{joint_index}:{short_key}"


def send_clear_errors(
    shared_data,
    Motor,
    Motor_errors_data,
    joint_num: int,
    send_repeats: int = 1,
    Gripper_data=None,
    active_CAN_gripper: int = 0,
):
    """
    Send CAN clear-error commands to motors with active errors and zero
    stale deque data so the next update_motor_and_gripper_data tick reads 0
    instead of pre-clear poll values.

    When a CAN gripper is present the same clear command is sent to the gripper
    motor (Motor[joint_num]) and its Gripper_data deque error entries are zeroed.

    Fresh responses from the hardware will overwrite the deque entries as they
    arrive over the next few poll cycles.

    Args:
        shared_data:        Shared memory module.
        Motor:              List of SpectralCAN motor objects (gripper at [joint_num]).
        Motor_errors_data:  Per-motor deques of error-poll dicts.
        joint_num:          Number of joints.
        send_repeats:       How many times to send Send_Clear_Error() per motor.
        Gripper_data:       Per-node deques of gripper CAN dicts (pass when gripper present).
        active_CAN_gripper: 1 if a CAN gripper is configured, 0 if not.
    """
    for i in range(joint_num):
        if shared_data.motor['global_motor_error'][i] != 0:
            for _ in range(send_repeats):
                Motor[i].Send_Clear_Error()
        for entry in Motor_errors_data[i]:
            for k in _MOTOR_ERROR_DEQUE_KEYS:
                entry[k] = 0

    if active_CAN_gripper and Gripper_data is not None:
        for _ in range(send_repeats):
            Motor[joint_num].Send_Clear_Error()
        for entry in Gripper_data[joint_num]:
            for k in _GRIPPER_ERROR_DEQUE_KEYS:
                entry[k] = 0
        # Also zero motor-level error deque entries for the gripper motor
        if joint_num < len(Motor_errors_data):
            for entry in Motor_errors_data[joint_num]:
                for k in _MOTOR_ERROR_DEQUE_KEYS:
                    entry[k] = 0


def tick_clear_countdown(
    countdown: int,
    latched_errors: set,
    shared_data,
    active_CAN_gripper: int = 0,
) -> tuple:
    """
    Decrement the clear-error settle countdown.  When it reaches zero, wipe
    the error latch and zero shared motor and gripper error flags so
    check_errors() gets a clean read on the same tick.

    Any error that is still genuinely present in the hardware will be written
    back by the next poll cycle and re-latched by check_errors() automatically.

    Args:
        countdown:          Current countdown value (ticks remaining).
        latched_errors:     The current latched error set.
        shared_data:        Shared memory module.
        active_CAN_gripper: 1 if a CAN gripper is configured, 0 if not.

    Returns:
        (new_countdown, new_latched_errors)
    """
    if countdown <= 0:
        return countdown, latched_errors

    countdown -= 1
    if countdown == 0:
        latched_errors = clear_errors(latched_errors, shared_data)
        for k in _MOTOR_ERROR_SHARED_KEYS:
            shared_data.motor[k][:] = 0
        if active_CAN_gripper:
            for k in _GRIPPER_ERROR_SHARED_KEYS:
                shared_data.gripper[k][:] = 0
            for k in _GRIPPER_MOTOR_ERROR_SHARED_KEYS:
                shared_data.gripper[k][:] = 0
        logging.info(
            "[ERRORS] Settle period complete — latch and error flags cleared. "
            "check_errors() will re-latch any still-active faults on the next poll cycle."
        )
    return countdown, latched_errors


def clear_errors(latched_errors: set, shared_data=None) -> set:
    """
    Wipe the error latch. Called when the user presses "Clear Errors".

    Also zeros CAN_node_connection_error in shared memory so that motors/gripper
    that have since recovered are not immediately re-latched. Any nodes that are
    still actually silent will re-set their CAN_node_connection_error within
    ~50 ticks and re-latch the error automatically.
    """
    latched_errors.clear()
    if shared_data is not None:
        shared_data.rti["CAN_node_connection_error"][:] = 0
    logging.info("[ERRORS] Error latch cleared by user. Active errors will re-latch immediately.")
    return latched_errors


class ExecLinkWatchdog:
    """Trips when command_executor's heartbeat stops advancing during active EXEC motion.

    Stateful — it remembers when the heartbeat last changed, so it lives across RTI ticks
    (instantiate once, call update() each tick). The heartbeat is a monotonic counter written by
    a dedicated executor thread, so a long TOPPRA solve keeps it advancing; it only stalls if the
    executor process dies/wedges. Gated on EXEC + buffered samples so a dead/idle executor with
    nothing moving is never tripped. See gui_executor_migration_plan.md Phase 5.
    """

    def __init__(self, timeout_ticks: int):
        self._timeout_ticks = timeout_ticks
        self._last_hb = None
        self._last_change_tick = 0

    def update(self, heartbeat: float, tick: int, in_exec: bool, samples_remaining: float) -> bool:
        if heartbeat != self._last_hb:
            self._last_hb = heartbeat
            self._last_change_tick = tick
        stale = (tick - self._last_change_tick) > self._timeout_ticks
        return stale and in_exec and samples_remaining > 0.0


class LoopDegradationMonitor:
    """Trips when the RT loop's p99 period stays above nominal for too long.

    Stateful — it counts how long p99 has been out of band, so it lives across RTI ticks
    (instantiate once, call update() each tick).

    Reads p99 rather than the instantaneous period on purpose. p99 is a rolling ~500-tick
    window, so one ugly tick cannot trip it (a single spike is the 99.8th percentile of the
    window and never moves the 99th), while a genuine sustained degradation moves it within
    ~2 s. That is exactly the difference between "a log write took a while" and "this loop
    is not meeting its deadline any more".

    warmup_ticks gates it until the rolling window holds only post-boot samples. Without it
    the boot transients the stats are explicitly wiped for (RTI's tick 8 selfcheck, the
    50/150/300 reconfig) would trip this on every single start.

    Both bands are fractions of delta_t, never absolute — see the constants above.
    """

    def __init__(self, delta_t: float, warmup_ticks: int,
                 warn_factor: float = LOOP_WARN_PERIOD_FACTOR,
                 error_factor: float = LOOP_ERROR_PERIOD_FACTOR,
                 sustain_s: float = LOOP_ERROR_SUSTAIN_S):
        self._warn_period   = delta_t * warn_factor
        self._error_period  = delta_t * error_factor
        self._sustain_ticks = max(1, int(round(sustain_s / delta_t))) if delta_t > 0 else 1
        self._warmup_ticks  = warmup_ticks
        self._over_since    = None

    def update(self, p99_period: float, tick: int) -> tuple:
        """Returns (degraded, critical): the warning flag and the latching-error flag."""
        if tick <= self._warmup_ticks:
            self._over_since = None
            return False, False

        degraded = p99_period > self._warn_period

        if p99_period > self._error_period:
            if self._over_since is None:
                self._over_since = tick
            critical = (tick - self._over_since) >= self._sustain_ticks
        else:
            self._over_since = None
            critical = False

        return degraded, critical


def check_errors(
    shared_data,
    joint_num: int,
    latched_errors: set,
    estop_active: bool,
    active_CAN_gripper: int = 0,
    exec_link_lost: bool = False,
    rti_link_lost: bool = False,
    loop_degraded: bool = False,
    loop_critical: bool = False,
) -> tuple:
    """
    Inspect all error sources and update the latched error set.

    Logs exactly once when an error first appears. Errors stay latched until
    clear_errors() is called — they do NOT auto-remove when the condition clears.

    Args:
        shared_data:        The shared memory object (from data.shared_data).
        joint_num:          Number of joints on the robot.
        latched_errors:     The set returned by the previous call (pass set() on the first call).
        estop_active:       Result of check_estop_active() — passed in so this module
                            stays independent of the GPIO import.
        active_CAN_gripper: 1 if a CAN gripper is configured, 0 if not.

    Returns:
        (error_active: bool, latched_errors: set)
        Pass latched_errors back on the next call.
    """

    current_errors = set()

    # ------------------------------------------------------------------
    # E-STOP
    # ------------------------------------------------------------------
    if estop_active:
        sw_flag = bool(shared_data.rti["software_estop"][0])
        current_errors.add("SW_ESTOP" if sw_flag else "ESTOP")

    # ------------------------------------------------------------------
    # EXEC control-link lost — command_executor went silent while EXEC motion
    # was in flight (caller pre-computes this from the executor heartbeat). A
    # hard, latching error: it disables + holds and requires a clear, same as
    # an E-STOP. See gui_executor_migration_plan.md Phase 5.
    # ------------------------------------------------------------------
    if exec_link_lost:
        current_errors.add("EXEC_LINK_LOST")

    # RTI command watchdog. Same shape as EXEC_LINK_LOST: the PC holding real-time control
    # went silent for longer than the timeout, so RTI stopped driving. A hard latching error
    # rather than a passing warning, because control was revoked mid-motion and the operator
    # should have to look before anyone re-claims it.
    #
    # It has to come through here as a KEY. The handler used to set rti['error_active']
    # directly, which did nothing at all: process_errors_tick() recomputes that flag from
    # this error set every tick, so the write was overwritten within one tick and no error
    # ever latched. Caught on hardware 2026-08-02 by 25_rti_comm.py --test-timeout.
    if rti_link_lost:
        current_errors.add("RTI_LINK_LOST")

    # ------------------------------------------------------------------
    # RT loop health (caller pre-computes both from LoopDegradationMonitor).
    # LOOP_DEGRADED is a warning: it is a live condition, self-clears when the loop
    # recovers, and does NOT set error_active. LOOP_CRITICAL is a hard, latching
    # error — sustained loss of the real-time guarantee, so it disables + holds and
    # requires a clear, same as an E-STOP.
    # ------------------------------------------------------------------
    if loop_degraded:
        current_errors.add("LOOP_DEGRADED")
    if loop_critical:
        current_errors.add("LOOP_CRITICAL")

    # ------------------------------------------------------------------
    # Motor driver errors (per joint)
    # These come from the motor over CAN and are updated every ~20 ticks.
    # A motor in error enters idle mode and can no longer hold position.
    # ------------------------------------------------------------------
    motor_error_fields = [
        ("motor_temperature_error", "TEMPERATURE"),
        ("motor_encoder_error",     "ENCODER"),
        ("motor_vbus_error",        "VBUS"),
        ("motor_driver_error",      "DRIVER"),
        ("motor_velocity_error",    "VELOCITY"),
        ("motor_current_error",     "CURRENT"),
        ("motor_estop_error",       "ESTOP_MOTOR"),
        ("motor_watchdog_error",    "WATCHDOG"),
    ]

    for i in range(joint_num):
        # Trust the motor's LIVE aggregate fault bit over the slow per-type breakdown.
        #
        # global_motor_error rides the DATA_PACK replies, so it is refreshed EVERY tick. The
        # per-type flags below come from the round-robin error poll, which only reaches a given
        # motor every ~3 * n_motors ticks (~84 ms on a 6-joint arm + gripper). That gap is what
        # made "Clear Errors" need two presses: a motor polled after the CAN clear was sent but
        # BEFORE it had processed it replies "still faulted", refilling its deque with a
        # pre-clear reading that survives the latch wipe and immediately re-latches. Which joints
        # got hit depended on where the round-robin happened to be — hence "some clear, some don't".
        #
        # So: if the motor says it has no fault right now, its per-type flags are stale and must
        # not be raised. send_clear_errors already treats this bit as authoritative (it only sends
        # the CAN clear to motors whose global bit is set), so this matches the existing contract.
        # Note the flags init to -1 (unknown, pre-first-reply), which is != 0 and so does NOT
        # suppress — unknown stays conservative.
        if shared_data.motor['global_motor_error'][i] == 0:
            continue
        for field, short_key in motor_error_fields:
            if shared_data.motor[field][i]:
                current_errors.add(_joint_key(i, short_key))

    # ------------------------------------------------------------------
    # CAN connection health (per joint)
    # ------------------------------------------------------------------
    for i in range(joint_num):
        # Hard error: motor has been completely silent for ~50 loop ticks.
        if shared_data.rti["CAN_node_connection_error"][i]:
            current_errors.add(_joint_key(i, "CAN_LOST"))

        # Soft warning: data is getting old (~10 ticks) but not yet a hard error.
        # Stored as a warning-only key so it does not set error_active.
        if shared_data.rti["old_data_warning"][i]:
            current_errors.add(_joint_key(i, "CAN_STALE"))

    # ------------------------------------------------------------------
    # Homing state — general warning when robot is not homed
    # ------------------------------------------------------------------
    if shared_data.homed_state.get() != "HOMED":
        current_errors.add("NOT_HOMED")

    # ------------------------------------------------------------------
    # Homing status (per joint)
    # homing_status values: 0=idle, 1=running, 2=done, 3=failed
    # ------------------------------------------------------------------
    for i in range(joint_num):
        if shared_data.rti["homing_status"][i] == 3:
            current_errors.add(_joint_key(i, "HOMING_FAILED"))

    # ------------------------------------------------------------------
    # Gripper errors (only checked when a CAN gripper is configured)
    # ------------------------------------------------------------------
    if active_CAN_gripper:
        # Disconnection: gripper node has been silent for ~50 ticks.
        # Uses CAN_node_connection_error[joint_num] — same threshold as joint motors.
        # This is much more robust than the per-tick received_ids check, which would
        # falsely report a disconnect on every single missed tick.
        if shared_data.rti["CAN_node_connection_error"][joint_num]:
            current_errors.add("GRIPPER_DISCONNECTED")

        # Fault bits reported by the gripper itself over CAN.
        if shared_data.gripper["gripper_timeout_error"][0]:
            current_errors.add("GRIPPER_TIMEOUT")
        if shared_data.gripper["gripper_estop_error"][0]:
            current_errors.add("GRIPPER_ESTOP")

        # The gripper's live aggregate fault bit. Read from node_error_bits, NOT from
        # shared_data.gripper['gripper_error']: that one is only refreshed by gripper-status
        # replies, so in MOTOR mode (where the gripper is driven with Send_data_pack_1 and answers
        # with DATA_PACK instead) it holds whatever firmware mode last saw — stale forever.
        # node_error_bits comes off the CAN ID of every reply, so it is live in both modes.
        _gripper_faulted = shared_data.rti["node_error_bits"][joint_num] != 0
        if _gripper_faulted:
            current_errors.add("GRIPPER_ERROR")

        # Motor-level faults from the gripper's motor driver (same as arm motors).
        gripper_motor_error_fields = [
            ("gripper_motor_temp_error",     "GRIPPER_MOTOR_TEMPERATURE"),
            ("gripper_motor_enc_error",      "GRIPPER_MOTOR_ENCODER"),
            ("gripper_motor_vbus_error",     "GRIPPER_MOTOR_VBUS"),
            ("gripper_motor_driver_error",   "GRIPPER_MOTOR_DRIVER"),
            ("gripper_motor_velocity_error", "GRIPPER_MOTOR_VELOCITY"),
            ("gripper_motor_current_error",  "GRIPPER_MOTOR_CURRENT"),
            ("gripper_motor_estop_error",    "GRIPPER_MOTOR_ESTOP"),
            ("gripper_motor_watchdog_error", "GRIPPER_MOTOR_WATCHDOG"),
        ]
        # Same rule as the arm joints: these per-type flags are the SLOW round-robin breakdown, so
        # only trust them while the gripper's live fault bit confirms a fault. Without this the
        # gripper had the arm's "Clear Errors needs two presses" race — a stale pre-clear reading
        # refilling its deque and surviving the latch wipe.
        if _gripper_faulted:
            for field, error_key in gripper_motor_error_fields:
                if shared_data.gripper[field][0]:
                    current_errors.add(error_key)

    # ------------------------------------------------------------------
    # Drop resolved WARNINGS from the latch (hard errors are handled below).
    #
    # Warnings are live CONDITIONS, not faults, so they must not outlive the condition. Latching
    # them made the active list lie: NOT_HOMED kept claiming "robot is not homed" AFTER a
    # successful home until the user hit Clear Errors (same for CAN_STALE once the bus recovered,
    # and HOMING_FAILED after a successful re-home). Per this module's own rule of thumb, latching
    # is for motor faults that need an explicit CAN clear — not for conditions that resolve
    # themselves. No history is lost: every appearance is already written to the activity log.
    # ------------------------------------------------------------------
    latched_errors -= {k for k in latched_errors
                       if k.endswith(_WARNING_SUFFIXES) and k not in current_errors}

    # ------------------------------------------------------------------
    # Log newly appeared errors and add them to the latch.
    # HARD errors stay latched until clear_errors() is called — they do NOT
    # self-remove when the underlying condition goes away.
    # ------------------------------------------------------------------
    newly_appeared = current_errors - latched_errors

    for key in newly_appeared:
        short_key = key.split(":", 1)[-1]          # strips the "J{i}:" prefix if present
        description = _cause_text(short_key, key)
        if key.startswith("J") and ":" in key:
            joint_idx = key.split(":")[0][1:]      # e.g. "J2:TEMPERATURE" → "2"
            if short_key in ("CAN_STALE", "HOMING_FAILED"):
                logging.warning(f"[WARNING] Joint {joint_idx}: {description}")
            else:
                logging.error(f"[ERROR] Joint {joint_idx}: {description}")
        elif short_key in ("NOT_HOMED",):
            logging.warning(f"[WARNING] {description}")
        else:
            logging.error(f"[ERROR] {description}")
        latched_errors.add(key)

    # ------------------------------------------------------------------
    # Decide whether the robot is in a hard error state.
    # CAN_STALE, HOMING_FAILED and NOT_HOMED are warnings only — they do not set error_active.
    # ------------------------------------------------------------------
    hard_errors = {k for k in latched_errors if not k.endswith(_WARNING_SUFFIXES)}
    error_active = bool(hard_errors)

    return error_active, latched_errors


def process_errors_tick(
    shared_data,
    Motor,
    Motor_errors_data,
    joint_num: int,
    clear_error_countdown: int,
    latched_errors: set,
    error_metadata: dict,
    active_CAN_gripper: int,
    current_mode,
    homing_ctrl,
    estop_active: bool,
    Gripper_data=None,
    exec_link_lost: bool = False,
    rti_link_lost: bool = False,
    loop_degraded: bool = False,
    loop_critical: bool = False,
) -> tuple:
    """Run one tick of error processing in the RTI superloop.

    Handles the clear-error signal, settle countdown, error detection,
    homing invalidation, mode forcing, metadata bookkeeping, and
    shared-memory publishing in one call.

    Args:
        shared_data:            Shared memory module.
        Motor:                  List of SpectralCAN motor objects.
        Motor_errors_data:      Per-motor deques of error-poll dicts.
        joint_num:              Number of joints.
        clear_error_countdown:  Current countdown value (ticks remaining).
        latched_errors:         The current latched error set.
        error_metadata:         Dict of error key → metadata (timestamp, tick, …).
        active_CAN_gripper:     1 if a CAN gripper is configured, 0 if not.
        current_mode:           Current RobotMode value.
        homing_ctrl:            HomingController instance (needed to abort homing on error).
        estop_active:           Result of check_estop_active() — pre-computed by caller.
        Gripper_data:           Per-node deques of gripper CAN dicts.

    Returns:
        (clear_error_countdown, latched_errors, error_metadata, current_mode)
    """
    # ── Clear-error signal: send CAN clear to errored motors, start settle countdown ──
    if shared_data.clear_error_signal.get() == "1":
        send_clear_errors(
            shared_data, Motor, Motor_errors_data, joint_num, CLEAR_ERROR_SEND_REPEATS,
            Gripper_data=Gripper_data, active_CAN_gripper=active_CAN_gripper,
        )
        clear_error_countdown = CLEAR_ERROR_SETTLE_TICKS
        shared_data.clear_error_signal.set("0")
        logging.info(
            f"[ERRORS] CAN clear sent to errored motors. "
            f"Settling for {CLEAR_ERROR_SETTLE_TICKS} ticks before wiping latch."
        )

    # ── Settle countdown: wipe latch only after N clean verification ticks ──
    clear_error_countdown, latched_errors = tick_clear_countdown(
        clear_error_countdown, latched_errors, shared_data, active_CAN_gripper,
    )

    # ── Detect errors ────────────────────────────────────────────────────────
    error_active, latched_errors = check_errors(
        shared_data, joint_num, latched_errors,
        estop_active=estop_active,
        active_CAN_gripper=active_CAN_gripper,
        exec_link_lost=exec_link_lost,
        rti_link_lost=rti_link_lost,
        loop_degraded=loop_degraded,
        loop_critical=loop_critical,
    )
    shared_data.rti["error_active"][0] = 1.0 if error_active else 0.0

    # ── CAN disconnect → invalidate homing ───────────────────────────────────
    disconnect_errors = {
        k for k in latched_errors
        if k.endswith(":CAN_LOST") or k == "GRIPPER_DISCONNECTED"
    }
    if disconnect_errors and shared_data.homed_state.get() == "HOMED":
        shared_data.homed_state.set("NOT HOMED")
        shared_data.rti["homing_status"][:] = 0
        logging.error(f"[RTI] Connection lost ({disconnect_errors}) — homing invalidated.")

    # ── Hard error → force DISABLED / ACTIVE_ERROR (velocity-hold) ─────────
    if error_active and current_mode != RobotMode.BOOTING:
        if shared_data.robot_state.get() != "DISABLED":
            shared_data.robot_state.set("DISABLED")
            logging.error("[RTI] Error active — robot disabled.")
        # MAINTENANCE_MODES are exempt from the forced mode change — they still
        # go DISABLED above (motors ARE limp), but they keep the mode.
        #
        # Without this, FLASHING is unreachable in the exact case it exists for.
        # A driver sitting in its bootloader answers nothing, which latches
        # CAN_LOST; state_machine.py deliberately allows ACTIVE_ERROR → FLASHING
        # for that reason, but this block ran every tick and dragged the mode
        # straight back out, so the transition was accepted and then reverted
        # ~4 ms later. Found on hardware 2026-08-04 (bench rig, boards faulting
        # on absent encoders): RTI logged "→ RobotMode.FLASHING" four times while
        # shared memory never left ACTIVE_ERROR and the bus never went quiet.
        # The transition table alone cannot express this; only the per-tick
        # handler can.
        if (current_mode != RobotMode.ACTIVE_ERROR
                and current_mode not in MAINTENANCE_MODES):
            if current_mode == RobotMode.HOMING:
                homing_ctrl.exit(Motor, joint_num)
                shared_data.homed_state.set("NOT HOMED")
                shared_data.rti["homing_status"][:] = 0
                logging.error("[RTI] Error during homing — homing aborted, homed state cleared.")
                # Restore normal PID config in shared memory — homing_ctrl.exit() restores
                # motor limits via CAN but error_checks bypasses _HomingDispatch.exit().
                mc = getattr(homing_ctrl, '_motor_config', None)
                if mc is not None:
                    write_motor_config_to_shared(
                        shared_data, joint_num,
                        KPP=mc['KPP'], KPV=mc['KPV'], KIV=mc['KIV'],
                        KPIQ=mc['KPIQ'], KIIQ=mc['KIIQ'],
                        KP=mc['KP'], KD=mc['KD'],
                        Ilim=mc['Ilim'], velocity_limit=mc['velocity_limit'],
                    )
            current_mode = RobotMode.ACTIVE_ERROR
            shared_data.robot_mode.set(current_mode)
            logging.error("[RTI] Error active — entering ACTIVE_ERROR (velocity-hold).")
    elif not error_active and current_mode == RobotMode.ACTIVE_ERROR:
        current_mode = RobotMode.IDLE
        shared_data.robot_mode.set(current_mode)
        logging.info("[RTI] Errors cleared — returning to IDLE.")

    # ── Update error metadata: add new, prune cleared ────────────────────────
    tick = int(shared_data.rti["loop_tick"][0])
    existing_error_keys = set(error_metadata)
    new_error_keys = latched_errors - existing_error_keys
    cleared_error_keys = existing_error_keys - latched_errors
    error_metadata_changed = bool(new_error_keys or cleared_error_keys)

    if new_error_keys:
        now = time.time()
    for key in new_error_keys:
        short_key   = key.split(":", 1)[-1]
        description = _cause_text(short_key, key)
        level = "warning" if short_key in _WARNING_SUFFIXES else "error"
        error_metadata[key] = {"timestamp": now, "tick": tick, "description": description, "level": level}
        # Activity log — debounced per-code so a flapping/marginal fault can't
        # spam the file (design doc §4). joint_idx folded into `message` since
        # `code` in the catalog is the short_key, not the "J{i}:KEY" form.
        if _activity_rate_limiter.allow(short_key, _ACTIVITY_MIN_INTERVAL):
            joint_msg = None
            if key.startswith("J") and ":" in key:
                joint_msg = f"joint {key.split(':')[0][1:]}"
            log_activity(get_rti_activity_logger(), "RTI", level, short_key, message=joint_msg)
    for key in cleared_error_keys:
        del error_metadata[key]

    # ── Publish rich error list to shared memory (sorted by timestamp) ────────
    # SharedString.set_json() serializes and zero-fills the whole shared-memory
    # block, so only publish on edge changes instead of every RT tick.
    if error_metadata_changed:
        if error_metadata:
            error_list = sorted(
                [{"key": k, **v} for k, v in error_metadata.items()],
                key=lambda e: e["timestamp"],
            )
            shared_data.error_state.set_json(error_list)
        else:
            shared_data.error_state.set_json("NONE")

    return clear_error_countdown, latched_errors, error_metadata, current_mode
