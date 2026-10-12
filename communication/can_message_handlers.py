#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: can_message_handlers.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    CAN message reception and processing handlers.
    Unpacks CAN messages and stores data in appropriate data structures.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""
import logging
import time
import numpy as np
from config.xml_parser import clamp_voltage_limit
from utility.data_init import (
    process_motor_data,
    process_gripper_data,
    process_motor_voltage,
    process_motor_errors,
    process_motor_temperature,
    process_motor_info,
    process_motor_kt
)

logger = logging.getLogger(__name__)

# Safety bound on how many CAN frames one tick may drain. NOT a target: the RX loop exits on
# the first empty read, which in steady state is after ~8 frames (6 joints + gripper + 1
# round-robin poll reply). The headroom above that is what lets a backlog catch up — ~24
# surplus frames/tick clears a ~100-frame backlog in about 4 ticks. Sized so even a full
# catch-up tick stays inside budget: 32 x ~28 us/frame = ~0.9 ms of a 4 ms tick.
MAX_RX_FRAMES_PER_TICK = 32

# ==========================================================================
# CAN connection Safety and debug stuff 
# ==========================================================================

# NOTE  We look how old is our Motor_data based on the loop tick. If data is too old we do something (error, stop...)
#   This is different from receive ID because Receive id tells us we got NOTHING from that node ID while this will tell us 
#   we got no motor data (pos, speed, current) that is needed for normal robot operation. 
#   NOTE THIS CHECK HAS LOWER PRIORITY THEN CHECK RECEIVED IDS
def update_can_connection_status(
    *,
    Motor_data,
    Motor,
    shared_data,
    joint_num: int,
    active_CAN_gripper: int,
    warning_threshold: int = 10,
    error_threshold: int = 50,
):
    """
    Monitor CAN node health based on data age and received IDs.
    """

    current_tick = shared_data.rti["loop_tick"][0]

    # ---- Freshness-based checks ----
    # old_data_warning: live flag — auto-resets when fresh data arrives.
    #   Used to determine the live "connected/disconnected" status of each node.
    # CAN_node_connection_error: latched flag — only set to 1, never auto-cleared.
    #   Only zeroed when the user explicitly calls "Clear Errors".
    for i in range(joint_num + active_CAN_gripper):
        tick_diff = current_tick - Motor_data[i][-1]["tick"]

        if tick_diff >= warning_threshold:
            shared_data.rti["old_data_warning"][i] = 1
        else:
            shared_data.rti["old_data_warning"][i] = 0

        if tick_diff >= error_threshold:
            shared_data.rti["CAN_node_connection_error"][i] = 1
        # No else — CAN_node_connection_error is latched; only clear_errors() resets it.

    # ---- Gripper live connection status ----
    # Derived from old_data_warning (live, auto-resets) so the display recovers
    # immediately when the gripper reconnects, independent of the error latch.
    if active_CAN_gripper == 1:
        shared_data.rti["CAN_gripper_connected"][0] = (
            0 if shared_data.rti["old_data_warning"][joint_num] else 1
        )
    else:
        shared_data.rti["CAN_gripper_connected"][0] = 0

    # ---- Overall robot CAN connection status ----
    # Live indicator — uses old_data_warning (auto-resets when fresh data arrives)
    # so the badge recovers immediately when the robot reconnects.
    shared_data.rti['robot_can_connected'][0] = float(
        not np.any(shared_data.rti['old_data_warning'][:joint_num])
    )


def handle_motor_reconnect(
    *,
    shared_data,
    Motor,
    joint_num: int,
    active_CAN_gripper: int,
    startup_selfcheck: int,
    prev_old_data_warning,
    reload_last_tick,
    reload_pending,
    cooldown_ticks,
    motor_watchdog_rate,
    motor_velocity_limit,
    motor_Ilim,
    motor_KPP, motor_KP, motor_KD,
    motor_KPIQ, motor_KIIQ,
    motor_KPV, motor_KIV,
    motor_voltage_limit=None,
    gripper_watchdog_rate=5000,
    gripper_velocity_limit=None,
    gripper_Ilim=None,
    gripper_voltage_limit=None,
    gripper_KP=None, gripper_KD=None,
    gripper_KPIQ=None, gripper_KIIQ=None,
    gripper_KPV=None, gripper_KIV=None,
    gripper_KPP=None,
):
    """Reload motor/gripper config when a CAN node recovers after being stale.

    Detects stale→fresh edge on old_data_warning per node and re-sends
    watchdog, limits, and gains to that node only (motor_index=i, num_repeats=2).
    prev_old_data_warning is a numpy array updated in-place each call.
    Gated behind startup_selfcheck to avoid a redundant reload at boot.
    Runs on the RT tick, so num_repeats stays small: a reconnected node needs 14 frames.

    STORM GUARD (2026-10-01). A recovered node is marked pending; pending nodes are reloaded ONE PER TICK, each
    at most once per cooldown_ticks. Measured: when the other cores saturate memory, the CAN path (RP1 SPI)
    starves, every node flaps stale/fresh several times a second, and reloading all of them on every flap made
    one tick take 2 s. A reload is DEFERRED, never dropped: a driver that really lost power (and its config)
    still gets it, at most one cooldown late. reload_last_tick / reload_pending: per-node state (RTI owns it).
    """
    if startup_selfcheck != 1:
        return

    tick = int(shared_data.rti['loop_tick'][0])
    nodes = joint_num + (1 if active_CAN_gripper else 0)
    for i in range(nodes):
        _cur_warn = int(shared_data.rti['old_data_warning'][i])
        if prev_old_data_warning[i] == 1 and _cur_warn == 0 and not reload_pending[i]:
            reload_pending[i] = True
            if tick - reload_last_tick[i] < cooldown_ticks:
                name = f"Motor J{i + 1}" if i < joint_num else "Gripper"
                logger.warning(f"[RTI] {name} went stale and recovered again within "
                               f"{cooldown_ticks} ticks of its last config reload — next reload deferred "
                               f"(many nodes at once = a host-side CAN stall, e.g. memory starvation)")
        prev_old_data_warning[i] = _cur_warn

    for i in range(nodes):                          # serve ONE pending node per tick
        if (reload_pending[i] and int(shared_data.rti['old_data_warning'][i]) == 0
                and tick - reload_last_tick[i] >= cooldown_ticks):
            reload_pending[i] = False
            reload_last_tick[i] = tick
            if i < joint_num:
                _reload_motor(i, shared_data, Motor, joint_num, motor_watchdog_rate, motor_velocity_limit,
                              motor_Ilim, motor_KPP, motor_KP, motor_KD, motor_KPIQ, motor_KIIQ, motor_KPV,
                              motor_KIV, motor_voltage_limit)
            else:
                _reload_gripper(shared_data, Motor, joint_num, gripper_watchdog_rate, gripper_velocity_limit,
                                gripper_Ilim, gripper_voltage_limit, gripper_KP, gripper_KD, gripper_KPIQ,
                                gripper_KIIQ, gripper_KPV, gripper_KIV, gripper_KPP)
            return


def _reload_motor(i, shared_data, Motor, joint_num, motor_watchdog_rate, motor_velocity_limit, motor_Ilim,
                  motor_KPP, motor_KP, motor_KD, motor_KPIQ, motor_KIIQ, motor_KPV, motor_KIV, motor_voltage_limit):
    """Re-send one joint's config after it recovered (handle_motor_reconnect's storm guard decides when)."""
    logger.info(f"[RTI] Motor J{i+1} reconnected — reloading config.")
    load_motor_configs(Motor,
                       joint_num=joint_num,
                       watchdog_rate=motor_watchdog_rate,
                       velocity_limit=motor_velocity_limit,
                       current_limit=motor_Ilim,
                       KP=motor_KP, KD=motor_KD,
                       KPIQ=motor_KPIQ, KIIQ=motor_KIIQ,
                       KPV=motor_KPV, KIV=motor_KIV,
                       KPP=motor_KPP,
                       watchdog_action="Idle",
                       num_repeats=2,
                       motor_index=i,
                       voltage_limit=motor_voltage_limit)
    write_motor_config_to_shared(
        shared_data, joint_num,
        KPP=motor_KPP, KPV=motor_KPV, KIV=motor_KIV,
        KPIQ=motor_KPIQ, KIIQ=motor_KIIQ,
        KP=motor_KP, KD=motor_KD,
        Ilim=motor_Ilim, velocity_limit=motor_velocity_limit,
        voltage_limit=motor_voltage_limit,
    )
    logger.info(f"[RTI] Motor J{i+1} config reload done.")


def _reload_gripper(shared_data, Motor, joint_num, gripper_watchdog_rate, gripper_velocity_limit, gripper_Ilim,
                    gripper_voltage_limit, gripper_KP, gripper_KD, gripper_KPIQ, gripper_KIIQ, gripper_KPV,
                    gripper_KIV, gripper_KPP):
    """Re-send the gripper's config after it recovered (handle_motor_reconnect's storm guard decides when)."""
    logger.info("[RTI] Gripper reconnected — reloading config.")
    load_gripper_configs(Motor,
                         joint_num=joint_num,
                         watchdog_rate=gripper_watchdog_rate,
                         velocity_limit=gripper_velocity_limit,
                         current_limit=gripper_Ilim,
                         KP=gripper_KP, KD=gripper_KD,
                         KPIQ=gripper_KPIQ, KIIQ=gripper_KIIQ,
                         KPV=gripper_KPV, KIV=gripper_KIV,
                         KPP=gripper_KPP,
                         watchdog_action="Idle",
                         num_repeats=2,
                         voltage_limit=gripper_voltage_limit)
    write_gripper_config_to_shared(
        shared_data,
        KPP=gripper_KPP, KPV=gripper_KPV, KIV=gripper_KIV,
        KPIQ=gripper_KPIQ, KIIQ=gripper_KIIQ,
        KP=gripper_KP, KD=gripper_KD,
        Ilim=gripper_Ilim, velocity_limit=gripper_velocity_limit,
        voltage_limit=gripper_voltage_limit,
    )
    logger.info("[RTI] Gripper config reload done.")


def send_motor_commands(motors, joint_num, positions=None, speeds=None, currents=None):
    """Send data_pack_1 to all arm joints.

    All arrays are optional and indexed by joint (0..joint_num-1).
    Defaults (all None / 0):
      Position=None, Speed=None, Current=0  → motor brake/idle hold.

    Args:
        motors:    list of motor objects
        joint_num: number of arm joints to command
        positions: target position array or None
        speeds:    target speed array or None
        currents:  feedforward current/torque array or None (default 0 each)
    """
    for i in range(joint_num):
        motors[i].Send_data_pack_1(
            Position=int(positions[i]) if positions is not None else None,
            Speed=int(speeds[i])    if speeds    is not None else None,
            Current=int(currents[i]) if currents is not None else 0,
        )

def send_motor_commands_pd(motors, joint_num, positions, speeds, currents=None):
    """Send data_pack_PD to all arm joints (PD impedance control mode).

    Unlike data_pack_1, the firmware uses its on-board KP/KD gains to compute
    torques directly from the position and speed errors — no integral term.

    Args:
        motors:    list of motor objects
        joint_num: number of arm joints to command
        positions: target position array (required, ticks)
        speeds:    target speed array (required, ticks/s)
        currents:  feedforward current/torque array or None (default 0 each)
    """
    for i in range(joint_num):
        motors[i].Send_data_pack_PD(
            Position=int(positions[i]),
            Speed=int(speeds[i]),
            Current=int(currents[i]) if currents is not None else 0,
        )


# How many consecutive action=0 packs announce "go idle" to the gripper firmware before
# we fall back to silent empty polls. More than one because the send layer swallows TX
# errors, and a single lost frame would strand the gripper holding position.
GRIPPER_IDLE_PACK_REPEATS = 3


def send_gripper_commands(motors, dummy_device, active_gripper, joint_num,
                          ctrl_mode=1, gripper_state=None, calibrated=True):
    """2. Handles gripper CAN messages or dummy timing messages.

    Args:
        active_gripper: 1 if a CAN gripper is configured, 0 otherwise.
        ctrl_mode:      1 = motor mode (idle ping), 2 = firmware mode (always
                        Send_gripper_data_pack with activate forced to 1).
        gripper_state:  dict for firmware mode with keys position, speed,
                        current, action, estop, release_dir.

    Motor mode falls back to Send_data_pack_1(Current=0), same as the arm
    motors in IDLE, which keeps the bus / watchdog alive without clearing
    latched firmware errors.
    """
    if active_gripper == 1:
        # Mirror the firmware's OWN gate, do not guess at it. motor_control.cpp runs the
        # position/velocity path only when
        #     action_status == 1 && calibrated && activated && I_AM_GRIPPER
        # so BOTH action=0 and an uncalibrated gripper land in its "Idle or disabled"
        # else-branch. Keying only on `action` would leave the boot case buzzing, because
        # at boot the gripper has not calibrated yet and no value of `action` changes that.
        _gripper_idle = (gripper_state is None
                         or not int(gripper_state.get('action', 0))
                         or not calibrated)
        if ctrl_mode == 2 and gripper_state is not None and _gripper_idle \
                and gripper_state.get('_idle_repeats', 0) > 0:
            # GOING idle: the firmware must be TOLD once, then left alone.
            #
            # The empty pack below cannot carry action=0 — communication_CAN.cpp only
            # assigns Gripper.action_status inside its len==5 branch, and nothing else
            # in the firmware ever clears it. So if we dropped straight to empty polls
            # after a move, the driver would keep action_status=1, keep running
            # Gripper_mode() (Controller_mode stays 6) and HOLD POSITION FOREVER —
            # Stop would silently stop working.
            #
            # So send a real pack with action forced to 0. That puts the firmware in
            # its "Idle or disabled" branch: velocity 0, SLEEP and RESET LOW, output
            # stage disabled — a genuinely limp gripper. Repeated a few ticks because
            # a lost frame here would strand it holding (the send layer swallows TX
            # errors), and a handful of ticks is a click, not an audible tone.
            motors[joint_num].Send_gripper_data_pack(
                position_=int(gripper_state.get('position', 0)) & 0xFF,
                speed_=int(gripper_state.get('speed', 0)) & 0xFF,
                current_=int(gripper_state.get('current', 0)),
                activate_=1,
                action_=0,
                ESTOP_=int(gripper_state.get('estop', 0)) & 0x1,
                Release_dir_=int(gripper_state.get('release_dir', 0)) & 0x1,
            )
            gripper_state['_idle_repeats'] = gripper_state.get('_idle_repeats', 0) - 1
        elif ctrl_mode == 2 and gripper_state is not None and _gripper_idle:
            # ALREADY idle — the firmware knows. Hold with the empty poll.
            #
            # Streaming a full 5-byte pack with action=0 makes the driver BUZZ, and
            # the firmware shows exactly why. Two places fight each other at the
            # loop rate:
            #   communication_CAN.cpp IN_GRIPPER_DATA_PACK, len==5 branch: on every
            #     pack it drives SLEEP and RESET HIGH — the driver stage is ENABLED.
            #   motor_control.cpp gripper control, the `else` at "Idle or disabled"
            #     (action_status==0): sets velocity 0 then drives SLEEP and RESET
            #     LOW — the driver stage is DISABLED.
            # So at 250 Hz the output stage is enabled by the CAN handler and
            # disabled by the control loop, over and over. That toggle is the buzz.
            # It stops the instant a real Open/Close arrives because action becomes
            # 1 and the control loop takes the position/velocity path instead.
            #
            # The empty (zero-length) pack is the firmware's own designed no-op for
            # this: its len==0 branch still sets watchdog_reset=1 (so the watchdog
            # stays fed), leaves Controller_mode alone, touches no setpoint, marks
            # Same_command=true so At_position is not reset, and still replies via
            # Gripper_pack_data() so telemetry keeps flowing. Critically it does NOT
            # raise SLEEP/RESET, so the stage stays quietly disabled — which is the
            # state action=0 was always asking for.
            motors[joint_num].Send_gripper_data_pack()
        elif ctrl_mode == 2 and gripper_state is not None:
            # Active command. Re-arm the idle handshake so that whenever this command
            # ends, the firmware is told once rather than left holding.
            gripper_state['_idle_repeats'] = GRIPPER_IDLE_PACK_REPEATS
            motors[joint_num].Send_gripper_data_pack(
                position_=int(gripper_state.get('position', 0)) & 0xFF,
                speed_=int(gripper_state.get('speed', 0)) & 0xFF,
                current_=int(gripper_state.get('current', 0)),
                activate_=1,
                action_=int(gripper_state.get('action', 0)) & 0x1,
                ESTOP_=int(gripper_state.get('estop', 0)) & 0x1,
                Release_dir_=int(gripper_state.get('release_dir', 0)) & 0x1,
            )
        else:
            motors[joint_num].Send_data_pack_1(Position=None, Speed=None, Current=0)
    else:
        # Dummy data keeps loop timing consistent (prevents 1ms jitter)
        dummy_device.Send_Respond_Ping()

def handle_polling_and_overrides(motors, state, overrides, override_queue, joint_num, active_CAN_gripper=0):
    """3. Manages high-priority overrides and round-robin health polling.
    Includes the gripper motor in the round-robin when active_CAN_gripper=1."""
    
    # --- Handle Override Queue ---
    if overrides["fn"] is None and override_queue:
        fn, count = override_queue.popleft()
        overrides["fn"] = fn
        overrides["count"] = count

    if overrides["fn"] is not None:
        try:
            overrides["fn"]() # Execute override (e.g., ESTOP or Reset)
        except Exception as e:
            logger.warning(f"Override execution failed: {e}")

        overrides["count"] -= 1
        if overrides["count"] <= 0:
            overrides["fn"] = None
        return # Skip polling if an override was processed this cycle

    # --- Handle Round-Robin Status Polling ---
    total_motors = joint_num + active_CAN_gripper  # includes gripper motor when present
    state["device_info_counter"] += 1
    step = state["step"]
    idx = state["joint_index"]

    # Sweep Window for Device Info (1000 to 1000+N)
    if 1000 <= state["device_info_counter"] < (1000 + total_motors):
        motors[idx].Send_Respond_Device_Info()
        if state["device_info_counter"] == (1000 + total_motors - 1):
            state["device_info_counter"] = 0
    else:
        # Normal Telemetry
        if step == 0:
            motors[idx].Send_Respond_Temperature()
        elif step == 1:
            motors[idx].Send_Respond_Voltage()
        elif step == 2:
            motors[idx].Send_Respond_State_of_Errors()

    # Advance indices
    state["joint_index"] += 1
    if state["joint_index"] >= total_motors:
        state["joint_index"] = 0
        if state["device_info_counter"] != 0:
            state["step"] = (state["step"] + 1) % 3


def secondary_data_send():
    None


# ==========================================================================
# Motor Configuration Pre-loading
# ==========================================================================

def load_motor_configs(
    Motor,
    joint_num: int,
    watchdog_rate,
    velocity_limit,
    current_limit,
    KP, KD, KPIQ, KIIQ, KPV, KIV, KPP,
    watchdog_action="Idle",
    num_repeats: int = 2,
    motor_index: int = None,
    voltage_limit=None,
    pace_s: float = 0.0,
):
    """
    Pre-load motor driver configurations (watchdog, limits, gains).
    All per-motor parameters must be array-like of length joint_num.

    motor_index: if set, only reload config for that one joint index.
                 If None (default), reload all joints.
    voltage_limit: per-motor voltage limit in mV (0 = use VBUS), or None to not send it at all.
                 Its own CAN command, so a driver whose firmware predates it simply ignores the
                 frame — it cannot corrupt the other config messages. Config-time only, so it
                 adds no per-tick bus load.
    pace_s: seconds to sleep after each message-type loop (0 = no pacing).
                 The CAN library SWALLOWS send failures, so overflowing the socketcan TX
                 queue (~270 frames of skb budget) silently DROPS frames — and an all-joints
                 load is ~170 frames enqueued in microseconds against a 1 Mbit bus that
                 drains ~10/ms. Pacing per message-type loop (len(targets) frames) bounds
                 the queue at ~a dozen frames. Boot passes ~0.0005; in-loop callers MUST
                 keep 0 (a sleep here blows the 4 ms tick) — their bursts are either too
                 small to overflow or idempotent re-sends of already-loaded config.
    """
    targets = [motor_index] if motor_index is not None else range(joint_num)

    def _pace():
        if pace_s > 0:
            time.sleep(pace_s)

    for repeat in range(num_repeats):
        logger.info(f"Configuration pass {repeat + 1}/{num_repeats}")

        for i in targets:
            Motor[i].Send_Watchdog(_rate=int(watchdog_rate[i]), _action=watchdog_action)
        _pace()

        for i in targets:
            Motor[i].Send_Limits(Velocity_limit=int(velocity_limit[i]), Current_limit=int(current_limit[i]))
        _pace()

        if voltage_limit is not None:
            for i in targets:
                # Final guarantee: whatever the caller passed, only 0 (use VBUS) or a value at/above
                # the floor reaches the wire. Callers upstream clamp too — this is what makes it a rule.
                Motor[i].Send_Voltage_Limit(
                    Voltage_limit=int(clamp_voltage_limit(voltage_limit[i])))
            _pace()

        for i in targets:
            Motor[i].Send_PD_Gains(KP=float(KP[i]), KD=float(KD[i]))
        _pace()

        for i in targets:
            Motor[i].Send_Current_Gains(KPIQ=float(KPIQ[i]), KIIQ=float(KIIQ[i]))
        _pace()

        for i in targets:
            Motor[i].Send_Velocity_Gains(KPV=float(KPV[i]), KIV=float(KIV[i]))
        _pace()

        for i in targets:
            Motor[i].Send_Position_Gains(KPP=float(KPP[i]))
        _pace()

    logger.info("Motor configuration complete")


def write_motor_config_to_shared(shared_data, joint_num, KPP, KPV, KIV, KPIQ, KIIQ, KP, KD, Ilim,
                                 velocity_limit, voltage_limit=None):
    """Write the currently active per-joint PID gains and limits to shared memory.

    Call this after every load_motor_configs() so the GUI always reflects what
    is actually loaded on the motor drivers.

    voltage_limit: mV per joint, or None to leave the mirrored value untouched.
    """
    shared_data.motor_config['cfg_KPP'][:joint_num]            = KPP[:joint_num]
    shared_data.motor_config['cfg_KPV'][:joint_num]            = KPV[:joint_num]
    shared_data.motor_config['cfg_KIV'][:joint_num]            = KIV[:joint_num]
    shared_data.motor_config['cfg_KPIQ'][:joint_num]           = KPIQ[:joint_num]
    shared_data.motor_config['cfg_KIIQ'][:joint_num]           = KIIQ[:joint_num]
    shared_data.motor_config['cfg_KP'][:joint_num]             = KP[:joint_num]
    shared_data.motor_config['cfg_KD'][:joint_num]             = KD[:joint_num]
    shared_data.motor_config['cfg_Ilim'][:joint_num]           = Ilim[:joint_num]
    shared_data.motor_config['cfg_velocity_limit'][:joint_num] = velocity_limit[:joint_num]
    if voltage_limit is not None:
        shared_data.motor_config['cfg_voltage_limit'][:joint_num] = voltage_limit[:joint_num]


def write_gripper_config_to_shared(shared_data, KPP, KPV, KIV, KPIQ, KIIQ, KP, KD, Ilim,
                                   velocity_limit, voltage_limit=None):
    """Write the currently active gripper PID gains and limits to shared memory."""
    shared_data.motor_config['cfg_gripper_KPP'][0]            = float(KPP)
    shared_data.motor_config['cfg_gripper_KPV'][0]            = float(KPV)
    shared_data.motor_config['cfg_gripper_KIV'][0]            = float(KIV)
    shared_data.motor_config['cfg_gripper_KPIQ'][0]           = float(KPIQ)
    shared_data.motor_config['cfg_gripper_KIIQ'][0]           = float(KIIQ)
    shared_data.motor_config['cfg_gripper_KP'][0]             = float(KP)
    shared_data.motor_config['cfg_gripper_KD'][0]             = float(KD)
    shared_data.motor_config['cfg_gripper_Ilim'][0]           = float(Ilim)
    shared_data.motor_config['cfg_gripper_velocity_limit'][0] = float(velocity_limit)
    if voltage_limit is not None:
        shared_data.motor_config['cfg_gripper_voltage_limit'][0] = float(voltage_limit)


def load_gripper_configs(
    Motor,
    joint_num: int,
    velocity_limit,
    current_limit,
    KP, KD,
    KPIQ, KIIQ,
    KPV, KIV,
    KPP,
    watchdog_rate: int = 5000,
    watchdog_action: str = "Idle",
    num_repeats: int = 2,
    voltage_limit=None,
    pace_s: float = 0.0,
):
    """
    Pre-load gripper motor driver configuration over CAN.

    The gripper motor sits at CAN ID = joint_num (right after the last joint).
    Only call this when active_CAN_gripper == 1.

    Args:
        Motor (list): Motor objects list (gripper is at index joint_num).
        joint_num (int): Number of arm joints — also the CAN ID of the gripper.
        velocity_limit: Maximum gripper motor velocity.
        current_limit: Maximum gripper motor current (mA).
        KP, KD: Position loop PD gains.
        KPIQ, KIIQ: Current loop PI gains.
        KPV, KIV: Velocity loop PI gains.
        KPP: Position loop P gain.
        watchdog_rate (int): Watchdog timeout in ms (default 5000).
        watchdog_action (str): Action on watchdog timeout (default "Idle").
        num_repeats (int): Number of config upload passes for reliability.
        voltage_limit: Gripper voltage limit in mV (0 = use VBUS), or None to not send it.
            Its own CAN command — firmware that predates it just ignores the frame.
        pace_s: seconds to sleep after each repeat pass (7 frames), 0 = no pacing.
            See load_motor_configs — same silent-TX-drop hazard; this burst is small
            (~28 frames) but at boot it rides the TAIL of the all-joints blast, exactly
            where the queue tips over. Boot passes ~0.0005; in-loop callers keep 0.
    """
    gripper_motor = Motor[joint_num]

    def _pace():
        if pace_s > 0:
            time.sleep(pace_s)

    for repeat in range(num_repeats):
        logger.info(f"Gripper configuration pass {repeat + 1}/{num_repeats}")

        gripper_motor.Send_Watchdog(_rate=watchdog_rate, _action=watchdog_action)

        gripper_motor.Send_Limits(
            Velocity_limit=int(velocity_limit),
            Current_limit=int(current_limit)
        )

        if voltage_limit is not None:
            gripper_motor.Send_Voltage_Limit(
                Voltage_limit=int(clamp_voltage_limit(voltage_limit)))

        gripper_motor.Send_PD_Gains(
            KP=float(KP),
            KD=float(KD)
        )

        gripper_motor.Send_Current_Gains(
            KPIQ=float(KPIQ),
            KIIQ=float(KIIQ)
        )

        gripper_motor.Send_Velocity_Gains(
            KPV=float(KPV),
            KIV=float(KIV)
        )

        gripper_motor.Send_Position_Gains(KPP=float(KPP))
        _pace()

    logger.info("Gripper configuration complete")

# ==========================================================================
# Main CAN Reception Loop
# ==========================================================================

def drain_can_discard(communication, max_frames=64):
    """Read pending CAN frames and throw them away. Returns the count dropped.

    Used while RTI is silent in FLASHING mode. Two reasons it must DISCARD
    rather than simply not read:

    1. Bootloader page-data frames use CAN ID (board_id << 7) | seq, which
       decodes in the Spectral application protocol as node=board_id with
       command ids 0-63 — they look exactly like telemetry from a real motor
       and would be unpacked as such, writing garbage into shared memory.
    2. Not reading at all lets the socket's RX buffer fill with tens of
       thousands of flash frames, which would then be processed as stale
       telemetry for many ticks after the flash finished.

    max_frames bounds the work per tick: ~6000 frames/s of flash traffic at a
    250 Hz loop is ~24 frames/tick, so 64 keeps up with headroom.
    """
    dropped = 0
    for _ in range(max_frames):
        message, _ = communication.receive_can_messages(timeout=0)
        if message is None:
            break
        dropped += 1
    return dropped


def receive_and_process_can_messages(communication, Motor, Motor_data, Gripper_data,
                                     Motor_temperature_data, Motor_voltage_data,
                                     Motor_errors_data, Motor_info_data,
                                     Motor_Kt_data,
                                     joint_num, active_CAN_gripper, loop_tick_cnt,
                                     received_ids, latest_commands_id,
                                     node_error_bits=None, frame_age_out=None, data_reply_count_out=None,
                                     reply_time_out=None, receive=None):
    """
    Receive and process CAN messages from all motors.
    
    Args:
        communication: CAN communication object
        Motor (list): List of motor objects
        Motor_data (dict): Motor data deques
        Gripper_data (dict): Gripper data deques
        Motor_temperature_data (dict): Temperature data deques
        Motor_voltage_data (dict): Voltage data deques
        Motor_errors_data (dict): Error data deques
        Motor_info_data (dict): Info data deques
        joint_num (int): Number of joints
        active_CAN_gripper (int): 1 if gripper active, 0 otherwise
        loop_tick_cnt (int): Current loop tick counter
        received_ids (np.array): Array to mark received IDs
        latest_commands_id (np.array): Array to store latest command IDs
        data_reply_count_out (list): if given, filled with the number of data replies (DATA_PACK 3 / 5 / 7) read
            from each joint this tick (2026-10-12: utility/can_reply_monitor.py counts the ticks that got none)
        receive (callable): if given, used instead of communication.receive_can_messages — the same
            (message, (node, command, error_bit)) contract; option B passes hardware/can_hardware.FastCanReader's
            (2026-10-12)
        reply_time_out (np.array): if given, each joint's newest data reply's kernel arrival time (CLOCK_REALTIME)
            — KEPT when a joint had none this tick: it is the time of the data RTI works with (2026-10-12,
            utility/loop_delay_meter.py)
    
    Returns:
        tuple: (Hall_index, received_ids, latest_commands_id)
    """
    Hall_index = 0
    additional_pin2 = 0
    Edge_detect = 0
    # (the old fixed read count, joint_num + 1 + 1 + active_CAN_gripper, is gone — the loop
    #  below drains until the socket is empty instead of guessing how many frames to expect)

    # FRAME AGE: how long a frame sat in the kernel buffer before RTI got to it. python-can's
    # socketcan backend sets SO_TIMESTAMPNS, so message.timestamp is the KERNEL arrival time
    # (CLOCK_REALTIME, same domain as time.time()) — not the moment userspace read it. So this
    # measures ingest lag directly: a few ms means the bus view is current; 100 ms+ means this
    # loop is behind and RTI is acting on the past, which no amount of PID tuning can fix.
    # One clock read per TICK (not per frame): ~139 ns, and nothing is allocated.
    _now = time.time()
    _age_max = 0.0
    _age_min = None     # None until a frame arrives; distinguishes "no frames" from "age 0"
    if data_reply_count_out is not None:
        for _j in range(len(data_reply_count_out)):
            data_reply_count_out[_j] = 0

    # DRAIN-WITH-CAP. The old loop read a FIXED loop_range-1 (= 8) frames with a 0.8 ms
    # blocking timeout, which broke in two ways, both measured on this arm:
    #
    #   Staleness — 8 reads/tick against ~8 arrivals/tick is exactly break-even, so a backlog
    #   formed once (a boot burst, one slow tick) could never drain. RTI ran ~50 ms behind the
    #   bus while an independent socket on the same wire saw frames 0.023 ms old. Every
    #   measurement, every error bit and every commanded-vs-actual comparison was that stale.
    #
    #   Blocking — on a momentarily empty socket each read waited the full timeout. Profiled
    #   at can_receive_process max = 6.512 ms, in a 4 ms budget. That is the disconnect-test
    #   overrun: the loop got MOST expensive exactly when hardware was failing.
    #
    # Reading until empty fixes both: timeout=0 never waits, and the cap gives ~24 frames/tick
    # of surplus so an existing backlog clears in a few ticks and then stays clear. The cap is
    # a safety bound, not a target — steady state exits on the first empty read after ~8
    # frames. Worst case 32 reads x ~28 us = ~0.9 ms, so a catch-up tick still fits the budget.
    _receive = communication.receive_can_messages if receive is None else receive
    for _ in range(MAX_RX_FRAMES_PER_TICK):
        message, msg_ID = _receive(timeout=0)
        if message is None:
            break       # socket drained — nothing more waiting this tick

        if msg_ID is not None:
            _age = _now - message.timestamp
            if _age > _age_max:
                _age_max = _age
            if _age_min is None or _age < _age_min:
                _age_min = _age
            node_id = msg_ID[0]
            command_id = msg_ID[1]
            error_bit = msg_ID[2]
            
            # Mark ID as received
            if 0 <= node_id < len(Motor):
                received_ids[node_id] = 1
                latest_commands_id[node_id] = command_id
                # Capture the node's live fault bit HERE, before the command dispatch below sorts
                # payloads into their per-type deques. error_bit rides the CAN ID, so this is
                # valid for every reply from every node — including the gripper in either mode,
                # whose two modes reply with different payloads (gripper status vs DATA_PACK)
                # and so would otherwise only refresh one of two different deques.
                if node_error_bits is not None:
                    node_error_bits[node_id] = error_bit
                if data_reply_count_out is not None and command_id in (3, 5, 7) \
                        and node_id < len(data_reply_count_out):
                    data_reply_count_out[node_id] += 1
                if reply_time_out is not None and command_id in (3, 5, 7) and node_id < len(reply_time_out):
                    reply_time_out[node_id] = message.timestamp
                
                # Unpack data
                Motor[node_id].UnpackData(message, msg_ID)
                
                # Process based on command ID
                Hall_index, additional_pin2, Edge_detect = process_can_message(
                    Motor[node_id], node_id, command_id, error_bit,
                    Motor_data, Gripper_data,
                    Motor_temperature_data, Motor_voltage_data,
                    Motor_errors_data, Motor_info_data,
                    Motor_Kt_data,
                    loop_tick_cnt, Hall_index, additional_pin2, Edge_detect
                )
    
    # [0] = worst age this tick, [1] = best. The SPREAD is what makes the number readable:
    #   min ~= max, both large  -> every frame is stale = a genuine receive backlog
    #   min small, max large    -> most frames are current and one CLASS of frame is old
    #                              (e.g. a round-robin poll reply that only comes every ~21
    #                              ticks), which is not a backlog at all.
    # 0.0 if no frame arrived — a silent bus is not "fresh", but CAN_LOST / old_data_warning
    # already cover that case.
    if frame_age_out is not None:
        frame_age_out[0] = _age_max
        frame_age_out[1] = _age_min if _age_min is not None else 0.0
        if len(frame_age_out) > 2:
            frame_age_out[2] = _now          # when this read ran (telemetry can_read_t, 2026-10-12)

    return Hall_index, additional_pin2, Edge_detect, received_ids, latest_commands_id


# ==========================================================================
# Message Processing by Command ID
# ==========================================================================

def process_can_message(motor_obj, node_id, command_id, error_bit,
                       Motor_data, Gripper_data,
                       Motor_temperature_data, Motor_voltage_data,
                       Motor_errors_data, Motor_info_data,
                       Motor_Kt_data,
                       loop_tick_cnt, Hall_index, additional_pin2, Edge_detect):
    """
    Process a single CAN message based on its command ID.
    
    Args:
        motor_obj: Motor object that received the message
        node_id (int): CAN node ID
        command_id (int): CAN command ID
        error_bit (int): Error bit from message
        Motor_data (dict): Motor data deques
        Gripper_data (dict): Gripper data deques
        Motor_temperature_data (dict): Temperature data deques
        Motor_voltage_data (dict): Voltage data deques
        Motor_errors_data (dict): Error data deques
        Motor_info_data (dict): Info data deques
        loop_tick_cnt (int): Current loop tick
        Hall_index (int): Current Hall sensor index
        additional_pin2 (int): Current state of additional_pin2 (bit 1 of Hall data byte)
    
    Returns:
        tuple: (Hall_index, additional_pin2)
    """
    # Check DLC warning
    if motor_obj.DLC_warrning == 1:
        return Hall_index, additional_pin2, Edge_detect
    
    # DATA_PACK messages (3, 5, 7)
    if command_id in [3, 5, 7]:
        process_motor_data(
            Motor_data, node_id,
            motor_obj.position,
            motor_obj.speed,
            motor_obj.current,
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )
    
    # RESPOND_DATA_HALL (32)
    elif command_id == 32:
        process_motor_data(
            Motor_data, node_id,
            motor_obj.position,
            0, 0,  # Speed and current not in hall message
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )
        Hall_index = motor_obj.HALL_trigger
        additional_pin2 = motor_obj.additional_pin2
        Edge_detect = motor_obj.hall_index if motor_obj.hall_index is not None else 0  # 0=no edge, 1=edge triggered
        #pin_state
    
    # RESPOND_GRIPPER_DATA_PACK (60)
    elif command_id == 60:
        process_gripper_data(
            Motor_data, Gripper_data, node_id,
            motor_obj.gripper_position,
            motor_obj.gripper_current,
            motor_obj.timestamp,
            motor_obj.gripper_activated,
            motor_obj.gripper_action_status,
            motor_obj.gripper_object_detection,
            motor_obj.gripper_temperature_error,
            motor_obj.gripper_timeout_error,
            motor_obj.gripper_estop_error,
            motor_obj.gripper_calibrated,
            command_id,
            loop_tick_cnt,
            error_bit
        )
    
    # SEND_RESPOND_VOLTAGE (24)
    elif command_id == 24:
        process_motor_voltage(
            Motor_voltage_data, node_id,
            motor_obj.voltage,
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )
    
    # SEND_RESPOND_STATE_OF_ERRORS (26)
    elif command_id == 26:
        process_motor_errors(
            Motor_errors_data, node_id,
            motor_obj.error,
            motor_obj.temperature_error,
            motor_obj.encoder_error,
            motor_obj.vbus_error,
            motor_obj.driver_error,
            motor_obj.velocity_error,
            motor_obj.current_error,
            motor_obj.estop_error,
            motor_obj.watchdog_error,
            motor_obj.calibrated,
            motor_obj.activated,
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )
    
    # SEND_RESPOND_TEMPERATURE (23)
    elif command_id == 23:
        process_motor_temperature(
            Motor_temperature_data, node_id,
            motor_obj.temperature,
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )
    
    # SEND_RESPOND_DEVICE_INFO (25)
    elif command_id == 25:
        process_motor_info(
            Motor_info_data, node_id,
            motor_obj.HARDWARE_VERSION,
            motor_obj.BATCH_DATE,
            motor_obj.SOFTWARE_VERSION,
            motor_obj.SERIAL_NUMBER,
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )
    
    # RESPOND_HEARTBEAT (9)
    elif command_id == 9:
        pass  # Heartbeat - no processing needed

    # SEND_RESPOND_KT (33)
    elif command_id == 33:
        process_motor_kt(
            Motor_Kt_data, node_id,
            motor_obj.Kt,
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )

    # SEND_RESPOND_ENCODER_DATA_ID (28)
    elif command_id == 28:
        process_motor_data(
            Motor_data, node_id,
            motor_obj.position,
            motor_obj.speed,
            0, # Current not in this message
            motor_obj.timestamp,
            command_id,
            loop_tick_cnt,
            error_bit
        )

    return Hall_index, additional_pin2, Edge_detect


# ==========================================================================
# Startup Kt fetch
# ==========================================================================

_KT_RESPOND_CMD_ID = 33  # SEND_RESPOND_KT


def _format_can_data(message):
    return " ".join(f"{byte:02X}" for byte in message.data)


def _drain_pending_can_messages(communication, max_messages=64):
    """Drop stale frames before a startup request/response transaction."""
    drained = 0
    while drained < max_messages:
        message, msg_ID = communication.receive_can_messages(timeout=0)
        if message is None or msg_ID is None:
            break
        drained += 1
    if drained:
        logger.info(f"[Startup][Kt] Drained {drained} stale CAN frame(s)")


def _read_kt_for_node(motors, communication, node_id, timeout_s, retries):
    """Request one node's Kt and wait for its matching command-33 response."""
    for attempt in range(1, retries + 1):
        _drain_pending_can_messages(communication)
        motors[node_id].Send_Respond_Kt()
        logger.info(
            f"[Startup][Kt] Requested node {node_id} Kt "
            f"(attempt {attempt}/{retries})"
        )

        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            remaining = max(0.0, deadline - time.perf_counter())
            message, msg_ID = communication.receive_can_messages(
                timeout=min(0.05, remaining)
            )
            if message is None or msg_ID is None:
                continue

            rx_node_id = msg_ID[0]
            command_id = msg_ID[1]
            error_bit = msg_ID[2]

            if command_id == _KT_RESPOND_CMD_ID:
                logger.info(
                    f"[Startup][Kt] RX id=0x{message.arbitration_id:03X} "
                    f"node={rx_node_id} cmd={command_id} err={error_bit} "
                    f"dlc={message.dlc} data=[{_format_can_data(message)}]"
                )

            if rx_node_id != node_id or command_id != _KT_RESPOND_CMD_ID:
                continue

            motors[rx_node_id].UnpackData(message, msg_ID)
            kt_can = motors[rx_node_id].Kt
            if kt_can is None or kt_can <= 0:
                logger.warning(
                    f"[Startup][Kt] Node {node_id} returned invalid Kt={kt_can}; "
                    "keeping fallback value"
                )
                continue

            return kt_can

    return None


def fetch_kt_from_can(motors, communication, n_joints, kt_xml, timeout_s=0.35,
                      gripper_idx=None, gripper_kt_xml=None, retries=3,
                      rounds=2, settle_s=0.2):
    """
    Request Kt from every arm motor over CAN and collect the responses.
    Optionally also fetches Kt from a CAN gripper motor.

    Requests one motor at a time, waits for that node's command-33 response,
    then moves to the next node. Missing nodes get another sweep after a short
    settle delay. Any motor that does not reply keeps its XML fallback value.

    Parameters
    ----------
    motors          : list[SpectralBLDC]  — Motor objects (indexed by node_id)
    communication   : CanCommunication    — CAN bus object
    n_joints        : int                 — number of arm joints to poll
    kt_xml          : np.ndarray          — arm Kt values from XML (fallback)
    timeout_s       : float               — wait time per attempt in seconds
    gripper_idx     : int or None         — motor index of the CAN gripper
                                            (typically joint_num); None = no gripper
    gripper_kt_xml  : float or None       — gripper Kt fallback from XML
    retries         : int                 — number of request attempts per node
    rounds          : int                 — number of missing-node sweeps
    settle_s        : float               — delay before retrying missing nodes

    Returns
    -------
    kt_final        : np.ndarray  — resolved arm Kt per joint
    kt_sources      : list[str]   — 'CAN' or 'XML' for each arm joint
    gripper_kt      : float       — resolved gripper Kt (unchanged if no gripper)
    gripper_kt_src  : str         — 'CAN' or 'XML' for the gripper
    """
    kt_final = np.array(kt_xml, dtype=float)
    kt_sources = ['XML'] * n_joints

    gripper_kt = float(gripper_kt_xml) if gripper_kt_xml is not None else 0.0
    gripper_kt_src = 'XML'

    pending_nodes = list(range(n_joints))
    for fetch_round in range(1, rounds + 1):
        if not pending_nodes:
            break

        if fetch_round > 1:
            logger.info(
                f"[Startup][Kt] Retrying missing node(s) {pending_nodes} "
                f"after {settle_s:.3f}s settle delay "
                f"(round {fetch_round}/{rounds})"
            )
            time.sleep(settle_s)

        still_missing = []
        for i in pending_nodes:
            kt_can = _read_kt_for_node(motors, communication, i, timeout_s, retries)
            if kt_can is not None:
                kt_final[i] = kt_can
                kt_sources[i] = 'CAN'
            else:
                still_missing.append(i)
        pending_nodes = still_missing

    if pending_nodes:
        logger.warning(
            f"[Startup][Kt] No CAN Kt response from node(s) {pending_nodes}; "
            "using XML fallback for those joints"
        )

    if gripper_idx is not None:
        for fetch_round in range(1, rounds + 1):
            if fetch_round > 1:
                logger.info(
                    f"[Startup][Kt] Retrying gripper node {gripper_idx} "
                    f"after {settle_s:.3f}s settle delay "
                    f"(round {fetch_round}/{rounds})"
                )
                time.sleep(settle_s)

            kt_can = _read_kt_for_node(
                motors, communication, gripper_idx, timeout_s, retries
            )
            if kt_can is not None:
                gripper_kt = kt_can
                gripper_kt_src = 'CAN'
                break

    return kt_final, kt_sources, gripper_kt, gripper_kt_src


# ==========================================================================
# Data Freshness Checking
# ==========================================================================

def check_data_freshness(Motor_data, joint_num, active_CAN_gripper, loop_tick_cnt,
                         old_data_warning, CAN_node_connection_error,
                         old_data_threshold=20, disconnect_threshold=50):
    """
    Check if motor data is fresh or stale.

    Args:
        Motor_data (dict): Motor data deques
        joint_num (int): Number of joints
        active_CAN_gripper (int): 1 if gripper active
        loop_tick_cnt (int): Current loop tick
        old_data_warning (np.array): Warning flags array
        CAN_node_connection_error (np.array): Connection error array
        old_data_threshold (int): Ticks before data considered old (default: 20)
        disconnect_threshold (int): Ticks before considering disconnected (default: 50)

    Returns:
        tuple: (old_data_warning, CAN_node_connection_error)
    """
    # Get latest tick for each motor
    loop_tick_latest = [Motor_data[i][-1]["tick"] for i in range(len(Motor_data))]

    # Check each motor
    for i in range(joint_num + active_CAN_gripper):
        tick_diff = loop_tick_cnt - loop_tick_latest[i]

        # Old data warning
        if tick_diff >= old_data_threshold:
            old_data_warning[i] = 1
        else:
            old_data_warning[i] = 0

        # Disconnection error
        if tick_diff >= disconnect_threshold:
            CAN_node_connection_error[i] = 1
        else:
            CAN_node_connection_error[i] = 0

    return old_data_warning, CAN_node_connection_error


# ==========================================================================
# Gripper Connection Check
# ==========================================================================

if __name__ == "__main__":
    """Test CAN message handlers"""
    print("CAN Message Handlers Test")
    print("-" * 50)
    
    print("\nThis module processes CAN messages in the main RTI loop.")
    print("It handles:")
    print("  - Motor position/speed/current data")
    print("  - Gripper data")
    print("  - Temperature readings")
    print("  - Voltage readings")
    print("  - Error states")
    print("  - Device information")
    print("\nNo standalone test available - requires CAN hardware and motor objects.")
