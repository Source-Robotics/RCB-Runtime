#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: RTI.py
Author: Petar Crnjak
Company: Source Robotics
Date: 13.5.2025
Version: 1.0
Description: 
    This script demonstrates a detailed 
    header format with additional metadata.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
Dependencies: os, sys
"""

import time
import signal
import atexit
from collections import deque
from utility.RTI_utils import *
from project_paths import RTI_LOG, SYSTEM_XML, ROBOTS_DIR, JOBLIB_DATA, TEMP_DATA_DIR, DATA_RECIPE_DIR, GRIPPERS_DIR, CRITICAL_LOG, CONFIG_DIR, FLASH_PERFORMED_MARKER
import logging
import sys
import numpy as np
from utility.robot_factory import create_robot
import threading
import zmq
from queue import Queue, Empty
from spatialmath import *
from gpiozero import pi_info
from utility.logger_setup import setup_logging
import config.xml_parser as read_XML2
from utility.selfcheck import run_selfcheck
from utility.state_machine import RobotMode, request_mode_change, request_enable, MAINTENANCE_MODES
from utility.error_checks import (
    process_errors_tick, clear_errors, send_clear_errors, ExecLinkWatchdog,
    LoopDegradationMonitor,
    CLEAR_ERROR_SETTLE_TICKS, CLEAR_ERROR_SEND_REPEATS, _WARNING_SUFFIXES,
)
from utility.activity_log import get_rti_activity_logger, log_activity
from robotics.homing import HomingController
from motion.rti_handler import RTIModeHandler
from communication.rti_protocol import CLOCK_ROBOT, CLOCK_PC

# ==========================================================================
# System and Configuration
# ==========================================================================
from config.system_config import setup_realtime_process
from hardware.can_hardware import start_can_interface
#from config.network_config import initialize_all_sockets

# ==========================================================================
# Hardware
# ==========================================================================
from hardware.gpio_handler import (
    initialize_gpio, update_gpio, check_estop_active, write_all_outputs, read_all_inputs,
    set_output, set_output_high, set_output_low, toggle_output, HIGH, LOW,
    ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3
)
from hardware.motor_setup import setup_motor_system
from hardware.can_hardware import scan_can_bus, initialize_can_communication
# init_rti_receive_socket is now an alias inside config/networking.py

# ==========================================================================
# Networking
# ==========================================================================
from config.networking import zmq_listener, zmq_sender_setup, UDP_socket_receive_setup, UDP_socket_send_setup, broadcast_UDP_telemetry, init_rti_receive_socket, drain_rti_udp, RTI_PORT_SENDER
from communication.can_message_handlers import receive_and_process_can_messages, handle_polling_and_overrides, update_can_connection_status, drain_can_discard
from communication.can_message_handlers import send_gripper_commands, send_motor_commands, secondary_data_send, load_motor_configs, load_gripper_configs, handle_motor_reconnect, write_motor_config_to_shared, write_gripper_config_to_shared
from utility.kt_init import resolve_and_preload_kt
from utility.tick_types import TickContext, TickResult
from utility.mode_dispatch import build_mode_dispatch, dispatch_step
from utility.kinematics_backend import set_kinematics_backend
from utility.pinocchio_attach import attach_pinocchio

# ==========================================================================
# Shared data setup — joint count driven by active robot in config/system.xml
# NOTE: load_system_config is a lightweight parse (two small XMLs only).
#       It runs before shared_data.setup() so the joint count is always correct.
# ==========================================================================
_sys = read_XML2.load_system_config(SYSTEM_XML, ROBOTS_DIR)
from data import shared_data
shared_data.setup(joints=_sys['joint_num'])
from data.update_shared_data import update_motor_and_gripper_data, update_joint_and_tcp_data
from utility.filter_config import load_filter_banks
from utility.signal_filters import describe_bank
from data.data_recipe import RecipeManager, build_payload

# ==========================================================================
# System configuration (RT scheduling, CPU affinity, system info)
#
# NOTE: setup_realtime_process used to run here, BEFORE setup_logging()
# attached the file handler — so its info/warning messages disappeared.
# It's been moved to right after setup_logging() (just below this block);
# keeping the section header so the call site is easy to find by greppers
# accustomed to the old layout.
# ==========================================================================

# ==========================================================================
# Heartbeat terminal debug output
# Enable:   RTI_HEARTBEAT=1 python RTI.py
# Interval: RTI_HEARTBEAT_INTERVAL=200 (ticks between prints, default 100)
# ==========================================================================
_HB_ENABLED  = os.environ.get("RTI_HEARTBEAT", "0") == "1" # TRUE
_HB_INTERVAL = int(os.environ.get("RTI_HEARTBEAT_INTERVAL", "100"))

# ==========================================================================
# Optional RTI section profiler
# Enable:   RTI_PROFILE=1 python3 RTI.py
# Interval: RTI_PROFILE_INTERVAL=500 (ticks per report, default 500)
# ==========================================================================
_PROFILE_ENABLED  = os.environ.get("RTI_PROFILE", "0") == "1"
_PROFILE_INTERVAL = int(os.environ.get("RTI_PROFILE_INTERVAL", "500"))
_PROFILE_TOP      = int(os.environ.get("RTI_PROFILE_TOP", "12"))
_profile_sum      = {}
_profile_max      = {}
_profile_count    = 0
_profile_loop_sum = 0.0

# ==========================================================================
# Overrun tracer — names the section responsible for a slow tick.
# Enable:    RTI_OVERRUN_TRACE=1 python3 RTI.py
# Threshold: RTI_OVERRUN_TRACE_MS=4.0   (default: the tick budget, delta_t)
#
# RTI_PROFILE answers "where does the AVERAGE tick go" — it averages over 500
# ticks, so a single 7 ms spike is smeared away and invisible. This answers the
# other question: "what blew up on THAT tick". It keeps the current tick's
# per-section times and dumps them, sorted, only when the tick exceeds the
# threshold. Rare by construction, so the log stays readable.
#
# Overhead when enabled: the same ~22 perf_counter calls RTI_PROFILE already
# does — 3.06 us/tick, 0.077% of a 4 ms tick. Zero when disabled (the mark
# function returns immediately). Writes into a preallocated dict, so no
# allocation once the section names are known.
# ==========================================================================
_OVERRUN_TRACE_ENABLED = os.environ.get("RTI_OVERRUN_TRACE", "0") == "1"
_OVERRUN_TRACE_MS = float(os.environ.get("RTI_OVERRUN_TRACE_MS", "0")) / 1e3  # 0 => use delta_t
_OVERRUN_TRACE_TOP = int(os.environ.get("RTI_OVERRUN_TRACE_TOP", "6"))
_tick_sections = {}      # section -> seconds spent THIS tick (overwritten each tick)
_overrun_trace_count = 0

# Marks must run if EITHER consumer is on.
_MARKS_ON = _PROFILE_ENABLED or _OVERRUN_TRACE_ENABLED


def _profile_mark(last_time, name):
    if not _MARKS_ON:
        return last_time
    now = time.perf_counter()
    dt = now - last_time
    if _PROFILE_ENABLED:
        _profile_sum[name] = _profile_sum.get(name, 0.0) + dt
        if dt > _profile_max.get(name, 0.0):
            _profile_max[name] = dt
    if _OVERRUN_TRACE_ENABLED:
        _tick_sections[name] = dt
    return now


def _overrun_trace(loop_total, tick, mode):
    """Log the per-section breakdown of a tick that blew its budget.

    Reports the UNACCOUNTED remainder too: if the sections sum to far less than the
    tick took, the cost is in an unprofiled gap between marks, which is itself the
    finding (it says "add a mark here", not "this section is slow").
    """
    global _overrun_trace_count
    if not _OVERRUN_TRACE_ENABLED or not _tick_sections:
        return
    limit = _OVERRUN_TRACE_MS if _OVERRUN_TRACE_MS > 0 else delta_t
    if loop_total <= limit:
        return
    _overrun_trace_count += 1
    ranked = sorted(_tick_sections.items(), key=lambda kv: kv[1], reverse=True)
    accounted = sum(_tick_sections.values())
    top = "  ".join(f"{n}={v * 1e3:.2f}ms" for n, v in ranked[:_OVERRUN_TRACE_TOP] if v > 0)
    logger.warning(
        f"[OVERRUN #{_overrun_trace_count}] tick={int(tick)} mode={mode} "
        f"exec={loop_total * 1e3:.3f}ms (limit {limit * 1e3:.2f}ms) | "
        f"worst: {top} | unaccounted={(loop_total - accounted) * 1e3:.2f}ms"
    )


def _profile_report(loop_total):
    global _profile_count, _profile_loop_sum
    if not _PROFILE_ENABLED:
        return

    _profile_count += 1
    _profile_loop_sum += loop_total
    if _profile_count < _PROFILE_INTERVAL:
        return

    rows = []
    for name, total in _profile_sum.items():
        avg_ms = (total / _profile_count) * 1e3
        max_ms = _profile_max.get(name, 0.0) * 1e3
        pct = (total / _profile_loop_sum * 100.0) if _profile_loop_sum > 0.0 else 0.0
        rows.append((total, name, avg_ms, max_ms, pct))
    rows.sort(reverse=True)

    sys.stdout.write(
        f"\n[RTI_PROFILE] last {_profile_count} ticks "
        f"(avg exec {(_profile_loop_sum / _profile_count) * 1e3:.3f} ms)\n"
    )
    for _total, name, avg_ms, max_ms, pct in rows[:_PROFILE_TOP]:
        sys.stdout.write(
            f"  {name:<24s} avg={avg_ms:8.3f} ms  "
            f"max={max_ms:8.3f} ms  share={pct:5.1f}%\n"
        )
    sys.stdout.flush()

    _profile_sum.clear()
    _profile_max.clear()
    _profile_count = 0
    _profile_loop_sum = 0.0

# ==========================================================================
# Init gpio
# ==========================================================================
initialize_gpio()

# ==========================================================================
# Setup logging
# ==========================================================================
# Logging if TRUE print to terminal and save to log file, If FALSE save to log file - only for this file
setup_logging(str(RTI_LOG), log_to_console=True, critical_log_file=str(CRITICAL_LOG))

# Activity log — separate mechanism from the above (see TODO/activity_log_design.md).
_rti_activity_logger = get_rti_activity_logger()

# RT scheduling + CPU affinity. Runs AFTER setup_logging so the verified
# RT-state line lands in RTI.log instead of disappearing.
setup_realtime_process(cpu_core=3, priority=99)
# When a script crashes with exception we write the error to log file
def handle_exception(exc_type, exc_value, exc_traceback):
    # Ignore keyboard interrupt
    # if issubclass(exc_type, KeyboardInterrupt):
    #    sys.__excepthook__(exc_type, exc_value, exc_traceback)
    #    return
    logging.critical("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))
sys.excepthook = handle_exception

# ==========================================================================
# Setup print format
# ==========================================================================
np.set_printoptions(suppress=True, precision=7, floatmode='fixed')

# ==========================================================================
# Setup networking
# ==========================================================================
# NOTE: Do NOT use ephemeral ports: 49152 to 65535
# u sudo ss -tunap to check all ports currently in use
robot_IP = get_local_ip()

# --- Create and start the thread
command_queue = Queue()
threading.Thread(target=zmq_listener, args=(command_queue,), daemon=True).start()

# --- Setup ZMQ response sender
status_socket = zmq_sender_setup()

# --- RTI socket receive ---
RTI_socket_receive = UDP_socket_receive_setup()

# --- RTI socket sender: RTI and telemetry---
RTI_socket_sender, telemetry_socket = UDP_socket_send_setup()

# ==========================================================================
# Get robot data from XML
# ==========================================================================
xml_path    = str(_sys['robot_xml_path'])
output_path = str(_sys['robot_xml_path'].parent / f"{_sys['active_robot']}_modified.xml")
joblib_path = str(JOBLIB_DATA)

# Load all robot/gripper parameters in one call
cfg = read_XML2.load_full_robot_config(xml_path, grippers_dir=GRIPPERS_DIR, active_gripper=_sys['active_gripper'])

# Validate XML config — abort immediately if any required field is broken
_cfg_ok, _cfg_issues = read_XML2.validate_robot_config(cfg)
if not _cfg_ok:
    for _msg in _cfg_issues:
        logging.critical(f"[Startup] {_msg}")
    raise SystemExit("RTI.py cannot start: robot XML has errors (see log above).")

robot_name            = cfg['robot_name']
joint_num             = cfg['joint_num']
motor_kt              = cfg['motor_kt']
motor_Ilim            = cfg['motor_Ilim']
motor_voltage_limit   = cfg['motor_voltage_limit']   # mV, per joint (0 = use VBUS)
motor_velocity_limit  = cfg['motor_velocity_limit']
motor_KPP             = cfg['motor_KPP']
motor_KPV             = cfg['motor_KPV']
motor_KIV             = cfg['motor_KIV']
motor_KIIQ            = cfg['motor_KIIQ']
motor_KPIQ            = cfg['motor_KPIQ']
motor_KP              = cfg['motor_KP']
motor_KD              = cfg['motor_KD']
motor_encoder         = cfg['motor_encoder']
joint_limit_negative  = cfg['joint_limit_negative']
joint_limit_positive  = cfg['joint_limit_positive']
master_position       = cfg['master_position']
gear_ratio            = cfg['gear_ratio']
gear_efficiency       = cfg['gear_efficiency']
motor_offset          = cfg['motor_offset']
motor_dir             = cfg['motor_dir']
motor_signs           = 1 - 2 * motor_dir  # 0→1, 1→−1

gripper_name          = cfg['gripper_name']
gripper_stroke        = cfg['gripper_stroke']
gripper_kt            = cfg['gripper_kt']
gripper_gear_r        = cfg['gripper_gear_r']
gripper_Ilim          = cfg['gripper_Ilim']
gripper_voltage_limit = cfg['gripper_voltage_limit']   # mV (0 = use VBUS)
gripper_velocity_limit = cfg['gripper_velocity_limit']
gripper_KPP           = cfg['gripper_KPP']
gripper_KPV           = cfg['gripper_KPV']
gripper_KIV           = cfg['gripper_KIV']
gripper_KIIQ          = cfg['gripper_KIIQ']
gripper_KPIQ          = cfg['gripper_KPIQ']
gripper_KP            = cfg['gripper_KP']
gripper_KD            = cfg['gripper_KD']

delta_t               = cfg['delta_t']
delta_t_RTI_joint     = cfg['delta_t_RTI_joint']
safety_cfg            = cfg['safety']

# Build homing controller from XML config
homing_ctrl = HomingController.from_config(cfg, joint_num)

# Raw XML objects kept for set_single_value / save_robot_config calls
_root_xml    = cfg['_root']
_joints_xml  = cfg['_joints']
_gripper_xml = cfg['_gripper_elem']

# ==========================================================================
# Setup Gripper
# ==========================================================================

name_of_the_gripper = cfg['gripper_name']  # read from system.xml -> grippers/<name>.xml
_can_raw = cfg['gripper_CAN_gripper']
active_CAN_gripper = 0 if (isinstance(_can_raw, float) and np.isnan(_can_raw)) else int(_can_raw)

# Write gripper control mode to shared memory (0=no gripper, 1=motor, 2=firmware)
_gripper_ctrl_mode = 0
if active_CAN_gripper == 1:
    _gripper_ctrl_mode = 1  # default: motor mode
    for _step in cfg.get('homing_sequence', []):
        if _step.get('gripper_mode') is not None:
            _gripper_ctrl_mode = 2 if _step['gripper_mode'] == 'firmware' else 1
            break
shared_data.rti['gripper_ctrl_mode'][0] = _gripper_ctrl_mode
if active_CAN_gripper:
    shared_data.gripper['gripper_gear_r'][0] = gripper_gear_r

# Firmware-mode gripper command state — updated by ZMQ "gripper_set:" commands
# from the GUI. Sent every tick by send_gripper_commands when
# gripper_ctrl_mode == 2 (activate is forced to 1 inside the sender).
# position/speed are 0–255, current is mA, action is 0/1.
_gripper_fw_state = {
    'position': 0, 'speed': 100, 'current': 200,
    'action': 0,
}
# Calibrate state machine. After Send_gripper_calibrate fires, the bus must be
# held with *empty* Send_gripper_data_pack() polls — resuming the full data
# pack mid-calibration cancels it. We don't try to auto-detect completion
# The user interrupts the state by clicking Open/Close/Goto/Stop, which aborts the calibration via the
# gripper_set handler. The hard timeout is a safety backstop only.
_gripper_calibrate_pending = False
_gripper_calibrate_running = False
_gripper_calibrate_ticks   = 0
_GRIPPER_CALIBRATE_MAX_TICKS = max(1, round(30.0 / delta_t))  # safety timeout


# ==========================================================================
# Setup robot
# ==========================================================================
robot = create_robot(robot_name, gripper_name=name_of_the_gripper)

_kinematics_backend = os.environ.get("RTI_KINEMATICS_BACKEND", "auto").strip().lower()
set_kinematics_backend(_kinematics_backend)
if _kinematics_backend == "rtb":
    logging.info("[Startup] Kinematics backend: robotics-toolbox")
else:
    try:
        attach_pinocchio(robot)
        logging.info(f"[Startup] Kinematics backend: {_kinematics_backend} (Pinocchio attached)")
    except Exception:
        if _kinematics_backend == "pinocchio":
            raise
        logging.exception(
            "[Startup] Pinocchio attach failed; falling back to robotics-toolbox kinematics"
        )

# ==========================================================================

# ==========================================================================
# Start / Restart CAN interface when starting this script
# ==========================================================================
start_can_interface()
communication1 = initialize_can_communication(bustype='socketcan', channel='can0', bitrate=1000000)

# ==========================================================================
# Motor objects, joint objects, filters, housekeeping — via motor_setup.py
# ==========================================================================
xml_config = {
    'motor_encoder':  motor_encoder,
    'master_position': master_position,
    'gear_ratio':     gear_ratio,
    'motor_offset':   motor_offset,
    'motor_dir':      motor_dir,
}
motor_system = setup_motor_system(joint_num, communication1, xml_config)

Motor        = motor_system['Motor']
Joint        = motor_system['Joint']
dummy_device = motor_system['dummy_device']

# Selectable per-signal, per-joint filters (config/filters.xml, hot-reloadable). Replaces the
# old hardcoded EMAs (joint_speed_filtered/alpha_Speed AND the TCP-accel EMA): joint_speed is now
# RAW with joint_speed_filtered from filter_banks['speed'], and Cartesian acceleration is filtered
# by filter_banks['tcp_accel']. The old motor_system['filters'] EMA-state arrays are left unused.
FILTER_CONFIG_PATH = CONFIG_DIR / "filters.xml"
filter_banks, _filter_src = load_filter_banks(FILTER_CONFIG_PATH, joint_num, delta_t)
logger.info(f"[filters] loaded from {_filter_src}: " +
            ", ".join(f"{s}={describe_bank(b)}" for s, b in filter_banks.items()))

received_ids       = motor_system['housekeeping']['received_ids']
latest_commands_id = motor_system['housekeeping']['latest_commands_id']
node_error_bits    = motor_system['housekeeping']['node_error_bits']

motor_error_bytes  = motor_system['control']['motor_error_bytes']
startup_selfcheck  = motor_system['housekeeping']['startup_selfcheck']
loop_execution_time  = motor_system['housekeeping']['loop_execution_time']
command_success_rate = motor_system['housekeeping']['command_success_rate']


# Zero out shared memory housekeeping arrays
shared_data.rti["scan_received_ids"][:]         = np.zeros(16, dtype=int)
shared_data.rti["CAN_node_connection_error"][:] = np.zeros(16, dtype=int)
shared_data.rti["old_data_warning"][:]          = np.zeros(16, dtype=int)
shared_data.rti["homing_status"][:]             = 0
shared_data.rti["joint_homed"][:]               = 0
shared_data.rti["gravity_comp_enabled"][0]      = 1.0

# ==========================================================================
# Data queue. Allows us to save previous x points of data for safety, filtering, 
# ==========================================================================
# -1 index in deque is LATEST item and 0 is the OLDEST stored
# range(16) because our CAN bus can handle 16 unique CAN nodes (0 - 15)
# This one is critical data array that if not filled properly will cause errors
# Initialize all structures at once
robot_init_data = initialize_robot_data(joint_num)

Motor_data             = robot_init_data["Motor_data"]
Gripper_data           = robot_init_data["Gripper_data"]
Motor_temperature_data = robot_init_data["Motor_temperature_data"]
Motor_voltage_data     = robot_init_data["Motor_voltage_data"]
Motor_errors_data      = robot_init_data["Motor_errors_data"]
Motor_info_data        = robot_init_data["Motor_info_data"]
Motor_Kt_data          = robot_init_data["Motor_Kt_data"]
Joint_data             = robot_init_data["Joint_data"]
System_data            = robot_init_data["System_data"]


scheduled_state = {"step": 0,"joint_index": 0,"device_info_counter": 0}
# Optional override function (set externally when needed)
scheduled_override_queue = deque()  # Each item is a tuple: (function, repeat_count)
current_override = {"fn": None,"count": 0}


# ==========================================================================
# Preload motor driver configs. NOTE this needs to be preloaded after
# Every fatal error or disconnect because motor drivers might lose this data
# Same goes for gripper configs if we have CAN gripper
# ==========================================================================
motor_watchdog_rate   = cfg['motor_watchdog_rate']
gripper_watchdog_rate = int(cfg['gripper_watchdog_rate']) if active_CAN_gripper else 5000

# ==========================================================================
# Helpers that bundle load + write_to_shared for arm and gripper configs.
# Called at startup, at the tick-50/150/300 reconfig, and from the
# clear_errors / set_pid_gains command handlers.
# num_repeats DEFAULT stays 4 solely for the tick-50/150/300 reconfig block,
# which calls with no arguments and must not change (dirty fix for the
# exec-time-doubling bug). Every other caller passes num_repeats=2 explicitly.
# ==========================================================================
def _reload_motor_config(num_repeats=4, motor_index=None, pace_s=0.0):
    load_motor_configs(Motor,
        joint_num=joint_num, watchdog_rate=motor_watchdog_rate,
        velocity_limit=motor_velocity_limit, current_limit=motor_Ilim,
        KP=motor_KP, KD=motor_KD, KPIQ=motor_KPIQ, KIIQ=motor_KIIQ,
        KPV=motor_KPV, KIV=motor_KIV, KPP=motor_KPP,
        watchdog_action="Idle", num_repeats=num_repeats, motor_index=motor_index,
        voltage_limit=motor_voltage_limit, pace_s=pace_s)
    write_motor_config_to_shared(
        shared_data, joint_num,
        KPP=motor_KPP, KPV=motor_KPV, KIV=motor_KIV,
        KPIQ=motor_KPIQ, KIIQ=motor_KIIQ, KP=motor_KP, KD=motor_KD,
        Ilim=motor_Ilim, velocity_limit=motor_velocity_limit,
        voltage_limit=motor_voltage_limit)

def _reload_gripper_config(num_repeats=4, pace_s=0.0):
    if not active_CAN_gripper:
        return
    load_gripper_configs(Motor,
        joint_num=joint_num, watchdog_rate=gripper_watchdog_rate,
        velocity_limit=gripper_velocity_limit, current_limit=gripper_Ilim,
        KP=gripper_KP, KD=gripper_KD, KPIQ=gripper_KPIQ, KIIQ=gripper_KIIQ,
        KPV=gripper_KPV, KIV=gripper_KIV, KPP=gripper_KPP,
        watchdog_action="Idle", num_repeats=num_repeats,
        voltage_limit=gripper_voltage_limit, pace_s=pace_s)
    write_gripper_config_to_shared(
        shared_data,
        KPP=gripper_KPP, KPV=gripper_KPV, KIV=gripper_KIV,
        KPIQ=gripper_KPIQ, KIIQ=gripper_KIIQ, KP=gripper_KP, KD=gripper_KD,
        Ilim=gripper_Ilim, velocity_limit=gripper_velocity_limit,
        voltage_limit=gripper_voltage_limit)

# Boot-only TX pacing. The CAN library swallows send failures, so overflowing
# the socketcan TX queue (~270 frames of skb budget) silently DROPS frames.
# The boot blast is ~196 back-to-back frames — arm (168) then gripper (28) —
# enqueued in microseconds against a bus that drains ~10 frames/ms, and the
# gripper's watchdog arming rides the tail: exactly the frames that die when
# the queue tips over. 0.5 ms per message-type loop bounds the queue at ~a
# dozen frames for ~16 ms of one-time pre-loop boot cost. The in-loop reloads
# (reconnect, tick-50/150/300, PID apply) stay at pace_s=0 on purpose: a sleep
# there blows the 4 ms tick, their bursts are ≤28 frames (cannot overflow),
# and the full re-send passes are idempotent — a dropped tail cannot
# un-configure an already-armed node.
BOOT_CONFIG_PACE_S = 0.0005

_reload_motor_config(num_repeats=2, pace_s=BOOT_CONFIG_PACE_S)

# ==========================================================================
# Preload gripper data (If CAN gripper is active)
# ==========================================================================
_reload_gripper_config(num_repeats=2, pace_s=BOOT_CONFIG_PACE_S)

# ==========================================================================
# Kt fetch from CAN bus (or fall back to XML values)
# ==========================================================================
motor_kt, gripper_kt = resolve_and_preload_kt(
    _sys, Motor, communication1, joint_num,
    motor_kt, gripper_kt, active_CAN_gripper,
    Motor_Kt_data, shared_data,
)

# ==========================================================================
# Initialize the timer before the loop starts
# ==========================================================================
prev_time = time.perf_counter()
shared_data.rti["loop_tick"][0] = 0
current_mode = RobotMode.BOOTING
pending_mode_request = None
# Set alongside pending_mode_request by "set_mode:<MODE>:PARKED|FORCE". Only
# maintenance modes (FLASHING) read these. There is no automatic test for "can
# this pose survive losing torque", so entry requires an explicit operator
# assertion; the assertion string is kept only so the log says which was made.
pending_mode_force = False
pending_mode_assertion = ""
shared_data.robot_mode.set(current_mode)
shared_data.robot_state.set("DISABLED")
shared_data.homed_state.set("NOT HOMED")
# Consume any flash marker left by a RUNTIME-DOWN flash. Starting RTI already
# clears homing (the line above), so the marker has done its job — leaving it on
# disk would fire on the next FLASHING exit and cost a re-home that nothing
# earned. Best-effort: a marker we cannot delete is a spurious re-home later,
# never a missed one.
try:
    FLASH_PERFORMED_MARKER.unlink(missing_ok=True)
except OSError:
    pass
shared_data.error_state.set_json("NONE")
shared_data.rti["error_active"][0] = 0
shared_data.rti["safety_violation"][0] = 0
shared_data.rti["software_estop"][0] = 0
# RTI-mode telemetry. These are only written while RobotMode.RTI is active, and shared
# memory survives an RTI.py restart — so without this, whatever the last session (or a
# test harness) left behind would sit in the GUI looking live, forever. Zeroed here so a
# fresh process always starts from "no PC paired".
shared_data.rti["rti_substate"][0] = 0
shared_data.rti["control_success_rate"][0] = 0
shared_data.rti["rti_consecutive_misses"][0] = 0
shared_data.rti["rti_worst_consecutive_misses"][0] = 0
shared_data.rti["rti_tx_bytes_per_s"][0] = 0
shared_data.rti["rti_rx_bytes_per_s"][0] = 0
shared_data.rti["rti_seq_rx_last"][0] = 0
shared_data.rti["rti_discard_pct"][0] = 0
startup_selfcheck = 0 # Flag to run startup selfcheck only once at startup
startup_motor_reconfig = 0 # Counts reconfiguration passes (0→1→2→3 = done)

# ==========================================================================
# Graceful shutdown — cleanup shared memory on exit / SIGTERM
# ==========================================================================
atexit.register(shared_data.cleanup)

def _handle_sigterm(_signum, _frame):
    logging.info("[RTI] SIGTERM received — shutting down gracefully.")
    shared_data.cleanup()
    sys.exit(0)

signal.signal(signal.SIGTERM, _handle_sigterm)

# ==========================================================================
# Error tracking state — latched faults, metadata, clear countdown
# ==========================================================================

# Holds all errors that have appeared since the last user-requested clear.
# Errors only leave this set when the user calls "Clear Errors" from the GUI
# or sends the "clear_errors" ZMQ command.
latched_errors: set = set()
error_metadata: dict = {}       # key → {"timestamp": float, "tick": int, "description": str, "level": str}
clear_error_countdown: int = 0  # counts down to 0 each time a clear is requested

# EXEC control-link watchdog (gui_executor_migration_plan.md Phase 5): trip if
# command_executor's heartbeat stops advancing while EXEC motion is in flight.
# Timeout must clear the ~20 ms heartbeat period plus scheduling jitter by a wide
# margin (a false trip disables a working arm), yet stay well under operator-
# confusion time. 0.5 s ≈ 25 missed heartbeats = unambiguously dead.
EXEC_LINK_TIMEOUT_S = 0.5
_exec_link_watchdog = ExecLinkWatchdog(max(1, int(round(EXEC_LINK_TIMEOUT_S / delta_t))))

# Rolling p95/p99 loop-timing window (see utility/signal_processing.py
# update_rolling_percentiles) — preallocated once, mutated in place every tick,
# so this stays a zero-allocation hot-path structure like everything else here.
LOOP_STATS_BUFFER_SIZE = 500   # ~2s of history at 250Hz (PAR6), ~5s at 100Hz
LOOP_STATS_INTERVAL = 50       # recompute p95/p99 every 50 ticks (~10us when it fires)
# Tick at which the "since start" peaks are auto-wiped a second time, so they
# describe STEADY STATE rather than boot. Must land after the last boot
# transient: startup_selfcheck (tick 8) and the multi-shot reconfig at ticks
# 50/150/300, whose 4 passes x 168 frames are the biggest bursts RTI ever sends.
# 350 clears tick 300 with ~50 ticks of slack. The discarded peaks are logged
# once before the wipe, so a pathological boot is still recorded in RTI.log.
LOOP_STATS_WARMUP_TICKS = 350
# [0]=worst, [1]=best CAN frame age this tick. Preallocated: the RX loop writes into it every
# tick, so it must not allocate.
_can_age_buf = np.zeros(2, dtype=np.float64)
_loop_period_buf = np.zeros(LOOP_STATS_BUFFER_SIZE, dtype=np.float64)
_loop_exec_buf = np.zeros(LOOP_STATS_BUFFER_SIZE, dtype=np.float64)
_loop_stats_buf_pos = 0
_loop_stats_buf_count = 0

# RT loop health monitor. Warmup is derived, not guessed: the peaks are wiped at
# LOOP_STATS_WARMUP_TICKS, and the rolling window then needs LOOP_STATS_BUFFER_SIZE
# more ticks to refill with post-boot samples before p99 describes anything real.
# Arming any earlier would trip on the boot transients the wipe exists to discard.
_loop_monitor = LoopDegradationMonitor(
    delta_t=delta_t,
    warmup_ticks=LOOP_STATS_WARMUP_TICKS + LOOP_STATS_BUFFER_SIZE,
)

# Per-node old_data_warning state from the previous tick — used to detect reconnect edges.
# Initialized to 0 so the first real stale→fresh transition fires correctly later.
_prev_old_data_warning = np.zeros(joint_num + 1, dtype=int)

# ==========================================================================
# Data recipe setup
# ==========================================================================
recipe_mgr = RecipeManager(DATA_RECIPE_DIR)

# Hoisted out of the RT loop — build once, references are stable.
# Passed to build_payload() every tick to route motor / motor-temp / motor-volt
# tags to the right per-node deque dict.
motor_buffers = {
    "data":        Motor_data,
    "temperature": Motor_temperature_data,
    "voltage":     Motor_voltage_data,
    "errors":      Motor_errors_data,
    "config":      shared_data.motor_config,
}

# ==========================================================================
# Mode dispatch — build TickContext and MODE_HANDLERS once before the loop.
# TickContext fields point to existing objects (no copies). Only
# rti_udp_data and current_tick are reassigned per tick.
# ==========================================================================
ctx = TickContext(
    Motor=Motor,
    Joint=Joint,
    joint_num=joint_num,
    active_CAN_gripper=active_CAN_gripper,
    shared_data=shared_data,
    robot=robot,
    delta_t=delta_t,
    rti_udp_data=None,
    socket_sender=RTI_socket_sender,
    recipe=None,
    Joint_data=Joint_data,
    Gripper_data=Gripper_data,
    current_tick=0,
    motor_buffers=motor_buffers,
)

MODE_HANDLERS = build_mode_dispatch(
    joint_num=joint_num,
    homing_ctrl=homing_ctrl,
    load_motor_configs_fn=load_motor_configs,
    motor_config={
        'velocity_limit': motor_velocity_limit,
        'Ilim': motor_Ilim,
        'KPP': motor_KPP, 'KP': motor_KP, 'KD': motor_KD,
        'KPIQ': motor_KPIQ, 'KIIQ': motor_KIIQ,
        'KPV': motor_KPV, 'KIV': motor_KIV,
    },
    jog_config={
        'max_vel_rad':   cfg['joint_velocity_limit'],
        'delta_t':       delta_t,
        'limit_neg':     cfg['software_joint_limit_negative'],
        'limit_pos':     cfg['software_joint_limit_positive'],
        # XML defaults from config/system.xml::<jog_defaults>
        'accel_time':    _sys['jog_defaults']['accel_time'],
        # Physical ceiling from the robot XML — jog's accel_time slider is a
        # preference and must not be able to exceed it.
        'accel_limit':   cfg['joint_acceleration_limit'],
        'profile':       _sys['jog_defaults']['profile'],
        'jerk_factor':   _sys['jog_defaults']['jerk_factor'],
        'control_mode':  _sys['jog_defaults']['control_mode'],
        'speed_pct':     _sys['jog_defaults']['speed_pct'],
    },
    motor_dynamics={
        'gear_ratio':      gear_ratio,
        'gear_efficiency': gear_efficiency,
        'motor_kt':        motor_kt,
        'motor_signs':     motor_signs,
    },
    shared_data=shared_data,
)

# RTI mode handler. Replaces the handle_idle placeholder build_mode_dispatch()
# installs for RobotMode.RTI. Module-level so the TCP command handlers below can
# drive its pairing lifecycle (connect_pc / claim_control / ...), which is separate
# from the enter(ctx)/exit(ctx) hooks dispatch calls on mode transitions.
rti_handler = RTIModeHandler(
    joint_num=joint_num,
    clock_mode=CLOCK_ROBOT,
    delta_t=delta_t,
    # Phase 2 rate-limiter budgets, from <rti_limits> per joint — RTI's own share of the
    # hardware ceiling in <joint_*_limit>. Separate from EXEC's <exec_limits> so that opening
    # a limit up to tune this limiter does not silently make every queued EXEC move punchier
    # (which is exactly what happened on 2026-08-06).
    # All four are optional in the robot XML at every level: a missing <rti_limits> falls back
    # to the ceiling, and a missing ceiling gets a documented derivation from
    # config/xml_parser.py — so these are never NaN. A NaN would silently disable clamping for
    # that joint, since every comparison against it is False.
    vel_limit=cfg['rti_velocity_limit'],
    accel_limit=cfg['rti_acceleration_limit'],
    jerk_limit=cfg['rti_jerk_limit'],
    torque_rate_limit=cfg['rti_torque_rate_limit'],
    # The soft joint limits. Already loaded and already handed to JogHandler — RTI simply
    # never asked for them, so nothing bounded an RTI command to the workspace.
    limit_neg=cfg['software_joint_limit_negative'],
    limit_pos=cfg['software_joint_limit_positive'],
    # RTI policy from <rti_defaults> in config/system.xml — see _parse_rti_defaults.
    # Robot-INDEPENDENT tuning; the per-joint physical limits above come from the robot XML.
    **{k: _sys['rti_defaults'][k] for k in (
        'command_timeout_s', 'stopping_hold_s', 'start_pose_tol_rad',
        'lowpass_cutoff_hz', 'success_window_ticks', 'success_warn', 'success_bad')},
)
MODE_HANDLERS[RobotMode.RTI] = rti_handler

# Maintenance policy from <maintenance_defaults> in config/system.xml.
flash_park_max_gravity_nm = _sys['maintenance_defaults']['flash_park_max_gravity_nm']

# IP of the PC currently paired over TCP. drain_rti_udp() filters on it so a
# stray sender cannot inject setpoints. None until rti_connect arrives, and the
# UDP read is skipped entirely while unpaired.
rti_pc_ip = None

# ==========================================================================
# RTI command dispatch — handlers for ZMQ commands (from command_executor,
# GUI, or Supervisor). Keyed by the first colon-delimited segment of the
# command string, e.g. "jog_set:2:1:0.5" dispatches on "jog_set". Handlers
# live here (not a separate module) because they mutate this file's own
# module-level state directly via `global` — the RT loop is a flat script,
# not a class, so this keeps the exact same data flow as before, just
# organized as one lookup table instead of a 130-line inline elif chain.
# Extracted 2026-07-03 — pure reorganization, identical logic per command,
# see TODO/exec_mode_implementation_checklist.md for the equivalence notes.
# ==========================================================================

def _handle_enable(cmd_msg, cmd_dict):
    _errors_active = bool(shared_data.rti['error_active'][0])
    _safety_active = bool(shared_data.rti['safety_violation'][0])
    success, reason = request_enable(_errors_active, _safety_active)
    if success:
        shared_data.robot_state.set("ENABLED")
    else:
        shared_data.robot_state.set(f"ENABLE_REJECTED:{reason}")


def _handle_disable(cmd_msg, cmd_dict):
    global pending_mode_request
    shared_data.robot_state.set("DISABLED")
    pending_mode_request = RobotMode.IDLE


def _handle_home(cmd_msg, cmd_dict):
    global pending_mode_request
    pending_mode_request = RobotMode.HOMING


def _handle_clear_errors(cmd_msg, cmd_dict):
    global clear_error_countdown
    send_clear_errors(shared_data, Motor, Motor_errors_data, joint_num, CLEAR_ERROR_SEND_REPEATS,
                      Gripper_data=Gripper_data, active_CAN_gripper=active_CAN_gripper)
    clear_error_countdown = CLEAR_ERROR_SETTLE_TICKS


def _handle_gravity_comp(cmd_msg, cmd_dict):
    # Guard added by the 2026-07-03 dispatch-table extraction: the old
    # `elif cmd_msg.startswith("gravity_comp:")` guaranteed a colon before
    # this branch was ever entered. A plain-string dispatch key doesn't, so
    # this checks explicitly instead of indexing straight into parts[1].
    parts = cmd_msg.split(":", 1)
    if len(parts) != 2:
        return
    val = parts[1].strip()
    new_val = 1.0 if val == "1" else 0.0
    old_val = shared_data.rti['gravity_comp_enabled'][0]
    shared_data.rti['gravity_comp_enabled'][0] = new_val
    if new_val != old_val:
        log_activity(_rti_activity_logger, "RTI", "info", "COMPENSATION_CHANGED",
                     message=f"gravity comp: {'ON' if new_val else 'OFF'}")


def _handle_set_mode(cmd_msg, cmd_dict):
    global pending_mode_request, pending_mode_force, pending_mode_assertion
    parts = cmd_msg.split(":")  # see _handle_gravity_comp for why this is guarded
    if len(parts) < 2:
        return
    pending_mode_request = parts[1].strip().upper()
    # Maintenance modes need an EXPLICIT operator assertion — there is no
    # automatic way to know the arm can survive going limp:
    #
    #   set_mode:FLASHING          -> refused, no assertion given
    #   set_mode:FLASHING:PARKED   -> operator moved it to the park pose and confirmed
    #   set_mode:FLASHING:FORCE    -> operator asserts it is supported WITHOUT parking
    #
    # PARKED and FORCE are equivalent to the gate; they are distinguished so the
    # log records which claim was made. Replaces a gravity-torque threshold that
    # could not do this job: PAR6's own standby pose measures 1.80 Nm against a
    # shipped 1.0 Nm limit, so the correct answer failed the check (2026-08-05).
    _assertion = parts[2].strip().upper() if len(parts) > 2 else ""
    pending_mode_force = _assertion in ("PARKED", "FORCE")
    pending_mode_assertion = _assertion


def _handle_jog_set(cmd_msg, cmd_dict):
    parts = cmd_msg.split(":")
    if len(parts) == 4 and current_mode == RobotMode.JOG:
        _jog = MODE_HANDLERS.get(RobotMode.JOG)
        if _jog is not None:
            _jog.set_jog(int(parts[1]), int(parts[2]), float(parts[3]))


def _handle_jog_stop(cmd_msg, cmd_dict):
    _jog = MODE_HANDLERS.get(RobotMode.JOG)
    if _jog is not None:
        _jog.stop_jog()


def _handle_jog_accel(cmd_msg, cmd_dict):
    parts = cmd_msg.split(":")
    if len(parts) == 2:
        _jog = MODE_HANDLERS.get(RobotMode.JOG)
        if _jog is not None:
            _jog.set_accel_time(float(parts[1]))


def _handle_jog_profile(cmd_msg, cmd_dict):
    parts = cmd_msg.split(":", 1)
    if len(parts) == 2:
        _jog = MODE_HANDLERS.get(RobotMode.JOG)
        if _jog is not None:
            try:
                _jog.set_profile(parts[1].strip())
            except ValueError as exc:
                logger.warning("jog_profile: %s", exc)


def _handle_jog_jerk_factor(cmd_msg, cmd_dict):
    parts = cmd_msg.split(":", 1)
    if len(parts) == 2:
        _jog = MODE_HANDLERS.get(RobotMode.JOG)
        if _jog is not None:
            _jog.set_jerk_factor(float(parts[1]))


def _handle_jog_control_mode(cmd_msg, cmd_dict):
    parts = cmd_msg.split(":", 1)
    if len(parts) == 2:
        _jog = MODE_HANDLERS.get(RobotMode.JOG)
        if _jog is not None:
            try:
                _jog.set_control_mode(parts[1].strip())
            except ValueError as exc:
                logger.warning("jog_control_mode: %s", exc)


def _handle_exec_pause(cmd_msg, cmd_dict):
    _exec = MODE_HANDLERS.get(RobotMode.EXEC)
    if _exec is not None:
        _exec.pause()


def _handle_exec_resume(cmd_msg, cmd_dict):
    _exec = MODE_HANDLERS.get(RobotMode.EXEC)
    if _exec is not None:
        _exec.resume()


def _handle_exec_completion_policy(cmd_msg, cmd_dict):
    parts = cmd_msg.split(":", 1)
    if len(parts) != 2:
        return
    _exec = MODE_HANDLERS.get(RobotMode.EXEC)
    if _exec is not None:
        try:
            _exec.set_completion_policy(parts[1].strip())
        except ValueError as exc:
            logger.warning("exec_completion_policy: %s", exc)


def _handle_gripper_set(cmd_msg, cmd_dict):
    # gripper_set:<position>:<speed>:<current>:<action>
    # position/speed: 0–255, current: mA, action: 0/1.
    # activate is always 1 in firmware mode (forced in send_gripper_commands).
    # A new gripper_set also aborts any in-progress calibration —
    # the user can see when the sweep is done, so an explicit
    # Open/Close/Goto/Stop is the cancel signal.
    global _gripper_calibrate_running, _gripper_calibrate_pending
    parts = cmd_msg.split(":")
    if len(parts) == 5 and active_CAN_gripper:
        try:
            _gripper_fw_state['position'] = int(parts[1])
            _gripper_fw_state['speed']    = int(parts[2])
            _gripper_fw_state['current']  = int(parts[3])
            _gripper_fw_state['action']   = int(parts[4])
            if _gripper_calibrate_running or _gripper_calibrate_pending:
                logger.info("[Gripper] calibrate aborted by gripper_set")
                _gripper_calibrate_running = False
                _gripper_calibrate_pending = False
        except ValueError as exc:
            logger.warning("gripper_set: bad payload %s", exc)


def _handle_gripper_calibrate(cmd_msg, cmd_dict):
    global _gripper_calibrate_pending
    if active_CAN_gripper:
        _gripper_calibrate_pending = True


def _handle_gripper_ctrl_mode(cmd_msg, cmd_dict):
    # 1 = motor mode, 2 = firmware mode
    parts = cmd_msg.split(":", 1)  # see _handle_gravity_comp for why this is guarded
    if len(parts) != 2 or not active_CAN_gripper:
        return
    try:
        _new_mode = int(parts[1].strip())
        if _new_mode in (1, 2):
            shared_data.rti['gripper_ctrl_mode'][0] = _new_mode
            # Stop any pending move when leaving firmware mode so the
            # gripper sits still on the next firmware re-entry.
            if _new_mode == 1:
                _gripper_fw_state['action'] = 0
    except ValueError as exc:
        logger.warning("gripper_ctrl_mode: bad payload %s", exc)


def _handle_set_pid_gains(cmd_msg, cmd_dict):
    global gripper_KPP, gripper_KPV, gripper_KIV, gripper_KPIQ, gripper_KIIQ
    global gripper_KP, gripper_KD, gripper_Ilim, gripper_velocity_limit
    global gripper_voltage_limit
    _joint = int(cmd_dict.get("joint", -1))
    if 0 <= _joint < joint_num:
        if "KPP"     in cmd_dict: motor_KPP[_joint]            = float(cmd_dict["KPP"])
        if "KPV"     in cmd_dict: motor_KPV[_joint]            = float(cmd_dict["KPV"])
        if "KIV"     in cmd_dict: motor_KIV[_joint]            = float(cmd_dict["KIV"])
        if "KPIQ"    in cmd_dict: motor_KPIQ[_joint]           = float(cmd_dict["KPIQ"])
        if "KIIQ"    in cmd_dict: motor_KIIQ[_joint]           = float(cmd_dict["KIIQ"])
        if "KP"      in cmd_dict: motor_KP[_joint]             = float(cmd_dict["KP"])
        if "KD"      in cmd_dict: motor_KD[_joint]             = float(cmd_dict["KD"])
        if "Ilim"    in cmd_dict: motor_Ilim[_joint]           = float(cmd_dict["Ilim"])
        if "vel_lim" in cmd_dict: motor_velocity_limit[_joint] = float(cmd_dict["vel_lim"])
        if "vlim"    in cmd_dict: motor_voltage_limit[_joint]  = read_XML2.clamp_voltage_limit(
                                                                     cmd_dict["vlim"])
        _reload_motor_config(num_repeats=2, motor_index=_joint)
    elif _joint == joint_num and active_CAN_gripper:
        if "KPP"     in cmd_dict: gripper_KPP            = float(cmd_dict["KPP"])
        if "KPV"     in cmd_dict: gripper_KPV            = float(cmd_dict["KPV"])
        if "KIV"     in cmd_dict: gripper_KIV            = float(cmd_dict["KIV"])
        if "KPIQ"    in cmd_dict: gripper_KPIQ           = float(cmd_dict["KPIQ"])
        if "KIIQ"    in cmd_dict: gripper_KIIQ           = float(cmd_dict["KIIQ"])
        if "KP"      in cmd_dict: gripper_KP             = float(cmd_dict["KP"])
        if "KD"      in cmd_dict: gripper_KD             = float(cmd_dict["KD"])
        if "Ilim"    in cmd_dict: gripper_Ilim           = float(cmd_dict["Ilim"])
        if "vel_lim" in cmd_dict: gripper_velocity_limit = float(cmd_dict["vel_lim"])
        if "vlim"    in cmd_dict: gripper_voltage_limit  = read_XML2.clamp_voltage_limit(
                                                               cmd_dict["vlim"])
        _reload_gripper_config(num_repeats=2)


# --- RTI mode (TODO/RTI_mode_plan.md §4d) ---------------------------------
# These drive the handler's PAIRING lifecycle. They must never call enter()/exit(),
# which belong to mode dispatch and take a TickContext.

def _handle_rti_connect(cmd_msg, cmd_dict):
    """rti_connect:<pc_ip> — pair with a PC and request RobotMode.RTI.

    The PC's IP is the one thing the robot cannot discover for itself (UDP has not
    started yet), so it rides the TCP handshake. State packets go to that IP on
    RTI_PORT_SENDER; commands are only accepted from that IP.
    """
    global pending_mode_request, rti_pc_ip

    parts = cmd_msg.split(":", 1)   # see _handle_gravity_comp for why this is guarded
    if len(parts) != 2 or not parts[1].strip():
        logger.warning("[RTI] rti_connect missing pc_ip")
        return
    pc_ip = parts[1].strip()

    rti_pc_ip = pc_ip
    rti_handler.connect_pc((pc_ip, RTI_PORT_SENDER),
                           current_tick=int(shared_data.rti["loop_tick"][0]))
    pending_mode_request = RobotMode.RTI


def _handle_rti_claim(cmd_msg, cmd_dict):
    """rti_claim — PC claims RT control (connected → control_active).

    Rejected while an error is latched: control must not begin before the error is
    explicitly recovered (plan §3 / §15).
    """
    if bool(shared_data.rti['error_active'][0]):
        logger.warning("[RTI] rti_claim rejected — error latched, clear errors first")
        return
    if current_mode != RobotMode.RTI:
        logger.warning("[RTI] rti_claim rejected — mode is %s, not RTI", current_mode)
        return
    if rti_handler.sub_state != 'connected':
        logger.warning("[RTI] rti_claim rejected — sub_state is '%s'", rti_handler.sub_state)
        return
    # Seed the rate limiter from the live arm, so its first output ramps from where the
    # robot actually is rather than from 0 rad.
    rti_handler.claim_control(
        current_tick=int(shared_data.rti["loop_tick"][0]),
        measured_pos=shared_data.joint['joint_position'][:joint_num],
    )


def _handle_rti_set_rate_limiter(cmd_msg, cmd_dict):
    """rti_set_rate_limiter:<0|1> — accel/jerk/torque-rate clamping on incoming commands."""
    parts = cmd_msg.split(":", 1)
    if len(parts) == 2:
        rti_handler.set_rate_limiter(parts[1].strip() == "1")


def _handle_rti_set_gravity_comp(cmd_msg, cmd_dict):
    """rti_set_gravity_comp:<0|1> — gravity feedforward on RTI commands (default ON)."""
    parts = cmd_msg.split(":", 1)
    if len(parts) == 2:
        rti_handler.set_gravity_comp(parts[1].strip() == "1")


def _handle_set_recipe(cmd_msg, cmd_dict):
    """set_recipe:<name> — choose which telemetry recipe the state stream carries.

    The recipe system exists so the controlling PC can decide what state it receives, but
    until 2026-08-07 only the GUI could set `active_recipe` — so the one process that most
    needs it could not. A remote PC had no way to ask for `diagnostics` (the recipe carrying
    rti_ok / rti_miss), which is the ONLY route to robot-side link quality over the network,
    since success rate and miss counters live in shared memory a remote host cannot read.

    Validated against the recipes actually on disk. An unknown name is REFUSED and logged
    rather than set: RecipeManager.get() silently falls back to a minimal recipe for an
    unknown name, so accepting a typo here would quietly strip the operator's telemetry down
    and look like the robot had stopped reporting.
    """
    parts = cmd_msg.split(":", 1)
    if len(parts) != 2:
        return
    name = parts[1].strip()
    known = recipe_mgr.names()
    if name not in known:
        logger.warning("[RTI] set_recipe refused: %r is not a known recipe. Available: %s",
                       name, ", ".join(known))
        return
    shared_data.active_recipe.set(name)
    logger.info("[RTI] Active recipe -> %s", name)


def _handle_rti_set_start_pose_check(cmd_msg, cmd_dict):
    """rti_set_start_pose_check:<0|1> — validate the first setpoint after rti_claim."""
    parts = cmd_msg.split(":", 1)
    if len(parts) == 2:
        rti_handler.set_start_pose_check(parts[1].strip() == "1")


def _handle_rti_set_lowpass(cmd_msg, cmd_dict):
    """rti_set_lowpass:<cutoff_hz> — command low-pass cutoff; 0 disables. Default OFF."""
    parts = cmd_msg.split(":", 1)
    if len(parts) != 2:
        return
    try:
        rti_handler.set_lowpass_filter(float(parts[1].strip()))
    except ValueError as exc:
        logger.warning("rti_set_lowpass: bad payload %s", exc)


def _handle_rti_release(cmd_msg, cmd_dict):
    """rti_release — TCP fallback for a CLEAN stop.

    The primary path is UDP CMD_STOP inside the command packet; this exists for when
    the PC's UDP sender is already dead. If neither arrives the timeout covers it.
    """
    if not rti_handler.request_release():
        logger.warning("[RTI] rti_release ignored — sub_state is '%s'", rti_handler.sub_state)


def _handle_rti_disconnect(cmd_msg, cmd_dict):
    """rti_disconnect — drop the pairing and return to IDLE."""
    global pending_mode_request, rti_pc_ip

    rti_handler.disconnect()
    rti_pc_ip = None
    pending_mode_request = RobotMode.IDLE
    # Blank the telemetry as we unpair. step() is what normally publishes it, and step()
    # stops running the moment we leave RTI mode — so without this the GUI would keep
    # showing the finished session's success rate and miss counts as if they were live.
    shared_data.rti["rti_substate"][0] = 0
    shared_data.rti["control_success_rate"][0] = 0
    shared_data.rti["rti_consecutive_misses"][0] = 0
    shared_data.rti["rti_worst_consecutive_misses"][0] = 0
    shared_data.rti["rti_tx_bytes_per_s"][0] = 0
    shared_data.rti["rti_rx_bytes_per_s"][0] = 0
    shared_data.rti["rti_seq_rx_last"][0] = 0
    shared_data.rti["rti_discard_pct"][0] = 0


def _rti_command_key(cmd_msg):
    """First colon-delimited segment — e.g. 'jog_set:2:1:0.5' -> 'jog_set'."""
    return cmd_msg.split(":", 1)[0]


RTI_COMMAND_HANDLERS = {
    "enable":            _handle_enable,
    "disable":           _handle_disable,
    "home":              _handle_home,
    "clear_errors":      _handle_clear_errors,
    "gravity_comp":      _handle_gravity_comp,
    "set_mode":          _handle_set_mode,
    "jog_set":           _handle_jog_set,
    "jog_stop":          _handle_jog_stop,
    "jog_accel":         _handle_jog_accel,
    "jog_profile":       _handle_jog_profile,
    "jog_jerk_factor":   _handle_jog_jerk_factor,
    "jog_control_mode":  _handle_jog_control_mode,
    "exec_pause":        _handle_exec_pause,
    "exec_resume":       _handle_exec_resume,
    "exec_completion_policy": _handle_exec_completion_policy,
    "gripper_set":       _handle_gripper_set,
    "gripper_calibrate": _handle_gripper_calibrate,
    "gripper_ctrl_mode": _handle_gripper_ctrl_mode,
    "set_pid_gains":     _handle_set_pid_gains,
    "rti_connect":       _handle_rti_connect,
    "rti_claim":         _handle_rti_claim,
    "rti_release":       _handle_rti_release,
    "rti_disconnect":    _handle_rti_disconnect,
    "rti_set_rate_limiter":     _handle_rti_set_rate_limiter,
    "rti_set_gravity_comp":     _handle_rti_set_gravity_comp,
    "set_recipe":               _handle_set_recipe,
    "rti_set_start_pose_check": _handle_rti_set_start_pose_check,
    "rti_set_lowpass":          _handle_rti_set_lowpass,
}

# ==========================================================================
# RTI superloop
# ==========================================================================
while True:

    shared_data.rti["loop_tick"][0]  = shared_data.rti["loop_tick"][0]  + 1
    time1 = time.perf_counter()
    _profile_last = time1

    # ==========================================================================
    # Handle GPIO inputs and outputs
    # ==========================================================================
    read_all_inputs()
    _profile_last = _profile_mark(_profile_last, "gpio_read")

    # ==========================================================================
    # RT loop timing stats
    # update_rt_loop_stats() is called at the TOP of the loop, before any work,
    # so loop_period measures wall-clock time from the previous tick's start to
    # this one's start.  loop_execution_time (measured at the BOTTOM of the loop
    # just before sleep) is how much of that budget was actually consumed.
    #
    # loop_stats_reset_signal: GUI/client sets to "1" to zero the "since start"
    # peaks below (max_loop_period/max_loop_execution_time/max_jitter/
    # overrun_count) — those persist in shared memory across RTI restarts (same
    # segment, re-attached not recreated) so a single old spike can silently
    # pollute them for hours otherwise. Checked before this tick's update so the
    # reset (and the rolling p95/p99 window reset alongside it) takes effect
    # starting this same tick, not one tick late.
    #
    # The same wipe also runs automatically at two boot ticks:
    #
    #   tick 2 — kills two lies that would otherwise last the whole run:
    #     (1) the peaks re-attached from shm, which belong to the PREVIOUS RTI
    #         run (a spike from days ago reads as if it happened this session);
    #     (2) tick 1's own sample — prev_time is captured before the Kt CAN fetch
    #         and the config loads, so tick 1's "period" is the entire boot
    #         sequence, not a tick. update_rt_loop_stats has no first-tick guard
    #         for the peaks: its avg_dur == 0 branch only re-seeds the EMAs,
    #         while the max_* comparisons and overrun_count run unconditionally.
    #     Tick 2 rather than tick 1 is deliberate — letting tick 1 record its
    #     garbage and then discarding it needs no special-casing of the updates.
    #
    #   LOOP_STATS_WARMUP_TICKS — kills the boot transients themselves (tick 8
    #     selfcheck, ticks 50/150/300 reconfig), so from here the peaks describe
    #     steady state. Logged once first: the discarded values are the only
    #     record of how ugly boot was, and without them a pathological boot would
    #     vanish silently.
    # ==========================================================================
    _stats_tick = shared_data.rti["loop_tick"][0]
    _stats_boot_wipe = _stats_tick == LOOP_STATS_WARMUP_TICKS
    _stats_warmup_wipe = _stats_tick == 2 or _stats_boot_wipe
    if _stats_warmup_wipe or shared_data.loop_stats_reset_signal.get() == "1":
        if _stats_boot_wipe:
            logger.info(
                f"[RTI] Boot peaks discarded at tick {int(LOOP_STATS_WARMUP_TICKS)} — "
                f"period {shared_data.rti['max_loop_period'][0]*1e3:.3f} ms, "
                f"exec {shared_data.rti['max_loop_execution_time'][0]*1e3:.3f} ms, "
                f"jitter {shared_data.rti['max_jitter'][0]*1e6:.1f} us, "
                f"{int(shared_data.rti['overrun_count'][0])} overruns. "
                f"Stats from here describe steady state."
            )
        shared_data.rti["max_loop_period"][0] = 0.0
        shared_data.rti["max_loop_execution_time"][0] = 0.0
        shared_data.rti["max_jitter"][0] = 0.0
        shared_data.rti["overrun_count"][0] = 0.0
        shared_data.rti["can_frame_age_peak"][0] = 0.0
        _loop_stats_buf_pos, _loop_stats_buf_count = reset_rolling_stats()
        shared_data.loop_stats_reset_signal.set("0")
        if _stats_tick == 2:
            # Tick 1 seeded the EMAs with that boot-length sample too. Zeroing
            # avg_loop_duration makes update_rt_loop_stats re-seed all three from
            # tick 2's real sample, instead of decaying the garbage over ~60 ticks.
            shared_data.rti["avg_loop_duration"][0] = 0.0

    (prev_time,                                        # updated to time1 — becomes prev_time for the next tick
     shared_data.rti["loop_period"][0],                # time1 - prev_time  [s]  — actual wall-clock gap between tick starts; ideal = delta_t
     shared_data.rti["avg_loop_duration"][0],          # EMA(loop_period, α=0.05) — smoothed period; low α → smooth but slow to react
     shared_data.rti["avg_loop_execution_time"][0],    # EMA(loop_execution_time, α=0.05) — smoothed compute budget consumption
     shared_data.rti["jitter"][0],                     # |loop_period - delta_t|  [s]  — how far this tick deviated from nominal timing
     shared_data.rti["avg_jitter"][0],                 # EMA(jitter, α=0.05) — smoothed jitter;  errors if jitter causes >20 missed packets
     shared_data.rti["max_loop_period"][0],            # max(loop_period) since start [s]  — worst-case tick gap ever seen
     shared_data.rti["max_loop_execution_time"][0],    # max(loop_execution_time) since start [s]  — worst-case compute time;  budget ≈ 300 µs
     shared_data.rti["max_jitter"][0],                 # max(jitter) since start [s]  — single worst deviation from nominal period
     shared_data.rti["overrun_count"][0],              # cumulative ticks where loop_execution_time > delta_t — should stay 0 on a healthy RT system
    ) = update_rt_loop_stats(
        time1, prev_time, loop_execution_time, delta_t,
        shared_data.rti["avg_loop_duration"][0],
        shared_data.rti["avg_loop_execution_time"][0],
        shared_data.rti["avg_jitter"][0],
        shared_data.rti["max_loop_period"][0],
        shared_data.rti["max_loop_execution_time"][0],
        shared_data.rti["max_jitter"][0],
        shared_data.rti["overrun_count"][0],
        alpha=0.05,
    )

    # Rolling p95/p99 window — always reflects only the last LOOP_STATS_BUFFER_SIZE
    # ticks, so unlike the "since start" peaks above it never needs a manual
    # reset to stay meaningful. Sampled from the exact same loop_period/
    # loop_execution_time values update_rt_loop_stats just used, so both stay
    # in sync.
    (_loop_stats_buf_pos, _loop_stats_buf_count,
     shared_data.rti["p95_loop_period"][0],
     shared_data.rti["p99_loop_period"][0],
     shared_data.rti["p95_loop_execution_time"][0],
     shared_data.rti["p99_loop_execution_time"][0],
    ) = update_rolling_percentiles(
        _loop_period_buf, _loop_exec_buf, _loop_stats_buf_pos, _loop_stats_buf_count,
        shared_data.rti["loop_period"][0], loop_execution_time,
        shared_data.rti["loop_tick"][0],
        shared_data.rti["p95_loop_period"][0],
        shared_data.rti["p99_loop_period"][0],
        shared_data.rti["p95_loop_execution_time"][0],
        shared_data.rti["p99_loop_execution_time"][0],
        stats_interval=LOOP_STATS_INTERVAL,
    )
    _profile_last = _profile_mark(_profile_last, "timing_stats")

    # ==========================================================================
    # Run selfcheck (only once at startup)
    # ==========================================================================
    if startup_selfcheck == 0 and shared_data.rti["loop_tick"][0] == 8:
        scan_ids = scan_can_bus(communication=communication1)
        shared_data.rti["scan_received_ids"][:] = scan_ids
        run_selfcheck(
            Motor=Motor,
            communication=communication1,
            joint_num=joint_num,
            active_CAN_gripper=active_CAN_gripper,
            shared_data=shared_data,
            scan_received_ids=scan_ids,
        )

        startup_selfcheck = 1
        pending_mode_request = RobotMode.IDLE  # selfcheck done → transition out of BOOTING
    _profile_last = _profile_mark(_profile_last, "startup_selfcheck")

    # ==========================================================================
    # Multi-shot motor reconfiguration at ticks 50 / 150 / 300 
    # Its a dirty fix for weird BUG caused our exec time to be double until we home the robot?????
    # Multiple passes cover motors with different boot times.
    # ==========================================================================
    _tick = shared_data.rti["loop_tick"][0]
    if startup_motor_reconfig < 3 and _tick in (50, 150, 300):
        _reload_motor_config()
        _reload_gripper_config()
        startup_motor_reconfig += 1

    # ==========================================================================
    # ZMQ Check for new commands and send confirmations
    # ========================================================================== 
    try:
        command = command_queue.get_nowait() # Try to remove one element from command_queue
        #TODO
            # If we are executing command already and somehow we get something in the queue here
            # ignore it and report that we got some trash somehow.
            # If we are executing some command and receive ESTOP, stop program...
            # they have higher priority and will overwrite the current command

        shared_data.robot_command.set(command)

        # Route mode-change commands to the state machine
        try:
            cmd_dict = json.loads(command)
            cmd_msg = cmd_dict.get("msg", "").lower()
            _handler = RTI_COMMAND_HANDLERS.get(_rti_command_key(cmd_msg))
            if _handler is not None:
                _handler(cmd_msg, cmd_dict)
        except (json.JSONDecodeError, AttributeError):
            pass  # Non-JSON or unexpected format — not a mode command

        try:
            status_socket.send_string("done", zmq.NOBLOCK)
        except zmq.Again:
            pass  # drop — don't block the RT loop


    except Empty:
        pass  # No command, keep loop going
    _profile_last = _profile_mark(_profile_last, "zmq_commands")
    
    # ==========================================================================
    # Try to read RTI UDP interface
    # ==========================================================================

    # sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024) # set large receive buffer???
    # When full, new packets are dropped by the OS???
    # NOTE: ordering can get confused with UDP; drain_rti_udp returns only the latest valid packet
    #
    # Only read while a PC is paired: rti_pc_ip is set by rti_connect and is the IP
    # filter, so an unpaired robot accepts no setpoints at all. allowed_port=None
    # accepts any source port — the PC's send port is ephemeral unless it explicitly
    # binds EXPECTED_REMOTE_PORT.
    rti_udp_count = 0
    if rti_pc_ip is None:
        rti_udp_data = None
    else:
        rti_udp_data, rti_udp_count = drain_rti_udp(
            RTI_socket_receive, allowed_ip=rti_pc_ip, allowed_port=None)
        if rti_udp_data is not None:
            # Process data
            logging.debug(f"Received valid RTI data: {rti_udp_data}")

    # Update per-tick TickContext fields (attribute swap, no allocation)
    ctx.rti_udp_data = rti_udp_data
    ctx.rti_udp_count = rti_udp_count
    ctx.current_tick = int(shared_data.rti["loop_tick"][0])
    # Resolved here rather than at the telemetry block below because RTI mode's
    # handler serialises its own state packet during dispatch, which runs first.
    # recipe_mgr.get() is a cached name comparison, so this is the same one
    # resolution per tick it always was — the telemetry block reuses it.
    ctx.recipe = recipe = recipe_mgr.get(shared_data.active_recipe.get())
    _profile_last = _profile_mark(_profile_last, "udp_rx_context")

    # ==========================================================================
    # Perform robot state and mode management
    # ==========================================================================
    if pending_mode_request is not None:
        _errors_active = bool(shared_data.rti['error_active'][0])
        _robot_enabled = shared_data.robot_state.get() == "ENABLED"
        _homed = shared_data.homed_state.get() == "HOMED"
        # Can the arm survive losing torque? Nothing on the robot can answer
        # that, so a HUMAN must assert it — see _handle_set_mode.
        #
        # This used to be inferred from joint_gravity_torque against
        # <flash_park_max_gravity_nm>. That was a proxy, and a wrong one: PAR6's
        # own standby pose (`qs`) measures 1.80 Nm against the shipped 1.0 Nm
        # limit, so the arm's designated resting pose FAILED the gate while a
        # pose the model knew nothing about could pass it. Gravity torque says
        # how hard the motors are working, not whether the arm is resting on
        # something. The number is still published for the operator to see; it
        # just no longer decides.
        _park_ok = bool(pending_mode_force)
        success, new_mode, reason = request_mode_change(
            current_mode, pending_mode_request,
            robot_enabled=_robot_enabled,
            homed=_homed,
            errors_active=_errors_active,
            park_ok=_park_ok,
        )
        if success:
            # Record WHICH claim the operator made to get in here. If an arm
            # ever falls during maintenance, this line is the evidence for
            # whether it was parked first or overridden.
            if new_mode in MAINTENANCE_MODES:
                _worst_gravity = float(np.max(np.abs(
                    shared_data.joint['joint_gravity_torque'][:joint_num])))
                if pending_mode_assertion == "FORCE":
                    logging.warning(
                        f"[RTI][Flash] Entering {new_mode} via FORCE — operator "
                        f"asserts the arm is supported WITHOUT parking "
                        f"(gravity {_worst_gravity:.2f} Nm).")
                else:
                    logging.info(
                        f"[RTI][Flash] Entering {new_mode} — operator confirmed "
                        f"the arm is PARKED (gravity {_worst_gravity:.2f} Nm).")

            # Exit old mode handler (e.g. homing restores motor configs)
            _old_handler = MODE_HANDLERS.get(current_mode)
            if _old_handler is not None and hasattr(_old_handler, 'exit'):
                _old_handler.exit(ctx)

            # Leaving FLASHING: re-base the CAN freshness clock. Nodes were
            # silent for the whole flash (deliberately — we stopped polling), so
            # tick_diff is now thousands of ticks and update_can_connection_status
            # would latch a disconnect error on every node before any of them got
            # a chance to answer. Stamping "last heard from" to now gives them the
            # normal error_threshold window to come back; anything that genuinely
            # stays dead is still caught 50 ticks later.
            if current_mode == RobotMode.FLASHING:
                _now_tick = shared_data.rti["loop_tick"][0]
                for _i in range(joint_num + active_CAN_gripper):
                    Motor_data[_i][-1]["tick"] = _now_tick
                logging.info("[RTI][Flash] CAN freshness re-based on FLASHING exit")

                # A flash REBOOTS the driver, and the encoders are on the motors,
                # so the flashed joint's position reference is gone. Nothing else
                # invalidates it here: FLASHING's whole point is that RTI keeps
                # running, and an RTI restart is what normally clears homing. The
                # marker is dropped by the flasher only on a SUCCESSFUL flash, so
                # a scan-only session does not cost a re-home.
                # Cleared for the WHOLE robot rather than per joint: one flag to
                # reason about, and it matches what an RTI restart already does.
                try:
                    if FLASH_PERFORMED_MARKER.exists():
                        FLASH_PERFORMED_MARKER.unlink()
                        shared_data.homed_state.set("NOT HOMED")
                        shared_data.rti["homing_status"][:] = 0
                        shared_data.rti["joint_homed"][:] = 0
                        logging.error("[RTI][Flash] Firmware was flashed — "
                                      "HOMING INVALIDATED. Re-home before moving.")
                except OSError as _err:
                    # Never let housekeeping take the loop down; fail LOUD and
                    # unhomed rather than silently keeping a stale reference.
                    shared_data.homed_state.set("NOT HOMED")
                    shared_data.rti["homing_status"][:] = 0
                    shared_data.rti["joint_homed"][:] = 0
                    logging.error(f"[RTI][Flash] Could not read the flash marker "
                                  f"({_err}) — clearing homing anyway.")

            _from_mode = current_mode
            current_mode = new_mode
            shared_data.robot_mode.set(current_mode)
            logging.info(f"[RTI][Mode] → {current_mode}")
            log_activity(_rti_activity_logger, "RTI", "info", "MODE_CHANGE",
                         message=f"{_from_mode} → {current_mode}")

            # Enter new mode handler
            _new_handler = MODE_HANDLERS.get(new_mode)
            if _new_handler is not None and hasattr(_new_handler, 'enter'):
                _new_handler.enter(ctx)
        else:
            logging.warning(f"[RTI][Mode] Request '{pending_mode_request}' rejected: {reason}")
        pending_mode_request = None
        pending_mode_force = False
        pending_mode_assertion = ""
    _profile_last = _profile_mark(_profile_last, "mode_state")


    # ==========================================================================
    # Mode dispatch — each handler sends its own CAN commands
    # ==========================================================================
    _tick_result = dispatch_step(MODE_HANDLERS.get(current_mode), ctx)

    if _tick_result.pending_mode is not None:
        pending_mode_request = _tick_result.pending_mode
    _profile_last = _profile_mark(_profile_last, "mode_dispatch")

    # FLASHING hands the bus to an external flasher process. RTI must put NOTHING
    # on can0 while it holds: the handler returning SELF_MANAGED covers the
    # dispatch send, gripper_owned covers the gripper block below, and this flag
    # covers the two remaining transmitters (telemetry polling, reconnect config
    # reloads). See utility/state_machine.py MAINTENANCE_MODES.
    _bus_silent = (current_mode == RobotMode.FLASHING)

    if not _tick_result.gripper_owned:
        if _gripper_calibrate_pending and active_CAN_gripper:
            Motor[joint_num].Send_gripper_calibrate()
            logger.info("[Gripper] firmware calibrate sent")
            _gripper_calibrate_pending = False
            _gripper_calibrate_running = True
            _gripper_calibrate_ticks   = 0
        elif _gripper_calibrate_running and active_CAN_gripper:
            # Empty poll — keeps the watchdog alive without overwriting the
            # calibration command in firmware. Stays in this state until the
            # user clicks Open/Close/Goto/Stop (which aborts via gripper_set)
            # or the safety timeout fires.
            Motor[joint_num].Send_gripper_data_pack()
            _gripper_calibrate_ticks += 1
            if _gripper_calibrate_ticks > _GRIPPER_CALIBRATE_MAX_TICKS:
                logger.warning("[Gripper] firmware calibration timed out")
                _gripper_calibrate_running = False
        else:
            send_gripper_commands(
                Motor, dummy_device, active_CAN_gripper, joint_num,
                ctrl_mode=int(shared_data.rti['gripper_ctrl_mode'][0]),
                gripper_state=_gripper_fw_state,
            )
    _profile_last = _profile_mark(_profile_last, "gripper_send")
    if not _bus_silent:
        handle_polling_and_overrides(Motor, scheduled_state, current_override, scheduled_override_queue, joint_num, active_CAN_gripper)
    _profile_last = _profile_mark(_profile_last, "polling_overrides")

    # ==========================================================================
    # Secondary data send; NOTE when sending these commands drivers do not respond anything!
    # ==========================================================================
    secondary_data_send()
    _profile_last = _profile_mark(_profile_last, "secondary_send")


    # ==========================================================================
    # Mainloop CAN receive and fill queues
    # ==========================================================================
    if _bus_silent:
        # Someone else owns the bus. Drop everything on the floor rather than
        # decoding it — flash stream frames alias onto real motor node IDs.
        drain_can_discard(communication1)
    else:
        Hall_index, additional_pin2, Edge_detect, received_ids, latest_commands_id = receive_and_process_can_messages(
            communication1, Motor, Motor_data, Gripper_data,
            Motor_temperature_data, Motor_voltage_data,
            Motor_errors_data, Motor_info_data, Motor_Kt_data,
            joint_num, active_CAN_gripper,
            shared_data.rti["loop_tick"][0],
            received_ids, latest_commands_id,
            node_error_bits=node_error_bits,
            frame_age_out=_can_age_buf,
        )
    # [0]=worst, [1]=best age among the frames read this tick. Both published: the SPREAD is
    # what tells a real backlog (min ~= max, both large) from a single stale frame class
    # (min small, max large).
    shared_data.rti["can_frame_age"][0] = _can_age_buf[0]
    shared_data.rti["can_frame_age_min"][0] = _can_age_buf[1]
    if _can_age_buf[0] > shared_data.rti["can_frame_age_peak"][0]:
        shared_data.rti["can_frame_age_peak"][0] = _can_age_buf[0]
    shared_data.motor["HALL_INDEX"][0] = Hall_index
    shared_data.rti["received_ids"][:] = received_ids[:] #Update the shared memory
    shared_data.rti["latest_commands_id"][:] = latest_commands_id[:]  #Update the shared memory
    # Live per-node fault bit (mutated in place at the CAN decode) — arm joints + gripper.
    shared_data.rti["node_error_bits"][:] = node_error_bits[:]

    received_ids[:]       = 0   # reset in-place — no allocation
    latest_commands_id[:] = -1  # reset in-place — no allocation
    _profile_last = _profile_mark(_profile_last, "can_receive_process")

    # ==========================================================================
    # CAN connection Safety and debug stuff
    # ==========================================================================
    # Skipped while the bus is handed over: nodes are silent BY DESIGN in
    # FLASHING, so freshness counting would latch CAN_node_connection_error on
    # every joint within error_threshold ticks, force ACTIVE_ERROR, and kick us
    # straight back out of the mode. The counters are re-based on exit — see the
    # FLASHING teardown in the mode-change block above.
    if not _bus_silent:
        update_can_connection_status(
            Motor_data=Motor_data,
            Motor=Motor,
            shared_data=shared_data,
            joint_num=joint_num,
            active_CAN_gripper=active_CAN_gripper,
            warning_threshold = 10,
            error_threshold = 50, #Number of loop ticks
        )
    _profile_last = _profile_mark(_profile_last, "can_connection_status")

    # ==========================================================================
    # Motor reconnect config reload — see communication/can_message_handlers.py
    # ==========================================================================
    # Also skipped while silent — a config reload is a burst of CAN sends, and
    # it would fire on every node the moment flashing made them look stale.
    if not _bus_silent:
        handle_motor_reconnect(
            shared_data=shared_data,
            Motor=Motor,
            joint_num=joint_num,
            active_CAN_gripper=active_CAN_gripper,
            startup_selfcheck=startup_selfcheck,
            prev_old_data_warning=_prev_old_data_warning,
            motor_watchdog_rate=motor_watchdog_rate,
            gripper_watchdog_rate=gripper_watchdog_rate,
            motor_velocity_limit=motor_velocity_limit,
            motor_Ilim=motor_Ilim,
            motor_voltage_limit=motor_voltage_limit,
            motor_KPP=motor_KPP, motor_KP=motor_KP, motor_KD=motor_KD,
            motor_KPIQ=motor_KPIQ, motor_KIIQ=motor_KIIQ,
            motor_KPV=motor_KPV, motor_KIV=motor_KIV,
            gripper_velocity_limit=gripper_velocity_limit,
            gripper_Ilim=gripper_Ilim,
            gripper_voltage_limit=gripper_voltage_limit,
            gripper_KP=gripper_KP, gripper_KD=gripper_KD,
            gripper_KPIQ=gripper_KPIQ, gripper_KIIQ=gripper_KIIQ,
            gripper_KPV=gripper_KPV, gripper_KIV=gripper_KIV,
            gripper_KPP=gripper_KPP,
        )
    _profile_last = _profile_mark(_profile_last, "motor_reconnect")

    # ==========================================================================
    # Fill the latest data shared arrays (Motor level)
    #
    # ⚠️ These next three calls are an ORDERED PIPELINE — do not reorder them:
    #   motor[] (here) -> joint[]+tcp[] (reads motor[]) -> Joint_data append (feeds
    #   the accel/jerk finite differences with THIS tick's values, so it must come
    #   last or [-1] stops being the previous tick).
    # Reordering breaks them silently — stale-by-one-tick data, no exception.
    # See data/update_shared_data.py's module docstring.
    # ==========================================================================
    update_motor_and_gripper_data(
        shared_data=shared_data,
        Motor_data=Motor_data,
        Motor_temperature_data=Motor_temperature_data,
        Motor_voltage_data=Motor_voltage_data,
        Motor_info_data=Motor_info_data,
        Motor_errors_data=Motor_errors_data,
        Motor_Kt_data=Motor_Kt_data,
        Gripper_data=Gripper_data,
        motor_error_bytes=motor_error_bytes,
        joint_num=joint_num,
        active_CAN_gripper=active_CAN_gripper,
        gripper_kt=float(gripper_kt),
        gripper_speed_bank=filter_banks['gripper_speed'],
    )
    _profile_last = _profile_mark(_profile_last, "motor_gripper_shared")
        
    # ==========================================================================
    # Fill the latest data shared arrays (Joint + TCP level) — via update_shared_data.py
    # ==========================================================================
    update_joint_and_tcp_data(
        shared_data=shared_data,
        Joint=Joint,
        Joint_data=Joint_data,
        filter_banks=filter_banks,
        joint_num=joint_num,
        delta_t=delta_t,
        gear_ratio=gear_ratio,
        gear_efficiency=gear_efficiency,
        motor_kt=motor_kt,
        motor_signs=motor_signs,
        robot=robot,
    )
    _profile_last = _profile_mark(_profile_last, "joint_tcp_shared")

    # ==========================================================================
    # Fill queues for joint + TCP level
    # ========================================================================== 
    process_Joint_data(Joint_data,
                       shared_data.joint['joint_position'],
                       shared_data.joint['joint_speed'],
                       shared_data.joint['joint_torque'],
                       shared_data.joint['joint_acceleration'],
                       shared_data.joint['joint_jerk'],
                       shared_data.joint['joint_speed_filtered'],
                       shared_data.joint['joint_torque_filtered'],
                       shared_data.joint['joint_acceleration_filtered'],
                       shared_data.joint['joint_jerk_filtered'],
                       shared_data.tcp['robot_pose'],
                       shared_data.tcp['robot_pose_matrix'],
                       shared_data.tcp['robot_pose_quaternion'],
                       shared_data.tcp['TCP_speed'],
                       shared_data.tcp['torque_external'],
                       shared_data.tcp['force_external'],
                       shared_data.tcp['TCP_speed_filtered'],
                       shared_data.tcp['TCP_acceleration_raw'],
                       shared_data.tcp['TCP_acceleration_filtered'],
                       shared_data.tcp['force_external_tool'],
                       shared_data.joint['joint_position_commanded'],
                       shared_data.joint['joint_speed_commanded'],
                       shared_data.joint['joint_acceleration_commanded'],
                       shared_data.joint['joint_torque_commanded'],
                       shared_data.tcp['TCP_pose_commanded'],
                       shared_data.tcp['TCP_speed_commanded'],
                       shared_data.joint['joint_position_target'],
                       shared_data.joint['joint_speed_target'],
                       shared_data.joint['joint_acceleration_target'],
                       shared_data.joint['joint_torque_target'],
                       shared_data.tcp['TCP_pose_target'],
                       shared_data.tcp['TCP_speed_target'],
                       time.perf_counter(),
                       loop_execution_time,
                       shared_data.rti["loop_tick"][0],
                       motor_error_bytes,
                       # RTI link diagnostics — read from shared memory rather than the
                       # handler so the value is whatever was published this tick, and so
                       # this call site does not care which mode is active. All zero
                       # outside RTI mode (RTI.py zeroes them at startup and on disconnect).
                       rti_seq=shared_data.rti["rti_seq_rx_last"][0],
                       rti_ok=shared_data.rti["control_success_rate"][0],
                       rti_miss=shared_data.rti["rti_consecutive_misses"][0],
                       rti_state=shared_data.rti["rti_substate"][0],
                       # .value, not str(): RobotMode is a (str, Enum), and str() on it
                       # yields "RobotMode.IDLE" rather than the bare name the PC expects.
                       mode=getattr(current_mode, "value", current_mode),
                       # MEASURED period, not delta_t — see the tag registry for why.
                       period_ms=shared_data.rti["loop_period"][0] * 1e3,
                    )
    _profile_last = _profile_mark(_profile_last, "joint_history")

    if active_CAN_gripper:
        process_Gripper_joint_data(
            Gripper_data, joint_num,
            shared_data.gripper['gripper_joint_position'][0],
            shared_data.gripper['gripper_joint_speed'][0],
            shared_data.gripper['gripper_joint_force'][0],
            shared_data.gripper['gripper_joint_position_target'][0],
            shared_data.gripper['gripper_joint_speed_target'][0],
            shared_data.gripper['gripper_joint_force_target'][0],
            shared_data.gripper['gripper_joint_position_commanded'][0],
            shared_data.gripper['gripper_joint_speed_commanded'][0],
            shared_data.gripper['gripper_joint_force_commanded'][0],
        ) 
    _profile_last = _profile_mark(_profile_last, "gripper_history")
    


    # ==========================================================================
    # Error checking — latched: each error is logged once when it appears and
    # stays latched until the user clears it. See utility/error_checks.py.
    # ==========================================================================
    # EXEC control-link watchdog: has command_executor's heartbeat stalled while EXEC motion
    # is in flight? Gated on EXEC + buffered samples so a dead/idle executor (nothing moving)
    # is never tripped — only a control link lost mid-motion. The heartbeat comes off the
    # executor's dedicated thread, so a long TOPPRA solve does NOT look like death.
    _exec_link_lost = _exec_link_watchdog.update(
        heartbeat=float(shared_data.rti["exec_heartbeat"][0]),
        tick=int(shared_data.rti["loop_tick"][0]),
        in_exec=(current_mode == RobotMode.EXEC),
        samples_remaining=float(shared_data.rti["exec_samples_remaining"][0]),
    )

    # RT loop health: LOOP_DEGRADED (warning, self-clearing) and LOOP_CRITICAL (hard,
    # latching → DISABLED + hold). Fed p99 rather than this tick's period, so one ugly
    # tick cannot trip it but a sustained loss of the deadline does.
    _loop_degraded, _loop_critical = _loop_monitor.update(
        p99_period=float(shared_data.rti["p99_loop_period"][0]),
        tick=int(shared_data.rti["loop_tick"][0]),
    )

    clear_error_countdown, latched_errors, error_metadata, current_mode = process_errors_tick(
        shared_data, Motor, Motor_errors_data, joint_num,
        clear_error_countdown, latched_errors, error_metadata,
        active_CAN_gripper, current_mode, homing_ctrl,
        check_estop_active(),
        Gripper_data=Gripper_data,
        exec_link_lost=_exec_link_lost,
        # RTI command watchdog. A KEY, not a direct error_active write — this
        # function owns that flag and recomputes it every tick.
        rti_link_lost=rti_handler.take_link_lost(),
        loop_degraded=_loop_degraded,
        loop_critical=_loop_critical,
    )
    _profile_last = _profile_mark(_profile_last, "error_checks")

    # utility/safety_checks.py ///// run_safety_checks()

    # ==========================================================================
    # Send UDP data — recipe-driven, switchable configuration via .xml
    # ==========================================================================
    # Reload all recipe files if the GUI uploaded a new one
    if shared_data.recipe_reload_signal.get() == "1":
        recipe_mgr.reload()
        shared_data.recipe_reload_signal.set("0")

    # Hot-reload the signal filters if the GUI asked. Same pattern as the recipe reload:
    # file I/O only on the one-shot signal, never per tick. Rebuilding resets each filter's
    # internal state, so expect a brief (~one filter window) transient after a reload.
    if shared_data.filter_reload_signal.get() == "1":
        filter_banks, _fsrc = load_filter_banks(FILTER_CONFIG_PATH, joint_num, delta_t)
        logger.info(f"[filters] hot-reloaded from {_fsrc}")
        shared_data.filter_reload_signal.set("0")
    # `recipe` was resolved into ctx near the top of this tick (see udp_rx_context).
    payload = build_payload(recipe, Joint_data, Gripper_data, motor_buffers, joint_num)
    broadcast_UDP_telemetry(telemetry_socket, payload)
    _profile_last = _profile_mark(_profile_last, "telemetry")



    # ==========================================================================
    # Good test if we are using RT, sleep will be "exactly" 1s if we are
    # ==========================================================================
    #real_time_test()

    # ==========================================================================
    # Write all outputs (apply computed commands to hardware)
    # ==========================================================================
    write_all_outputs()
    _profile_last = _profile_mark(_profile_last, "gpio_write")

    # ==========================================================================
    # Terminal heartbeat
    # Enable:   RTI_HEARTBEAT=1 python3 RTI.py
    # Interval: RTI_HEARTBEAT=1 RTI_HEARTBEAT_INTERVAL=50 python3 RTI.py
    # ==========================================================================
    if _HB_ENABLED and int(shared_data.rti["loop_tick"][0]) % _HB_INTERVAL == 0:
        print_heartbeat(shared_data)
    _profile_last = _profile_mark(_profile_last, "heartbeat")

    # ==========================================================================
    # Sleep for the rest of the loop
    # ==========================================================================
    shared_data.rti["timestamp"][0] = time.perf_counter()
    loop_execution_time = shared_data.rti["timestamp"][0] - time1
    shared_data.rti["loop_execution_time"][0] = loop_execution_time
    _profile_last = _profile_mark(_profile_last, "timing_finalize")
    _profile_report(loop_execution_time)
    # Names the section responsible when THIS tick blew its budget (RTI_OVERRUN_TRACE=1).
    # No-op unless enabled, and only logs on an actual overrun.
    _overrun_trace(loop_execution_time, shared_data.rti["loop_tick"][0], current_mode)

    sleep_time = delta_t - loop_execution_time

    if(sleep_time > 0):
        time.sleep(sleep_time)
