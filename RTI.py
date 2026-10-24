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

import os
# BEFORE anything imports numpy: OpenBLAS starts its worker threads at import. In RTI they were 3 extra threads
# at NORMAL priority on core 0 that the RT loop would have to wait on for any BLAS call large enough to be
# split (2026-10-01). RTI's linear algebra is 6x6 — one thread is always right. setdefault: overridable.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import time
import signal
import atexit
import faulthandler
import traceback   # used by the command-handler guard in the superloop
from collections import deque
import json
# These used to arrive through `from utility.RTI_utils import *` (2026-10-03, refactor Phase 2). Until 2026-10-15
# RTI.py also imported data_init's module LOGGER and logged through it (RTI.log labelled those lines
# `utility.data_init`); RTI now has its own named loggers, defined after `import logging` below.
from utility.data_init import initialize_robot_data, process_gripper_joint_data, process_joint_data
from utility.signal_processing import reset_rolling_stats, update_rolling_percentiles, update_rt_loop_stats, pace_deadline
from utility.RTI_utils import print_heartbeat
from project_paths import RTI_LOG, SYSTEM_XML, ROBOTS_DIR, TEMP_DATA_DIR, DATA_RECIPE_DIR, GRIPPERS_DIR, CRITICAL_LOG, FILTER_CONFIG_PATH, FLASH_PERFORMED_MARKER, CELLS_DIR, PROJECT_ROOT
from utility.code_version import describe as describe_code
from utility import sim_mode
import logging
logger = logging.getLogger("rti")  # docs/logging.md "Logger names"
log_can = logging.getLogger("rti.can")  # docs/logging.md "Logger names"
log_dynamics = logging.getLogger("rti.dynamics")  # docs/logging.md "Logger names"
log_errors = logging.getLogger("rti.errors")  # docs/logging.md "Logger names"
log_exec = logging.getLogger("rti.exec")  # docs/logging.md "Logger names"
log_filters = logging.getLogger("filters")  # docs/logging.md "Logger names"
log_flash = logging.getLogger("rti.flash")  # docs/logging.md "Logger names"
log_gripper = logging.getLogger("rti.gripper")  # docs/logging.md "Logger names"
log_handler = logging.getLogger("rti.handler")  # docs/logging.md "Logger names"
log_homing = logging.getLogger("rti.homing")  # docs/logging.md "Logger names"
log_loop = logging.getLogger("rti.loop")  # docs/logging.md "Logger names"
log_mode = logging.getLogger("rti.mode")  # docs/logging.md "Logger names"
log_recipe = logging.getLogger("rti.recipe")  # docs/logging.md "Logger names"
log_safety = logging.getLogger("rti.safety")  # docs/logging.md "Logger names"
log_startup = logging.getLogger("rti.startup")  # docs/logging.md "Logger names"
import sys
import numpy as np
from utility.robot_factory import create_robot
import threading
import zmq
from queue import Queue, Empty
from gpiozero import pi_info
from utility.logger_setup import setup_logging, apply_log_level
from utility.fault_details import fault_detail
import config.xml_parser as read_XML2
from utility.selfcheck import run_selfcheck
from utility.state_machine import RobotMode, request_mode_change, request_enable, MAINTENANCE_MODES
from utility.error_checks import (
    process_errors_tick, clear_errors, send_clear_errors, ExecLinkWatchdog,
    LoopDegradationMonitor,
    CLEAR_ERROR_SETTLE_TICKS, CLEAR_ERROR_SEND_REPEATS, BOOT_CLEAR_TICK, _WARNING_SUFFIXES,
)
from utility.activity_log import get_rti_activity_logger, log_activity, note_origin
from utility.trace_ring import TraceWriter
from robotics.hall_check import HallCheck, hall_joint_index
from robotics.free_joint import FreeJoint
from robotics.joint_test import JointTest
from robotics.homing import HomingController
from robotics.homing_arms import HomingArms
from motion.rti_handler import RTIModeHandler
from communication.rti_protocol import CLOCK_ROBOT, CLOCK_PC, CMD_SIZE

# ==========================================================================
# System and Configuration
# ==========================================================================
from config.system_config import lock_memory, locked_mb, setup_realtime_process
from utility.gc_pauses import GcPauseMonitor, freeze_startup
from hardware.can_hardware import start_can_interface, place_can_path, FirstFrameWait, FastCanReader
from config.cpu_cores import CAN_CPU_CORE, GENERAL_CPU_CORES
from utility.can_reply_monitor import CanReplyMonitor
from utility.loop_delay_meter import LoopDelayMeter

# ==========================================================================
# Hardware
# ==========================================================================
from hardware.gpio_handler import (
    initialize_gpio, update_gpio, check_estop_active, write_all_outputs, read_inputs_into_shared, configure_estop,
    set_output, set_output_high, set_output_low, toggle_output, HIGH, LOW,
    ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3
)
from hardware.motor_setup import setup_motor_system
from hardware.can_hardware import scan_can_bus, initialize_can_communication
# init_rti_receive_socket is now an alias inside config/networking.py

# ==========================================================================
# Networking
# ==========================================================================
from config.networking import RT_CPU_CORE, zmq_listener, zmq_sender_setup, udp_socket_receive_setup, udp_socket_send_setup, broadcast_udp_telemetry, init_rti_receive_socket, drain_rti_udp, RTI_PORT_SENDER, init_rti_tcp_listener, accept_rti_tcp, drain_rti_tcp, send_rti_tcp, drain_rti_tcp_backlog
from communication.can_message_handlers import receive_and_process_can_messages, handle_polling_and_overrides, update_can_connection_status, drain_can_discard
from communication.can_message_handlers import send_gripper_commands, send_motor_commands, secondary_data_send, load_motor_configs, load_gripper_configs, handle_motor_reconnect, write_motor_config_to_shared, write_gripper_config_to_shared, GRIPPER_IDLE_PACK_REPEATS
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
from data.update_shared_data import (update_motor_and_gripper_data, update_joint_and_tcp_data, update_joint_state,
                                     update_tcp_pose, update_joint_rest)
from utility.filter_config import build_filter_banks, load_filter_banks
from utility.payload import PayloadModel, apply_payload_command
from hardware.control_box import capabilities as pcb_capabilities, describe as describe_pcb
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
# ==========================================================================
# Loop state — every piece of mutable state the superloop and its command handlers share
# ==========================================================================
class LoopState:
    """One holder, `st`, for the state the RTI superloop and its ZMQ command handlers mutate.

    Until 2026-10-03 (refactor Phase 3, step 1) these were 39 module globals, written by the loop and by 14
    handlers through `global` statements; nothing showed in one place what state the loop carries. Now every
    such name is an attribute here, assigned at exactly the same points, in the same order, as before. Read-only
    configuration (joint_num, Motor, shared_data, the handlers table, ...) stays a module global on purpose.

    __slots__ so that a typo, or a new piece of state nobody listed, fails loudly at its first assignment
    instead of silently becoming a new global: NEW LOOP STATE IS ADDED HERE. Attribute access costs ~15 ns more
    than a module global; the loop makes ~40 such accesses per tick.
    """
    __slots__ = (
        "next_tick_deadline",            # absolute pacing: when the next tick is due (perf_counter seconds)
        # profiler / overrun tracer
        "_profile_count", "_profile_loop_sum", "_overrun_trace_count",
        # gripper gains and limits (the values currently loaded; set_pid_gains rewrites them)
        "gripper_KPP", "gripper_KPV", "gripper_KIV", "gripper_KPIQ", "gripper_KIIQ", "gripper_KP", "gripper_KD", "gripper_Ilim", "gripper_velocity_limit", "gripper_voltage_limit",
        # gripper firmware calibration + ownership
        "_gripper_calibrate_pending", "_gripper_calibrate_running", "_gripper_calibrate_ticks", "_gripper_was_owned",
        # filters, CAN bookkeeping, start-up
        "filter_banks", "received_ids", "latest_commands_id", "startup_selfcheck", "startup_motor_reconfig", "loop_execution_time", "prev_time",
        # mode and pending mode request (handlers write, the loop consumes)
        "current_mode", "pending_mode_request", "pending_mode_force", "pending_mode_assertion", "pending_mode_rate_hz",
        "command_id", "pending_mode_request_id",          # the activity log's command ids (2026-10-15)
        # errors
        "latched_errors", "error_metadata", "clear_error_countdown", "homing_failures",
        # loop statistics ring
        "_loop_stats_buf_pos", "_loop_stats_buf_count",
        # RTI link (pairing / transport)
        "rti_pc_ip", "rti_transport", "rti_tcp_conn",
        # freedrive walls
        "_joint_walls", "_wall_haptic",
    )


st = LoopState()


def main():
    """RTI: configure the system, build the robot, the CAN bus, the motors, the shared data and the mode handlers, then
    run the real-time superloop forever. Everything in here ran at module level until 2026-10-03 (refactor Phase 3,
    step 3): importing RTI.py started the robot, so nothing could import it — not even to read a handler. The body is
    moved verbatim: the section banners are the same, the order is the same, the command handlers are the same
    functions (now closures over this scope), and `st` (LoopState) holds the loop's mutable state as before. One
    measurable difference: the superloop reads its ~40 names as function locals instead of module globals, which is
    faster, not slower. Startup sections as separate functions are a later step, if wanted."""


    st._profile_count    = 0
    st._profile_loop_sum = 0.0

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
    st._overrun_trace_count = 0

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
        if not _OVERRUN_TRACE_ENABLED or not _tick_sections:
            return
        limit = _OVERRUN_TRACE_MS if _OVERRUN_TRACE_MS > 0 else delta_t
        if loop_total <= limit:
            return
        st._overrun_trace_count += 1
        ranked = sorted(_tick_sections.items(), key=lambda kv: kv[1], reverse=True)
        accounted = sum(_tick_sections.values())
        top = "  ".join(f"{n}={v * 1e3:.2f}ms" for n, v in ranked[:_OVERRUN_TRACE_TOP] if v > 0)
        log_loop.warning(
            f"[OVERRUN #{st._overrun_trace_count}] tick={int(tick)} mode={mode} "
            f"exec={loop_total * 1e3:.3f}ms (limit {limit * 1e3:.2f}ms) | "
            f"worst: {top} | unaccounted={(loop_total - accounted) * 1e3:.2f}ms"
        )


    def _profile_report(loop_total):
        if not _PROFILE_ENABLED:
            return

        st._profile_count += 1
        st._profile_loop_sum += loop_total
        if st._profile_count < _PROFILE_INTERVAL:
            return

        rows = []
        for name, total in _profile_sum.items():
            avg_ms = (total / st._profile_count) * 1e3
            max_ms = _profile_max.get(name, 0.0) * 1e3
            pct = (total / st._profile_loop_sum * 100.0) if st._profile_loop_sum > 0.0 else 0.0
            rows.append((total, name, avg_ms, max_ms, pct))
        rows.sort(reverse=True)

        sys.stdout.write(
            f"\n[RTI_PROFILE] last {st._profile_count} ticks "
            f"(avg exec {(st._profile_loop_sum / st._profile_count) * 1e3:.3f} ms)\n"
        )
        for _total, name, avg_ms, max_ms, pct in rows[:_PROFILE_TOP]:
            sys.stdout.write(
                f"  {name:<24s} avg={avg_ms:8.3f} ms  "
                f"max={max_ms:8.3f} ms  share={pct:5.1f}%\n"
            )
        sys.stdout.flush()

        _profile_sum.clear()
        _profile_max.clear()
        st._profile_count = 0
        st._profile_loop_sum = 0.0

    # ==========================================================================
    # Init gpio
    # ==========================================================================
    initialize_gpio()
    # Which PCB is fitted decides how many E-stop channels can be trusted (hardware/control_box.py, 2026-10-04).
    _pcb = pcb_capabilities(_sys.get('pcb_version', 1))
    configure_estop(_pcb['estop_channels'])

    # ==========================================================================
    # Setup logging
    # ==========================================================================
    # Logging if TRUE print to terminal and save to log file, If FALSE save to log file - only for this file.
    # queued=True (2026-10-03): the RT thread never writes a file — a logging call only enqueues; the listener
    # thread below does the I/O. One 18.4 ms tick (LOOP_CRITICAL) was a log write blocking on the SD card.
    _log_listener = setup_logging(str(RTI_LOG), log_to_console=True, critical_log_file=str(CRITICAL_LOG), queued=True)

    # Activity log — separate mechanism from the above (see TODO/activity_log_design.md).
    _rti_activity_logger = get_rti_activity_logger()

    # The simulated robot (utility/sim_mode.py, TODO/simulated_robot_plan.md, 2026-10-13): RCB_SIM=1 = the simulated
    # drives on the virtual bus instead of the arm, no real-time core, the real CAN chip left alone, logs and data under
    # temp_data/sim/. Refused while another RTI runs (the robot service) or without the virtual bus.
    _SIM = sim_mode.enabled()
    _CAN_IF = sim_mode.can_interface()
    # The dashboard's SIMULATION badge reads this (2026-10-13): until then the string said "ON" for ever — set once
    # when the segment was created — so the real robot's dashboard showed "SIM: ON".
    shared_data.sim_mode.set("ON" if _SIM else "OFF")
    if _SIM:
        _why = sim_mode.refuse_reason()
        if _why:
            print(f"[SIM] RTI refused: {_why}", file=sys.stderr, flush=True)
            sys.exit(2)

    # RT scheduling + CPU affinity. Runs AFTER setup_logging so the verified
    # RT-state line lands in RTI.log instead of disappearing.
    # enforce (2026-10-03): <realtime_required> in system.xml, default 1 — a control loop without SCHED_FIFO is
    # not the robot anyone tested; refusing is loud and safe (the arm never moves), as industrial real-time interfaces do.
    if not _SIM:
        setup_realtime_process(cpu_core=RT_CPU_CORE, priority=99, enforce=bool(_sys.get('realtime_required', 1)))
        # Keep the loop's memory in RAM (2026-10-16: a memory shortage dropped RTI's code pages and stalled it 194 s).
        # Never raises; without the service's LimitMEMLOCK it logs an ERROR and the loop runs as before. The result
        # goes to shared memory for the health summary (MB locked, NaN = locked but not measured; 0 = not locked;
        # -1 = simulation, not tried).
        _locked = lock_memory()
        shared_data.rti['memory_locked_mb'][0] = (locked_mb() or float('nan')) if _locked else 0.0
    else:
        shared_data.rti['memory_locked_mb'][0] = -1.0
    # The logging thread starts HERE, after the RT scheduling, so it inherits SCHED_FIFO 99 on core 3 like the
    # ZMQ listener thread does (2026-10-01): a Python thread at normal priority on another core can be preempted
    # while holding the GIL, and the loop would wait for it. At FIFO 99 on core 3 it only runs while the loop
    # sleeps; its file writes release the GIL, so a blocked write stalls the thread, not the loop. Stopped at exit
    # (atexit) so everything queued — including an uncaught exception's traceback — reaches the files.
    if _log_listener is not None:
        _log_listener.start()
        atexit.register(_log_listener.stop)
    # Logged only now, never before setup_logging: a root logging.info() with no handler makes Python install its
    # own stderr handler, and setup_logging then saw "already configured" and set up NOTHING — no RTI.log, INFO
    # dropped, writes unqueued in the RT thread — from 2026-10-04 17:52 until this fix (2026-10-05).
    log_startup.info(f"control box {describe_pcb(_sys.get('pcb_version', 1))}")
    # Which commit this RTI runs (2026-10-13; tools/arm_smoke.py records it) — read from the .git files, no subprocess.
    log_startup.info(f"code: {describe_code(PROJECT_ROOT)}")
    if _SIM:
        log_startup.warning(f"SIMULATION — the simulated drives on {_CAN_IF}, NOT the arm; no real-time core or "
                        f"priority; the I/O simulated (no real pin read or written); logs and data under "
                        f"{TEMP_DATA_DIR.parent}")
    # When a script crashes with exception we write the error to log file
    def handle_exception(exc_type, exc_value, exc_traceback):
        logger.critical("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))
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

    # --- Create and start the thread
    command_queue = Queue()
    threading.Thread(target=zmq_listener, args=(command_queue,), daemon=True).start()

    # --- Setup ZMQ response sender
    status_socket = zmq_sender_setup()

    # --- RTI socket receive ---
    RTI_socket_receive = udp_socket_receive_setup()

    # --- RTI socket sender: RTI and telemetry---
    RTI_socket_sender, telemetry_socket = udp_socket_send_setup()

    # Listening socket for the optional SOFT_RTI TCP transport. Bound at startup because it is
    # cheap and idle until a PC dials in; nothing accepts a connection unless rti_connect has
    # named an allowed IP and asked for "tcp".
    RTI_tcp_listener = init_rti_tcp_listener()

    # ==========================================================================
    # Get robot data from XML
    # ==========================================================================
    xml_path    = str(_sys['robot_xml_path'])

    # Load all robot/gripper parameters in one call
    cfg = read_XML2.load_full_robot_config(xml_path, grippers_dir=GRIPPERS_DIR, active_gripper=_sys['active_gripper'])

    # Validate XML config — abort immediately if any required field is broken
    _cfg_ok, _cfg_issues = read_XML2.validate_robot_config(cfg)
    if not _cfg_ok:
        for _msg in _cfg_issues:
            log_startup.critical(f"{_msg}")
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
    master_position       = cfg['master_position']
    gear_ratio            = cfg['gear_ratio']
    gear_efficiency       = cfg['gear_efficiency']
    motor_offset          = cfg['motor_offset']
    motor_dir             = cfg['motor_dir']
    motor_signs           = 1 - 2 * motor_dir  # 0→1, 1→−1

    gripper_kt            = cfg['gripper_kt']
    gripper_gear_r        = cfg['gripper_gear_r']
    st.gripper_Ilim          = cfg['gripper_Ilim']
    st.gripper_voltage_limit = cfg['gripper_voltage_limit']   # mV (0 = use VBUS)
    st.gripper_velocity_limit = cfg['gripper_velocity_limit']
    st.gripper_KPP           = cfg['gripper_KPP']
    st.gripper_KPV           = cfg['gripper_KPV']
    st.gripper_KIV           = cfg['gripper_KIV']
    st.gripper_KIIQ          = cfg['gripper_KIIQ']
    st.gripper_KPIQ          = cfg['gripper_KPIQ']
    st.gripper_KP            = cfg['gripper_KP']
    st.gripper_KD            = cfg['gripper_KD']

    delta_t               = cfg['delta_t']

    # Build homing controller from XML config
    homing_ctrl = HomingController.from_config(cfg, joint_num)
    # 2026-10-09 (TODO/homing_plan.md H1): which arm is connected, and did every joint home where THIS arm's joint always
    # homes (a false stall / skipped belt fails homing). The table is read here; a worker thread off this core re-reads it
    # when a homing starts and writes it when one ends — the loop itself touches no file.
    homing_ctrl.arm_check = HomingArms(
        TEMP_DATA_DIR / "homing_arms.json", cfg['gear_ratio'][:joint_num], cfg['home_offset_gripper_dependent'][:joint_num],
        [2 ** int(b) for b in cfg['motor_encoder'][:joint_num]], cfg['gripper_name'], prefix=cfg['robot_name'])
    homing_ctrl.arm_check.start()

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

    def _freeze_gripper(why):
        """Hold the jaws where they are (2026-10-24, user: a stop "should freeze it also"). The firmware-mode target
        becomes the measured jaw position, so a gripper still travelling to its last target stops there. Called on a
        stop (the executor's gripper_hold), a disable and an error latching. An idle gripper (action 0) stays idle; in
        motor mode RTI keeps no gripper target, so there is nothing to hold."""
        if not active_CAN_gripper or not int(_gripper_fw_state.get('action', 0)):
            return
        pos = max(0, min(255, int(round(float(shared_data.gripper['gripper_position'][0])))))
        if pos != int(_gripper_fw_state['position']):
            log_gripper.info("jaws held at %d (their target was %d): %s", pos, _gripper_fw_state['position'], why)
            _gripper_fw_state['position'] = pos

    _gripper_err_prev = [False]          # the error pass's last error_active: an error latching holds the jaws
    # Calibrate state machine. After Send_gripper_calibrate fires, the bus must be
    # held with *empty* Send_gripper_data_pack() polls — resuming the full data
    # pack mid-calibration cancels it. We don't try to auto-detect completion
    # The user interrupts the state by clicking Open/Close/Goto/Stop, which aborts the calibration via the
    # gripper_set handler. The hard timeout is a safety backstop only.
    st._gripper_calibrate_pending = False
    st._gripper_calibrate_running = False
    st._gripper_calibrate_ticks   = 0
    # Tracks the gripper CAN slot changing hands, so RTI can tell the firmware to release
    # when homing gives it back. See the handover block in the superloop.
    st._gripper_was_owned = False
    _GRIPPER_CALIBRATE_MAX_TICKS = max(1, round(30.0 / delta_t))  # safety timeout


    # ==========================================================================
    # Setup robot
    # ==========================================================================
    robot = create_robot(robot_name, gripper_name=name_of_the_gripper)

    _kinematics_backend = os.environ.get("RTI_KINEMATICS_BACKEND", "auto").strip().lower()
    set_kinematics_backend(_kinematics_backend)
    if _kinematics_backend == "rtb":
        log_startup.info("Kinematics backend: robotics-toolbox")
    else:
        try:
            attach_pinocchio(robot)
            log_startup.info(f"Kinematics backend: {_kinematics_backend} (Pinocchio attached)")
        except Exception:
            if _kinematics_backend == "pinocchio":
                raise
            log_startup.exception(
                "Pinocchio attach failed; falling back to robotics-toolbox kinematics"
            )

    # ==========================================================================

    # ==========================================================================
    # Start / Restart CAN interface when starting this script
    # ==========================================================================
    if _SIM:
        log_startup.warning(f"{_CAN_IF}: no bring-up (bit rate, bus-off restart) and no CAN core placement — a "
                        f"virtual bus; the real can0 is left alone")
    else:
        start_can_interface()
        # The CAN chip's interrupt threads are created when can0 comes up: give them core 2 alone and keep interactive
        # programs on the general cores — on RTI's core they froze all CAN traffic while RTI's tick ran, on a shared
        # core they stalled under load (2026-10-12, api_tests/69, config/cpu_cores.py).
        place_can_path(CAN_CPU_CORE, GENERAL_CPU_CORES)
    communication1 = initialize_can_communication(bustype='socketcan', channel=_CAN_IF, bitrate=1000000)
    # TODO/can_tx_timing_plan.md option B (read first, send early): the waiter's state IS the switch — ON at start when
    # system.xml says <can_read_first>1 (the default since 2026-10-12, §9.11; below, once the meter exists),
    # `can_read_first:1|0` at runtime.
    first_frame_wait = FirstFrameWait(_CAN_IF)
    # Option B's CAN read: python-can's receive without its per-frame select() and Message object, on the same socket
    # (hardware/can_hardware.FastCanReader, 2026-10-12).
    fast_can_reader = FastCanReader(communication1)
    # The control loop's delay per joint (a reply -> the command computed from it), sampled live for the GUI's RT
    # Diagnostics (2026-10-12, utility/loop_delay_meter.py): a read-only socket that sees our own frames leave.
    loop_delay_meter = LoopDelayMeter(joint_num, interface=_CAN_IF)
    shared_data.rti["loop_delay"][:] = np.nan          # not measured yet (the GUI shows a dash)
    shared_data.rti["loop_delay_max"][:] = np.nan
    if _sys['can_read_first']:
        first_frame_wait.set_enabled(True)      # logs "[CAN] read-first ON"; a socket that cannot open: the old order
    else:
        log_can.info("read-first OFF at start (system.xml <can_read_first>0): the old tick order")

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
    st.filter_banks, _filter_src = load_filter_banks(FILTER_CONFIG_PATH, joint_num, delta_t)
    log_filters.info(f"loaded from {_filter_src}: " +
                ", ".join(f"{s}={describe_bank(b)}" for s, b in st.filter_banks.items()))

    st.received_ids       = motor_system['housekeeping']['received_ids']
    st.latest_commands_id = motor_system['housekeeping']['latest_commands_id']
    node_error_bits    = motor_system['housekeeping']['node_error_bits']

    motor_error_bytes  = motor_system['control']['motor_error_bytes']
    st.startup_selfcheck  = motor_system['housekeeping']['startup_selfcheck']
    st.loop_execution_time  = motor_system['housekeeping']['loop_execution_time']


    # Zero out shared memory housekeeping arrays
    shared_data.rti["scan_received_ids"][:]         = np.zeros(16, dtype=int)
    shared_data.rti["CAN_node_connection_error"][:] = np.zeros(16, dtype=int)
    shared_data.rti["old_data_warning"][:]          = np.zeros(16, dtype=int)
    shared_data.rti["homing_status"][:]             = 0
    shared_data.rti["joint_homed"][:]               = 0
    shared_data.rti["gravity_comp_enabled"][0]      = 1.0
    # Runtime payload (utility/payload.py): the gripper alone at every start — a payload is a program's claim.
    _payload_model = PayloadModel(robot)
    for _k in ("payload_mass", "payload_com_x", "payload_com_y", "payload_com_z"):
        shared_data.rti[_k][0] = 0.0
    shared_data.rti["friction_comp_enabled"][0]     = 0.0   # operator switch: OFF at every start (utility/friction_comp.py)
    # inertia / Coriolis feedforward (utility/mode_dispatch.py _add_dynamics_ff): the start state from system.xml
    # <dynamics_feedforward> (2026-10-10, user: both ON by default)
    _dff = _sys.get("dynamics_feedforward", {"inertia": True, "coriolis": True})
    shared_data.rti["inertia_comp_enabled"][0]      = 1.0 if _dff.get("inertia", True) else 0.0
    shared_data.rti["coriolis_comp_enabled"][0]     = 1.0 if _dff.get("coriolis", True) else 0.0
    # friction feedforward (utility/mode_dispatch.py _add_friction_ff, 2026-10-17): OFF unless system.xml says <friction>1
    shared_data.rti["friction_ff_enabled"][0]       = 1.0 if _dff.get("friction", False) else 0.0
    log_dynamics.info(f"dynamics feedforward at start: inertia {'ON' if _dff.get('inertia', True) else 'off'}, "
                f"Coriolis {'ON' if _dff.get('coriolis', True) else 'off'}, friction "
                f"{'ON' if _dff.get('friction', False) else 'off'} (system.xml <dynamics_feedforward>)")

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
    # Every tick's values into a ring in shared memory, for recorder.py — the incident dumps and captures are made THERE,
    # with its own interpreter (utility/trace_ring.py, 2026-10-16: made in RTI's process they cost loop time). The same
    # arrays and names as the history entry (utility/data_init.process_joint_data); each keeps its identity for the run.
    trace = TraceWriter((
        ("j_p", shared_data.joint['joint_position']), ("j_s", shared_data.joint['joint_speed']),
        ("j_t", shared_data.joint['joint_torque']), ("j_a", shared_data.joint['joint_acceleration']),
        ("j_j", shared_data.joint['joint_jerk']), ("j_s_f", shared_data.joint['joint_speed_filtered']),
        ("j_t_f", shared_data.joint['joint_torque_filtered']), ("j_a_f", shared_data.joint['joint_acceleration_filtered']),
        ("j_j_f", shared_data.joint['joint_jerk_filtered']), ("TCP_p", shared_data.tcp['robot_pose']),
        ("TCP_p_SE3", shared_data.tcp['robot_pose_matrix']), ("TCP_p_quat", shared_data.tcp['robot_pose_quaternion']),
        ("TCP_s", shared_data.tcp['TCP_speed']), ("TCP_s_f", shared_data.tcp['TCP_speed_filtered']),
        ("TCP_a", shared_data.tcp['TCP_acceleration_raw']), ("TCP_a_f", shared_data.tcp['TCP_acceleration_filtered']),
        ("t_ext", shared_data.tcp['torque_external']), ("TCP_f_ext", shared_data.tcp['force_external']),
        ("TCP_f_ext_tool", shared_data.tcp['force_external_tool']),
        ("j_pc", shared_data.joint['joint_position_commanded']), ("j_sc", shared_data.joint['joint_speed_commanded']),
        ("j_ac", shared_data.joint['joint_acceleration_commanded']), ("j_tc", shared_data.joint['joint_torque_commanded']),
        ("TCP_p_c", shared_data.tcp['TCP_pose_commanded']), ("TCP_s_c", shared_data.tcp['TCP_speed_commanded']),
        ("j_pt", shared_data.joint['joint_position_target']), ("j_st", shared_data.joint['joint_speed_target']),
        ("j_at", shared_data.joint['joint_acceleration_target']), ("j_tt", shared_data.joint['joint_torque_target']),
        ("j_t_inertia", shared_data.joint['joint_inertia_torque']), ("j_t_coriolis", shared_data.joint['joint_coriolis_torque']),
        ("TCP_p_t", shared_data.tcp['TCP_pose_target']), ("TCP_s_t", shared_data.tcp['TCP_speed_target']),
        ("j_err", motor_error_bytes),
    ), tick_s=delta_t)


    scheduled_state = {"step": 0,"joint_index": 0,"device_info_counter": 0}
    # Optional override function (set externally when needed)
    scheduled_override_queue = deque()  # Each item is a tuple: (function, repeat_count)
    current_override = {"fn": None,"count": 0}


    # ==========================================================================
    # Preload motor driver configs. NOTE this needs to be preloaded after
    # Every fatal error or disconnect because motor drivers might lose this data
    # Same goes for gripper configs if we have CAN gripper
    # ==========================================================================
    # Every drive's CAN watchdog: ONE value, system.xml <drive_watchdog_ms> (2026-10-13; the robot / gripper XML
    # <watchdog_rate> — 5000 ms, a bring-up value — is no longer read). A drive that hears no command or poll for this
    # long goes idle, so none keeps running RTI's last speed or torque once RTI is gone.
    motor_watchdog_rate   = [_sys['drive_watchdog_ms']] * joint_num
    gripper_watchdog_rate = _sys['drive_watchdog_ms']
    log_can.info(f"drive watchdog {_sys['drive_watchdog_ms']} ms on every drive (system.xml <drive_watchdog_ms>)")

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
            velocity_limit=st.gripper_velocity_limit, current_limit=st.gripper_Ilim,
            KP=st.gripper_KP, KD=st.gripper_KD, KPIQ=st.gripper_KPIQ, KIIQ=st.gripper_KIIQ,
            KPV=st.gripper_KPV, KIV=st.gripper_KIV, KPP=st.gripper_KPP,
            watchdog_action="Idle", num_repeats=num_repeats,
            voltage_limit=st.gripper_voltage_limit, pace_s=pace_s)
        write_gripper_config_to_shared(
            shared_data,
            KPP=st.gripper_KPP, KPV=st.gripper_KPV, KIV=st.gripper_KIV,
            KPIQ=st.gripper_KPIQ, KIIQ=st.gripper_KIIQ, KP=st.gripper_KP, KD=st.gripper_KD,
            Ilim=st.gripper_Ilim, velocity_limit=st.gripper_velocity_limit,
            voltage_limit=st.gripper_voltage_limit)

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

    # The start-up clear (2026-10-05) is sent in the loop at tick BOOT_CLEAR_TICK since 2026-10-13: the drivers keep the
    # watchdog trip the LAST run's end left them (the Kt replies above carry err=1 from every node — before the clear,
    # the gripper latched GRIPPER_ERROR at tick 1 of every start), and with a 100 ms watchdog the set-up below trips
    # them again before tick 1, so a clear sent here would be undone before the loop starts.

    # ==========================================================================
    # Initialize the timer before the loop starts
    # ==========================================================================
    st.prev_time = time.perf_counter()
    st.next_tick_deadline = time.perf_counter()   # absolute pacing (pace_deadline): the first tick is due now + delta_t
    shared_data.rti["loop_tick"][0] = 0
    st.current_mode = RobotMode.BOOTING
    st.pending_mode_request = None
    # The id of the command being handled (its "id" field), and of the command that made the pending mode request
    # (2026-10-15): the activity log's RTI entries caused by a command carry its id, as the executor's do. A mode
    # change RTI makes on its own (start-up done, an error, homing finished) keeps a null id.
    st.command_id = None
    st.pending_mode_request_id = None
    # Set alongside pending_mode_request by "set_mode:<MODE>:PARKED|FORCE". Only
    # maintenance modes (FLASHING) read these. There is no automatic test for "can
    # this pose survive losing torque", so entry requires an explicit operator
    # assertion; the assertion string is kept only so the log says which was made.
    st.pending_mode_force = False
    st.pending_mode_assertion = ""
    # SOFT_RTI only: the PC's command rate in Hz, used to size the command timeout on entry.
    st.pending_mode_rate_hz = None
    shared_data.robot_mode.set(st.current_mode)
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
    # RTI-mode telemetry. Written while EITHER RTI contract is active (RTI or SOFT_RTI),
    # and shared
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
    # Self-collision fields (the worker writes them; zero so a stale run never looks live): no hit, no block, heartbeat 0.
    for _k in ("sc_heartbeat", "sc_hit", "sc_pair_a", "sc_pair_b", "sc_min_gap", "jog_session", "sc_ray_session", "sc_ray_free", "sc_ray_done"):
        shared_data.rti[_k][0] = 0
    shared_data.rti["sc_pair_gaps"][:] = np.inf                     # no pair within the margin (0 would mean contact)
    shared_data.rti["jog_dir"][:] = 0.0                             # no jog
    st.startup_selfcheck = 0 # Flag to run startup selfcheck only once at startup
    st.startup_motor_reconfig = 0 # Counts reconfiguration passes (0→1→2→3 = done)

    # ==========================================================================
    # Graceful shutdown — cleanup shared memory on exit / SIGTERM
    # ==========================================================================
    atexit.register(shared_data.cleanup)

    # 2026-10-06: a stop still hangs now and then until systemd's 90 s SIGKILL (10-03 09:43, 10-06 16:12 — the latter
    # with the two SIGTERMs' handlers NESTED and no "atexit reached"; nothing names where it waits). So the handler
    # arms faulthandler's C-level timer first: still alive SHUTDOWN_WATCHDOG_S later, every thread's Python stack goes
    # to stderr (logs/RTI.out.log) and the process _exits — it fires even if Python is deadlocked. A clean stop takes
    # under a second; a hung one now restarts in ~10 s instead of 90 and says where it hung.
    SHUTDOWN_WATCHDOG_S = 10.0

    def _handle_sigterm(_signum, _frame):
        faulthandler.dump_traceback_later(SHUTDOWN_WATCHDOG_S, exit=True)
        logger.info("SIGTERM received — shutting down gracefully.")
        shared_data.cleanup()
        logger.info("shared memory released — raising SystemExit.")   # breadcrumb (2026-10-03 hang)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _handle_sigterm)
    # 2026-10-03: one service stop hung for 90 s after "SIGTERM received" with nothing logged after it (systemd then
    # SIGKILLed RTI). The breadcrumbs above/below say how far a shutdown got; and `kill -USR1 <pid>` makes a hung RTI
    # write every thread's Python stack to stderr (logs/RTI.out.log) — the standard library's faulthandler, no cost.
    faulthandler.register(signal.SIGUSR1, all_threads=True)
    atexit.register(lambda: logger.info("atexit reached — interpreter shutting down."))

    # ==========================================================================
    # Error tracking state — latched faults, metadata, clear countdown
    # ==========================================================================

    # Holds all errors that have appeared since the last user-requested clear.
    # Errors only leave this set when the user calls "Clear Errors" from the GUI
    # or sends the "clear_errors" ZMQ command.
    st.latched_errors: set = set()
    st.error_metadata: dict = {}       # key → {"timestamp": float, "tick": int, "description": str, "level": str}
    # Homing failures the controller reported this tick, {key: reason} (2026-10-14): handed to the error pass, which
    # latches them as hard errors with the reason as their description; emptied right after.
    st.homing_failures: dict = {}
    st.clear_error_countdown: int = 0  # counts down to 0 each time a clear is requested

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
    _can_age_buf = np.zeros(3, dtype=np.float64)        # + [2]: time.time() of the read (telemetry can_read_t)
    # The data replies the CAN read took from each joint this tick, and the warning when a joint's replies miss the
    # read too often (2026-10-12, utility/can_reply_monitor.py). The read in tick k holds the answers to tick k-1's
    # commands (they go out while RTI sleeps), so the monitor is told whether the PREVIOUS tick sent them.
    _can_reply_buf = [0] * joint_num
    _can_reply_time = np.full(joint_num, np.nan)        # each joint's newest data reply's arrival (the loop-delay meter)
    _loop_delay_published = -1
    can_reply_monitor = CanReplyMonitor(joint_num, delta_t)
    _can_sent_prev = False
    _can_sent_now = False         # option B reads before the first dispatch has set it
    Hall_index = additional_pin2 = Edge_detect = 0   # bound before the loop: _can_read() keeps them across ticks
    _bus_silent = False
    _profile_last = 0.0
    _loop_period_buf = np.zeros(LOOP_STATS_BUFFER_SIZE, dtype=np.float64)
    _loop_exec_buf = np.zeros(LOOP_STATS_BUFFER_SIZE, dtype=np.float64)
    st._loop_stats_buf_pos = 0
    st._loop_stats_buf_count = 0

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
    # Reconnect STORM GUARD (communication/can_message_handlers.handle_motor_reconnect): a recovered node's config
    # reload is pending until served — one node per tick, each at most once per RECONNECT_RELOAD_COOLDOWN_S.
    RECONNECT_RELOAD_COOLDOWN_S = 2.0
    _reconnect_last_tick = np.full(joint_num + 1, -(10 ** 9), dtype=np.int64)
    _reconnect_pending = np.zeros(joint_num + 1, dtype=bool)
    _reconnect_cooldown_ticks = int(round(RECONNECT_RELOAD_COOLDOWN_S / delta_t))

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

    # The Hall sensor check (robotics/hall_check.py, 2026-10-13): the robot XML's <has_hall_sensor> joint, its live
    # limits (stop() restores them; PID apply edits these arrays in place).
    hall_check = HallCheck(Motor, hall_joint_index(cfg['has_hall_sensor'], joint_num), motor_velocity_limit, motor_Ilim,
                           ctx, shared_data.rti, shared_data.joint['joint_position'], log=log_homing.info)
    # One joint freed for hand turning (robotics/free_joint.py, 2026-10-18; the first-run check's S3) — the same live
    # limits as the Hall check; never both at once.
    free_joint = FreeJoint(Motor, joint_num, motor_velocity_limit, motor_Ilim, shared_data.rti, log=log_homing.info)
    # A small powered test move of one joint, homed or not (robotics/joint_test.py, 2026-10-18; the first-run check's
    # stage 3) — the same live limits; never beside the Hall check or a freed joint.
    joint_test = JointTest(Motor, Joint, joint_num, motor_velocity_limit, motor_Ilim, ctx, shared_data.rti, delta_t,
                           q_meas=shared_data.joint['joint_position'],
                           soft_lo=cfg['software_joint_limit_negative'], soft_hi=cfg['software_joint_limit_positive'],
                           log=log_homing.info,
                           done=lambda text: log_activity(_rti_activity_logger, "RTI", "info", "JOINT_TEST", message=text))

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
            # the collision worker's free length along the jog's ray limits the jog (exact stop at the jog margin)
            'self_collision': bool(_sys['self_collision']['enabled']),
        },
        motor_dynamics={
            'gear_ratio':      gear_ratio,
            'gear_efficiency': gear_efficiency,
            'motor_kt':        motor_kt,
            'motor_signs':     motor_signs,
        },
        shared_data=shared_data,
        # EXEC's controlled pause / stop (motion/exec_handler.py, 2026-10-13): brake along the planned path at the
        # hardware ceilings (the limits hard RTI's controlled stops use), speed back up at EXEC's own acceleration.
        exec_limits={
            'stop_accel':   cfg['joint_acceleration_limit'],
            'stop_jerk':    cfg['joint_jerk_limit'],
            'resume_accel': cfg['exec_acceleration_limit'],
        },
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
        # config/xml_parser/robot.py — so these are never NaN. A NaN would silently disable clamping for
        # that joint, since every comparison against it is False.
        vel_limit=cfg['rti_velocity_limit'],
        accel_limit=cfg['rti_acceleration_limit'],
        jerk_limit=cfg['rti_jerk_limit'],
        torque_rate_limit=cfg['rti_torque_rate_limit'],
        # The soft joint limits. Already loaded and already handed to JogHandler — RTI simply
        # never asked for them, so nothing bounded an RTI command to the workspace.
        limit_neg=cfg['software_joint_limit_negative'],
        limit_pos=cfg['software_joint_limit_positive'],
        # TCP translational speed cap, from <safety><max_tcp_velocity> in the robot XML.
        # 0 (the shipped default) = disabled; joint velocity limits do not bound TCP speed
        # near singularities, which is what this exists for. Runtime: rti_set_tcp_vel_limit.
        tcp_vel_limit=cfg['safety']['max_tcp_velocity'],
        # Operator's ceiling on how far a streaming PC may widen the SOFT_RTI watchdog, from
        # <safety><max_soft_rti_timeout>. Hard RTI ignores it: its 40 ms can only be tightened.
        soft_timeout_ceiling=cfg['safety']['max_soft_rti_timeout'],
        # RTI policy from <rti_defaults> in config/system.xml — see _parse_rti_defaults.
        # Robot-INDEPENDENT tuning; the per-joint physical limits above come from the robot XML.
        **{k: _sys['rti_defaults'][k] for k in (
            'command_timeout_s', 'stopping_hold_s', 'start_pose_tol_rad',
            'lowpass_cutoff_hz', 'success_window_ticks', 'success_warn', 'success_bad')},
    )
    MODE_HANDLERS[RobotMode.RTI] = rti_handler
    # ONE handler instance for both contracts. Not two: every safety layer the handler owns —
    # soft joint limits, brake-at-limits, the TCP velocity cap, gravity comp, start-pose
    # validation — is then shared by construction rather than by discipline. The contract
    # difference (timeout, generator, transport semantics) is applied by
    # rti_handler.apply_mode_contract() on mode entry, below.
    MODE_HANDLERS[RobotMode.SOFT_RTI] = rti_handler

    # Safety monitor — the independent check on what the arm ACTUALLY does, every tick, every mode
    # (utility/safety_monitor.py; TODO/safety_stack_research.md). Replaces run_safety_checks(), which
    # was never called. Reads FIXED views of the shared-memory arrays, so a tick allocates nothing.
    # Policy from <safety_monitor> in config/system.xml; robot numbers from the robot XML.
    from utility.safety_monitor import SafetyMonitor
    # Safety zones of the active cell (config/cells/<active_cell>.xml, docs/safety_zones.md). A bad cell
    # file STOPS RTI here, loudly: running without the zones the operator configured would silently
    # disarm them. Empty <active_cell> = no zones.
    _GRIPPER_XML = GRIPPERS_DIR / f"{_sys['active_gripper']}.xml"
    try:
        _zone_setup = read_XML2.load_zone_setup(SYSTEM_XML, CELLS_DIR, _GRIPPER_XML)
    except (ValueError, FileNotFoundError) as _e:
        log_safety.critical(f"safety zones: {_e} — fix the cell file (or empty <active_cell>), then restart")
        raise
    # Freedrive joint walls (utility/haptic_walls.py JOINT WALLS): per-joint specs from the robot XML
    # (<freedrive_wall>), resolved against the <haptic_walls> policy and each joint's smallest inertia. The live
    # specs change with set_joint_walls (dashboard Walls tab) and are re-resolved on reload_zones.
    from utility.haptic_walls import (HapticWalls, effective_inertia_range, make_effective_inertia, make_inverse_mass,
                                      resolve_joint_walls)
    # An endstop is what a joint WITHOUT a Hall sensor homes against. J6 (Hall) has none.
    _has_endstop = np.nan_to_num(np.asarray(cfg['has_hall_sensor'], dtype=float), nan=0.0)[:joint_num] < 0.5
    _wall_specs = [dict(s) for s in cfg['freedrive_walls'][:joint_num]]
    # Effective inertia each joint's wall pushes (other joints free), from a Pinocchio DYNAMICS model with its own Data
    # (used by this thread only): the per-tick pose-dependent scale, and its lightest/heaviest pose for the settings.
    _wall_inertia_fn = make_effective_inertia(robot)
    _wall_minv_fn = make_inverse_mass(robot)              # the zone walls' own (one Pinocchio Data per fn)
    _wall_inertia_range = effective_inertia_range(_wall_inertia_fn, cfg['software_joint_limit_negative'],
                                                  cfg['software_joint_limit_positive'])
    _wall_inertia, _wall_inertia_max = _wall_inertia_range if _wall_inertia_range is not None else (None, None)


    def _resolve_walls(haptic):
        return resolve_joint_walls(_wall_specs, haptic, cfg['software_joint_limit_negative'],
                                   cfg['software_joint_limit_positive'], cfg['joint_limit_negative'],
                                   cfg['joint_limit_positive'], _has_endstop, _wall_inertia, _wall_inertia_max)


    def _log_walls(walls, what):
        on = [w for w in walls if w['enabled']]
        log_safety.info(f"freedrive joint walls {what}: " + (", ".join(
            f"J{w['joint'] + 1} {w['at']} {np.degrees(w['lo']):.1f}/{np.degrees(w['hi']):.1f} deg "
            f"{w['f']:.2f} Hz zeta {w['zeta']:.2f} filter {1000 * w['tau']:g} ms ({np.radians(w['k']):.3g}..{np.radians(w['k']) * w['scale_max']:.3g} "
            f"Nm/deg {'by pose' if w.get('adaptive', True) else 'FIXED'}) max {w['tmax']:.2f} Nm "
            f"bt {np.degrees(w['bt']):.0f} deg {w['freedrive']}"
            for w in on) or "OFF"))
        for w in walls:
            if w['note']:
                log_safety.warning(f"J{w['joint'] + 1} freedrive wall: {w['note']}")


    # The external torque WHILE MOVING (2026-10-16, utility/momentum_observer.py): a momentum observer on the dynamics
    # model, minus the measured sliding friction; written to tcp['torque_external'] (and the TCP force from it) each
    # tick after the send. Without the model's terms: the static estimate (measured - gravity), as before.
    from utility.momentum_observer import MomentumObserver
    ext_observer = MomentumObserver(joint_num, friction_coulomb=cfg['friction_coulomb_nm'],
                                    friction_viscous=cfg['friction_viscous_nm_s'], friction_load=cfg['friction_load_per_nm'])
    log_dynamics.info(f"external torque: momentum observer, gain {ext_observer.gain:g}/s, friction "
                      + ext_observer.model.describe() + " Nm"
                      + ("" if getattr(robot, "pinocchio_momentum", None) is not None
                         else " — NO dynamics model: the static estimate (measured - gravity, right only at rest)"))
    st._wall_haptic = _zone_setup['haptic']
    st._joint_walls = _resolve_walls(st._wall_haptic)
    safety_monitor = SafetyMonitor(
        joint_num, delta_t,
        q_meas=shared_data.joint['joint_position'][:joint_num],
        q_cmd=shared_data.joint['joint_position_commanded'][:joint_num],
        v_filt=shared_data.joint['joint_speed_filtered'][:joint_num],
        tcp_v_filt=shared_data.tcp['TCP_speed_filtered'],
        homing_status=shared_data.rti['homing_status'][:joint_num],
        soft_lo=cfg['software_joint_limit_negative'], soft_hi=cfg['software_joint_limit_positive'],
        hard_lo=cfg['joint_limit_negative'], hard_hi=cfg['joint_limit_positive'],
        has_endstop=_has_endstop,
        vel_ceiling=cfg['joint_velocity_limit'],
        tolerance_rad=np.radians(cfg['safety']['joint_limit_tolerance_deg']),
        endstop_tolerance_rad=np.radians(cfg['safety']['endstop_tolerance_deg']),
        tracking_limit_rad=np.radians(np.nan_to_num(np.asarray(cfg['tracking_limit_deg'], dtype=float), nan=0.0)),
        tcp_limit=cfg['safety']['max_tcp_velocity'],
        policy=_sys['safety_monitor'],
        tcp_pose=shared_data.tcp['robot_pose_matrix'],
        zones=_zone_setup['zones'], tool_spheres=_zone_setup['tool_spheres'],
        haptic=_zone_setup['haptic'], joint_walls=st._joint_walls,
        i_meas=shared_data.motor['motor_current'][:joint_num],
        v_cmd=shared_data.joint['joint_speed_commanded'][:joint_num],
        hold_osc_ma=cfg['hold_oscillation_ma'],
        # the collision worker's fields (self_collision_worker.py): SELF_COLLISION is judged on them, never computed here
        sc={"hit": shared_data.rti["sc_hit"], "heartbeat": shared_data.rti["sc_heartbeat"], "pair_a": shared_data.rti["sc_pair_a"],
            "pair_b": shared_data.rti["sc_pair_b"], "gap": shared_data.rti["sc_min_gap"],
            "pair_gaps": shared_data.rti["sc_pair_gaps"]},
        sc_policy=_sys['self_collision'],
        # collision detection (2026-10-16): the observer's estimate against the robot XML's thresholds; the observer
        # itself so the check waits while it (re)starts
        t_ext=shared_data.tcp['torque_external'][:joint_num],
        contact_nm=cfg['contact_torque_nm'], collision_nm=cfg['collision_torque_nm'], ext_observer=ext_observer,
    )
    _COLLISION_MODE_CODE = {"off": 0.0, "warn": 1.0, "stop": 2.0}
    _COLLISION_MODE_NAME = {0: "off", 1: "warn", 2: "stop"}
    shared_data.rti['collision_mode'][0] = _COLLISION_MODE_CODE[
        safety_monitor.collision_mode if safety_monitor.check_collision else "off"]

    # Haptic zone walls for IDLE gravity-comp freedrive (utility/haptic_walls.py). The TCP Jacobian is not in
    # shared memory, so RTI keeps its own buffer, refreshed every tick by the measurement (with option B: in IDLE by
    # update_tcp_pose before the send, then by update_joint_rest after it).
    tcp_jacobian = np.zeros((6, joint_num))
    haptic_walls = HapticWalls(
        joint_num, delta_t, tcp_pose=shared_data.tcp['robot_pose_matrix'], tcp_twist_raw=shared_data.tcp['TCP_speed'],
        jacobian=tcp_jacobian, policy=_zone_setup['haptic'],
        zones=_zone_setup['zones'], tool_spheres=_zone_setup['tool_spheres'],
        q=shared_data.joint['joint_position'][:joint_num], qd_raw=shared_data.joint['joint_speed'][:joint_num],
        joint_walls=st._joint_walls, inertia_fn=_wall_inertia_fn, minv_fn=_wall_minv_fn)
    ctx.haptic_walls = haptic_walls
    # Friction compensation in freedrive (utility/friction_comp.py): the measured friction / gravity error per joint
    # (robot XML) and the <friction_comp> policy; only acts while the operator switch is ON.
    from utility.friction_comp import FrictionComp
    ctx.friction_comp = FrictionComp(joint_num, cfg['friction_static_nm'], cfg['gravity_error_nm'],
                                     _sys['friction_comp'], friction_sliding=cfg['friction_coulomb_nm'],
                                     friction_load=cfg['friction_load_per_nm'])
    # THE friction model (utility/friction_model.py, 2026-10-17): the friction feedforward on the commanded speed
    # (utility/mode_dispatch._add_friction_ff, behind rti['friction_ff_enabled']) reads it from ctx.
    from utility.friction_model import FrictionModel
    ctx.friction_model = FrictionModel.from_config(cfg, joint_num, ff_fraction=_dff.get("friction_fraction", 1.0))
    log_dynamics.info("friction model (robot XML, api_tests/77): " + ctx.friction_model.describe()
                      + f" Nm; feedforward gives back {ctx.friction_model.ff_fraction:g} of it")
    log_dynamics.info("friction compensation (freedrive, switch OFF at start): " + ", ".join(
        f"J{j + 1} {c:.3f} Nm" for j, c in enumerate(ctx.friction_comp.c)) +
        f" (fraction {ctx.friction_comp.fraction:g} of friction - gravity error; full beyond "
        f"{_sys['friction_comp']['band_dps']:g} deg/s, fading {_sys['friction_comp']['fade_start_dps']:g}-"
        f"{_sys['friction_comp']['fade_end_dps']:g} deg/s)")
    log_safety.info(f"haptic zone walls: {'ON' if haptic_walls.active else 'OFF'} "
                f"(k={haptic_walls.k:.0f} N/m, d={haptic_walls.d:.0f} N.s/m at 0.1 kg"
                f"{' x effective mass / 0.1 kg' if haptic_walls.mass_scaled else ''}, max {haptic_walls.f_max:.0f} N)")
    _log_walls(st._joint_walls, "ON at start-up")

    # Maintenance policy from <maintenance_defaults> in config/system.xml.

    # IP of the PC currently paired over TCP. drain_rti_udp() filters on it so a
    # stray sender cannot inject setpoints. None until rti_connect arrives, and the
    # UDP read is skipped entirely while unpaired.
    st.rti_pc_ip = None

    # --- SOFT_RTI TCP transport (optional; "udp" is the default and unchanged) ---
    st.rti_transport = "udp"          # set by _handle_rti_connect from the PC's request
    st.rti_tcp_conn = None            # the accepted stream, or None
    rti_tcp_rxbuf = bytearray()    # partial COMMAND frame carried between ticks


    class _TcpStateSender:
        """Adapter so the handler's single state-send line works over TCP unchanged.

        `rti_handler` does `ctx.socket_sender.sendto(payload, addr)` in exactly one place.
        Presenting that same shape here keeps the transport decision OUT of the handler: it
        should not know which transport it is on, and a second code path in there would be a
        second place to forget a fix. Swapping this object into ctx.socket_sender is the whole
        integration.
        """

        __slots__ = ("conn", "out", "failed")

        def __init__(self):
            self.conn = None
            self.out = bytearray()
            self.failed = False

        def sendto(self, data, addr):     # addr is ignored: TCP already knows its peer
            if self.conn is None:
                return
            if not send_rti_tcp(self.conn, self.out, data):
                self.failed = True        # backlog past the cap — the PC has stopped reading

        def reset(self, conn):
            self.conn = conn
            del self.out[:]
            self.failed = False


    rti_tcp_sender = _TcpStateSender()


    def _drop_rti_tcp(why):
        """Close the stream and clear every scrap of its state. One place, so a half-reset
        connection cannot survive into the next session and desynchronise its framing."""
        if st.rti_tcp_conn is not None:
            log_handler.info("TCP stream closed: %s", why)
            try:
                st.rti_tcp_conn.close()
            except OSError:
                pass
        st.rti_tcp_conn = None
        rti_tcp_sender.reset(None)
        del rti_tcp_rxbuf[:]
        # DELIBERATELY still "tcp". An earlier version reset this to "udp", which made a dropped
        # stream UNRECOVERABLE: the accept path is gated on the transport, so the PC could never
        # dial back in without re-issuing rti_connect, and meanwhile the robot was listening on
        # UDP for a PC that was not speaking it. Staying on tcp lets a reconnect be accepted on
        # the next tick. That is safe because the claim does NOT survive: no commands arrive
        # while the stream is down, so the command timeout drops control and the PC has to
        # re-claim — which is the honest sequence after losing the link.

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
        _before = shared_data.robot_state.get()
        if success:
            shared_data.robot_state.set("ENABLED")
            if _before != "ENABLED":           # the activity log: the state change, with who asked (2026-10-15)
                log_activity(_rti_activity_logger, "RTI", "info", "STATE_CHANGE", id=st.command_id,
                             message="DISABLED → ENABLED")
        else:
            shared_data.robot_state.set(f"ENABLE_REJECTED:{reason}")
            log_activity(_rti_activity_logger, "RTI", "warning", "ENABLE_REJECTED", id=st.command_id, message=reason)


    def _handle_disable(cmd_msg, cmd_dict):
        _before = shared_data.robot_state.get()
        shared_data.robot_state.set("DISABLED")
        if _before == "ENABLED":
            log_activity(_rti_activity_logger, "RTI", "info", "STATE_CHANGE", id=st.command_id,
                         message="ENABLED → DISABLED")
        _freeze_gripper("disable")
        st.pending_mode_request = RobotMode.IDLE


    def _handle_gripper_hold(cmd_msg, cmd_dict):
        """gripper_hold — the executor's stop() (2026-10-24): the jaws stop where they are."""
        _freeze_gripper("stop")


    def _handle_home(cmd_msg, cmd_dict):
        st.pending_mode_request = RobotMode.HOMING


    def _handle_clear_errors(cmd_msg, cmd_dict):
        send_clear_errors(shared_data, Motor, Motor_errors_data, joint_num, CLEAR_ERROR_SEND_REPEATS,
                          Gripper_data=Gripper_data, active_CAN_gripper=active_CAN_gripper)
        st.clear_error_countdown = CLEAR_ERROR_SETTLE_TICKS


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
            log_activity(_rti_activity_logger, "RTI", "info", "COMPENSATION_CHANGED", id=st.command_id,
                         message=f"gravity comp: {'ON' if new_val else 'OFF'}")


    def _handle_friction_comp(cmd_msg, cmd_dict):
        """friction_comp:1|0 — the operator switch for friction compensation in freedrive (utility/friction_comp.py)."""
        parts = cmd_msg.split(":", 1)        # see _handle_gravity_comp for why this is guarded
        if len(parts) != 2:
            return
        new_val = 1.0 if parts[1].strip() == "1" else 0.0
        old_val = shared_data.rti['friction_comp_enabled'][0]
        shared_data.rti['friction_comp_enabled'][0] = new_val
        if new_val != old_val:
            log_activity(_rti_activity_logger, "RTI", "info", "COMPENSATION_CHANGED", id=st.command_id,
                         message=f"friction comp: {'ON' if new_val else 'OFF'}")
            log_dynamics.info(f"friction compensation {'ON' if new_val else 'OFF'} (freedrive)")


    def _handle_collision_detection(cmd_msg, cmd_dict):
        """collision_detection:<off|warn|stop> — collision detection's reaction until RTI restarts (2026-10-16): a PC
        program that pushes on purpose switches it off, and back. Start state: system.xml <safety_monitor><collision>.
        A switch to a weaker reaction is a WARNING in RTI.log (a protection switched off)."""
        parts = cmd_msg.split(":", 1)        # see _handle_gravity_comp for why this is guarded
        if len(parts) != 2 or parts[1].strip() not in _COLLISION_MODE_CODE:
            log_safety.warning(f"collision_detection: {cmd_msg!r} is not off | warn | stop — ignored")
            return
        old = _COLLISION_MODE_NAME[int(shared_data.rti['collision_mode'][0])]
        new = safety_monitor.set_collision_mode(parts[1].strip())
        shared_data.rti['collision_mode'][0] = _COLLISION_MODE_CODE[new]
        if new != old:
            log_activity(_rti_activity_logger, "RTI", "info", "COLLISION_DETECTION_CHANGED", id=st.command_id,
                         message=f"collision detection: {old} -> {new}")
        (log_safety.warning if _COLLISION_MODE_CODE[new] < _COLLISION_MODE_CODE[old] else log_safety.info)(
            f"collision detection {old} -> {new}" + ("" if new == parts[1].strip()
                                                     else " (no thresholds or no estimate: nothing to check)"))


    def _dynamics_switch(cmd_msg, key, label):
        """inertia_comp:1|0 / coriolis_comp:1|0 — the operator switches of the inertia and Coriolis feedforward on the
        commanded motion (EXEC, JOG, RTI / SOFT_RTI position and velocity commands; utility/mode_dispatch.py)."""
        parts = cmd_msg.split(":", 1)        # see _handle_gravity_comp for why this is guarded
        if len(parts) != 2:
            return
        new_val = 1.0 if parts[1].strip() == "1" else 0.0
        old_val = shared_data.rti[key][0]
        shared_data.rti[key][0] = new_val
        if new_val != old_val:
            log_activity(_rti_activity_logger, "RTI", "info", "COMPENSATION_CHANGED", id=st.command_id,
                         message=f"{label} comp: {'ON' if new_val else 'OFF'}")
            log_dynamics.info(f"{label} feedforward {'ON' if new_val else 'OFF'}")

    def _handle_inertia_comp(cmd_msg, cmd_dict):
        _dynamics_switch(cmd_msg, 'inertia_comp_enabled', "inertia")

    def _handle_friction_ff(cmd_msg, cmd_dict):
        """friction_ff:1|0 — the friction feedforward on the commanded speed (2026-10-17, utility/friction_model.py)."""
        _dynamics_switch(cmd_msg, 'friction_ff_enabled', "friction")

    def _handle_can_read_first(cmd_msg, cmd_dict):
        """can_read_first:1|0 — TODO/can_tx_timing_plan.md option B (2026-10-12; its start state is system.xml
        <can_read_first>, ON by default): the CAN read and the minimal measurement at the START of the tick, and after
        queueing the commands RTI sleeps until its first frame has left (hardware/can_hardware.FirstFrameWait); the rest
        after the send. The waiter's state IS the switch: a socket that cannot open leaves it OFF (the old order)."""
        parts = cmd_msg.split(":", 1)        # see _handle_gravity_comp for why this is guarded
        if len(parts) != 2:
            return
        first_frame_wait.set_enabled(parts[1].strip() == "1")

    def _handle_coriolis_comp(cmd_msg, cmd_dict):
        _dynamics_switch(cmd_msg, 'coriolis_comp_enabled', "Coriolis")

    def _handle_hall_check(cmd_msg, cmd_dict):
        """hall_check:1|0 — the Hall sensor check (robotics/hall_check.py, 2026-10-13): the Hall-sensor joint (J6) limp,
        its Hall pin read every tick for the dashboard's I/O tab. Only in IDLE (enabled or not, homed or not); it stops
        itself before any mode change."""
        parts = cmd_msg.split(":", 1)        # see _handle_gravity_comp for why this is guarded
        if len(parts) != 2:
            return
        if parts[1].strip() == "1":
            why = ("a joint is free for hand turning (free_joint) or in a test move — stop that first"
                   if free_joint.active or joint_test.active else
                   hall_check.start(idle=st.current_mode == RobotMode.IDLE,
                                    mode_pending=st.pending_mode_request is not None,
                                    booting=st.startup_motor_reconfig < 3))
            if why:
                log_homing.warning(f"Hall check refused: {why}")
                log_activity(_rti_activity_logger, "RTI", "warning", "HALL_CHECK_REFUSED", id=st.command_id, message=why)
            elif hall_check.active:
                log_activity(_rti_activity_logger, "RTI", "info", "HALL_CHECK", id=st.command_id,
                             message=f"J{hall_check.joint + 1} limp: Hall sensor check ON")
        elif hall_check.active:
            hall_check.stop("stopped by the operator")
            log_activity(_rti_activity_logger, "RTI", "info", "HALL_CHECK", id=st.command_id, message="Hall sensor check OFF")

    def _handle_free_joint(cmd_msg, cmd_dict):
        """free_joint:<n> — one joint limp for hand turning (robotics/free_joint.py, 2026-10-18; the first-run check's
        S3): n = 1..joint_num frees that joint (the same n again renews its 2-minute timeout), 0 holds it again. Only in
        IDLE (enabled or not, homed or not); it stops itself before any mode change."""
        parts = cmd_msg.split(":", 1)        # see _handle_gravity_comp for why this is guarded
        if len(parts) != 2:
            return
        try:
            n = int(parts[1].strip())
        except ValueError:
            return
        if n == 0:
            if free_joint.active:
                was = free_joint.joint + 1
                free_joint.stop("stopped by the operator")
                log_activity(_rti_activity_logger, "RTI", "info", "FREE_JOINT", id=st.command_id,
                             message=f"J{was} held again")
            return
        renewing = free_joint.joint == n - 1
        why = free_joint.start(n - 1, idle=st.current_mode == RobotMode.IDLE,
                               mode_pending=st.pending_mode_request is not None,
                               booting=st.startup_motor_reconfig < 3,
                               hall_check_active=hall_check.active or joint_test.active)
        if why:
            log_homing.warning(f"Free joint refused: {why}")
            log_activity(_rti_activity_logger, "RTI", "warning", "FREE_JOINT_REFUSED", id=st.command_id, message=why)
        elif not renewing:
            log_activity(_rti_activity_logger, "RTI", "info", "FREE_JOINT", id=st.command_id,
                         message=f"J{n} free for hand turning (current limit 0)")

    def _handle_joint_test(cmd_msg, cmd_dict):
        """joint_test:<n>:<amplitude deg>:<speed deg/s>:<current mA> — a small powered test move of joint n and back,
        homed or not (robotics/joint_test.py, 2026-10-18; the first-run check's stage 3); n = 0 stops it. Only in IDLE
        and ENABLED; it stops itself before any mode change, on a disable, and when the joint falls behind."""
        parts = cmd_msg.split(":")
        try:
            n = int(parts[1])
            a, v, c = (float(x) for x in parts[2:5]) if n else (0.0, 0.0, 0.0)
        except (IndexError, ValueError):
            return
        if n == 0:
            if joint_test.active:
                joint_test.stop("stopped by the operator")
            return
        why = joint_test.start(n - 1, a, v, c, idle=st.current_mode == RobotMode.IDLE,
                               enabled=shared_data.robot_state.get() == "ENABLED",
                               mode_pending=st.pending_mode_request is not None,
                               booting=st.startup_motor_reconfig < 3,
                               other_active=hall_check.active or free_joint.active,
                               homed=bool(np.all(shared_data.rti['homing_status'][:joint_num] == 2)))
        if why:
            log_homing.warning(f"Joint test refused: {why}")
            log_activity(_rti_activity_logger, "RTI", "warning", "JOINT_TEST_REFUSED", id=st.command_id, message=why)
        else:
            log_activity(_rti_activity_logger, "RTI", "info", "JOINT_TEST", id=st.command_id,
                         message=f"J{n} test move {a:+.1f} deg at {v:.1f} deg/s, {c:.0f} mA")


    def _handle_set_mode(cmd_msg, cmd_dict):
        parts = cmd_msg.split(":")  # see _handle_gravity_comp for why this is guarded
        if len(parts) < 2:
            return
        if parts[1].strip().upper() == "IDLE" and st.current_mode == RobotMode.EXEC:
            # IDLE while EXEC plays (the client's enter_idle_mode(); the stop's fallback): brake along the planned
            # path at the joints' ceilings first; the EXEC handler asks for IDLE itself once the arm is at rest
            # (2026-10-13, user's choice: braked along the path, as industrial arms commonly do — IDLE at speed was
            # freedrive: the arm coasted). A stop itself is exec_stop and stays in EXEC (2026-10-14).
            _exec = MODE_HANDLERS.get(RobotMode.EXEC)
            if _exec is not None:
                _exec.begin_stop()
                return
        st.pending_mode_request = parts[1].strip().upper()
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
        st.pending_mode_force = _assertion in ("PARKED", "FORCE")
        st.pending_mode_assertion = _assertion
        # The same third field carries the PC's command rate for SOFT_RTI. Distinguished by the
        # mode, not by the value: FLASHING reads it as an operator assertion, SOFT_RTI as Hz.
        # A rate that will not parse is ignored rather than refused — the mode change is the
        # safety-relevant part, and the timeout simply keeps its previous value.
        st.pending_mode_rate_hz = None
        if st.pending_mode_request == "SOFT_RTI" and _assertion:
            try:
                st.pending_mode_rate_hz = float(_assertion)
            except ValueError:
                log_handler.warning("set_mode:SOFT_RTI rate %r is not a number — timeout "
                               "left unchanged", _assertion)


    def _handle_jog_set(cmd_msg, cmd_dict):
        parts = cmd_msg.split(":")
        if len(parts) == 4 and st.current_mode == RobotMode.JOG:
            _jog = MODE_HANDLERS.get(RobotMode.JOG)
            if _jog is not None:
                _jog.set_jog(int(parts[1]), int(parts[2]), float(parts[3]), cmd_id=st.command_id)


    def _handle_jog_stop(cmd_msg, cmd_dict):
        _jog = MODE_HANDLERS.get(RobotMode.JOG)
        if _jog is not None:
            _jog.stop_jog(cmd_id=st.command_id)


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


    def _handle_exec_stop(cmd_msg, cmd_dict):
        """exec_stop:<epoch> — the client's stop() (2026-10-14, user: the arm stays in EXEC, holding): brake along the
        planned path, drop the program up to the planner's fence of <epoch>, hold. Not in EXEC (the mode changed
        since the executor looked): IDLE, as a stop always did there."""
        parts = cmd_msg.split(":")
        try:
            epoch = int(parts[1])
        except (IndexError, ValueError):
            log_exec.warning("exec_stop without a valid epoch: %r", cmd_msg)
            return
        _exec = MODE_HANDLERS.get(RobotMode.EXEC)
        if st.current_mode == RobotMode.EXEC and _exec is not None:
            _exec.begin_stop(epoch)
        else:
            _handle_set_mode("set_mode:IDLE", cmd_dict)


    def _handle_exec_completion_policy(cmd_msg, cmd_dict):
        parts = cmd_msg.split(":", 1)
        if len(parts) != 2:
            return
        _exec = MODE_HANDLERS.get(RobotMode.EXEC)
        if _exec is not None:
            try:
                _exec.set_completion_policy(parts[1].strip())
            except ValueError as exc:
                log_exec.warning("exec_completion_policy: %s", exc)


    def _handle_gripper_set(cmd_msg, cmd_dict):
        # gripper_set:<position>:<speed>:<current>:<action>
        # position/speed: 0–255, current: mA, action: 0/1.
        # activate is always 1 in firmware mode (forced in send_gripper_commands).
        # A new gripper_set also aborts any in-progress calibration —
        # the user can see when the sweep is done, so an explicit
        # Open/Close/Goto/Stop is the cancel signal.
        parts = cmd_msg.split(":")
        if len(parts) == 5 and active_CAN_gripper:
            try:
                _gripper_fw_state['position'] = int(parts[1])
                _gripper_fw_state['speed']    = int(parts[2])
                _gripper_fw_state['current']  = int(parts[3])
                _gripper_fw_state['action']   = int(parts[4])
                if st._gripper_calibrate_running or st._gripper_calibrate_pending:
                    log_gripper.info("calibrate aborted by gripper_set")
                    st._gripper_calibrate_running = False
                    st._gripper_calibrate_pending = False
            except ValueError as exc:
                log_gripper.warning("gripper_set: bad payload %s", exc)


    def _handle_gripper_calibrate(cmd_msg, cmd_dict):
        if active_CAN_gripper:
            st._gripper_calibrate_pending = True


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
            log_gripper.warning("gripper_ctrl_mode: bad payload %s", exc)


    def _handle_set_pid_gains(cmd_msg, cmd_dict):
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
            if "KPP"     in cmd_dict: st.gripper_KPP            = float(cmd_dict["KPP"])
            if "KPV"     in cmd_dict: st.gripper_KPV            = float(cmd_dict["KPV"])
            if "KIV"     in cmd_dict: st.gripper_KIV            = float(cmd_dict["KIV"])
            if "KPIQ"    in cmd_dict: st.gripper_KPIQ           = float(cmd_dict["KPIQ"])
            if "KIIQ"    in cmd_dict: st.gripper_KIIQ           = float(cmd_dict["KIIQ"])
            if "KP"      in cmd_dict: st.gripper_KP             = float(cmd_dict["KP"])
            if "KD"      in cmd_dict: st.gripper_KD             = float(cmd_dict["KD"])
            if "Ilim"    in cmd_dict: st.gripper_Ilim           = float(cmd_dict["Ilim"])
            if "vel_lim" in cmd_dict: st.gripper_velocity_limit = float(cmd_dict["vel_lim"])
            if "vlim"    in cmd_dict: st.gripper_voltage_limit  = read_XML2.clamp_voltage_limit(
                                                                   cmd_dict["vlim"])
            _reload_gripper_config(num_repeats=2)


    # --- RTI mode (TODO/RTI_mode_plan.md §4d) ---------------------------------
    # These drive the handler's PAIRING lifecycle. They must never call enter()/exit(),
    # which belong to mode dispatch and take a TickContext.

    def _handle_rti_connect(cmd_msg, cmd_dict):
        """rti_connect:<pc_ip>[:<rate_hz>] — pair with a PC and request an RTI mode.

        The PC's IP is the one thing the robot cannot discover for itself (UDP has not
        started yet), so it rides the TCP handshake. State packets go to that IP on
        RTI_PORT_SENDER; commands are only accepted from that IP.

        WITH a rate, this requests RobotMode.SOFT_RTI and sizes the command timeout for that
        rate. Without one, RobotMode.RTI — the hard-real-time contract, unchanged, and the
        default so that every existing caller keeps exactly the behaviour it had.

        The rate is what distinguishes the two contracts because it is the honest
        discriminator: a PC that knows its own command rate is telling you it does not intend
        to close the loop every tick.
        """
        # current_mode is read-only here; the mode CHANGE still goes through the
        # superloop's request_mode_change(), which owns every gate.

        parts = cmd_msg.split(":")      # see _handle_gravity_comp for why this is guarded
        if len(parts) < 2 or not parts[1].strip():
            log_handler.warning("rti_connect missing pc_ip")
            return
        pc_ip = parts[1].strip()

        rate_hz = None
        if len(parts) > 2 and parts[2].strip():
            try:
                rate_hz = float(parts[2].strip())
            except ValueError:
                log_handler.warning("rti_connect rate %r is not a number — pairing as hard "
                               "RTI", parts[2])
                rate_hz = None
            else:
                if not np.isfinite(rate_hz) or rate_hz <= 0.0:
                    log_handler.warning("rti_connect rate %r is not a positive finite Hz — "
                                   "pairing as hard RTI", parts[2])
                    rate_hz = None

        # Transport is the OPTIONAL fourth field: rti_connect:<ip>:<rate>:<transport>. Absent
        # or unrecognised means "udp", so every caller that predates this keeps exactly the
        # behaviour it had. TCP is refused for hard RTI on purpose — head-of-line blocking
        # stalls the stream on any loss, which is precisely the contract RTI exists to make.
        want_transport = "udp"
        if len(parts) > 3 and parts[3].strip():
            req = parts[3].strip().lower()
            if req in ("udp", "tcp"):
                want_transport = req
            else:
                log_handler.warning("rti_connect transport %r is not udp/tcp — using udp",
                               parts[3])
        if want_transport == "tcp" and rate_hz is None:
            log_handler.warning("rti_connect: TCP is refused for hard RTI. A lost packet stalls "
                           "every packet behind it, which breaks the one guarantee hard RTI "
                           "makes. Using UDP; give a rate_hz for SOFT_RTI if you want TCP.")
            want_transport = "udp"
        if st.rti_tcp_conn is not None:
            try:
                st.rti_tcp_conn.close()
            except OSError:
                pass
            st.rti_tcp_conn = None
            rti_tcp_sender.reset(None)
            del rti_tcp_rxbuf[:]
        st.rti_transport = want_transport
        log_handler.info("Paired with %s over %s", pc_ip, st.rti_transport.upper())

        st.rti_pc_ip = pc_ip
        rti_handler.connect_pc((pc_ip, RTI_PORT_SENDER),
                               current_tick=int(shared_data.rti["loop_tick"][0]))
        _want = RobotMode.SOFT_RTI if rate_hz is not None else RobotMode.RTI

        # RE-PAIRING WITHOUT LEAVING THE MODE. The state machine rejects X -> X, correctly, so
        # the mode-change block below never runs and apply_mode_contract() never fires. A PC
        # that re-connects with a DIFFERENT rate would then keep the previous rate's timeout
        # and lose control on its first command gap. Apply the contract here instead: the mode
        # is already right, only the contract needs refreshing.
        if st.current_mode == _want:
            rti_handler.apply_mode_contract(soft=(_want == RobotMode.SOFT_RTI), rate_hz=rate_hz)
            st.pending_mode_rate_hz = None
            return

        st.pending_mode_rate_hz = rate_hz
        st.pending_mode_request = _want


    def _handle_rti_claim(cmd_msg, cmd_dict):
        """rti_claim — PC claims RT control (connected → control_active).

        Rejected while an error is latched: control must not begin before the error is
        explicitly recovered (plan §3 / §15).
        """
        if bool(shared_data.rti['error_active'][0]):
            log_handler.warning("rti_claim rejected — error latched, clear errors first")
            return
        # BOTH RTI contracts. Gating on RobotMode.RTI alone would make SOFT_RTI enterable but
        # unusable — the exact "a legal transition is not a reachable one" failure that made
        # FLASHING unreachable on 2026-08-04, arriving from the other direction.
        if st.current_mode not in (RobotMode.RTI, RobotMode.SOFT_RTI):
            log_handler.warning("rti_claim rejected — mode is %s, not an RTI mode",
                           st.current_mode)
            return
        if rti_handler.sub_state != 'connected':
            log_handler.warning("rti_claim rejected — sub_state is '%s'", rti_handler.sub_state)
            return
        # Seed the rate limiter from the live arm, so its first output ramps from where the
        # robot actually is rather than from 0 rad.
        #
        # Can be REFUSED: with the rate limiter enabled there is no fallback planner, so a
        # missing ruckig or a robot XML without finite limits means control is not granted.
        # The handler logs why. Refusing before anything moves beats moving badly.
        if not rti_handler.claim_control(
                current_tick=int(shared_data.rti["loop_tick"][0]),
                measured_pos=shared_data.joint['joint_position'][:joint_num]):
            log_handler.warning("rti_claim rejected — see the error above")


    def _handle_rti_set_rate_limiter(cmd_msg, cmd_dict):
        """rti_set_rate_limiter:<0|1> — accel/jerk/torque-rate clamping on incoming commands."""
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            rti_handler.set_rate_limiter(parts[1].strip() == "1")


    def _handle_rti_set_tcp_vel_limit(cmd_msg, cmd_dict):
        """rti_set_tcp_vel_limit:<m/s> — Cartesian speed cap on RTI commands. 0 disables."""
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            try:
                rti_handler.set_tcp_velocity_limit(float(parts[1].strip()))
            except ValueError:
                log_handler.warning("rti_set_tcp_vel_limit: not a number: %r", parts[1])


    def _handle_rti_set_command_timeout(cmd_msg, cmd_dict):
        """rti_set_command_timeout:<seconds> — command watchdog, sized for the PC's rate.

        Needed by every servo-mode option: at 20 Hz the 50 ms gap between two healthy commands
        is longer than the shipped 40 ms dead-link budget. Clamped in the handler.
        """
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            try:
                rti_handler.set_command_timeout(float(parts[1].strip()))
            except ValueError:
                log_handler.warning("rti_set_command_timeout: not a number: %r", parts[1])


    def _handle_rti_set_vel_scale(cmd_msg, cmd_dict):
        """rti_set_vel_scale:<fraction> — scale RTI joint velocity limits. 1.0 = the XML values."""
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            try:
                rti_handler.set_vel_scale(float(parts[1].strip()))
            except ValueError:
                log_handler.warning("rti_set_vel_scale: not a number: %r", parts[1])


    def _handle_rti_set_min_duration(cmd_msg, cmd_dict):
        """rti_set_min_duration:<multiple> — each plan lasts >= N command intervals. 0 = off.

        A minimum trajectory execution time. Must exceed 1 to have any effect:
        at 1x the plan's deceleration still runs. 1.5 is the suggested starting point.
        """
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            try:
                rti_handler.set_min_duration(float(parts[1].strip()))
            except ValueError:
                log_handler.warning("rti_set_min_duration: not a number: %r", parts[1])


    def _handle_reload_zones(cmd_msg, cmd_dict):
        """reload_zones — install the zone setup command_executor parsed and validated (cmd_dict["setup"], the
        JSON form of utility.zones.zones_summary; rebuilt here by config.xml_parser.zone_setup_from_wire).

        Since 2026-10-04 this reads NO file: the parse used to run in the tick (review S4 — file I/O is the
        unbounded part of a tick, the 18 ms LOOP_CRITICAL of 2026-10-03 was a file write). A message without a
        setup (an old sender) is refused and the zones in force stay, as they do for anything malformed.
        """
        wire = cmd_dict.get("setup") if isinstance(cmd_dict, dict) else None
        if not isinstance(wire, dict):
            log_safety.error("reload_zones without a setup (send it through command_executor) — zones unchanged")
            return
        try:
            setup = read_XML2.zone_setup_from_wire(wire)
        except (ValueError, KeyError, TypeError) as e:
            log_safety.error(f"reload_zones refused, zones unchanged: {e}")
            return
        safety_monitor.set_zones(setup['zones'], setup['tool_spheres'], haptic=setup['haptic'])
        haptic_walls.set_policy(setup['haptic'])
        haptic_walls.set_zones(setup['zones'], setup['tool_spheres'])
        st._wall_haptic = setup['haptic']
        st._joint_walls = _resolve_walls(st._wall_haptic)
        haptic_walls.set_joint_walls(st._joint_walls)
        safety_monitor.set_joint_walls(st._joint_walls)
        log_safety.info(f"haptic walls reloaded: {'ON' if haptic_walls.active else 'OFF'} "
                    f"(k={haptic_walls.k:.0f} N/m, d={haptic_walls.d:.0f} N.s/m at 0.1 kg"
                    f"{' x effective mass / 0.1 kg' if haptic_walls.mass_scaled else ''}, max {haptic_walls.f_max:.0f} N)")
        _log_walls(st._joint_walls, "reloaded")


    def _handle_set_joint_walls(cmd_msg, cmd_dict):
        """set_joint_walls — the live per-joint freedrive wall specs (dashboard Walls tab, via command_executor,
        which validated them and refuses an unstable one). cmd_dict["specs"]: one wall_spec dict per joint, user
        units. Applied to the walls AND the monitor; every wall re-arms (never pushes on its own). Not saved: the
        robot XML is written only by save_joint_walls, so an RTI restart goes back to the file."""
        specs = cmd_dict.get("specs")
        if not isinstance(specs, list) or len(specs) != joint_num:
            log_safety.warning(f"set_joint_walls: need {joint_num} specs, got {specs!r:.120}")
            return
        _wall_specs[:] = [read_XML2.wall_spec(s if isinstance(s, dict) else None, f"set_joint_walls J{i + 1}")
                          for i, s in enumerate(specs)]
        st._joint_walls = _resolve_walls(st._wall_haptic)
        haptic_walls.set_joint_walls(st._joint_walls)
        safety_monitor.set_joint_walls(st._joint_walls)
        _log_walls(st._joint_walls, "set live")


    def _handle_reload_recipes(cmd_msg, cmd_dict):
        """reload_recipes — install the recipes from the file TEXTS command_executor read (cmd_dict["recipes"],
        {file name: xml}); RecipeManager.reload_from_texts parses them from memory (2026-10-04: this used to set a
        shared-memory signal and the tick then scanned data_recipe/ — file I/O in the loop, review S4). Without
        this, installing a recipe meant restarting RTI.py, which clears homing. ONE path: the GUI's button goes
        through the executor like every other command.
        """
        texts = cmd_dict.get("recipes") if isinstance(cmd_dict, dict) else None
        if not isinstance(texts, dict):
            log_recipe.error("reload_recipes without the files' text (send it through command_executor) — recipes unchanged")
            return
        recipe_mgr.reload_from_texts(texts)
        log_recipe.info(f"recipes reloaded from {len(texts)} file(s) sent by command_executor")


    def _handle_reload_filters(cmd_msg, cmd_dict):
        """reload_filters — rebuild the signal filter banks from the spec table command_executor parsed out of
        config/filters.xml (cmd_dict["config"], utility/filter_config.parse_filter_config's dict): no file is read
        here (2026-10-04; the tick used to parse the file on a shared-memory signal). Rebuilding resets each
        filter's state, so expect a brief (~one filter window) transient. An invalid table is refused and the
        banks in force stay.
        """
        config = cmd_dict.get("config") if isinstance(cmd_dict, dict) else None
        if not isinstance(config, dict):
            log_filters.error("reload_filters without a spec table (send it through command_executor) — filters unchanged")
            return
        try:
            banks = build_filter_banks(config, joint_num, delta_t)
        except (ValueError, KeyError, TypeError) as e:
            log_filters.error(f"reload refused, filters unchanged: {e}")
            return
        st.filter_banks = banks
        log_filters.info("hot-reloaded from the table command_executor parsed (config/filters.xml)")


    def _handle_set_payload(cmd_msg, cmd_dict):
        """set_payload — the part the gripper holds: {"mass": kg, "com": [x, y, z] m in the tool frame}
        (utility/payload.py, 2026-10-04). Combined with the gripper's body, the dynamics models rebuilt (~1 ms,
        no file I/O — a cold path, one late tick at most) and swapped in: gravity feedforward in every mode, the
        walls' effective inertia and inverse mass. Bounds: the robot XML's <max_payload_kg>, |COM| <= 0.5 m.
        """
        ok, msg = apply_payload_command(cmd_dict, _payload_model, haptic_walls, shared_data.rti, cfg['safety']['max_payload_kg'])
        if ok:
            log_dynamics.info(f"payload: {msg}")
            log_activity(_rti_activity_logger, "RTI", "info", "PAYLOAD_CHANGED", id=st.command_id, message=msg)
        else:
            log_dynamics.warning(f"payload: set_payload refused: {msg}")


    def _handle_set_log_level(cmd_msg, cmd_dict):
        """set_log_level:<name>:<LEVEL> — one logger's level in THIS process (the executor set its own and forwarded
        it; utility/logger_setup.apply_log_level). Cold; not persisted."""
        parts = cmd_msg.split(":", 2)
        ok, text = apply_log_level(parts[1] if len(parts) > 1 else "", parts[2] if len(parts) > 2 else "")
        (logger.info if ok else logger.warning)(f"log level: {text}")


    def _handle_rti_set_state_rate(cmd_msg, cmd_dict):
        """rti_set_state_rate:<hz> — how often to unicast the recipe payload. 0 = every tick.

        The state stream runs at the full loop rate regardless of the PC's command rate, so a
        5 Hz session still gets 250 packets/s. Matters most on the TCP transport, where state is
        20-40x the command traffic and is the direction that builds backpressure.
        """
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            try:
                rti_handler.set_state_rate(float(parts[1].strip()))
            except ValueError:
                log_handler.warning("rti_set_state_rate: not a number: %r", parts[1])


    def _handle_rti_set_pace(cmd_msg, cmd_dict):
        """rti_set_pace:<factor> — cap the generator at factor x the measured target speed.

        Self-tuning alternative to rti_set_vel_scale: the robot derives the cap from how fast
        the target is actually moving, so the operator need not know their travel speed.
        0 disables (the default).
        """
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            try:
                rti_handler.set_pace(float(parts[1].strip()))
            except ValueError:
                log_handler.warning("rti_set_pace: not a number: %r", parts[1])


    def _handle_rti_set_servo_generator(cmd_msg, cmd_dict):
        """rti_set_servo_generator:<name>[:<gain>[:<lookahead>]] — pick the position generator.

        "ruckig" (default, point-to-point planner) or "tracker" (servo-style tracking law for
        low-rate streams). Gain/lookahead are optional and only meaningful for the tracker;
        empty strings mean "leave it alone", so the name can be changed without retuning.
        """
        parts = cmd_msg.split(":")
        if len(parts) < 2:
            return
        gain = lookahead = self_lookahead = None
        try:
            if len(parts) > 2 and parts[2].strip():
                gain = float(parts[2].strip())
            if len(parts) > 3 and parts[3].strip():
                lookahead = float(parts[3].strip())
            if len(parts) > 4 and parts[4].strip():
                self_lookahead = float(parts[4].strip())
        except ValueError:
            log_handler.warning("rti_set_servo_generator: bad numeric payload: %r", cmd_msg)
            return
        rti_handler.set_servo_generator(parts[1].strip(), gain=gain, lookahead=lookahead,
                                        self_lookahead=self_lookahead)


    def _handle_rti_set_dynamics_ff(cmd_msg, cmd_dict):
        """rti_set_dynamics_ff:<off|stream> — inertia / Coriolis feedforward source for RTI / SOFT_RTI position
        streams (every RTI entry starts with motion/rti_handler.DYNAMICS_FF_DEFAULT, "stream", 2026-10-11)."""
        parts = cmd_msg.split(":", 1)
        if len(parts) == 2:
            rti_handler.set_dynamics_ff_source(parts[1].strip())


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
        known = recipe_mgr.names          # a @property, NOT a method — calling it raised
                                          # TypeError and killed the RT loop (see below)
        if name not in known:
            log_recipe.warning("set_recipe refused: %r is not a known recipe. Available: %s",
                           name, ", ".join(known))
            return
        shared_data.active_recipe.set(name)
        log_recipe.info("Active recipe -> %s", name)


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
            log_handler.warning("rti_set_lowpass: bad payload %s", exc)


    def _handle_rti_release(cmd_msg, cmd_dict):
        """rti_release — TCP fallback for a CLEAN stop.

        The primary path is UDP CMD_STOP inside the command packet; this exists for when
        the PC's UDP sender is already dead. If neither arrives the timeout covers it.
        """
        if not rti_handler.request_release():
            log_handler.warning("rti_release ignored — sub_state is '%s'", rti_handler.sub_state)


    def _handle_rti_disconnect(cmd_msg, cmd_dict):
        """rti_disconnect — drop the pairing and return to IDLE."""

        rti_handler.disconnect()
        st.rti_pc_ip = None
        _drop_rti_tcp("unpaired")
        st.pending_mode_request = RobotMode.IDLE
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
        "friction_comp":     _handle_friction_comp,
        "collision_detection": _handle_collision_detection,
        "inertia_comp":      _handle_inertia_comp,
        "friction_ff":       _handle_friction_ff,
        "can_read_first":    _handle_can_read_first,
        "hall_check":        _handle_hall_check,
        "free_joint":        _handle_free_joint,
        "joint_test":        _handle_joint_test,
        "coriolis_comp":     _handle_coriolis_comp,
        "set_mode":          _handle_set_mode,
        "jog_set":           _handle_jog_set,
        "jog_stop":          _handle_jog_stop,
        "jog_accel":         _handle_jog_accel,
        "jog_profile":       _handle_jog_profile,
        "jog_jerk_factor":   _handle_jog_jerk_factor,
        "jog_control_mode":  _handle_jog_control_mode,
        "exec_pause":        _handle_exec_pause,
        "exec_resume":       _handle_exec_resume,
        "exec_stop":         _handle_exec_stop,
        "exec_completion_policy": _handle_exec_completion_policy,
        "gripper_set":       _handle_gripper_set,
        "gripper_hold":      _handle_gripper_hold,
        "gripper_calibrate": _handle_gripper_calibrate,
        "gripper_ctrl_mode": _handle_gripper_ctrl_mode,
        "set_pid_gains":     _handle_set_pid_gains,
        "rti_connect":       _handle_rti_connect,
        "rti_claim":         _handle_rti_claim,
        "rti_release":       _handle_rti_release,
        "rti_disconnect":    _handle_rti_disconnect,
        "rti_set_rate_limiter":     _handle_rti_set_rate_limiter,
        "rti_set_gravity_comp":     _handle_rti_set_gravity_comp,
        "rti_set_dynamics_ff":      _handle_rti_set_dynamics_ff,
        "rti_set_tcp_vel_limit":    _handle_rti_set_tcp_vel_limit,
        "rti_set_command_timeout":  _handle_rti_set_command_timeout,
        "rti_set_vel_scale":        _handle_rti_set_vel_scale,
        "rti_set_servo_generator":  _handle_rti_set_servo_generator,
        "rti_set_pace":             _handle_rti_set_pace,
        "rti_set_state_rate":       _handle_rti_set_state_rate,
        "reload_recipes":           _handle_reload_recipes,
        "reload_filters":           _handle_reload_filters,
        "set_payload":              _handle_set_payload,
        "set_log_level":            _handle_set_log_level,    # diagnostics (2026-10-15)
        "reload_zones":             _handle_reload_zones,
        "set_joint_walls":          _handle_set_joint_walls,
        "rti_set_min_duration":     _handle_rti_set_min_duration,
        "set_recipe":               _handle_set_recipe,
        "rti_set_start_pose_check": _handle_rti_set_start_pose_check,
        "rti_set_lowpass":          _handle_rti_set_lowpass,
    }

    # ==========================================================================
    # The tick's CAN read, its measurement and its safety pass, as functions (2026-10-12): option B runs them at the
    # START of the tick and before dispatch, the old order where they always were (TODO/can_tx_timing_plan.md §9).
    # Their bodies are the loop's own blocks, moved verbatim.
    # ==========================================================================
    def _can_read():
        """Drain and decode the CAN replies; the reply monitor; the connection status."""
        nonlocal Hall_index, additional_pin2, Edge_detect, _can_sent_prev, _profile_last
        # ==========================================================================
        # Mainloop CAN receive and fill queues
        # ==========================================================================
        if _bus_silent:
            # Someone else owns the bus. Drop everything on the floor rather than
            # decoding it — flash stream frames alias onto real motor node IDs.
            drain_can_discard(communication1)
        else:
            Hall_index, additional_pin2, Edge_detect, st.received_ids, st.latest_commands_id = receive_and_process_can_messages(
                communication1, Motor, Motor_data, Gripper_data,
                Motor_temperature_data, Motor_voltage_data,
                Motor_errors_data, Motor_info_data, Motor_Kt_data,
                joint_num, active_CAN_gripper,
                _tick_now,
                st.received_ids, st.latest_commands_id,
                node_error_bits=node_error_bits,
                frame_age_out=_can_age_buf,
                data_reply_count_out=_can_reply_buf,
                reply_time_out=_can_reply_time,
                receive=fast_can_reader.receive_can_messages if _read_first else None,
            )
        loop_delay_meter.on_read(_tick_now, _can_reply_time, _read_first,
                                 st.current_mode != RobotMode.HOMING and st.current_mode != RobotMode.FLASHING)
        can_reply_monitor.step(_can_reply_buf, _tick_now, _can_sent_prev and not _bus_silent)
        _can_sent_prev = _can_sent_now
        # [0]=worst, [1]=best age among the frames read this tick. Both published: the SPREAD is
        # what tells a real backlog (min ~= max, both large) from a single stale frame class
        # (min small, max large).
        shared_data.rti["can_frame_age"][0] = _can_age_buf[0]
        shared_data.rti["can_frame_age_min"][0] = _can_age_buf[1]
        if _can_age_buf[0] > shared_data.rti["can_frame_age_peak"][0]:
            shared_data.rti["can_frame_age_peak"][0] = _can_age_buf[0]
        shared_data.motor["HALL_INDEX"][0] = Hall_index
        shared_data.rti["received_ids"][:] = st.received_ids[:] #Update the shared memory
        shared_data.rti["latest_commands_id"][:] = st.latest_commands_id[:]  #Update the shared memory
        # Live per-node fault bit (mutated in place at the CAN decode) — arm joints + gripper.
        shared_data.rti["node_error_bits"][:] = node_error_bits[:]

        st.received_ids[:]       = 0   # reset in-place — no allocation
        st.latest_commands_id[:] = -1  # reset in-place — no allocation
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

    def _measure_motor():
        """The CAN deques -> motor[] + gripper[] (the measurement's first step)."""
        nonlocal _profile_last
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
            gripper_speed_bank=st.filter_banks['gripper_speed'],
        )
        _profile_last = _profile_mark(_profile_last, "motor_gripper_shared")

    def _history_entry():
        """This tick's Joint_data entry (the measurement's last step; the finite differences read it next tick), and the
        same tick in the shared-memory trace (utility/trace_ring.py)."""
        nonlocal _profile_last
        _hist_t = time.perf_counter()
        # .value, not str(): RobotMode is a (str, Enum), and str() on it yields "RobotMode.IDLE" rather than the bare name
        _hist_mode = getattr(st.current_mode, "value", st.current_mode)
        _hist_period_ms = shared_data.rti["loop_period"][0] * 1e3
        # ==========================================================================
        # Fill queues for joint + TCP level
        # ========================================================================== 
        process_joint_data(Joint_data,
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
                           _hist_t,
                           st.loop_execution_time,
                           _tick_now,
                           motor_error_bytes,
                           # RTI link diagnostics — read from shared memory rather than the
                           # handler so the value is whatever was published this tick, and so
                           # this call site does not care which mode is active. All zero
                           # outside RTI mode (RTI.py zeroes them at startup and on disconnect).
                           # inertia / Coriolis feedforward added this tick (0 when off; mode_dispatch._add_dynamics_ff)
                           torque_inertia=shared_data.joint['joint_inertia_torque'],
                           torque_coriolis=shared_data.joint['joint_coriolis_torque'],
                           rti_seq=shared_data.rti["rti_seq_rx_last"][0],
                           rti_ok=shared_data.rti["control_success_rate"][0],
                           rti_miss=shared_data.rti["rti_consecutive_misses"][0],
                           rti_state=shared_data.rti["rti_substate"][0],
                           mode=_hist_mode,
                           # MEASURED period, not delta_t — see the tag registry for why.
                           period_ms=_hist_period_ms,
                           # when this tick's CAN read ran (CLOCK_REALTIME, the clock candump stamps frames with)
                           can_read_t=_can_age_buf[2],
                        )
        # ...and the same tick into the shared-memory trace for recorder.py (one C call of these arrays, ~9 us)
        trace.write(_hist_t, st.loop_execution_time, _tick_now, shared_data.rti["rti_seq_rx_last"][0],
                    shared_data.rti["control_success_rate"][0], shared_data.rti["rti_consecutive_misses"][0],
                    shared_data.rti["rti_substate"][0], _hist_mode, _hist_period_ms, _can_age_buf[2])
        _profile_last = _profile_mark(_profile_last, "joint_history")

    def _measure():
        """The ordered measurement pipeline: motor[] -> joint[] + tcp[] -> the Joint_data history entry."""
        nonlocal _profile_last
        _measure_motor()
        # ==========================================================================
        # Fill the latest data shared arrays (Joint + TCP level) — via update_shared_data.py
        # ==========================================================================
        update_joint_and_tcp_data(
            shared_data=shared_data,
            Joint=Joint,
            Joint_data=Joint_data,
            filter_banks=st.filter_banks,
            joint_num=joint_num,
            delta_t=delta_t,
            gear_ratio=gear_ratio,
            gear_efficiency=gear_efficiency,
            motor_kt=motor_kt,
            motor_signs=motor_signs,
            robot=robot,
            jacobian_out=tcp_jacobian,
            observer=ext_observer,
        )
        _profile_last = _profile_mark(_profile_last, "joint_tcp_shared")
        _history_entry()

    def _measure_fast():
        """Option B, BEFORE the send (2026-10-12, TODO/can_tx_timing_plan.md §9.10): what this tick's commands need —
        motor[], the joint position, speed (raw + filtered) and gravity; in IDLE also the TCP pose, Jacobian and raw
        twist (the haptic walls). Nothing a handler or dispatch reads may come only from _measure_rest()
        (pinned by test_can_read_first)."""
        nonlocal _profile_last
        _measure_motor()
        update_joint_state(shared_data=shared_data, Joint=Joint, filter_banks=st.filter_banks, joint_num=joint_num,
                           gear_ratio=gear_ratio, gear_efficiency=gear_efficiency, motor_kt=motor_kt,
                           motor_signs=motor_signs, robot=robot)
        if st.current_mode == RobotMode.IDLE:      # the freedrive walls (handle_idle); on a tick that ENTERS IDLE they
            update_tcp_pose(shared_data=shared_data, robot=robot, jacobian_out=tcp_jacobian)   # use the last tick's
        _profile_last = _profile_mark(_profile_last, "measure_fast")

    def _measure_rest():
        """Option B, AFTER the send: the rest of the measurement, then this tick's history entry — which now pairs this
        tick's measurement with this tick's commands by itself."""
        nonlocal _profile_last
        update_joint_rest(shared_data=shared_data, Joint_data=Joint_data, filter_banks=st.filter_banks,
                          joint_num=joint_num, delta_t=delta_t, gear_ratio=gear_ratio, gear_efficiency=gear_efficiency,
                          motor_kt=motor_kt, motor_signs=motor_signs, robot=robot, jacobian_out=tcp_jacobian,
                          observer=ext_observer)
        _profile_last = _profile_mark(_profile_last, "joint_tcp_shared")
        _history_entry()

    def _fault_detail(key):
        """The measured value behind a key that latches THIS tick (utility/fault_details.py, 2026-10-15) — the error
        list's description and the activity entry's message. Cold: _update_error_metadata asks only for new keys."""
        return fault_detail(key, safety_monitor.details, shared_data.rti, shared_data.motor, shared_data.gripper,
                            Motor_data, Gripper_data, int(shared_data.rti["loop_tick"][0]), delta_t, joint_num)

    def _check_safety():
        """The EXEC link watchdog, the loop monitor, the safety monitor and the error pass (latches, mode)."""
        nonlocal _profile_last
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
            tick=int(_tick_now),
            in_exec=(st.current_mode == RobotMode.EXEC),
            samples_remaining=float(shared_data.rti["exec_samples_remaining"][0]),
        )

        # RT loop health: LOOP_DEGRADED (warning, self-clearing) and LOOP_CRITICAL (hard,
        # latching → DISABLED + hold). Fed p99, so a slow burst only warns; critical = a sustained
        # loss (p99 > 1.5x for 1 s) or ONE stalled tick (> 3x). See error_checks.LOOP_ERROR_*.
        _loop_degraded, _loop_critical = _loop_monitor.update(
            p99_period=float(shared_data.rti["p99_loop_period"][0]),
            tick=int(_tick_now),
            period=float(shared_data.rti["loop_period"][0]),
        )
        if _gc_pauses.pending:                     # a garbage collection over 5 ms paused the loop: log it (rare, cold)
            _gc_pauses.flush(int(_tick_now))

        # Safety monitor BEFORE the error pass, so a trip latches — and disables the robot — this tick.
        _monitor_errors = safety_monitor.step(st.current_mode)
        if homing_ctrl.has_failures:                 # a failed homing: a latched error naming its reason (2026-10-14)
            st.homing_failures.update(homing_ctrl.take_failures())
        _profile_last = _profile_mark(_profile_last, "safety_monitor")

        st.clear_error_countdown, st.latched_errors, st.error_metadata, st.current_mode = process_errors_tick(
            shared_data, Motor, Motor_errors_data, joint_num,
            st.clear_error_countdown, st.latched_errors, st.error_metadata,
            active_CAN_gripper, st.current_mode, homing_ctrl,
            check_estop_active(),
            Gripper_data=Gripper_data,
            exec_link_lost=_exec_link_lost,
            # RTI command watchdog. A KEY, not a direct error_active write — this
            # function owns that flag and recomputes it every tick.
            rti_link_lost=rti_handler.take_link_lost(),
            # The strict command checks' keys (J{i}:RTI_*_LIMIT), raised once the controlled stop is at rest.
            rti_errors=rti_handler.take_command_errors(),
            # J{i}:CAN_REPLY_MISSED_WARNING (warn-only) — a joint's replies missing RTI's read too often.
            can_warnings=can_reply_monitor.warnings,
            # In simulation (utility/sim_mode) the loop has no real-time core or priority and shares two cores with the
            # simulator: a critically slow loop is reported as LOOP_DEGRADED, never latched — timing is not simulated.
            loop_degraded=_loop_degraded or (_SIM and _loop_critical),
            loop_critical=_loop_critical and not _SIM,
            monitor_errors=_monitor_errors,
            monitor_warnings=safety_monitor.warnings,
            # The settle after the start-up clear (sent at tick BOOT_CLEAR_TICK): a driver reports the watchdog trip the
            # last run's end and the set-up left it until the clear lands.
            driver_faults=_tick_now > BOOT_CLEAR_TICK + CLEAR_ERROR_SETTLE_TICKS,
            homing_errors=st.homing_failures,
            detail_of=_fault_detail,
        )
        if st.homing_failures:
            st.homing_failures.clear()             # latched by the call above (the latch keeps them until Clear Errors)
        _err_now = bool(shared_data.rti['error_active'][0])
        if _err_now and not _gripper_err_prev[0]:  # an error latched: the jaws stop where they are (2026-10-24)
            _freeze_gripper("an error latched")
        _gripper_err_prev[0] = _err_now
        _profile_last = _profile_mark(_profile_last, "error_checks")

    # Python's garbage collector (2026-10-24, utility/gc_pauses.py). Every object start-up made is frozen, so a full
    # collection inside a tick no longer walks them: one such tick (~70 ms) latched LOOP_CRITICAL a few seconds after
    # every homing since 2026-10-22. The two collections freeze_startup() times run HERE, before the first tick; from
    # here on every collection over 5 ms is logged with its generation (the check sits in _check_safety).
    freeze_startup()
    _gc_pauses = GcPauseMonitor()
    _gc_pauses.install()

    # ==========================================================================
    # RTI superloop
    # ==========================================================================
    while True:

        shared_data.rti["loop_tick"][0]  = shared_data.rti["loop_tick"][0]  + 1
        _tick_now = shared_data.rti["loop_tick"][0]      # read once per tick; nothing changes it within a tick
        time1 = time.perf_counter()
        _profile_last = time1

        # ==========================================================================
        # Handle GPIO inputs and outputs
        # ==========================================================================
        read_inputs_into_shared()
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
        _stats_tick = _tick_now
        _stats_boot_wipe = _stats_tick == LOOP_STATS_WARMUP_TICKS
        _stats_warmup_wipe = _stats_tick == 2 or _stats_boot_wipe
        if _stats_warmup_wipe or shared_data.loop_stats_reset_signal.get() == "1":
            if _stats_boot_wipe:
                log_loop.info(
                    f"Boot peaks discarded at tick {int(LOOP_STATS_WARMUP_TICKS)} — "
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
            st._loop_stats_buf_pos, st._loop_stats_buf_count = reset_rolling_stats()
            shared_data.loop_stats_reset_signal.set("0")
            if _stats_tick == 2:
                # Tick 1 seeded the EMAs with that boot-length sample too. Zeroing
                # avg_loop_duration makes update_rt_loop_stats re-seed all three from
                # tick 2's real sample, instead of decaying the garbage over ~60 ticks.
                shared_data.rti["avg_loop_duration"][0] = 0.0

        (st.prev_time,                                        # updated to time1 — becomes prev_time for the next tick
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
            time1, st.prev_time, st.loop_execution_time, delta_t,
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
        (st._loop_stats_buf_pos, st._loop_stats_buf_count,
         shared_data.rti["p95_loop_period"][0],
         shared_data.rti["p99_loop_period"][0],
         shared_data.rti["p95_loop_execution_time"][0],
         shared_data.rti["p99_loop_execution_time"][0],
        ) = update_rolling_percentiles(
            _loop_period_buf, _loop_exec_buf, st._loop_stats_buf_pos, st._loop_stats_buf_count,
            shared_data.rti["loop_period"][0], st.loop_execution_time,
            _tick_now,
            shared_data.rti["p95_loop_period"][0],
            shared_data.rti["p99_loop_period"][0],
            shared_data.rti["p95_loop_execution_time"][0],
            shared_data.rti["p99_loop_execution_time"][0],
            stats_interval=LOOP_STATS_INTERVAL,
        )
        _profile_last = _profile_mark(_profile_last, "timing_stats")

        # ==========================================================================
        # Start-up clear (2026-10-05; here, at tick BOOT_CLEAR_TICK, since 2026-10-13)
        # ==========================================================================
        # The drivers start with a watchdog trip: the last run's end left one, and the ~0.5 s of set-up between the
        # config preload and tick 1 (the joint walls' inertia search) trips them again with a 100 ms watchdog. A
        # command only restarts a drive's watchdog timer, only a clear removes a trip — so the clear goes out after tick
        # 1's commands restarted every timer. One clear to every driver; the loop ignores what the drivers REPORT until
        # the settle after it is over (driver_faults in the error pass). Connection losses, the E-stop, the monitor and
        # every system error are read from tick 1.
        if _tick_now == BOOT_CLEAR_TICK:
            send_clear_errors(shared_data, Motor, Motor_errors_data, joint_num, CLEAR_ERROR_SEND_REPEATS,
                              Gripper_data=Gripper_data, active_CAN_gripper=active_CAN_gripper, all_motors=True)
            log_errors.info("start-up clear sent to every driver at tick %d; their own faults are read from "
                         "tick %d.", BOOT_CLEAR_TICK, BOOT_CLEAR_TICK + CLEAR_ERROR_SETTLE_TICKS + 1)

        # ==========================================================================
        # Run selfcheck (only once at startup)
        # ==========================================================================
        if st.startup_selfcheck == 0 and _tick_now == 8:
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

            st.startup_selfcheck = 1
            st.pending_mode_request = RobotMode.IDLE  # selfcheck done → transition out of BOOTING
        _profile_last = _profile_mark(_profile_last, "startup_selfcheck")

        # ==========================================================================
        # Multi-shot motor reconfiguration at ticks 50 / 150 / 300 
        # Its a dirty fix for weird BUG caused our exec time to be double until we home the robot?????
        # Multiple passes cover motors with different boot times.
        # ==========================================================================
        _tick = _tick_now
        if st.startup_motor_reconfig < 3 and _tick in (50, 150, 300):
            _reload_motor_config()
            _reload_gripper_config()
            st.startup_motor_reconfig += 1

        # ==========================================================================
        # OPTION B — read first, send early (TODO/can_tx_timing_plan.md §9; system.xml <can_read_first>, ON by default).
        # The CAN read and the measurement run HERE, at the tick's start: they collect the replies to the previous
        # tick's commands, which left right after its dispatch (the waiter below), so this tick's commands are
        # computed from them — about a tick fresher — and leave ~0.3 ms after dispatch instead of when RTI sleeps.
        # HOMING keeps the old order (user, 2026-10-12): it sends its own frames, which the waiter does not watch.
        # Off: the same functions run where they always did (below), the old order exactly.
        # ==========================================================================
        _read_first = first_frame_wait.enabled and st.current_mode != RobotMode.HOMING
        if _read_first:
            _bus_silent = (st.current_mode == RobotMode.FLASHING)
            _can_sent_prev = _can_sent_now       # this read collects the replies to the LAST dispatch's commands
            _can_read()
            _measure_fast()

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

            # The dashboard's "CMD:" display (256 bytes). Truncated, never raising: this runs BEFORE the isolated
            # handler call below, and a long command (set_joint_walls, ~1.1 kB) used to raise here and kill RTI.
            shared_data.robot_command.set_truncated(command)

            # Route mode-change commands to the state machine
            _handler = _cmd_msg = _cmd_dict = None
            try:
                _cmd_dict = json.loads(command)
                _cmd_msg = _cmd_dict.get("msg", "").lower()
                _handler = RTI_COMMAND_HANDLERS.get(_rti_command_key(_cmd_msg))
            except (json.JSONDecodeError, AttributeError):
                pass  # Non-JSON or unexpected format — not a mode command
            if _handler is not None:
                # THE HANDLER CALL IS ISOLATED, and this is a safety property rather than
                # tidiness. Before 2026-08-21 an exception in ANY of the ~30 command handlers
                # propagated straight out of the superloop and killed RTI.py — the real-time
                # controller — leaving the arm to its watchdog. It was not hypothetical: a
                # one-character bug in set_recipe (calling a @property) crashed the runtime the
                # first time anything called that command.
                #
                # A bad command must cost the command, never the loop. Logged CRITICAL with the
                # offending message and a full traceback so it is loud rather than silent —
                # swallowing it quietly would be its own failure mode.
                _mode_request_before = st.pending_mode_request
                st.command_id = _cmd_dict.get("id")
                note_origin(st.command_id, _cmd_dict.get("origin"))   # who sent it: its activity entries name them
                try:
                    _handler(_cmd_msg, _cmd_dict)
                except Exception:
                    logger.critical("Command handler RAISED on %r — command IGNORED, the "
                                    "control loop continues.\n%s",
                                    _cmd_msg, traceback.format_exc())
                if st.pending_mode_request is not None and st.pending_mode_request is not _mode_request_before:
                    st.pending_mode_request_id = st.command_id      # this command asked for the mode change
                st.command_id = None

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
        if st.rti_pc_ip is None or st.rti_transport != "tcp" or st.rti_tcp_conn is not None:
            # Nothing is waiting to be accepted into a session right now, so REAP whatever is
            # sitting in the listener's queue. Skipping this lets probes accumulate until the
            # backlog fills and the port stops answering — see drain_rti_tcp_backlog.
            drain_rti_tcp_backlog(RTI_tcp_listener)
        if st.rti_pc_ip is None:
            rti_udp_data = None
        elif st.rti_transport == "tcp":
            # Accept at most one stream, only from the paired IP. Non-blocking, so a tick with
            # nobody dialling in costs one failed syscall.
            if st.rti_tcp_conn is None:
                _conn, _addr = accept_rti_tcp(RTI_tcp_listener, st.rti_pc_ip)
                if _conn is not None:
                    st.rti_tcp_conn = _conn
                    rti_tcp_sender.reset(_conn)
                    del rti_tcp_rxbuf[:]
                    log_handler.info("TCP stream accepted from %s", _addr[0])
            if st.rti_tcp_conn is None:
                rti_udp_data = None
            else:
                rti_udp_data, rti_udp_count = drain_rti_tcp(
                    st.rti_tcp_conn, rti_tcp_rxbuf, CMD_SIZE)
                if rti_udp_count < 0:
                    # An ORDERLY close (process exit, socket closed) is a real disconnect signal
                    # that UDP cannot give us, and we act on it immediately.
                    #
                    # BUT NOT EVERY DEATH IS ORDERLY. A yanked cable or a hard-crashed PC sends
                    # no FIN, so the socket stays open and TCP looks exactly like UDP silence —
                    # SO_KEEPALIVE below reaps it eventually, and the handler's command timeout
                    # is what actually stops the arm. Do not read this branch as "TCP always
                    # detects disconnects"; it detects the polite ones.
                    _drop_rti_tcp("peer closed the stream")
                    rti_udp_data, rti_udp_count = None, 0
                elif rti_tcp_sender.failed:
                    _drop_rti_tcp("PC stopped reading; state backlog past the cap")
                    rti_udp_data, rti_udp_count = None, 0
        else:
            rti_udp_data, rti_udp_count = drain_rti_udp(
                RTI_socket_receive, allowed_ip=st.rti_pc_ip, allowed_port=None)
            if rti_udp_data is not None:
                # Process data. Lazy %s, not an f-string: this runs every streaming tick and the root
                # logger is at INFO, so the packet's repr is never built unless DEBUG is on (hot-path rule).
                log_handler.debug("Received valid RTI data: %s", rti_udp_data)

        # Update per-tick TickContext fields (attribute swap, no allocation)
        # The handler sends its state packet through ctx.socket_sender.sendto() and must not
        # know which transport that is — see _TcpStateSender.
        ctx.socket_sender = rti_tcp_sender if st.rti_transport == "tcp" else RTI_socket_sender
        ctx.rti_udp_data = rti_udp_data
        ctx.rti_udp_count = rti_udp_count
        ctx.current_tick = int(_tick_now)
        # Resolved here rather than at the telemetry block below because RTI mode's
        # handler serialises its own state packet during dispatch, which runs first.
        # recipe_mgr.get() is a cached name comparison, so this is the same one
        # resolution per tick it always was — the telemetry block reuses it.
        ctx.recipe = recipe = recipe_mgr.get(shared_data.active_recipe.get())
        _profile_last = _profile_mark(_profile_last, "udp_rx_context")

        # The Hall sensor check (robotics/hall_check.py): read the joint's pin; stop it — the joint's limits restored —
        # BEFORE a mode change takes effect (homing sends its own limits on entry) or once the robot left IDLE.
        if hall_check.active:
            hall_check.tick(st.current_mode == RobotMode.IDLE and st.pending_mode_request is None)
        # A joint freed for hand turning (robotics/free_joint.py): held again the same way, and after its timeout.
        if free_joint.active:
            free_joint.tick(st.current_mode == RobotMode.IDLE and st.pending_mode_request is None)
        # A test move (robotics/joint_test.py): this tick's command for its joint, or stopped — before dispatch.
        if joint_test.active:
            joint_test.tick(st.current_mode == RobotMode.IDLE and st.pending_mode_request is None
                            and shared_data.robot_state.get() == "ENABLED")

        # ==========================================================================
        # Perform robot state and mode management
        # ==========================================================================
        if st.pending_mode_request is not None:
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
            _park_ok = bool(st.pending_mode_force)
            success, new_mode, reason = request_mode_change(
                st.current_mode, st.pending_mode_request,
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
                    if st.pending_mode_assertion == "FORCE":
                        log_flash.warning(
                            f"Entering {new_mode} via FORCE — operator "
                            f"asserts the arm is supported WITHOUT parking "
                            f"(gravity {_worst_gravity:.2f} Nm).")
                    else:
                        log_flash.info(
                            f"Entering {new_mode} — operator confirmed "
                            f"the arm is PARKED (gravity {_worst_gravity:.2f} Nm).")

                # Exit old mode handler (e.g. homing restores motor configs)
                _old_handler = MODE_HANDLERS.get(st.current_mode)
                if _old_handler is not None and hasattr(_old_handler, 'exit'):
                    _old_handler.exit(ctx)

                # Leaving FLASHING: re-base the CAN freshness clock. Nodes were
                # silent for the whole flash (deliberately — we stopped polling), so
                # tick_diff is now thousands of ticks and update_can_connection_status
                # would latch a disconnect error on every node before any of them got
                # a chance to answer. Stamping "last heard from" to now gives them the
                # normal error_threshold window to come back; anything that genuinely
                # stays dead is still caught 50 ticks later.
                if st.current_mode == RobotMode.FLASHING:
                    _now_tick = _tick_now
                    for _i in range(joint_num + active_CAN_gripper):
                        Motor_data[_i][-1]["tick"] = _now_tick
                    log_flash.info("CAN freshness re-based on FLASHING exit")

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
                            log_flash.error("Firmware was flashed — "
                                          "HOMING INVALIDATED. Re-home before moving.")
                            log_activity(_rti_activity_logger, "RTI", "warning", "HOMING_INVALIDATED",
                                         message="firmware was flashed")
                    except OSError as _err:
                        # Never let housekeeping take the loop down; fail LOUD and
                        # unhomed rather than silently keeping a stale reference.
                        shared_data.homed_state.set("NOT HOMED")
                        shared_data.rti["homing_status"][:] = 0
                        shared_data.rti["joint_homed"][:] = 0
                        log_flash.error(f"Could not read the flash marker "
                                      f"({_err}) — clearing homing anyway.")

                _from_mode = st.current_mode
                st.current_mode = new_mode
                shared_data.robot_mode.set(st.current_mode)
                log_mode.info(f"→ {st.current_mode}")
                log_activity(_rti_activity_logger, "RTI", "info", "MODE_CHANGE", id=st.pending_mode_request_id,
                             message=f"{_from_mode} → {st.current_mode}")

                # Apply the streaming contract BEFORE enter(). Both RTI modes share one
                # handler, so the mode is the only thing that says which contract this session
                # runs under — timeout, generator, pacing. Set before enter() so claim_control()
                # seeds the limiter with the settings that will actually be used.
                if new_mode in (RobotMode.RTI, RobotMode.SOFT_RTI):
                    rti_handler.apply_mode_contract(
                        soft=(new_mode == RobotMode.SOFT_RTI),
                        rate_hz=st.pending_mode_rate_hz,
                    )

                # Enter new mode handler
                _new_handler = MODE_HANDLERS.get(new_mode)
                if _new_handler is not None and hasattr(_new_handler, 'enter'):
                    _new_handler.enter(ctx)
            else:
                log_mode.warning(f"Request '{st.pending_mode_request}' rejected: {reason}")
                log_activity(_rti_activity_logger, "RTI", "warning", "MODE_REJECTED", id=st.pending_mode_request_id,
                             message=f"{st.current_mode} → {st.pending_mode_request}: {reason}")
            st.pending_mode_request = None
            st.pending_mode_request_id = None
            st.pending_mode_force = False
            st.pending_mode_assertion = ""
            st.pending_mode_rate_hz = None
        _profile_last = _profile_mark(_profile_last, "mode_state")


        # ==========================================================================
        # Mode dispatch — each handler sends its own CAN commands
        # ==========================================================================
        loop_delay_meter.before_dispatch(_tick_now, _read_first,
                                         st.current_mode != RobotMode.HOMING and st.current_mode != RobotMode.FLASHING)
        if loop_delay_meter.samples != _loop_delay_published:          # a sample finished: publish (5 times a second)
            _loop_delay_published = loop_delay_meter.samples
            shared_data.rti["loop_delay"][:joint_num] = loop_delay_meter.median
            shared_data.rti["loop_delay_max"][:joint_num] = loop_delay_meter.maximum
        shared_data.rti["can_read_first"][0] = _read_first
        rti_handler.defer_state = _read_first   # option B: the state packet after the send (flush_state, below)
        if first_frame_wait.enabled:
            first_frame_wait.prepare()           # stale echoes out BEFORE this tick's commands are queued
        _tick_result = dispatch_step(MODE_HANDLERS.get(st.current_mode), ctx)
        # Every mode but HOMING / FLASHING (they send their own way, or nothing) just sent each joint a data command.
        _can_sent_now = st.current_mode != RobotMode.HOMING and st.current_mode != RobotMode.FLASHING
        if first_frame_wait.enabled:
            # Option B's wait (option A's waiter): sleep until J1's command has left, so the joints' exchange runs
            # during this tick.
            if _can_sent_now:
                first_frame_wait.wait()
            else:
                first_frame_wait.skip()

        if _tick_result.pending_mode is not None:
            st.pending_mode_request = _tick_result.pending_mode
        _profile_last = _profile_mark(_profile_last, "mode_dispatch")

        # FLASHING hands the bus to an external flasher process. RTI must put NOTHING
        # on can0 while it holds: the handler returning SELF_MANAGED covers the
        # dispatch send, gripper_owned covers the gripper block below, and this flag
        # covers the two remaining transmitters (telemetry polling, reconnect config
        # reloads). See utility/state_machine.py MAINTENANCE_MODES.
        _bus_silent = (st.current_mode == RobotMode.FLASHING)

        if _tick_result.gripper_owned:
            st._gripper_was_owned = True
        else:
            if st._gripper_was_owned:
                # HANDOVER: homing (or FLASHING) just gave the gripper CAN slot back.
                #
                # Homing sends its own full packs — _send_idle REPLAYS the last
                # <gripper_move> every tick to hold the jaws through later phases — so the
                # firmware can be left with action_status=1 holding a position that RTI
                # never issued and does not know about. Empty polls cannot correct that:
                # only a len==5 frame assigns action_status.
                #
                # So force idle and let the announcement below transmit it. Homing ends with
                # the arm in IDLE, and this makes the gripper match: no active command on
                # either. It also repairs Stop, which would otherwise do nothing until the
                # operator had issued one Open/Close to arm the handshake.
                st._gripper_was_owned = False
                _gripper_fw_state['action'] = 0
                _gripper_fw_state['_idle_repeats'] = GRIPPER_IDLE_PACK_REPEATS
            if st._gripper_calibrate_pending and active_CAN_gripper:
                Motor[joint_num].Send_gripper_calibrate()
                log_gripper.info("firmware calibrate sent")
                st._gripper_calibrate_pending = False
                st._gripper_calibrate_running = True
                st._gripper_calibrate_ticks   = 0
            elif st._gripper_calibrate_running and active_CAN_gripper:
                # Empty poll — keeps the watchdog alive without overwriting the
                # calibration command in firmware. Stays in this state until the
                # user clicks Open/Close/Goto/Stop (which aborts via gripper_set)
                # or the safety timeout fires.
                Motor[joint_num].Send_gripper_data_pack()
                st._gripper_calibrate_ticks += 1
                if st._gripper_calibrate_ticks > _GRIPPER_CALIBRATE_MAX_TICKS:
                    log_gripper.warning("firmware calibration timed out")
                    st._gripper_calibrate_running = False
            else:
                send_gripper_commands(
                    Motor, dummy_device, active_CAN_gripper, joint_num,
                    ctrl_mode=int(shared_data.rti['gripper_ctrl_mode'][0]),
                    gripper_state=_gripper_fw_state,
                    calibrated=bool(shared_data.gripper['gripper_calibrated'][0]),
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


        if not _read_first:
            _can_read()

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
                startup_selfcheck=st.startup_selfcheck,
                prev_old_data_warning=_prev_old_data_warning,
                reload_last_tick=_reconnect_last_tick,
                reload_pending=_reconnect_pending,
                cooldown_ticks=_reconnect_cooldown_ticks,
                motor_watchdog_rate=motor_watchdog_rate,
                gripper_watchdog_rate=gripper_watchdog_rate,
                motor_velocity_limit=motor_velocity_limit,
                motor_Ilim=motor_Ilim,
                motor_voltage_limit=motor_voltage_limit,
                motor_KPP=motor_KPP, motor_KP=motor_KP, motor_KD=motor_KD,
                motor_KPIQ=motor_KPIQ, motor_KIIQ=motor_KIIQ,
                motor_KPV=motor_KPV, motor_KIV=motor_KIV,
                gripper_velocity_limit=st.gripper_velocity_limit,
                gripper_Ilim=st.gripper_Ilim,
                gripper_voltage_limit=st.gripper_voltage_limit,
                gripper_KP=st.gripper_KP, gripper_KD=st.gripper_KD,
                gripper_KPIQ=st.gripper_KPIQ, gripper_KIIQ=st.gripper_KIIQ,
                gripper_KPV=st.gripper_KPV, gripper_KIV=st.gripper_KIV,
                gripper_KPP=st.gripper_KPP,
            )
        _profile_last = _profile_mark(_profile_last, "motor_reconnect")

        if _read_first:
            # Option B: the rest of the measurement and the history entry now, after the commands left; then the RTI
            # state packet (this tick's measurement with this tick's command).
            _measure_rest()
            rti_handler.flush_state(ctx)
        else:
            _measure()

        if active_CAN_gripper:
            process_gripper_joint_data(
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



        _check_safety()        # both orders here, after the send (user, 2026-10-12: a trip acts on the next tick's commands)

        # ==========================================================================
        # Send UDP data — recipe-driven, switchable configuration via .xml
        # ==========================================================================
        # Recipe and filter reloads arrive as commands carrying the parsed data (reload_recipes / reload_filters,
        # 2026-10-04) — the loop reads no file for them; the two shared-memory signals are no longer polled.
        # `recipe` was resolved into ctx near the top of this tick (see udp_rx_context).
        payload = build_payload(recipe, Joint_data, Gripper_data, motor_buffers, joint_num)
        broadcast_udp_telemetry(telemetry_socket, payload)
        _profile_last = _profile_mark(_profile_last, "telemetry")



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
        if _HB_ENABLED and int(_tick_now) % _HB_INTERVAL == 0:
            print_heartbeat(shared_data)
        _profile_last = _profile_mark(_profile_last, "heartbeat")

        # ==========================================================================
        # Sleep for the rest of the loop
        # ==========================================================================
        shared_data.rti["timestamp"][0] = time.perf_counter()
        st.loop_execution_time = shared_data.rti["timestamp"][0] - time1
        shared_data.rti["loop_execution_time"][0] = st.loop_execution_time
        _profile_last = _profile_mark(_profile_last, "timing_finalize")
        _profile_report(st.loop_execution_time)
        # Names the section responsible when THIS tick blew its budget (RTI_OVERRUN_TRACE=1).
        # No-op unless enabled, and only logs on an actual overrun.
        _overrun_trace(st.loop_execution_time, _tick_now, st.current_mode)

        # Absolute-deadline pacing (2026-10-04): the next tick is due at the previous deadline + delta_t, so the
        # average period is exactly delta_t (it was 4.024 ms with "delta_t - work"); a stall re-bases, never a
        # catch-up burst. utility/signal_processing.pace_deadline, net test_loop_pacing.
        st.next_tick_deadline, sleep_time = pace_deadline(st.next_tick_deadline, time.perf_counter(), delta_t)

        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
