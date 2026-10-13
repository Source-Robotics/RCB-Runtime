#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: shared_data.py
Author: Petar Crnjak
Company: Source Robotics
Date: 13.5.2025
Version: 1.2
Description: 
    Structured shared memory setup for RTI, including motor, joint, TCP, 
    gripper, GPIO, and RTI process data, with clean memory names.
    Supports runtime joint count change via shared_data.setup(joints=N)
"""

import numpy as np
from multiprocessing import shared_memory, resource_tracker
import json
import argparse
import os

# Test namespace (2026-10-02, TODO/refactor_plan.md Phase 0.2). Segment names are fixed, so any process that calls
# setup() on the robot attaches to RTI's LIVE memory — including an offline test net, which then reads or WRITES
# the running robot's state (test_kt_source.py wrote motor_Kt_source[:] = 9). With RCB_SHM_PREFIX set, every
# segment name gets that prefix: a net creates its own private segments and cannot reach the robot's. tools/run_nets.py
# sets it per net; dummy_test_scripts/_shm_sandbox.py sets it for a net run by hand. Unset (the robot's processes,
# the dashboard, the api_tests) = the names are exactly what they always were.
_SHM_PREFIX = os.environ.get("RCB_SHM_PREFIX", "")

# ======================= SharedString Class ==========================
class SharedString:
    """A wrapper for string shared memory with 4-byte length prefix."""
    def __init__(self, name: str, size: int = 256):
        name = _SHM_PREFIX + name
        self.size = size
        try:
            self.shm = shared_memory.SharedMemory(name=name, create=True, size=size)
            self.shm.buf[:size] = b'\x00' * size
            self.created = True
            print(f"[INFO] Shared string '{name}' created ({size} bytes).")
        except FileExistsError:
            self.shm = shared_memory.SharedMemory(name=name)
            self.created = False
            print(f"[INFO] Shared string '{name}' attached (already exists).")
        # Prevent resource_tracker from unlinking this segment on process exit;
        # lifetime is managed explicitly (shared across processes).
        resource_tracker.unregister(self.shm._name, "shared_memory")
        self.name = name

    def set_json(self, data):
        """Converts Python list/dict to JSON and saves it."""
        json_str = json.dumps(data)
        self.set(json_str)

    def get_json(self):
        """Reads from memory and converts JSON back to Python objects."""
        raw = self.get()
        if not raw or raw == "":
            return []  # Return empty list if memory is empty
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return []


    def set(self, value: str):
        byte_data = value.encode("utf-8")
        if len(byte_data) > self.size - 4:
            raise ValueError("String too long for shared memory block")
        self.shm.buf[:4] = len(byte_data).to_bytes(4, "little")
        self.shm.buf[4:4+len(byte_data)] = byte_data
        end = 4 + len(byte_data)
        pad_len = self.size - end
        self.shm.buf[end:end+pad_len] = b"\x00" * pad_len

    def set_truncated(self, value: str):
        """set(), cut to fit the block at a character boundary — for DISPLAY strings that must never raise.
        RTI.py copies every received command into robot_command (256 bytes) before handling it; a 1.1 kB
        set_joint_walls message made set() raise there and killed RTI (2026-10-01)."""
        data = value.encode("utf-8")
        if len(data) > self.size - 4:
            value = data[:self.size - 4].decode("utf-8", "ignore")
        self.set(value)

    def get(self) -> str:
        length = int.from_bytes(self.shm.buf[:4], "little")
        return bytes(self.shm.buf[4:4+length]).decode("utf-8")

    def close(self):
        self.shm.close()

    def unlink(self):
        self.shm.unlink()


# ======================= Module state ==========================
joint_num = 6
_initialized = False

# Global containers – filled by setup()
robot_mode = robot_state = error_state = None
homed_state = sim_mode = robot_command = None  # SharedString globals
force_confirm_who = None                   # SharedString: who is asking to force a control
                                            # takeover ("ip:port"), for the OLED prompt
motor_shms = motor_arrays = joint_shms = joint_arrays = None
tcp_shms = tcp_arrays = ext_shms = ext_arrays = None
shm_GPIO = gpio_states = None
control_board_shms = None
control_board = {}
control_board_data = None   # flat (4,) int32 — backward-compatible PCB register array
cb_command_ = cb_status = cb_current_error = None
active_recipe = None                       # SharedString: name of the active data recipe
recipe_reload_signal = None                # SharedString: set to "1" to trigger recipe_mgr.reload()
filter_reload_signal = None                # SharedString: set to "1" to rebuild filter banks from config/filters.xml (hot reload, no restart)
clear_error_signal = None                  # SharedString: GUI sets to "1" to clear the RTI error latch
loop_stats_reset_signal = None             # SharedString: set to "1" to zero max_jitter/max_loop_period/
                                            # max_loop_execution_time/overrun_count and the rolling p95/p99 window
rti_shms = gripper_shms = gripper_arrays = None
system_info_shms = {}  # Holds system-info SharedString references (prevents GC leak)
motor_config_shms = {}
motor_config = {}  # Per-joint PID gains and limits currently loaded to motor drivers


# ======================= Helper ==========================
def _attach_or_create(name: str, dtype, count: int):
    name = _SHM_PREFIX + name
    try:
        shm = shared_memory.SharedMemory(name=name)
        # --- FIX FOR LINUX ---
        # Stop the resource tracker from destroying this memory when this process exits
        resource_tracker.unregister(shm._name, "shared_memory")

        expected_size = np.dtype(dtype).itemsize * count
        if shm.size != expected_size:
            # Stale segment with wrong size (e.g. dtype changed) — recreate it.
            print(f"[WARN] Shared memory '{name}' has wrong size "
                  f"({shm.size} bytes, expected {expected_size}). Unlinking and recreating.")
            shm.close()
            shm.unlink()
            raise FileNotFoundError  # fall through to create branch

        arr = np.ndarray((count,), dtype=dtype, buffer=shm.buf)
        print(f"[INFO] Attached to existing shared memory: {name}")
    except FileNotFoundError:
        size = np.dtype(dtype).itemsize * count
        shm = shared_memory.SharedMemory(create=True, size=size, name=name)
        # --- FIX FOR LINUX ---
        # Stop the resource tracker from destroying this memory when this process exits
        resource_tracker.unregister(shm._name, "shared_memory")
        
        arr = np.ndarray((count,), dtype=dtype, buffer=shm.buf)
        arr[:] = 0
        print(f"[INFO] Created new shared memory: {name} ({size} bytes)")
    except Exception as e:
        raise RuntimeError(f"Failed to attach/create {name}: {e}")
    return shm, arr


# ======================= Main setup function ==========================
def setup(joints: int = 6, force_reinit: bool = False):
    """
    Call this to configure the number of joints.
    Can be used anytime – even after import.
    
    Examples:
        import RTI
        RTI.setup(7)     # 7-DOF robot
        RTI(7)           # shortcut
        RTI.setup()      # back to default 6-DOF
    """
    global joint_num, _initialized
    global robot_mode, robot_state, error_state, homed_state, sim_mode, robot_command
    global force_confirm_who
    global motor_shms, motor, joint_shms, joint
    global tcp_shms, tcp
    global shm_GPIO, gpio_states
    global control_board_shms, control_board, control_board_data
    global cb_command_, cb_status, cb_current_error, active_recipe, recipe_reload_signal, filter_reload_signal, clear_error_signal
    global loop_stats_reset_signal
    global rti, rti_shms, gripper_shms, gripper
    global system_info_shms
    global motor_config_shms, motor_config

    if _initialized and not force_reinit and joints == joint_num:
        return

    # Clean up old per-joint blocks if changing joint count
    if _initialized and joints != joint_num:
        print(f"[RTI] Reconfiguring from {joint_num} → {joints} joints – cleaning old blocks...")
        for shm in motor_shms.values():
            shm.close()
        for shm in joint_shms.values():
            shm.close()
        ext_shms["torque_external"].close()

    joint_num = max(1, joints)
    print(f"\n[RTI] Initializing shared memory for {joint_num}-DOF robot...\n")

    # ========================= String-based states =========================
    robot_mode  = SharedString("robot_mode", 256)
    sim_mode  = SharedString("sim_mode", 256)
    robot_state = SharedString("robot_state", 256)
    error_state = SharedString("error_state", 4096)
    homed_state = SharedString("homed_state", 256)
    robot_command = SharedString("robot_command", 256)
    # Who is asking to take control, e.g. "192.168.1.50:51377". The OLED must NAME them:
    # granting blind is how the wrong session ends up with the arm.
    force_confirm_who = SharedString("force_confirm_who", 256)

    strings_created = force_reinit or any(
        ref.created for ref in (robot_mode, sim_mode, robot_state, error_state, homed_state,
                                robot_command, force_confirm_who)
    )
    if strings_created:
        sim_mode.set("ON")
        robot_command.set("HOME")
        force_confirm_who.set("")
        robot_mode.set("IDLE")
        robot_state.set("DISABLED") # 2 States: "DISABLED" and "ENABLED"
        homed_state.set("NOT HOMED") # 2 States: "HOMED" and "NOT HOMED"
        error_state.set_json("NONE") # No errors → JSON string "NONE"; otherwise a list of error keys, e.g. 
        #[1774525371.103433 | tick 16] J1:WATCHDOG — watchdog error — motor did not receive a command in time.
        #[1774525371.154405 | tick 21] GRIPPER_MOTOR_TEMPERATURE — gripper motor is overheating.
    
    """
                [
            {
                "key": "J1:WATCHDOG",
                "timestamp": 1711468800.123456,
                "tick": 21,
                "description": "watchdog error — motor did not receive a command in time."
            },
            {
                "key": "GRIPPER_MOTOR_TEMPERATURE",
                "timestamp": 1711468800.123456,
                "tick": 21,
                "description": "gripper motor is overheating."
            }
            ]
    """

    system_info_shms = {}
    for name, size in [
        ("pi_model",          256),
        ("platform_release",  256),
        ("kernel_version",    256),
        ("distro_name",       256),
        ("distro_version",    256),
        ("cpu_processor",     256),
        ("cpu_architecture",  256),
        ("cpu_count",         256),
        ("memory_size",       256),
        ("memory_used",       256),
        ("memory_usage",      256),
        # Disk backing / — on a Pi that is usually the SD card, which is both the slowest and the
        # most likely thing to quietly fill up. disk_device is static; the rest move very slowly.
        ("disk_device",       256),
        ("disk_size",         256),
        ("disk_used",         256),
        ("disk_usage",        256),
    ]:
        system_info_shms[name] = SharedString(name, size)



    # ========================= Motor Data =========================
    motor_shms = {}
    motor = {}

    for name, dtype in [
        ("motor_position",      np.int32),
        ("motor_speed",         np.int32),
        ("motor_current",       np.int32),
        ("motor_temperature",   np.int32),
        ("global_motor_error",  np.int32),
        ("motor_voltage",       np.int32),
        ("motor_Kt",            np.float64),
        # Where motor_Kt actually came from, per joint: 1 = read from the motor over CAN,
        # 0 = XML fallback (motor did not answer, or kt_source=xml). Resolved once at startup
        # by utility/kt_init.py; the GUI shows it next to the Kt value.
        ("motor_Kt_source",     np.float64),
        ("HARDWARE_VERSION",    np.int32),
        ("BATCH_DATE",          np.int32),
        ("SOFTWARE_VERSION",    np.int32),
        ("SERIAL_NUMBER",       np.int32),
        ("HALL_INDEX",          np.int32),
        ("ADDITIONAL_PIN2",     np.int32),
        ("HALL_EDGE_DETECT",    np.int32),
        # ---- What was actually PUT ON THE CAN WIRE this tick (motor space) ----
        # Everything else in motor[] is measured FROM the driver; these three are what RTI
        # SENT TO it — the exact ints handed to Send_data_pack_1 / Send_data_pack_PD, after
        # the rad->ticks / Nm->mA conversion and the int() truncation. Captured at the send
        # site rather than recomputed by a reader, so the two can never drift apart.
        # float64 (not int32) so NaN can mean "channel not commanded this tick" — which is
        # genuinely different from 0: data_pack_1 omits an unset channel entirely, while a
        # commanded 0 is a real setpoint. Only data_pack_1 can leave a channel unset;
        # data_pack_PD always sends all three.
        # Written by utility/mode_dispatch.py _send(); covers the arm joints on the dispatch
        # path only — homing is SELF_MANAGED and does its own sends, and the gripper has its
        # own path, so neither shows up here.
        ("motor_position_sent", np.float64),
        ("motor_speed_sent",    np.float64),
        ("motor_current_sent",  np.float64)
    ]:
        shm, arr = _attach_or_create(name, dtype, joint_num)
        motor_shms[name] = shm
        motor[name] = arr

    # Motor Error / Status
    for name in [
        "motor_temperature_error",
        "motor_encoder_error",
        "motor_vbus_error",
        "motor_driver_error",
        "motor_velocity_error",
        "motor_current_error",
        "motor_estop_error",
        "motor_watchdog_error",
        "motor_calibrated"
    ]:
        shm, arr = _attach_or_create(name, np.int32, joint_num)
        motor_shms[name] = shm
        motor[name] = arr

    # ========================= Motor Config (PID / Limits) =========================
    motor_config_shms = {}
    motor_config = {}

    for name in [
        "cfg_KPP", "cfg_KPV", "cfg_KIV",
        "cfg_KPIQ", "cfg_KIIQ",
        "cfg_KP", "cfg_KD",
        "cfg_Ilim", "cfg_velocity_limit", "cfg_voltage_limit",
    ]:
        shm, arr = _attach_or_create(name, np.float64, joint_num)
        motor_config_shms[name] = shm
        motor_config[name] = arr

    # Gripper config — single values (one gripper)
    for name in [
        "cfg_gripper_KPP", "cfg_gripper_KPV", "cfg_gripper_KIV",
        "cfg_gripper_KPIQ", "cfg_gripper_KIIQ",
        "cfg_gripper_KP", "cfg_gripper_KD",
        "cfg_gripper_Ilim", "cfg_gripper_velocity_limit", "cfg_gripper_voltage_limit",
    ]:
        shm, arr = _attach_or_create(name, np.float64, 1)
        motor_config_shms[name] = shm
        motor_config[name] = arr

    # ========================= Joint Data =========================
    joint_shms = {}
    joint= {}

    for name, dtype, count in [
        ("joint_position",           np.float64, joint_num),
        # RAW vs _filtered split (see utility/signal_filters.py + config/filters.xml).
        # The no-suffix speed/accel/jerk/torque are now the RAW measured/derived values;
        # the _filtered twins are what the selectable per-joint filters produce. A signal
        # needs as many filter poles as it has differentiations (jerk=2 -> 2 poles), which
        # is why each derivative has its own filter stage rather than inheriting one.
        # joint_position has no _filtered twin — it is measured, not differentiated.
        ("joint_speed",              np.float64, joint_num),   # RAW (was filtered before 2026-07-17)
        ("joint_speed_filtered",     np.float64, joint_num),
        ("joint_torque",             np.float64, joint_num),   # RAW
        ("joint_torque_filtered",    np.float64, joint_num),
        ("joint_acceleration",      np.float64, joint_num),    # RAW = d(joint_speed)/dt
        ("joint_acceleration_filtered", np.float64, joint_num),  # d(joint_speed_filtered)/dt [+ optional own filter]
        ("joint_jerk",               np.float64, joint_num),   # RAW = d(joint_acceleration)/dt
        ("joint_jerk_filtered",      np.float64, joint_num),   # needs a 2-pole filter (2 differentiations)
        ("joint_position_commanded", np.float64, joint_num),
        ("joint_speed_commanded",    np.float64, joint_num),
        ("joint_acceleration_commanded", np.float64, joint_num),
        ("joint_torque_commanded",     np.float64, joint_num),
        ("joint_position_target", np.float64, joint_num),
        ("joint_speed_target",    np.float64, joint_num),
        ("joint_acceleration_target", np.float64, joint_num),
        ("joint_torque_target",     np.float64, joint_num),
        ("joint_gravity_torque", np.float64, joint_num),  # gravity compensation torque in Nm (joint space)
        ("joint_friction_comp_torque", np.float64, joint_num),  # friction compensation added this tick, Nm (0 when off)
        ("joint_inertia_torque",   np.float64, joint_num),  # inertia feedforward M(q)·qdd added this tick, Nm (0 when off)
        ("joint_coriolis_torque",  np.float64, joint_num),  # Coriolis feedforward C(q, qd)·qd added this tick, Nm (0 when off)
        ("compensation_torque",  np.int32,   joint_num),  # gravity compensation current in motor mA (motor space)
        ("jog_blocked",          np.int8,    joint_num),
    ]:
        shm, arr = _attach_or_create(name, dtype, count)
        joint_shms[name] = shm
        joint[name] = arr

    # ========================= TCP / Robot Pose =========================
    tcp_shms = {}
    tcp = {}

    for name, dtype, count in [
        ("robot_pose",                 np.float64, 6),
        ("robot_pose_matrix",          np.float64, 16),
        # Same measured orientation as the two above, as a unit quaternion.
        # ⚠️ ORDER IS (w, x, y, z) — SCALAR FIRST (spatialmath/Eigen/MATLAB convention,
        # which is what this stack already uses). Many other libraries want (x, y, z, w) —
        # reorder with q[[1, 2, 3, 0]]. Mixing them up yields a wrong rotation, not an error.
        # Canonicalized to w >= 0. Unlike robot_pose's RPY this has no gimbal lock, so it is
        # the channel to use for orientation error / SLERP. Measured only: the commanded and
        # target equivalents are derivable from TCP_pose_{commanded,target}_matrix.
        ("robot_pose_quaternion",      np.float64, 4),
        # TCP twist [vx,vy,vz, wx,wy,wz] in the WORLD/base frame. TCP_speed is J·q̇ from RAW
        # joint speed; TCP_speed_filtered is J·q̇_filtered (clean by propagation — no separate
        # TCP filter needed, the joint filter already did the work).
        ("TCP_speed",                  np.float64, 6),
        ("TCP_speed_filtered",         np.float64, 6),
        # TCP spatial acceleration [ax,ay,az, alx,aly,alz], world frame. _raw from raw joint
        # inputs; _filtered = analytic J·q̈_f + J̇·q̇_f through the 'tcp_accel' filter (config/
        # filters.xml) — the J̇·q̇ term is quadratic in velocity so it needs its own low-pass
        # even on filtered joint inputs.
        ("TCP_acceleration_raw",       np.float64, 6),
        ("TCP_acceleration_filtered",  np.float64, 6),
        ("jacobian_sigma_min",         np.float64, 1),
        ("jacobian_condition",         np.float64, 1),
        ("jacobian_manipulability",    np.float64, 1),
        ("TCP_force",                  np.float64, 6),
        ("TCP_pose_commanded",         np.float64, 6),
        ("TCP_pose_commanded_matrix",  np.float64, 16),
        ("TCP_speed_commanded",        np.float64, 6),
        ("TCP_pose_target",            np.float64, 6),
        ("TCP_pose_target_matrix",     np.float64, 16),
        ("TCP_speed_target",           np.float64, 6),
    # ========================= External Forces / Torques =========================
        ("torque_external", np.float64, joint_num),
        # External wrench [Fx,Fy,Fz, Mx,My,Mz] at the TCP. force_external is in the WORLD/base
        # frame (pinv(Jᵀ)·torque_external with the base Jacobian); force_external_tool is the
        # same wrench rotated into the TOOL frame (f_tool = Rᵀ·f_world per 3-vector). Same
        # physical force, different axes: world for absolute sensing (collision, weight), tool
        # for task-relative force (insertion along the tool axis, polishing normal to a surface).
        ("force_external",  np.float64, 6),
        ("force_external_tool", np.float64, 6)
    ]:
        shm, arr = _attach_or_create(name, dtype, count)
        tcp_shms[name] = shm
        tcp[name] = arr

    # ========================= Supervisor GPIO =========================
    shm_GPIO, gpio_states = _attach_or_create("GPIO", np.uint8, 16)

    # ========================= Control Board =========================
    control_board_shms = {}
    control_board = {}

    # Flat (4,) array kept for backward compatibility with supervisor.py and tools
    _cb_shm, control_board_data = _attach_or_create("memb_control_board", np.int32, 4)
    control_board_shms["memb_control_board"] = _cb_shm

    # --- Integer fields (np.int32, single value) ---
    for name in [
        "cb_filtered_VBUS1_mV",
        "cb_filtered_VBUS2_mV",
        "cb_filtered_Current_mA",
        "cb_relayState",
        "cb_PrechargeState",
        "cb_BrakeResistorState",
        "cb_error_state",
        "cb_SERIAL_NUMBER",
        "cb_HARDWARE_VERSION",
        "cb_BATCH_DATE",
        "cb_SOFTWARE_VERSION",
    ]:
        shm, arr = _attach_or_create(name, np.int32, 1)
        control_board_shms[name] = shm
        control_board[name] = arr

    # --- Float fields (np.float64, single value) ---
    for name in [
        "cb_power_usage",
    ]:
        shm, arr = _attach_or_create(name, np.float64, 1)
        control_board_shms[name] = shm
        control_board[name] = arr

    # --- String fields (SharedString) ---
    cb_command_      = SharedString("cb_command_",      256)
    cb_status        = SharedString("cb_status",        256)
    cb_current_error = SharedString("cb_current_error", 256)

    # ========================= Active data recipe =========================
    active_recipe = SharedString("active_recipe", 64)
    active_recipe.set("standard")   # default recipe — override from GUI

    # Signal flag: GUI sets to "1" after uploading a new recipe XML file;
    # RTI calls recipe_mgr.reload() and clears it back to "0".
    recipe_reload_signal = SharedString("recipe_reload_signal", 8)
    recipe_reload_signal.set("0")

    filter_reload_signal = SharedString("filter_reload_signal", 8)
    filter_reload_signal.set("0")

    # GUI sets to "1" to request an error latch clear; RTI resets it to "0" after handling.
    clear_error_signal = SharedString("clear_error_signal", 8)
    clear_error_signal.set("0")

    # GUI/client sets to "1" to zero the "since start" loop-timing peaks
    # (max_jitter/max_loop_period/max_loop_execution_time/overrun_count) and the
    # rolling p95/p99 window; RTI resets it to "0" after handling.
    loop_stats_reset_signal = SharedString("loop_stats_reset_signal", 8)
    loop_stats_reset_signal.set("0")

    # ========================= RTI Process Data =========================

    rti_shms = {}
    rti = {}
    for name, count in [
        ("scan_received_ids", 16),
        ("Error_counter",     16),
        ("old_data_warning",   16),
        ("received_ids",  joint_num + 1),
        ("latest_commands_id", joint_num + 1),
        # Live aggregate fault bit per node (arm joints + gripper), from msg_ID[2] of every reply.
        # Mode-independent: it rides the CAN ID, so it is valid whatever payload the node sent.
        # -1 = nothing heard from that node yet. See error_checks.py for how it gates errors.
        ("node_error_bits", joint_num + 1),
        ("CAN_node_connection_error", 16),
        ("homing_status",     joint_num + 1),   # joints + gripper slot: 0=idle,1=running,2=done,3=failed
        ("joint_homed",      joint_num + 1),   # joints + gripper slot: 0=not homed, 1=homed
        ("gripper_ctrl_mode", 1),              # 0=no gripper, 1=motor mode, 2=firmware mode
        # JOG live state — mirror of JogHandler config so new GUI tabs can read
        # the current values instead of stale XML defaults. Set on startup and
        # on every set_* call from the ZMQ command handler.
        ("jog_profile_idx",  1),               # 0=trapezoidal, 1=scurve
        ("jog_control_mode", 1),               # 0=pid, 1=pd
    ]:
        shm, arr = _attach_or_create(name, np.int32, count)
        rti_shms[name] = shm
        rti[name] = arr

    # Single-value float64 variables
    for name in [
        "timestamp",
        "loop_tick",
        # Which CAN pack carried this tick's joint motion command. The three
        # motor_*_sent arrays are the same three bytes either way, but the firmware
        # interprets them differently: 0 = data_pack_1 (cascade PID; Current is a
        # feedforward), 1 = data_pack_PD (firmware KP/KD impedance law on the position
        # and speed ERROR). NaN = nothing was dispatched this tick (e.g. homing, which
        # owns its own sends).
        "motor_pack_mode_sent",
        # --- RT loop timing stats ---
        "loop_period",              # Actual wall-clock duration of current tick (s)
        "loop_execution_time",      # Compute time used this tick (s); budget = delta_t
        "avg_loop_duration",        # EMA of loop_period
        "avg_loop_execution_time",  # EMA of loop_execution_time
        "jitter",                   # |loop_period - delta_t| this tick (s)
        "avg_jitter",               # EMA of jitter
        "max_jitter",               # Peak jitter since robot start (s)
        "max_loop_period",          # Peak loop period since robot start (s)
        "max_loop_execution_time",  # Peak execution time since robot start (s)
        "overrun_count",            # Cumulative ticks where execution_time > delta_t
        # max_jitter/max_loop_period/max_loop_execution_time/overrun_count above
        # never reset on their own — they persist in shared memory across RTI
        # restarts (same segment, re-attached, not recreated), so a single old
        # spike can silently pollute them for hours. These percentiles are
        # instead recomputed every tick from only the last ~500 samples (see
        # utility/signal_processing.py:update_rolling_percentiles), so they
        # always reflect current behavior with nothing to manually reset.
        # loop_stats_reset_signal (below, with the other one-shot signals)
        # still exists to explicitly zero the four "since start" values above.
        # CAN ingest lag: how long the oldest frame read this tick had been sitting in the
        # kernel buffer (seconds). From the kernel's SO_TIMESTAMPNS, so it measures how far
        # BEHIND the bus this loop is running — a few ms is healthy, 100 ms+ means RTI is
        # controlling on stale measurements and no PID tuning can compensate.
        # can_frame_age_peak is a since-start peak, cleared by loop_stats_reset_signal
        # alongside the other peaks.
        "can_frame_age",            # worst frame age this tick [s]
        "can_frame_age_min",        # best frame age this tick [s] — see the spread note above
        "can_frame_age_peak",       # peak since start/reset [s]
        "p95_loop_period",          # 95th percentile of loop_period over the last ~500 ticks
        "p99_loop_period",          # 99th percentile of loop_period over the last ~500 ticks
        "p95_loop_execution_time",  # 95th percentile of loop_execution_time over the last ~500 ticks
        "p99_loop_execution_time",  # 99th percentile of loop_execution_time over the last ~500 ticks
        # --- other ---
        "safety_violation",         # Latched by safety checks when level is ERROR/CRITICAL
        "CAN_gripper_connected",
        "error_active",
        "software_estop",           # GUI sets to 1 to assert software e-stop; 0 to release
        "robot_can_connected",      # 1.0 if all motor joints have no CAN errors, 0.0 otherwise
        "gravity_comp_enabled",     # 1.0 = gravity compensation active, 0.0 = zeroed (hold with vel=0 in IDLE)
        "friction_comp_enabled",    # 1.0 = friction compensation in freedrive (utility/friction_comp.py); starts 0
        # Inertia M(q)·qdd and Coriolis C(q, qd)·qd feedforward on the COMMANDED motion (utility/mode_dispatch.py
        # _add_dynamics_ff, 2026-10-10) — two separate operator switches; RTI sets their start state from system.xml.
        "inertia_comp_enabled",
        "coriolis_comp_enabled",
        # Runtime payload (utility/payload.py, set_payload): the part the gripper holds — mass [kg] and its centre
        # of mass [m] in the tool link's frame. 0 at every RTI start (not persisted). Read by status() and the GUI.
        "payload_mass",
        "payload_com_x",
        "payload_com_y",
        "payload_com_z",
        # --- Self-collision (utility/self_collision.py, self_collision_worker.py; TODO/self_collision_plan.md 2026-10-06) ---
        # Written by the collision WORKER process (another core, ~100 Hz) from the measured pose; read by RTI's safety
        # monitor and jog handler each tick. Frames: 0 = base, 1-5 = the links after joints 1-5, 6 = the tool.
        "sc_heartbeat",             # monotonic counter, +1 per worker cycle; RTI treats a stalled one as SELF_COLLISION_STALE
        "sc_hit",                   # 1.0 = two links are within the MONITOR margin on the measured pose (-> SELF_COLLISION)
        "sc_pair_a",                # the deepest flagged pair's frames (0-6), valid while sc_hit
        "sc_pair_b",
        "sc_min_gap",               # m: the flagged pair's gap (the deepest, negative = overlap) while sc_hit, else the planner margin
        # The jog's ray (robotics/jog.py, 2026-10-07): the jog handler publishes its session (rti['jog_ref'] /
        # rti['jog_dir'] arrays below, then this number); the worker answers with the free length along it.
        "jog_session",              # written by RTI's jog handler: +1 per new jog direction (a "session" = one ray)
        "sc_ray_session",           # the session the worker's answer is for (written AFTER sc_ray_free / sc_ray_done)
        "sc_ray_free",              # rad along the ray from jog_ref: free of the jog margin (a frontier, then the exact entry)
        "sc_ray_done",              # 1.0 = sc_ray_free is final (the margin's entry, or the scan reached the soft limits)
        # The worker's state for the dashboard (2026-10-07, S3f / S3g) — written by the worker only, read by the GUI and
        # api_tests; RTI never reads them.
        "sc_near_a",                # the link pair nearest its own JOG margin (frames 0-6), -1 when none within readout_mm
        "sc_near_b",
        "sc_near_gap",              # m: that pair's gap (+inf when none); refreshed every 20th worker cycle (5 Hz)
        "sc_near_margin",           # m: that pair's jog margin (where a jog stops)
        "sc_jaw_closed",            # the closing fraction (0 open .. 1 closed) the worker places the jaws at; -1 = swept
        "sc_cycle_ms",              # the worker's slowest recent cycle (ms; decays over ~1 s)
        # --- Physical confirmation for a FORCED control takeover (arbitration plan 4.4) ---
        # Industrial arms commonly require a physical action at the robot (a button on the arm, a
        # pendant's enabling device) before one session can take control from another. We have neither, so without
        # this any machine on the network can seize control from any other — which matters
        # more now that the IP allowlist can be turned off entirely.
        #
        # Only a FORCED takeover is gated. A first claim on an idle robot needs no press, and
        # should not: ownership attaches on the first owned command so a single-client setup
        # behaves exactly as it always did.
        "force_confirm_enabled",    # 1.0 = a forced takeover needs a button press. Operator
                                    #   toggle, set from the GUI. Ships 0 so behaviour does
                                    #   not change under anyone until it has run on the arm.
        "force_confirm_pending",    # 1.0 = the executor is WAITING for a press right now.
                                    #   Supervisor draws the OLED prompt while this is set;
                                    #   the executor clears it on grant, refusal or timeout.
        "force_confirm_deadline",   # time.monotonic() (system-wide on Linux; NOT time.time(),
                                    #   which NTP can step) by which the press must arrive, so the OLED
                                    #   can count down and a crashed executor's stale pending
                                    #   flag is visibly expired rather than prompting forever.
        # JOG live state — mirror of JogHandler config (see int32 block above for profile/control_mode)
        "jog_speed_pct",            # 0.0–1.0, last speed-slider value
        "jog_accel_time",           # seconds, ramp duration from 0 to full speed
        "jog_jerk_factor",          # jerk = accel * factor (S-curve only)
        # EXEC live state — mirrors ExecHandler state (RTI-process-local) so
        # command_executor (separate process) can report it in status events.
        "exec_paused",              # 1.0 = ExecHandler frozen mid-batch, 0.0 = running
        "exec_settling",            # 1.0 = final target settle check active
        "exec_completion_policy_idx", # 0=commanded, 1=settled, 2=strict
        "exec_samples_remaining",   # samples RTI still has buffered (current+next), for the
                                    # planner's rolling-buffer deadline cross-check
                                    # (TODO/blending_design.md §6). 0.0 when idle/held.
        "exec_heartbeat",           # monotonic counter, incremented ~50 Hz by a dedicated
                                    # command_executor thread. RTI's EXEC-link watchdog trips
                                    # if it stops advancing during active EXEC motion (the
                                    # executor died/wedged). See gui_executor_migration_plan.md P5.
        # --- RTI (real-time interface) mode live state ---
        # Fraction of RT ticks that received a valid UDP command while control was
        # claimed. The link-quality metric for an external control session.
        # Resets to 1.0 on every rti_claim. <0.95 warns, <0.90 warns harder.
        # Meaningless unless rti_substate == 2. See TODO/RTI_mode_plan.md §3.
        "control_success_rate",
        # RTIModeHandler sub-state as a number so the GUI/telemetry can read it:
        # 0 = disconnected (no PC paired), 1 = connected (streaming, no timing rules),
        # 2 = control_active (timing rules enforced), 3 = stopping_clean, 4 = stopping_error.
        "rti_substate",
        # Ticks since the last accepted command, and the worst such run this claim.
        #
        # Read these, not control_success_rate, when you want to know NOW whether the link
        # is healthy. The success rate is a 100-sample moving average — at 250 Hz that is a
        # deliberately damped 400 ms gauge, because a shorter window cannot carry its own
        # thresholds (a 25-sample window would read 2 dropped packets as "unreliable").
        # These two react on the very next tick instead, and are what the stop actually
        # keys off: control stops when rti_consecutive_misses exceeds the timeout
        # (10 ticks / 40 ms on PAR6). rti_worst_consecutive_misses is the session peak —
        # the number to look at after a run to see how close the link came to stopping.
        "rti_consecutive_misses",
        "rti_worst_consecutive_misses",
        # Link bandwidth in each direction, bytes/second, averaged over a 1 s window.
        # TX (robot -> PC) is exact. RX (PC -> robot) counts what the handler CONSUMES —
        # at most one command packet per tick, because drain_rti_udp keeps only the newest
        # and discards the rest. Exact in robot-clocked mode (1 command per tick by
        # definition); an undercount if a PC-clocked sender outruns the loop rate.
        # Both scale directly with the active data recipe, which is the point of watching
        # them: switching to `full` multiplies TX several times over.
        "rti_tx_bytes_per_s",
        "rti_rx_bytes_per_s",
        # Last RTI command sequence number the robot accepted. Published here so the
        # telemetry path can pick it up without reaching into the handler, and so the
        # `rti_seq` recipe tag can echo it to the PC — which is what makes round-trip
        # latency measurable at all (the PC times its own send against seeing it come
        # back). float64 like everything in this block; a uint32 seq is exactly
        # representable well past its 199-day wrap at 250 Hz.
        "rti_seq_rx_last",
        # Percentage of received command packets thrown away in the last window. Only the
        # NEWEST setpoint per tick can be applied, so a PC sending faster than the loop
        # ticks loses the rest — and no other metric shows it: success rate reads 100%
        # (a command did arrive) and consecutive misses read 0. A PC-clocked controller
        # at 500 Hz against a 250 Hz loop reads ~50% here and 100% everywhere else.
        "rti_discard_pct",
    ]:
        shm, arr = _attach_or_create(name, np.float64, 1)  # only 1 entry
        rti_shms[name] = shm
        rti[name] = arr

    # Self-collision per link pair (2026-10-07, the escape rule): sc_pair_gaps[a * 7 + b] = that pair's gap (m,
    # negative = overlap) while it is within the MONITOR margin on the measured pose, +inf otherwise. Written by the
    # collision worker; RTI's monitor keeps what was already within the margin when an active mode began and
    # stops only if a pair gets closer than that, or another pair comes within the margin.
    # The control loop's delay per joint (2026-10-12, utility/loop_delay_meter.py): a reply's arrival -> the departure of
    # the command computed from it [s], median / max of the last ~5 s; NaN until measured. can_read_first: 1 while
    # RTI runs option B's tick order (read first, send early), 0 in the normal order.
    # The Hall sensor check (robotics/hall_check.py, 2026-10-13): hall_check_joint = the limp joint's number (1-based)
    # while the check runs, 0 = off; hall_check_pin = its Hall pin (0 / 1), -1 = not read yet; hall_check_changes = pin
    # changes seen; hall_check_edge_deg = the joint's angle (deg) at the last change to 0 / to 1 (NaN = none yet).
    for name, count in [("sc_pair_gaps", 49), ("jog_ref", joint_num), ("jog_dir", joint_num),
                        ("loop_delay", joint_num), ("loop_delay_max", joint_num), ("can_read_first", 1),
                        ("hall_check_joint", 1), ("hall_check_pin", 1), ("hall_check_changes", 1),
                        ("hall_check_edge_deg", 2)]:
        shm, arr = _attach_or_create(name, np.float64, count)
        rti_shms[name] = shm
        rti[name] = arr

    # ========================= Gripper Data =========================
    gripper_shms = {}
    gripper = {}

    # --- Raw / status fields (int32) ---
    for name in [
        "gripper_position",          # When in gripper mode
        "gripper_current",           # When in gripper mode
        "gripper_activated",         # When in gripper mode
        "gripper_action",            # When in gripper mode
        "gripper_object_detection",  # When in gripper mode
        "gripper_calibrated",        # When in gripper mode
        "gripper_temperature_error", # When in gripper mode
        "gripper_timeout_error",     # When in gripper mode
        "gripper_estop_error",       # When in gripper mode
        "gripper_error",             # When in gripper mode & motor mode
        "gripper_motor_position",    # When in motor mode
        "gripper_motor_speed",       # When in motor mode
        "gripper_motor_current",     # When in motor mode
        # The gripper motor's LIVE aggregate fault bit, refreshed every tick off its DATA_PACK
        # reply — the gripper's equivalent of the arm's global_motor_error. check_errors trusts
        # it over the slow per-type flags below (see error_checks.py).
        "gripper_motor_global_error",
        # --- Gripper motor health (polled via round-robin, same as joint motors) ---
        "gripper_motor_temperature",
        "gripper_motor_voltage",
        "gripper_motor_hw_version",
        "gripper_motor_sw_version",
        "gripper_motor_serial",
        "gripper_motor_temp_error",
        "gripper_motor_enc_error",
        "gripper_motor_vbus_error",
        "gripper_motor_driver_error",
        "gripper_motor_velocity_error",
        "gripper_motor_current_error",
        "gripper_motor_estop_error",
        "gripper_motor_watchdog_error",
        "gripper_motor_calibrated",
        "gripper_homed",                 # 0=not homed, 1=homed
    ]:
        shm, arr = _attach_or_create(name, np.int32, 1)
        gripper_shms[name] = shm
        gripper[name] = arr

    # --- Gripper motor Kt and conversion constants (float64) ---
    for name in [
        "gripper_motor_kt",
        "gripper_motor_kt_source",       # 1 = Kt read from the gripper over CAN, 0 = XML fallback
        "gripper_homed_endstop_ticks",   # Encoder ticks at endstop captured during homing
        "gripper_ticks_per_meter",       # Conversion factor: encoder ticks per meter of stroke
        "gripper_gear_r",                # Gear pitch radius (m), loaded from XML
        "gripper_stroke_m",              # Full stroke length (m), loaded from XML
    ]:
        shm, arr = _attach_or_create(name, np.float64, 1)
        gripper_shms[name] = shm
        gripper[name] = arr

    # --- Computed joint-level fields (float64, SI units) ---
    for name in [
        "gripper_joint_position",            # When in motor mode (Unit: m)
        "gripper_joint_speed",               # When in motor mode (Unit: m/s) — RAW
        "gripper_joint_speed_filtered",      # filtered (gripper_speed in config/filters.xml)
        "gripper_joint_force",               # When in motor mode (Unit: N)
        "gripper_joint_position_target",     # When in motor mode (Unit: m)
        "gripper_joint_speed_target",        # When in motor mode (Unit: m/s)
        "gripper_joint_force_target",        # When in motor mode (Unit: N)
        "gripper_joint_position_commanded",  # When in motor mode (Unit: m)
        "gripper_joint_speed_commanded",     # When in motor mode (Unit: m/s)
        "gripper_joint_force_commanded",     # When in motor mode (Unit: N)
    ]:
        shm, arr = _attach_or_create(name, np.float64, 1)
        gripper_shms[name] = shm
        gripper[name] = arr

    _initialized = True
    print("All shared memory elements are attached or created successfully.\n")


def cleanup():
    """Close all shared memory handles gracefully. Safe to call multiple times."""
    global _initialized
    if not _initialized:
        return
    _initialized = False  # prevent double-cleanup

    # SharedString globals
    for ref in [robot_mode, robot_state, error_state, homed_state, sim_mode, robot_command]:
        if ref is not None:
            try: ref.close()
            except Exception: pass

    # SharedString system-info dict
    for ref in system_info_shms.values():
        try: ref.close()
        except Exception: pass

    # numpy shared memory dicts
    for shm_dict in [motor_shms, motor_config_shms, joint_shms, tcp_shms, rti_shms, gripper_shms, control_board_shms]:
        if shm_dict:
            for shm in shm_dict.values():
                try: shm.close()
                except Exception: pass

    # single shared memory handles
    for ref in [cb_command_, cb_status, cb_current_error, active_recipe, recipe_reload_signal,
                filter_reload_signal, clear_error_signal, loop_stats_reset_signal]:
        if ref is not None:
            try: ref.close()
            except Exception: pass

    for shm in [shm_GPIO]:
        if shm is not None:
            try: shm.close()
            except Exception: pass

    print("[shared_data] Shared memory handles closed.")


# Allow RTI(7) shortcut
__call__ = setup


# Auto-initialize with 6 joints when imported (so old code works)
setup(6)


# ======================= Command-line support (optional) =======================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="RCB Runtime — RTI shared memory setup")
    parser.add_argument("-j", "--joints", type=int, default=6, help="Number of joints")
    args = parser.parse_args()
    setup(args.joints, force_reinit=True)
    print(f"RTI ready – configured for {joint_num} joints")
