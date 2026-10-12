"""Write computed motor, joint, and TCP state into shared memory arrays.

This module is the ONLY writer of MEASURED ("actual") state — the no-suffix channels.
The _commanded / _target channels are written by utility/mode_dispatch.py instead; see
docs/commanded_target_actual.md.

⚠️ CALL ORDER IS LOAD-BEARING. The two functions below form a pipeline and must be
called in this order, every tick:

    1. update_motor_and_gripper_data()   CAN deques -> motor[] + gripper[]
    2. update_joint_and_tcp_data()       motor[]    -> joint[] + tcp[]

update_joint_and_tcp_data() reads shared_data.motor['motor_position'/'motor_speed'/
'motor_current'] — the exact arrays update_motor_and_gripper_data() writes. Call them
the other way round and joint/TCP state is silently built from the PREVIOUS tick's motor
data: one tick (4 ms on PAR6) of skew, with no exception and no log to point at it.

Nothing enforces this, so it is written down here instead. RTI.py's superloop is the
only caller and has them in the right order (motor at ~1235, joint/TCP at ~1254).

A third step closes the loop, and its position matters too: process_joint_data() appends
this tick's joint values to Joint_data AFTER both functions run (RTI.py ~1276). That is
what makes Joint_data[-1] the PREVIOUS tick inside update_joint_and_tcp_data(), which the
acceleration/jerk finite differences depend on. Append earlier and both silently read 0.
"""

# robot/data_sync.py
import numpy as np
from robotics.dynamics import motor_current_to_joint_torque, torque_to_motor_current, estimate_external_force_tcp
from robotics.dynamics import gravity_torque_rt
from robotics.robot_kinematics import calculate_all_kinematics
from utility.kinematics_backend import write_quaternion_wxyz
from utility.signal_filters import apply_filter_bank

# Reused finite-difference scratch (the intermediate d(filtered)/dt before it goes through a
# filter bank). Preallocated once per joint_num — the RT loop is single-threaded, so one
# module-level buffer is safe and keeps the hot path allocation-free.
_filt_scratch = {}


def _scratch(n):
    a = _filt_scratch.get(n)
    if a is None:
        a = np.zeros(n, dtype=np.float64)
        _filt_scratch[n] = a
    return a


def update_joint_and_tcp_data(
    *,
    shared_data,
    Joint,
    Joint_data,
    filter_banks,
    joint_num,
    delta_t,
    gear_ratio,
    gear_efficiency,
    motor_kt,
    motor_signs,
    robot,
    jacobian_out=None,
):
    """
    Update joint-level and TCP-level shared memory.

    jacobian_out: optional (6, joint_num) buffer that receives this tick's TCP Jacobian [J_v; J_w] at the
    measured pose (the haptic walls need it; it is not in shared memory).

    Computes joint positions/speeds/accel/jerk/torque,
    runs FK, gravity compensation, and external force estimation,
    then writes all results to shared_data.

    ⚠️ MUST be called AFTER update_motor_and_gripper_data() on the same tick — it reads the
    shared_data.motor[] arrays that function writes. See the module docstring.

    Two halves since 2026-10-12 (option B "send early", TODO/can_tx_timing_plan.md §9.10): update_joint_state() —
    what a command needs: position, speed (raw + filtered), gravity — and update_joint_rest() — everything else. This
    function calls them in that order: the same values as the single function before (gravity moved ahead of the
    derivatives — it reads only the position; the external torque now reads it back from shared memory).

    Args:
        shared_data:          shared memory module
        Joint:                list of Joint objects
        Joint_data:           deque of historical joint dicts (supplies the t-1 sample for
                              every finite difference — raw AND filtered)
        filter_banks:         {"speed"/"accel"/"jerk"/"torque": bank} from utility.filter_config.
                              Each bank holds one independent per-joint filter's state; created
                              once at boot / rebuilt on hot reload.
                              (banks include 'tcp_accel', a 6-channel bank for the Cartesian
                              acceleration — it replaces the old hardcoded TCP-accel EMA)
        joint_num:            number of joints
        delta_t:              RTI loop period (s)
        gear_ratio, gear_efficiency, motor_kt, motor_signs: drive-train constants
        robot:                robot model (PAR6, INDIGO, etc.)
    """
    update_joint_state(shared_data=shared_data, Joint=Joint, filter_banks=filter_banks, joint_num=joint_num,
                       gear_ratio=gear_ratio, gear_efficiency=gear_efficiency, motor_kt=motor_kt,
                       motor_signs=motor_signs, robot=robot)
    update_joint_rest(shared_data=shared_data, Joint_data=Joint_data, filter_banks=filter_banks,
                      joint_num=joint_num, delta_t=delta_t, gear_ratio=gear_ratio, gear_efficiency=gear_efficiency,
                      motor_kt=motor_kt, motor_signs=motor_signs, robot=robot, jacobian_out=jacobian_out)


def update_joint_state(*, shared_data, Joint, filter_banks, joint_num, gear_ratio, gear_efficiency, motor_kt,
                       motor_signs, robot):
    """What a command needs, and nothing else (2026-10-12): the joint position, the speed (raw and filtered) and the
    gravity torque (+ its motor current). Option B computes the tick's commands right after this, BEFORE
    update_joint_rest(); every measured value a mode handler or dispatch reads must be produced here (pinned by
    dummy_test_scripts/test_can_read_first.py). update_joint_and_tcp_data() calls it first."""
    # --- Joint position (encoder ticks → radians) ---
    shared_data.joint['joint_position'][:] = [
        Joint[i].get_joint_position(shared_data.motor['motor_position'][i])
        for i in range(joint_num)
    ]

    # --- Joint speed: RAW from CAN, then its filtered twin ---
    # joint_speed is now the RAW measured speed (it was the filtered value before the
    # 2026-07-17 split). The filter bank produces joint_speed_filtered.
    js = shared_data.joint['joint_speed']
    for i in range(joint_num):
        js[i] = Joint[i].get_joint_speed(shared_data.motor['motor_speed'][i])
    apply_filter_bank(filter_banks['speed'], js, shared_data.joint['joint_speed_filtered'])

    # --- Gravity/dynamics compensation ---
    grav_torque = gravity_torque_rt(robot, shared_data.joint['joint_position'][:])
    shared_data.joint['joint_gravity_torque'][:] = grav_torque  # Nm — joint space, always updated
    if shared_data.rti['gravity_comp_enabled'][0]:
        shared_data.joint['compensation_torque'][:] = torque_to_motor_current(
            grav_torque, gear_ratio, gear_efficiency, motor_kt, motor_signs
        )
    else:
        shared_data.joint['compensation_torque'][:] = 0


def update_tcp_pose(*, shared_data, robot, jacobian_out=None):
    """The TCP pose, the Jacobian and the RAW TCP twist at the measured position (2026-10-12) — for the haptic walls
    in freedrive when option B computes the commands before update_joint_rest(). A position-only kinematics call (no
    diagnostics); update_joint_rest() then computes and writes the full set as before."""
    kin = calculate_all_kinematics(robot, shared_data.joint['joint_position'], include_diagnostics=False,
                                   return_transform=False)
    J = kin['J']
    if jacobian_out is not None:
        jacobian_out[:] = J
    shared_data.tcp['robot_pose'][:] = kin['tcp_pose']
    shared_data.tcp['robot_pose_matrix'][:] = kin['tcp_pose_matrix']
    shared_data.tcp['TCP_speed'][:] = J @ shared_data.joint['joint_speed']


def update_joint_rest(*, shared_data, Joint_data, filter_banks, joint_num, delta_t, gear_ratio, gear_efficiency,
                      motor_kt, motor_signs, robot, jacobian_out=None):
    """Everything update_joint_state() leaves out (2026-10-12): acceleration and jerk (raw + filtered), torque (raw +
    filtered), the kinematics (pose, Jacobian, TCP speed / acceleration, quaternion, diagnostics), the external torque
    and force. Must run AFTER update_joint_state() on the same tick and BEFORE this tick's Joint_data append."""
    js = shared_data.joint['joint_speed']
    jsf = shared_data.joint['joint_speed_filtered']

    # --- Acceleration & jerk: RAW from the raw chain, filtered from the filtered chain ---
    # Every finite difference reads the PREVIOUS tick from Joint_data (appended after this
    # function, so [-1] is t-1). The deque carries both raw (j_s, j_a) and filtered
    # (j_s_filtered, j_a_filtered), so each derivative differences like with like. The filter
    # banks supply the smoothing state; the deque supplies the 1-sample history — separate
    # concerns. A signal needs as many poles as differentiations, so the accel bank is a
    # single stage (default 'none' — its filtered-speed input already carries one pole) and
    # the jerk bank must be 2-pole.
    # Whole-array ufuncs with out= (2026-10-03): the same (a - b) / dt per element as the per-joint loops
    # they replace — bit-identical — at ~1/4 of the cost (the loops were ~50 us of a 316 us function).
    prev = Joint_data[-1]
    ja = shared_data.joint['joint_acceleration']
    np.subtract(js, prev['j_s'], out=ja)                                # RAW
    np.divide(ja, delta_t, out=ja)
    scratch = _scratch(joint_num)
    np.subtract(jsf, prev['j_s_f'], out=scratch)
    np.divide(scratch, delta_t, out=scratch)
    apply_filter_bank(filter_banks['accel'], scratch, shared_data.joint['joint_acceleration_filtered'])
    jaf = shared_data.joint['joint_acceleration_filtered']

    jj = shared_data.joint['joint_jerk']
    np.subtract(ja, prev['j_a'], out=jj)                                # RAW
    np.divide(jj, delta_t, out=jj)
    np.subtract(jaf, prev['j_a_f'], out=scratch)
    np.divide(scratch, delta_t, out=scratch)
    apply_filter_bank(filter_banks['jerk'], scratch, shared_data.joint['joint_jerk_filtered'])

    # --- Joint torque from motor current: RAW, then filtered ---
    # NOTE: torque = current × Kt × gear_ratio. motor_current is in mA.
    shared_data.joint['joint_torque'][:] = motor_current_to_joint_torque(
        shared_data.motor['motor_current'][:], gear_ratio, gear_efficiency, motor_kt, motor_signs
    )
    apply_filter_bank(filter_banks['torque'],
                      shared_data.joint['joint_torque'],
                      shared_data.joint['joint_torque_filtered'])

    # --- FK, Jacobian, TCP velocity/acceleration ---
    # The kinematics call carries the FILTERED joint signals: J·q̇ and J·q̈+J̇·q̇ from the clean
    # inputs give the FILTERED Cartesian twist/acceleration. J itself depends only on joint
    # POSITION, so it is reused below to map the RAW joint speed into the raw Cartesian twist.
    kin = calculate_all_kinematics(
        robot,
        shared_data.joint['joint_position'],
        shared_data.joint['joint_speed_filtered'],
        joint_acceleration=shared_data.joint['joint_acceleration_filtered'],
    )
    J = kin['J']
    if jacobian_out is not None:
        jacobian_out[:] = J
    shared_data.tcp['robot_pose'][:]        = kin['tcp_pose']
    shared_data.tcp['robot_pose_matrix'][:] = kin['tcp_pose_matrix']

    # TCP twist: filtered = J·q̇_filtered (from the kin call); raw = J·q̇_raw (same J, raw speed).
    shared_data.tcp['TCP_speed_filtered'][:] = kin['tcp_speed']
    shared_data.tcp['TCP_speed'][:]          = J @ shared_data.joint['joint_speed']

    # Quaternion (w, x, y, z — scalar first; see write_quaternion_wxyz). Derived from the
    # matrix we just stored rather than recomputed: reshape(4, 4) is a view, and the write
    # lands straight in shared memory, so this adds no allocation to the hot path.
    # Unlike robot_pose's RPY it has no gimbal lock — use it for orientation error/SLERP.
    write_quaternion_wxyz(
        shared_data.tcp['robot_pose_matrix'].reshape(4, 4),
        shared_data.tcp['robot_pose_quaternion'],
    )

    # TCP acceleration.
    #   filtered = the analytic Cartesian accel from FILTERED joint inputs (J·q̈_f + J̇·q̇_f,
    #     returned by the kin call) through the 'tcp_accel' filter — its own low-pass because the
    #     J̇·q̇ term is quadratic in velocity and stays noisy even on filtered joint inputs.
    #   raw = d/dt of the RAW Cartesian twist (numeric). This equals J·q̈_raw + J̇·q̇_raw without
    #     recomputing J̇ for the raw speed; noisy by nature (a raw diagnostic), like raw joint jerk.
    apply_filter_bank(filter_banks['tcp_accel'], kin['tcp_acceleration'],
                      shared_data.tcp['TCP_acceleration_filtered'])
    shared_data.tcp['TCP_acceleration_raw'][:] = (
        shared_data.tcp['TCP_speed'] - prev['TCP_s']
    ) / delta_t

    shared_data.tcp['jacobian_sigma_min'][0] = kin['jacobian_sigma_min']
    shared_data.tcp['jacobian_condition'][0] = kin['condition_number']
    shared_data.tcp['jacobian_manipulability'][0] = kin['manipulability']

    # --- External torque and force estimation ---
    # Uses FILTERED torque: raw motor current is noisy, and torque_external / force_external
    # feed collision/interaction estimates where that noise matters most. (This is the former
    # "consider low-pass filtering torque_external" TODO — now done via the torque filter bank.)
    # The gravity torque is update_joint_state()'s, read back from shared memory (the same values).
    shared_data.tcp['torque_external'][:] = (shared_data.joint['joint_torque_filtered'][:]
                                             - shared_data.joint['joint_gravity_torque'][:])
    f_ext_world = estimate_external_force_tcp(shared_data.tcp['torque_external'], J)
    shared_data.tcp['force_external'][:] = f_ext_world

    # Same wrench in the TOOL frame: rotate the force and moment 3-vectors by Rᵀ (R is the TCP
    # orientation, top-left 3x3 of the pose matrix). Pure rotation, no moment-arm term — it is
    # the same reference point (the TCP), only different axes. World frame suits absolute sensing
    # (collision, payload weight); tool frame suits task-relative force (insertion along the tool
    # axis, contact normal to a surface).
    R = shared_data.tcp['robot_pose_matrix'].reshape(4, 4)[:3, :3]
    ft = shared_data.tcp['force_external_tool']
    ft[:3] = R.T @ f_ext_world[:3]
    ft[3:] = R.T @ f_ext_world[3:]


def update_motor_and_gripper_data(
    *,
    shared_data,
    Motor_data,
    Motor_temperature_data,
    Motor_voltage_data,
    Motor_info_data,
    Motor_errors_data,
    Motor_Kt_data,
    Gripper_data,
    motor_error_bytes,
    joint_num: int,
    active_CAN_gripper: int,
    gripper_kt: float = 0.111,
    gripper_speed_bank=None,
):
    """Update shared memory with latest motor and gripper data.

    ⚠️ MUST be called BEFORE update_joint_and_tcp_data() on the same tick — that function
    consumes the shared_data.motor[] arrays written here. See the module docstring.

    Note the staggered cadence below: position/speed/current/global_motor_error refresh
    every tick, but the per-type error flags are on tick%5 and device info on tick%500,
    so a flag read here can be up to ~20 ms old.
    """

    tick = shared_data.rti["loop_tick"][0]

    # === Always update ===
    # One pass over the joints writing in place (2026-10-03) — the four list comprehensions this replaces built
    # four lists per tick; the values written are the same. (Written as explicit shared_data.motor[...] subscripts
    # on purpose: test_motor_array_writers recognises the ONE legitimate writer of these arrays by that text.)
    for i in range(joint_num):
        latest = Motor_data[i][-1]
        shared_data.motor['motor_position'][i] = latest["position"]
        shared_data.motor['motor_speed'][i] = latest["speed"]
        shared_data.motor['motor_current'][i] = latest["current"]
        shared_data.motor['global_motor_error'][i] = latest["error"]

    # === 5-tick staggered updates ===
    if tick % 5 == 0:
        shared_data.motor['motor_temperature'][:] = [
            Motor_temperature_data[i][-1]["temp"] for i in range(joint_num)
        ]
        shared_data.motor['motor_voltage'][:] = [
            Motor_voltage_data[i][-1]["volt"] for i in range(joint_num)
        ]

    # === 500-tick staggered updates (static motor info - rarely changes) ===
    elif tick % 500 == 3:
        shared_data.motor['motor_Kt'][:] = [
            Motor_Kt_data[i][-1]["Kt"] for i in range(joint_num)
        ]
        shared_data.motor['HARDWARE_VERSION'][:] = [
            Motor_info_data[i][-1]["HARDWARE_VERSION"] for i in range(joint_num)
        ]
        shared_data.motor['BATCH_DATE'][:] = [
            Motor_info_data[i][-1]["BATCH_DATE"] for i in range(joint_num)
        ]
        shared_data.motor['SOFTWARE_VERSION'][:] = [
            Motor_info_data[i][-1]["SOFTWARE_VERSION"] for i in range(joint_num)
        ]
        shared_data.motor['SERIAL_NUMBER'][:] = [
            Motor_info_data[i][-1]["SERIAL_NUMBER"] for i in range(joint_num)
        ]

    elif tick % 5 == 1:
        shared_data.motor['motor_temperature_error'][:] = [
            Motor_errors_data[i][-1]["temperature_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_encoder_error'][:] = [
            Motor_errors_data[i][-1]["encoder_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_vbus_error'][:] = [
            Motor_errors_data[i][-1]["vbus_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_driver_error'][:] = [
            Motor_errors_data[i][-1]["driver_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_velocity_error'][:] = [
            Motor_errors_data[i][-1]["velocity_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_current_error'][:] = [
            Motor_errors_data[i][-1]["current_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_estop_error'][:] = [
            Motor_errors_data[i][-1]["estop_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_watchdog_error'][:] = [
            Motor_errors_data[i][-1]["watchdog_error"] for i in range(joint_num)
        ]
        shared_data.motor['motor_calibrated'][:] = [
            Motor_errors_data[i][-1]["calibrated"] for i in range(joint_num)
        ]

    elif tick % 20 == 2:
        for i in range(joint_num):
            motor_error_bytes[i] = (
                (shared_data.motor['motor_temperature_error'][i] & 1) << 0 |
                (shared_data.motor['motor_encoder_error'][i] & 1) << 1 |
                (shared_data.motor['motor_vbus_error'][i] & 1) << 2 |
                (shared_data.motor['motor_driver_error'][i] & 1) << 3 |
                (shared_data.motor['motor_velocity_error'][i] & 1) << 4 |
                (shared_data.motor['motor_current_error'][i] & 1) << 5 |
                (shared_data.motor['motor_estop_error'][i] & 1) << 6 |
                (shared_data.motor['motor_watchdog_error'][i] & 1) << 7
            )

    # === Gripper ===
    if active_CAN_gripper == 1:
        g = Gripper_data[joint_num][-1]
        m = Motor_data[joint_num][-1]

        shared_data.gripper['gripper_position'][:] = g["g_p"]
        shared_data.gripper['gripper_current'][:] = g["g_c"]
        shared_data.gripper['gripper_activated'][:] = g["g_activated"]
        shared_data.gripper['gripper_action'][:] = g["g_action"]
        shared_data.gripper['gripper_object_detection'][:] = g["g_obj"]
        shared_data.gripper['gripper_calibrated'][:] = g["g_cal"]
        shared_data.gripper['gripper_temperature_error'][:] = g["g_temp_err"]
        shared_data.gripper['gripper_timeout_error'][:] = g["g_tim_err"]
        shared_data.gripper['gripper_error'][:] = g["g_err"]
        shared_data.gripper['gripper_estop_error'][:] = g["g_estop_err"]

        shared_data.gripper['gripper_motor_position'][:] = m["position"]
        shared_data.gripper['gripper_motor_speed'][:] = m["speed"]
        shared_data.gripper['gripper_motor_current'][:] = m["current"]

        # --- Gripper motor health (polled fields, same cadence as joint motors) ---
        gm_idx = joint_num  # gripper is the last motor in all data arrays
        if tick % 5 == 0:
            shared_data.gripper['gripper_motor_temperature'][0] = Motor_temperature_data[gm_idx][-1]["temp"]
            shared_data.gripper['gripper_motor_voltage'][0]     = Motor_voltage_data[gm_idx][-1]["volt"]
        elif tick % 5 == 1:
            e = Motor_errors_data[gm_idx][-1]
            shared_data.gripper['gripper_motor_temp_error'][0]     = e["temperature_error"]
            shared_data.gripper['gripper_motor_enc_error'][0]      = e["encoder_error"]
            shared_data.gripper['gripper_motor_vbus_error'][0]     = e["vbus_error"]
            shared_data.gripper['gripper_motor_driver_error'][0]   = e["driver_error"]
            shared_data.gripper['gripper_motor_velocity_error'][0] = e["velocity_error"]
            shared_data.gripper['gripper_motor_current_error'][0]  = e["current_error"]
            shared_data.gripper['gripper_motor_estop_error'][0]    = e["estop_error"]
            shared_data.gripper['gripper_motor_watchdog_error'][0] = e["watchdog_error"]
            shared_data.gripper['gripper_motor_calibrated'][0]     = e["calibrated"]
        elif tick % 500 == 3:
            shared_data.gripper['gripper_motor_kt'][0]         = Motor_Kt_data[gm_idx][-1]["Kt"]
            shared_data.gripper['gripper_motor_hw_version'][0] = Motor_info_data[gm_idx][-1]["HARDWARE_VERSION"]
            shared_data.gripper['gripper_motor_sw_version'][0] = Motor_info_data[gm_idx][-1]["SOFTWARE_VERSION"]
            shared_data.gripper['gripper_motor_serial'][0]     = Motor_info_data[gm_idx][-1]["SERIAL_NUMBER"]

  
        # --- Gripper SI unit conversions ---
        ticks_per_m = shared_data.gripper['gripper_ticks_per_meter'][0]
        gear_r = shared_data.gripper['gripper_gear_r'][0]
        kt = shared_data.gripper['gripper_motor_kt'][0]
        endstop_ticks = shared_data.gripper['gripper_homed_endstop_ticks'][0]
        stroke_m = shared_data.gripper['gripper_stroke_m'][0]

        # Position and speed require homing (ticks_per_meter must be set).
        #
        # THEY ALSO REQUIRE MOTOR MODE. m["position"] is the gripper motor's
        # ENCODER, which the driver only reports in motor mode (ctrl_mode 1). In
        # firmware mode (ctrl_mode 2, the default) it stays 0, so this computes
        # stroke_m - endstop_ticks/ticks_per_m — a fixed negative number that
        # looks like a real reading. Measured 2026-08-06: -50.96 mm on a 106 mm
        # stroke with the jaws fully CLOSED.
        #
        # Firmware mode reports gripper_position on the raw 0-255 scale instead,
        # and that is what the queued gripper commands use. TODO: gate these SI
        # fields on ctrl_mode (or publish NaN in firmware mode) so a caller
        # cannot mistake the constant for a measurement — and see
        # TODO/future_todo.md for the raw-to-mm mapping that would give firmware
        # mode real units too.
        if ticks_per_m > 0:
            # Position referenced from endstop: endstop = stroke_m (fully open), 0 = fully closed
            pos_m = stroke_m + (m["position"] - endstop_ticks) / ticks_per_m
            spd_mps = m["speed"] / ticks_per_m
        else:
            pos_m = 0.0
            spd_mps = 0.0

        # Force: F = (I * Kt) / Gear_r — does not require homing
        if gear_r > 0 and kt > 0:
            force_n = (m["current"] / 1000.0) * kt / gear_r
        else:
            force_n = 0.0

        # Write to shared memory
        shared_data.gripper['gripper_joint_position'][:] = pos_m
        shared_data.gripper['gripper_joint_speed'][:] = spd_mps          # RAW
        shared_data.gripper['gripper_joint_force'][:] = force_n
        if gripper_speed_bank is not None:
            apply_filter_bank(gripper_speed_bank,
                              shared_data.gripper['gripper_joint_speed'],
                              shared_data.gripper['gripper_joint_speed_filtered'])
