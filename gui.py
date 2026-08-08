"""NiceGUI operator interface for monitoring and controlling the robot."""

from nicegui import ui
from data import shared_data
import math
import psutil
import os 

shared_data.setup(joints=6)

JOINTS = 6


from utility.process_utils import setup_process
setup_process(core=1, rt=False)   # GUI: own core, normal scheduling (only RTI is RT)

# --- helper to format floats safely ---
def fmt(x):
    """Format numbers as fixed-width strings with 3 decimals."""
    if not math.isfinite(x):
        return "   ---   "
    return f"{x:11.3f}"


# ----------------- Jog Button Handlers -----------------
def handle_jog_click():
    print("JOG pressed")

def handle_cart_jog_click():
    print("CART JOG pressed")

# ----------------- UI Buttons -----------------
with ui.row().classes("items-center gap-2"):
    with ui.dropdown_button('Jogts', icon='settings', split=True) as jog_dropdown:
        jog_dropdown.on('click', handle_jog_click)
        with ui.row().classes('p-4 items-center'):
            ui.icon('volume_up', size='sm')
            ui.switch().props('color=negative')
            ui.separator().props('vertical')
            ui.icon('mic', size='sm')
            ui.switch().props('color=negative')

    with ui.dropdown_button('CART JOG', icon='settings', split=True) as cart_dropdown:
        cart_dropdown.on('click', handle_cart_jog_click)
        with ui.row().classes('p-4 items-center'):
            ui.switch('TRF').props('color=positive')
            ui.switch('WRF').props('color=positive')

# =====================================================
# ===================== COLUMN DEFS ===================
# =====================================================

motor_columns = [
    {'name': 'joint', 'label': 'Joint', 'field': 'joint'},
    {'name': 'pos', 'label': 'Position', 'field': 'pos'},
    {'name': 'spd', 'label': 'Speed', 'field': 'spd'},
    {'name': 'cur', 'label': 'Current', 'field': 'cur'},
    {'name': 'err', 'label': 'Error', 'field': 'err'},
    {'name': 'HARDWARE_VERSION', 'label': 'HARDWARE_VERSION', 'field': 'HARDWARE_VERSION'},
    {'name': 'BATCH_DATE', 'label': 'BATCH_DATE', 'field': 'BATCH_DATE'},
    {'name': 'SOFTWARE_VERSION', 'label': 'SOFTWARE_VERSION', 'field': 'SOFTWARE_VERSION'},
    {'name': 'SERIAL_NUMBER', 'label': 'SERIAL_NUMBER', 'field': 'SERIAL_NUMBER'},
]

info_columns = [
    {'name': 'joint', 'label': 'Joint', 'field': 'joint'},
    {'name': 'temp', 'label': 'Temp', 'field': 'temp'},
    {'name': 'volt', 'label': 'Volt', 'field': 'volt'},
    {'name': 'kt', 'label': 'Kt (Nm/A)', 'field': 'kt'},
]

error_columns = [
    {'name': 'joint', 'label': 'Joint', 'field': 'joint'},
    {'name': 'temp', 'label': 'Temp Err', 'field': 'temp'},
    {'name': 'enc', 'label': 'Enc Err', 'field': 'enc'},
    {'name': 'vbus', 'label': 'VBUS Err', 'field': 'vbus'},
    {'name': 'drv', 'label': 'Driver Err', 'field': 'drv'},
    {'name': 'vel', 'label': 'Vel Err', 'field': 'vel'},
    {'name': 'cur', 'label': 'Cur Err', 'field': 'cur'},
    {'name': 'estop', 'label': 'Estop Err', 'field': 'estop'},
    {'name': 'wd', 'label': 'Watchdog Err', 'field': 'wd'},
    {'name': 'cal', 'label': 'Cal', 'field': 'cal'},
]

gripper_columns = [
    {'name': 'pos', 'label': 'Pos', 'field': 'pos'},
    {'name': 'cur', 'label': 'Cur', 'field': 'cur'},
    {'name': 'act', 'label': 'Activated', 'field': 'act'},
    {'name': 'gripper_action', 'label': 'gripper_action', 'field': 'gripper_action'},
    {'name': 'obj', 'label': 'Object', 'field': 'obj'},
    {'name': 'cal', 'label': 'Cal', 'field': 'cal'},
    {'name': 'gripper_temperature_error', 'label': 'gripper_temperature_error', 'field': 'gripper_temperature_error'},
    {'name': 'gripper_timeout_error', 'label': 'gripper_timeout_error', 'field': 'gripper_timeout_error'},
    {'name': 'gripper_error', 'label': 'gripper_error', 'field': 'gripper_error'},
    {'name': 'gripper_estop_error', 'label': 'gripper_estop_error', 'field': 'gripper_estop_error'},
    {'name': 'gripper_motor_position', 'label': 'gripper_motor_position', 'field': 'gripper_motor_position'},
    {'name': 'gripper_motor_speed', 'label': 'gripper_motor_speed', 'field': 'gripper_motor_speed'},
    {'name': 'gripper_motor_current', 'label': 'gripper_motor_current', 'field': 'gripper_motor_current'},

]

joint_columns = [
    {'name': 'joint', 'label': 'Joint', 'field': 'joint'},
    {'name': 'joint_position', 'label': 'joint_position', 'field': 'joint_position'},
    {'name': 'joint_speed', 'label': 'joint_speed', 'field': 'joint_speed'},
    {'name': 'joint_torque', 'label': 'joint_torque', 'field': 'joint_torque'},
    {'name': 'joint_acceleration', 'label': 'joint_acceleration', 'field': 'joint_acceleration'},
    {'name': 'joint_jerk', 'label': 'joint_jerk', 'field': 'joint_jerk'},
    {'name': 'joint_position_commanded', 'label': 'joint_position_commanded', 'field': 'joint_position_commanded'},
    {'name': 'joint_speed_commanded', 'label': 'joint_speed_commanded', 'field': 'joint_speed_commanded'},
    {'name': 'joint_acceleration_commanded', 'label': 'joint_acceleration_commanded', 'field': 'joint_acceleration_commanded'},
    {'name': 'joint_torque_commanded', 'label': 'joint_torque_commanded', 'field': 'joint_torque_commanded'},
    {'name': 'torque_external', 'label': 'torque_external', 'field': 'torque_external'},

]

TCP_columns = [
    {'name': 'axis', 'label': 'axis', 'field': 'axis'},
    {'name': 'robot_pose', 'label': 'robot_pose', 'field': 'robot_pose'},
    {'name': 'TCP_speed', 'label': 'TCP_speed', 'field': 'TCP_speed'},
    {'name': 'force_external', 'label': 'force_external', 'field': 'force_external'},
    {'name': 'TCP_pose_commanded', 'label': 'TCP_pose_commanded', 'field': 'TCP_pose_commanded'},
    {'name': 'TCP_speed_commanded', 'label': 'TCP_speed_commanded', 'field': 'TCP_speed_commanded'},

]

rti_columns = [
    {'name': 'node', 'label': 'Node', 'field': 'node'},
    {'name': 'scan_received_ids', 'label': 'scan_received_ids', 'field': 'scan_received_ids'},
    {'name': 'Error_counter', 'label': 'Error_counter', 'field': 'Error_counter'},
    {'name': 'old_data_warning', 'label': 'old_data_warning', 'field': 'old_data_warning'},
    {'name': 'CAN_node_connection_error', 'label': 'CAN_node_connection_error', 'field': 'CAN_node_connection_error'},
]

# =====================================================
# ===================== LAYOUT ========================
# =====================================================
# We create the labels and tables here so they render in order

ui.separator()
ui.label("Motor Telemetry").classes("text-lg font-bold")
motor_table = ui.table(columns=motor_columns, rows=[]).classes('w-full')

ui.separator()
ui.label("Motor Temp / Voltage").classes("text-lg font-bold")
info_table = ui.table(columns=info_columns, rows=[]).classes('w-full')

ui.separator()
ui.label("Motor Error Flags").classes("text-lg font-bold")
error_table = ui.table(columns=error_columns, rows=[]).classes('w-full')

ui.separator()
ui.label("Gripper Telemetry").classes("text-lg font-bold")
gripper_table = ui.table(columns=gripper_columns, rows=[]).classes('w-full')

ui.separator()
ui.label("Joint data").classes("text-lg font-bold")
joint_table = ui.table(columns=joint_columns, rows=[]).classes('w-full')

ui.separator()
ui.label("TCP data").classes("text-lg font-bold")
tcp_table = ui.table(columns=TCP_columns, rows=[]).classes('w-full')


ui.separator()
ui.label("RTI Loop Diagnostics").classes("text-lg font-bold")

with ui.row().classes("gap-6"):
    ts_label = ui.label("Timestamp: 0.000000").classes("font-mono")
    tick_label = ui.label("Loop Tick: 0").classes("font-mono")
    dur_label = ui.label("Avg Loop Duration: 0.000 ms").classes("font-mono")
    exec_label = ui.label("Avg Loop Execution: 0.000 ms").classes("font-mono")
    gripper_connected_label = ui.label("Gripper Connected: False").classes("font-mono")
    error_active_label = ui.label("error_active: False").classes("font-mono")


ui.separator()
ui.label("RTI CAN Node Status").classes("text-lg font-bold")
rti_table = ui.table(columns=rti_columns, rows=[]).classes('w-full')

ui.separator()
ui.label("RTI Received / Command IDs").classes("text-lg font-bold")

# Labels to display the arrays
received_ids_label = ui.label("Received IDs: []").classes("font-mono")
latest_commands_id_label = ui.label("Latest Commands IDs: []").classes("font-mono")

robot_mode_label = ui.label("robot_mode:").classes("font-mono")
robot_state_label = ui.label("robot_state_label: []").classes("font-mono")
robot_homed_state_label = ui.label("robot_homed_state_label: []").classes("font-mono")
robot_error_state_label = ui.label("robot_error_state_label: []").classes("font-mono")
robot_sim_mode_label = ui.label("robot_sim_mode_label: []").classes("font-mono")
robot_command_label = ui.label("robot_command_label: []").classes("font-mono")

# =====================================================
# ================= UPDATE FUNCTION ===================
# =====================================================

def update_tables():
    # ---------- MOTOR TELEMETRY ----------
    motor_rows = []
    for i in range(JOINTS):
        motor_rows.append({
            'joint': f'J{i+1}',
            'pos': int(shared_data.motor['motor_position'][i]),
            'spd': int(shared_data.motor['motor_speed'][i]),
            'cur': int(shared_data.motor['motor_current'][i]),
            'err': int(shared_data.motor['global_motor_error'][i]),
            'HARDWARE_VERSION': int(shared_data.motor['HARDWARE_VERSION'][i]),
            'BATCH_DATE': int(shared_data.motor['BATCH_DATE'][i]),
            'SOFTWARE_VERSION': int(shared_data.motor['SOFTWARE_VERSION'][i]),
            'SERIAL_NUMBER': int(shared_data.motor['SERIAL_NUMBER'][i]),


        })
    motor_table.rows = motor_rows

    # ---------- TEMP / VOLT ----------
    info_rows = []
    for i in range(JOINTS):
        info_rows.append({
            'joint': f'J{i+1}',
            'temp': int(shared_data.motor['motor_temperature'][i]),
            'volt': int(shared_data.motor['motor_voltage'][i]),
            'kt':   f"{shared_data.motor['motor_Kt'][i]:.4f}",
        })
    info_table.rows = info_rows

    # ---------- ERROR FLAGS ----------
    error_rows = []
    for i in range(JOINTS):
        error_rows.append({
            'joint': f'J{i+1}',
            'temp': int(shared_data.motor['motor_temperature_error'][i]),
            'enc': int(shared_data.motor['motor_encoder_error'][i]),
            'vbus': int(shared_data.motor['motor_vbus_error'][i]),
            'drv': int(shared_data.motor['motor_driver_error'][i]),
            'vel': int(shared_data.motor['motor_velocity_error'][i]),
            'cur': int(shared_data.motor['motor_current_error'][i]),
            'estop': int(shared_data.motor['motor_estop_error'][i]),
            'wd': int(shared_data.motor['motor_watchdog_error'][i]),
            'cal': int(shared_data.motor['motor_calibrated'][i]),
        })
    error_table.rows = error_rows

    # ---------- GRIPPER ----------
    gripper_table.rows = [{
        'pos': int(shared_data.gripper['gripper_position'][0]),
        'cur': int(shared_data.gripper['gripper_current'][0]),
        'act': int(shared_data.gripper['gripper_activated'][0]),
        'gripper_action': int(shared_data.gripper['gripper_action'][0]),
        'obj': int(shared_data.gripper['gripper_object_detection'][0]),
        'cal': int(shared_data.gripper['gripper_calibrated'][0]),
        'gripper_temperature_error': int(shared_data.gripper['gripper_temperature_error'][0]),
        'gripper_timeout_error': int(shared_data.gripper['gripper_timeout_error'][0]),
        'gripper_error': int(shared_data.gripper['gripper_error'][0]),
        'gripper_estop_error': int(shared_data.gripper['gripper_estop_error'][0]),
        'gripper_motor_position': int(shared_data.gripper['gripper_motor_position'][0]),
        'gripper_motor_speed': int(shared_data.gripper['gripper_motor_speed'][0]),
        'gripper_motor_current': int(shared_data.gripper['gripper_motor_current'][0]),
    }]


    # --- build joint table rows ---
    joint_rows = []
    for i in range(JOINTS):
        joint_rows.append({
            'joint': f'J{i+1}',
            'joint_position': fmt(shared_data.joint['joint_position'][i]),
            'joint_speed': fmt(shared_data.joint['joint_speed'][i]),
            'joint_torque': fmt(shared_data.joint['joint_torque'][i]),
            'joint_acceleration': fmt(shared_data.joint['joint_acceleration'][i]),
            'joint_jerk': fmt(shared_data.joint['joint_jerk'][i]),
            'joint_position_commanded': fmt(shared_data.joint['joint_position_commanded'][i]),
            'joint_speed_commanded': fmt(shared_data.joint['joint_speed_commanded'][i]),
            'joint_acceleration_commanded': fmt(shared_data.joint['joint_acceleration_commanded'][i]),
            'joint_torque_commanded': fmt(shared_data.joint['joint_torque_commanded'][i]),
            'torque_external': fmt(shared_data.tcp['torque_external'][i]),
        })

    # --- update the table ---
    joint_table.rows = joint_rows


    # ---------- TCP data ----------


    # TCP labels
    TCP_LABELS = ["X", "Y", "Z", "RX", "RY", "RZ"]

    # ---------- TCP data ----------
    tcp_rows = []
    for i, label in enumerate(TCP_LABELS):
        tcp_rows.append({
            'axis': label,
            'robot_pose': fmt(shared_data.tcp['robot_pose'][i]),
            'TCP_speed': fmt(shared_data.tcp['TCP_speed'][i]),
            'force_external': fmt(shared_data.tcp['force_external'][i]),
            'TCP_pose_commanded': fmt(shared_data.tcp['TCP_pose_commanded'][i]),
            'TCP_speed_commanded': fmt(shared_data.tcp['TCP_speed_commanded'][i]),
        })

    tcp_table.rows = tcp_rows

        # ---------- RTI DATA ----------
    ts = shared_data.rti["timestamp"][0]
    tick = shared_data.rti["loop_tick"][0]
    dur = shared_data.rti["avg_loop_duration"][0]
    exec_t = shared_data.rti["avg_loop_execution_time"][0]
    gripper_connected = bool(shared_data.rti["CAN_gripper_connected"][0])

    ts_label.text = f"Timestamp: {ts:12.6f}"
    tick_label.text = f"Loop Tick: {int(tick)}"
    dur_label.text = f"Avg Loop Duration: {dur*1000:8.3f} ms"
    exec_label.text = f"Avg Loop Execution: {exec_t*1000:8.3f} ms"
    gripper_connected_label.text = f"Gripper Connected: {gripper_connected}"
    error_active_label.text = f"Active error: {shared_data.rti['error_active'][0]}"


        # ---------- RTI NODE ARRAYS ----------
    rti_rows = []
    for i in range(16):
        rti_rows.append({
            'node': f'N{i+1}',
            'scan_received_ids': int(shared_data.rti["scan_received_ids"][i]),
            'Error_counter': int(shared_data.rti["Error_counter"][i]),
            'old_data_warning': int(shared_data.rti["old_data_warning"][i]),
            'CAN_node_connection_error': int(shared_data.rti["CAN_node_connection_error"][i]),
        })

    rti_table.rows = rti_rows

    # ---------- RTI RECEIVED / LATEST COMMANDS ----------
    # Convert numpy arrays to lists for display
    received_ids_label.text = f"Received IDs: {shared_data.rti['received_ids'][:].tolist()}"
    latest_commands_id_label.text = f"Latest Commands IDs: {shared_data.rti['latest_commands_id'][:].tolist()}"


     # ---------- Robot modes----------
    robot_mode_label.text = f"robot_mode: {shared_data.robot_mode.get()}"
    robot_state_label.text = f"robot_state: {shared_data.robot_state.get()}"
    robot_homed_state_label.text = f"robot_homed_state: {shared_data.homed_state.get()}"
    robot_error_state_label.text = f"robot_error_state: {shared_data.error_state.get()}"
    robot_sim_mode_label.text = f"robot_sim_mode: {shared_data.sim_mode.get()}"
    robot_command_label.text = f"robot_command: {shared_data.robot_command.get()}"


# ----------------- AUTO REFRESH -----------------
ui.timer(0.1, update_tables)

ui.run(reload=False,port=8081)