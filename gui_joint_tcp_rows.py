"""Build the existing Joints & TCP table rows without depending on NiceGUI.

The dashboard supplies its current format and angle-unit functions. This keeps
its display rules in one place while letting another view consume the same rows.
"""


def joint_tcp_rows(shared_data, joints, fmt, ang, ang_tcp):
    """Return (joint rows, TCP rows) in the dashboard's current display units."""
    joint_rows = [{
        'j':       f'J{i+1}',
        'homed':   'YES' if shared_data.rti['joint_homed'][i] else 'NO',
        'pos':     fmt(ang(shared_data.joint['joint_position'][i])),
        'spd':     fmt(ang(shared_data.joint['joint_speed'][i])),
        'trq':     fmt(shared_data.joint['joint_torque'][i]),
        'acc':     fmt(ang(shared_data.joint['joint_acceleration'][i])),
        'jrk':     fmt(ang(shared_data.joint['joint_jerk'][i])),
        'pos_cmd': fmt(ang(shared_data.joint['joint_position_commanded'][i])),
        'spd_cmd': fmt(ang(shared_data.joint['joint_speed_commanded'][i])),
        'trq_cmd': fmt(shared_data.joint['joint_torque_commanded'][i]),
        'trq_ext': fmt(shared_data.tcp['torque_external'][i]),
    } for i in range(joints)]

    tcp_rows = [{
        'axis':      lbl,
        'pose':      fmt(ang_tcp(shared_data.tcp['robot_pose'][i], i)),
        'spd':       fmt(ang_tcp(shared_data.tcp['TCP_speed'][i], i)),
        'force_ext': fmt(shared_data.tcp['force_external'][i]),
        'pose_cmd':  fmt(ang_tcp(shared_data.tcp['TCP_pose_commanded'][i], i)),
        'spd_cmd':   fmt(ang_tcp(shared_data.tcp['TCP_speed_commanded'][i], i)),
    } for i, lbl in enumerate(["X", "Y", "Z", "RX", "RY", "RZ"])]

    return joint_rows, tcp_rows
