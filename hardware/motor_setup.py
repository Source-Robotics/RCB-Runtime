#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: motor_setup.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    Motor and joint object initialization for the robot arm.
    Creates motor controllers and joint objects based on XML configuration.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import logging
import Spectral_BLDC as Spectral
import SourceRoboticsToolbox
import numpy as np

logger = logging.getLogger(__name__)


# ==========================================================================
# Motor Initialization
# ==========================================================================

def create_motor_objects(joint_num, communication, append_gripper=True):
    """
    Create motor controller objects for robot joints and optional gripper.
    
    Args:
        joint_num (int): Number of robot joints
        communication: CAN communication object
        append_gripper (bool): Add gripper motor at end (default: True)
    
    Returns:
        list: List of SpectralCAN motor objects
    """
    Motor = []
    
    # Create motor objects for robot joints (CAN IDs 0 to joint_num-1)
    for i in range(joint_num):
        Motor.append(Spectral.SpectralCAN(node_id=i, communication=communication))
    
    # Append gripper motor (CAN ID = joint_num)
    if append_gripper:
        Motor.append(Spectral.SpectralCAN(node_id=joint_num, communication=communication))
        logger.info(f"Created {joint_num} joint motors + 1 gripper motor")
    else:
        logger.info(f"Created {joint_num} joint motors")
    
    return Motor


def create_dummy_device(communication, node_id=13):
    """
    Create a dummy CAN device (reserved ID).

    CAN node reservations:
    - 13 is reserved for this RTI dummy timing device.
    - 14 and 15 are reserved for the host application's own use,
      including bootloader-related host functions.
    Do not assign physical joints, grippers, tools, or other devices to
    nodes 13, 14, or 15.

    Args:
        communication: CAN communication object
        node_id (int): CAN node ID (default: 13, reserved for dummy)
    
    Returns:
        SpectralCAN: Dummy motor object
    """
    dummy = Spectral.SpectralCAN(node_id=node_id, communication=communication)
    logger.info(f"Dummy device created at CAN ID {node_id}")
    return dummy


# ==========================================================================
# Joint Initialization
# ==========================================================================

def create_joint_objects(joint_num, motor_encoder, master_position, gear_ratio, 
                        motor_offset, motor_dir):
    """
    Create joint kinematics objects.
    
    Args:
        joint_num (int): Number of joints
        motor_encoder (np.array): Encoder resolution for each joint
        master_position (np.array): Master/home position for each joint
        gear_ratio (np.array): Gear ratio for each joint
        motor_offset (np.array): Position offset for each joint
        motor_dir (np.array): Direction (0 or 1) for each joint
    
    Returns:
        list: List of Joint objects
    """
    Joint = []
    
    for i in range(joint_num):
        Joint.append(
            SourceRoboticsToolbox.Joint(
                encoder_resolution=int(motor_encoder[i]),
                master_position=master_position[i],
                gear_ratio=gear_ratio[i],
                offset=motor_offset[i],
                dir=motor_dir[i]
            )
        )
    
    logger.info(f"Created {joint_num} joint objects")
    return Joint


# ==========================================================================
# Filter Setup
# ==========================================================================

def initialize_filters(
    joint_num,
    alpha_speed=0.15,
    alpha_torque=0.25,
    alpha_tcp_acceleration=0.15,
):
    """
    Initialize filter arrays and parameters.
    
    Args:
        joint_num (int): Number of joints
        alpha_speed (float): Smoothing factor for speed filter (default: 0.15)
        alpha_torque (float): Smoothing factor for torque filter (default: 0.25)
        alpha_tcp_acceleration (float): Smoothing factor for TCP acceleration
    
    Returns:
        dict: Dictionary containing filter arrays and parameters
    """
    filters = {
        'joint_speed_filtered': np.zeros(joint_num, dtype=float),
        'tcp_acceleration_filtered': np.zeros(6, dtype=float),
        'tcp_acceleration_filter_initialized': np.zeros(1, dtype=bool),
        'joint_torque_filtered': np.zeros(joint_num, dtype=float),
        'alpha_Speed': alpha_speed,
        'alpha_TCP_acceleration': alpha_tcp_acceleration,
        'alpha_Torque': alpha_torque
    }
    
    logger.info(f"Filters initialized (Speed α={alpha_speed}, Torque α={alpha_torque})")
    return filters


# ==========================================================================
# Control Variable Initialization
# ==========================================================================

def initialize_control_variables(joint_num):
    """
    Initialize control-related variables.
    
    Args:
        joint_num (int): Number of joints
    
    Returns:
        dict: Dictionary containing control variables
    """
    control_vars = {
        'compensation_torque': np.zeros(joint_num, dtype=int),
        'sector_home_status': np.zeros(joint_num, dtype=int),
        'motor_error_bytes': np.zeros(joint_num, dtype=np.int32)
    }
    
    logger.info("Control variables initialized")
    return control_vars


# ==========================================================================
# Housekeeping Variable Initialization
# ==========================================================================

def initialize_housekeeping_variables(num_motors):
    """
    Initialize housekeeping and diagnostic variables.
    
    Args:
        num_motors (int): Total number of motors (joints + gripper)
    
    Returns:
        dict: Dictionary containing housekeeping variables
    """
    housekeeping = {
        'received_ids': np.zeros(num_motors, dtype=int),
        'scan_received_ids': np.zeros(16, dtype=int),
        'latest_commands_id': np.full(num_motors, -1, dtype=int),
        # Live aggregate fault bit per node, taken from msg_ID[2] of EVERY reply that node sends.
        # Because it rides the CAN ID rather than the payload, it is valid whatever the node
        # replied with — which is what makes it work for the gripper in BOTH firmware and motor
        # mode (those return different payloads into different deques). -1 = nothing heard yet.
        'node_error_bits': np.full(num_motors, -1, dtype=int),
        'CAN_node_connection_error': np.zeros(16, dtype=int),
        'old_data_warning': np.zeros(16, dtype=int),
        'data_fresh_limit': 10,
        'startup_selfcheck': 0,
        'gripper_old_data': 0,
        'loop_tick_cnt': 0,
        'loop_execution_time': 0.0,
        'avg_loop_duration': 0.0,
        'avg_loop_execution_time': 0.0,
        'command_success_rate': 0.0,
        'CAN_gripper_connected': 0,
        'Hall_index': 0
    }
    
    logger.info("Housekeeping variables initialized")
    return housekeeping


# ==========================================================================
# Complete Motor System Setup
# ==========================================================================

def setup_motor_system(joint_num, communication, xml_config, append_gripper=True):
    """
    Complete motor system setup including motors, joints, filters, and variables.
    
    Args:
        joint_num (int): Number of robot joints
        communication: CAN communication object
        xml_config (dict): Configuration from XML file containing motor parameters
        append_gripper (bool): Add gripper motor (default: True)
    
    Returns:
        dict: Dictionary containing all motor system components:
            - Motor: List of motor objects
            - Joint: List of joint objects
            - dummy_device: Dummy CAN device
            - filters: Filter arrays and parameters
            - control: Control variables
            - housekeeping: Housekeeping variables
    """
    logger.info("Motor system setup starting...")
    
    # Create motor objects
    Motor = create_motor_objects(joint_num, communication, append_gripper)
    
    # Create dummy device
    dummy_device = create_dummy_device(communication)
    
    # Create joint objects
    Joint = create_joint_objects(
        joint_num,
        xml_config['motor_encoder'],
        xml_config['master_position'],
        xml_config['gear_ratio'],
        xml_config['motor_offset'],
        xml_config['motor_dir']
    )
    
    # Initialize filters
    filters = initialize_filters(joint_num)
    
    # Initialize control variables
    control = initialize_control_variables(joint_num)
    
    # Initialize housekeeping variables
    num_motors = joint_num + (1 if append_gripper else 0)
    housekeeping = initialize_housekeeping_variables(num_motors)
    
    logger.info("Motor system setup complete")
    
    return {
        'Motor': Motor,
        'Joint': Joint,
        'dummy_device': dummy_device,
        'filters': filters,
        'control': control,
        'housekeeping': housekeeping
    }


if __name__ == "__main__":
    """Test motor setup (requires hardware)"""
    print("Motor Setup Test")
    print("-" * 50)
    print("\nNote: This test requires actual CAN hardware")
    print("Skipping hardware initialization...")
    
    # Example XML config structure
    example_config = {
        'motor_encoder': np.array([14, 14, 14, 14, 14, 14]),
        'master_position': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'gear_ratio': np.array([100, 100, 100, 100, 100, 100]),
        'motor_offset': np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        'motor_dir': np.array([0, 0, 0, 0, 0, 0])
    }
    
    print("\nExample configuration:")
    for key, value in example_config.items():
        print(f"  {key}: {value}")
