#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: UDP_telemetry.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    Telemetry data serialization and broadcasting.
    Handles UDP telemetry transmission and data serialization.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

from utility.RTI_utils import serialize_optimized


# ==========================================================================
# Telemetry Configuration
# ==========================================================================

# Default fields to include in telemetry
DEFAULT_JOINT_FIELDS = ["j_p", "j_s", "j_t", "tick", "j_err"]

DEFAULT_PRECISION_MAP = {
    "j_p": 4,       # Joint position - 4 decimal places
    "j_s": 3,       # Joint speed - 3 decimal places
    "j_t": 3,       # Joint torque - 3 decimal places
    "j_a": 3,       # Joint acceleration - 3 decimal places
    "tick": 0,      # Loop tick - integer
    "j_err": 0,     # Joint errors - integer
    "g_p": 3,       # Gripper position - 3 decimal places
    "g_c": 2,       # Gripper current - 2 decimal places
    "TCP_p": 4,     # TCP pose - 4 decimal places
    "TCP_s": 3      # TCP speed - 3 decimal places
}


# ==========================================================================
# Data Serialization
# ==========================================================================

def serialize_joint_telemetry(Joint_data, fields=None, precision_map=None):
    """
    Serialize joint data for telemetry transmission.
    
    Args:
        Joint_data (deque): Joint data deque
        fields (list): List of field names to include (default: DEFAULT_JOINT_FIELDS)
        precision_map (dict): Precision for each field (default: DEFAULT_PRECISION_MAP)
    
    Returns:
        str: Serialized data string
    """
    if fields is None:
        fields = DEFAULT_JOINT_FIELDS
    if precision_map is None:
        precision_map = DEFAULT_PRECISION_MAP
    
    return serialize_optimized(Joint_data, fields, precision_map)


def serialize_gripper_telemetry(Gripper_data, gripper_id, fields=None, precision_map=None):
    """
    Serialize gripper data for telemetry transmission.
    
    Args:
        Gripper_data (dict): Gripper data dictionary
        gripper_id (int): Gripper CAN ID
        fields (list): List of field names to include
        precision_map (dict): Precision for each field
    
    Returns:
        str: Serialized data string
    """
    if fields is None:
        fields = ["g_p", "g_c", "g_activated", "g_action"]
    if precision_map is None:
        precision_map = DEFAULT_PRECISION_MAP
    
    return serialize_optimized(Gripper_data[gripper_id], fields, precision_map)


def create_full_telemetry_packet(Joint_data, Gripper_data, System_data,
                                 joint_num, active_CAN_gripper,
                                 loop_execution_time, loop_tick_cnt):
    """
    Create a complete telemetry packet with all robot data.
    
    Args:
        Joint_data (deque): Joint data
        Gripper_data (dict): Gripper data
        System_data (deque): System data
        joint_num (int): Number of joints
        active_CAN_gripper (int): 1 if gripper active
        loop_execution_time (float): Current loop execution time
        loop_tick_cnt (int): Current loop tick
    
    Returns:
        dict: Complete telemetry packet
    """
    packet = {
        'timestamp': Joint_data[-1]['timestamp'],
        'loop_tick': loop_tick_cnt,
        'loop_time': loop_execution_time,
        'joint_position': Joint_data[-1]['j_p'].tolist(),
        'joint_speed': Joint_data[-1]['j_s'].tolist(),
        'joint_torque': Joint_data[-1]['j_t'].tolist(),
        'tcp_pose': Joint_data[-1]['TCP_p'].tolist(),
        'tcp_speed': Joint_data[-1]['TCP_s'].tolist(),
        'joint_errors': Joint_data[-1]['j_err'].tolist()
    }
    
    # Add gripper data if active
    if active_CAN_gripper:
        gripper_id = joint_num
        packet['gripper_position'] = Gripper_data[gripper_id][-1]['g_p']
        packet['gripper_current'] = Gripper_data[gripper_id][-1]['g_c']
        packet['gripper_status'] = Gripper_data[gripper_id][-1]['g_action']
    
    return packet


# ==========================================================================
# Telemetry Broadcasting
# ==========================================================================

def broadcast_telemetry(telemetry_socket, data, broadcast_address='255.255.255.255',
                       port=30003):
    """
    Broadcast telemetry data via UDP.
    
    Args:
        telemetry_socket: UDP socket for broadcasting
        data (str or bytes): Data to broadcast
        broadcast_address (str): Broadcast address (default: '255.255.255.255')
        port (int): Destination port (default: 30003)
    """
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    telemetry_socket.sendto(data, (broadcast_address, port))


def send_simple_telemetry(telemetry_socket, loop_execution_time, 
                          broadcast_address='255.255.255.255', port=30003):
    """
    Send simple telemetry (just loop execution time).
    
    Args:
        telemetry_socket: UDP socket for broadcasting
        loop_execution_time (float): Loop execution time
        broadcast_address (str): Broadcast address
        port (int): Destination port
    """
    data = str(loop_execution_time).encode('utf-8')
    telemetry_socket.sendto(data, (broadcast_address, port))


# ==========================================================================
# Telemetry Rate Control
# ==========================================================================

class TelemetryRateLimiter:
    """Rate limiter for telemetry broadcasting."""
    
    def __init__(self, max_rate_hz=100):
        """
        Initialize rate limiter.
        
        Args:
            max_rate_hz (int): Maximum telemetry rate in Hz (default: 100)
        """
        self.max_rate = max_rate_hz
        self.counter = 0
        self.divisor = 1  # Will be calculated based on loop frequency
    
    def should_send(self, loop_frequency_hz):
        """
        Check if telemetry should be sent this loop.
        
        Args:
            loop_frequency_hz (float): Current loop frequency
        
        Returns:
            bool: True if should send, False otherwise
        """
        # Calculate how many loops to skip
        self.divisor = max(1, int(loop_frequency_hz / self.max_rate))
        
        self.counter += 1
        if self.counter >= self.divisor:
            self.counter = 0
            return True
        return False


# ==========================================================================
# Advanced Telemetry Functions
# ==========================================================================

def create_diagnostic_telemetry(housekeeping, avg_loop_duration, 
                                avg_loop_execution_time, CAN_gripper_connected):
    """
    Create diagnostic telemetry packet with system health info.
    
    Args:
        housekeeping (dict): Housekeeping variables
        avg_loop_duration (float): Average loop duration
        avg_loop_execution_time (float): Average loop execution time
        CAN_gripper_connected (int): Gripper connection status
    
    Returns:
        dict: Diagnostic telemetry packet
    """
    return {
        'avg_loop_duration': avg_loop_duration,
        'avg_loop_execution_time': avg_loop_execution_time,
        'received_ids': housekeeping['received_ids'].tolist(),
        'old_data_warnings': housekeeping['old_data_warning'].tolist(),
        'connection_errors': housekeeping['CAN_node_connection_error'].tolist(),
        'gripper_connected': CAN_gripper_connected,
        'loop_tick': housekeeping['loop_tick_cnt']
    }


def serialize_diagnostic_telemetry(diag_data):
    """
    Serialize diagnostic data to string.
    
    Args:
        diag_data (dict): Diagnostic data dictionary
    
    Returns:
        str: Serialized diagnostic data
    """
    import json
    return json.dumps(diag_data)


if __name__ == "__main__":
    """Test telemetry functions"""
    print("Telemetry Module Test")
    print("-" * 50)
    
    import numpy as np
    from collections import deque
    
    # Create test data
    test_joint_data = deque([{
        'j_p': np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]),
        'j_s': np.array([0.01, 0.02, 0.03, 0.04, 0.05, 0.06]),
        'j_t': np.array([1.1, 2.2, 3.3, 4.4, 5.5, 6.6]),
        'tick': 1234,
        'j_err': np.array([0, 0, 0, 0, 0, 0], dtype=np.int32),
        'timestamp': 123456.789,
        'TCP_p': np.array([0.5, 0.5, 0.5, 0.1, 0.2, 0.3]),
        'TCP_s': np.array([0.1, 0.1, 0.1, 0.01, 0.01, 0.01])
    }], maxlen=50)
    
    print("\nTest 1: Serialize joint telemetry")
    serialized = serialize_joint_telemetry(test_joint_data)
    print(f"Serialized data (length: {len(serialized)} bytes):")
    print(serialized[:200] + "..." if len(serialized) > 200 else serialized)
    
    print("\nTest 2: Create full telemetry packet")
    test_gripper_data = {6: deque([{
        'g_p': 50,
        'g_c': 100,
        'g_activated': 1,
        'g_action': 2
    }], maxlen=50)}
    
    packet = create_full_telemetry_packet(
        test_joint_data, test_gripper_data, None,
        joint_num=6, active_CAN_gripper=1,
        loop_execution_time=0.002, loop_tick_cnt=1234
    )
    
    print("Telemetry packet keys:", list(packet.keys()))
    print(f"Joint position: {packet['joint_position']}")
    print(f"Gripper position: {packet['gripper_position']}")
    
    print("\nTest 3: Rate limiter")
    limiter = TelemetryRateLimiter(max_rate_hz=50)
    loop_freq = 200  # 200 Hz loop
    
    send_count = 0
    for i in range(200):
        if limiter.should_send(loop_freq):
            send_count += 1
    
    print(f"Loop frequency: {loop_freq} Hz")
    print(f"Telemetry rate limit: 50 Hz")
    print(f"Telemetry sent: {send_count} times in 200 loops")
    print(f"Effective rate: ~{send_count} Hz (expected ~50)")
    
    print("\nTelemetry module test complete!")
