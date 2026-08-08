#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: can_utilities.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    CAN bus utility functions for RTI main loop.
    Handles scheduled messaging, override commands, and main loop sends.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

from collections import deque


# ==========================================================================
# Scheduled Message State
# ==========================================================================

# Global state for scheduled messages (temperature, voltage, errors)
scheduled_state = {
    "step": 0,           # 0 = temp, 1 = voltage, 2 = error
    "joint_index": 0     # joint to send scheduled message to
}

# Override command queue - allows inserting priority messages
scheduled_override_queue = deque()  # Each item is a tuple: (function, repeat_count)

current_override = {
    "fn": None,
    "count": 0
}

device_info_counter = 0  # Counts full polling cycles


# ==========================================================================
# Override Queue Management
# ==========================================================================

def add_override_command(command_function, repeat_count=1):
    """
    Add a command to the override queue.
    
    Args:
        command_function: Function to call (e.g., lambda: Motor[i].Send_firmware_version())
        repeat_count (int): Number of times to repeat this command (default: 1)
    
    Example:
        add_override_command(lambda: Motor[3].Send_firmware_version(), 3)
    """
    scheduled_override_queue.append((command_function, repeat_count))
    print(f"[CAN Utilities] Override command added (repeat {repeat_count}x)")


def clear_override_queue():
    """Clear all override commands."""
    scheduled_override_queue.clear()
    current_override["fn"] = None
    current_override["count"] = 0
    print("[CAN Utilities] Override queue cleared")


# ==========================================================================
# Main Loop Send Function
# ==========================================================================

def main_loop_send(Motor, joint_num, active_CAN_gripper, dummy_device, compensation_torque):
    """
    Main CAN send function - sends control data and scheduled telemetry.
    
    This function:
    1. Sends primary control data to all motors
    2. Sends gripper data if active
    3. Handles override commands if present
    4. Performs round-robin telemetry polling (temp, voltage, errors, device info)
    
    Args:
        Motor (list): List of motor objects
        joint_num (int): Number of joints
        active_CAN_gripper (int): 1 if gripper is active, 0 otherwise
        dummy_device: Dummy CAN device for timing
        compensation_torque (np.array): Torque compensation values
    """
    global device_info_counter, scheduled_state, current_override, scheduled_override_queue
    
    # 1. Send primary control data to all motors
    for i in range(joint_num):
        Motor[i].Send_data_pack_1(
            Position=None,
            Speed=None,
            Current=compensation_torque[i]
        )
    
    # 2. Send gripper data if active
    if active_CAN_gripper == 1:
        Motor[joint_num].Send_data_pack_1(Position=None, Speed=0, Current=0)
    else:
        # Send dummy data to maintain loop timing consistency
        dummy_device.Send_gripper_data_pack(0, 0, 0, 1, 1, 0, 0)
    
    # 3. Handle override queue
    if current_override["fn"] is None and scheduled_override_queue:
        fn, count = scheduled_override_queue.popleft()
        current_override["fn"] = fn
        current_override["count"] = count
    
    if current_override["fn"] is not None:
        try:
            current_override["fn"]()  # Call the current override command
            # print("[CAN Utilities] Sent override command")
        except Exception as e:
            print(f"[CAN Utilities][ERROR] Override command failed: {e}")
        
        current_override["count"] -= 1
        if current_override["count"] <= 0:
            current_override["fn"] = None  # Clear it so next in queue is loaded
        
        return  # Skip regular scheduling this loop if override is active
    
    # 4. Handle round-robin status polling (only if no override is active)
    device_info_counter += 1
    step = scheduled_state["step"]
    joint_idx = scheduled_state["joint_index"]
    
    # Device Info sweep window (loops 1000 to 1000 + joint_num)
    # This ensures every motor is polled for device info exactly once per 1000 loops
    if 1000 <= device_info_counter < (1000 + joint_num):
        Motor[joint_idx].Send_Respond_Device_Info()
        
        # Reset counter after last motor in sweep
        if device_info_counter == (1000 + joint_num - 1):
            device_info_counter = 0
    else:
        # Normal round-robin telemetry (Temp, Volt, Errors)
        if step == 0:
            Motor[joint_idx].Send_Respond_Temperature()
        elif step == 1:
            Motor[joint_idx].Send_Respond_Voltage()
        elif step == 2:
            Motor[joint_idx].Send_Respond_State_of_Errors()
    
    # 5. Advance indices for next loop iteration
    scheduled_state["joint_index"] += 1
    if scheduled_state["joint_index"] >= joint_num:
        scheduled_state["joint_index"] = 0
        
        # Only advance telemetry step if we didn't use this cycle for Device Info
        if device_info_counter != 0:
            scheduled_state["step"] = (scheduled_state["step"] + 1) % 3


# ==========================================================================
# Secondary Data Send (Watchdog, etc.)
# ==========================================================================

def send_watchdog_to_all(Motor, joint_num, rate=5000, action='Idle'):
    """
    Send watchdog configuration to all motors.
    
    Args:
        Motor (list): List of motor objects
        joint_num (int): Number of joints
        rate (int): Watchdog timeout in ms (default: 5000)
        action (str): Action on timeout (default: 'Idle')
    """
    for i in range(joint_num):
        Motor[i].Send_Watchdog(_rate=rate, _action=action)


# ==========================================================================
# Utility Functions
# ==========================================================================

def reset_scheduled_state():
    """Reset scheduled message state to initial values."""
    global scheduled_state, device_info_counter
    scheduled_state["step"] = 0
    scheduled_state["joint_index"] = 0
    device_info_counter = 0
    print("[CAN Utilities] Scheduled state reset")


def get_scheduled_state_info():
    """
    Get current scheduling state for diagnostics.
    
    Returns:
        dict: Current scheduling state
    """
    return {
        'step': scheduled_state["step"],
        'joint_index': scheduled_state["joint_index"],
        'device_info_counter': device_info_counter,
        'override_active': current_override["fn"] is not None,
        'override_remaining': current_override["count"],
        'queue_length': len(scheduled_override_queue)
    }


def print_scheduled_state():
    """Print current scheduling state."""
    info = get_scheduled_state_info()
    
    step_names = {0: "Temperature", 1: "Voltage", 2: "Errors"}
    
    print("\n" + "="*50)
    print("CAN SCHEDULING STATE")
    print("="*50)
    print(f"Current Step:         {step_names.get(info['step'], 'Unknown')}")
    print(f"Current Joint:        J{info['joint_index']}")
    print(f"Device Info Counter:  {info['device_info_counter']}")
    print(f"Override Active:      {info['override_active']}")
    if info['override_active']:
        print(f"Override Remaining:   {info['override_remaining']}")
    print(f"Override Queue:       {info['queue_length']} items")
    print("="*50 + "\n")


if __name__ == "__main__":
    """Test CAN utilities"""
    print("CAN Utilities Test")
    print("-" * 50)
    
    # Test scheduled state
    print("\nInitial state:")
    print_scheduled_state()
    
    # Simulate adding overrides
    print("\nAdding test override commands...")
    add_override_command(lambda: print("Override command 1"), 2)
    add_override_command(lambda: print("Override command 2"), 1)
    
    print("\nState after adding overrides:")
    print_scheduled_state()
    
    # Reset
    print("\nResetting state...")
    reset_scheduled_state()
    clear_override_queue()
    
    print("\nState after reset:")
    print_scheduled_state()
