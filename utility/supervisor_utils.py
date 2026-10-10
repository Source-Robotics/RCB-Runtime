#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor_utils.py
Description: Utility functions for supervisor, including PCB UART communication.
"""

from __future__ import annotations

import serial
import numpy as np
import logging

logger = logging.getLogger(__name__)







# ==========================================================================
# Reads data from the PCB serial connection in a non-blocking manner.
# ==========================================================================
def read_pcb_data(ser: serial.Serial, expected_length: int = 4) -> np.ndarray | None:
    """
    Reads data from the PCB serial connection in a non-blocking manner.
    
    Args:
        ser (serial.Serial): The serial connection object.
        expected_length (int): The expected number of data points.
    
    Returns:
        np.ndarray: Parsed data points as a NumPy array if valid, otherwise None.
    """
    if ser.in_waiting > 0:  # Check if data is available
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line.startswith("$"):
            data_points = line[1:].split()  # Remove '$' and split into data points
            if len(data_points) == expected_length:  # Ensure correct number of values
                try:
                    pcb_data = np.array(data_points, dtype=np.int32)  # Convert to NumPy array
                    logger.info(f"Received PCB data: {pcb_data}")
                    return pcb_data  
                except ValueError:
                    logger.info(f"Received PCB data: ValueError")
                    return None  # Discard if conversion fails (e.g., non-numeric data)
    return None  # Invalid message discarded


# ==========================================================================
# Sends data to the PCB control board via serial connection in a non-blocking manner.
# ==========================================================================
def send_data(ser: serial.Serial, data: str) -> None:
    """
    Sends data to the PCB control board via serial connection in a non-blocking manner.
    
    Args:
        ser (serial.Serial): The serial connection object.
        data (string): String data we are sending

    Example:
        send_data(ser,"test")
    """
    data_2_send = data.encode('utf-8')
    ser.write(data_2_send)
    # The heartbeat goes out every blink period (~0.3 s): DEBUG, or it floods the log (2026-10-01).
    (logger.debug if data == "heartbeat\n\r" else logger.info)(f"Send data to PCB: {data_2_send}")

