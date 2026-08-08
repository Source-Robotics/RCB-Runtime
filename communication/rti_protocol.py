"""RTI (Real-Time Interface) protocol: command packet parsing and constants.

Binary command packet format (101 bytes):
  Offset  Type        Field              Count   Size
  ─────────────────────────────────────────────────────
  0       uint32      sequence_number    1       4
  4       uint8       command_type       1       1
  5       float64     payload            6       48
  53      float64     feedforward        6       48
  ─────────────────────────────────────────────────────
  Total: 101 bytes

State packets use the recipe-driven format (see data/data_recipe.py build_payload).
"""

import struct
import numpy as np

# Command type constants
CMD_JOINT_POSITION = 0x01
CMD_JOINT_VELOCITY = 0x02
CMD_JOINT_TORQUE = 0x03
CMD_CARTESIAN_POSE = 0x04
CMD_CARTESIAN_VELOCITY = 0x05
CMD_STOP = 0xFF

# Clock mode constants
CLOCK_ROBOT = 0  # Robot-clocked: PC blocks on recv, robot-paced
CLOCK_PC = 1     # PC-clocked: PC sends when it wants, robot uses latest

# Binary packet format string: uint32 + uint8 + 12*float64
CMD_FMT = '<I B 12d'
CMD_SIZE = struct.calcsize(CMD_FMT)  # 101 bytes


def unpack_command_packet(data):
    """Unpack PC command from binary UDP packet.

    Args:
        data: raw bytes from UDP

    Returns:
        (seq, cmd_type, payload[6], feedforward[6]) as numpy arrays if valid, None on error.
    """
    if len(data) < CMD_SIZE:
        return None
    try:
        vals = struct.unpack(CMD_FMT, data[:CMD_SIZE])
        seq = vals[0]
        cmd_type = vals[1]
        payload = np.array(vals[2:8], dtype=np.float64)
        feedforward = np.array(vals[8:14], dtype=np.float64)
        return seq, cmd_type, payload, feedforward
    except struct.error:
        return None


def pack_command_packet(seq, cmd_type, payload, feedforward):
    """Pack command packet for sending to robot.

    Args:
        seq: sequence number (uint32)
        cmd_type: command type code (uint8)
        payload: 6-element array (float64)
        feedforward: 6-element array (float64)

    Returns:
        bytes: 101-byte binary packet
    """
    payload = np.asarray(payload, dtype=np.float64)
    feedforward = np.asarray(feedforward, dtype=np.float64)

    if payload.shape != (6,) or feedforward.shape != (6,):
        raise ValueError("payload and feedforward must be 6-element arrays")

    data = struct.pack(
        CMD_FMT,
        seq, cmd_type,
        *payload, *feedforward
    )
    return data
