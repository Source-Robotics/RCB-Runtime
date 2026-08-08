"""
Backward-compatible facade for utility/RTI_utils.py.

Functions have been split into focused modules:
  - utility/data_init.py        — data structure setup & process_* helpers
  - utility/signal_processing.py — EMA timing, angle unwrap, filters
  - utility/safety_checks.py    — RT test, elbow config, singularity check
  - utility/serialization.py    — serialize_joint_data, serialize_optimized

All names from those modules are re-exported here so existing
`from utility.RTI_utils import *` and specific imports continue to work.
"""

# --- Re-exports from sub-modules ---
from utility.data_init import *
from utility.signal_processing import *
from utility.safety_checks import *
from utility.serialization import *

# --- Standard library imports needed by the functions below ---
import sys
import time
import psutil
import os
import socket
import subprocess
import random
import string
import json
from multiprocessing import shared_memory
import numpy as np

# CAN interface helpers — canonical in hardware/can_hardware.py
from hardware.can_hardware import is_can_interface_up, start_can_interface, stop_can_interface

# --------------------------------------------------------------------------- #
# Misc utilities that do not belong to any sub-module
# --------------------------------------------------------------------------- #

BASE62 = string.digits + string.ascii_letters


def generate_short_id(length=6):
    """Generate a random Base-62 short ID."""
    return ''.join(random.choices(BASE62, k=length))


def cmd_name(full_command: str) -> str:
    """Return the bare command name, stripping any argument list."""
    return full_command.partition("(")[0]


def get_local_ip():
    """Return the machine's primary non-loopback IP address."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP


def init_shared_string(name, size=256):
    """Create or attach a shared memory block. Returns the SharedMemory object."""
    try:
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
        shm.buf[:size] = b'\x00' * size
        print(f"Shared memory '{name}' created with size {size} bytes.")
    except FileExistsError:
        shm = shared_memory.SharedMemory(name=name)
        print(f"Shared memory '{name}' already exists.")
    return shm


def write_shared_string(shm, data):
    """Write a UTF-8 string to shared memory with a 4-byte length prefix."""
    byte_data = data.encode('utf-8')
    if len(byte_data) > len(shm.buf) - 4:
        raise ValueError("Data too large for shared memory block")
    shm.buf[:4] = len(byte_data).to_bytes(4, 'little')
    shm.buf[4:4 + len(byte_data)] = byte_data
    shm.buf[4 + len(byte_data):] = b'\x00' * (len(shm.buf) - 4 - len(byte_data))


def read_shared_string(shm):
    """Read a UTF-8 string from shared memory (4-byte length prefix)."""
    data_len = int.from_bytes(shm.buf[:4], 'little')
    return bytes(shm.buf[4:4 + data_len]).decode('utf-8')


def write_shared_json(shm, obj):
    """Write a JSON-serialisable object to shared memory."""
    json_bytes = json.dumps(obj, separators=(',', ':')).encode('utf-8')
    write_shared_string(shm, json_bytes.decode('utf-8'))


def read_shared_json(shm):
    """Read a JSON object from shared memory."""
    return json.loads(read_shared_string(shm))


def print_heartbeat(shared_data):
    """Write a single overwriting status line to stdout.
    Logger output goes to stderr so the two streams don't clobber each other.
    Call only when the tick interval condition is met (checked by the caller).

    Enable:   RTI_HEARTBEAT=1 python3 RTI.py
    Interval: RTI_HEARTBEAT=1 RTI_HEARTBEAT_INTERVAL=50 python3 RTI.py
    """
    tick    = int(shared_data.rti["loop_tick"][0])
    period  = shared_data.rti["loop_period"][0] * 1e3
    jitter  = shared_data.rti["jitter"][0] * 1e6
    err     = int(shared_data.rti["error_active"][0])
    can     = "OK  " if shared_data.rti["robot_can_connected"][0] else "FAIL"
    mode    = shared_data.robot_mode.get()
    sys.stdout.write(
        f"\r[RTI] tick={tick:9d} | {mode:<14s} | "
        f"period={period:6.3f}ms | jitter={jitter:5.1f}µs | "
        f"CAN={can} | err={err}   "
    )
    sys.stdout.flush()
