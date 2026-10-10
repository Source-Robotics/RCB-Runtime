"""Small helpers used by RTI.py and command_executor.py.

Until 2026-10-03 this was a facade that star-re-exported utility.data_init / signal_processing / safety_checks /
serialization so `from utility.RTI_utils import *` kept working. Importers now name what they use from the module
that defines it (refactor Phase 2); what is left here are the helpers that belong to no other module.
"""
from __future__ import annotations

from typing import Any

import json
import random
import socket
import string
import sys

BASE62 = string.digits + string.ascii_letters


def generate_short_id(length: int = 6) -> str:
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


def read_shared_string(shm):
    """Read a UTF-8 string from shared memory (4-byte length prefix)."""
    data_len = int.from_bytes(shm.buf[:4], 'little')
    return bytes(shm.buf[4:4 + data_len]).decode('utf-8')


def read_shared_json(shm):
    """Read a JSON object from shared memory."""
    return json.loads(read_shared_string(shm))


def print_heartbeat(shared_data: Any) -> None:
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
