#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/data_sources.py
Description:
    Read-only getters for the OLED. Every getter degrades to "—" if the
    underlying shared-memory segment / system call isn't available (e.g. RTI not
    running), so the UI never crashes on missing data. Pages call these; they
    never touch shared memory or run subprocesses directly.
"""

import os
import time
import socket
import subprocess

import psutil
from data import shared_data
from data.shared_memory_index import ESTOP_1
from utility.system_temperature import get_raspberry_pi_temperature
from supervisor import state

_ip = {"v": "—", "t": 0.0}


def _safe(fn, default="—"):
    try:
        v = fn()
        return v if v not in (None, "") else default
    except Exception:
        return default


# --- network / system ---
def ip():
    """Primary IP, refreshed at most every 3 s."""
    now = time.perf_counter()
    if now - _ip["t"] >= 3.0:
        out = subprocess.getoutput("hostname -I | cut -d' ' -f1").strip()
        _ip["v"] = out or "—"
        _ip["t"] = now
    return _ip["v"]


def force_confirm():
    """(pending, who, seconds_left) for a forced control takeover awaiting a button press.

    Returns (False, "", 0.0) when nothing is pending, when the fields do not exist yet
    (an older RTI that has not been restarted since the field was added), or on any read
    error — a prompt that cannot be read must not become a prompt that never clears.
    """
    try:
        rti = shared_data.rti
        if float(rti["force_confirm_pending"][0]) < 0.5:
            return False, "", 0.0
        # monotonic: must match the clock command_executor._await_physical_confirm writes.
        left = float(rti["force_confirm_deadline"][0]) - time.monotonic()
        if left <= 0:
            return False, "", 0.0     # stale: the executor died mid-request
        return True, shared_data.force_confirm_who.get() or "unknown", left
    except Exception:
        return False, "", 0.0


def hostname(): return _safe(socket.gethostname)
def mem_pct():  return _safe(lambda: f"{psutil.virtual_memory().percent:.0f}%")
def load1():    return _safe(lambda: f"{os.getloadavg()[0]:.2f}")


def temp():
    t = get_raspberry_pi_temperature()
    return f"{t:.0f}C" if t is not None else "—"


def uptime():
    def _f():
        with open("/proc/uptime") as fh:
            secs = float(fh.read().split()[0])
        return f"{int(secs // 3600)}h{int((secs % 3600) // 60):02d}"
    return _safe(_f)


# --- robot state (shared memory) ---
def mode():   return _safe(lambda: shared_data.robot_mode.get())
def state_(): return _safe(lambda: shared_data.robot_state.get())
def homed():  return _safe(lambda: shared_data.homed_state.get())
def can():    return _safe(lambda: "OK" if float(shared_data.rti["robot_can_connected"][0]) else "no")
def estop():  return _safe(lambda: "!" if int(shared_data.gpio_states[ESTOP_1]) else "-")


def motor_positions():
    try:
        return [int(v) for v in shared_data.motor["motor_position"]]
    except Exception:
        return [0] * 6


# --- managed scripts (from the monitor thread snapshot) ---
def scripts():     return state.global_monitor_data
def rti_status():  return _safe(lambda: state.global_monitor_data.get("RTI", {}).get("status"))
