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
