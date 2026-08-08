#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: system_config.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    System configuration and initialization for RTI real-time control.
    Handles CPU affinity, real-time scheduling, system information collection,
    and logging setup.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import logging
import os
import re
import psutil
import platform
import distro
from gpiozero import pi_info
import numpy as np

logger = logging.getLogger(__name__)


# ==========================================================================
# System Configuration
# ==========================================================================

def setup_realtime_process(cpu_core=3, priority=99):
    """
    Configure process for real-time operation.

    Args:
        cpu_core (int): CPU core to bind process to (default: 3)
        priority (int): Real-time priority (1-99, default: 99)

    Behavior:
        Always reads back the *actual* policy / priority / affinity after the
        setting attempt and surfaces the result on stdout AND through the
        logger. This is the only way to know for sure whether the RT setup
        applied — Python's `sched_setscheduler` can be silently rejected by
        rlimit / capability checks on misconfigured hosts, and `ulimit -r`
        in a parent shell does not always reflect the kernel's effective
        cap (e.g. when PREEMPT_RT kernels relax checks for sudo users).

        Printing to stdout in addition to logger.info() ensures the message
        is visible even if this function runs before setup_logging() attaches
        a file handler — useful during process startup before the logger
        is fully wired.
    """
    pid = os.getpid()

    try:
        psutil.Process(pid).cpu_affinity([cpu_core])
        os.sched_setscheduler(pid, os.SCHED_FIFO, os.sched_param(priority))
    except Exception as e:
        logger.warning(f"Failed to set real-time parameters: {e}")
        logger.warning("Running without real-time guarantees")
        print(f"[RT setup] FAILED: {e}")

    # Verify what actually applied — single source of truth post-call
    actual_policy = os.sched_getscheduler(pid)
    actual_prio   = os.sched_getparam(pid).sched_priority
    actual_aff    = sorted(psutil.Process(pid).cpu_affinity())
    policy_name   = {
        os.SCHED_FIFO:  "FIFO",
        os.SCHED_RR:    "RR",
        os.SCHED_OTHER: "OTHER",
    }.get(actual_policy, f"policy={actual_policy}")

    summary = f"SCHED_{policy_name} prio={actual_prio} affinity={actual_aff}"
    if actual_policy == os.SCHED_FIFO and actual_prio == priority and actual_aff == [cpu_core]:
        logger.info(f"[RT setup] OK — {summary}")
        print(f"[RT setup] OK — {summary}")
    else:
        logger.warning(f"[RT setup] DEGRADED — wanted SCHED_FIFO {priority} on core {cpu_core}, got {summary}")
        print(f"[RT setup] DEGRADED — wanted SCHED_FIFO {priority} on core {cpu_core}, got {summary}")


def get_system_info():
    """
    Collect comprehensive system information.
    
    Returns:
        dict: System information including platform, distro, and hardware details
    """
    info = {}
    
    try:
        # Platform information
        info['platform_release'] = platform.release()
        info['kernel_version'] = platform.version()
        
        # Linux distribution
        info['distro_name'] = distro.name()
        info['distro_version'] = distro.version()
        
        # Raspberry Pi specific
        try:
            pi = pi_info()
            info['pi_model'] = pi.model
            info['pi_revision'] = pi.revision
        except Exception:
            info['pi_model'] = "Unknown"
            info['pi_revision'] = "Unknown"
        
        # CPU information
        info['cpu_count'] = psutil.cpu_count(logical=True)
        info['cpu_architecture'] = platform.machine()
        
        # Memory information
        mem = psutil.virtual_memory()
        info['memory_total_gb'] = mem.total / (1024**3)
        info['memory_available_gb'] = mem.available / (1024**3)
        
    except Exception as e:
        logger.warning(f"Error collecting system info: {e}")
    
    return info


# ARM CPU part IDs -> human name. /proc/cpuinfo carries no "model name" line on aarch64, so a Pi
# would otherwise report its processor as "aarch64" — identical to the architecture field, and
# therefore useless. Falls back to the raw part id for anything not listed.
_ARM_CPU_PARTS = {
    "0xd03": "Cortex-A53",   # Pi 3
    "0xd08": "Cortex-A72",   # Pi 4
    "0xd0b": "Cortex-A76",   # Pi 5
}


def get_cpu_processor():
    """Human-readable CPU name. Prefers /proc/cpuinfo's "model name" (x86 and friends), then
    decodes the ARM "CPU part" id, and only then falls back to platform.processor()."""
    model_name = part = None
    try:
        with open("/proc/cpuinfo") as f:
            for line in f:
                key, _, value = line.partition(":")
                key, value = key.strip(), value.strip()
                if key == "model name" and not model_name:
                    model_name = value
                elif key == "CPU part" and not part:
                    part = value.lower()
    except OSError:
        pass
    if model_name:
        return model_name
    if part:
        return _ARM_CPU_PARTS.get(part, f"ARM part {part}")
    return platform.processor() or platform.machine() or "Unknown"


def _root_block_device(path="/"):
    """(partition, base_disk) backing `path` — e.g. ('mmcblk0p2', 'mmcblk0'). (None, None) if
    it cannot be resolved."""
    device = None
    for part in psutil.disk_partitions(all=False):
        if part.mountpoint == path:
            device = part.device
            break
    if not device:
        return None, None
    name = os.path.basename(device)                 # mmcblk0p2 / nvme0n1p2 / sda2
    # mmcblk/nvme put a 'p' before the partition number; sd*/vd* do not.
    match = re.match(r"^((?:mmcblk\d+|nvme\d+n\d+))p\d+$", name)
    if match:
        return name, match.group(1)
    return name, re.sub(r"\d+$", "", name)          # sda2 -> sda


def get_disk_device(path="/"):
    """Human-readable description of the disk backing `path`.

    Reports the media kind plus whatever model string the kernel exposes: SD/MMC cards publish
    their CID product name at /sys/block/<dev>/device/name (e.g. "SD16G"), while NVMe/USB/SATA
    use .../device/model. Falls back to just the kind + device node.
    """
    _part, base = _root_block_device(path)
    if not base:
        return "Unknown"
    kind = ("SD card" if base.startswith("mmcblk")
            else "NVMe SSD" if base.startswith("nvme")
            else "USB/SATA" if base.startswith("sd")
            else "disk")
    for attr in ("model", "name"):                  # NVMe/USB expose 'model', SD/MMC 'name'
        try:
            with open(f"/sys/block/{base}/device/{attr}") as f:
                model = f.read().strip()
            if model:
                return f"{model} ({kind}, {base})"
        except OSError:
            continue
    return f"{kind} ({base})"


def _publish(shared_data, name, value):
    """Write one system-info SharedString, if it exists."""
    shm = shared_data.system_info_shms.get(name)
    if shm is not None:
        shm.set(str(value))


def publish_static_system_info(shared_data):
    """Publish the system-info fields that never change (platform, distro, CPU).

    The GUI is the producer: it runs on the box at normal priority and already uses psutil, so
    this work must never land in RTI's RT loop. Publishing into shared memory rather than reading
    locally keeps the values available to any other consumer (telemetry, a remote dashboard).
    Call once at startup — none of these can change without a reboot.
    """
    info = get_system_info()
    for name in ("pi_model", "platform_release", "kernel_version",
                 "distro_name", "distro_version", "cpu_architecture", "cpu_count"):
        _publish(shared_data, name, info.get(name, "Unknown"))
    # get_system_info() does not collect these — see get_cpu_processor() / get_disk_device().
    _publish(shared_data, "cpu_processor", get_cpu_processor())
    # Which disk we boot from cannot change without a remount/reboot, so it belongs here rather
    # than in the periodic refresh.
    _publish(shared_data, "disk_device", get_disk_device())


def publish_memory_info(shared_data):
    """Publish memory totals/usage. Cheap (one psutil call) and slow-moving, so an occasional
    refresh is plenty — no need to run it at the GUI's table rate.

    Note the naming mismatch this bridges: get_system_info() reports memory_total_gb /
    memory_available_gb, while the shared strings (and the GUI) want memory_size / memory_used /
    memory_usage. Available is not used, so this computes the values the consumer actually wants.
    """
    mem = psutil.virtual_memory()
    _publish(shared_data, "memory_size",  f"{mem.total / (1024**3):.2f} GB")
    _publish(shared_data, "memory_used",  f"{mem.used / (1024**3):.2f} GB")
    _publish(shared_data, "memory_usage", f"{mem.percent:.1f} %")


def publish_disk_info(shared_data, path="/"):
    """Publish disk totals/usage for the filesystem `path` lives on.

    Moves even more slowly than memory, so it rides the same occasional refresh — one statvfs
    call. Worth having on screen: on a Pi this is the SD card, and a full root filesystem takes
    the logs and the runtime down with it long before anything else complains.
    """
    disk = psutil.disk_usage(path)
    _publish(shared_data, "disk_size",  f"{disk.total / (1024**3):.1f} GB")
    _publish(shared_data, "disk_used",  f"{disk.used / (1024**3):.1f} GB")
    _publish(shared_data, "disk_usage", f"{disk.percent:.1f} %")


def print_system_info():
    """Print formatted system information to console."""
    print("\n" + "="*70)
    print("SYSTEM INFORMATION")
    print("="*70)
    
    info = get_system_info()
    
    print(f"Platform Release:    {info.get('platform_release', 'Unknown')}")
    print(f"Kernel Version:      {info.get('kernel_version', 'Unknown')}")
    print(f"Distro Name:         {info.get('distro_name', 'Unknown')}")
    print(f"Distro Version:      {info.get('distro_version', 'Unknown')}")
    print(f"Pi Model:            {info.get('pi_model', 'Unknown')}")
    print(f"Pi Revision:         {info.get('pi_revision', 'Unknown')}")
    print(f"CPU Count:           {info.get('cpu_count', 'Unknown')}")
    print(f"CPU Architecture:    {info.get('cpu_architecture', 'Unknown')}")
    print(f"Memory Total:        {info.get('memory_total_gb', 0):.2f} GB")
    print(f"Memory Available:    {info.get('memory_available_gb', 0):.2f} GB")
    
    print("="*70 + "\n")


def setup_numpy_printing():
    """Configure NumPy print options for consistent output formatting."""
    np.set_printoptions(suppress=True, precision=7, floatmode='fixed')
    logger.info("NumPy print options configured")


def initialize_system(cpu_core=3, rt_priority=99, verbose=True):
    """
    Complete system initialization for RTI.
    
    Args:
        cpu_core (int): CPU core for affinity (default: 3)
        rt_priority (int): Real-time priority (default: 99)
        verbose (bool): Print system information (default: True)
    """
    if verbose:
        print_system_info()
    
    setup_realtime_process(cpu_core, rt_priority)
    setup_numpy_printing()
    
    if verbose:
        logger.info("System initialization complete")


# ==========================================================================
# CAN Interface Management — canonical implementation in hardware/can_hardware.py
# ==========================================================================
from hardware.can_hardware import (
    is_can_interface_up,
    start_can_interface,
    stop_can_interface,
)


# ==========================================================================
# Logging Setup – use logger_setup.setup_logging() instead
# ==========================================================================
# Duplicate removed: this module no longer defines setup_logging().
# Import from logger_setup:  from logger_setup import setup_logging



if __name__ == "__main__":
    """Test system configuration"""
    print("System Configuration Test")
    print("-" * 50)
    
    # Print system info
    print_system_info()
    
    # Get system info as dict
    info = get_system_info()
    print("\nSystem Info Dictionary:")
    for key, value in info.items():
        print(f"  {key}: {value}")
    
    print("\nNote: Real-time setup requires sudo privileges")
    print("Run with sudo to test RT features")
