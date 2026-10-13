"""Simulated-robot mode (TODO/simulated_robot_plan.md, 2026-10-13; the user's choices: an environment variable, this
Pi with the robot service stopped, no supervisor).

RCB_SIM=1 in a process's environment: RTI talks to the simulated drives on the virtual bus `vcan0` (sim/drives.py)
instead of the arm on `can0`, takes no real-time core or priority, leaves the real CAN chip alone, and every file the
runtime writes (logs, incident dumps, the homing arm table) goes to temp_data/sim/ (project_paths.SIMULATION). Read
once, at start — no per-tick cost. tools/sim_run.py sets it for every process of a simulated run.
"""
from __future__ import annotations

import os

SIM_ENV = "RCB_SIM"
SIM_CAN_INTERFACE = "vcan0"
REAL_CAN_INTERFACE = "can0"

VCAN_HELP = ("sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up "
             "(once per boot)")


def enabled(environ=None):
    return (os.environ if environ is None else environ).get(SIM_ENV, "") == "1"


def can_interface(environ=None):
    return SIM_CAN_INTERFACE if enabled(environ) else REAL_CAN_INTERFACE


def interface_up(name, sys_root="/sys"):
    """True when the network interface exists and is up (/sys/class/net/<name>/operstate is up / unknown — a vcan
    reports 'unknown' while up)."""
    try:
        with open(os.path.join(sys_root, "class", "net", name, "flags")) as fh:
            return bool(int(fh.read().strip(), 16) & 0x1)          # IFF_UP
    except (OSError, ValueError):
        return False


def other_rti_pids(proc_root="/proc", self_pid=None):
    """PIDs of other running RTI.py processes (a token ending in RTI.py with no whitespace — not a launcher shell)."""
    me = os.getpid() if self_pid is None else self_pid
    out = []
    try:
        names = os.listdir(proc_root)
    except OSError:
        return out
    for name in names:
        if not name.isdigit() or int(name) == me:
            continue
        try:
            with open(os.path.join(proc_root, name, "cmdline"), "rb") as fh:
                argv = fh.read().split(b"\0")
        except OSError:
            continue
        if any(a.endswith(b"RTI.py") and not any(c in a for c in (b" ", b"\t")) for a in argv):
            out.append(int(name))
    return sorted(out)


def refuse_reason(sys_root="/sys", proc_root="/proc", self_pid=None):
    """Why a simulated RTI must not start here, or None."""
    others = other_rti_pids(proc_root, self_pid)
    if others:
        return (f"another RTI is running (pid {others}) — the robot service? Stop it first: "
                f"sudo systemctl stop autorun.service (the arm goes limp: park or support it)")
    if not interface_up(SIM_CAN_INTERFACE, sys_root):
        return f"the virtual CAN bus {SIM_CAN_INTERFACE} is not up — {VCAN_HELP}"
    return None
