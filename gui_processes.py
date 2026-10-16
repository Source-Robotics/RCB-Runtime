"""Processes, threads and the core plan — a card in the dashboard's System Info tab (2026-10-13).

User: the Processes table "is not changing at all" (it showed the supervisor's launcher shells, not the programs — see
utility/process_utils.process_info), the new processes and threads should be in it, and the core setup — what is
pinned where, at what priority — too. Everything here READS kernel state (/proc, sched_getaffinity, sched_getscheduler);
nothing is changed. The plan it checks against is config/cpu_cores.py: RTI on core 3 (SCHED_FIFO 99), the CAN chip's
interrupt threads on core 2 alone, everything else (and your own sessions — systemd's user.slice) on cores 0-1.

Refreshed by the dashboard's 1 Hz timer only while the System Info tab is open; the threads table only while its
expansion is open (~40 threads, a few hundred small /proc reads — on the GUI's own core, never RTI's).
"""
from __future__ import annotations

import os
import time

from nicegui import ui

from config.cpu_cores import CAN_CPU_CORE, GENERAL_CPU_CORES, RT_CPU_CORE
from hardware.can_hardware import can_path_tasks
from utility import process_utils as pu

# (display name, the command-line token that identifies it, started on demand)
PROCESSES = (
    ("RTI", "RTI.py", False),
    ("Self-collision worker", "self_collision_worker.py", False),
    ("Executor", "command_executor.py", False),
    ("GUI", "nicegui_test.py", False),
    ("Supervisor", "supervisor.py", False),
    ("Recorder", "recorder.py", False),                  # incident dumps + captures (2026-10-16)
    ("Block program", "blockly.runner", True),
    ("Firmware flasher", "flash_motor_firmware.py", True),
)
_PATTERNS = {name: token for name, token, _ in PROCESSES}
_ON_DEMAND = {name for name, _, on_demand in PROCESSES if on_demand}

PROCESS_COLUMNS = [
    {"name": "name", "label": "Name", "field": "name", "align": "left"},
    {"name": "pid", "label": "PID", "field": "pid"},
    {"name": "cores", "label": "Cores", "field": "cores"},
    {"name": "sched", "label": "Scheduling", "field": "sched", "align": "left"},
    {"name": "cpu", "label": "CPU%", "field": "cpu"},
    {"name": "mem", "label": "Mem", "field": "mem"},
    {"name": "threads", "label": "Threads", "field": "threads"},
    {"name": "plan", "label": "Core plan", "field": "plan", "align": "left"},
]
THREAD_COLUMNS = [
    {"name": "process", "label": "Process", "field": "process", "align": "left"},
    {"name": "tid", "label": "TID", "field": "tid"},
    {"name": "name", "label": "Thread", "field": "name", "align": "left"},
    {"name": "sched", "label": "Scheduling", "field": "sched", "align": "left"},
    {"name": "cores", "label": "Allowed cores", "field": "cores"},
    {"name": "last", "label": "Last ran on", "field": "last"},
    {"name": "cpu", "label": "CPU%", "field": "cpu"},
    {"name": "plan", "label": "Core plan", "field": "plan", "align": "left"},
]

HELP = [
    ("The core plan (config/cpu_cores.py)", True),
    (f"Core {RT_CPU_CORE}: RTI's real-time loop, SCHED_FIFO 99 — nothing else may compete with it. Its helper threads "
     f"that must answer at once (the command listener, the log writer) stay there at FIFO 99 too; ZMQ's network "
     f"threads run on the general cores.", False),
    (f"Core {CAN_CPU_CORE}: the CAN chip's and its SPI controller's interrupt threads ALONE (FIFO 50) — the CAN driver's "
     f"send work runs where they run; on RTI's core it froze the bus, on a busy core it stalled (2026-10-12).", False),
    (f"Cores {'-'.join(map(str, GENERAL_CPU_CORES))}: everything else — supervisor, GUI, executor, the self-collision "
     f"worker, block programs — and your own sessions (SSH, VS Code, scripts: systemd's user.slice, set by RTI at "
     f"its start). Normal scheduling.", False),
    ("Scheduling: FIFO n = real-time, priority n (99 highest) — runs until it sleeps; OTHER = normal, shared by "
     "'nice' (lower = more share).", False),
    ("Cores = where the thread MAY run; Last ran on = where it last did (a thread that sleeps keeps its old value "
     "after being moved). CPU% = of one core, over the last second.", False),
    ("Core plan: ✓ as planned, otherwise what is off. A Python thread is named 'python3' — the OS does not see "
     "Python's own thread names.", False),
]


def _fmt_sched(sched, prio, nice=None):
    if sched in ("FIFO", "RR"):
        return f"{sched} {prio} (real-time)"
    return f"{sched}" + (f", nice {nice}" if nice not in (None, 0) else "")


def _fmt_cores(cores):
    cores = sorted(cores)
    if len(cores) > 1 and cores == list(range(cores[0], cores[-1] + 1)):
        return f"{cores[0]}-{cores[-1]}"
    return ",".join(map(str, cores))


def _fmt_cpu(cpu):
    return "—" if cpu is None else f"{cpu:.0f}%"


_CAN = {"t": -1e9, "tasks": None}


def _can_tasks():
    """can_path_tasks(), re-read every 30 s or as soon as a cached interrupt thread is gone (they change only when can0
    is brought up again, at RTI's start) — the full scan reads every process's name (~12 ms)."""
    now = time.monotonic()
    c = _CAN
    if (c["tasks"] is None or now - c["t"] > 30.0
            or any(not os.path.exists(f"/proc/{pid}") for pid in c["tasks"]["threads"])):
        c["tasks"], c["t"] = can_path_tasks(), now
    return c["tasks"]


def collect(with_threads):
    """(process rows, thread rows, the core-plan line) — one read of the kernel state."""
    rows, trows = [], []
    for info in pu.process_info(_PATTERNS):
        name = info["name"]
        if not info["running"]:
            idle = name in _ON_DEMAND
            rows.append({"name": name, "pid": "—", "cores": "—", "sched": "not running" if idle else "NOT RUNNING",
                         "cpu": "—", "mem": "—", "threads": "—", "plan": "started on demand" if idle else "✗ not running"})
            continue
        threads = pu.thread_info(info["pid"])
        problems = pu.plan_problems(name, info["pid"], threads, RT_CPU_CORE, CAN_CPU_CORE, GENERAL_CPU_CORES)
        rows.append({
            "name": name, "pid": info["pid"], "cores": _fmt_cores(int(c) for c in str(info["cores"]).split(",")),
            "sched": _fmt_sched(info["sched"], info["prio"], info.get("nice")),
            "cpu": _fmt_cpu(info["cpu"]), "mem": f"{info['mem_mb']} MB" if info["mem_mb"] is not None else "—",
            "threads": info.get("threads") or len(threads),
            "plan": "✓" if not problems else f"✗ {problems[0]}" + (f" (+{len(problems) - 1})" if len(problems) > 1 else ""),
        })
        if with_threads:
            for t in threads:
                why = pu.thread_plan(name, t["tid"] == info["pid"], t, RT_CPU_CORE, CAN_CPU_CORE, GENERAL_CPU_CORES)
                trows.append({"process": name, "tid": t["tid"], "name": t["name"] + (" (main)" if t["tid"] == info["pid"] else ""),
                              "sched": _fmt_sched(t["sched"], t["prio"]), "cores": _fmt_cores(t["cores"]),
                              "last": t["last_core"], "cpu": _fmt_cpu(t["cpu"]), "plan": "✓" if not why else f"✗ {why}"})
    can = _can_tasks()
    for pid, (role, tname) in sorted(can["threads"].items(), key=lambda kv: kv[1][0]):
        t = pu.task_info(pid)
        if t is None:
            continue
        ok = set(t["cores"]) == {CAN_CPU_CORE}
        plan = "✓" if ok else f"✗ on cores {_fmt_cores(t['cores'])} (want core {CAN_CPU_CORE} alone)"
        rows.append({"name": f"{role} ({tname})", "pid": pid, "cores": _fmt_cores(t["cores"]),
                     "sched": _fmt_sched(t["sched"], t["prio"]), "cpu": _fmt_cpu(t["cpu"]), "mem": "—", "threads": 1,
                     "plan": plan})
        if with_threads:
            trows.append({"process": "kernel", "tid": pid, "name": tname, "sched": _fmt_sched(t["sched"], t["prio"]),
                          "cores": _fmt_cores(t["cores"]), "last": t["last_core"], "cpu": _fmt_cpu(t["cpu"]),
                          "plan": plan})
    loads = pu.core_loads()
    user = pu.user_session_cores()
    line = (f"Core {RT_CPU_CORE}: RTI (real-time) · core {CAN_CPU_CORE}: CAN interrupts · cores "
            f"{_fmt_cores(GENERAL_CPU_CORES)}: everything else · your sessions (user.slice): {user or 'unknown'}"
            f" · SPI power: {can['power'] or 'unknown'}")
    load_line = "Load now: " + " · ".join(f"core {i} {v:.0f}%" for i, v in enumerate(loads))
    return rows, trows, line, load_line


def add_processes_card():
    """Build the card (call inside the System Info tab's panel). Returns refresh() -> the process rows (the dashboard's
    TEMP diagnostic copy on the Plots tab shows the same rows)."""
    with ui.card().classes("w-full"):
        with ui.row().classes("items-center gap-2"):
            ui.label("Processes & cores").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
            with ui.icon("help_outline").classes("text-gray-400 cursor-help text-sm"):
                with ui.tooltip().classes("max-w-md"), ui.column().classes("gap-1 py-1"):
                    for text, bold in HELP:
                        lbl = ui.label(text)
                        if bold:
                            lbl.classes("font-semibold")
        plan_lbl = ui.label("—").classes("text-xs text-gray-600")
        load_lbl = ui.label("—").classes("text-xs text-gray-600 font-mono")
        table = ui.table(columns=PROCESS_COLUMNS, rows=[], row_key="name").classes("w-full").props("dense flat")
        with ui.expansion("Threads — every thread of these processes and the CAN interrupt threads").classes("w-full") as exp:
            ttable = ui.table(columns=THREAD_COLUMNS, rows=[], row_key="tid").classes("w-full").props("dense flat")

    def refresh():
        rows, trows, line, load_line = collect(with_threads=bool(exp.value))
        plan_lbl.text = line
        load_lbl.text = load_line
        table.rows = rows
        table.update()
        if exp.value:
            ttable.rows = trows
            ttable.update()
        return rows

    return refresh


def process_rows():
    """The process rows alone (for the TEMP diagnostic copy on the Plots tab)."""
    return collect(with_threads=False)[0]
