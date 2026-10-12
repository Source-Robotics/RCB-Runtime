#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: utility/process_utils.py
Description:
    Two helpers around process CPU affinity / scheduling:

      setup_process(core, rt, priority)  - SETTER. A process calls this ONCE at
          startup to pin itself to a core and (optionally) request SCHED_FIFO.
          Reads back the actual kernel state and logs OK/DEGRADED, so a silently
          rejected RT request is always visible. Only RTI should pass rt=True.

      process_info(patterns)             - READER. The GUI calls this to inspect
          ALL managed processes. Scans psutil, matches each by a substring of its
          command line, and reads LIVE kernel state (scheduler policy, priority,
          affinity, CPU%, RSS). Reads only — never changes any process.

    Single source of truth for "what core / policy / priority is this process on",
    used both to apply the layout and to display it in the dashboard.
"""

from __future__ import annotations

import os
import logging

import psutil

logger = logging.getLogger(__name__)

_POLICY_NAME = {
    os.SCHED_FIFO:  "FIFO",
    os.SCHED_RR:    "RR",
    os.SCHED_OTHER: "OTHER",
}

# pid -> psutil.Process, kept across process_info() calls so cpu_percent() can
# measure usage between successive calls (a fresh object always reports 0.0).
_PROC_CACHE = {}


def policy_name(policy):
    """Human label for an os.SCHED_* constant."""
    return _POLICY_NAME.get(policy, f"policy={policy}")


# ==========================================================================
# SETTER — called once per process at startup
# ==========================================================================
def setup_process(core, rt: bool = False, priority: int = 0) -> bool:
    """
    Pin this process to `core` — one core, or several (an iterable, e.g. config/cpu_cores.GENERAL_CPU_CORES).
    If rt=True, also request SCHED_FIFO at `priority` (1-99); otherwise leave normal (SCHED_OTHER) scheduling.

    Only RTI.py should use rt=True — every other process is non-real-time and
    just gets a dedicated core so it never lands on RTI's.

    Always reads back and logs the ACTUAL applied state (OK vs DEGRADED), because
    sched_setscheduler can be silently rejected by rlimit/capability checks.

    The affinity goes to EVERY thread the process already has, not just the caller (2026-10-12): Linux keeps an
    affinity per thread, and threads that libraries start while the process imports (before this call) kept all four
    cores — the supervisor had two waking ~2600 and ~80 times a second, half the time on the CAN core (2) or RTI's (3).
    Threads started after this call inherit the caller's affinity.
    """
    pid = os.getpid()
    cores = [core] if isinstance(core, int) else sorted(core)

    try:
        psutil.Process(pid).cpu_affinity(cores)
        if rt:
            os.sched_setscheduler(pid, os.SCHED_FIFO, os.sched_param(priority))
    except Exception as exc:
        logger.warning(f"[proc setup] FAILED to set affinity/scheduling: {exc}")
        print(f"[proc setup] FAILED: {exc}")
    elsewhere = pin_all_threads(cores)

    # Verify what actually applied — single source of truth post-call.
    actual_policy = os.sched_getscheduler(pid)
    actual_prio = os.sched_getparam(pid).sched_priority
    actual_aff = sorted(psutil.Process(pid).cpu_affinity())
    summary = f"SCHED_{policy_name(actual_policy)} prio={actual_prio} affinity={actual_aff}"
    if elsewhere:
        summary += f"; {elsewhere} other thread(s) NOT on {cores}"

    want_policy = os.SCHED_FIFO if rt else os.SCHED_OTHER
    want_prio = priority if rt else 0
    ok = (actual_policy == want_policy and actual_prio == want_prio and actual_aff == cores and not elsewhere)

    if ok:
        logger.info(f"[proc setup] OK — {summary}")
        print(f"[proc setup] OK — {summary}")
    else:
        want = f"SCHED_{policy_name(want_policy)} prio={want_prio} on core {core}"
        logger.warning(f"[proc setup] DEGRADED — wanted {want}, got {summary}")
        print(f"[proc setup] DEGRADED — wanted {want}, got {summary}")
    return ok


def pin_all_threads(cores, task_dir="/proc/self/task", set_affinity=os.sched_setaffinity,
                    get_affinity=os.sched_getaffinity) -> int:
    """Give every thread of this process the affinity `cores`, then read each back. Returns how many threads are
    still allowed elsewhere (0 = all pinned). A thread that ends meanwhile is skipped."""
    want = set(cores)
    elsewhere = 0
    for name in os.listdir(task_dir):
        tid = int(name)
        try:
            set_affinity(tid, want)
            if set(get_affinity(tid)) != want:
                elsewhere += 1
        except ProcessLookupError:
            continue                     # the thread ended between the listing and now
        except OSError as exc:
            logger.warning(f"[proc setup] thread {tid}: cannot set its affinity: {exc}")
            elsewhere += 1
    return elsewhere


# ==========================================================================
# READER — called by the GUI to inspect all processes
# ==========================================================================
def process_info(patterns: dict[str, str]) -> list[dict]:
    """
    Inspect managed processes for the dashboard.

    Args:
        patterns: dict {display_name: cmdline_substring}, e.g.
            {"RTI": "RTI.py", "GUI": "nicegui_test.py", ...}

    Returns:
        list of dicts (one per pattern, in insertion order):
            {name, running, pid, cores, sched, prio, cpu, mem_mb}
        Fields are "—"/None when the process isn't found. Reads live kernel
        state; never modifies any process.

    Note on cpu: psutil.Process.cpu_percent() reports usage *since the previous
    call on the same object*, so the Process objects are cached across calls in
    _PROC_CACHE. The first reading for a freshly-seen pid is 0.0 (no baseline);
    subsequent calls (e.g. the GUI's 1 Hz timer) report real values.
    """
    # One pass over the process table. Match a pattern only when the script name
    # is an actual command-line *token* whose basename matches (e.g. arg
    # ".../RTI.py" -> "RTI.py"). Matching tokens — not the whole joined string —
    # avoids false hits from a process that merely mentions another script's name
    # in its arguments (e.g. `python3 -c "...RTI.py..."`). The GUI still matches
    # itself correctly, because its argv genuinely contains `nicegui_test.py`.
    found = {}
    for proc in psutil.process_iter(["pid", "cmdline"]):
        try:
            argv = proc.info["cmdline"] or []
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
        if not argv:
            continue
        basenames = {os.path.basename(a) for a in argv}
        for name, needle in patterns.items():
            if name not in found and needle in basenames:
                found[name] = proc.info["pid"]

    # Drop cache entries whose pid is no longer one of the current matches
    # (e.g. a process was restarted and now has a different pid).
    live_pids = set(found.values())
    for pid in list(_PROC_CACHE):
        if pid not in live_pids:
            del _PROC_CACHE[pid]

    rows = []
    for name in patterns:
        pid = found.get(name)
        if pid is None:
            rows.append({"name": name, "running": False, "pid": None,
                         "cores": "—", "sched": "—", "prio": "—",
                         "cpu": None, "mem_mb": None})
            continue
        try:
            # Reuse the cached Process so cpu_percent() can diff against the
            # previous call; create+remember it the first time we see this pid.
            p = _PROC_CACHE.get(pid)
            if p is None:
                p = psutil.Process(pid)
                _PROC_CACHE[pid] = p
            cores = sorted(p.cpu_affinity())
            rows.append({
                "name": name,
                "running": True,
                "pid": pid,
                "cores": ",".join(map(str, cores)) if cores else "—",
                "sched": policy_name(os.sched_getscheduler(pid)),
                "prio": os.sched_getparam(pid).sched_priority,
                "cpu": p.cpu_percent(None),          # non-blocking; needs prior baseline
                "mem_mb": round(p.memory_info().rss / (1024 * 1024)),
            })
        except (psutil.NoSuchProcess, psutil.AccessDenied, OSError):
            _PROC_CACHE.pop(pid, None)   # stale — drop so a new object is made next time
            rows.append({"name": name, "running": False, "pid": pid,
                         "cores": "—", "sched": "—", "prio": "—",
                         "cpu": None, "mem_mb": None})
    return rows
