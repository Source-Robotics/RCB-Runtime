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
          ALL managed processes. Scans psutil, matches each by a command-line
          token, and reads LIVE kernel state (scheduler policy, priority,
          affinity, CPU%, RSS). Reads only — never changes any process.

      thread_info(pid) / kernel_threads(prefix, contains) / core_loads() /
      user_session_cores() / plan_problems(...) - READERS for the dashboard's
          threads and core-plan view (2026-10-13): every thread of a process
          (name, policy, priority, allowed cores, last core, CPU%), kernel
          threads by name (the CAN chip's interrupt threads), each core's load,
          where systemd puts your own sessions, and what breaks
          config/cpu_cores.py's plan.

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
        # Tokens with whitespace are not file names: the supervisor starts every program through a shell
        # (`/bin/sh -c "python3 .../RTI.py"`), whose one argument would otherwise match first (a lower pid) — the
        # table then showed the sleeping shell: 1 MB, core 0, normal priority, never changing (fixed 2026-10-13).
        basenames = {os.path.basename(a) for a in argv if not any(ch.isspace() for ch in a)}
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
                         "cpu": None, "mem_mb": None, "nice": None, "threads": None})
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
                "nice": p.nice(),
                "threads": p.num_threads(),
            })
        except (psutil.NoSuchProcess, psutil.AccessDenied, OSError):
            _PROC_CACHE.pop(pid, None)   # stale — drop so a new object is made next time
            rows.append({"name": name, "running": False, "pid": pid,
                         "cores": "—", "sched": "—", "prio": "—",
                         "cpu": None, "mem_mb": None, "nice": None, "threads": None})
    return rows


# ==========================================================================
# READERS — threads, kernel threads, cores (the dashboard's core-plan view, 2026-10-13)
# ==========================================================================
_TICK_HZ = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100
_THREAD_CPU = {}          # (pid, tid) -> (cpu ticks, time.monotonic()) of the previous call: CPU% between calls


def _read_stat(path):
    """(name, last core, utime + stime in clock ticks) from a /proc/.../stat file, or None if it is gone. The name
    sits in parentheses and may contain spaces, so the fields are counted from the last ')'."""
    try:
        with open(path) as fh:
            raw = fh.read()
    except OSError:
        return None
    close = raw.rfind(")")
    name = raw[raw.find("(") + 1:close]
    rest = raw[close + 2:].split()          # rest[0] = field 3 (state)
    try:
        return name, int(rest[36]), int(rest[11]) + int(rest[12])   # field 39 processor; fields 14 + 15
    except (IndexError, ValueError):
        return None


def _sched(tid):
    """(policy label, rt priority, allowed cores) of one thread, or None if it is gone."""
    try:
        return (policy_name(os.sched_getscheduler(tid)), os.sched_getparam(tid).sched_priority,
                tuple(sorted(os.sched_getaffinity(tid))))
    except OSError:
        return None


def _cpu_percent(key, ticks, now):
    prev = _THREAD_CPU.get(key)
    _THREAD_CPU[key] = (ticks, now)
    if prev is None or now <= prev[1]:
        return None
    return 100.0 * (ticks - prev[0]) / _TICK_HZ / (now - prev[1])


def thread_info(pid, proc_root="/proc"):
    """Every thread of process `pid`: [{tid, name, sched, prio, cores, last_core, cpu}], cpu in % of one core since the
    previous call (None on the first). Threads that end while being read are skipped. Reads only."""
    import time
    now = time.monotonic()
    out = []
    try:
        tids = sorted(int(t) for t in os.listdir(f"{proc_root}/{pid}/task"))
    except OSError:
        return out
    for tid in tids:
        st = _read_stat(f"{proc_root}/{pid}/task/{tid}/stat")
        sc = _sched(tid)
        if st is None or sc is None:
            continue
        name, last, ticks = st
        out.append({"tid": tid, "name": name, "sched": sc[0], "prio": sc[1], "cores": sc[2], "last_core": last,
                    "cpu": _cpu_percent((pid, tid), ticks, now)})
    live = {(pid, t["tid"]) for t in out}
    for key in [k for k in _THREAD_CPU if k[0] == pid and k not in live]:
        del _THREAD_CPU[key]
    return out


def task_info(pid, proc_root="/proc"):
    """One task (a kernel thread such as an interrupt thread, or a process's main thread) by pid: {pid, name, sched,
    prio, cores, last_core, cpu}, cpu in % of one core since the previous call (None on the first); None if gone."""
    import time
    st = _read_stat(f"{proc_root}/{pid}/stat")
    sc = _sched(pid)
    if st is None or sc is None:
        return None
    return {"pid": pid, "name": st[0], "sched": sc[0], "prio": sc[1], "cores": sc[2], "last_core": st[1],
            "cpu": _cpu_percent((pid, pid), st[2], time.monotonic())}


def core_loads():
    """Each core's load in % since the previous call (psutil; the first call reads 0)."""
    return psutil.cpu_percent(interval=None, percpu=True)


def user_session_cores(sys_root="/sys"):
    """The cores systemd gives your own sessions (SSH, VS Code, scripts — user.slice), as RTI set them at its start
    (hardware/can_hardware.place_can_path), e.g. "0-1"; None if unknown."""
    try:
        with open(os.path.join(sys_root, "fs", "cgroup", "user.slice", "cpuset.cpus.effective")) as fh:
            return fh.read().strip() or None
    except OSError:
        return None


def thread_plan(process, is_main, thread, rt_core, can_core, general, rt_prio=99):
    """'' when one thread (a thread_info() row) follows config/cpu_cores.py's plan, else the reason. RTI ("RTI"): its
    main thread SCHED_FIFO `rt_prio` on `rt_core` alone, its other threads on `rt_core` alone or on the general cores.
    Every other process: normal scheduling on the general cores. Nobody may run on the CAN core (its interrupt threads
    are not process threads)."""
    cores = set(thread["cores"])
    gen = set(general)
    if can_core in cores:
        return f"may run on the CAN core {can_core}"
    if process == "RTI":
        if is_main:
            if thread["sched"] != "FIFO" or thread["prio"] != rt_prio or cores != {rt_core}:
                return (f"{thread['sched']} {thread['prio']} on cores {sorted(cores)} (want FIFO {rt_prio} on core "
                        f"{rt_core} alone)")
            return ""
        if cores != {rt_core} and not cores <= gen:
            return f"on cores {sorted(cores)} (want core {rt_core} alone or {sorted(gen)})"
        return ""
    if thread["sched"] != "OTHER":
        return f"{thread['sched']} {thread['prio']} (want normal scheduling)"
    if not cores <= gen:
        return f"on cores {sorted(cores)} (want {sorted(gen)})"
    return ""


def plan_problems(process, pid, threads, rt_core, can_core, general, rt_prio=99):
    """Every thread of one process that breaks the plan: ['<name> (<tid>): <reason>', ...]; [] = as planned."""
    out = []
    for t in threads:
        why = thread_plan(process, t["tid"] == pid, t, rt_core, can_core, general, rt_prio)
        if why:
            out.append(f"{t['name']} ({t['tid']}): {why}")
    return out
