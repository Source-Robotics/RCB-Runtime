#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: process_manager.py
Description: Script lifecycle management for supervisor.
    Handles starting, stopping, restarting, and monitoring of managed scripts.

    Key robustness rules (learned the hard way):
    - Each script runs in its OWN session/process group (start_new_session=True)
      so we can kill the whole tree on stop/restart. Without this, a restart can
      leave the old process alive holding its port -> the new one fails to bind
      (e.g. GUI: "Errno 98 Address already in use") -> crash/restart spiral.
    - Child stdout/stderr go to logs/<NAME>.out.log. Previously they went to
      PIPE and were never read, so crash tracebacks were silently swallowed
      (and a chatty child could deadlock when the 64 KB pipe buffer filled).
    - kill_stale() clears leftovers from a previous supervisor run on boot.
    - <NAME>.out.log is rotated when its script is (re)started and the file is over
      OUT_LOG_MAX_BYTES (never while a child holds it open). RTI.out.log had reached
      240 MB in three months with no rotation at all (2026-10-01).
"""

import os
import shlex
import signal
import subprocess
import psutil
import logging
import time
from project_paths import PROJECT_ROOT, TEMP_DATA_DIR
import shutil
from utility.housekeeping import (DISK_LOW_BYTES, DISK_OK_BYTES, RECORDINGS_BUDGET_BYTES, copy_truncate, enforce_budget,
                                  rotate_by_size)
from utility.system_temperature import get_raspberry_pi_temperature

logger = logging.getLogger("sup.process")  # docs/logging.md "Logger names"

LOG_DIR = PROJECT_ROOT / "logs"
_TERM_TIMEOUT = 5  # seconds to wait for graceful exit before SIGKILL
OUT_LOG_MAX_BYTES = 20 * 1024 * 1024   # rotate <name>.out.log at (re)start when it is bigger than this
OUT_LOG_KEEP = 3                        # ...keeping <name>.out.log.1 .. .3
_temp_unreadable = False                # the temperature warning is logged once per failure streak
BUDGET_CHECK_S = 3600.0                 # the test recordings' budget is checked once an hour (2026-10-15)
_last_budget_check = [float("-inf")]    # monotonic time of the last check (the first monitor pass checks)


# ==========================================================================
# Low-level process helpers
# ==========================================================================
def _rotate_out_log(path, max_bytes=OUT_LOG_MAX_BYTES, keep=OUT_LOG_KEEP):
    """<name>.out.log -> .1 -> .2 ... (the oldest dropped) when it is over max_bytes. Called only before a
    script starts, so no running child is writing to the file being renamed (utility/housekeeping.py)."""
    if rotate_by_size(str(path), max_bytes, keep):
        logger.info(f"Rotated {path.name} (was over {max_bytes // (1024 * 1024)} MB)")


def _cap_running_out_logs(processes, max_bytes=OUT_LOG_MAX_BYTES, keep=OUT_LOG_KEEP):
    """A RUNNING child's <name>.out.log over max_bytes is copied to .1 and truncated (2026-10-15): until then it grew
    for as long as the child ran — RTI runs for weeks. Checked every monitor pass; a stat per file."""
    for name in processes:
        path = LOG_DIR / f"{name}.out.log"
        if copy_truncate(str(path), max_bytes, keep):
            logger.info(f"Rotated {path.name} while {name} runs (was over {max_bytes // (1024 * 1024)} MB)")


DISK_CHECK_S = 30.0                     # the free space is checked every 30 s (review I6)
_last_disk_check = [float("-inf")]
_disk_low = [False]


def _check_disk(now=None, root=PROJECT_ROOT, flag=None, free_bytes=None):
    """Every DISK_CHECK_S: the SD card's free space -> rti['disk_low'] [1 = low, free MB], which RTI's error pass shows
    as DISK_LOW_WARNING (review I6, 2026-11-20) — under DISK_LOW_BYTES, cleared above DISK_OK_BYTES; logged when it
    changes. Returns low / not, or None when not checked. Never raises."""
    now = time.monotonic() if now is None else now
    if now - _last_disk_check[0] < DISK_CHECK_S:
        return None
    _last_disk_check[0] = now
    try:
        free = shutil.disk_usage(str(root)).free if free_bytes is None else int(free_bytes)
        if flag is None:
            from data import shared_data
            flag = shared_data.rti["disk_low"]
    except (OSError, KeyError, AttributeError) as exc:
        logger.warning(f"free disk space not checked: {exc}")
        return None
    low = free < DISK_LOW_BYTES or (_disk_low[0] and free < DISK_OK_BYTES)
    if low != _disk_low[0]:
        _disk_low[0] = low
        if low:
            logger.warning(f"the SD card has {free / 1e6:.0f} MB free (under {DISK_LOW_BYTES // 1_000_000} MB): "
                           f"DISK_LOW_WARNING — free space (tools/disk_cleanup.py) before logs and saves fail")
        else:
            logger.info(f"the SD card has {free / 1e6:.0f} MB free again: DISK_LOW_WARNING cleared")
    flag[0], flag[1] = (1.0 if low else 0.0), free / 1e6
    return low


def _check_recordings_budget(now=None, root=TEMP_DATA_DIR, budget=RECORDINGS_BUDGET_BYTES):
    """Once an hour (the first monitor pass too): the test recordings in temp_data together at most `budget`
    (500 MB, utility/housekeeping.RECORDINGS_BUDGET_BYTES), the oldest runs deleted first
    (utility/housekeeping.enforce_budget; user, 2026-10-15). Returns the deleted run
    keys. Never raises: housekeeping must not take the supervisor down."""
    now = time.monotonic() if now is None else now
    if now - _last_budget_check[0] < BUDGET_CHECK_S:
        return []
    _last_budget_check[0] = now
    try:
        before, after, deleted = enforce_budget(root, budget)
    except Exception as exc:                      # pragma: no cover - enforce_budget does not raise
        logger.warning(f"recordings budget check failed: {exc}")
        return []
    if deleted:
        logger.info(f"Test recordings over the {budget // 1_000_000} MB budget ({before / 1e6:.0f} MB): deleted the "
                    f"oldest {len(deleted)} run(s), now {after / 1e6:.0f} MB: {', '.join(deleted[:5])}"
                    f"{' ...' if len(deleted) > 5 else ''}")
    return deleted


def _spawn(name, cmd):
    """Launch one script in its own process group, output -> logs/<name>.out.log (rotated first if large)."""
    _rotate_out_log(LOG_DIR / f"{name}.out.log")
    logf = open(LOG_DIR / f"{name}.out.log", "ab", buffering=0)
    # Review I4 (2026-11-20): the script itself, not `sh -c` — with the shell the child was the shell and Python its
    # child: a stop saw the shell exit at once and logged "stopped" while Python still shut down (two RTIs for up to
    # 10 s, the dashboard's "Address already in use"), and the SIGKILL step could never fire.
    proc = subprocess.Popen(  # nosec B603 — supervisor.py's own SCRIPTS (this Python + a repo script), no outside input
        shlex.split(str(cmd)),
        stdout=logf,
        stderr=subprocess.STDOUT,
        start_new_session=True,  # new pgid so we can kill the whole tree later
    )
    proc._logf = logf  # keep a reference so the file handle isn't GC'd
    proc._started = time.monotonic()   # review I4: a death soon after the start counts towards a crash loop
    logger.info(f"Started {name} (PID {proc.pid})")
    return proc


def _signal_group(proc, sig):
    """Send a signal to the process's whole group; fall back to the process."""
    try:
        os.killpg(os.getpgid(proc.pid), sig)
    except ProcessLookupError:
        pass  # already gone
    except Exception:
        try:
            proc.send_signal(sig)
        except Exception:
            pass


def _close_log(proc):
    logf = getattr(proc, "_logf", None)
    if logf:
        try:
            logf.close()
        except Exception:
            pass


def _terminate(name, proc):
    """Kill a process and its entire group; SIGTERM then SIGKILL. Reap it."""
    if proc.poll() is not None:
        _close_log(proc)
        return

    logger.info(f"Stopping {name} (PID {proc.pid})")
    _signal_group(proc, signal.SIGTERM)
    try:
        proc.wait(timeout=_TERM_TIMEOUT)
    except subprocess.TimeoutExpired:
        logger.warning(f"{name} did not exit on SIGTERM, sending SIGKILL")
        _signal_group(proc, signal.SIGKILL)
        try:
            proc.wait(timeout=_TERM_TIMEOUT)
        except subprocess.TimeoutExpired:
            logger.error(f"{name} survived SIGKILL (zombie?)")
    _close_log(proc)
    logger.info(f"{name} stopped")


# ==========================================================================
# Public API
# ==========================================================================
def kill_stale(SCRIPTS):
    """Kill leftover instances of managed scripts from a previous supervisor run.

    Matches a process only if its executable is python AND its command line
    contains one of the managed script paths — so it won't kill an editor that
    merely has the file open.
    """
    paths = [cmd.split()[-1] for cmd in SCRIPTS.values()]  # full .py paths
    me = os.getpid()
    for proc in psutil.process_iter(["pid", "cmdline"]):
        try:
            if proc.pid == me:
                continue
            cmd = proc.info["cmdline"] or []
            if not cmd:
                continue
            is_python = "python" in os.path.basename(cmd[0])
            joined = " ".join(cmd)
            if is_python and any(p in joined for p in paths):
                logger.warning(f"Killing a stale process from a previous run, PID {proc.pid}: {joined}")
                proc.kill()
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass


def start_scripts(SCRIPTS, processes):
    """Start all scripts and store process references."""
    logger.info("Starting all managed scripts")
    for name, cmd in SCRIPTS.items():
        processes[name] = _spawn(name, cmd)


def stop_scripts(processes):
    """Stop all running scripts safely (does not touch supervisor.py itself)."""
    logger.info("Stopping all managed scripts")
    for name, proc in list(processes.items()):
        _terminate(name, proc)
        del processes[name]
    logger.info("All managed scripts stopped")


def restart_script(name, processes, SCRIPTS):
    """Restart a stalled or crashed script — old tree is fully killed first."""
    logger.info(f"Restarting {name}")
    proc = processes.get(name)
    if proc is not None:
        _terminate(name, proc)
    processes[name] = _spawn(name, SCRIPTS[name])
    logger.info(f"Restarted {name} (PID {processes[name].pid})")


# tools/restart_gui_and_executor.sh writes "<unix time> <script> <script> ..." here before it stops those scripts (the
# gripper swap runs it): their exits within PLANNED_RESTART_S are planned (2026-11-19: they were logged as ERRORs
# "stopped unexpectedly"). In the repo's own temp_data, not TEMP_DATA_DIR (the simulator moves that).
PLANNED_RESTART_FILE = PROJECT_ROOT / "temp_data" / "planned_restart"
PLANNED_RESTART_S = 60.0


def _planned_restart(cmd, now=None, path=None):
    """True when the script `cmd` runs (its last word: the .py path) is named in a planned-restart note at most
    PLANNED_RESTART_S old. Never raises: no note, an old or unreadable one = not planned."""
    try:
        words = (path or PLANNED_RESTART_FILE).read_text().split()
        stamp = float(words[0])
    except (OSError, ValueError, IndexError):
        return False
    if not -5.0 <= (time.time() if now is None else now) - stamp <= PLANNED_RESTART_S:     # -5: whole-second stamps
        return False
    script = os.path.splitext(os.path.basename(str(cmd).split()[-1] if str(cmd).split() else ""))[0]
    return bool(script) and script in words[1:]


# Review I4 (2026-11-20): a crash loop (a bad XML: RTI dies at start) restarted the script every 2 s for ever, with one
# line each. A death within QUICK_DEATH_S of the start is a QUICK death: the first is restarted at once (as before),
# the next ones after CRASH_BACKOFF_S, and the CRASH_LOOP_DEATHS-th logs one ERROR naming the loop.
QUICK_DEATH_S = 30.0
CRASH_BACKOFF_S = (2.0, 4.0, 8.0, 16.0, 30.0, 60.0)
CRASH_LOOP_DEATHS = 5
_quick_deaths: dict = {}                  # name -> quick deaths in a row
_restart_at: dict = {}                    # name -> monotonic time of its delayed restart


def _restart_crashed(name, process, processes, SCRIPTS, now=None) -> str:
    """A script that stopped unexpectedly: restarted at once, or after its back-off. Returns its status text."""
    now = time.monotonic() if now is None else now
    if name in _restart_at:                                       # waiting out its back-off
        if now < _restart_at[name]:
            return "waiting to restart (crash loop back-off)"
        del _restart_at[name]
        restart_script(name, processes, SCRIPTS)
        return "restarted (after a back-off)"
    lived = now - getattr(process, "_started", now)
    n = _quick_deaths.get(name, 0) + 1 if lived < QUICK_DEATH_S else 0
    _quick_deaths[name] = n
    if n <= 1:
        logger.error(f"{name} stopped unexpectedly (exit {process.returncode}) — restarting it. "
                     f"See logs/{name}.out.log")
        restart_script(name, processes, SCRIPTS)
        return "restarted (crashed)"
    wait = CRASH_BACKOFF_S[min(n - 2, len(CRASH_BACKOFF_S) - 1)]
    _restart_at[name] = now + wait
    logger.error(f"{name} stopped again {lived:.1f} s after its start (exit {process.returncode}; quick death {n} in a "
                 f"row) — restarting it in {wait:g} s. See logs/{name}.out.log")
    if n == CRASH_LOOP_DEATHS:
        logger.error(f"{name} is in a CRASH LOOP: {n} deaths within {QUICK_DEATH_S:g} s of their start — it will keep "
                     f"being restarted every {CRASH_BACKOFF_S[-1]:g} s at most; logs/{name}.out.log says why (a broken "
                     f"configuration file, most often)")
    return f"crashed — restart in {wait:g} s"


def monitor_scripts(emergency_stop, processes, SCRIPTS):
    """
    Monitor running scripts, check responsiveness, and restart if necessary.
    Also reads the Raspberry Pi temperature.

    Returns:
        dict: {script_name: {"status": <state>}, ..., "temperature_C": float|None}
    """
    status_dict = {}

    # Emergency stop: kill everything and report.
    if emergency_stop:
        stop_scripts(processes)
        logger.warning("Emergency stop: all managed scripts terminated")
        for name in SCRIPTS:
            status_dict[name] = {"status": "stopped (emergency)"}
        return status_dict

    # Recover after an emergency stop was lifted.
    if not processes and not emergency_stop:
        logger.info("Emergency stop lifted — restarting all managed scripts")
        start_scripts(SCRIPTS=SCRIPTS, processes=processes)
        for name in SCRIPTS:
            status_dict[name] = {"status": "restarted after estop"}
        return status_dict

    # Normal health check.
    for name in SCRIPTS:
        process = processes.get(name)

        if process is None:
            logger.error(f"{name} is not in the process list — restarting it")
            restart_script(name, processes, SCRIPTS)
            status_dict[name] = {"status": "restarted (missing)"}
            continue

        if process.poll() is not None:
            if _planned_restart(SCRIPTS.get(name, "")):
                logger.info(f"{name} stopped as planned (exit {process.returncode}): tools/restart_gui_and_executor.sh "
                            f"restarts it (a gripper swap, or a restart by hand) — restarting it")
                restart_script(name, processes, SCRIPTS)
                status_dict[name] = {"status": "restarted (planned)"}
                continue
            status_dict[name] = {"status": _restart_crashed(name, process, processes, SCRIPTS)}
        else:
            if time.monotonic() - getattr(process, "_started", float("-inf")) > QUICK_DEATH_S:
                _quick_deaths[name] = 0                   # it has run a while: a later death starts afresh
            status_dict[name] = {"status": "running"}

    _cap_running_out_logs(processes)
    _check_recordings_budget()
    _check_disk()                                          # review I6: DISK_LOW_WARNING

    # Raspberry Pi temperature.
    temp = get_raspberry_pi_temperature()
    status_dict["temperature_C"] = round(temp, 2) if temp is not None else None
    # Once per failure streak, not every pass (it ran every SCRIPT_MONITOR_TIME, at INFO, 2026-10-15).
    global _temp_unreadable
    if temp is None and not _temp_unreadable:
        logger.warning("Could not read the Raspberry Pi temperature (reported again once it has been read)")
    _temp_unreadable = temp is None

    return status_dict
