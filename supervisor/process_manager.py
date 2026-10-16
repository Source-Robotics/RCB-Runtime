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
import signal
import subprocess
import psutil
import logging
import time
from project_paths import PROJECT_ROOT, TEMP_DATA_DIR
from utility.housekeeping import RECORDINGS_BUDGET_BYTES, copy_truncate, enforce_budget, rotate_by_size
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


def _check_recordings_budget(now=None, root=TEMP_DATA_DIR, budget=RECORDINGS_BUDGET_BYTES):
    """Once an hour (the first monitor pass too): the test recordings in temp_data together at most `budget` (1 GB),
    the oldest runs deleted first (utility/housekeeping.enforce_budget; user, 2026-10-15). Returns the deleted run
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
    proc = subprocess.Popen(
        cmd,
        shell=True,
        stdout=logf,
        stderr=subprocess.STDOUT,
        start_new_session=True,  # new pgid so we can kill the whole tree later
    )
    proc._logf = logf  # keep a reference so the file handle isn't GC'd
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
            logger.error(f"{name} stopped unexpectedly (exit {process.returncode}) — restarting it. "
                        f"See logs/{name}.out.log")
            restart_script(name, processes, SCRIPTS)
            status_dict[name] = {"status": "restarted (crashed)"}
        else:
            status_dict[name] = {"status": "running"}

    _cap_running_out_logs(processes)
    _check_recordings_budget()

    # Raspberry Pi temperature.
    temp = get_raspberry_pi_temperature()
    status_dict["temperature_C"] = round(temp, 2) if temp is not None else None
    # Once per failure streak, not every pass (it ran every SCRIPT_MONITOR_TIME, at INFO, 2026-10-15).
    global _temp_unreadable
    if temp is None and not _temp_unreadable:
        logger.warning("Could not read the Raspberry Pi temperature (reported again once it has been read)")
    _temp_unreadable = temp is None

    return status_dict
