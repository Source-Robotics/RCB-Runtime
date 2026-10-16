#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: activity_log.py
Author: Petar Crnjak
Company: Source Robotics
Description:
    Unified activity log — the persistent, chronological record of everything
    that happens to the robot: RTI hardware faults, EXEC command outcomes,
    routine events (jog start/stop, mode changes, limits). See
    TODO/activity_log_design.md for the full design and rationale.

    This module owns the entry shape (the catalog itself is utility/error_catalog.py), and the
    two dedicated loggers (activity_rti / activity_exec). It does not decide
    *when* to log — emission call sites live in RTI.py, error_checks.py,
    command_executor.py, and motion/trajectory_planner.py.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
Dependencies: json, logging, os, time
"""

import json
import logging
import os
import threading
import time
from collections import OrderedDict
from logging.handlers import QueueHandler, RotatingFileHandler

from project_paths import ACTIVITY_RTI_LOG, ACTIVITY_EXEC_LOG, ACTIVITY_REC_LOG
from utility.error_catalog import CATALOG as _CATALOG, entry as _catalog_entry


# ---------------------------------------------------------------------------
# The catalog — code -> {number, title, cause, effect, fix}: ONE dict for every code since 2026-10-15
# (utility/error_catalog.CATALOG; until then the RTI fault keys and the rest lived in two dicts).
# ---------------------------------------------------------------------------
CATALOG = _CATALOG
_FALLBACK = {"number": None, "title": None, "cause": None, "effect": None,
             "fix": "Not catalogued yet: check the logs in the Logs tab for the detail."}


def get_catalog_entry(code: str) -> dict:
    """{number, title, cause, effect, fix} of a code; the not-yet-catalogued fallback for an unknown one
    (dummy_test_scripts/test_error_catalog.py keeps every emitted code catalogued)."""
    return _catalog_entry(code) or _FALLBACK


# ---------------------------------------------------------------------------
# Who asked: the ORIGIN of a command's entries (2026-10-15). The executor learns each client's name (the
# robotclient sends it with every message — "dashboard", "pick_place.py on lab-pc") and records origin per command
# id here; RTI records the origin that comes with each forwarded command. Every entry of that id — queued, started,
# completed, the mode change it caused — then carries it. A process remembers the last ORIGINS_KEPT ids.
# ---------------------------------------------------------------------------
ORIGINS_KEPT = 2000
_origins: OrderedDict[str, str] = OrderedDict()
_origins_lock = threading.Lock()


def note_origin(cid, origin):
    """Remember who sent command `cid` (None / empty: nothing to remember)."""
    if not cid or not origin:
        return
    with _origins_lock:
        _origins[cid] = origin
        _origins.move_to_end(cid)
        while len(_origins) > ORIGINS_KEPT:
            _origins.popitem(last=False)


def origin_of(cid):
    """The origin recorded for command `cid`, or None."""
    if not cid:
        return None
    with _origins_lock:
        return _origins.get(cid)


def format_entry(source, level, code, id=None, command=None, message=None, context=None, origin=None):
    """Build one activity-log entry dict — the entry shape from the design doc, extended 2026-10-15.

    number / title: the catalog's, for every code. cause / effect / fix: only for warning / error (an info entry has
    nothing to explain). origin: who asked — given, else what was recorded for `id`, else null (the robot or the
    executor itself). `context` is only populated for CMD_STARTED (the resolved MotionOptions for that command).
    """
    cat = get_catalog_entry(code)
    cause = effect = fix = None
    if level in ("warning", "error"):
        cause, effect, fix = cat.get("cause"), cat.get("effect"), cat.get("fix")
    return {
        "timestamp": time.time(),
        "source": source,
        "origin": origin or origin_of(id),
        "level": level,
        "number": cat.get("number"),
        "code": code,
        "title": cat.get("title"),
        "id": id,
        "command": command,
        "message": message,
        "cause": cause,
        "effect": effect,
        "fix": fix,
        "context": context,
    }


# ---------------------------------------------------------------------------
# Two dedicated loggers — one per file, one writer per file, no shared root.
# propagate=False keeps entries out of RTI.log/high_level.log/console; a bare
# formatter means the JSON line *is* the message, no added prefix.
#
# IN A REAL-TIME PROCESS the file is NOT written by the caller (2026-10-03). propagate=False put these
# loggers outside the root logger's QueueHandler, so every activity entry — a mode change, a newly latched
# error, a jog limit — was a synchronous file write (and at the rotation boundary a rename) inside the RT
# tick: the same mechanism as the 18.4 ms LOOP_CRITICAL tick that queued RTI.log. route_through_queue()
# (called by utility.logger_setup.setup_logging(queued=True)) moves each activity file handler onto the
# process's one QueueListener thread and leaves the logger a QueueHandler on the same queue. The handlers
# filter by logger NAME, so activity lines never reach RTI.log and RTI.log lines never reach the activity
# file. Content and format are unchanged. Processes that set up logging without `queued` (the executor,
# the supervisor) keep the direct file handler.
# ---------------------------------------------------------------------------
_LOGGERS = {}
_QUEUE = None        # the RT process's log queue, once route_through_queue() has been called
_LISTENER = None     # its QueueListener — file handlers of loggers created later are added to it


def _only_logger(name: str):
    """Handler filter: exactly this logger's records (the listener hands every record to every handler)."""
    def accept(record):
        return record.name == name
    return accept


def _queue_logger(name: str, logger: logging.Logger) -> None:
    """Move the logger's file handler(s) to the listener; leave the logger a QueueHandler. Idempotent."""
    for h in [h for h in logger.handlers if not isinstance(h, QueueHandler)]:
        logger.removeHandler(h)
        h.addFilter(_only_logger(name))
        _LISTENER.handlers = (*_LISTENER.handlers, h)
    if not any(isinstance(h, QueueHandler) for h in logger.handlers):
        from utility.logger_setup import BoundedQueueHandler   # the same cap as the root's (2026-10-15)
        logger.addHandler(BoundedQueueHandler(_QUEUE))


def route_through_queue(q, listener) -> None:
    """Real-time processes only: activity loggers (existing and future) enqueue; `listener` writes their files.

    Called once by setup_logging(queued=True) after the listener exists. Start the listener after the
    real-time scheduling is set, stop it at exit — exactly as for RTI.log (utility/logger_setup.py).
    """
    global _QUEUE, _LISTENER
    _QUEUE, _LISTENER = q, listener
    for name, logger in _LOGGERS.items():
        _queue_logger(name, logger)


def _get_activity_logger(name: str, log_file: str, max_bytes: int, backup_count: int = 5) -> logging.Logger:
    if name in _LOGGERS:
        return _LOGGERS[name]

    logger = logging.getLogger(name)
    logger.propagate = False
    logger.setLevel(logging.INFO)

    if not logger.handlers:
        os.makedirs(os.path.dirname(log_file) or "logs", exist_ok=True)
        handler = RotatingFileHandler(log_file, maxBytes=max_bytes, backupCount=backup_count)
        handler.setFormatter(logging.Formatter("%(message)s"))
        logger.addHandler(handler)
    if _QUEUE is not None:
        _queue_logger(name, logger)

    _LOGGERS[name] = logger
    return logger


def get_rti_activity_logger(log_file: str = str(ACTIVITY_RTI_LOG)) -> logging.Logger:
    """RTI's activity logger. RTI logs discrete state transitions — inherently
    rare — so a modest rotation size is enough."""
    return _get_activity_logger("activity_rti", log_file, max_bytes=2_000_000)


def get_rec_activity_logger(log_file: str = str(ACTIVITY_REC_LOG)) -> logging.Logger:
    """The recorder's activity logger (recorder.py, 2026-10-16): its captures — a handful of entries a day."""
    return _get_activity_logger("activity_rec", log_file, max_bytes=2_000_000)


def get_exec_activity_logger(log_file: str = str(ACTIVITY_EXEC_LOG)) -> logging.Logger:
    """EXEC's activity logger. Volume scales with command throughput, which
    has no natural ceiling, so this file gets a larger rotation size."""
    return _get_activity_logger("activity_exec", log_file, max_bytes=20_000_000)


def log_activity(logger: logging.Logger, source, level, code, id=None, command=None, message=None, context=None,
                 origin=None):
    """Format one entry and write it as a single JSON line."""
    entry = format_entry(source, level, code, id=id, command=command, message=message, context=context, origin=origin)
    line = json.dumps(entry)
    getattr(logger, level)(line)
    return entry


class RateLimiter:
    """Debounce guard — allow(code) returns True at most once per min_interval
    seconds for a given code. Design doc §4: guards RTI's hardware-fault
    emission against a flaky/marginal connection flapping a fault rapidly
    (e.g. repeated clear -> instant re-latch). EXEC's side does not use this —
    its volume is legitimate command throughput, not a fault flapping."""

    def __init__(self):
        self._last_emit = {}

    def allow(self, code: str, min_interval: float = 1.0) -> bool:
        now = time.time()
        last = self._last_emit.get(code, 0.0)
        if now - last < min_interval:
            return False
        self._last_emit[code] = now
        return True
