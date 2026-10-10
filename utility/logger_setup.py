#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: logger_setup.py
Author: Petar Crnjak
Date: 2024-06-11
Version: 1.0
Description: 
    Helper function for setting up logging.
    

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
Dependencies: os, sys
"""

from __future__ import annotations

import logging
import queue
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler
import os


_FMT = '%(levelname)-8s | %(created)f | %(name)s | %(message)s'


def setup_logging(log_file: str = "logs/system.log", log_to_console: bool = True, critical_log_file: str | None = None,
                  queued: bool = False) -> QueueListener | None:
    """
    Configure the root logger for the process. Returns the QueueListener when `queued`, else None.

    Call this once at the top of each main script (RTI.py, supervisor.py, etc.).
    All submodules that use logging.getLogger(__name__) automatically inherit
    the configuration — no extra setup needed in submodules.

    Args:
        log_file (str):           Path to the main rotating log file.
        log_to_console (bool):    If True, also print output to the terminal.
        critical_log_file (str):  Optional path for a persistent CRITICAL-only log
                                  that survives multiple restarts and is not rotated
                                  as aggressively. Pass str(CRITICAL_LOG) from project_paths.
        queued (bool):            For a REAL-TIME process (RTI.py, 2026-10-03). The root logger gets only a
                                  QueueHandler: a logging call formats the record and puts it on an unbounded
                                  queue (microseconds, no I/O). The file / console / critical handlers are
                                  driven by a QueueListener thread, which the CALLER must start — after its
                                  real-time scheduling is set, so the thread inherits it — and stop at exit
                                  (atexit) so queued records are flushed. Why: on 2026-10-03 one RT tick
                                  took 18.4 ms (LOOP_CRITICAL, the robot disabled) because a logging call's
                                  file write blocked on the SD card. Records logged before start() wait on
                                  the queue; only a SIGKILL can lose them.
    """
    root = logging.getLogger()
    if root.handlers:
        return None  # Already configured — avoid duplicate handlers on re-import

    os.makedirs(os.path.dirname(log_file) or "logs", exist_ok=True)
    formatter = logging.Formatter(_FMT)

    handlers = []
    # Main rotating file handler (10 MB x 5 backups = up to 50 MB history)
    file_handler = RotatingFileHandler(log_file, maxBytes=10_000_000, backupCount=5)
    file_handler.setFormatter(formatter)
    handlers.append(file_handler)

    # Optional console output
    if log_to_console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        handlers.append(console_handler)

    # Optional persistent CRITICAL-only log (larger backups, survives restarts)
    if critical_log_file:
        os.makedirs(os.path.dirname(critical_log_file) or "logs", exist_ok=True)
        critical_handler = RotatingFileHandler(critical_log_file, maxBytes=5_000_000, backupCount=10)
        critical_handler.setLevel(logging.CRITICAL)
        critical_handler.setFormatter(formatter)
        handlers.append(critical_handler)

    root.setLevel(logging.INFO)
    if not queued:
        for h in handlers:
            root.addHandler(h)
        return None
    # Real-time process: the root logger only enqueues; the listener thread does the I/O.
    q = queue.SimpleQueue()
    root.addHandler(QueueHandler(q))
    # The activity loggers (utility/activity_log.py: their own files, propagate=False) share this queue and
    # listener since 2026-10-03 — until then their writes were synchronous in the RT thread. Records are routed
    # by logger name: the root handlers refuse activity records, each activity file handler takes only its own.
    for h in handlers:
        h.addFilter(_not_activity)
    listener = QueueListener(q, *handlers, respect_handler_level=True)
    from utility.activity_log import route_through_queue
    route_through_queue(q, listener)
    return listener


def _not_activity(record: logging.LogRecord) -> bool:
    return not record.name.startswith("activity_")
