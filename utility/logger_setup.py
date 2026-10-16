#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: logger_setup.py
Author: Petar Crnjak
Date: 2024-06-11
Version: 2.0 (2026-10-15: one line format, the loop tick, one list of logger names)
Description:
    The robot's logging: ONE setup call per process, ONE line format, ONE list of logger names.
    docs/logging.md is the reference.

    Line format (2026-10-15):

        INFO     | 2026-10-12T23:05:46.048790Z |   8736850 | rti.can       | J6: a 0.9 s burst of late replies

    the level; the time in UTC, ISO 8601, to the microsecond (the same resolution as the epoch float before it);
    RTI's loop tick when the line was logged ("-" before the shared memory exists) — the same tick in every
    process, so RTI.log, high_level.log and an incident file line up by tick; the logger name from NAMES; the
    message, without a "[Tag]" (the name is the tag). Files keep UTC; the dashboard SHOWS times in the chosen
    <display_timezone> (utility/display_time.py). parse_line() reads this format and the one before it.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

from __future__ import annotations

import datetime as _dt
import logging
import os
import queue
import re
import sys
import time
from logging.handlers import QueueHandler, QueueListener, RotatingFileHandler


_FMT = "%(levelname)-8s | %(asctime)s | %(tick)9s | %(name)-13s | %(message)s"

# Every logger the robot's own code uses, and what it covers. A module takes its name from here —
# logging.getLogger("rti.homing") — never __name__ (a module path says where the code is, not what it is about,
# and five files make up homing) and never the root logger. dummy_test_scripts/test_logging_format.py holds every
# getLogger() in the runtime tree to this list. "<area>.<part>": the area is the process that owns the subsystem;
# code that runs in several processes has an area of its own.
NAMES = {
    # RTI — the real-time process, logs/RTI.log
    "rti":           "RTI's lines without a narrower home: commands it refuses, shutdown, uncaught exceptions",
    "rti.startup":   "start-up: control box, kinematics backend, simulation, motor Kt",
    "rti.loop":      "the loop itself: overruns, the dispatch profile",
    "rti.mode":      "mode transitions and refused mode requests",
    "rti.flash":     "the firmware flashing mode",
    "rti.errors":    "the error latch: faults latched and cleared, the robot disabled",
    "rti.can":       "CAN: drive configuration, reconnects, late replies, the drive watchdog",
    "rti.safety":    "safety monitor, zones, joint and haptic walls",
    "rti.homing":    "homing, the arm table, the Hall sensor check",
    "rti.handler":   "RTI mode: the external PC's stream (motion/rti_handler.py)",
    "rti.exec":      "EXEC batches as RTI runs them (motion/exec_handler.py)",
    "rti.gripper":   "gripper commands and calibration",
    "rti.dynamics":  "payload, dynamics feedforward, friction and gravity compensation",
    "rti.selfcheck": "the start-up self-check",
    "rti.recipe":    "telemetry recipes",
    "rti.incident":  "incident dumps (temp_data/incidents)",
    "rti.net":       "RTI's command listener and UDP sockets",
    # the executor — command_executor.py, logs/high_level.log
    "exec":          "the executor's lines without a narrower home: start-up, reloads, walls",
    "exec.control":  "who holds control: claims, releases, takeovers",
    "exec.tcp":      "client connections",
    "exec.forward":  "commands forwarded to RTI",
    "exec.status":   "the status watcher",
    "exec.planner":  "the trajectory planner, G-code",
    "exec.stop":     "stops",
    # the supervisor — logs/supervisor.log
    "sup":           "the supervisor's lines without a narrower home: start-up, shutdown, power actions",
    "sup.process":   "starting, stopping and restarting the managed scripts",
    "sup.pcb":       "the control box's UART",
    "sup.gpio":      "GPIO state changes",
    "sup.can_link":  "the CAN link's kernel counters (bus-off, restarts)",
    "sup.ui":        "the OLED menu, its settings",
    # the dashboard — logs/gui.log
    "gui":           "the dashboard",
    # the self-collision worker — logs/self_collision.log (and the executor's planner check)
    "sc":            "self-collision: the model, margins, the worker's cycle times",
    "rec":           "the recorder (recorder.py): incident dumps and captures from RTI's shared-memory trace",
    # code that runs in several processes
    "config":        "the robot / system / gripper XMLs, zones and walls files",
    "filters":       "signal filter specs",
    "shm":           "shared memory",
    "kin":           "kinematics: IK, the Pinocchio model",
    "proc":          "process setup: real-time scheduling, CPU cores",
    "io":            "GPIO pins, the E-stop channels, the control box",
}

# Libraries whose INFO lines are noise in our files (2026-10-15): toppra logged "Finish parametrization in 0.007
# secs" for every plan (1,736 lines in high_level.log); python-can's bus lines in RTI.log. Their warnings stay.
QUIET_LIBRARIES = {"toppra": logging.WARNING, "can": logging.WARNING}


class UtcFormatter(logging.Formatter):
    """_FMT with the time in UTC, ISO 8601, microseconds; a record without a tick (made before
    setup_logging installed the record factory) gets "-"."""

    def formatTime(self, record, datefmt=None):
        t = record.created
        if datefmt:                                   # a caller's own pattern — still UTC
            return time.strftime(datefmt, time.gmtime(t))
        return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t)) + ".%06dZ" % int((t - int(t)) * 1_000_000)

    def format(self, record):
        if not hasattr(record, "tick"):
            record.tick = "-"
        return super().format(record)


def make_formatter() -> UtcFormatter:
    return UtcFormatter(_FMT)


def _current_tick() -> str:
    """RTI's loop tick from shared memory — only if THIS process has set it up (never imports or creates it)."""
    sd = sys.modules.get("data.shared_data")
    if sd is None or not getattr(sd, "_initialized", False):
        return "-"
    try:
        return str(int(sd.rti["loop_tick"][0]))
    except Exception:
        return "-"


_base_record_factory = logging.getLogRecordFactory()


def _record_factory(*args, **kwargs):
    """Stamps the tick when the record is MADE — in the caller's thread, before RTI's queue — not when it is written."""
    record = _base_record_factory(*args, **kwargs)
    record.tick = _current_tick()
    return record


def _is_terminal(stream) -> bool:
    try:
        return stream is not None and stream.isatty()
    except (AttributeError, ValueError):
        return False


def setup_logging(log_file: str = "logs/system.log", log_to_console: bool = True, critical_log_file: str | None = None,
                  queued: bool = False) -> QueueListener | None:
    """
    Configure the root logger for the process. Returns the QueueListener when `queued`, else None.

    Call this once at the top of each main script (RTI.py, supervisor.py, etc.). Every logger in NAMES
    propagates to the root, so modules need nothing but logging.getLogger("<name>").

    Args:
        log_file (str):           Path to the main rotating log file.
        log_to_console (bool):    Also write to stderr — only when stderr IS a terminal (2026-10-15). Under the
                                  supervisor stderr is logs/<NAME>.out.log, and every line went to the disk twice
                                  (two thirds of RTI.out.log were copies of RTI.log); the .out.log now holds only
                                  what bypasses logging — crashes, thread dumps, a library's own stderr.
        critical_log_file (str):  Optional path for a persistent CRITICAL-only log
                                  that survives multiple restarts and is not rotated
                                  as aggressively. Pass str(CRITICAL_LOG) from project_paths.
        queued (bool):            For a REAL-TIME process (RTI.py, 2026-10-03). The root logger gets only a
                                  QueueHandler: a logging call formats the record and puts it on a bounded
                                  queue (microseconds, no I/O). The file / console / critical handlers are
                                  driven by a QueueListener thread, which the CALLER must start — after its
                                  real-time scheduling is set, so the thread inherits it — and stop at exit
                                  (atexit) so queued records are flushed. Why: on 2026-10-03 one RT tick
                                  took 18.4 ms (LOOP_CRITICAL, the robot disabled) because a logging call's
                                  file write blocked on the SD card. Records logged before start() wait on
                                  the queue; only a SIGKILL can lose them.
    """
    root = logging.getLogger()
    if any(getattr(h, "_rcb", False) for h in root.handlers):
        return None  # already configured by this function — avoid duplicate handlers on re-import
    # This function is the process's logging authority (2026-10-15). A handler someone else put on the root — a
    # module-level logging.info() before this call runs logging.basicConfig() — used to make this function set up
    # NOTHING: RTI ran without RTI.log, INFO or the queue from 2026-10-04 17:52 to 2026-10-05. Now it is replaced.
    for h in list(root.handlers):
        root.removeHandler(h)
    logging.setLogRecordFactory(_record_factory)
    for name, level in QUIET_LIBRARIES.items():
        logging.getLogger(name).setLevel(level)

    os.makedirs(os.path.dirname(log_file) or "logs", exist_ok=True)
    formatter = make_formatter()

    handlers = []
    # Main rotating file handler (10 MB x 5 backups = up to 50 MB history)
    file_handler = RotatingFileHandler(log_file, maxBytes=10_000_000, backupCount=5)
    file_handler.setFormatter(formatter)
    handlers.append(file_handler)

    # Console output — a person at a terminal, never the supervisor's .out.log
    if log_to_console and _is_terminal(sys.stderr):
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
            h._rcb = True  # type: ignore[attr-defined]  # marks this function's handlers (the re-import guard)
            root.addHandler(h)
        return None
    # Real-time process: the root logger only enqueues; the listener thread does the I/O.
    q = queue.SimpleQueue()
    queue_handler = BoundedQueueHandler(q)     # at most LOG_QUEUE_MAX records waiting (2026-10-15)
    queue_handler._rcb = True  # type: ignore[attr-defined]  # marks this function's handler (the re-import guard)
    root.addHandler(queue_handler)
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


# ---------------------------------------------------------------------------------------------------------------
# Reading the files — the one parser for tools, api_tests and the dashboard's Logs tab. Both formats: this one and
# the one before 2026-10-15 ("LEVEL | 1791846346.048790 | logger | [Tag] message"), which the rotated files keep.
# ---------------------------------------------------------------------------------------------------------------
_LEVELS = r"(DEBUG|INFO|WARNING|ERROR|CRITICAL)"
_NEW_LINE = re.compile(_LEVELS + r" *\| (\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d\.\d{6})Z \| *(-|\d+) \| (\S+) *\| ?(.*)$")
_OLD_LINE = re.compile(_LEVELS + r" *\| (\d+\.\d+) \| (\S*) \| ?(.*)$")

# The old lines' "[Tag]" -> (the logger name it became, the words it became; "" = dropped), so a reader sees one
# vocabulary in old and new files. Where one tag became several names (RTI.py's "[RTI]" is now rti / rti.handler /
# rti.dynamics / ...; command_executor's "[RTI]" is exec.forward), the old line gets the broadest one.
LOG_QUEUE_MAX = 10_000      # records waiting for the listener (~10 MB); past it new records are dropped and counted


class BoundedQueueHandler(QueueHandler):
    """A QueueHandler whose queue never grows past `limit` records (2026-10-15). Normally the listener keeps the queue
    near empty; if the disk stalls for long (a slow or full SD card) records would pile up in RAM without end. Past the
    limit a new record is dropped and counted — the caller never blocks — and the next record that fits is preceded
    by ONE warning saying how many were lost."""

    def __init__(self, q, limit: int = LOG_QUEUE_MAX, note_name: str = "rti"):
        super().__init__(q)
        self.limit = int(limit)
        self.note_name = note_name
        self.dropped = 0

    def enqueue(self, record):
        if self.queue.qsize() >= self.limit:
            self.dropped += 1
            return
        if self.dropped:
            n, self.dropped = self.dropped, 0
            note = logging.getLogRecordFactory()(
                self.note_name, logging.WARNING, __file__, 0,
                "%d log record(s) dropped: the log queue was full (the disk did not keep up)", (n,), None)
            self.queue.put_nowait(self.prepare(note))
        self.queue.put_nowait(record)


LEVEL_NAMES = ("DEBUG", "INFO", "WARNING", "ERROR")


def apply_log_level(name, level) -> tuple:
    """Set one logger's level in THIS process without a restart (set_log_level, 2026-10-15): `name` from NAMES or
    QUIET_LIBRARIES ("toppra", "can"), `level` one of LEVEL_NAMES (any case). Returns (ok, text): the change made, or
    why it was refused. Not persisted: a restart returns to the configured levels. Nothing else is touched."""
    name = str(name or "").strip()
    level = str(level or "").strip().upper()
    if name not in NAMES and name not in QUIET_LIBRARIES:
        return False, f"unknown logger {name!r}: one of {', '.join(sorted(NAMES))}, toppra, can"
    if level not in LEVEL_NAMES:
        return False, f"unknown level {level!r}: one of {', '.join(LEVEL_NAMES)}"
    logging.getLogger(name).setLevel(getattr(logging, level))
    return True, f"{name} -> {level}"


LEGACY_TAGS = {
    "[RTI]": ("rti", ""), "[RTI][Mode]": ("rti.mode", ""), "[RTI][Flash]": ("rti.flash", ""),
    "[Startup]": ("rti.startup", ""), "[Startup][Kt]": ("rti.startup", "Kt: "), "[SIM]": ("rti.startup", ""),
    "[control box]": ("rti.startup", "control box "), "[CAN]": ("rti.can", ""), "[SAFETY]": ("rti.safety", ""),
    "[Hall check]": ("rti.homing", "Hall check "), "[Homing]": ("rti.homing", ""), "[filters]": ("filters", ""),
    "[payload]": ("rti.dynamics", "payload: "), "[Gripper]": ("rti.gripper", ""), "[ERRORS]": ("rti.errors", ""),
    "[WARNING]": ("rti.errors", ""), "[ERROR]": ("rti.errors", ""), "[Selfcheck]": ("rti.selfcheck", ""),
    "[DataRecipe]": ("rti.recipe", ""), "[RecipeManager]": ("rti.recipe", ""), "[incident]": ("rti.incident", ""),
    "[ZMQ]": ("rti.net", ""), "[EXEC]": ("exec", ""), "[CTRL]": ("exec.control", ""), "[TCP]": ("exec.tcp", ""),
    "[STOP]": ("exec.stop", ""), "[STATUS]": ("exec.status", ""), "[Planner]": ("exec.planner", ""),
    "[G-code]": ("exec.planner", "G-code: "), "[CAN link]": ("sup.can_link", ""), "[SC]": ("sc", ""),
    "[SELF-COLLISION]": ("sc", ""), "[GUI]": ("gui", ""), "[system.xml]": ("config", "system.xml "),
    "[robot XML]": ("config", "robot XML "), "[robot.xml]": ("config", "robot XML "), "[XML]": ("config", "robot XML "),
    "[XML Validation]": ("config", "XML validation: "), "[shm]": ("shm", ""), "[proc setup]": ("proc", "setup "),
    "[RT setup]": ("proc", "real-time setup "), "[E-stop]": ("io", "E-stop "), "[pinocchio]": ("kin", "Pinocchio: "),
    "[Kinematics]": ("kin", ""),
}
_LEGACY_TAG = re.compile(r"^((?:\[[^\]]+\])+) ?")


def parse_line(line: str) -> dict | None:
    """One log line -> {"level", "time" (UTC epoch, float), "tick" (int or None), "name", "message"}; None for a
    line that does not start a record (a traceback's continuation, a print in an .out.log). An old-format line
    with a known "[Tag]" comes back as the new format would have written it (LEGACY_TAGS)."""
    line = line.rstrip("\n")
    m = _NEW_LINE.match(line)
    if m:
        level, stamp, tick, name, message = m.groups()
        t = _dt.datetime.fromisoformat(stamp).replace(tzinfo=_dt.timezone.utc).timestamp()
        return {"level": level, "time": t, "tick": None if tick == "-" else int(tick), "name": name,
                "message": message}
    m = _OLD_LINE.match(line)
    if m:
        level, stamp, name, message = m.groups()
        tag = _LEGACY_TAG.match(message)
        if tag and tag.group(1) in LEGACY_TAGS:
            name, words = LEGACY_TAGS[tag.group(1)]
            message = words + message[tag.end():]
        return {"level": level, "time": float(stamp), "tick": None, "name": name, "message": message}
    return None
