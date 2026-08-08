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

    This module owns the entry shape, the code->{cause, fix} catalog, and the
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
import time
from logging.handlers import RotatingFileHandler

from project_paths import ACTIVITY_RTI_LOG, ACTIVITY_EXEC_LOG


# ---------------------------------------------------------------------------
# Catalog — code -> {"cause": str|None, "fix": str|None}.
#
# RTI hardware-fault codes already live in error_checks.ERROR_DESCRIPTIONS
# (that's the existing source of truth for those); this dict covers
# everything else — EXEC's command lifecycle/rejection/planner codes and
# RTI's routine (non-fault) events. get_catalog_entry() checks both.
# ---------------------------------------------------------------------------
_FALLBACK = {"cause": None, "fix": "not yet documented — check logs/high_level.log for detail."}

CATALOG = {
    # EXEC — command lifecycle (info; nothing to explain)
    "CMD_QUEUED":    {"cause": None, "fix": None},
    "CMD_STARTED":   {"cause": None, "fix": None},
    "CMD_COMPLETED": {"cause": None, "fix": None},
    "CMD_EXECUTED":  {"cause": None, "fix": None},

    # EXEC — rejection codes (warning) — reused verbatim from the reply's own "code" field
    "BAD_ARGS":      {"cause": "command arguments failed validation.", "fix": "check the command's argument count/types and retry."},
    "QUEUE_FULL":    {"cause": "the motion queue is full.", "fix": "wait for queued commands to complete, or clear the queue, then retry."},
    "BAD_PIN":       {"cause": "referenced an invalid/out-of-range digital IO pin.", "fix": "check the pin number against the robot's IO configuration."},
    "UNKNOWN_CMD":   {"cause": "command name is not recognized.", "fix": "check spelling/casing against the command reference."},
    "BAD_JSON":      {"cause": "message was not valid JSON.", "fix": "check the client's wire-protocol encoding."},
    "FORBIDDEN":     {"cause": "command is not allowed in the current mode/state.", "fix": "check the robot's current mode before sending this command."},
    "BAD_OPTIONS":   {"cause": "per-command motion options failed validation.", "fix": "check profile/speed/blend/completion_policy values against allowed ranges."},
    "BAD_PROFILE":   {"cause": "requested trajectory profile is not recognized.", "fix": "check the profile name against the supported list."},
    "BAD_IK_SOLVER": {"cause": "requested IK solver is not recognized.", "fix": "check the solver name against the supported list."},
    "BAD_POLICY":    {"cause": "requested completion policy is not recognized.", "fix": "use one of: commanded, settled, strict."},
    "UNHANDLED":     {"cause": "an unexpected exception occurred while handling the command.", "fix": "check logs/high_level.log for the full traceback."},
    "REJECTED":      {"cause": None, "fix": None},

    # EXEC — planner/motion failures (error)
    "IK_TARGET_UNREACHABLE": {"cause": "requested pose is outside the arm's reachable workspace.", "fix": "check the target pose against the robot's reach/joint limits."},
    "IK_PARTIAL_PATH":       {"cause": "IK solver could only satisfy part of the requested Cartesian path.", "fix": "shorten the path or move the target closer to a reachable region."},
    "IK_MARGIN_VIOLATION":   {"cause": "solved path passed too close to a joint or singularity margin.", "fix": "adjust the target pose or relax the margin, if appropriate."},
    "IK_POSE_ERROR":         {"cause": "solved joint configuration does not reproduce the requested pose within tolerance.", "fix": "check for a nearby singularity or an unreachable orientation."},
    "PLAN_FAIL":              {"cause": "trajectory planning failed.", "fix": "check logs/high_level.log for the underlying planner error."},
    "GRIPPER_TIMEOUT":        {"cause": "gripper did not respond in time.", "fix": "check gripper CAN wiring/power."},
    "SEND_TIMEOUT":           {"cause": "command reply was not sent in time.", "fix": "check the client connection."},
    "SUPERSEDED":             {"cause": None, "fix": None},
    "TOPPRA_UNAVAILABLE":     {"cause": "TOPPRA profile was requested but the toppra package is not importable.", "fix": "install toppra, or select a different profile (trapezoid/quintic/s_curve)."},
    "TOPPRA_INFEASIBLE":      {"cause": "TOPPRA could not compute a valid time-parameterization for this path.", "fix": "check the requested speed/accel limits, or select a different profile."},

    # EXEC — warning (not a failure, planning still succeeds)
    "IK_SINGULARITY_WARNING": {"cause": "planned path passes near a kinematic singularity.", "fix": "check the singularity metrics; consider an alternate path if this recurs."},

    # RTI — routine events (info; nothing to explain)
    "MODE_CHANGE":         {"cause": None, "fix": None},
    "JOG_STARTED":         {"cause": None, "fix": None},
    "JOG_STOPPED":         {"cause": None, "fix": None},
    "LIMIT_CLEARED":       {"cause": None, "fix": None},
    "COMPENSATION_CHANGED": {"cause": None, "fix": None},

    # RTI — routine warning
    "LIMIT_REACHED": {"cause": "jog motion blocked by a joint/workspace limit.", "fix": "jog away from the limit to clear it."},
}


def get_catalog_entry(code: str) -> dict:
    """Look up {cause, fix} for a code.

    RTI hardware-fault codes (short_keys from error_checks.ERROR_DESCRIPTIONS)
    take priority since that's their existing source of truth; falls back to
    this module's CATALOG, then to the generic not-yet-documented fallback.

    Import is local (not module-level) — error_checks.py imports this module
    for logging/emission, so a top-level import here would be circular.
    """
    from utility.error_checks import ERROR_DESCRIPTIONS
    if code in ERROR_DESCRIPTIONS:
        return ERROR_DESCRIPTIONS[code]
    return CATALOG.get(code, _FALLBACK)


def format_entry(source, level, code, id=None, command=None, message=None, context=None):
    """Build one activity-log entry dict — the entry shape from the design doc.

    cause/fix are only populated for warning/error levels; info entries have
    nothing to explain and stay null. `context` is only populated for
    CMD_STARTED (the resolved MotionOptions for that command) — see
    TODO/activity_log_design.md's context-field discussion.
    """
    cause = fix = None
    if level in ("warning", "error"):
        entry = get_catalog_entry(code)
        cause, fix = entry.get("cause"), entry.get("fix")
    return {
        "timestamp": time.time(),
        "source": source,
        "level": level,
        "code": code,
        "id": id,
        "command": command,
        "message": message,
        "cause": cause,
        "fix": fix,
        "context": context,
    }


# ---------------------------------------------------------------------------
# Two dedicated loggers — one per file, one writer per file, no shared root.
# propagate=False keeps entries out of RTI.log/high_level.log/console; a bare
# formatter means the JSON line *is* the message, no added prefix.
# ---------------------------------------------------------------------------
_LOGGERS = {}


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

    _LOGGERS[name] = logger
    return logger


def get_rti_activity_logger(log_file: str = str(ACTIVITY_RTI_LOG)) -> logging.Logger:
    """RTI's activity logger. RTI logs discrete state transitions — inherently
    rare — so a modest rotation size is enough."""
    return _get_activity_logger("activity_rti", log_file, max_bytes=2_000_000)


def get_exec_activity_logger(log_file: str = str(ACTIVITY_EXEC_LOG)) -> logging.Logger:
    """EXEC's activity logger. Volume scales with command throughput, which
    has no natural ceiling, so this file gets a larger rotation size."""
    return _get_activity_logger("activity_exec", log_file, max_bytes=20_000_000)


def log_activity(logger: logging.Logger, source, level, code, id=None, command=None, message=None, context=None):
    """Format one entry and write it as a single JSON line."""
    entry = format_entry(source, level, code, id=id, command=command, message=message, context=context)
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
