#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/settings_store.py
Description:
    Persist the OLED menu's user settings (state.settings) across reboots.

    - load() merges a saved JSON file over the in-memory defaults, validating
      each key by type. Missing or corrupt file -> defaults are kept (no crash).
    - save() writes the current settings atomically (temp file + os.replace) so a
      power loss mid-write can never leave a half-written / corrupt file.

    Only keys that already exist in state.settings are loaded, so an old or
    hand-edited file with junk keys can't inject anything unexpected.
"""

import os
import json
import logging
import tempfile

from project_paths import SUPERVISOR_SETTINGS
from supervisor import state

logger = logging.getLogger("sup.ui")  # docs/logging.md "Logger names"


def load():
    """Merge persisted settings over the defaults; keep defaults on any problem."""
    try:
        with open(SUPERVISOR_SETTINGS) as f:
            data = json.load(f)
    except FileNotFoundError:
        logger.info("No saved settings yet; using defaults")
        return
    except (json.JSONDecodeError, OSError) as exc:
        logger.warning(f"Could not read {SUPERVISOR_SETTINGS}: {exc}; using defaults")
        return

    for key, default in state.settings.items():
        if key not in data:
            continue
        try:
            # Coerce to the default's type so a bad value can't change behaviour.
            state.settings[key] = type(default)(data[key])
        except (TypeError, ValueError):
            logger.warning(f"Bad value for '{key}': {data[key]!r}; keeping default")
    logger.info(f"Loaded settings: {dict(state.settings)}")


def save():
    """Atomically persist the current settings. Best-effort; never raises."""
    try:
        SUPERVISOR_SETTINGS.parent.mkdir(parents=True, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=str(SUPERVISOR_SETTINGS.parent), suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as f:
                json.dump(state.settings, f, indent=2)
            os.replace(tmp, SUPERVISOR_SETTINGS)   # atomic on POSIX
        except Exception:
            try:
                os.unlink(tmp)
            except OSError:
                pass
            raise
    except Exception as exc:
        logger.warning(f"Could not save settings: {exc}")
