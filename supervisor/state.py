#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/state.py
Description:
    Shared runtime state for the supervisor + OLED UI (single process). Modules
    import these instead of passing everything around or importing each other
    circularly. Only simple values / dicts / a queue — GIL-safe across the
    supervisor's threads.
"""

import queue
import time

# User-editable settings (changed from the OLED Settings menu).
settings = {
    "blink_time": 1.0,    # LED + PCB heartbeat period (s)
    "contrast": 255,      # OLED contrast 0-255
}

# OLED UI navigation state.
ui = {
    "mode": "HOME",        # HOME | INFO | SETTINGS | EDIT | CONFIRM
    "tab": 0,              # index into SECTIONS
    "info_page": 0,        # index into INFO_PAGES
    "set_index": 0,        # index into SETTINGS
    "edit_value": 0.0,     # working value while in EDIT
    "confirm": None,       # pending action dict while in CONFIRM
    "last_input": time.perf_counter(),
    "_applied_contrast": None,
}

# Process-management runtime (owned/written by the monitor thread).
processes = {}
global_monitor_data = {}
action_queue = queue.Queue()   # UI -> monitor thread (single owner of `processes`)
