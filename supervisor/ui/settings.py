#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/ui/settings.py
Description:
    Data-driven Settings registry. Adding a setting is ONE constructor call —
    EDIT (numbers) and the generic toggle/action handling are done by app.py, so
    new items need zero new UI code. Order here = order on screen.

    Item types:
        number(label, key, lo, hi, step, fmt)   editable value in state.settings[key]
        toggle(label, key)                       on/off value in state.settings[key]
        action(label, run, confirm=False)        runs run(); confirm gates it (incr 4)
"""

from supervisor import state

SETTINGS = []   # list of item dicts, in display order


def _get(key):
    return lambda: state.settings[key]


def _set(key):
    def setter(value):
        state.settings[key] = value
    return setter


def number(label, key, lo, hi, step, fmt="{:.0f}"):
    SETTINGS.append({
        "label": label, "type": "number",
        "get": _get(key), "set": _set(key),
        "min": lo, "max": hi, "step": step, "fmt": fmt,
    })


def toggle(label, key):
    SETTINGS.append({
        "label": label, "type": "toggle",
        "get": _get(key), "set": _set(key),
    })


def action(label, run, confirm=False):
    SETTINGS.append({
        "label": label, "type": "action", "run": run, "confirm": confirm,
    })


def value_str(item):
    """Right-hand text for an item in the list."""
    if item["type"] == "toggle":
        return "ON" if item["get"]() else "OFF"
    if item["type"] == "number":
        return item["fmt"].format(item["get"]())
    return ">"   # action — hints "press to run"


# ==========================================================================
# The settings (edit this list to add/remove/reorder)
# ==========================================================================
number("Contrast",   "contrast",   0,   255, step=16,  fmt="{:.0f}")
number("Blink rate",  "blink_time", 0.1, 5.0, step=0.1, fmt="{:.1f}s")
toggle("Simulate",    "simulate")
action("Restart RTI", lambda: state.action_queue.put(("restart", "RTI")))
action("Restart GUI", lambda: state.action_queue.put(("restart", "GUI")))
action("Restart HL",  lambda: state.action_queue.put(("restart", "HIGH_LEVEL")))
# --- destructive: gated behind the CONFIRM screen ---
action("Stop all",    lambda: state.action_queue.put(("stop_all",)),  confirm=True)
action("Reboot Pi",   lambda: state.action_queue.put(("reboot",)),    confirm=True)
action("Shutdown Pi", lambda: state.action_queue.put(("shutdown",)),  confirm=True)
