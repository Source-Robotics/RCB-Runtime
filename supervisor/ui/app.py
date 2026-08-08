#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/ui/app.py
Description:
    OLED UI state machine. Rule everywhere: tap = move, hold = select/enter,
    hold-B1 = back. Button callbacks and the render dispatch both switch on
    state.ui["mode"].

    HOME (dashboard + tab bar), INFO (carousel), SETTINGS (list), EDIT (number),
    and CONFIRM (destructive actions: hold B2 = YES, B1 = cancel) all work.
"""

import time

from supervisor import state
from supervisor import settings_store
from supervisor.ui import pages
from supervisor.ui import settings as settings_menu
from supervisor.ui.render import Screen

SETTINGS = settings_menu.SETTINGS

SECTIONS = ["Home", "Info", "Settings"]
IDLE_RETURN = 12.0   # seconds before non-Home screens fall back to Home


def _touch():
    state.ui["last_input"] = time.perf_counter()


# ==========================================================================
# Input — tap = move
# ==========================================================================
def on_b1_short():
    _touch()
    ui = state.ui
    if ui["mode"] == "HOME":
        ui["tab"] = (ui["tab"] - 1) % len(SECTIONS)
    elif ui["mode"] == "INFO" and pages.INFO_PAGES:
        ui["info_page"] = (ui["info_page"] - 1) % len(pages.INFO_PAGES)
    elif ui["mode"] == "SETTINGS" and SETTINGS:
        ui["set_index"] = (ui["set_index"] - 1) % len(SETTINGS)
    elif ui["mode"] == "EDIT":
        _edit_step(-1)
    elif ui["mode"] == "CONFIRM":
        _cancel_confirm()        # a tap is the safe cancel in a confirm prompt


def on_b2_short():
    _touch()
    ui = state.ui
    if ui["mode"] == "HOME":
        ui["tab"] = (ui["tab"] + 1) % len(SECTIONS)
    elif ui["mode"] == "INFO" and pages.INFO_PAGES:
        ui["info_page"] = (ui["info_page"] + 1) % len(pages.INFO_PAGES)
    elif ui["mode"] == "SETTINGS" and SETTINGS:
        ui["set_index"] = (ui["set_index"] + 1) % len(SETTINGS)
    elif ui["mode"] == "EDIT":
        _edit_step(+1)
    elif ui["mode"] == "CONFIRM":
        _cancel_confirm()        # any tap is the safe cancel in a confirm prompt


# ==========================================================================
# Input — hold = select / back
# ==========================================================================
def on_b2_long():
    _touch()
    ui = state.ui
    if ui["mode"] == "HOME":
        section = SECTIONS[ui["tab"]]
        if section == "Info":
            ui["mode"] = "INFO"
            ui["info_page"] = 0
        elif section == "Settings":
            ui["mode"] = "SETTINGS"
            ui["set_index"] = 0
        # "Home" tab: already here, no-op
    elif ui["mode"] == "SETTINGS" and SETTINGS:
        _activate(SETTINGS[ui["set_index"]])
    elif ui["mode"] == "EDIT":
        # save the working value back to the setting, return to the list
        SETTINGS[ui["set_index"]]["set"](ui["edit_value"])
        settings_store.save()
        ui["mode"] = "SETTINGS"
    elif ui["mode"] == "CONFIRM":
        # YES — run the pending action, then back to the list
        item = ui["confirm"]
        ui["confirm"] = None
        ui["mode"] = "SETTINGS"
        if item:
            item["run"]()


def on_b1_long():
    _touch()
    ui = state.ui
    if ui["mode"] == "EDIT":
        ui["mode"] = "SETTINGS"        # cancel: discard edit_value
    elif ui["mode"] == "CONFIRM":
        _cancel_confirm()              # cancel the destructive action
    elif ui["mode"] in ("INFO", "SETTINGS"):
        ui["mode"] = "HOME"


def _cancel_confirm():
    state.ui["confirm"] = None
    state.ui["mode"] = "SETTINGS"


# ==========================================================================
# Settings / Edit helpers
# ==========================================================================
def _activate(item):
    """B2-hold on a settings item."""
    if item["type"] == "toggle":
        item["set"](not item["get"]())
        settings_store.save()
    elif item["type"] == "number":
        state.ui["edit_value"] = item["get"]()
        state.ui["mode"] = "EDIT"
    elif item["type"] == "action":
        if item.get("confirm"):
            state.ui["confirm"] = item
            state.ui["mode"] = "CONFIRM"
        else:
            item["run"]()


def _edit_step(direction):
    item = SETTINGS[state.ui["set_index"]]
    value = state.ui["edit_value"] + direction * item["step"]
    value = max(item["min"], min(item["max"], value))
    state.ui["edit_value"] = round(value, 6)


def idle_check():
    ui = state.ui
    if ui["mode"] != "HOME" and time.perf_counter() - ui["last_input"] > IDLE_RETURN:
        ui["mode"] = "HOME"


# ==========================================================================
# Render — dispatch on mode
# ==========================================================================
def render():
    s = Screen().clear()
    mode = state.ui["mode"]

    if mode == "INFO":
        n = len(pages.INFO_PAGES)
        if n:
            idx = state.ui["info_page"] % n
            title, render_page = pages.INFO_PAGES[idx]
            s.title_bar(title, f"{idx + 1}/{n}")
            render_page(s)
        else:
            s.title_bar("INFO")
            s.body(1, "(no pages)")

    elif mode == "SETTINGS":
        s.title_bar("SETTINGS")
        if SETTINGS:
            s.list_view(SETTINGS, state.ui["set_index"], settings_menu.value_str)
        else:
            s.body(1, "(no settings)")

    elif mode == "EDIT":
        item = SETTINGS[state.ui["set_index"]]
        s.edit_view(item["label"], item["fmt"].format(state.ui["edit_value"]))

    elif mode == "CONFIRM":
        item = state.ui["confirm"]
        s.confirm_view(item["label"] if item else "Confirm")

    else:  # HOME
        pages.render_home(s)
        s.tabbar(SECTIONS, state.ui["tab"])

    return s
