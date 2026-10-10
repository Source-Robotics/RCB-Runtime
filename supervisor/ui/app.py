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

import math
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
# Takeover prompt owns the buttons
# ==========================================================================
# While a forced control takeover is waiting for confirmation, the two box buttons belong to
# the EXECUTOR (hold B2 = allow, tap B1 = deny), which reads them from shared memory. The menu
# handlers below must not ALSO act on those presses — and "is the prompt up right now?" is
# not enough to stop them, because of when ButtonHandler fires:
#
#   * short press fires on RELEASE. Tap B1 to deny: the executor refuses on the press, the
#     prompt clears instantly, and the release then lands on whatever menu is underneath.
#   * long press fires WHILE HELD at LONG_PRESS_TIME. If the underlying screen is a CONFIRM
#     ("Reboot?"), holding B2 to ALLOW a takeover would also run on_b2_long -> item["run"]()
#     and reboot the Pi — possibly while someone else is driving the arm.
#
# So the latch is SET the moment the prompt appears and CLEARED only once the prompt is gone
# AND both buttons have been released. Every press that overlapped the prompt is swallowed
# whole, start to finish. supervisor.py's loop drives the two halves around check_button().
_modal_latch = [False]

# What each box button does on the takeover prompt. B1 is the LEFT button (it moves left
# through the tabs), so its label is drawn in the left soft key. The EXECUTOR decides what the
# buttons actually do (command_executor._BTN_ALLOW / _BTN_DENY); the supervisor does not import
# it, so these two lines must be swapped together with those. test_rti_command_wiring.py fails
# if they disagree. Default matches confirm_view: B2 confirms, B1 cancels.
TAKEOVER_B1 = "DENY"
TAKEOVER_B2 = "ALLOW"


def modal_latch_arm():
    """Call BEFORE check_button(): latch the moment a takeover prompt is showing."""
    if pages.data.force_confirm()[0]:
        _modal_latch[0] = True


def modal_latch_release(b1_down, b2_down):
    """Call AFTER check_button(): unlatch only when the prompt is gone and both buttons are up.

    Must run after, not before, check_button(): on the release tick the short-press callback
    fires inside check_button(), and it has to see the latch still set.
    """
    if _modal_latch[0] and not pages.data.force_confirm()[0] and not (b1_down or b2_down):
        _modal_latch[0] = False


def _swallowed():
    return _modal_latch[0]


# ==========================================================================
# Input — tap = move
# ==========================================================================
def on_b1_short():
    if _swallowed():
        return
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
    if _swallowed():
        return
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
    if _swallowed():
        return        # see _modal_latch: this is the one that could reboot the Pi
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
    if _swallowed():
        return
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

    # A forced control takeover is waiting on a button press. This pre-empts every other
    # screen on purpose: it is a safety prompt with a deadline, and a menu the operator
    # happens to be in must not hide it. Industrial arms commonly do the same with a button on
    # the robot; the press is what stops any machine on the network seizing control of an
    # arm someone else is using.
    pending, who, left = pages.data.force_confirm()
    if pending:
        # The requester is "ip:port". The IP is what a person recognises; the port only
        # separates two programs on one PC, so it is the part to drop when space runs out.
        who = who or "unknown"
        if not s.fits(who) and ":" in who:
            who = who.rsplit(":", 1)[0]
        keys = (f"B1 {TAKEOVER_B1}", f"B2 {TAKEOVER_B2}")
        hints = tuple("hold 1s" if k == "ALLOW" else "" for k in (TAKEOVER_B1, TAKEOVER_B2))
        # ceil, not round: never show "0s" while the request is still live.
        s.alert_view("TAKE CONTROL?", f"{math.ceil(max(left, 0.0)):.0f}s",
                     ["Request from:", who], keys, hints)
        return s

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
