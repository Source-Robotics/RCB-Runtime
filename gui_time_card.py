"""The "Time" card in the dashboard's System Info tab (2026-10-09) — which time zone times are SHOWN in.

User: "it is annoying that this shows UTC while I am in Europe/Zagreb; users should be able to select it — and what
without internet?" The zone is `<display_timezone>` in config/system.xml (utility/display_time.py): picked here from the
zone database on the Pi (no internet needed), saved, applied at once to everything the dashboard shows. The card also
says whether the robot's CLOCK is right: synchronised from the internet (NTP), or running on its real-time clock.
"""
from __future__ import annotations

import threading
import time
from typing import Any

import utility.display_time as dt

# The clock's state (`timedatectl`: a subprocess, up to 2 s) is read OFF the dashboard's event loop and once per
# PROCESS every CLOCK_REFRESH_S — not on the loop by every open page every 10 s (review G8, 2026-11-21). This module
# is imported once, so the cache is shared by every page load.
CLOCK_REFRESH_S = 10.0
_clock: dict[str, Any] = {"status": {}, "at": None, "running": False}
_clock_lock = threading.Lock()


def _refresh_clock():
    try:
        status = dt.clock_status()
    except Exception:                    # never let the reader thread die with the flag set
        status = {}
    with _clock_lock:
        _clock["status"] = status
        _clock["running"] = False


def cached_clock_status(now: float | None = None) -> dict:
    """The last clock status read ({} until the first read is back); starts a background read when the last one is
    CLOCK_REFRESH_S old and none is running. Never blocks on the subprocess."""
    now = time.monotonic() if now is None else now
    with _clock_lock:
        due = not _clock["running"] and (_clock["at"] is None or now - _clock["at"] >= CLOCK_REFRESH_S)
        if due:
            _clock["running"] = True
            _clock["at"] = now
        status = dict(_clock["status"])
    if due:
        threading.Thread(target=_refresh_clock, name="gui-clock-status", daemon=True).start()
    return status


def clock_text(status: dict) -> str:
    if status.get("synced") is True:
        return "Clock: synchronised from the internet (NTP)."
    rtc = status.get("rtc")
    if status.get("synced") is False:
        return ("Clock: NOT synchronised — no internet time. It runs on the Pi's real-time clock"
                + (", which keeps time through a power cut only with its coin battery fitted; check it is right."
                   if rtc else " — none found: after a power cut the time may be wrong."))
    return "Clock: synchronisation unknown."


def add_time_card(system_xml_path: str):
    """The card. Returns its handles (for the net)."""
    from nicegui import ui
    zones = ["system"] + dt.all_zones()
    with ui.card().classes("flex-1 min-w-64"):
        ui.label("Time").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
        now_lbl = ui.label("").classes("font-mono text-sm")
        with ui.row().classes("items-end gap-2"):
            sel = ui.select(options={z: (f"system (the Pi's own: {dt.system_zone_name()})" if z == "system" else z) for z in zones},
                            value=dt.zone_name(), with_input=True, label="Show times in").classes("w-64")
            apply_btn = ui.button("Apply", icon="schedule")
        clock_lbl = ui.label("").classes("text-xs text-gray-500")
        ui.label("Logs keep UTC inside; the dashboard and the tools show them in this zone.").classes("text-xs text-gray-400")

    def apply():
        try:
            dt.save_to_system_xml(system_xml_path, sel.value)
        except (ValueError, OSError) as exc:
            ui.notify(str(exc), type="negative")
            return
        ui.notify(f"Times are shown in {dt.zone_name()} now.", type="positive")
        tick()

    def tick():
        now_lbl.set_text("Robot time: " + dt.fmt(time.time(), "%Y-%m-%d %H:%M:%S %Z")
                         + (f"  ({dt.zone_name()})" if dt.zone_name() != "system" else f"  ({dt.system_zone_name()})"))
        text = clock_text(cached_clock_status())          # the process's cache: no subprocess here (review G8)
        if clock_lbl.text != text:
            clock_lbl.set_text(text)

    apply_btn.on_click(apply)
    ui.timer(1.0, tick)
    tick()
    return {"select": sel, "apply": apply, "now": now_lbl, "clock": clock_lbl, "tick": tick}
