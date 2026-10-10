"""The "Time" card in the dashboard's System Info tab (2026-10-09) — which time zone times are SHOWN in.

User: "it is annoying that this shows UTC while I am in Europe/Zagreb; users should be able to select it — and what
without internet?" The zone is `<display_timezone>` in config/system.xml (utility/display_time.py): picked here from the
zone database on the Pi (no internet needed), saved, applied at once to everything the dashboard shows. The card also
says whether the robot's CLOCK is right: synchronised from the internet (NTP), or running on its real-time clock.
"""
from __future__ import annotations

import time

import utility.display_time as dt


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
    state = {"last_status": 0.0}
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
        if time.monotonic() - state["last_status"] > 10.0:
            state["last_status"] = time.monotonic()
            clock_lbl.set_text(clock_text(dt.clock_status()))

    apply_btn.on_click(apply)
    ui.timer(1.0, tick)
    tick()
    return {"select": sel, "apply": apply, "now": now_lbl, "clock": clock_lbl, "tick": tick}
