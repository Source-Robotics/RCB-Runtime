"""The Hall sensor check — a card in the dashboard's I/O tab (2026-10-13, user: "check the Hall sensor even when not
homed or enabled: J6 at 0 current, spin it by hand, see if the magnet triggers the sensor").

Start / Stop go through the executor (`robot.hall_check`, IDLE only); RTI does the rest (robotics/hall_check.py) and
publishes the pin, the number of changes and where they happened in rti['hall_check_*']. This card only reads them.
"""
from __future__ import annotations

import math

from nicegui import ui

HELP = (
    "Makes the joint homed on a Hall sensor (J6) limp — its drive's current limit is set to 0 — and reads its Hall "
    "sensor every tick. Turn the joint slowly by hand through the magnet and watch the badge: it should change when the "
    "magnet passes and change back after it. Works in IDLE, enabled or not, homed or not; anything else (homing, a "
    "jog, a program, an error, the E-stop) stops it first and gives the joint its torque back. With a gripper whose "
    "weight is off its axis the joint may turn by itself once limp — hold it. Stay within the joint's range (its "
    "cable). The sensor's output is HIGH with no magnet (its normal state in the drive firmware) and LOW at the magnet."
)


def add_hall_check_card(shared_data, submit, joint_index):
    """Build the card (call inside the I/O tab). `submit(fn, label)` runs fn(robot) on the dashboard's executor
    connection; `joint_index` is the Hall-sensor joint (0-based) or None. Returns refresh()."""
    name = f"J{joint_index + 1}" if joint_index is not None else "—"
    with ui.card().classes("flex-1 min-w-72"):
        with ui.row().classes("items-center gap-2"):
            ui.label(f"Hall sensor check ({name})").classes(
                "text-sm font-semibold text-gray-600 uppercase tracking-wide")
            with ui.icon("help_outline").classes("text-gray-400 cursor-help text-sm"):
                ui.tooltip(HELP).classes("max-w-md")
        with ui.row().classes("gap-2 items-center"):
            start_btn = ui.button("Start", on_click=lambda: submit(lambda r: r.hall_check(True), "hall_check"))
            stop_btn = ui.button("Stop", on_click=lambda: submit(lambda r: r.hall_check(False), "hall_check"))
            start_btn.props("dense color=primary")
            stop_btn.props("dense color=grey")
            state_lbl = ui.label("—").classes("text-sm text-gray-600")
        with ui.grid(columns=2).classes("gap-x-4 gap-y-2 font-mono text-sm items-center mt-2"):
            ui.label("Sensor")
            magnet_badge = ui.badge("—").props("color=grey outline")
            ui.label("Changes")
            changes_lbl = ui.label("—")
            ui.label("Switched")
            edges_lbl = ui.label("—")
    if joint_index is None:
        start_btn.disable()
        stop_btn.disable()
        state_lbl.text = "this robot has no joint with a Hall sensor (<has_hall_sensor>)"

    def refresh():
        if joint_index is None:
            return
        rti = shared_data.rti
        active = int(rti["hall_check_joint"][0]) > 0
        idle = shared_data.robot_mode.get() == "IDLE"
        start_btn.set_enabled(idle and not active)
        stop_btn.set_enabled(active)
        if not active:
            state_lbl.text = "off" if idle else "off — only in IDLE"
            magnet_badge.text = "—"
            magnet_badge.props("color=grey outline")
            changes_lbl.text = "—"
            edges_lbl.text = "—"
            return
        state_lbl.text = f"ON — {name} is limp, turn it by hand"
        pin = int(rti["hall_check_pin"][0])
        if pin < 0:
            magnet_badge.text = "reading…"
            magnet_badge.props("color=grey outline")
        elif pin == 0:
            magnet_badge.text = "MAGNET (pin LOW)"
            magnet_badge.props("color=positive")
        else:
            magnet_badge.text = "no magnet (pin HIGH)"
            magnet_badge.props("color=grey outline")
        changes_lbl.text = str(int(rti["hall_check_changes"][0]))
        low, high = float(rti["hall_check_edge_deg"][0]), float(rti["hall_check_edge_deg"][1])
        parts = []
        if not math.isnan(low):
            parts.append(f"to LOW at {low:.1f}°")
        if not math.isnan(high):
            parts.append(f"to HIGH at {high:.1f}°")
        if parts and not math.isnan(low) and not math.isnan(high):
            parts.append(f"(apart {abs(high - low):.1f}°)")
        if parts and shared_data.homed_state.get() != "HOMED":
            parts.append("— not homed: relative angles")
        edges_lbl.text = " ".join(parts) if parts else "—"

    return refresh
