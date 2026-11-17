"""The System Info tab's "Sound" card (2026-11-17, utility/sound.py): the five sounds switched on / off (boot, homing
done, an error latched, identify this arm, programs' tones), the playing drive, the robot's own sounds' axis (programs'
tones always play on d), loudness and voltage cap, and the Identify / test-tone buttons. Every change goes through
set_sound (saved in system.xml's <sound> block, used at once).
"""
from __future__ import annotations

SWITCH_LABELS = (("boot", "Ready after start"), ("homing_done", "Homing done"), ("error", "Error latched"),
                 ("identify", "Identify this arm"), ("programs", "Programs' tones"))
AXES = {"q": "q (louder)", "d": "d (no torque)"}


def drive_options(joint_num: int, has_gripper: bool) -> dict:
    """{node: label} of the drives that can play: J1..J<n>, then a CAN gripper."""
    out = {i: f"J{i + 1}" for i in range(joint_num)}
    if has_gripper:
        out[joint_num] = "Gripper"
    return out


def changed(before: dict, after: dict) -> dict:
    """The settings whose value differs (numbers compared as numbers)."""
    out = {}
    for key, value in after.items():
        old = before.get(key)
        if isinstance(value, (int, float)) and isinstance(old, (int, float)):
            if float(value) != float(old):
                out[key] = value
        elif value != old:
            out[key] = value
    return out


def add_sound_card(settings: dict, joint_num: int, has_gripper: bool, exec_fn, poll_fn):
    """The card. settings = system.xml's <sound> as parsed at page load; exec_fn(fn, label) changes the robot (it takes
    control); poll_fn(fn, label) sends identify_arm, which needs no control."""
    from nicegui import ui
    state = dict(settings)

    def set_switch(key, value):
        exec_fn(lambda r, k=key, v=int(bool(value)): r.set_sound(**{k: v}), "set_sound")
        state[key] = int(bool(value))

    def save_tone():
        new = {"axis": axis.value, "current_ma": float(current.value or 0), "node": int(node.value),
               "limit_mv": float(limit.value or 0)}
        diff = changed(state, new)
        if diff:
            exec_fn(lambda r, d=dict(diff): r.set_sound(**d), "set_sound")
            state.update(diff)

    with ui.card().classes("w-full mt-3"):
        ui.label("Sound").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
        ui.label("Tones played by one drive through its motor. Nothing moves; it plays with the arm enabled or "
                 "disabled, not with the motors off. Each sound is switched on or off here (saved in system.xml).")\
            .classes("text-xs text-gray-500 mb-2")
        with ui.row().classes("items-center gap-4 flex-wrap"):
            for key, text in SWITCH_LABELS:
                ui.switch(text, value=bool(state.get(key, 1)),
                          on_change=lambda e, k=key: set_switch(k, e.value))
        with ui.row().classes("items-end gap-3 flex-wrap mt-2"):
            node = ui.select(drive_options(joint_num, has_gripper), value=int(state.get("node", 1)),
                             label="Playing drive").classes("w-32")
            with ui.column().classes("gap-0"):
                ui.label("Robot's own sounds on").classes("text-xs text-gray-500")
                axis = ui.toggle(AXES, value=state.get("axis", "q"))
            current = ui.number("Loudness (mA)", value=float(state.get("current_ma", 250)), min=10, max=1000,
                                step=10).classes("w-32")
            limit = ui.number("Voltage cap (mV)", value=float(state.get("limit_mv", 2500)), min=500, max=6000,
                              step=100).classes("w-36")
            ui.button("Save", icon="save", on_click=save_tone).props("outline size=sm")
        with ui.row().classes("items-center gap-3 mt-2"):
            ui.button("Identify this arm", icon="volume_up",
                      on_click=lambda: poll_fn(lambda r: r.identify_arm(), "identify_arm")).props("color=primary size=sm")
            ui.button("Program tone (A4)", icon="music_note",
                      on_click=lambda: exec_fn(lambda r: r.beep(440, 0.4), "beep")).props("outline size=sm")
            ui.label("The robot's own sounds play at rest, on the axis above. Programs' tones always play on d: no "
                     "torque, so a tone during a move never disturbs it or the contact checks.")\
                .classes("text-xs text-gray-500")
    return {"state": state, "save_tone": save_tone, "set_switch": set_switch}
