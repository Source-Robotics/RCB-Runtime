"""The System Info tab's "Sound" card (2026-11-17, utility/sound.py): the five sounds switched on / off (boot, homing
done, an error latched, identify this arm, programs' tones); for each of the robot's own four, the tune it plays, chosen
from the library of 15 with a ▶ to hear it (user: "a list of 15 different sounds ... select what i like the most"); the
playing drive, the robot's own sounds' axis (programs' raw tones always play on d), loudness and voltage cap; the
Identify / program-tone buttons. Every change goes through set_sound (saved in system.xml's <sound> block, used at
once).
"""
from __future__ import annotations

EVENT_LABELS = (("boot", "Ready after start"), ("homing_done", "Homing done"), ("error", "Error latched"),
                ("identify", "Identify this arm"))
SWITCH_LABELS = EVENT_LABELS + (("programs", "Programs' tones"),)
AXES = {"q": "q (louder)", "d": "d (no torque)"}


def drive_options(joint_num: int, has_gripper: bool) -> dict:
    """{node: label} of the drives that can play: J1..J<n>, then a CAN gripper."""
    out = {i: f"J{i + 1}" for i in range(joint_num)}
    if has_gripper:
        out[joint_num] = "Gripper"
    return out


def api_settings(settings: dict) -> dict:
    """The card keeps system.xml's <sound> values, the playing drive as <node> counted from 0; set_sound takes it as
    `drive`, 1 for J1, since command protocol 4 (2026-11-22, review P1)."""
    return {("drive" if k == "node" else k): (int(v) + 1 if k == "node" else v) for k, v in settings.items()}


def tune_options() -> dict:
    """{name: label} of the library's tunes, in the library's order ("ding_dong" -> "ding dong")."""
    from utility.sound import TUNES
    return {name: name.replace("_", " ") for name in TUNES}


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


def send_setting(state: dict, key: str, value, exec_fn) -> bool:
    """Send one setting when it differs from what the robot holds; True when it was sent.

    `state` takes the value only once the robot has taken it — inside the call exec_fn runs, after set_sound returned
    without raising (review C18, 2026-11-21): it was recorded at the click, so a refused change (control held by
    someone else, the executor down) read as saved and the same click again sent nothing.
    """
    if state.get(key) == value:
        return False

    def send(r, k=key, v=value):
        r.set_sound(**api_settings({k: v}))
        state[k] = v

    exec_fn(send, "set_sound")
    return True


def send_changes(state: dict, new: dict, exec_fn) -> dict:
    """The tone settings that differ from what the robot holds, sent in one set_sound; `state` takes them only once the
    robot has (send_setting). Returns what was sent."""
    diff = changed(state, new)
    if diff:
        sent = dict(diff)

        def send(r):
            r.set_sound(**api_settings(sent))
            state.update(sent)

        exec_fn(send, "set_sound")
    return diff


def add_sound_card(settings: dict, joint_num: int, has_gripper: bool, exec_fn, poll_fn):
    """The card. settings = system.xml's <sound> as parsed at page load; exec_fn(fn, label) changes the robot (it takes
    control); poll_fn(fn, label) sends identify_arm, which needs no control."""
    from nicegui import ui

    from utility.sound import DEFAULT_TUNES
    state = dict(settings)

    def set_value(key, value):
        send_setting(state, key, value, exec_fn)

    def save_tone():
        new = {"axis": axis.value, "current_ma": float(current.value or 0), "node": int(node.value),
               "limit_mv": float(limit.value or 0)}
        send_changes(state, new, exec_fn)

    def hear(name):
        exec_fn(lambda r, n=name: r.play_sound(n), "play_sound")

    with ui.card().classes("w-full mt-3"):
        ui.label("Sound").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
        ui.label("Tones played by one drive through its motor. Nothing moves; it plays with the arm enabled or "
                 "disabled, not with the motors off. Each sound is switched on or off here, and each of the robot's "
                 "own sounds plays the tune picked for it (▶ plays it; it needs Programs' tones on). Saved in "
                 "system.xml.").classes("text-xs text-gray-500 mb-2")
        tunes = tune_options()
        for key, text in EVENT_LABELS:
            with ui.row().classes("items-center gap-3"):
                ui.switch(text, value=bool(state.get(key, 1)),
                          on_change=lambda e, k=key: set_value(k, int(bool(e.value)))).classes("w-48")
                pick = ui.select(tunes, value=state.get(f"{key}_tune", DEFAULT_TUNES[key]), label="Tune",
                                 on_change=lambda e, k=key: set_value(f"{k}_tune", e.value)).classes("w-40")
                ui.button(icon="play_arrow", on_click=lambda p=pick: hear(p.value)).props("flat round size=sm")
        ui.switch("Programs' tones (beep, play_tune, play_sound)", value=bool(state.get("programs", 1)),
                  on_change=lambda e: set_value("programs", int(bool(e.value))))
        with ui.row().classes("items-end gap-3 flex-wrap mt-2"):
            node = ui.select(drive_options(joint_num, has_gripper), value=int(state.get("node", 1)),
                             label="Playing drive").classes("w-32")
            with ui.column().classes("gap-0"):
                ui.label("Robot's own sounds on").classes("text-xs text-gray-500")
                axis = ui.toggle(AXES, value=state.get("axis", "q"))
            current = ui.number("Loudness (mA)", value=float(state.get("current_ma", 290)), min=10, max=1000,
                                step=10).classes("w-32")
            limit = ui.number("Voltage cap (mV)", value=float(state.get("limit_mv", 3600)), min=500, max=6000,
                              step=100).classes("w-36")
            ui.button("Save", icon="save", on_click=save_tone).props("outline size=sm")
        with ui.row().classes("items-center gap-3 mt-2"):
            ui.button("Identify this arm", icon="volume_up",
                      on_click=lambda: poll_fn(lambda r: r.identify_arm(), "identify_arm")).props("color=primary size=sm")
            ui.button("Program tone (A4)", icon="music_note",
                      on_click=lambda: exec_fn(lambda r: r.beep(440, 0.4), "beep")).props("outline size=sm")
            ui.label("At rest the robot's own sounds and ▶ play on the axis above; in EXEC, JOG and streaming on d. "
                     "Programs' beep / play_tune always play on d: no torque, so a tone during a move never disturbs "
                     "it or the contact checks.").classes("text-xs text-gray-500")
    return {"state": state, "save_tone": save_tone, "set_value": set_value, "hear": hear}
