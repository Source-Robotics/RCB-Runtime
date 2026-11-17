"""The Gripper tab's "Change the gripper" card (2026-11-17, TODO/live_gripper_swap_plan.md P1): begin the
Change-gripper mode (the arm disabled and holding its homing), see whether the gripper's drive answers, end it.
For now the same kind of gripper is fitted again (the robot keeps the active gripper XML; P2 changes it live).
"""
from __future__ import annotations


def change_view(changing: bool, answering: bool, enabled: bool, mode: str) -> dict:
    """What the card shows: {text, colour (a Quasar text class), begin, done (whether each button is usable)}."""
    if changing:
        if answering:
            return {"text": "Changing the gripper: it answers. Press Done, then calibrate its jaws (Calibrate).",
                    "colour": "text-positive", "begin": False, "done": True}
        return {"text": "Changing the gripper: the arm holds and keeps its homing. Unplug the gripper and fit it "
                       "again; it does not answer yet.", "colour": "text-warning", "begin": False, "done": False}
    if enabled:
        return {"text": "Disable the arm to change the gripper (its joints keep holding).", "colour": "text-grey-7",
                "begin": False, "done": False}
    if mode != "IDLE":
        return {"text": f"The gripper is changed in IDLE (now {mode}).", "colour": "text-grey-7", "begin": False,
                "done": False}
    return {"text": "Change the gripper without losing the homing: the arm keeps holding while you unplug it and fit "
                    "it again.", "colour": "text-grey-7", "begin": True, "done": False}


def add_change_card(shared_data, exec_fn, has_gripper: bool):
    """The card (a CAN gripper only). exec_fn(fn, label): the page's _exec. -> {"refresh": fn}."""
    from nicegui import ui
    if not has_gripper:
        return None
    with ui.card().classes("w-full mt-2"):
        ui.label("Change the gripper").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
        text = ui.label("---").classes("text-sm")
        with ui.row().classes("gap-2 mt-1"):
            begin = ui.button("Change gripper", icon="build",
                              on_click=lambda: exec_fn(lambda r: r.begin_gripper_change(), "begin_gripper_change"))\
                .props("outline size=sm")
            done = ui.button("Done", icon="check",
                             on_click=lambda: exec_fn(lambda r: r.end_gripper_change(), "end_gripper_change"))\
                .props("color=primary size=sm")

    def refresh():
        try:
            changing = float(shared_data.rti["gripper_change"][0]) >= 0.5
            answering = float(shared_data.rti["CAN_gripper_connected"][0]) >= 0.5
        except (KeyError, IndexError, TypeError):
            return                                # an RTI from before the mode: nothing to show
        view = change_view(changing, answering, shared_data.robot_state.get() == "ENABLED",
                           shared_data.robot_mode.get() or "---")
        text.text = view["text"]
        text.classes(replace=f"text-sm {view['colour']}")
        begin.set_enabled(view["begin"])
        done.set_enabled(view["done"])
        done.set_visibility(changing)

    ui.timer(0.5, refresh)
    return {"refresh": refresh}
