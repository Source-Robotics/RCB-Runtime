"""The Gripper tab's "Change the gripper" card (2026-11-17, TODO/live_gripper_swap_plan.md P1 + P2a): begin the
Change-gripper mode (the arm disabled and holding its homing), see whether the gripper's drive answers and which
gripper file its tool ID proposes, pick the file (a gripper with a drive; the user confirms) and whether the gripper
left the flange (J5 + J6 to home), end it. Another file: RTI applies it live, the executor / checker / dashboard
restart (this page reloads).
"""
from __future__ import annotations

from pathlib import Path


def can_gripper_files(grippers_dir) -> dict:
    """{file name: its <tool_id> or None} of the gripper XMLs with a drive (<CAN_gripper>1)."""
    import xml.etree.ElementTree as ET
    out = {}
    for path in sorted(Path(grippers_dir).glob("*.xml")):
        try:
            root = ET.parse(path).getroot()          # nosec B314 — our own files
        except (OSError, ET.ParseError):
            continue
        if (root.findtext("CAN_gripper") or "").strip() != "1":
            continue
        tid = (root.findtext("tool_id") or "").strip()
        out[path.stem] = int(tid) if tid.isdigit() else None
    return out


def proposal(drive_id, files: dict) -> list:
    """The files whose tool ID is the fitted drive's (none for an unknown drive or one with 0)."""
    if drive_id is None or drive_id <= 0:
        return []
    return [name for name, tid in files.items() if tid == drive_id]


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


def add_change_card(shared_data, exec_fn, has_gripper: bool, current: str = "", grippers_dir=None):
    """The card (a CAN gripper only). exec_fn(fn, label): the page's _exec; current: the active gripper's file.
    -> {"refresh": fn}."""
    from nicegui import ui
    if not has_gripper:
        return None
    if grippers_dir is None:
        from project_paths import GRIPPERS_DIR as grippers_dir
    files = can_gripper_files(grippers_dir)
    state = {"proposed_for": None}
    with ui.card().classes("w-full mt-2"):
        ui.label("Change the gripper").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
        text = ui.label("---").classes("text-sm")
        with ui.column().classes("gap-1 mt-1") as pick_box:
            hint = ui.label("").classes("text-xs text-gray-600")
            pick = ui.select({n: n for n in files}, value=current if current in files else None,
                             label="The fitted gripper's file").classes("w-72")
            # unticked by default (user 2026-11-18): the usual unplug and re-plug keeps every joint's homing;
            # ticking it asks first at Done (J5 and J6 lose theirs)
            removed = ui.checkbox("It was taken off the flange (J5 and J6 to home after)", value=False)
        with ui.row().classes("gap-2 mt-1"):
            begin = ui.button("Change gripper", icon="build",
                              on_click=lambda: exec_fn(lambda r: r.begin_gripper_change(), "begin_gripper_change"))\
                .props("outline size=sm")
            done = ui.button("Done", icon="check").props("color=primary size=sm")

    async def on_done():
        name, gone = pick.value, bool(removed.value)
        other = bool(name) and name != current
        if other or gone:
            keeps = "the homing of J1 to J4; J5 and J6 must be homed again" if gone else "the homing of every joint"
            with ui.dialog() as dlg, ui.card().classes("max-w-lg"):
                if other:
                    ui.label(f"Apply {name}?").classes("text-lg font-bold")
                    ui.label(f"The robot takes {name}'s settings, tool model and homing set-up now and saves it as "
                             f"the active gripper. Then the executor, the collision checker and this dashboard "
                             f"restart (about 10 s; this page reloads). The arm keeps holding and keeps {keeps}."
                             ).classes("text-sm")
                else:
                    ui.label("Home J5 and J6 again?").classes("text-lg font-bold")
                    ui.label("You ticked that the gripper was taken off the flange: J6's Hall magnet moved with it, so "
                             "J5 and J6 lose their homing and must be homed before the arm moves. The arm keeps "
                             "holding and keeps the homing of J1 to J4.").classes("text-sm")
                with ui.row().classes("gap-2 justify-end w-full mt-2"):
                    ui.button("Cancel", on_click=lambda: dlg.submit(False)).props("outline")
                    ui.button(f"Apply {name}" if other else "Yes, it was taken off",
                              on_click=lambda: dlg.submit(True)).props("color=primary")
            if not await dlg:
                return
        if other:
            exec_fn(lambda r, n=name, g=gone: r.end_gripper_change(n, removed=g), "end_gripper_change")
        else:
            exec_fn(lambda r, g=gone: r.end_gripper_change(None, removed=g), "end_gripper_change")

    done.on_click(on_done)

    def refresh():
        try:
            changing = float(shared_data.rti["gripper_change"][0]) >= 0.5
            failed = float(shared_data.rti["gripper_change"][0]) >= 1.5
            answering = float(shared_data.rti["CAN_gripper_connected"][0]) >= 0.5
            drive_id = float(shared_data.rti["gripper_tool"][0])
        except (KeyError, IndexError, TypeError):
            return                                # an RTI from before the mode: nothing to show
        view = change_view(changing, answering, shared_data.robot_state.get() == "ENABLED",
                           shared_data.robot_mode.get() or "---")
        if failed:
            view = {"text": "Applying the gripper failed: restart the robot service (the arm cannot be enabled "
                            "until then).", "colour": "text-negative", "begin": False, "done": False}
        text.text = view["text"]
        text.classes(replace=f"text-sm {view['colour']}")
        begin.set_enabled(view["begin"])
        done.set_enabled(view["done"])
        done.set_visibility(changing)
        pick_box.set_visibility(changing and not failed)
        did = None if drive_id < 0 else int(drive_id)
        props = proposal(did, files)
        hint.text = (f"Its drive's tool ID: {did if did is not None else 'not known yet'}"
                     + (f" — proposed: {', '.join(props)}" if props else
                        (" — no gripper file has this ID" if did not in (None, 0) else "")))
        if changing and props and state["proposed_for"] != did:       # propose once per drive seen
            state["proposed_for"] = did
            pick.value = props[0]

    ui.timer(0.5, refresh)
    return {"refresh": refresh}
