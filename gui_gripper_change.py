"""The Gripper tab's "Change the gripper" card (2026-11-17, TODO/live_gripper_swap_plan.md P1 + P2a): begin the
Change-gripper mode (the arm disabled and holding its homing), see whether the gripper's drive answers and which
gripper file its tool ID proposes, pick the file (a gripper with a drive; the user confirms) and whether the gripper
left the flange (J5 + J6 to home), end it. Another file: RTI applies it live, the executor / checker / dashboard
restart (this page reloads).

2026-11-18 (the user: "when changing gripper we just do the full home"): Change gripper is one step — with the arm
enabled and homed it moves to the pose a homing ends in (each joint's last move_to target in the homing sequence — the
user: "one where we end up after homing"; homings start there whenever the arm is homed again, J1 ~80 deg from its
endstop in the logs) (a checked joint move at 25 %, QUEUED: a call that waited for the move would
hold a Stop behind it), and once the arm has arrived and stopped, this card's timer disables it and begins the change;
the joints hold at zero speed while the gripper is swapped (the user unplugged the gripper after a move alone: the
gripper's loss un-homed the arm). Done always ends it as a swap (`removed`): the whole arm homes again from there, and
that homing re-learns J5 and J6 for the gripper without asking.
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


def tool_files(grippers_dir) -> dict:
    """{file name: True when the tool has a drive (<CAN_gripper>1)} of every gripper XML (P2b, 2026-11-18: a tool
    without a drive — the Flange, a vacuum tool — is applied live too)."""
    import xml.etree.ElementTree as ET
    out = {}
    for path in sorted(Path(grippers_dir).glob("*.xml")):
        try:
            root = ET.parse(path).getroot()          # nosec B314 — our own files
        except (OSError, ET.ParseError):
            continue
        out[path.stem] = (root.findtext("CAN_gripper") or "").strip() == "1"
    return out


def done_ok(answering: bool, active_drive: bool, picked_drive: bool) -> bool:
    """Done is usable: the fitted drive answers — or no drive is to answer: the active tool has none (RTI hears no
    node 6 then), or the picked file names none (P2b)."""
    return answering or not active_drive or not picked_drive


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


ARRIVED_DEG = 0.5            # the move to the park pose has arrived: every joint this close to its target
AUTO_TIMEOUT_S = 60.0         # ... or the one-step change gives up (the move was stopped, refused or failed)


def homing_end_pose(steps, joint_num: int = 6) -> list | None:
    """The pose a homing ends in (radians): each joint's LAST move_to target in the homing sequence (config/*_homing.xml);
    None for a joint no step moves; None when no step moves any joint."""
    pose: list = [None] * joint_num
    for step in steps or ():
        for m in step.get('move_to') or ():
            j = int(m['joint'])
            if 0 <= j < joint_num:
                pose[j] = float(m['position_rad'])
    return pose if any(v is not None for v in pose) else None


def arrived(joints_deg, target_deg, tol=ARRIVED_DEG) -> bool:
    """Every joint within `tol` of the move's target."""
    return all(abs(float(n) - float(t)) <= tol for n, t in zip(joints_deg, target_deg))


def add_change_card(shared_data, exec_fn, has_gripper: bool, current: str = "", grippers_dir=None, pose_deg=None):
    """The card (any tool since P2b, 2026-11-18; has_gripper = the active tool has a drive). exec_fn(fn, label): the
    page's _exec; current: the active gripper's file;
    pose_deg: where the arm goes first, the pose a homing ends in (degrees, homing_end_pose; None per joint = stays;
    None: no move first — the change begins where the arm stands).
    -> {"refresh": fn}."""
    from nicegui import ui
    if grippers_dir is None:
        from project_paths import GRIPPERS_DIR as grippers_dir
    files = can_gripper_files(grippers_dir)               # the ones with a drive: proposed by its tool ID
    tools = tool_files(grippers_dir)                      # every tool: the list to pick from
    state: dict = {"proposed_for": None, "auto": None, "swapped": False}   # auto: the one-step change under way
    with ui.card().classes("w-full mt-2"):
        ui.label("Change the gripper").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
        text = ui.label("---").classes("text-sm")
        with ui.column().classes("gap-1 mt-1") as pick_box:
            hint = ui.label("").classes("text-xs text-gray-600")
            pick = ui.select({n: (n if d else f"{n} (no drive)") for n, d in tools.items()},
                             value=current if current in tools else None,
                             label="The fitted tool's file").classes("w-72")
        with ui.row().classes("gap-2 mt-1"):
            begin = ui.button("Change gripper", icon="build").props("outline size=sm")
            done = ui.button("Done", icon="check").props("color=primary size=sm")
            home = ui.button("Home the arm", icon="home",
                             on_click=lambda: exec_fn(lambda r: r.home(), "home")).props("color=primary size=sm")

    async def on_done():
        name = pick.value
        other = bool(name) and name != current
        with ui.dialog() as dlg, ui.card().classes("max-w-lg"):
            ui.label(f"Apply {name}?" if other else "Done changing the gripper?").classes("text-lg font-bold")
            ui.label((f"The robot takes {name}'s settings, tool model and homing set-up now and saves it as the active "
                      f"gripper. Then the executor, the collision checker and this dashboard restart: this page reloads "
                      f"within a few seconds, and the robot answers again within about a minute (the executor "
                      f"rebuilds its models). " if other else "")
                     + "The arm keeps holding but is then not homed: enable it and home it again from here. "
                       "That homing takes J5's and J6's new spots for this gripper without asking.").classes("text-sm")
            with ui.row().classes("gap-2 justify-end w-full mt-2"):
                ui.button("Cancel", on_click=lambda: dlg.submit(False)).props("outline")
                ui.button(f"Apply {name}" if other else "Done", on_click=lambda: dlg.submit(True)).props("color=primary")
        if not await dlg:
            return
        state["swapped"] = True
        exec_fn(lambda r, n=(name if other else None): r.end_gripper_change(n, removed=True), "end_gripper_change")

    def _joints_deg():
        import math
        return [math.degrees(float(v)) for v in shared_data.joint["joint_position"][:6]]

    async def on_begin():
        import time
        enabled = shared_data.robot_state.get() == "ENABLED"
        homed = shared_data.homed_state.get() == "HOMED"
        if not enabled or not homed or pose_deg is None:       # disabled (or no pose): begin where it stands
            exec_fn(lambda r: r.begin_gripper_change(), "begin_gripper_change")
            return
        target = [float(n) if p is None else float(p) for n, p in zip(_joints_deg(), pose_deg)]
        pretty = ", ".join(f"J{j + 1} {v:.0f}" for j, v in enumerate(target))
        with ui.dialog() as dlg, ui.card().classes("max-w-lg"):
            ui.label("Change the gripper?").classes("text-lg font-bold")
            ui.label(f"The arm first moves at 25 % speed to the pose a homing ends in ({pretty} deg), checked against "
                     f"its limits, "
                     f"the safety zones and itself. Then it disables itself (every joint keeps holding) and the change "
                     f"begins. Unplug the gripper only when this card says so. Keep the space around the arm clear."
                     ).classes("text-sm")
            with ui.row().classes("gap-2 justify-end w-full mt-2"):
                ui.button("Cancel", on_click=lambda: dlg.submit(False)).props("outline")
                ui.button("Park and begin", on_click=lambda: dlg.submit(True)).props("color=primary")
        if not await dlg:
            return
        state["auto"] = {"target": target, "t0": time.monotonic()}
        if arrived(_joints_deg(), target):                     # already parked: no move
            return
        # Queued, not waited for: the dashboard runs its commands one after the other, and a call that waited for the
        # move would hold a Stop behind it. EXEC first (its switch is a few ticks), then the move; refresh() goes on.
        exec_fn(lambda r: r.enter_exec_mode(wait=True), "enter_exec_mode")
        exec_fn(lambda r, t=target: r.move_joints(*t, speed=25), "move_joints(gripper change)")

    def _auto_step(mode: str, enabled: bool):
        """The one-step change, each refresh: arrived and at rest -> IDLE, disabled, the change begun; the move stopped,
        refused or failed (the arm not in EXEC after 3 s, an error, 60 s) -> given up, said so. Returns the card's text
        while it is under way."""
        import time
        a = state["auto"]
        if a is None:
            return None
        age = time.monotonic() - a["t0"]
        speeds = shared_data.joint["joint_speed_filtered"][:6]
        if arrived(_joints_deg(), a["target"]) and max(abs(float(v)) for v in speeds) < 0.01 and mode in ("EXEC", "IDLE"):
            state["auto"] = None
            if mode == "EXEC":
                exec_fn(lambda r: r.enter_idle_mode(wait=True), "enter_idle_mode")
            if enabled:
                exec_fn(lambda r: r.disable(wait=True), "disable")
            exec_fn(lambda r: r.begin_gripper_change(), "begin_gripper_change")
            return None
        if (age > 3.0 and mode != "EXEC") or age > AUTO_TIMEOUT_S or float(shared_data.rti["error_active"][0]) > 0.5:
            state["auto"] = None
            ui.notify("The move did not finish (stopped, refused or an error): the gripper change did "
                      "not begin — do not unplug the gripper.", type="warning", timeout=10000)
            return None
        return ("Moving to the pose a homing ends in; then the arm disables itself and the change begins. Do NOT "
                "unplug the gripper yet.")

    begin.on_click(on_begin)
    done.on_click(on_done)

    def refresh():
        try:
            changing = float(shared_data.rti["gripper_change"][0]) >= 0.5
            failed = float(shared_data.rti["gripper_change"][0]) >= 1.5
            answering = float(shared_data.rti["CAN_gripper_connected"][0]) >= 0.5
            drive_id = float(shared_data.rti["gripper_tool"][0])
        except (KeyError, IndexError, TypeError):
            return                                # an RTI from before the mode: nothing to show
        enabled = shared_data.robot_state.get() == "ENABLED"
        mode = shared_data.robot_mode.get() or "---"
        homed = shared_data.homed_state.get() == "HOMED"
        view = change_view(changing, answering, enabled, mode)
        picked_drive = tools.get(pick.value or current, True)
        if changing and done_ok(answering, has_gripper, picked_drive) and not answering:
            view = {"text": ("Changing the tool: pick the fitted tool's file and press Done"
                             + (" (it has no drive: nothing to wait for)." if not picked_drive else
                                " — its drive is checked once it answers, after Done.")),
                    "colour": "text-positive", "begin": False, "done": True}
        moving = _auto_step(mode, enabled)
        if moving:
            view = {"text": moving, "colour": "text-warning", "begin": False, "done": False}
        elif not changing and enabled and homed and mode == "IDLE" and pose_deg is not None:
            view = {"text": "Change gripper moves the arm to the pose a homing ends in, then disables it and begins the "
                            "change; unplug the gripper only after that.", "colour": "text-grey-7", "begin": True,
                    "done": False}
        if homed:
            state["swapped"] = False
        rehome = state["swapped"] and not changing and not homed and not failed
        if rehome:
            view = {"text": ("Home the arm again: its homing takes J5's and J6's new spots for "
                             "this gripper." if enabled else "Enable the arm, then home it again here: its homing "
                             "takes J5's and J6's new spots for this gripper."), "colour": "text-warning",
                    "begin": False, "done": False}
        home.set_visibility(rehome)
        home.set_enabled(rehome and enabled and mode == "IDLE")
        if failed:
            view = {"text": "Applying the gripper failed: restart the robot service (the arm cannot be enabled "
                            "until then).", "colour": "text-negative", "begin": False, "done": False}
        text.text = view["text"]
        text.classes(replace=f"text-sm {view['colour']}")
        begin.set_enabled(view["begin"])
        done.set_enabled(view["done"])
        done.set_visibility(changing)
        pick_box.set_visibility(changing and not failed)
        did = None if drive_id < 0 or not has_gripper else int(drive_id)
        props = proposal(did, files)
        hint.text = ("The active tool has no drive: pick the file of the tool you fit." if not has_gripper else
                     f"Its drive's tool ID: {did if did is not None else 'not known yet'}"
                     + (f" — proposed: {', '.join(props)}" if props else
                        (" — no gripper file has this ID" if did not in (None, 0) else "")))
        if changing and props and state["proposed_for"] != did:       # propose once per drive seen
            state["proposed_for"] = did
            pick.value = props[0]

    ui.timer(0.5, refresh)
    return {"refresh": refresh}
