"""The Gripper tab's "Tool ID" line (2026-11-17, utility/drive_checks.py): the gripper drive's tool id next to the
gripper XML's <tool_id>, as RTI checks them (rti['gripper_tool']: the drive's, the XML's, the state), and a button that
stores the XML's id in the drive (store_gripper_tool_id: its parameter 1 and a flash save; the arm disabled) with a
confirmation — worded more strongly when the drive already holds another id (user: a mismatch is a stop).
"""
from __future__ import annotations

STATE_NAMES = {0: "none", 1: "unknown", 2: "unset", 3: "match", 4: "mismatch"}     # utility/drive_checks.TOOL_STATES


def tool_line(drive_id, xml_id, state_code) -> dict:
    """What the line shows: {text, colour (a Quasar text class), store (the id the button stores, or None), confirm
    (the confirmation's text, or None)}."""
    from utility.drive_checks import tool_name
    state = STATE_NAMES.get(int(state_code), "unknown")
    d = None if drive_id is None or drive_id < 0 else int(drive_id)
    x = None if xml_id is None or xml_id < 0 else int(xml_id)
    if state == "none" or x is None:
        return {"text": "not checked (the gripper XML declares no <tool_id>)", "colour": "text-grey-7", "store": None,
                "confirm": None}
    if state == "unknown" or d is None:
        return {"text": f"waiting for the drive (the XML: {x})", "colour": "text-grey-7", "store": None, "confirm": None}
    store = (f"Store tool ID {x} ({tool_name(x)}) in the gripper's drive? It writes the ID and saves the drive's flash "
             f"(its whole working config, as Save to drive does); the drive does not answer for about a second. The "
             f"arm must be disabled. Nothing moves.")
    if state == "match":
        return {"text": f"{d} — {tool_name(d)} (matches the XML)", "colour": "text-positive", "store": None,
                "confirm": None}
    if state == "unset":
        return {"text": f"not stored in the drive (the XML: {x}, {tool_name(x)})", "colour": "text-warning",
                "store": x, "confirm": store}
    return {"text": f"the drive says {d} ({tool_name(d)}), the XML {x} ({tool_name(x)}): the arm cannot be enabled",
            "colour": "text-negative", "store": x,
            "confirm": (f"The drive says it is in a gripper with tool ID {d} ({tool_name(d)}). Overwrite it with {x} "
                        f"only if the fitted gripper really is {tool_name(x)}; if it is not, select the gripper XML of "
                        f"the fitted gripper instead. " + store)}


def add_tool_cell(exec_fn):
    """The line's value cell (inside the tab's grid): a label and a Store button. -> {"update": fn(gripper_tool)}."""
    from nicegui import ui
    state = {"store": None, "confirm": None}
    with ui.row().classes("items-center gap-2"):
        lbl = ui.label("---")
        btn = ui.button("Store", icon="save").props("outline size=sm")
        btn.set_visibility(False)

    async def on_store():
        if state["store"] is None:
            return
        with ui.dialog() as dlg, ui.card().classes("max-w-lg"):
            ui.label("Store the gripper's tool ID?").classes("text-lg font-bold")
            ui.label(state["confirm"]).classes("text-sm")
            with ui.row().classes("gap-2 justify-end w-full mt-2"):
                ui.button("Cancel", on_click=lambda: dlg.submit(False)).props("outline")
                ui.button(f"Store {state['store']}", on_click=lambda: dlg.submit(True)).props("color=primary")
        if await dlg:
            exec_fn(lambda r: r.store_gripper_tool_id(), "store_gripper_tool_id")

    btn.on_click(on_store)

    def update(gripper_tool):
        if gripper_tool is None or len(gripper_tool) < 3:
            return
        line = tool_line(float(gripper_tool[0]), float(gripper_tool[1]), float(gripper_tool[2]))
        lbl.text = line["text"]
        lbl.classes(replace=line["colour"])
        state["store"], state["confirm"] = line["store"], line["confirm"]
        btn.text = f"Store {line['store']}" if line["store"] is not None else "Store"
        btn.set_visibility(line["store"] is not None)

    return {"update": update, "label": lbl, "button": btn, "state": state}
