"""The dashboard's Drive parameters tab (2026-11-16, user: "all actual parameters listed in the GUI"; the read only —
writing comes next, TODO/driver_110_rtb15_plan.md).

Every drive's parameters as the drive holds them, read on demand: "Read all" (or one drive), and — the user's choice,
on by default, remembered per browser — a read when the tab opens. RTI paces the read inside its poll slot
(utility/drive_params.py: one request every other free slot, the frames per tick unchanged; ~5 s for all drives); this
tab only shows shared memory: the progress, each drive's read time, and per group a table (J1 .. J<n>, the gripper). The
ten values RTI sends every drive from the robot XML (gains and limits) show the XML's value too when they differ.
"""
from __future__ import annotations

import math
import time

from utility.drive_params import ASKED, NO_ANSWER, NOT_READ, READ, SLOTS, UNKNOWN, XML_KEYS, catalogue, fmt, grouped

AUTO_KEY = "rcb.driveParams.autoRead"           # the browser's localStorage key of the auto-read switch


def drives(joint_num: int, has_gripper: bool) -> list:
    """[(label, node)]: J1 .. J<n>, then the gripper (a CAN gripper only)."""
    return [(f"J{i + 1}", i) for i in range(joint_num)] + ([("Gripper", joint_num)] if has_gripper else [])


def xml_value(xml_cfg: dict, number: int, node: int, joint_num: int):
    """The robot / gripper XML's value of a parameter RTI sends (XML_KEYS), or None."""
    key = XML_KEYS.get(number)
    if key is None or not xml_cfg:
        return None
    try:
        if node < joint_num:
            v = xml_cfg.get(f"motor_{key}")
            v = None if v is None else v[node]
        else:
            v = xml_cfg.get(f"gripper_{key}")
        v = float(v)
    except (TypeError, ValueError, IndexError):
        return None
    return None if math.isnan(v) else v


def cell(number: int, state: int, value: float, xml) -> str:
    """One table cell: the value (and the XML's when it differs), or why there is none."""
    if state == READ:
        text = fmt(number, value)
        if xml is not None and not math.isclose(float(value), xml, rel_tol=1e-6, abs_tol=1e-9):
            text += f"  ≠ XML {fmt(number, xml)}"
        return text
    return {NOT_READ: "—", UNKNOWN: "n/a", NO_ANSWER: "no answer", ASKED: "…"}.get(state, "—")


def rows_for(numbers, names, values, states, cols, xml_cfg, joint_num) -> list:
    """The table rows of one group: #, name, one column per drive."""
    rows = []
    for num in numbers:
        row = {"num": num, "name": names[num]}
        for label, node in cols:
            i = node * SLOTS + num
            row[label] = cell(num, int(states[i]), float(values[i]), xml_value(xml_cfg, num, node, joint_num))
        rows.append(row)
    return rows


def ages(times, cols, now=None) -> str:
    """"J1 read 12 s ago · ..." from each drive's read time (0 = never)."""
    now = time.time() if now is None else now
    out = []
    for label, node in cols:
        t = float(times[node])
        out.append(f"{label} {'never read' if t <= 0 else f'read {max(0, int(now - t))} s ago'}")
    return " · ".join(out)


def add_drive_params_tab(shared_data, joint_num: int, has_gripper: bool, xml_cfg: dict, poll_fn, is_visible):
    """The tab. poll_fn(fn, label) sends a RobotClient call (no 'activity': a read is not operating the robot);
    is_visible() says whether the tab is shown (it refreshes and auto-reads only then)."""
    from nicegui import ui
    names = catalogue()
    groups = grouped(names)
    cols = drives(joint_num, has_gripper)
    state = {"visible": False, "seq": None, "last_rows": 0.0}

    def read(drive=None):
        poll_fn((lambda r: r.read_drive_parameters()) if drive is None
                else (lambda r: r.read_drive_parameters(drive)), "read_drive_parameters")

    ui.label("Drive parameters").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
    ui.label("What each drive holds now, read on demand. RTI asks one parameter every other status poll, so its control "
             "loop keeps its pace (about 5 s for every drive). The gains and limits RTI sends from the robot XML show "
             "the XML's value too when the drive holds another.").classes("text-xs text-gray-500 mb-2")
    with ui.row().classes("items-center gap-3"):
        ui.button("Read all", icon="download", on_click=lambda: read()).props("color=primary size=sm")
        one = ui.select({k + 1: label for k, (label, _) in enumerate(cols)}, value=1, label="Drive").classes("w-28")
        ui.button("Read this drive", icon="download", on_click=lambda: read(int(one.value))).props("outline size=sm")
        auto = ui.switch("Read all when this tab opens", value=True)
    progress = ui.linear_progress(value=0, show_value=False).props("size=8px").classes("w-full mt-2")
    progress_lbl = ui.label("").classes("text-xs text-gray-600")
    ages_lbl = ui.label("").classes("text-xs text-gray-500 mb-2")

    columns = [{"name": "num", "label": "#", "field": "num", "align": "right"},
               {"name": "name", "label": "Parameter", "field": "name", "align": "left"}] \
        + [{"name": label, "label": label, "field": label, "align": "right"} for label, _ in cols]
    tables = []
    for k, (title, numbers) in enumerate(groups):
        with ui.expansion(title, value=k < 3).classes("w-full"):
            tables.append((numbers, ui.table(columns=columns, rows=[], row_key="num")
                           .classes("w-full").props("dense flat")))

    async def load_auto():
        try:
            v = await ui.run_javascript(f"localStorage.getItem('{AUTO_KEY}')", timeout=3.0)
        except Exception:       # noqa: BLE001 — no browser storage: the default stays
            return
        if v in ("0", "1"):
            auto.value = v == "1"

    def save_auto(e):
        ui.run_javascript(f"localStorage.setItem('{AUTO_KEY}', '{1 if e.value else 0}')")

    auto.on_value_change(save_auto)
    ui.timer(0.5, load_auto, once=True)

    def refresh():
        rti = getattr(shared_data, "rti", {})
        prog = rti.get("drive_params_read") if isinstance(rti, dict) else None
        visible = bool(is_visible())
        opened = visible and not state["visible"]
        state["visible"] = visible
        if prog is None or not visible:
            return
        if opened and auto.value and int(prog[0]) != 1:
            read()
        done, total, reading = int(prog[1]), int(prog[2]), int(prog[0]) == 1
        progress.value = (done / total) if total else 0.0
        node = int(prog[3])
        label = next((lab for lab, n in cols if n == node), None)
        progress_lbl.set_text(f"Reading{' ' + label if label else ''}: {done} / {total}" if reading
                              else (f"Read {done} / {total}" if total else "Not read yet"))
        ages_lbl.set_text(ages(rti["drive_params_time"], cols))
        now = time.monotonic()
        seq = (int(prog[4]), int(prog[0]))
        if seq == state["seq"] and not (reading and now - state["last_rows"] >= 1.0) and not opened:
            return
        state["seq"], state["last_rows"] = seq, now
        values, states = rti["drive_params"], rti["drive_params_state"]
        for numbers, table in tables:
            table.rows = rows_for(numbers, names, values, states, cols, xml_cfg, joint_num)
            table.update()

    ui.timer(0.5, refresh)
    return {"refresh": refresh, "auto": auto, "tables": tables, "progress": progress, "label": progress_lbl}
