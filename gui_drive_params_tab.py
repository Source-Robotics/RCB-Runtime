"""The dashboard's Drive parameters tab (2026-11-16, user: "all actual parameters listed in the GUI";
TODO/driver_110_rtb15_plan.md).

Every drive's parameters as the drive holds them, read on demand: "Read all" (or one drive), and — the user's choice,
on by default, remembered per browser — a read when the tab opens. RTI paces the read inside its poll slot
(utility/drive_params.py: one request every other free slot, the frames per tick unchanged; ~5 s for all drives); this
tab only shows shared memory: the progress, each drive's read time, and per group a table (J1 .. J<n>, the gripper). The
ten values RTI sends every drive from the robot XML (gains and limits) show the XML's value too when they differ.

Changing a value (the user's decisions, the same day), one value at a time, read back after: a gain / limit RTI sends —
"Apply live" (a joint: set_pid_gains) and "Save to XML" (robot / gripper XML, only that number in the text, a backup
first); a drive-only value (utility/drive_params.DRIVE_EDITABLE) — "Write to drive" (its RAM, the arm disabled); and
"Save this drive to its flash" (its whole working config, the arm disabled, after a confirmation). The rest is read-only.
"""
from __future__ import annotations

import math
import time

from utility.drive_params import (ASKED, NO_ANSWER, NOT_READ, PID_KEYS, READ, SLOTS, UNKNOWN, XML_KEYS, catalogue, fmt,
                                  grouped, kind)

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


def editor_actions(number: int, node: int, joint_num: int, enabled: bool) -> dict:
    """Which of the editor's buttons a parameter / drive gets, and why some are off: {"live", "xml", "write", "save",
    "hint"}. RTI-sent: live (joints only: set_pid_gains has no gripper) + xml; drive-only: write (arm disabled);
    save (arm disabled) for every drive."""
    k = kind(number)
    out = {"live": k == "rti" and node < joint_num, "xml": k == "rti", "write": k == "drive" and not enabled,
           "save": not enabled, "hint": ""}
    if k == "rti":
        out["hint"] = ("RTI sends this from the XML at every start: 'Apply live' changes it until the next start, "
                       "'Save to XML' from the next start." if node < joint_num else
                       "RTI sends this from the gripper XML at every start: 'Save to XML' takes effect at the next start.")
    elif k == "drive":
        out["hint"] = ("Into the drive's working memory — kept after a power-off only once the drive is saved to its "
                       "flash." if not enabled else "Disable the arm to write a drive's own values.")
    else:
        out["hint"] = "Read-only here (an id, a measured or calibrated value, a status)."
    return out


def add_drive_params_tab(shared_data, joint_num: int, has_gripper: bool, xml_cfg: dict, poll_fn, is_visible,
                         exec_fn=None, toast_fn=None):
    """The tab. poll_fn(fn, label) sends a RobotClient call (no 'activity': a read is not operating the robot);
    exec_fn(fn, label) one that changes the robot (the page's _exec: it takes control); toast_fn(text, kind) shows a
    message from the sender's thread; is_visible() says whether the tab is shown (it refreshes and auto-reads only
    then)."""
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
    # ---- changing a value (one at a time, read back after) ----
    editable = [n for n in names if kind(n) != "read-only"]
    with ui.card().classes("w-full mt-2"):
        ui.label("Change a value").classes("text-xs font-semibold text-gray-600 uppercase")
        with ui.row().classes("items-end gap-3"):
            e_drive = ui.select({k + 1: label for k, (label, _) in enumerate(cols)}, value=1,
                                label="Drive").classes("w-28")
            e_param = ui.select({n: f"{n}  {names[n]}" for n in editable}, value=editable[0] if editable else None,
                                label="Parameter").classes("w-96")
            e_now = ui.label("").classes("text-sm font-mono")
            e_value = ui.number("New value").classes("w-36")
        with ui.row().classes("items-center gap-2"):
            b_live = ui.button("Apply live", icon="bolt").props("size=sm")
            b_xml = ui.button("Save to XML", icon="save").props("outline size=sm")
            b_write = ui.button("Write to drive", icon="edit").props("size=sm")
            b_save = ui.button("Save this drive to its flash…", icon="memory").props("color=warning outline size=sm")
        e_hint = ui.label("").classes("text-xs text-gray-500")

    def current(node, number):
        rti = getattr(shared_data, "rti", {})
        vals, states = rti.get("drive_params"), rti.get("drive_params_state")
        if vals is None or int(states[node * SLOTS + number]) != READ:
            return None
        return float(vals[node * SLOTS + number])

    def show_editor():
        number, drive = e_param.value, int(e_drive.value or 1)
        if number is None:
            return
        node = cols[drive - 1][1]
        enabled = getattr(shared_data, "robot_state", None) is not None and shared_data.robot_state.get() == "ENABLED"
        acts = editor_actions(int(number), node, joint_num, enabled)
        now = current(node, int(number))
        e_now.set_text(f"now: {fmt(int(number), now) if now is not None else 'not read'}")
        for b, on in ((b_live, acts["live"]), (b_xml, acts["xml"]), (b_write, kind(int(number)) == "drive")):
            b.set_visibility(on)
        b_write.set_enabled(acts["write"])
        b_save.set_enabled(acts["save"])
        e_hint.set_text(acts["hint"])

    def toast(text, level="positive"):
        if toast_fn is not None:
            toast_fn(text, level)

    def reread(drive, after_s=0.5):
        ui.timer(after_s, lambda: read(drive), once=True)

    def chosen():
        if e_value.value is None or e_param.value is None:
            ui.notify("Type the new value first", type="warning")
            return None
        return int(e_drive.value or 1), int(e_param.value), float(e_value.value)

    def do_live():
        c = chosen()
        if c is None or exec_fn is None:
            return
        drive, number, value = c
        exec_fn(lambda r: r.set_pid_gains(drive - 1, **{PID_KEYS[number]: value}), "set_pid_gains")
        reread(drive)

    def do_xml():
        c = chosen()
        if c is None or exec_fn is None:
            return
        drive, number, value = c

        def save(r):
            res = r.save_drive_parameter_to_xml(drive, number, value)
            toast(f"Saved {res['key']} = {res['value']:g} to {res['file']} (backup {res['backup']}) — used from the "
                  f"next start")
        exec_fn(save, "save_drive_parameter_to_xml")

    def do_write():
        c = chosen()
        if c is None or exec_fn is None:
            return
        drive, number, value = c
        exec_fn(lambda r: r.write_drive_parameter(drive, number, value), "write_drive_parameter")

    async def do_save():
        drive = int(e_drive.value or 1)
        label = cols[drive - 1][0]
        with ui.dialog() as dlg, ui.card().classes("max-w-xl"):
            ui.label(f"Save {label}'s drive to its flash?").classes("text-lg font-bold")
            ui.markdown("The drive stores **everything it holds now** — the values written here, and also the gains, "
                        "limits, watchdog and brake setting the robot sent it — and comes up with them at every "
                        "power-up, also without the robot software. It does not answer for up to about a second "
                        "while it saves. Its values are read again afterwards.").classes("text-sm")
            with ui.row().classes("gap-2 justify-end w-full mt-2"):
                ui.button("Cancel", on_click=lambda: dlg.submit(False)).props("outline")
                ui.button(f"Save {label}", on_click=lambda: dlg.submit(True)).props("color=warning")
        if not await dlg or exec_fn is None:
            return
        exec_fn(lambda r: r.save_drive_config(drive), "save_drive_config")
        reread(drive, after_s=3.5)

    b_live.on_click(do_live)
    b_xml.on_click(do_xml)
    b_write.on_click(do_write)
    b_save.on_click(do_save)
    e_drive.on_value_change(lambda _: show_editor())
    e_param.on_value_change(lambda _: show_editor())

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
        show_editor()
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
    return {"refresh": refresh, "auto": auto, "tables": tables, "progress": progress, "label": progress_lbl,
            "editor": {"drive": e_drive, "param": e_param, "value": e_value, "live": b_live, "xml": b_xml,
                       "write": b_write, "save": b_save, "show": show_editor}}
