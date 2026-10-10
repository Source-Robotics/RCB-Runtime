"""The arms homing has learned — a card in the dashboard's System Info tab (2026-10-09, TODO/homing_plan.md H1).

User: "could we have that naming and stuff in the GUI? Users will not have access to a terminal." The same operations as
`tools/homing_arms.py`, on the same file (`temp_data/homing_arms.json`) through the same functions in
`robotics/homing_arms.py`: the last homing's verdict, the learned arms, rename, forget, and accept the last failed homing
after a deliberate mechanical change. Editing is locked while the arm homes (RTI writes the file when a homing ends);
every edit re-reads the file first, so a homing that just ended is never overwritten.
"""
from __future__ import annotations

import os

import robotics.homing_arms as ha
import utility.display_time as dt

HELP = ("Homing recognises which arm is connected from where each joint's endstop is, and FAILS when a joint homes more "
        f"than {ha.TOL_DEG:g}° away from where this arm's joint always homes — a false stall, a skipped belt, a slipping "
        f"coupling. A new arm is learned over its first {ha.KNOWN_AFTER} homings.")


def last_homing(table: dict, gears, gripper_dependent, counts) -> tuple[str, str]:
    """(text, level) for the most recent homing in the table: level "ok" | "learning" | "failed" | "none"."""
    done = [a for a in table["arms"] if a.get("last_utc")]
    newest = max(done, key=lambda a: a["last_utc"]) if done else None
    lf = table.get("last_failed")
    if lf and (newest is None or lf["utc"] >= newest["last_utc"]):
        arm = next((a for a in table["arms"] if a["id"] == lf["arm"]), None)
        offs = ha.last_failed_offs(table, gears, gripper_dependent, counts)
        what = ", ".join(f"J{j + 1} {d:+.2f}°" for j, d in offs) or "a joint"
        return (f"Last homing FAILED ({dt.fmt_utc_text(lf['utc'], '%Y-%m-%d %H:%M:%S %Z')}): {what} away from where {ha.arm_label(arm) if arm else lf['arm']} "
                f"always homes. Home again. If that joint's mechanics were changed on purpose, accept it below.", "failed")
    if newest is None:
        return "No homing recorded yet.", "none"
    if newest["known"]:
        return f"Last homing ({dt.fmt_utc_text(newest['last_utc'], '%Y-%m-%d %H:%M:%S %Z')}): {ha.arm_label(newest)} recognised.", "ok"
    return (f"Last homing ({dt.fmt_utc_text(newest['last_utc'], '%Y-%m-%d %H:%M:%S %Z')}): {ha.arm_label(newest)} — learning, "
            f"{newest['homings']}/{ha.KNOWN_AFTER} homings.", "learning")


def arm_rows(table: dict, gears, counts) -> list[dict]:
    """The rows of the arms table."""
    rows = []
    for a in table["arms"]:
        spots = ha.joint_spots(a, gears, counts)
        rows.append({"id": a["id"], "uid": a.get("uid", "—"), "name": a.get("name") or "—",
                     "status": "known" if a["known"] else f"learning {a['homings']}/{ha.KNOWN_AFTER}",
                     "homings": a["homings"], "last": dt.fmt_utc_text(a.get("last_utc", "")),
                     "spread": f"±{max((s[3] for s in spots), default=0.0):.3f}°"})
    return rows


RESULT = {"known": "recognised", "learning": "learning", "new": "new arm", "relearn": "learning (restarted)", "fault": "FAILED"}


def _num(v, fmt, none="—"):
    return none if v is None else format(v, fmt)


def stats_rows(arm: dict, gears, counts) -> list[dict]:
    """The selected arm's per-joint stats, formatted for the table."""
    return [{"joint": r["joint"], "homings": r["homings"], "within": f"±{r['within_deg']:.3f}°", "sd": f"{r['sd_deg']:.3f}°",
             "worst": _num(r["worst_seen_deg"], ".3f") + ("°" if r["worst_seen_deg"] is not None else ""),
             "two_pass": _num(r["two_pass_ticks"], ".0f"), "relaxed": _num(r["relaxed_ticks"], "+.0f")}
            for r in ha.joint_stats(arm, gears, counts)]


def history_rows(arm: dict, joint_num: int, limit: int = ha.HISTORY) -> list[dict]:
    """The selected arm's homings, newest first: time, result, duration, the joint furthest from its usual spot, and
    every joint's distance from it (deg)."""
    rows = []
    for k, h in enumerate(reversed(arm.get("history", [])[-limit:])):
        devs = h.get("devs", {})
        worst = max(devs.items(), key=lambda kv: abs(kv[1])) if devs else None
        row = {"n": k, "utc": dt.fmt_utc_text(h["utc"]), "result": RESULT.get(h["kind"], h["kind"]),
               "duration": _num(h.get("duration_s"), ".1f") + (" s" if h.get("duration_s") is not None else ""),
               "worst": f"{worst[0]} {worst[1]:+.3f}°" if worst else "—"}
        for j in range(joint_num):
            row[f"J{j + 1}"] = _num(devs.get(f"J{j + 1}"), "+.3f")
        rows.append(row)
    return rows


def add_homing_arms_card(shared_data, cfg: dict, joint_num: int, path: str | None = None):
    """The card. Returns its handles (refresh, the widgets, and the confirm dialogs' actions — for the net)."""
    from nicegui import ui
    from project_paths import TEMP_DATA_DIR
    path = path or os.path.join(str(TEMP_DATA_DIR), "homing_arms.json")
    gears = [float(g) for g in cfg["gear_ratio"][:joint_num]]
    dep = [int(f) for f in cfg["home_offset_gripper_dependent"][:joint_num]]
    counts = [2 ** int(b) for b in cfg["motor_encoder"][:joint_num]]
    prefix = str(cfg.get("robot_name") or "ARM")
    state = {"mtime": None, "table": ha.empty_table(), "homing": None}
    columns = [{"name": "name", "label": "Name", "field": "name", "align": "left"},
               {"name": "uid", "label": "ID", "field": "uid", "align": "left"},
               {"name": "status", "label": "Status", "field": "status", "align": "left"},
               {"name": "homings", "label": "Homings", "field": "homings"},
               {"name": "spread", "label": "Joints repeat within", "field": "spread"},
               {"name": "last", "label": "Last homed", "field": "last", "align": "left"}]

    with ui.card().classes("w-full"):
        ui.label("Arms (homing)").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
        ui.label(HELP).classes("text-xs text-gray-500 mb-2")
        last_lbl = ui.label("").classes("text-sm mb-2")
        table_el = ui.table(columns=columns, rows=[], row_key="id").classes("w-full").props("dense flat")
        with ui.row().classes("items-end gap-3 mt-2"):
            arm_sel = ui.select(options={}, label="Arm").classes("w-48")
            name_in = ui.input("New name").classes("w-48")
            rename_btn = ui.button("Rename", icon="edit")
            forget_btn = ui.button("Forget…", icon="delete").props("color=negative outline")
        ui.label("Joints of the selected arm").classes("text-xs font-semibold text-gray-600 uppercase mt-3")
        ui.label("Repeats within / SD: the spread of its last homings' endstop spots; worst seen: the furthest any homing "
                 "in the history was from the usual spot; two-pass: the gap between homing's two approaches; relaxation: "
                 "how far the joint eased back after the release (motor ticks).").classes("text-xs text-gray-500")
        joints_el = ui.table(columns=[
            {"name": "joint", "label": "Joint", "field": "joint", "align": "left"},
            {"name": "homings", "label": "Homings", "field": "homings"},
            {"name": "within", "label": "Repeats within", "field": "within"},
            {"name": "sd", "label": "SD", "field": "sd"},
            {"name": "worst", "label": "Worst seen", "field": "worst"},
            {"name": "two_pass", "label": "Two-pass (ticks)", "field": "two_pass"},
            {"name": "relaxed", "label": "Relaxation (ticks)", "field": "relaxed"}],
            rows=[], row_key="joint").classes("w-full").props("dense flat")
        ui.label("Homing history of the selected arm (newest first) — each joint's distance from its usual spot")\
            .classes("text-xs font-semibold text-gray-600 uppercase mt-3")
        history_el = ui.table(columns=[
            {"name": "utc", "label": "Time", "field": "utc", "align": "left"},
            {"name": "result", "label": "Result", "field": "result", "align": "left"},
            {"name": "duration", "label": "Duration", "field": "duration"},
            {"name": "worst", "label": "Furthest joint", "field": "worst", "align": "left"}]
            + [{"name": f"J{j + 1}", "label": f"J{j + 1} (°)", "field": f"J{j + 1}"} for j in range(joint_num)],
            rows=[], row_key="n", pagination={"rowsPerPage": 10}).classes("w-full").props("dense flat")
        with ui.row().classes("items-center gap-3 mt-2") as failed_row:
            failed_lbl = ui.label("").classes("text-sm")
            accept_btn = ui.button("Accept as new reference…", icon="published_with_changes").props("color=warning")
        lock_lbl = ui.label("").classes("text-xs text-orange-700")

    def edit(op):
        """Run op(table) -> message on a FRESH read of the file, save, report."""
        if state["homing"]:
            ui.notify("Not while the arm is homing — RTI writes this table when the homing ends.", type="warning")
            return
        table = ha.load_table(path)
        try:
            msg = op(table)
        except KeyError as exc:
            ui.notify(str(exc.args[0]), type="negative")
            return
        ha.save_table(path, table)
        ui.notify(msg, type="positive")
        state["mtime"] = None
        refresh()

    def do_rename():
        if not arm_sel.value or not (name_in.value or "").strip():
            ui.notify("Pick an arm and type a name.", type="warning")
            return
        edit(lambda t: ha.rename(t, arm_sel.value, name_in.value))
        name_in.value = ""

    def confirm(title, text, action):
        with ui.dialog() as dlg, ui.card():
            ui.label(title).classes("text-base font-semibold")
            ui.label(text).classes("text-sm max-w-md")
            with ui.row().classes("justify-end w-full"):
                ui.button("Cancel", on_click=dlg.close).props("flat")
                ui.button("Yes", on_click=lambda: (dlg.close(), action())).props("color=negative")
        dlg.open()

    def forget_now(arm_id):
        edit(lambda t: ha.forget(t, arm_id))

    def accept_now():
        edit(lambda t: ha.accept_last(t, gears, dep, counts))

    def do_forget():
        if not arm_sel.value:
            ui.notify("Pick an arm.", type="warning")
            return
        a = arm_sel.value
        confirm(f"Forget {a}?", "Its homing spots are deleted; the next time this arm homes it is learned again from "
                f"scratch over {ha.KNOWN_AFTER} homings, and until then a false stall or a skipped belt is NOT caught.",
                lambda: forget_now(a))

    def do_accept():
        confirm("Accept the failed homing as the new reference?",
                "Only after a DELIBERATE mechanical change to that joint (a belt tensioned, a motor re-mounted) — and home "
                "once more first: if it fails at the same spot again, the new spot repeats. If nothing was changed, do "
                "NOT accept: the joint stopped somewhere it should not (a stiff spot, a skipped belt).",
                accept_now)

    rename_btn.on_click(do_rename)
    forget_btn.on_click(do_forget)
    accept_btn.on_click(do_accept)

    def refresh():
        homing = (shared_data.robot_mode.get() or "") == "HOMING"
        if homing != state["homing"]:
            state["homing"] = homing
            for b in (rename_btn, forget_btn, accept_btn):
                b.set_enabled(not homing)
            lock_lbl.set_text("Homing in progress — editing is locked until it ends." if homing else "")
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            mtime = 0.0
        if mtime == state["mtime"]:
            return
        state["mtime"] = mtime
        state["table"] = t = ha.load_table(path)
        if ha.ensure_uids(t, prefix) and not homing:      # arms learned before IDs existed get theirs now, once
            ha.save_table(path, t)
            state["mtime"] = os.path.getmtime(path)
        text, level = last_homing(t, gears, dep, counts)
        last_lbl.set_text(text)
        last_lbl.classes(replace="text-sm mb-2 " + {"failed": "text-red-700 font-semibold", "ok": "text-green-700",
                                                    "learning": "text-blue-700", "none": "text-gray-500"}[level])
        table_el.rows = arm_rows(t, gears, counts)
        table_el.update()
        arm_sel.options = {a["id"]: ha.arm_label(a) for a in t["arms"]}
        if arm_sel.value not in arm_sel.options:
            arm_sel.value = next(iter(arm_sel.options), None)
        arm_sel.update()
        show_joints()
        lf = t.get("last_failed")
        failed_row.set_visibility(bool(lf))
        failed_lbl.set_text(f"Stored failed homing: {lf['arm']} at {dt.fmt_utc_text(lf['utc'], '%Y-%m-%d %H:%M:%S %Z')}" if lf else "")

    def show_joints():
        arm = next((a for a in state["table"]["arms"] if a["id"] == arm_sel.value), None)
        joints_el.rows = [] if arm is None else stats_rows(arm, gears, counts)
        joints_el.update()
        history_el.rows = [] if arm is None else history_rows(arm, joint_num)
        history_el.update()

    arm_sel.on_value_change(lambda _: show_joints())
    ui.timer(1.0, refresh)
    return {"refresh": refresh, "table": table_el, "last": last_lbl, "select": arm_sel, "name": name_in,
            "rename": do_rename, "forget_now": forget_now, "accept_now": accept_now, "failed_row": failed_row,
            "buttons": (rename_btn, forget_btn, accept_btn), "joints": joints_el, "history": history_el, "state": state}
