"""Activity Log tab — the robot's event history (logs/activity_rti.log + logs/activity_exec.log), and the Errors
tab's rows (2026-10-15; moved out of nicegui_test.py, docs/logging.md "The activity log").

  * EVERYTHING is shown by default (user, 2026-10-15); filters narrow it: the lowest level, who asked (origin), a
    code, text (in the code, title, command, message, id and origin), and two switches that hide the dashboard's
    jog commands and a command's queued / started steps. The newest SHOW_ROWS matching entries of the newest
    KEEP entries are shown, newest first.
  * Columns: time (the display zone, to the microsecond — display_time.fmt_precise, as every tab), level, number,
    code, title (utility/error_catalog), origin (who asked: "dashboard (127.0.0.1)", "pick_place.py on lab-pc
    (192.168.0.50)"; the robot / the executor for what nobody asked for), id, command, message. Clicking a row shows
    the whole entry: its cause, effect and fix, its motion context.
  * Entries written before 2026-10-15 have no number, title or origin: number and title come from the catalog by
    code; the origin reads "robot" / "executor" when there was no command, "?" when there was.

Reading is incremental (gui_logs_tab.read_new: byte offsets, the last 512 kB first, a rotated file from its start).
"""
from __future__ import annotations

import json
import os
from collections import deque

from gui_logs_tab import read_new
from utility.error_catalog import entry as catalog_entry

KEEP = 5000
SHOW_ROWS = 500
LEVEL_RANK = {"info": 0, "warning": 1, "error": 2}
JOG_COMMANDS = ("jog_set", "jog_stop")
STEP_CODES = ("CMD_QUEUED", "CMD_STARTED")


# ---------------------------------------------------------------------------------------------------------------
# Pure functions (dummy_test_scripts/test_activity_tab.py)
# ---------------------------------------------------------------------------------------------------------------
def parse_lines(lines):
    """JSON lines -> entries; a line that is not JSON is skipped."""
    out = []
    for line in lines:
        line = line.strip()
        if not line:
            continue
        try:
            e = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(e, dict):
            out.append(e)
    return out


def origin_label(e):
    """Who asked, for display: the entry's origin; else "robot" (RTI) / "executor" (EXEC) / "recorder" (REC) when no
    command caused it, "?" when a command did but the entry predates origins."""
    if e.get("origin"):
        return e["origin"]
    if e.get("id"):
        return "?"
    return {"RTI": "robot", "REC": "recorder"}.get(e.get("source"), "executor")


def number_title(e):
    """(number, title) — the entry's own, else the catalog's by code (entries written before 2026-10-15)."""
    if e.get("number") is not None and e.get("title"):
        return e["number"], e["title"]
    cat = catalog_entry(e.get("code") or "")
    return (cat["number"], cat["title"]) if cat else (None, "")


def matches(e, flt):
    """flt: {"level": None | "warning" | "error" (the lowest shown), "origin": None | label, "code": None | code,
    "text": None | text, "hide_jog": bool, "hide_steps": bool}."""
    if flt.get("level") and LEVEL_RANK.get(e.get("level"), 0) < LEVEL_RANK[flt["level"]]:
        return False
    if flt.get("origin") and origin_label(e) != flt["origin"]:
        return False
    if flt.get("code") and e.get("code") != flt["code"]:
        return False
    if flt.get("hide_steps") and e.get("code") in STEP_CODES:
        return False
    if (flt.get("hide_jog") and e.get("code") == "CMD_EXECUTED"
            and (e.get("command") or "").startswith(JOG_COMMANDS)):
        return False
    text = (flt.get("text") or "").strip().lower()
    if text:
        number, title = number_title(e)
        hay = " ".join(str(x) for x in (number, e.get("code"), title, e.get("command"), e.get("message"), e.get("id"),
                                        origin_label(e)) if x is not None).lower()
        if text not in hay:
            return False
    return True


def row(e, key, fmt_time):
    number, title = number_title(e)
    ts = e.get("timestamp")
    return {"key": key, "time": fmt_time(ts) if isinstance(ts, (int, float)) else "?", "level": e.get("level", "info"),
            "number": "" if number is None else number, "code": e.get("code", ""), "title": title,
            "origin": origin_label(e), "id": e.get("id") or "", "command": e.get("command") or "",
            "message": e.get("message") or ""}


def details(e, fmt_time):
    """The whole entry for the details panel: [(field, text)] — nothing empty."""
    number, title = number_title(e)
    cat = catalog_entry(e.get("code") or "") or {}
    out = [("Time", fmt_time(e["timestamp"]) if isinstance(e.get("timestamp"), (int, float)) else "?"),
           ("Code", f"{number} {e.get('code')}" if number is not None else str(e.get("code"))),
           ("Title", title), ("Level", e.get("level")), ("Origin", origin_label(e)),
           ("Written by", e.get("source")), ("Command id", e.get("id")), ("Command", e.get("command")),
           ("Message", e.get("message"))]
    if e.get("level") in ("warning", "error"):
        out += [("Cause", e.get("cause") or cat.get("cause")), ("Effect", e.get("effect") or cat.get("effect")),
                ("Fix", e.get("fix") or cat.get("fix"))]
    if e.get("context"):
        out.append(("Motion context", ", ".join(f"{k}={v}" for k, v in e["context"].items())))
    return [(k, str(v)) for k, v in out if v not in (None, "")]


def error_view(err, fmt_time):
    """One row of the Errors tab from the live error list (shared memory error_state): the catalog's number, title,
    cause, effect and fix next to the error's own time, tick and description."""
    key = err.get("key", "")
    short = key.split(":", 1)[-1]
    cat = catalog_entry(key) or {}
    where = None
    if key.startswith("J") and ":" in key:
        where = f"joint {key.split(':')[0][1:]}"
    elif key.startswith("Z_") and ":" in key:
        where = f"zone {key.split(':')[0][2:]}"
    ts = err.get("timestamp")
    return {"level": err.get("level", "error"), "code": f"{cat['number']} {short}" if cat else short,
            "title": cat.get("title") or short, "where": where,
            "time": fmt_time(ts) if isinstance(ts, (int, float)) else "?", "tick": err.get("tick", "?"),
            "description": err.get("description", ""),
            "cause": cat.get("cause"), "effect": cat.get("effect"), "fix": cat.get("fix")}


# ---------------------------------------------------------------------------------------------------------------
# The tab
# ---------------------------------------------------------------------------------------------------------------
def add_activity_tab(is_visible, sources, fmt_time):
    """sources: {"RTI": path, "EXEC": path}. Returns its handles (for the net)."""
    from nicegui import ui

    st = {"pos": {}, "entries": deque(maxlen=KEEP), "dirty": True, "shown": [],
          "flt": {"level": None, "origin": None, "code": None, "text": None, "hide_jog": False, "hide_steps": False}}

    def read():
        new = []
        for name, path in sources.items():
            lines, st["pos"][name] = read_new(path, st["pos"].get(name))
            new += parse_lines(lines)
        if new:
            merged = sorted(list(st["entries"]) + new, key=lambda e: e.get("timestamp") or 0.0)
            st["entries"].clear()
            st["entries"].extend(merged[-KEEP:])
            st["dirty"] = True

    with ui.row().classes("items-end gap-3 w-full flex-wrap"):
        level_sel = ui.select({"": "every level", "warning": "warnings and errors", "error": "errors only"},
                              value="", label="Level").classes("w-44")
        origin_sel = ui.select({"": "everyone"}, value="", label="Origin (who asked)", with_input=True).classes("w-64")
        code_sel = ui.select({"": "every code"}, value="", label="Code", with_input=True).classes("w-56")
        text_in = ui.input(label="Search text").classes("w-56")
        jog_sw = ui.switch("Hide jog commands", value=False)
        steps_sw = ui.switch("Hide queued / started steps", value=False)
    note = ui.label("").classes("text-xs text-gray-400")
    columns = [
        {"name": "time", "label": "Time", "field": "time", "align": "left"},
        {"name": "level", "label": "Level", "field": "level", "align": "left"},
        {"name": "number", "label": "No.", "field": "number", "align": "right"},
        {"name": "code", "label": "Code", "field": "code", "align": "left"},
        {"name": "title", "label": "Title", "field": "title", "align": "left"},
        {"name": "origin", "label": "Origin", "field": "origin", "align": "left"},
        {"name": "id", "label": "ID", "field": "id", "align": "left"},
        {"name": "command", "label": "Command", "field": "command", "align": "left"},
        {"name": "message", "label": "Message", "field": "message", "align": "left"},
    ]
    table = ui.table(columns=columns, rows=[], row_key="key").classes("w-full").props("dense flat bordered")
    table.add_slot("body-cell-level", r'''
        <q-td :props="props">
            <q-badge :color="props.value === 'error' ? 'negative' : props.value === 'warning' ? 'orange' : 'grey-6'">
                {{ props.value }}
            </q-badge>
        </q-td>
    ''')
    detail_card = ui.card().classes("w-full")
    with detail_card:
        detail_title = ui.label("Click an entry to see all of it.").classes("text-sm text-gray-500")
        detail_body = ui.column().classes("gap-0 text-sm")

    def show_details(ev):
        try:
            key = int((ev.args[1] if isinstance(ev.args, list) else ev.args)["key"])
            e = st["shown"][key]
        except (KeyError, IndexError, TypeError, ValueError):
            return
        detail_title.text = "Selected entry"
        detail_body.clear()
        with detail_body:
            for name, text in details(e, fmt_time):
                with ui.row().classes("gap-2 no-wrap"):
                    ui.label(name).classes("font-semibold w-32 shrink-0")
                    ui.label(text).classes("whitespace-pre-wrap")

    table.on("rowClick", show_details)

    def filters_changed(_=None):
        st["flt"].update(level=level_sel.value or None, origin=origin_sel.value or None, code=code_sel.value or None,
                         text=(text_in.value or "").strip() or None, hide_jog=bool(jog_sw.value),
                         hide_steps=bool(steps_sw.value))
        st["dirty"] = True
        refresh(force=True)

    for w in (level_sel, origin_sel, code_sel, text_in, jog_sw, steps_sw):
        w.on_value_change(filters_changed)

    def refresh(force=False):
        if not is_visible() and not force:
            return
        read()
        if not st["dirty"]:
            return
        st["dirty"] = False
        entries = list(st["entries"])
        origins = sorted({origin_label(e) for e in entries})
        codes = sorted({e.get("code") for e in entries if e.get("code")})
        origin_sel.set_options({"": "everyone", **{o: o for o in origins}}, value=origin_sel.value or "")
        code_sel.set_options({"": "every code", **{c: (f"{number_title({'code': c})[0]} {c}"
                                                       if number_title({"code": c})[0] is not None else c)
                                                   for c in codes}}, value=code_sel.value or "")
        shown = [e for e in reversed(entries) if matches(e, st["flt"])][:SHOW_ROWS]
        st["shown"] = shown
        table.rows = [row(e, i, fmt_time) for i, e in enumerate(shown)]
        table.update()
        note.text = (f"{len(shown)} shown of {len(entries)} newest entries, newest first. Full history: "
                     f"{', '.join(os.path.basename(p) for p in sources.values())} in the logs folder.")

    ui.timer(1.0, refresh)
    return {"state": st, "refresh": refresh, "table": table, "level": level_sel, "origin": origin_sel,
            "code": code_sel, "text": text_in, "jog": jog_sw, "steps": steps_sw, "details": show_details}
