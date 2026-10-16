"""Logs tab — the robot's log files in the dashboard, and the support bundle (2026-10-15).

The third view an operator panel has next to the alarm list (Errors tab) and the event history (Activity Log tab):
the diagnostic logs, for whoever services the robot — without SSH. docs/logging.md "Logs tab".

  * "All services" merges RTI.log, high_level.log, supervisor.log, gui.log and self_collision.log by time; any one
    file can be picked instead, including the processes' captured stdout (<NAME>.out.log: crashes, thread dumps)
    and the block programs' log, shown as raw text.
  * Filters: lowest level, logger (a name from utility/logger_setup.NAMES also matches its children: "rti" shows
    rti.can, rti.homing, ...), text. "Follow" reads what was added each second.
  * Times are shown in the dashboard's <display_timezone>; the files keep UTC. A traceback stays with its record.
  * Log levels (2026-10-15): one logger's level — a name from NAMES, or toppra / can — set to DEBUG / INFO / WARNING
    / ERROR without a restart, in RTI and the executor (through `set_log_level`) and in the dashboard itself (its
    own loggers: gui, config, ...). Not persisted: a restart returns every logger to INFO. "rti.can at DEBUG for
    ten minutes while the problem shows" is the use.
  * Support bundle: a zip of logs/, the incident dumps (temp_data/incidents), the configuration XMLs and a short
    info.txt (code version, robot state, disk, uptime) in temp_data/support/ (the newest 3 kept), downloaded by the
    browser. Refused while the robot is ENABLED — reading ~60 MB disturbs the real-time loop (memory traffic,
    TODO/future_todo.md) — and when the disk has under 200 MB free. Built in its own thread at the lowest priority.

Reading is incremental (each file's byte offset), the first read takes the last 512 kB, a rotated file (smaller
than the offset) is read from its start again — the same rules as the Activity Log tab's tailer.
"""
from __future__ import annotations

import asyncio
import concurrent.futures
import glob
import os
import shutil
import threading
import time
import zipfile
from collections import deque

from project_paths import CONFIG_DIR, GRIPPERS_DIR, LOGS_DIR, PROJECT_ROOT, ROBOTS_DIR, TEMP_DATA_DIR
from utility.logger_setup import LEVEL_NAMES, NAMES, QUIET_LIBRARIES, apply_log_level, parse_line

MERGED = "All services"
# label -> (file name in logs/, structured: True = the logging line format, False = raw text)
FILES = {
    "RTI.log": ("RTI.log", True),
    "high_level.log (executor)": ("high_level.log", True),
    "supervisor.log": ("supervisor.log", True),
    "gui.log (dashboard)": ("gui.log", True),
    "self_collision.log": ("self_collision.log", True),
    "recorder.log": ("recorder.log", True),
    "critical_errors.log": ("critical_errors.log", True),
    "RTI.out.log (stdout: crashes, dumps)": ("RTI.out.log", False),
    "HIGH_LEVEL.out.log": ("HIGH_LEVEL.out.log", False),
    "GUI.out.log": ("GUI.out.log", False),
    "SELF_COLLISION.out.log": ("SELF_COLLISION.out.log", False),
    "RECORDER.out.log": ("RECORDER.out.log", False),
    "block_programs.log": ("block_programs.log", False),
}
MERGED_FILES = ("RTI.log", "high_level.log (executor)", "supervisor.log", "gui.log (dashboard)", "self_collision.log",
                "recorder.log")
LEVELS = {"DEBUG": 10, "INFO": 20, "WARNING": 30, "ERROR": 40, "CRITICAL": 50}
FIRST_READ_BYTES = 512_000
KEEP_RECORDS = 4000            # per file
SHOW_ROWS = 1000
BUNDLE_KEEP = 3
BUNDLE_MIN_FREE_BYTES = 200_000_000


# ---------------------------------------------------------------------------------------------------------------
# Reading — pure functions (dummy_test_scripts/test_logs_tab.py)
# ---------------------------------------------------------------------------------------------------------------
def read_new(path, pos):
    """(new complete lines, next offset). pos None = first read: the last FIRST_READ_BYTES, a partial first line
    dropped. A file smaller than pos was rotated: read it from the start. A last line without its newline yet is
    left for the next call."""
    try:
        size = os.path.getsize(path)
    except OSError:
        return [], pos
    first = pos is None
    if first:
        pos = max(0, size - FIRST_READ_BYTES)
    elif size < pos:
        pos = 0
    if size == pos:
        return [], pos
    try:
        with open(path, "rb") as fh:
            fh.seek(pos)
            data = fh.read(size - pos)
    except OSError:
        return [], pos
    end = data.rfind(b"\n")
    if end < 0:
        return [], pos
    chunk = data[:end + 1]
    lines = chunk.decode("utf-8", errors="replace").splitlines()
    if first and pos > 0 and lines:
        lines = lines[1:]
    return lines, pos + len(chunk)


def group(lines, source, structured, last=None):
    """Lines -> records {"level", "time", "tick", "name", "message", "source"}. A line that does not start a record
    (a traceback's continuation) joins the record before it — `last`, the newest record already kept, for the first
    ones. Raw files: one record per line, level "", time None."""
    out = []
    for line in lines:
        rec = parse_line(line) if structured else None
        if rec is not None:
            rec["source"] = source
            out.append(rec)
            continue
        if structured and (out or last is not None):
            target = out[-1] if out else last
            target["message"] += "\n" + line
            continue
        out.append({"level": "", "time": None, "tick": None, "name": "", "message": line, "source": source})
    return out


def matches(rec, min_level, name, text):
    """min_level: None or a LEVELS key; name: None, or a logger name that also matches its children; text: None or a
    case-insensitive substring of the message. A raw line has no level: it passes only without a level filter."""
    if min_level is not None:
        if LEVELS.get(rec["level"], -1) < LEVELS[min_level]:
            return False
    if name and not (rec["name"] == name or rec["name"].startswith(name + ".")):
        return False
    if text and text.lower() not in rec["message"].lower():
        return False
    return True


def merge_newest(buffers, limit):
    """The newest `limit` records of several per-file buffers, newest first (by time; raw lines keep file order)."""
    recs = [r for b in buffers for r in b]
    recs.sort(key=lambda r: r["time"] if r["time"] is not None else 0.0)
    return recs[::-1][:limit]


# ---------------------------------------------------------------------------------------------------------------
# Support bundle
# ---------------------------------------------------------------------------------------------------------------
_BUNDLE_POOL = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix="support-bundle")


def bundle_refusal(robot_state, free_bytes):
    """Why a bundle cannot be built now, or None."""
    if robot_state == "ENABLED":
        return ("Disable the robot first: building the bundle reads ~60 MB, which can slow the real-time loop "
                "while the arm is powered.")
    if free_bytes < BUNDLE_MIN_FREE_BYTES:
        return f"Not enough free disk space ({free_bytes / 1e6:.0f} MB free, {BUNDLE_MIN_FREE_BYTES / 1e6:.0f} MB needed)."
    return None


def bundle_files():
    """[(path on disk, name in the zip)] — logs, incident dumps, configuration XMLs."""
    out = []
    for p in sorted(glob.glob(os.path.join(str(LOGS_DIR), "*"))):
        if os.path.isfile(p) and not p.endswith(".gitkeep"):
            out.append((p, os.path.join("logs", os.path.basename(p))))
    for p in sorted(glob.glob(os.path.join(str(TEMP_DATA_DIR), "incidents", "*.npz"))):
        out.append((p, os.path.join("incidents", os.path.basename(p))))
    for folder, sub in ((CONFIG_DIR, "config"), (os.path.join(str(CONFIG_DIR), "cells"), "config/cells"),
                        (ROBOTS_DIR, "robots"), (GRIPPERS_DIR, "grippers")):
        for p in sorted(glob.glob(os.path.join(str(folder), "*.xml")) + glob.glob(os.path.join(str(folder), "*.json"))):
            out.append((p, os.path.join(sub, os.path.basename(p))))
    return out


def build_bundle(info_text, out_dir=None, keep=BUNDLE_KEEP):
    """Write the zip (ZIP_DEFLATED, level 1: the Pi's CPU over the last few percent); return its path. Runs in the
    bundle thread at nice 19."""
    try:
        os.setpriority(os.PRIO_PROCESS, threading.get_native_id(), 19)
    except OSError:
        pass
    out_dir = out_dir or os.path.join(str(TEMP_DATA_DIR), "support")
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    path = os.path.join(out_dir, f"support_{stamp}.zip")
    tmp = path + ".part"
    with zipfile.ZipFile(tmp, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=1) as zf:
        zf.writestr("info.txt", info_text)
        for src, name in bundle_files():
            try:
                zf.write(src, name)
            except OSError:
                continue                     # a file rotated away mid-bundle: skip it
    os.replace(tmp, path)
    old = sorted(glob.glob(os.path.join(out_dir, "support_*.zip")))
    for p in old[:-keep] if keep > 0 else []:
        try:
            os.remove(p)
        except OSError:
            pass
    return path


def info_text(robot):
    """info.txt: when, which code, the robot's state, the box."""
    from utility.code_version import describe as describe_code
    lines = [f"support bundle, {time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())} UTC",
             f"code: {describe_code(PROJECT_ROOT)}"]
    lines += [f"{k}: {v}" for k, v in robot.items()]
    du = shutil.disk_usage(str(PROJECT_ROOT))
    lines.append(f"disk: {du.free / 1e9:.2f} GB free of {du.total / 1e9:.1f} GB")
    try:
        lines.append(f"uptime: {float(open('/proc/uptime').read().split()[0]) / 3600:.1f} h")
    except (OSError, ValueError, IndexError):
        pass
    return "\n".join(lines) + "\n"


# ---------------------------------------------------------------------------------------------------------------
# The tab
# ---------------------------------------------------------------------------------------------------------------
def level_request(name, level):
    """(ok, name, level | why) for the Log levels control — apply_log_level's own checks, without applying."""
    name = str(name or "").strip()
    level = str(level or "").strip().upper()
    if name not in NAMES and name not in QUIET_LIBRARIES:
        return False, name, f"unknown logger {name!r}"
    if level not in LEVEL_NAMES:
        return False, name, f"unknown level {level!r}"
    return True, name, level


def add_logs_tab(is_visible, robot_fn, fmt_time, zone_name, level_fn=None):
    """robot_fn() -> {"robot_state": ..., "robot_mode": ..., ...} for the bundle; fmt_time(epoch) -> shown time;
    level_fn(name, level): sends set_log_level to the robot (the executor + RTI; 2026-10-15) — None hides the control;
    zone_name() -> the display zone. Returns its handles (for the net)."""
    from nicegui import ui

    st = {"file": MERGED, "pos": {}, "buf": {}, "dirty": True, "level": None, "name": None, "text": None}

    def _buffer(label):
        return st["buf"].setdefault(label, deque(maxlen=KEEP_RECORDS))

    def _read(label):
        fname, structured = FILES[label]
        lines, st["pos"][label] = read_new(os.path.join(str(LOGS_DIR), fname), st["pos"].get(label))
        if lines:
            buf = _buffer(label)
            buf.extend(group(lines, fname, structured, last=buf[-1] if buf else None))
            st["dirty"] = True

    with ui.row().classes("items-end gap-3 w-full flex-wrap"):
        file_sel = ui.select([MERGED] + list(FILES), value=MERGED, label="Log").classes("w-72")
        level_sel = ui.select({"": "every level", "INFO": "INFO and up", "WARNING": "WARNING and up",
                               "ERROR": "ERROR and up"}, value="", label="Level").classes("w-40")
        name_sel = ui.select({"": "every logger", **{n: n for n in sorted(NAMES)}}, value="", label="Logger",
                             with_input=True).classes("w-48")
        text_in = ui.input(label="Search text").classes("w-56")
        follow = ui.switch("Follow", value=True)
        bundle_btn = ui.button("Support bundle", icon="archive")
    if level_fn is not None:
        with ui.row().classes("items-end gap-2 flex-wrap mb-2"):
            lvl_name = ui.select({**{n: n for n in sorted(NAMES)}, **{n: n for n in sorted(QUIET_LIBRARIES)}},
                                 value="rti", label="Set logger").classes("w-48")
            lvl_level = ui.select(list(LEVEL_NAMES), value="DEBUG", label="to level").classes("w-32")

            def set_level():
                ok, name, text = level_request(lvl_name.value, lvl_level.value)
                if not ok:
                    ui.notify(text, type="warning")
                    return
                apply_log_level(name, text)        # the dashboard's own copy of that logger (gui, config, ...)
                level_fn(name, text)               # RTI's and the executor's, through set_log_level
                ui.notify(f"log level {name} -> {text} (until the next restart)", type="info")

            ui.button("Apply", icon="tune", on_click=set_level).props("flat")
            ui.label("Runtime log levels: RTI, the executor and the dashboard, no restart; back to INFO at the next "
                     "start.").classes("text-xs text-gray-500 max-w-md")
    note = ui.label("").classes("text-xs text-gray-400")
    columns = [
        {"name": "time", "label": "Time", "field": "time", "align": "left"},
        {"name": "level", "label": "Level", "field": "level", "align": "left"},
        {"name": "tick", "label": "Tick", "field": "tick", "align": "right"},
        {"name": "name", "label": "Logger", "field": "name", "align": "left"},
        {"name": "source", "label": "File", "field": "source", "align": "left"},
        {"name": "message", "label": "Message", "field": "message", "align": "left"},
    ]
    table = ui.table(columns=columns, rows=[], row_key="key").classes("w-full").props("dense flat bordered")
    table.add_slot("body-cell-level", r'''
        <q-td :props="props">
            <q-badge v-if="props.value" :color="props.value === 'ERROR' || props.value === 'CRITICAL' ? 'negative'
                : props.value === 'WARNING' ? 'orange' : 'grey-6'">{{ props.value }}</q-badge>
        </q-td>
    ''')
    table.add_slot("body-cell-message", r'''
        <q-td :props="props" style="white-space: pre-wrap; font-family: monospace; font-size: 12px">{{ props.value }}</q-td>
    ''')

    def _filters_changed(_=None):
        st["level"] = level_sel.value or None
        st["name"] = name_sel.value or None
        st["text"] = (text_in.value or "").strip() or None
        st["dirty"] = True
        refresh(force=True)

    def _file_changed(_=None):
        st["file"] = file_sel.value
        st["dirty"] = True
        refresh(force=True)

    file_sel.on_value_change(_file_changed)
    level_sel.on_value_change(_filters_changed)
    name_sel.on_value_change(_filters_changed)
    text_in.on_value_change(_filters_changed)

    def refresh(force=False):
        if not is_visible() and not force:
            return
        labels = MERGED_FILES if st["file"] == MERGED else (st["file"],)
        if follow.value or force:
            for label in labels:
                _read(label)
        if not st["dirty"]:
            return
        st["dirty"] = False
        recs = merge_newest([_buffer(lb) for lb in labels], KEEP_RECORDS * len(labels))
        shown = [r for r in recs if matches(r, st["level"], st["name"], st["text"])][:SHOW_ROWS]
        table.rows = [{"key": i, "time": fmt_time(r["time"]) if r["time"] is not None else "",
                       "level": r["level"], "tick": "" if r["tick"] is None else r["tick"], "name": r["name"],
                       "source": r["source"], "message": r["message"]} for i, r in enumerate(shown)]
        table.update()
        note.text = (f"{len(shown)} of {len(recs)} records shown, newest first. Times in {zone_name()}; the files "
                     f"keep UTC. Folder: {LOGS_DIR}")

    async def _bundle():
        robot = robot_fn()
        why = bundle_refusal(robot.get("robot_state"), shutil.disk_usage(str(PROJECT_ROOT)).free)
        if why:
            ui.notify(why, type="warning", timeout=10000)
            return
        bundle_btn.disable()
        ui.notify("Building the support bundle…", type="info")
        try:
            path = await asyncio.wrap_future(_BUNDLE_POOL.submit(build_bundle, info_text(robot)))
        except Exception as exc:                 # the tab stays usable whatever happens
            ui.notify(f"Support bundle failed: {exc}", type="negative", timeout=10000)
            return
        finally:
            bundle_btn.enable()
        ui.notify(f"Support bundle ready: {os.path.relpath(path, str(PROJECT_ROOT))} "
                  f"({os.path.getsize(path) / 1e6:.1f} MB)", type="positive")
        ui.download(path)

    bundle_btn.on_click(_bundle)
    ui.timer(1.0, refresh)
    return {"state": st, "refresh": refresh, "table": table, "file": file_sel, "level": level_sel,
            "name": name_sel, "text": text_in, "follow": follow, "note": note, "bundle": _bundle}
