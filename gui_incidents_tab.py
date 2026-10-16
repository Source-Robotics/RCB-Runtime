"""Recordings tab — captures on request and incidents around a fault (2026-10-15; 2026-10-16: made by recorder.py;
split into two sub-tabs the same day, user: "a capture runs whenever I want, an incident has a specific purpose").

A capture is every per-tick channel from `capture_before_s` (5 s) before a request to 1-60 s after it, asked for by
the operator (here) or a program (`robot.capture_telemetry`); an incident dump is the same channels from `before_s`
(5 s) before a hard error latched to `after_s` (2 s) after it, made automatically. The recorder process writes both
(utility/trace_recorder.py); the files are the same kind.

The tab ("Recordings"; the recorder's state on top):
  * sub-tab Captures: Capture now (length, name), while one runs its progress and "Stop & save"
    (`robot.stop_capture`), the capture settings (seconds before the request, files kept), the captures, newest first;
  * sub-tab Incidents: what the recorder did (files written, not saved by the rate limit or for too little history),
    the incident settings (seconds before / after a fault, files kept, the rate limit), the incidents, newest first;
  * the viewer beside them, for a file selected in either list: any of its channels (per joint, the TCP's, the loop's
    timing, ...), the fault's or the request's tick marked, the mode changes; Download, Delete.
Settings: config/system.xml <recorder>, utility/recorder_settings.py; the recorder applies a change within ~1 s.
docs/logging.md "Recordings tab".

Pure functions first (dummy_test_scripts/test_incidents_tab.py), the NiceGUI part at the end.
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np

from project_paths import SYSTEM_XML, TEMP_DATA_DIR
from utility import recorder_settings
from utility.incident_dump import CAPTURE_MAX_S, CAPTURE_MIN_S, list_files as _list_files, load as _load, safe_capture_name

INCIDENTS_DIR = TEMP_DATA_DIR / "incidents"
CAPTURES_DIR = TEMP_DATA_DIR / "captures"
MAX_POINTS = 2000              # per series: a 60 s capture has 15 000 ticks; the browser needs no more than this
MAX_PLOTS = 8                  # charts shown at once
LIST_REFRESH_S = 3.0
STATUS_REFRESH_S = 0.5
STATUS_STALE_S = 3.0           # the recorder writes its status at least once a second

DEG = 180.0 / np.pi
_POSE_SCALE = (1000.0, 1000.0, 1000.0, DEG, DEG, DEG)           # m -> mm, rad -> deg
_POSE = ("x", "y", "z", "rx", "ry", "rz")
_TWIST = ("vx", "vy", "vz", "wx", "wy", "wz")
_ACCEL = ("ax", "ay", "az", "alpha x", "alpha y", "alpha z")
_WRENCH = ("fx", "fy", "fz", "mx", "my", "mz")
# key -> (title, unit, scale: one number or one per column, column names: None = the joints J1..Jn)
# The channels of RTI's trace (utility/data_init.process_joint_data's history entry + the loop's scalars); a channel
# not listed here is still offered, under its own name.
CHANNELS = {
    "j_p": ("Position", "deg", DEG, None),
    "j_s_f": ("Speed", "deg/s", DEG, None),
    "j_s": ("Speed (raw)", "deg/s", DEG, None),
    "j_a_f": ("Acceleration", "deg/s²", DEG, None),
    "j_a": ("Acceleration (raw)", "deg/s²", DEG, None),
    "j_j_f": ("Jerk", "deg/s³", DEG, None),
    "j_j": ("Jerk (raw)", "deg/s³", DEG, None),
    "j_t": ("Torque", "Nm", 1.0, None),
    "j_t_f": ("Torque (filtered)", "Nm", 1.0, None),
    "t_ext": ("External torque", "Nm", 1.0, None),
    "j_pc": ("Commanded position", "deg", DEG, None),
    "j_sc": ("Commanded speed", "deg/s", DEG, None),
    "j_ac": ("Commanded acceleration", "deg/s²", DEG, None),
    "j_tc": ("Commanded torque", "Nm", 1.0, None),
    "j_pt": ("Target position", "deg", DEG, None),
    "j_st": ("Target speed", "deg/s", DEG, None),
    "j_at": ("Target acceleration", "deg/s²", DEG, None),
    "j_tt": ("Target torque", "Nm", 1.0, None),
    "j_t_inertia": ("Inertia feedforward", "Nm", 1.0, None),
    "j_t_coriolis": ("Coriolis feedforward", "Nm", 1.0, None),
    "j_err": ("Motor error bytes", "", 1.0, "motors"),
    "TCP_p": ("TCP pose", "mm, deg", _POSE_SCALE, _POSE),
    "TCP_p_c": ("Commanded TCP pose", "mm, deg", _POSE_SCALE, _POSE),
    "TCP_p_t": ("Target TCP pose", "mm, deg", _POSE_SCALE, _POSE),
    "TCP_s_f": ("TCP speed", "mm/s, deg/s", _POSE_SCALE, _TWIST),
    "TCP_s": ("TCP speed (raw)", "mm/s, deg/s", _POSE_SCALE, _TWIST),
    "TCP_s_c": ("Commanded TCP speed", "mm/s, deg/s", _POSE_SCALE, _TWIST),
    "TCP_s_t": ("Target TCP speed", "mm/s, deg/s", _POSE_SCALE, _TWIST),
    "TCP_a_f": ("TCP acceleration", "mm/s², deg/s²", _POSE_SCALE, _ACCEL),
    "TCP_a": ("TCP acceleration (raw)", "mm/s², deg/s²", _POSE_SCALE, _ACCEL),
    "TCP_f_ext": ("External force at the TCP", "N, Nm", 1.0, _WRENCH),
    "TCP_f_ext_tool": ("External force, tool frame", "N, Nm", 1.0, _WRENCH),
    "TCP_p_quat": ("TCP orientation (quaternion)", "", 1.0, ("w", "x", "y", "z")),
    "TCP_p_SE3": ("TCP pose matrix", "", 1.0, tuple(f"r{i}{j}" for i in range(4) for j in range(4))),
    "exec_t": ("Loop work per tick", "ms", 1000.0, ("work",)),
    "period_ms": ("Loop period", "ms", 1.0, ("period",)),
    "rti_ok": ("RTI link success rate", "", 1.0, ("success",)),
    "rti_miss": ("RTI consecutive misses", "", 1.0, ("misses",)),
    "rti_state": ("RTI link state", "", 1.0, ("state",)),
    "rti_seq": ("RTI packet sequence", "", 1.0, ("seq",)),
    "can_read_t": ("CAN read time", "s", 1.0, ("t",)),
    "timestamp": ("Tick time stamp", "s", 1.0, ("t",)),
}
DEFAULT_KEYS = ("j_p", "j_s_f", "j_t", "t_ext")
NOT_PLOTTED = frozenset({"tick", "mode"})       # the x axis; the mode is text (listed in the file's line)
PLOTS = tuple((CHANNELS[k][0], k, CHANNELS[k][1], CHANNELS[k][2]) for k in DEFAULT_KEYS)   # the default charts


# ---------------------------------------------------------------------------------------------------------------
def file_rows(files, fmt_time):
    """Table rows for list_files()'s entries: kind, time (through the dashboard's formatter), name, size."""
    return [{"path": f["path"], "kind": f["kind"], "time": fmt_time(f["time"]), "name": f["name"],
             "size": f"{f['size'] / 1e6:.2f} MB"} for f in files]


def downsample(values, max_points=MAX_POINTS):
    """Every k-th row of a (ticks, ...) array so at most `max_points` remain; the whole array when it is short."""
    values = np.asarray(values)
    k = max(1, int(np.ceil(len(values) / max_points))) if max_points > 0 else 1
    return values[::k]


def channel_options(channels) -> dict:
    """{key: label} of the plottable channels in a file: the known ones in CHANNELS' order, then any others."""
    out = {}
    for key, (title, unit, _scale, _cols) in CHANNELS.items():
        if key in channels:
            out[key] = f"{title} ({unit})" if unit else title
    for key in sorted(channels):
        if key not in out and key not in NOT_PLOTTED and np.asarray(channels[key]).dtype.kind in "fiub":
            out[key] = key
    return out


def _columns(key, n_cols, joint_num):
    """(column indexes, their names) of a channel."""
    cols = CHANNELS.get(key, (None, None, None, ()))[3]
    if cols is None:                                         # per joint: the robot's joints only
        n = min(int(joint_num), n_cols)
        return list(range(n)), [f"J{j + 1}" for j in range(n)]
    if cols == "motors":                                     # the joints, then the gripper
        return list(range(n_cols)), [f"J{j + 1}" if j < joint_num else "gripper" for j in range(n_cols)]
    return list(range(n_cols)), [cols[j] if j < len(cols) else f"c{j + 1}" for j in range(n_cols)]


def series(channels, ticks, joint_num, keys=DEFAULT_KEYS, max_points=MAX_POINTS):
    """The plot data: [{key, title, unit, x: [tick...], series: [{name, data: [...]}, ...]}] per key the file has;
    `ticks` is the file's tick channel. Downsampled together, so every plot shares its x; NaN -> None (a gap)."""
    out = []
    x = [int(t) for t in downsample(np.asarray(ticks), max_points)]
    for key in keys:
        if key not in channels or key in NOT_PLOTTED:
            continue
        arr = np.asarray(channels[key])
        if arr.dtype.kind not in "fiub":
            continue
        arr = arr.astype(float)
        if arr.ndim == 1:
            arr = arr[:, None]
        if arr.ndim != 2:
            continue
        title, unit, scale, _cols = CHANNELS.get(key, (key, "", 1.0, ()))
        idx, names = _columns(key, arr.shape[1], joint_num)
        arr = downsample(arr, max_points)
        scales = np.broadcast_to(np.asarray(scale, dtype=float), (arr.shape[1],)) if np.ndim(scale) == 0 \
            else np.asarray(scale, dtype=float)
        data = []
        for j, name in zip(idx, names):
            col = arr[:, j] * (scales[j] if j < len(scales) else 1.0)
            data.append({"name": name, "data": [None if not np.isfinite(v) else round(float(v), 4) for v in col]})
        out.append({"key": key, "title": title, "unit": unit, "x": x, "series": data})
    return out


def chart_options(plot, mark_tick=None, mark_label="fault"):
    """ECharts options for one plot: a line per column over the tick, the fault's (or a capture's request) tick as a
    vertical mark labelled `mark_label`."""
    opts = {
        "title": {"text": f"{plot['title']} ({plot['unit']})" if plot["unit"] else plot["title"], "left": "center",
                  "textStyle": {"fontSize": 13}},
        "tooltip": {"trigger": "axis"},
        "legend": {"bottom": 0, "type": "scroll"},
        "grid": {"left": 50, "right": 20, "top": 36, "bottom": 40},
        "xAxis": {"type": "category", "data": plot["x"], "name": "tick"},
        "yAxis": {"type": "value", "scale": True},
        "series": [{"name": s["name"], "type": "line", "showSymbol": False, "data": s["data"]} for s in plot["series"]],
    }
    if mark_tick is not None and opts["series"]:
        opts["series"][0]["markLine"] = {"symbol": "none", "lineStyle": {"color": "#c10015", "width": 2},
                                         "label": {"formatter": mark_label},
                                         "data": [{"xAxis": _nearest_index(plot["x"], mark_tick)}]}
    return opts


def _nearest_index(x, tick):
    if not x:
        return 0
    best = min(range(len(x)), key=lambda i: abs(x[i] - tick))
    return best


def mode_changes(channels, limit=12) -> list:
    """[(tick, mode)]: the mode at the file's first tick and every change after it (at most `limit`)."""
    modes, ticks = channels.get("mode"), channels.get("tick")
    if modes is None or ticks is None or not len(modes):
        return []
    out = [(int(ticks[0]), str(modes[0]))]
    for i in range(1, len(modes)):
        if modes[i] != modes[i - 1]:
            out.append((int(ticks[i]), str(modes[i])))
            if len(out) >= limit:
                break
    return out


def describe(meta):
    """One line about a loaded file: kind, keys / name, the ticks before and after its mark, time. Files written
    before 2026-10-15's after-window have no before / after counts: all their ticks are before."""
    kind = meta.get("kind", "incident")
    keys = ", ".join(meta.get("keys", []))
    ticks = int(meta.get("ticks", 0))
    before = int(meta.get("before_ticks", ticks))
    after = int(meta.get("after_ticks", 0))
    when = f"{meta.get('time_utc', '')} UTC"
    if kind == "capture":
        return (f"capture {keys}: {ticks} ticks — {before} before the request, {after} after "
                f"({float(meta.get('seconds', 0)):g} s asked), {when}")
    return f"incident at tick {meta.get('tick')}: {keys} — {before} ticks before it (the fault tick last), {after} after, {when}"


def mark_of(meta):
    """(tick, label) to mark on the plots: an incident's fault tick, a capture's request tick (None when unknown)."""
    if meta.get("kind", "incident") == "capture":
        return meta.get("request_tick"), "request"
    return meta.get("tick"), "fault"


def capture_request(seconds, name):
    """(ok, seconds, name | why): what the Capture button sends — the same bounds as the executor's check."""
    try:
        s = float(seconds)
    except (TypeError, ValueError):
        return False, None, f"the length is not a number: {seconds!r}"
    if not CAPTURE_MIN_S <= s <= CAPTURE_MAX_S:
        return False, None, f"the length must be {CAPTURE_MIN_S:g} to {CAPTURE_MAX_S:g} s"
    return True, s, safe_capture_name(name)


def list_files():
    return _list_files(INCIDENTS_DIR, CAPTURES_DIR)


def deletable(path, incidents_dir=None, captures_dir=None) -> bool:
    """True only for an .npz file directly inside the incidents or the captures folder (nothing else is deleted)."""
    try:
        p = Path(path).resolve()
    except (OSError, RuntimeError):
        return False
    folders = {Path(incidents_dir or INCIDENTS_DIR).resolve(), Path(captures_dir or CAPTURES_DIR).resolve()}
    return p.suffix == ".npz" and p.parent in folders and p.is_file()


def delete_file(path, incidents_dir=None, captures_dir=None) -> tuple[bool, str]:
    """(ok, message): delete one recording — refused for anything `deletable` does not accept."""
    if not deletable(path, incidents_dir, captures_dir):
        return False, f"not a recording: {path}"
    try:
        os.remove(path)
    except OSError as e:
        return False, f"cannot delete {os.path.basename(path)}: {e}"
    return True, f"deleted {os.path.basename(path)}"


def status_view(status, now=None) -> dict:
    """What the tab shows from the recorder's status JSON (recorder_status in shared memory):
    {alive, line, incidents, captures, capturing, fraction, progress} — `alive` False when there is none or it is
    older than 3 s."""
    now = time.time() if now is None else now
    if not isinstance(status, dict) or now - float(status.get("time", 0.0)) > STATUS_STALE_S:
        return {"alive": False, "line": "the recorder is not running — no incident files, no captures",
                "incidents": "", "captures": "", "capturing": False, "fraction": 0.0, "progress": ""}
    incidents = (f"incident files since the recorder started: {status.get('incidents_written', 0)} (not saved: "
                 f"{status.get('dropped', 0)} by the rate limit, {status.get('too_early', 0)} too early)")
    captures = f"captures since the recorder started: {status.get('captures_written', 0)}"
    out = {"alive": True, "line": "recorder running", "incidents": incidents, "captures": captures, "capturing": False,
           "fraction": 0.0, "progress": "no capture running"}
    c = status.get("capture")
    if c:
        gaps = f", {c['gaps']} gap(s)" if c.get("gaps") else ""
        out.update(capturing=True, fraction=float(c.get("fraction", 0.0)),
                   progress=f"capturing '{c['name']}': {float(c.get('done_s', 0)):.1f} of {float(c['seconds']):g} s{gaps}")
    return out


def settings_text(values) -> str:
    """The worst case a set of settings allows: the recorder's memory and the SD card."""
    hist = recorder_settings.history_s(values)
    return (f"With these: the recorder keeps {hist:g} s of ticks (~{hist * 250 * 1.8 / 1024:.0f} MB of memory); at "
            f"most ~{recorder_settings.disk_estimate_mb(values):.0f} MB of files on the SD card (every incident full, "
            f"every capture {CAPTURE_MAX_S:g} s).")


def folder_size_mb(files) -> float:
    return sum(f["size"] for f in files) / 1e6


_status_str = None
_last_status = None


def read_recorder_status():
    """The recorder's status (shared memory's recorder_status JSON, written by recorder.py), or None. A read that
    catches the string half-written (not JSON) returns the last good one — its own time says how old it is."""
    global _status_str, _last_status
    try:
        if _status_str is None:
            from data.shared_data import SharedString
            _status_str = SharedString("recorder_status", 2048)
        st = _status_str.get_json()
    except Exception:
        st = None
    if isinstance(st, dict):
        _last_status = st
    return _last_status


# ---------------------------------------------------------------------------------------------------------------
def add_incidents_tab(is_visible, fmt_time, capture_fn, joint_num, robot_state_fn=None, stop_fn=None, status_fn=None,
                      system_xml=SYSTEM_XML):
    """Build the Recordings tab (call inside its ui.tab_panel). `capture_fn(seconds, name)` sends capture_telemetry and
    `stop_fn()` stop_capture through the dashboard's command sender; `status_fn()` -> the recorder's status dict (or
    None); `robot_state_fn()` -> "ENABLED" / "DISABLED" (a capture is allowed either way). Files are loaded in a worker
    thread off the GUI. Returns handles for the nets."""
    from nicegui import run, ui

    if status_fn is None:
        status_fn = read_recorder_status
    state = {"files": [], "selected": None, "listed": 0.0, "loaded": None, "keys": list(DEFAULT_KEYS), "quiet": False}
    columns = [{"name": "time", "label": "Time", "field": "time", "align": "left"},
               {"name": "name", "label": "File", "field": "name", "align": "left"},
               {"name": "size", "label": "Size", "field": "size", "align": "right"}]
    current, problems = recorder_settings.read(system_xml)
    inputs = {}

    def settings_panel(title, keys):
        """An expansion with these settings' inputs and a Save that writes them (the others as in the file)."""
        with ui.expansion(title, icon="settings").classes("w-full mb-2"):
            with ui.row().classes("items-end gap-3 flex-wrap"):
                for key in keys:
                    lo, hi = recorder_settings.BOUNDS[key]
                    inputs[key] = ui.number(recorder_settings.LABELS[key], value=current[key], min=lo, max=hi,
                                            step=1 if key in recorder_settings.INTS else 0.5).classes("w-52")
                save = ui.button("Save", icon="save")
            applied = ui.label("").classes("text-xs text-gray-500")
            worst = ui.label(settings_text(current)).classes("text-xs text-gray-600")
            if problems:
                ui.label("system.xml <recorder>: " + "; ".join(problems)).classes("text-xs text-negative")
        save.on_click(lambda: save_settings(keys))
        return applied, worst

    recorder_label = ui.label("").classes("text-sm font-medium mb-1")
    with ui.row().classes("w-full gap-4 items-start no-wrap"):
        with ui.column().classes("w-full max-w-xl"):
            with ui.tabs().classes("w-full") as sub:
                cap_tab = ui.tab("Captures", icon="fiber_manual_record")
                inc_tab = ui.tab("Incidents", icon="bug_report")
            with ui.tab_panels(sub, value=cap_tab).classes("w-full"):
                with ui.tab_panel(cap_tab):
                    ui.label("Record the robot's channels whenever you want: every tick (4 ms) from the seconds before "
                             "the request (a setting) to the length asked.").classes("text-xs text-gray-500")
                    with ui.row().classes("items-end gap-3 flex-wrap"):
                        seconds_in = ui.number("Length (s)", value=10, min=CAPTURE_MIN_S, max=CAPTURE_MAX_S,
                                               step=1).classes("w-28")
                        name_in = ui.input("Name", placeholder="e.g. jerky_j3").classes("w-40")
                        capture_btn = ui.button("Capture now", icon="fiber_manual_record")
                        stop_btn = ui.button("Stop & save", icon="stop").props("color=negative outline")
                    with ui.row().classes("items-center gap-3 w-full my-1"):
                        progress = ui.linear_progress(value=0.0, show_value=False).classes("w-48")
                        progress_label = ui.label("").classes("text-sm")
                    captures_label = ui.label("").classes("text-xs text-gray-600")
                    cap_applied, cap_worst = settings_panel("Capture settings", recorder_settings.CAPTURE_KEYS)
                    cap_table = ui.table(columns=columns, rows=[], row_key="path", selection="single").classes("w-full")
                    cap_disk = ui.label("").classes("text-xs text-gray-500")
                with ui.tab_panel(inc_tab):
                    ui.label("Made automatically when a hard error latches (E-stop, a lost drive, a safety stop, a "
                             "critically slow loop, ...): the seconds before and after the fault.").classes(
                                 "text-xs text-gray-500")
                    incidents_label = ui.label("").classes("text-xs text-gray-600 my-1")
                    inc_applied, inc_worst = settings_panel("Incident settings", recorder_settings.INCIDENT_KEYS)
                    inc_table = ui.table(columns=columns, rows=[], row_key="path", selection="single").classes("w-full")
                    inc_disk = ui.label("").classes("text-xs text-gray-500")
            with ui.row().classes("gap-2"):
                refresh_btn = ui.button("Refresh lists", icon="refresh").props("flat")
        with ui.column().classes("flex-1 min-w-96"):
            info = ui.label("Select a capture or an incident to plot it.").classes("text-sm text-gray-600")
            modes_label = ui.label("").classes("text-xs text-gray-500")
            with ui.row().classes("gap-2 items-end w-full"):
                download_btn = ui.button("Download", icon="download").props("flat")
                delete_btn = ui.button("Delete", icon="delete").props("flat color=negative")
                picker = ui.select({k: CHANNELS[k][0] for k in DEFAULT_KEYS}, multiple=True, value=list(DEFAULT_KEYS),
                                   label=f"Channels (up to {MAX_PLOTS})").props("use-chips").classes("flex-1 min-w-72")
            charts = ui.column().classes("w-full")

    tables = {"capture": (cap_table, cap_disk), "incident": (inc_table, inc_disk)}

    def refresh_list(force=False):
        if not force and not is_visible():
            return
        files = list_files()
        if files != state["files"] or force:
            state["files"] = files
            for kind, (table, disk) in tables.items():
                mine = [f for f in files if f["kind"] == kind]
                table.rows = file_rows(mine, fmt_time)
                table.update()
                disk.text = f"{len(mine)} file(s), {folder_size_mb(mine):.1f} MB"
        state["listed"] = time.monotonic()

    def refresh_status():
        if not is_visible():
            return
        st = status_fn()
        view = status_view(st)
        recorder_label.text = view["line"]
        captures_label.text = view["captures"]
        incidents_label.text = view["incidents"]
        progress.value = view["fraction"]
        progress_label.text = view["progress"]
        progress.set_visibility(view["capturing"])
        stop_btn.set_enabled(view["capturing"])
        capture_btn.set_enabled(view["alive"] and not view["capturing"])
        if view["alive"] and st.get("settings"):
            for label, keys in ((cap_applied, recorder_settings.CAPTURE_KEYS),
                                (inc_applied, recorder_settings.INCIDENT_KEYS)):
                label.text = "In force: " + ", ".join(f"{recorder_settings.LABELS[k]} {st['settings'][k]:g}"
                                                      for k in keys if k in st["settings"])

    def capture():
        ok, sec, text = capture_request(seconds_in.value, name_in.value or "")
        if not ok:
            ui.notify(text, type="warning")
            return
        capture_fn(sec, text)
        ui.notify(f"capture {text!r} asked for: {sec:g} s", type="info")

    def stop_capture():
        if stop_fn is not None:
            stop_fn()
            ui.notify("stop asked: the recorder saves what it has", type="info")

    def save_settings(keys):
        values, _probs = recorder_settings.read(system_xml)
        values.update({k: inputs[k].value for k in keys})
        try:
            saved = recorder_settings.write(system_xml, values)
        except (ValueError, OSError) as e:
            ui.notify(f"not saved: {e}", type="warning", multi_line=True)
            return
        cap_worst.text = inc_worst.text = settings_text(saved)
        ui.notify("saved to config/system.xml — the recorder applies it within ~1 s", type="positive")

    def on_settings_change(_e=None):
        values, probs = recorder_settings.validate({k: inp.value for k, inp in inputs.items()})
        cap_worst.text = inc_worst.text = settings_text(values) + (f" — not valid: {'; '.join(probs)}" if probs else "")

    def draw():
        charts.clear()
        if state["loaded"] is None:
            return
        meta, ch = state["loaded"]
        keys = [k for k in state["keys"] if k in ch][:MAX_PLOTS]
        plots = series(ch, ch.get("tick", np.arange(int(meta.get("ticks", 0)))), joint_num, keys)
        mark, label = mark_of(meta)
        with charts:
            for p in plots:
                ui.echart(chart_options(p, mark, label)).classes("w-full").style("height:240px;")

    def set_picker(options=None, value=None):
        state["quiet"] = True                     # a change made here is not the operator's: no redraw from it
        try:
            if options is not None:
                picker.options = options
            if value is not None:
                picker.value = value
            picker.update()
        finally:
            state["quiet"] = False

    def on_pick(e):
        if state["quiet"]:
            return
        keys = list(e.value or [])
        if len(keys) > MAX_PLOTS:
            keys = keys[:MAX_PLOTS]
            set_picker(value=keys)
            ui.notify(f"at most {MAX_PLOTS} charts at once", type="warning")
        state["keys"] = keys
        draw()

    async def show_selected(e, mine, other):
        rows = e.selection if hasattr(e, "selection") else mine.selected
        if not rows:
            return
        other.selected = []                       # one file at a time, from either list
        other.update()
        path = rows[0]["path"]
        state["selected"] = path
        state["loaded"] = None
        info.text = "loading…"
        try:
            meta, ch = await run.io_bound(_load, path)
        except Exception as exc:
            info.text = f"cannot read {os.path.basename(path)}: {exc}"
            charts.clear()
            return
        state["loaded"] = (meta, ch)
        info.text = describe(meta)
        changes = mode_changes(ch)
        modes_label.text = ("modes: " + " → ".join(f"{m} (tick {t})" for t, m in changes)) if changes else ""
        options = channel_options(ch)
        set_picker(options, [k for k in state["keys"] if k in options])
        draw()

    def download():
        if state["selected"]:
            ui.download(state["selected"])

    async def delete():
        path = state["selected"]
        if not path:
            ui.notify("select a recording first", type="warning")
            return
        with ui.dialog() as dialog, ui.card():
            ui.label(f"Delete {os.path.basename(path)}? It cannot be undone.")
            with ui.row():
                ui.button("Delete", on_click=lambda: dialog.submit(True)).props("color=negative")
                ui.button("Cancel", on_click=lambda: dialog.submit(False)).props("flat")
        if not await dialog:
            return
        ok, msg = delete_file(path)
        ui.notify(msg, type="positive" if ok else "warning")
        if ok:
            state["selected"], state["loaded"] = None, None
            info.text, modes_label.text = "Select a capture or an incident to plot it.", ""
            charts.clear()
            refresh_list(force=True)

    cap_table.on("selection", lambda e: show_selected(e, cap_table, inc_table))
    inc_table.on("selection", lambda e: show_selected(e, inc_table, cap_table))
    capture_btn.on_click(capture)
    stop_btn.on_click(stop_capture)
    refresh_btn.on_click(lambda: refresh_list(force=True))
    download_btn.on_click(download)
    delete_btn.on_click(delete)
    picker.on_value_change(on_pick)
    for inp in inputs.values():
        inp.on_value_change(on_settings_change)
    progress.set_visibility(False)
    stop_btn.set_enabled(False)
    ui.timer(LIST_REFRESH_S, refresh_list)
    ui.timer(STATUS_REFRESH_S, refresh_status)
    refresh_list(force=True)
    return {"tables": tables, "refresh": refresh_list, "capture": capture, "state": state, "stop": stop_capture,
            "save_settings": save_settings, "inputs": inputs, "picker": picker, "status": refresh_status}
