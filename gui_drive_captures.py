"""The Recordings tab's "Drive captures" card (2026-11-17): a drive's own recording of its fault
(utility/drive_capture.py; saved by recorder.py in temp_data/drive_captures/): the current, the speed and the position
error inside the drive's control loop at 1562 Hz, 0.5 s before the fault and 0.14 s after it — 25 times finer than the
250 Hz incident recordings next to it. A list (newest first) and, for the one picked, a chart per signal over the time
from the fault (ms).
"""
from __future__ import annotations

import json
from pathlib import Path

TITLES = {"iq": "Motor current", "velocity_16": "Motor speed", "position_error": "Position error"}


def list_captures(folder) -> list:
    """The saved captures, newest first."""
    folder = Path(folder)
    return sorted(folder.glob("*_drive_capture.npz"), reverse=True) if folder.is_dir() else []


def load_capture(path):
    """(info dict, {signal: values in its units}) of a saved capture."""
    import numpy as np
    with np.load(path, allow_pickle=False) as z:
        info = json.loads(str(z["meta"]))
        return info, {s: [float(v) for v in z[s]] for s in info["signals"]}


def trigger_known(info) -> bool:
    """The drive answered where its trigger sample is (a lost reply is stored as -1)."""
    trig = info.get("trigger_index", -1)
    return trig is not None and trig >= 0


def times_ms(info) -> list:
    """Each sample's time from the fault (the trigger sample = 0), ms; from the first sample when the trigger is
    unknown."""
    dt = info.get("sample_s") or 0.0
    trig = info["trigger_index"] if trigger_known(info) else 0
    return [round((i - trig) * dt * 1000.0, 2) for i in range(info["count"])]


def label(path, info) -> str:
    name = Path(path).name.replace("_drive_capture.npz", "")
    lost = f", {info['lost']} lost" if info.get("lost") else ""
    unknown = "" if trigger_known(info) else ", the fault's moment unknown"
    return f"{name} — {info['count']} samples{lost}{unknown}"


def chart(signal, unit, x, values, fault_known=True) -> dict:
    """ECharts options: the signal over the time from the fault, the fault marked. With the trigger unknown (review
    C10, 2026-11-21: it was drawn as "fault" at the first sample) the time runs from the first sample, nothing marked."""
    series = {"type": "line", "showSymbol": False, "data": [[t, v] for t, v in zip(x, values)]}
    if fault_known:
        series["markLine"] = {"symbol": "none", "lineStyle": {"color": "#c10015", "width": 2},
                              "label": {"formatter": "fault"}, "data": [{"xAxis": 0}]}
    axis = "ms from the fault" if fault_known else "ms from the first sample (the fault's moment unknown)"
    return {"title": {"text": f"{TITLES.get(signal, signal)} ({unit})", "left": "center", "textStyle": {"fontSize": 13}},
            "tooltip": {"trigger": "axis"}, "grid": {"left": 60, "right": 20, "top": 36, "bottom": 30},
            "xAxis": {"type": "value", "name": axis, "nameLocation": "middle", "nameGap": 22},
            "yAxis": {"type": "value", "scale": True}, "series": [series]}


def add_drive_captures_card(folder=None):
    """The card: the list and the charts of the capture picked."""
    from nicegui import ui
    if folder is None:
        from project_paths import TEMP_DATA_DIR
        folder = TEMP_DATA_DIR / "drive_captures"
    state = {"files": None}
    with ui.card().classes("w-full mt-3"):
        ui.label("Drive captures").classes("text-sm font-semibold text-gray-600 uppercase tracking-wide")
        ui.label("Each drive records its own next fault (current, speed, position error at 1562 Hz, 0.5 s before it); "
                 "the robot reads it back after the fault. Armed at start and after Clear Errors.")\
            .classes("text-xs text-gray-500 mb-2")
        pick = ui.select({}, label="Capture").classes("w-full")
        charts = ui.column().classes("w-full")

    def show(_=None):
        charts.clear()
        if not pick.value:
            return
        try:
            info, data = load_capture(pick.value)
        except Exception as e:      # noqa: BLE001 — a damaged file is shown as such
            with charts:
                ui.label(f"cannot read it: {e}").classes("text-negative")
            return
        x = times_ms(info)
        with charts:
            for signal, unit in zip(info["signals"], info["units"]):
                ui.echart(chart(signal, unit, x, data[signal], trigger_known(info))).classes("w-full").style("height:220px;")

    def refresh():
        files = list_captures(folder)
        if files == state["files"]:
            return
        state["files"] = files
        opts = {}
        for f in files:
            try:
                info, _ = load_capture(f)
                opts[str(f)] = label(f, info)
            except Exception:       # noqa: BLE001
                opts[str(f)] = f.name
        pick.options = opts
        if pick.value not in opts:
            pick.value = next(iter(opts), None)
        pick.update()
        show()

    pick.on_value_change(show)
    ui.timer(5.0, refresh)
    return {"refresh": refresh, "pick": pick}
