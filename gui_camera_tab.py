"""The dashboard's Camera tab (2026-11-24, TODO/camera_plan.md P1): the camera server's live preview, its status and
its settings.

  * Preview: a plain <img> pointed at the camera server's /preview stream (at
    <camera preview_fps>, config/networking.CAMERA_PORT) on the address the browser reached the dashboard with. The video never passes through the
    dashboard's process; the stream runs only while this tab is the one shown (the user's D4: watch its cost).
  * Status: the server's JSON in shared memory (communication/camera_protocol.STATUS_NAME), once a second.
  * Settings: written into config/system.xml <camera> (utility/camera_settings.write); the server applies them within
    ~1 s. The camera's own controls are built from what the camera reports (types, ranges, menus); a control changed
    here is stored in <camera controls>, so it is applied again at every open.

NiceGUI 3.3 runs nicegui_test.py again at every page load while imported modules stay as first imported
(gui_page_lifecycle.py): per-process state lives here at module level (_STATUS), per-page state inside add_camera_tab.
"""
from __future__ import annotations

import logging
import time

from nicegui import ui

from communication import camera_protocol as cp
from config.networking import CAMERA_PORT
from project_paths import SYSTEM_XML
from utility import camera_settings

logger = logging.getLogger("gui")
STATUS_REFRESH_S = 1.0
STALE_S = 5.0                      # a status older than this: the camera server is not running
_STATUS: list = [None]             # the shared-memory string, attached once per process


def _status() -> dict | None:
    """The camera server's status, or None when it never published or stopped publishing."""
    try:
        if _STATUS[0] is None:
            from data.shared_data import SharedString
            _STATUS[0] = SharedString(cp.STATUS_NAME, cp.STATUS_SIZE)
        st = _STATUS[0].get_json()
    except Exception:  # noqa: BLE001 — the tab must draw without shared memory
        return None
    if not isinstance(st, dict) or time.time() - float(st.get("time", 0)) > STALE_S:
        return None
    return st


def _save(**changes) -> bool:
    """Merge `changes` into system.xml's <camera> settings; a notification on failure."""
    current, _problems = camera_settings.read(SYSTEM_XML)
    current.update(changes)
    try:
        camera_settings.write(SYSTEM_XML, current)
    except (ValueError, OSError) as e:
        ui.notify(f"Camera settings not saved: {e}", type="negative")
        return False
    return True


def _set_control(key: str, value) -> None:
    current, _ = camera_settings.read(SYSTEM_XML)
    pairs = camera_settings.parse_controls(current["controls"])
    for i, (k, _v) in enumerate(pairs):
        if k == key:
            pairs[i] = (key, int(value))
            break
    else:
        pairs.append((key, int(value)))
    _save(controls=camera_settings.format_controls(pairs))


def preview_options(current: float) -> dict:
    """The preview rates offered, the configured one among them even when it is none of them (review 2026-11-25,
    F15: a hand-edited 20 made the select raise, and the settings row stayed half built)."""
    options = {5.0: "5 fps", 10.0: "10 fps", 15.0: "15 fps", 30.0: "30 fps"}
    if float(current) not in options:
        options[float(current)] = f"{float(current):g} fps (system.xml)"
    return options


def menu_options(control: dict, value) -> dict:
    """A menu control's choices, its current value among them even when the camera's menu lacks it (F15)."""
    options = {int(v): str(n) for v, n in control["menu"]}
    if int(value) not in options:
        options[int(value)] = f"{int(value)} (not in the camera's menu)"
    return options


def controls_shape(st: dict) -> tuple:
    """What the controls grid is built from — not the values: a value the camera changes itself (under an automatic
    mode) is updated in place (F17: the grid was rebuilt every second, closing an open menu)."""
    return tuple((c["key"], c.get("type"), c.get("min"), c.get("max"), c.get("step"), c.get("default"),
                  tuple(tuple(m) for m in c.get("menu") or []), c.get("read_only"), c.get("inactive"))
                 for c in st.get("controls") or [])


def _clock_text(clk: dict) -> str:
    """Where the frames' times come from (the status' "clock", P2): the camera's own clock or the driver's."""
    if clk.get("source") == "camera":
        text = (f"the camera's own clock (fit ±{clk.get('fit_us')} µs, the camera's clock {clk.get('device_mhz')} "
                f"MHz); the driver's timestamps come {clk.get('driver_after_ms')} ms after it")
        cal = clk.get("calibration") or {}
        if clk.get("calibrated"):
            return text + f"; **calibrated** to the image's moment (run {cal.get('run') or '?'})"
        return text + (f"; not calibrated: {clk['calibration_why']}" if clk.get("calibration_why") else "")
    return f"the driver's timestamps (4 ms steps): {clk.get('why') or 'the camera clock is not known yet'}"


def _host_url(path: str) -> str:
    """A JavaScript expression: the camera server's URL on the host the browser used for the dashboard."""
    return f"location.protocol + '//' + location.hostname + ':{CAMERA_PORT}{path}'"


def add_camera_tab(is_visible) -> None:
    state = {"preview_on": False, "controls_key": None, "modes_key": None, "pid": None,
             "widgets": {}, "seen": {}}    # seen: each control's value as the camera reported it (F17)

    def from_ui(key: str, value) -> None:
        """A widget's change: stored, unless it is the camera's own value being shown (update_values sets it)."""
        if state["seen"].get(key) == int(value):
            return
        _set_control(key, int(value))

    with ui.row().classes("w-full items-start gap-4"):
        with ui.column().classes("gap-2").style("flex: 2 1 480px; min-width: 320px"):
            preview = ui.element("img").classes("w-full rounded").style(
                "background:#111; aspect-ratio:16/9; object-fit:contain")
            with ui.row().classes("gap-2 items-center"):
                ui.button("Snapshot", icon="photo_camera",
                          on_click=lambda: ui.run_javascript(f"window.open({_host_url('/snapshot.jpg')}, '_blank')")
                          ).props("outline size=sm")
                ui.button("Full-rate stream", icon="open_in_new",
                          on_click=lambda: ui.run_javascript(f"window.open({_host_url('/')}, '_blank')")
                          ).props("outline size=sm")
                link_hint = ui.label("").classes("text-xs text-gray-500")
        with ui.column().classes("gap-1").style("flex: 1 1 320px; min-width: 280px"):
            ui.label("Camera").classes("text-lg font-bold")
            headline = ui.label("").classes("font-bold")
            message = ui.label("").classes("text-sm text-gray-600")
            details = ui.markdown("").classes("text-sm")
            problems = ui.label("").classes("text-sm text-negative")
            notes = ui.label("").classes("text-sm text-gray-600")

    ui.separator()
    ui.label("Settings").classes("text-md font-bold")
    settings_row = ui.row().classes("w-full items-center gap-4")
    ui.label("The camera's own controls (stored in system.xml, applied at every open)").classes("text-md font-bold")
    controls_box = ui.column().classes("w-full gap-1")

    def draw_settings(st: dict) -> None:
        settings_row.clear()
        cfg = st.get("settings") or camera_settings.DEFAULTS
        modes = st.get("modes") or []
        with settings_row:
            ui.switch("Camera on", value=bool(cfg.get("enabled", 1)),
                      on_change=lambda e: _save(enabled=1 if e.value else 0))
            options = {f"{m['width']}x{m['height']}@{f:g}": f"{m['width']} x {m['height']} at {f:g} fps"
                       for m in modes for f in m["fps"]}
            current = f"{cfg['width']}x{cfg['height']}@{float(cfg['fps']):g}"
            if current not in options:
                options[current] = f"{cfg['width']} x {cfg['height']} at {float(cfg['fps']):g} fps (asked)"

            def on_mode(e):
                w, rest = e.value.split("x", 1)
                h, f = rest.split("@", 1)
                _save(width=int(w), height=int(h), fps=float(f))
            ui.select(options, value=current, label="Mode (MJPEG)", on_change=on_mode).classes("min-w-[220px]")
            fps_now = float(cfg.get("preview_fps", 15.0))
            ui.select(preview_options(fps_now), value=fps_now, label="Dashboard preview",
                      on_change=lambda e: _save(preview_fps=float(e.value))).classes("min-w-[140px]")
            devices = {"auto": "the gripper's camera (auto)"}
            for c in st.get("cameras") or []:
                devices[f"usb:{c['usb_id']}"] = f"USB {c['usb_id']}"
            if cfg.get("device") not in devices:
                devices[cfg.get("device")] = str(cfg.get("device"))
            ui.select(devices, value=cfg.get("device", "auto"), label="Which camera",
                      on_change=lambda e: _save(device=e.value)).classes("min-w-[220px]")

    def int_text(c: dict, label: str, value) -> str:
        return f"{label}: {value} (default {c['default']}, {c['min']}..{c['max']})"

    def draw_controls(st: dict) -> None:
        controls_box.clear()
        state["widgets"], state["seen"] = {}, {}
        with controls_box:
            controls = [c for c in st.get("controls") or [] if not c.get("read_only")]
            if not controls:
                ui.label("No camera open.").classes("text-sm text-gray-500")
                return
            with ui.grid(columns=2).classes("w-full gap-x-6 gap-y-1"):
                for c in controls:
                    # Inactive = the camera ignores it while its automatic mode is on (2026-11-24: the user moved the
                    # white balance temperature with automatic white balance on): shown, greyed, with the reason.
                    inactive = bool(c.get("inactive"))
                    label = c["name"] + ("  (switch its automatic mode off to change it)" if inactive else "")
                    value = c["value"] if c["value"] is not None else c["default"]
                    state["seen"][c["key"]] = int(value)
                    with ui.column().classes("gap-0"):
                        text = None
                        if c["type"] == "bool":
                            w = ui.switch(label, value=bool(value),
                                          on_change=lambda e, k=c["key"]: from_ui(k, 1 if e.value else 0))
                        elif c["type"] in ("menu", "intmenu") and c["menu"]:
                            w = ui.select(menu_options(c, value), value=int(value), label=label,
                                          on_change=lambda e, k=c["key"]: from_ui(k, e.value)).classes("w-full")
                        elif c["type"] == "int":
                            text = ui.label(int_text(c, label, value)).classes("text-xs text-gray-600")
                            w = ui.slider(min=c["min"], max=c["max"], step=max(1, int(c["step"] or 1)),
                                          value=int(value)).props("label").on(
                                "change", lambda e, k=c["key"]: _set_control(k, e.args))
                        else:
                            continue
                        state["widgets"][c["key"]] = (c["type"], w, text, label)
                        if inactive:
                            w.props("disable")
            ui.button("Controls back to the defaults", icon="restart_alt",
                      on_click=lambda: _save(controls=camera_settings.DEFAULTS["controls"])
                      ).props("flat size=sm")

    def update_values(st: dict) -> None:
        """The camera's own value changes, in place (F17); the handlers see them as the camera's (from_ui)."""
        for c in st.get("controls") or []:
            entry = state["widgets"].get(c["key"])
            if entry is None:
                continue
            kind, w, text, label = entry
            value = int(c["value"] if c["value"] is not None else c["default"])
            if state["seen"].get(c["key"]) == value:
                continue
            state["seen"][c["key"]] = value
            if kind == "bool":
                w.value = bool(value)
            elif kind in ("menu", "intmenu"):
                if value not in w.options:
                    w.set_options(menu_options(c, value), value=value)
                else:
                    w.value = value
            else:
                w.value = value
                if text is not None:
                    text.set_text(int_text(c, label, value))

    def set_preview(on: bool) -> None:
        if on == state["preview_on"]:
            return
        state["preview_on"] = on
        if on:
            ui.run_javascript(f"var e = document.getElementById('{preview.html_id}'); "
                              f"if (e) e.src = {_host_url('/preview')} + '?t=' + Date.now();")
        else:
            ui.run_javascript(f"var e = document.getElementById('{preview.html_id}'); "
                              f"if (e) e.removeAttribute('src');")

    def refresh() -> None:
        visible = is_visible()
        st = _status()
        if not visible:
            set_preview(False)
            return
        if st is None:
            set_preview(False)
            headline.set_text("The camera server is not running")
            message.set_text("It starts with the robot service (camera_server.py; logs/camera.log).")
            details.set_content("")
            problems.set_text("")
            return
        streaming = st.get("state") == "streaming"
        if st.get("pid") != state["pid"]:          # the camera server restarted: its stream is a new one (F16)
            if state["pid"] is not None:
                set_preview(False)
            state["pid"] = st.get("pid")
        set_preview(streaming)
        mode = st.get("mode") or {}
        dev = st.get("device") or {}
        headline.set_text({"streaming": "Streaming", "no camera": "No camera", "off": "Switched off",
                           "error": "Camera error", "starting": "Starting"}.get(st.get("state"), str(st.get("state"))))
        message.set_text(st.get("message") or "")
        gcam = st.get("gripper_camera") or {}
        clients = ", ".join(f"{c['kind']} {c['addr']} ({c['skipped']} skipped)" for c in st.get("clients") or [])
        lines = [
            f"- **Gripper:** {st.get('gripper') or '—'}" + (f" (declares {gcam.get('usb_id')})" if gcam else
                                                          " (declares no camera)"),
            f"- **Device:** {dev.get('path', '—')} {dev.get('usb_id', '')}",
        ]
        if mode:
            lines += [f"- **Mode:** {mode['width']} x {mode['height']} at {float(mode['fps']):g} fps; delivered "
                      f"{st.get('fps', 0)} fps, {st.get('mbit_s', 0)} Mbit/s",
                      f"- **Frames:** {st.get('frames', 0)}, lost by the driver {st.get('lost', 0)}, broken and "
                      f"dropped {st.get('bad_frames', 0)}, camera reopened {st.get('reopens', 0)} times",
                      "- **RTI tick on each frame:** " + ("yes" if st.get("tick_mapping") else
                                                          "no (no recent rows of RTI's trace: RTI not running?)"),
                      f"- **Frame times:** {_clock_text(st.get('clock') or {})}"]
        lines.append(f"- **Clients:** {clients or 'none'}")
        details.set_content("\n".join(lines))
        problems.set_text("; ".join(st.get("problems") or []))
        notes.set_text("; ".join(st.get("notes") or []))
        link_hint.set_text(f"programs: http://<robot>:{CAMERA_PORT}/frames (camera protocol {st.get('protocol', '?')})")
        modes_key = (repr(st.get("settings")), repr([(m["width"], m["height"]) for m in st.get("modes") or []]),
                     repr(st.get("cameras")))
        if modes_key != state["modes_key"]:
            state["modes_key"] = modes_key
            draw_settings(st)
        controls_key = (dev.get("path"), repr(st.get("settings", {}).get("controls")), controls_shape(st))
        if controls_key != state["controls_key"]:
            state["controls_key"] = controls_key
            draw_controls(st)
        else:
            update_values(st)

    ui.timer(STATUS_REFRESH_S, refresh)
