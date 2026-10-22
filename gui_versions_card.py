"""The "Versions" card in the dashboard's System Info tab (2026-10-21; versioning step V1).

Every part's version in one place: the runtime (the VERSION file, its branch and commit), Python, the operating system,
the description package, the control box (the PCB revision configured in system.xml next to what the board reports),
and per drive the firmware, hardware revision and serial the drive reports. The same data as `robot.get_versions()`
and the support bundle's info.txt (utility/versions.py). Refreshed every 2 s while the tab is open: a drive reports its
versions only after the start-up scan, so a page opened right after a restart fills in by itself.
"""
from __future__ import annotations

import os


def _v(x) -> str:
    return "—" if x is None else str(x)


def config_text(rep: dict) -> str:
    """One line about the configuration files' formats (versioning V2): all current, or which are older."""
    cf = rep.get("config_formats") or {}
    reads = cf.get("reads") or {}
    files = cf.get("files") or {}
    if not reads:
        return "—"
    older = sorted(os.path.basename(n) for n, f in files.items() if f.get("schema", 0) < reads.get(f.get("kind"), 0))
    if older:
        return f"{len(older)} in an older format: " + ", ".join(older[:3]) + (" …" if len(older) > 3 else "")
    return f"{len(files)} read, all in the current format" if files else "none read by this process"


def firmware_text(d: dict) -> str:
    """A drive's firmware, + the supported range when it is outside it (V4, hardware/firmware_support.py)."""
    if d.get("supported") in ("too_old", "too_new"):
        return f"{_v(d.get('firmware'))} (supported: {d.get('supported_range')})"
    return _v(d.get("firmware"))


def hardware_text(rep: dict) -> str:
    """The arm's and the gripper's declared revisions (V4): "PAR6 v1.0 · MSG_small_motor_150mm_rail v1.0"."""
    hw = rep.get("hardware") or {}
    if not hw.get("robot"):
        return "—"
    text = f"{hw['robot']} {_v(hw.get('revision'))}"
    if hw.get("gripper"):
        text += f" · {hw['gripper']} {_v(hw.get('gripper_revision'))}"
    return text


def package_text(rep: dict) -> str:
    """The description package's version, + the robot XML's minimum when the installed one is older (V4)."""
    pkg = rep.get("description_package") or {}
    text = _v(pkg.get("version"))
    if pkg.get("ok") is False:
        text += f" (older than the minimum {pkg.get('min')})"
    return text


def drive_rows(rep: dict) -> list[dict]:
    """Rows of the drives table: one per joint, then the gripper (when there is one)."""
    rows = [{"part": f"J{d['joint']}", "can_id": d["can_id"], "firmware": firmware_text(d),
             "hardware": _v(d["hardware"]), "serial": _v(d["serial"])} for d in rep.get("drives", [])]
    g = rep.get("gripper")
    if g:
        rows.append({"part": "Gripper", "can_id": g["can_id"], "firmware": firmware_text(g),
                     "hardware": _v(g["hardware"]), "serial": _v(g["serial"])})
    return rows


def software_lines(rep: dict) -> list[tuple[str, str]]:
    """(label, value) pairs of the card's upper grid."""
    rt = rep.get("runtime", {})
    sha = rt.get("commit")
    osi = rep.get("os", {})
    cb = rep.get("control_box", {})
    rt_flag = {True: " (real-time)", False: " (NOT real-time)"}.get(osi.get("realtime"), "")
    return [
        ("Runtime", _v(rt.get("version"))),
        ("Code", f"{rt.get('branch') or 'detached'} @ {sha[:12]}" if sha else "—"),
        ("Python", _v(rep.get("python"))),
        ("OS", _v(osi.get("name"))),
        ("Kernel", _v(osi.get("kernel")) + rt_flag),
        ("Hardware", hardware_text(rep)),
        ("Description package", package_text(rep)),
        ("Control box PCB", f"v{cb['pcb_configured']} (configured)" if cb.get("pcb_configured") is not None else "—"),
        ("Board reports", f"hw {_v(cb.get('hardware'))} · sw {_v(cb.get('software'))} · serial {_v(cb.get('serial'))}"),
        ("Config files", config_text(rep)),
        ("Protocols", " · ".join(f"{k} {v}" for k, v in (rep.get("protocols") or {}).items()) or "—"),
    ]


def add_versions_card(report_fn, is_visible=lambda: True):
    """The card. report_fn() -> utility.versions.report(...). Returns its handles (for the net)."""
    from nicegui import ui
    columns = [{"name": k, "label": lbl, "field": k, "align": "left"}
               for k, lbl in (("part", "Part"), ("can_id", "CAN id"), ("firmware", "Firmware"),
                              ("hardware", "Hardware"), ("serial", "Serial"))]
    labels = {}
    with ui.card().classes("w-full"):
        ui.label("Versions").classes("text-sm font-semibold mb-2 text-gray-600 uppercase tracking-wide")
        with ui.grid(columns=2).classes("gap-x-6 gap-y-1 font-mono text-sm"):
            for name, _ in software_lines({}):
                ui.label(name)
                labels[name] = ui.label("—")
        table = ui.table(columns=columns, rows=[], row_key="part").props("dense flat").classes("w-full mt-2")
        ui.label("Drive versions appear once the drives have answered the start-up scan.") \
            .classes("text-xs text-gray-400")

    def refresh():
        try:
            if not is_visible():
                return
            rep = report_fn()
        except Exception as exc:                    # a page must keep working; say why instead
            labels["Runtime"].set_text(f"unavailable: {exc}")
            return
        for name, value in software_lines(rep):
            labels[name].set_text(value)
        table.rows = drive_rows(rep)
        table.update()

    # The first fill comes from the timer, not from here: the dashboard defines the names its callbacks use (the tab
    # helper, the gripper check) further down its script, after this card is built.
    ui.timer(2.0, refresh)
    return {"labels": labels, "table": table, "refresh": refresh}
