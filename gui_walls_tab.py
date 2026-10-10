"""Walls tab — a TEMPORARY dashboard tuner for the freedrive joint walls (docs/safety_zones.md §2b).

Requested 2026-10-01: tune each joint's wall by feel in IDLE gravity-comp freedrive, then save it into the robot
XML. A wall is set by its natural FREQUENCY (Hz) and DAMPING RATIO (2026-10-02): the same at every pose and with the
same stable limits for every joint; its stiffness and damping in Nm/deg follow the joint's inertia at each pose
(shown live). Each joint also has its own speed FILTER for the damping (0-8 ms, 1 ms steps); the stable frequency
and ratio range follow it. Everything goes through command_executor (get_joint_walls / set_joint_wall / save_joint_walls), which
REFUSES a setting outside the stability table (WALL_UNSTABLE, shown as a toast). The sliders are limited to the same
range only for convenience — the executor's check is the one that counts. Live readouts come from shared memory.

Remove: delete this file and the three lines in nicegui_test.py that mention add_walls_tab / tab_walls.
"""
import time

import numpy as np
from nicegui import ui

from config.xml_parser import WALL_FILTER_MAX_MS
from utility.haptic_walls import damping_ratio_window, wall_dynamics_limits

DEG = np.pi / 180.0


def _f(v, n=4):
    return "auto" if v is None else f"{v:.{n}g}"


# What each setting does — shown on the small "?" next to it (user request 2026-10-02).
HELP = {
    "f": "Frequency (Hz): how quickly the wall springs back — the natural frequency of the joint bouncing on the "
         "wall. Higher feels harder. It is set in Hz rather than Nm/deg because the same Hz feels the same on every "
         "joint and in every pose: the robot works out the Nm/deg from the joint's inertia (shown in the table and "
         "the live line). The maximum depends on the speed filter (the stability table).",
    "z": "Damping ratio: how much the wall absorbs. About 0.7 comes back without bouncing; below ~0.3 it rings "
         "(bounces); above 1 it feels dead and slow. The allowed range depends on the frequency and the filter.",
    "flt": "Speed filter (ms): how much the joint's speed is smoothed before the damping uses it. 0 = the raw speed: "
           "allows the stiffest wall, but the damping follows the motors' ripple and can buzz. More = smoother, but a "
           "lower maximum frequency. Only whole ms have been checked for stability (0-8).",
    "tmax": "Max torque (Nm): the most this wall ever pushes, however deep you go — past it the push stays flat. "
            "The slider stops at what this joint's drive can deliver.",
    "bt": "Breakthrough (deg past the wall): push further than this and you have pushed THROUGH the wall — the "
          "'pushed through' reaction happens. A joint with an endstop cannot be set past it.",
    "at": "Where the wall sits: at the joint's soft limit (you feel it before the endstop), or at its hard limit — "
          "only for a joint WITHOUT an endstop (J6, whose hard limit is the cable wind-up).",
    "fd": "Pushed through: warn = a live JOINT_WALL_WARNING, nothing stops; stop = the robot stops with a "
          "JOINT_WALL error (Clear Errors, then Enable, to continue).",
    "sc": "Adaptive: stiffness and damping follow the inertia the joint carries in the current pose — it springs back "
          "the same everywhere and is harder where the arm is stretched out. Fixed: the lightest pose's Nm/deg "
          "everywhere — the same push per degree, so it feels softer and slower where the joint carries more. Both "
          "stay inside the stability table.",
    "live": "The selected joint now: its angle, how far it is from (or into) its wall, the frequency, stiffness and "
            "damping in force at this pose, and the torque RTI sends on top of gravity (the wall's push).",
}


def _help(key):
    """A small "?" with the explanation as its tooltip."""
    return ui.icon("help_outline", size="xs").classes("text-gray-400 cursor-help").tooltip(HELP[key])


def add_walls_tab(exec_fn, shared_data, joints, is_visible, poll_fn=None):
    """Build the tab's content inside the current tab panel. exec_fn(fn, label) submits fn(robot) to the
    dashboard's command sender; is_visible() says whether the tab is on screen (the live readout timer).
    poll_fn(fn, label): the same, but for the 0.5 s pose-scale poll — it must NOT count as dashboard activity
    (the dashboard releases control after 5 s idle) and must not toast failures. Returns its handles (state,
    form, actions) for the offline test."""
    st = {"data": None, "version": 0, "shown": -1, "asked": 0.0,   # last executor reply (the sender thread writes)
          "scale_here": None, "polled": 0.0}                      # ...and the latest pose scales (the poll writes)
    form = {"joint": joints, "at": "soft", "f": 7.0, "z": 0.7, "flt": 4.0, "tmax": 0.0, "bt": 5.0, "freedrive": "warn",
            "scaling": "adaptive"}

    def _store(call):
        def run(r):
            st["data"] = call(r)
            st["version"] += 1
        return run

    def load():
        exec_fn(_store(lambda r: r.get_joint_walls()), "get_joint_walls")

    def _store_scales(r):                    # the poll: ONLY the pose scales — never re-fills the form
        st["scale_here"] = [w.get("scale_here", 1.0) for w in r.get_joint_walls()["walls"]]

    def wall(j=None):
        data = st["data"]
        j = form["joint"] if j is None else j
        return None if data is None else data["walls"][j - 1]

    ui.label("Freedrive joint walls — TEMPORARY tuner").classes("text-sm font-semibold text-gray-600 uppercase")
    ui.label("A wall acts only in IDLE with gravity compensation ON (freedrive). Push a joint into it by hand to feel "
             "it; Apply changes it live, Save writes the robot XML. Frequency = how stiff it is relative to the arm's "
             "inertia (higher = harder); damping ratio = how much it absorbs (about 0.7 = no bounce); filter = how much the "
             "speed behind the damping is smoothed (less = a stiffer wall is allowed, but the damping follows the "
             "motors' ripple; more = smoother, but a lower maximum). All are limited to what the stability table "
             "allows (an unstable wall buzzes and moves the arm). "
             "Every change re-arms the walls: a joint already inside one feels nothing until it has left it once."
             ).classes("text-xs text-gray-500 mb-2")

    table = ui.table(columns=[
        {"name": "j", "label": "J", "field": "j"},
        {"name": "at", "label": "at", "field": "at"},
        {"name": "wall", "label": "wall [deg]", "field": "wall"},
        {"name": "f", "label": "frequency Hz", "field": "f"},
        {"name": "z", "label": "damping ratio", "field": "z"},
        {"name": "flt", "label": "filter ms", "field": "flt"},
        {"name": "sc", "label": "scaling", "field": "sc"},
        {"name": "k", "label": "stiffness Nm/deg (lightest .. heaviest pose)", "field": "k"},
        {"name": "tmax", "label": "max Nm", "field": "tmax"},
        {"name": "bt", "label": "breakthrough deg", "field": "bt"},
        {"name": "fd", "label": "pushed through", "field": "fd"},
        {"name": "src", "label": "setting", "field": "src"},
        {"name": "note", "label": "note", "field": "note"},
    ], rows=[]).classes("w-full font-mono text-xs")

    with ui.card().classes("w-full mt-3"):
        with ui.row().classes("items-center gap-4"):
            sel = ui.select({j: f"J{j}" for j in range(1, joints + 1)}, value=joints, label="Joint").classes("w-24")
            at_toggle = ui.toggle({"soft": "at soft limit", "hard": "at hard limit"}, value="soft").bind_value(form, "at")
            _help("at")
            fd_toggle = ui.toggle({"warn": "pushed through: warn", "stop": "pushed through: stop"},
                                  value="warn").bind_value(form, "freedrive")
            _help("fd")
            sc_toggle = ui.toggle({"adaptive": "adaptive", "fixed": "fixed"}, value="adaptive").bind_value(form, "scaling")
            _help("sc")
        ranges = ui.label("").classes("text-xs text-gray-500 font-mono")
        with ui.grid(columns=5).classes("gap-x-6 gap-y-2 w-full mt-2"):
            sliders = {}
            for key, label in (("f", "Frequency (Hz)"), ("z", "Damping ratio"), ("flt", "Speed filter (ms)"),
                               ("tmax", "Max torque (Nm)"), ("bt", "Breakthrough (deg past the wall)")):
                with ui.column().classes("gap-1"):
                    with ui.row().classes("items-center gap-1 no-wrap"):
                        ui.label(label).classes("text-xs text-gray-500 font-mono")
                        _help(key)
                    ui.number(format="%.5g").classes("w-full font-mono text-sm").bind_value(form, key)
                    sliders[key] = ui.slider(min=0.0, max=1.0, step=0.001).classes("w-full").bind_value(form, key)
        warn_lbl = ui.label("").classes("text-xs text-orange-700")
        with ui.row().classes("gap-3 mt-2"):
            ui.button("Load", icon="download", on_click=load).props("color=primary outline size=sm") \
                .tooltip("Read the walls in force from the robot again (your unapplied edits are lost)")
            ui.button("Apply (live)", icon="send", on_click=lambda: apply()).props("color=positive size=sm") \
                .tooltip("Change this joint's wall on the robot now — not saved: a restart goes back to the robot XML")
            ui.button("Reset joint to auto", icon="settings_backup_restore",
                      on_click=lambda: reset()).props("color=secondary outline size=sm") \
                .tooltip("This joint back to the defaults (live, not saved)")
            ui.button("Save all to robot XML", icon="save", on_click=lambda: save()).props("color=warning size=sm") \
                .tooltip("Write every joint's wall as it is now into the robot XML (kept after a restart)")

    with ui.card().classes("w-full mt-3"):
        with ui.row().classes("items-center gap-1"):
            ui.label("Live (selected joint)").classes("text-xs font-semibold text-gray-600 uppercase")
            _help("live")
        live = ui.label("---").classes("font-mono text-sm")

    def _set_range(key, lo, hi):
        s = sliders[key]
        span = max(hi - lo, 1e-9)
        s.props["min"], s.props["max"], s.props["step"] = lo, hi, span / 500.0

    def _f_max():                            # the stable maximum frequency with the filter in the form
        return wall_dynamics_limits(st["data"]["policy"], form["flt"] / 1000.0)[2]

    def fill_form():
        w = wall()
        if w is None:
            return
        form.update(at=w["at"], freedrive=w["freedrive"], f=w["frequency_hz"], z=w["damping_ratio"],
                    flt=w["filter_ms"], tmax=w["max_torque_nm"], bt=w["breakthrough_deg"],
                    scaling=w.get("scaling", "adaptive"))
        sliders["flt"].props["min"], sliders["flt"].props["max"], sliders["flt"].props["step"] = \
            0.0, WALL_FILTER_MAX_MS, 1.0           # the stability table's columns
        _set_range("f", 0.5, w["frequency_max_hz"] or 1.0)
        # Up to what this joint's drive can give (Ilim x Kt x gear x efficiency) — more is clipped by the drive.
        _set_range("tmax", 0.01, min(float(st["data"]["policy"]["joint_max_torque"]),
                                     float(w.get("drive_torque_nm") or st["data"]["policy"]["joint_max_torque"])))
        _set_range("bt", 2.0, min(90.0, w["breakthrough_max_deg"] or 90.0))
        at_toggle.set_enabled(w["breakthrough_max_deg"] is None)     # a joint with an endstop: soft only
        on_frequency()

    def on_filter():
        if wall() is None:
            return
        f_max = _f_max()
        _set_range("f", 0.5, f_max or 1.0)
        if form["f"] > f_max:
            form["f"] = f_max
            warn_lbl.text = "frequency lowered to the stable maximum for this filter"
        on_frequency()

    def on_frequency():
        w = wall()
        if w is None:
            return
        win = damping_ratio_window(form["f"], form["flt"] / 1000.0)
        if win is None:
            ranges.text = "no stable damping at this frequency"
            return
        _set_range("z", win[0], win[1])
        if not (win[0] <= form["z"] <= win[1]):
            form["z"] = min(max(form["z"], win[0]), win[1])
            warn_lbl.text = "damping ratio moved into the stable range for this frequency"
        om = 2 * np.pi * form["f"]
        k_lo = om * om * w["inertia_kg_m2"] * DEG
        ranges.text = (f"With a {form['flt']:g} ms filter (any joint): up to {_f_max():.1f} Hz; at {form['f']:.2f} Hz the "
                       f"damping ratio may be "
                       f"{win[0]:.2f}..{win[1]:.2f}. J{form['joint']}: that is {k_lo:.4g} Nm/deg at its lightest pose, up "
                       f"to {k_lo * w.get('stiffness_scale_max', 1.0):.4g} at its heaviest; max torque up to "
                       f"{w.get('drive_torque_nm', 0):.2f} Nm (the drive); wall at [{w['lo_deg']:.1f}, {w['hi_deg']:.1f}] deg")

    sliders["f"].on_value_change(lambda e: on_frequency())
    sliders["flt"].on_value_change(lambda e: on_filter())
    sel.on_value_change(lambda e: (form.update(joint=int(e.value)), fill_form()))

    def apply():
        j, f = form["joint"], dict(form)
        warn_lbl.text = ""
        exec_fn(_store(lambda r: r.set_joint_wall(
            j, at=f["at"], frequency_hz=float(f["f"]), damping_ratio=float(f["z"]), filter_ms=float(f["flt"]),
            max_torque_nm=float(f["tmax"]), breakthrough_deg=float(f["bt"]), freedrive=f["freedrive"],
            scaling=f["scaling"])), "set_joint_wall")

    def reset():
        j = form["joint"]
        exec_fn(_store(lambda r: r.set_joint_wall(
            j, at="auto", frequency_hz="auto", damping_ratio="auto", filter_ms="auto", max_torque_nm="auto",
            breakthrough_deg="auto", freedrive="warn", scaling="auto")), "set_joint_wall")

    def save():
        exec_fn(_store(lambda r: r.save_joint_walls()), "save_joint_walls")

    def refresh():
        if not is_visible():
            return
        if st["data"] is None and time.monotonic() - st["asked"] > 5.0:     # first view, or the executor was down
            st["asked"] = time.monotonic()
            load()
        if poll_fn is not None and st["data"] is not None and time.monotonic() - st["polled"] > 0.5:
            st["polled"] = time.monotonic()
            poll_fn(_store_scales, "walls_poll")
        if st["version"] != st["shown"] and st["data"] is not None:
            st["shown"] = st["version"]
            rows = []
            for w in st["data"]["walls"]:
                spec = w["spec"]
                custom = [k for k in ("at", "frequency_hz", "damping_ratio", "filter_ms", "max_torque_nm",
                                      "breakthrough_deg", "scaling") if spec.get(k) is not None]
                rows.append({"j": f"J{w['joint']}", "at": w["at"] if w["enabled"] else "off",
                             "wall": f"{w['lo_deg']:.1f} .. {w['hi_deg']:.1f}",
                             "f": _f(w["frequency_hz"], 3), "z": _f(w["damping_ratio"], 3),
                             "flt": _f(w["filter_ms"], 2), "sc": w.get("scaling", "adaptive"),
                             "k": f"{w['stiffness_nm_per_deg']:.3g} .. "
                                  f"{w['stiffness_nm_per_deg'] * w.get('stiffness_scale_max', 1.0):.3g}",
                             "tmax": _f(w["max_torque_nm"], 3), "bt": _f(w["breakthrough_deg"], 3),
                             "fd": w["freedrive"],
                             "src": ("custom" if custom else "auto") + (" (UNSAVED)" if w["unsaved"] else ""),
                             "note": w["note"]})
            table.rows = rows
            table.update()
            fill_form()
        j = form["joint"] - 1
        w = wall()
        q = float(shared_data.joint["joint_position"][j]) / DEG
        tau = float(shared_data.joint["joint_torque_commanded"][j] - shared_data.joint["joint_gravity_torque"][j])
        mode = shared_data.robot_mode.get() or "---"
        grav = bool(shared_data.rti["gravity_comp_enabled"][0])
        where = ""
        if w is not None and w["enabled"]:
            past = max(q - w["hi_deg"], w["lo_deg"] - q)
            where = (f"{past:.1f} deg INTO the wall (pushed through at {w['breakthrough_deg']:.1f})" if past > 0
                     else f"{-past:.1f} deg from the wall")
        freedrive = mode == "IDLE" and grav
        scale = (st["scale_here"] or [1.0] * joints)[j]           # the inertia here / at the lightest pose
        if w is None or not w["enabled"]:
            stiff = ""
        elif w.get("scaling", "adaptive") == "fixed":            # the lightest pose's Nm/deg on a heavier joint
            stiff = (f"   FIXED stiffness {w['stiffness_nm_per_deg']:.4g} Nm/deg, damping {w['damping_nm_s_per_deg']:.3g} "
                     f"Nm.s/deg — with inertia x{scale:.2f} its lightest that is {w['frequency_hz'] / scale ** 0.5:.2f} Hz here")
        else:
            stiff = (f"   {w['frequency_hz']:.2f} Hz here = stiffness {w['stiffness_nm_per_deg'] * scale:.4g} Nm/deg, "
                     f"damping {w['damping_nm_s_per_deg'] * scale:.3g} Nm.s/deg (inertia x{scale:.2f} its lightest)")
        live.text = (f"J{j + 1} {q:8.2f} deg   {where}{stiff}   wall torque {tau:+.3f} Nm   "
                     f"[{mode}, gravity comp {'ON' if grav else 'OFF'}"
                     f"{'' if freedrive else ' — walls act only in IDLE with gravity comp ON'}]")

    ui.timer(0.2, refresh)
    return {"state": st, "form": form, "refresh": refresh, "apply": apply, "reset": reset, "save": save,
            "load": load, "sliders": sliders, "table": table, "live": live, "at_toggle": at_toggle,
            "sc_toggle": sc_toggle}
