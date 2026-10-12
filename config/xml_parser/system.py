"""config/system.xml: the active robot / gripper / cell and the robot-independent policies — jog and RTI
defaults, maintenance (flashing park gate), friction-compensation policy, safety-monitor switches. Every
policy block falls back to its hardcoded default, with a warning, when it is missing or out of range.

Split out of the single-file config/xml_parser.py on 2026-10-03 (refactor Phase 4); definitions moved verbatim.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from config.xml_parser.xml_access import get_active_gripper, get_joint_count, parse_robot_config


def load_system_config(system_xml_path: str | Path, robots_dir: str | Path) -> dict:
    """
    Read config/system.xml and return the active robot's XML path + joint count.

    This is intentionally lightweight — it only parses two small XML files so
    it can be called before shared_data.setup() at the very top of RTI.py.

    Parameters
    ----------
    system_xml_path : str or Path  — path to config/system.xml
    robots_dir      : str or Path  — path to the robots/ directory

    Returns
    -------
    dict with keys:
        active_robot    : str  — robot name, e.g. "PAR6"
        robot_xml_path  : Path — absolute path to robots/<active_robot>.xml
        joint_num       : int  — number of joints read from that XML
    """
    tree = ET.parse(str(system_xml_path))
    root = tree.getroot()
    active_robot = root.findtext("active_robot", default="").strip()
    if not active_robot:
        raise ValueError(
            f"[system.xml] <active_robot> is empty or missing in {system_xml_path}"
        )

    robot_xml_path = Path(robots_dir) / f"{active_robot}.xml"
    if not robot_xml_path.exists():
        available = [p.stem for p in Path(robots_dir).glob("*.xml")]
        raise FileNotFoundError(
            f"[system.xml] active_robot='{active_robot}' but "
            f"{robot_xml_path} does not exist. "
            f"Available robot XMLs: {available}"
        )

    # Minimal parse — just enough to read the joint count and fallback gripper name
    robot_root, joints_elem = parse_robot_config(str(robot_xml_path))
    joint_num = get_joint_count(joints_elem)

    # active_gripper: prefer system.xml value; fall back to robot XML
    active_gripper = root.findtext("active_gripper", default="").strip()
    if not active_gripper:
        active_gripper = get_active_gripper(robot_root)

    kt_source = root.findtext("kt_source", default="auto").strip().lower()
    if kt_source not in ("auto", "xml"):
        kt_source = "auto"

    jog_defaults = _parse_jog_defaults(root.find("jog_defaults"))
    rti_defaults = _parse_rti_defaults(root.find("rti_defaults"))
    maintenance_defaults = _parse_maintenance_defaults(root.find("maintenance_defaults"))
    safety_monitor = _parse_safety_monitor(root.find("safety_monitor"))
    self_collision = _parse_self_collision(root.find("self_collision"))
    friction_comp = _parse_friction_comp(root.find("friction_comp"))
    dynamics_feedforward = _parse_dynamics_feedforward(root.find("dynamics_feedforward"))
    # <realtime_required>: 1 (default) = RTI refuses to start without SCHED_FIFO on its core (as industrial
    # real-time interfaces do); 0 = log and run degraded (a development box without the RT kernel or caps).
    _rt_text = (root.findtext("realtime_required") or "1").strip()
    realtime_required = 0 if _rt_text in ("0", "false", "no", "off") else 1
    # <can_read_first>: the CAN tick order at every RTI start (2026-10-12, user: ON after the measurement,
    # TODO/can_tx_timing_plan.md §9.11) — 1 = read first, send early (option B); 0 = the old order. Missing -> 1; a value
    # that is not 0/1 -> 1 with a warning. robot.set_can_read_first() still switches it at runtime.
    _crf_text = (root.findtext("can_read_first") or "1").strip().lower()
    if _crf_text in ("0", "false", "no", "off"):
        can_read_first = 0
    else:
        if _crf_text not in ("1", "true", "yes", "on"):
            logging.warning(f"[system.xml] <can_read_first>{_crf_text!r} is not 0/1 — ON (read first, send early)")
        can_read_first = 1
    # <control_box><pcb_version>: which PCB is fitted (hardware/control_box.py has the capability table; the
    # first consumer is the E-stop channel policy). Missing or not a number -> 1 (the board the fleet has).
    _cb = root.find("control_box")
    _pcb_text = ((_cb.findtext("pcb_version") if _cb is not None else None) or "1").strip()
    try:
        pcb_version = int(float(_pcb_text))
    except ValueError:
        logging.warning(f"[system.xml] <pcb_version>{_pcb_text!r} is not a number — assuming PCB v1")
        pcb_version = 1

    # <display_timezone>: the zone times are SHOWN in (utility/display_time.py) — "system" (default) = the Pi's own zone;
    # a name the zone database does not know falls back to "system" with a warning (a display setting never stops a start)
    display_timezone = (root.findtext("display_timezone") or "system").strip() or "system"
    if display_timezone != "system":
        try:
            import zoneinfo
            zoneinfo.ZoneInfo(display_timezone)
        except (ValueError, zoneinfo.ZoneInfoNotFoundError):
            logging.warning(f"[system.xml] <display_timezone>{display_timezone!r} is not a known time zone — using the Pi's own")
            display_timezone = "system"

    return {
        "active_robot":   active_robot,
        "robot_xml_path": robot_xml_path,
        "joint_num":      joint_num,
        "active_gripper": active_gripper,
        "kt_source":      kt_source,
        "realtime_required": realtime_required,
        "can_read_first": can_read_first,
        "pcb_version":    pcb_version,
        "jog_defaults":   jog_defaults,
        "rti_defaults":   rti_defaults,
        "maintenance_defaults": maintenance_defaults,
        "safety_monitor": safety_monitor,
        "self_collision": self_collision,
        "friction_comp":  friction_comp,
        "dynamics_feedforward": dynamics_feedforward,
        "display_timezone": display_timezone,
    }


# --- Hardcoded fallbacks if <jog_defaults> is missing or malformed in system.xml.
#     Keep aligned with the example values in the XML schema.
_JOG_DEFAULTS_FALLBACK = {
    "speed_pct":    0.20,
    "accel_time":   0.55,
    "profile":      "scurve",       # "trapezoidal" or "scurve"
    "jerk_factor":  3.0,
    "control_mode": "pid",          # "pid" or "pd"
}


# RTI policy fallbacks. Deliberately mirror the module constants in
# motion/rti_handler.py — that file remains the single definition of each default, and
# these exist so a system.xml without an <rti_defaults> block behaves identically.
_RTI_DEFAULTS_FALLBACK = {
    "command_timeout_s":    0.040,
    "stopping_hold_s":      0.500,
    "start_pose_tol_rad":   0.100,
    "lowpass_cutoff_hz":    0.0,      # 0 = off
    "success_window_ticks": 100,
    "success_warn":         0.95,
    "success_bad":          0.90,
}


_RTI_DEFAULT_BOUNDS = {
    # (min, max) — a nonsense value in the XML should be refused loudly rather than
    # silently disarming a safety feature. A 0 s timeout would stop control on the
    # first tick; a 10 s one would let a dead PC drive the arm for ten seconds.
    "command_timeout_s":    (0.004, 1.0),
    "stopping_hold_s":      (0.0,   5.0),
    "start_pose_tol_rad":   (0.001, 1.0),
    "lowpass_cutoff_hz":    (0.0,   1000.0),
    "success_window_ticks": (10,    10000),
    "success_warn":         (0.0,   1.0),
    "success_bad":          (0.0,   1.0),
}


# Maintenance-mode policy. Robot-INDEPENDENT thresholds, so system.xml rather
# than the robot XML — the value answers "how much gravity load do we accept
# before refusing to let the motors go limp", which is a risk decision, not a
# property of the arm.
_MAINTENANCE_DEFAULTS_FALLBACK = {
    # Max |joint_gravity_torque| on any joint for the arm to count as "parked"
    # when entering FLASHING. joint_gravity_torque IS the torque needed to hold
    # the current pose up, so near-zero means the pose is self-supporting and
    # survives losing torque. MEASURE THIS ON THE REAL ARM at the standby pose
    # and tune — the shipped value is deliberately strict, and the operator can
    # always override with `set_mode:FLASHING:FORCE` after supporting the arm.
    "flash_park_max_gravity_nm": 1.0,
}


_MAINTENANCE_DEFAULT_BOUNDS = {
    # An enormous ceiling would silently disable the park check, so refuse it
    # loudly rather than accept it.
    "flash_park_max_gravity_nm": (0.0, 50.0),
}


def _parse_maintenance_defaults(elem: ET.Element | None) -> dict:
    """Read the optional <maintenance_defaults> block from system.xml.

    Same contract as _parse_rti_defaults: a missing block, missing field, or
    out-of-range value falls back to the hardcoded default with a warning,
    never to a value that would disarm the check.
    """
    out = dict(_MAINTENANCE_DEFAULTS_FALLBACK)
    if elem is None:
        return out
    for key, default in _MAINTENANCE_DEFAULTS_FALLBACK.items():
        text = elem.findtext(key, default="").strip()
        if not text:
            continue
        try:
            val = float(text)
        except ValueError:
            logging.warning("[system.xml] <maintenance_defaults><%s> is not a number "
                            "(%r) — using %s", key, text, default)
            continue
        lo, hi = _MAINTENANCE_DEFAULT_BOUNDS[key]
        if not (lo <= val <= hi):
            logging.warning("[system.xml] <maintenance_defaults><%s> = %s is outside "
                            "[%s, %s] — using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    return out


# Friction compensation POLICY (utility/friction_comp.py; TODO/friction_compensation_plan.md) — robot-independent.
# The measured friction and gravity error per joint are the ARM's: <friction_sliding_nm> / <gravity_error_nm>
# (and the breakaway <friction_static_nm>) in the robot XML. The operator switch (dashboard / set_friction_comp) starts OFF at every RTI start.
_FRICTION_COMP_FALLBACK = {
    "fraction":       0.5,     # of (sliding friction - gravity error) given back: below friction -> still passive
    "sliding_ratio":  0.7,     # a joint with no <friction_sliding_nm>: sliding friction = this x its breakaway
    "band_dps":       2.0,     # deg/s: full compensation beyond this speed, 0 at rest (never breaks a joint loose)
    "fade_start_dps": 20.0,    # deg/s: fading out above this — friction matters most for SLOW motion (fine
    "fade_end_dps":   40.0,    # positioning, stick-slip); gone at this: caps how fast a joint could glide with help
}


_FRICTION_COMP_BOUNDS = {
    "fraction":       (0.0, 0.9),
    "sliding_ratio":  (0.3, 1.0),
    "band_dps":       (0.5, 30.0),
    "fade_start_dps": (5.0, 360.0),
    "fade_end_dps":   (10.0, 720.0),
}


def _parse_dynamics_feedforward(elem: ET.Element | None) -> dict:
    """<dynamics_feedforward><inertia>1</inertia><coriolis>1</coriolis></dynamics_feedforward> — the START state of the two
    switches (2026-10-10, user: both ON by default; the dashboard / set_inertia_comp / set_coriolis_comp still switch
    them at runtime). Missing block, missing key or a value that is not 0/1 -> ON (with a warning for the bad value)."""
    out = {"inertia": True, "coriolis": True}
    if elem is None:
        return out
    for key in out:
        text = (elem.findtext(key) or "").strip().lower()
        if not text:
            continue
        if text in ("1", "true", "on", "yes"):
            out[key] = True
        elif text in ("0", "false", "off", "no"):
            out[key] = False
        else:
            logging.warning("[system.xml] <dynamics_feedforward><%s> is not 0/1 (%r) — ON", key, text)
    return out


def _parse_friction_comp(elem: ET.Element | None) -> dict:
    """<friction_comp> in system.xml. A missing or out-of-range value falls back with a warning; a fade_end not above
    fade_start brings both back to their defaults."""
    out = dict(_FRICTION_COMP_FALLBACK)
    if elem is None:
        return out
    for key, default in _FRICTION_COMP_FALLBACK.items():
        text = elem.findtext(key, default="").strip()
        if not text:
            continue
        try:
            val = float(text)
        except ValueError:
            logging.warning("[system.xml] <friction_comp><%s> is not a number (%r) — using %s", key, text, default)
            continue
        lo, hi = _FRICTION_COMP_BOUNDS[key]
        if not (lo <= val <= hi):
            logging.warning("[system.xml] <friction_comp><%s> = %s is outside [%s, %s] — using %s", key, val, lo, hi,
                            default)
            continue
        out[key] = val
    if out["fade_end_dps"] <= out["fade_start_dps"]:
        logging.warning("[system.xml] <friction_comp> fade_end_dps must be above fade_start_dps — using %s / %s",
                        _FRICTION_COMP_FALLBACK["fade_start_dps"], _FRICTION_COMP_FALLBACK["fade_end_dps"])
        out["fade_start_dps"] = _FRICTION_COMP_FALLBACK["fade_start_dps"]
        out["fade_end_dps"] = _FRICTION_COMP_FALLBACK["fade_end_dps"]
    return out


# Safety monitor POLICY (utility/safety_monitor.py) — robot-independent switches and windows. The
# robot-specific numbers (tolerance, per-joint tracking limits, TCP limit) live in the robot XML.
# Windows are in SECONDS so they mean the same on a 100 Hz robot as on PAR6's 250 Hz.
# overspeed / tracking / tcp_speed ship OFF: their thresholds need measured per-joint maxima
# (TODO/safety_stack_research.md §8, step 2a).
_SAFETY_MONITOR_FALLBACK = {
    "enabled":              1,
    "joint_limit":          1,
    "endstop":              1,
    "position_sustain_s":   0.012,
    # The speed-dependent checks have a MODE: "off" | "warn" | "stop" (legacy 0/1 = off/stop). "warn" = a live
    # warning + the excursion's peak in RTI.log, never a stop — on before the thresholds are measured (user
    # decision 2026-10-01); switch to "stop" once normal use shows no false warnings.
    "overspeed":            "warn",
    "overspeed_margin":     1.2,
    "overspeed_sustain_s":  0.020,
    "tracking":             "warn",
    "tracking_sustain_s":   0.040,
    "tcp_speed":            "off",
    "tcp_speed_margin":     1.1,
    "tcp_speed_sustain_s":  0.020,
    # Safety zones (utility/zones.py). On by default: with no <active_cell> there are no zones, so
    # this costs nothing until a cell is chosen. The tolerance is how far past the zone's own surface
    # (its margin NOT counted — the margin is for prevention) the measured pose may go before a trip.
    "zones":                1,
    "zone_tolerance":       0.005,
    # A joint buzzing while it holds (2026-10-06, api_tests/59_hold_noise.py): warn-only (off | warn). The
    # per-joint threshold (mA) is a property of the arm — the robot XML's <hold_oscillation_ma>.
    "hold_oscillation":            "warn",
    "hold_oscillation_hz":         25.0,
    "hold_oscillation_sustain_s":  1.5,
    "hold_still_dps":              0.5,
}


_SAFETY_MONITOR_BOUNDS = {
    "enabled":              (0, 1),
    "joint_limit":          (0, 1),
    "endstop":              (0, 1),
    # at least one tick at 1 kHz; at most 1/4 s, past which a real excursion runs on too long
    "position_sustain_s":   (0.001, 0.25),
    "overspeed_margin":     (1.0, 3.0),      # below 1.0 it would trip at legal speeds
    "overspeed_sustain_s":  (0.001, 0.25),
    "tracking_sustain_s":   (0.001, 0.5),
    "tcp_speed_margin":     (1.0, 3.0),
    "tcp_speed_sustain_s":  (0.001, 0.25),
    "zones":                (0, 1),
    "zone_tolerance":       (0.0, 0.05),     # metres; 0 trips on the surface itself
    "hold_oscillation_hz":  (5.0, 100.0),    # under 5 Hz a hand's push and the hold's settling count; 125 Hz is Nyquist
    # under 0.5 s the kick a hold gives as it engages (0.25-0.5 s measured) would warn
    "hold_oscillation_sustain_s":  (0.5, 10.0),
    "hold_still_dps":       (0.05, 5.0),
}


# Self-collision (TODO/self_collision_plan.md, 2026-10-06). Robot-independent POLICY: the two margins, the worker's
# rate and the reaction. The geometry (convex pieces) and the per-pair overrides are properties of the arm / the
# gripper and live in their XMLs (robots/<robot>.xml <self_collision>, grippers/<gripper>.xml <collision>).
_SELF_COLLISION_FALLBACK = {
    "enabled":           1,
    "planner_margin_mm": 15.0,    # a planned path is REFUSED if two links would come closer than this (model distance)
    "monitor_margin_mm": 5.0,     # the measured pose closer than this -> SELF_COLLISION (stop)
    "jog_margin_mm":     8.0,     # a jog is blocked when the pose it would STOP at is closer than this (2026-10-07, user)
    "worker_hz":         100.0,   # the collision worker's rate (self_collision_worker.py)
    "reaction":          "stop",  # off | warn | stop — what the monitor does
    # 2026-10-07 (S3f): where the worker (monitor + jog) puts a gripper's jaws. "swept" = over their whole stroke (always
    # safe, the planner's model); "measured" = where the gripper reports them (firmware mode, calibrated, fresh data —
    # else swept), each widened by jaw_band_mm either way along its travel
    "jaws":              "swept",
    "jaw_band_mm":       3.0,
    # 2026-10-07 (S3g): the dashboard's readout shows the pair nearest its own jog margin when it is within this much
    # above it (the worker computes it every 10th cycle)
    "readout_mm":        30.0,
}
_SELF_COLLISION_BOUNDS = {
    "enabled":           (0, 1),
    "planner_margin_mm": (0.0, 100.0),
    "monitor_margin_mm": (0.0, 100.0),
    "jog_margin_mm":     (0.0, 100.0),
    "worker_hz":         (10.0, 500.0),
    "jaw_band_mm":       (0.0, 20.0),
    "readout_mm":        (0.0, 200.0),
}
_SELF_COLLISION_JAWS = ("swept", "measured")


def _parse_self_collision(elem: ET.Element | None) -> dict:
    """Read the optional <self_collision> block from system.xml (same contract as <safety_monitor>)."""
    out = dict(_SELF_COLLISION_FALLBACK)
    if elem is None:
        return out
    for key, default in _SELF_COLLISION_FALLBACK.items():
        text = elem.findtext(key, default="").strip()
        if not text:
            continue
        if key == "jaws":
            if text.lower() not in _SELF_COLLISION_JAWS:
                logging.warning("[system.xml] <self_collision><jaws> must be swept or measured (got %r) — using %s", text, default)
                continue
            out[key] = text.lower()
            continue
        if isinstance(default, str):
            mode = {"0": "off", "1": "stop"}.get(text, text.lower())
            if mode not in ("off", "warn", "stop"):
                logging.warning("[system.xml] <self_collision><%s> must be off, warn or stop (got %r) — using %s", key, text, default)
                continue
            out[key] = mode
            continue
        try:
            val = int(text) if isinstance(default, int) else float(text)
        except ValueError:
            logging.warning("[system.xml] <self_collision><%s> is not a number (%r) — using %s", key, text, default)
            continue
        lo, hi = _SELF_COLLISION_BOUNDS[key]
        if not (lo <= val <= hi):
            logging.warning("[system.xml] <self_collision><%s> = %s is outside [%s, %s] — using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    if out["monitor_margin_mm"] > out["planner_margin_mm"]:
        logging.warning("[system.xml] <self_collision> monitor_margin_mm (%s) > planner_margin_mm (%s): a legal program could "
                        "be stopped mid-move — using the planner margin for the monitor", out["monitor_margin_mm"], out["planner_margin_mm"])
        out["monitor_margin_mm"] = out["planner_margin_mm"]
    # the jog margin sits between the two: above the monitor's (a jog must come to rest before the trip line — at the
    # monitor margin itself a jog ended exactly ON it, 2026-10-07) and at most the planner's
    if out["jog_margin_mm"] <= out["monitor_margin_mm"]:
        logging.warning("[system.xml] <self_collision> jog_margin_mm (%s) <= monitor_margin_mm (%s): a jog would come to rest on "
                        "the monitor's trip line — using the monitor margin + 3 mm", out["jog_margin_mm"], out["monitor_margin_mm"])
        out["jog_margin_mm"] = out["monitor_margin_mm"] + 3.0
    if out["jog_margin_mm"] > out["planner_margin_mm"]:
        out["jog_margin_mm"] = out["planner_margin_mm"]
    return out


def _parse_safety_monitor(elem: ET.Element | None) -> dict:
    """Read the optional <safety_monitor> block from system.xml. Same contract as <rti_defaults>:
    a missing block, missing field or out-of-range value falls back with a warning."""
    out = dict(_SAFETY_MONITOR_FALLBACK)
    if elem is None:
        return out
    for key, default in _SAFETY_MONITOR_FALLBACK.items():
        text = elem.findtext(key, default="").strip()
        if not text:
            continue
        if isinstance(default, str):                 # a check mode
            mode = {"0": "off", "1": "stop"}.get(text, text.lower())
            if mode not in ("off", "warn", "stop"):
                logging.warning("[system.xml] <safety_monitor><%s> must be off, warn or stop (got %r) — using %s",
                                key, text, default)
                continue
            out[key] = mode
            continue
        try:
            val = int(text) if isinstance(default, int) else float(text)
        except ValueError:
            logging.warning("[system.xml] <safety_monitor><%s> is not a number (%r) — using %s",
                            key, text, default)
            continue
        lo, hi = _SAFETY_MONITOR_BOUNDS[key]
        if not (lo <= val <= hi):
            logging.warning("[system.xml] <safety_monitor><%s> = %s is outside [%s, %s] — using %s",
                            key, val, lo, hi, default)
            continue
        out[key] = val
    return out


def _parse_rti_defaults(elem: ET.Element | None) -> dict:
    """Read the optional <rti_defaults> block from system.xml.

    Missing block, missing field, or out-of-range value all fall back to
    _RTI_DEFAULTS_FALLBACK with a warning — never to a value that would disarm a
    safety feature.
    """
    out = dict(_RTI_DEFAULTS_FALLBACK)
    if elem is None:
        return out
    for key, default in _RTI_DEFAULTS_FALLBACK.items():
        text = elem.findtext(key, default="").strip()
        if not text:
            continue
        try:
            val = int(text) if isinstance(default, int) else float(text)
        except ValueError:
            logging.warning("[system.xml] <rti_defaults><%s> is not a number (%r) — "
                            "using %s", key, text, default)
            continue
        lo, hi = _RTI_DEFAULT_BOUNDS[key]
        if not (lo <= val <= hi):
            logging.warning("[system.xml] <rti_defaults><%s> = %s is outside [%s, %s] — "
                            "using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    if out["success_bad"] > out["success_warn"]:
        logging.warning("[system.xml] success_bad (%s) > success_warn (%s) — the 'bad' "
                        "threshold would fire first. Swapping them.",
                        out["success_bad"], out["success_warn"])
        out["success_warn"], out["success_bad"] = out["success_bad"], out["success_warn"]
    return out


def _parse_jog_defaults(elem: ET.Element | None) -> dict:
    """Read the optional <jog_defaults> section from system.xml.

    Missing or malformed individual fields fall back to _JOG_DEFAULTS_FALLBACK
    so older system.xml files (without the section) still load cleanly.
    """
    if elem is None:
        return dict(_JOG_DEFAULTS_FALLBACK)

    out = dict(_JOG_DEFAULTS_FALLBACK)

    speed = elem.findtext("speed_pct", default="").strip()
    if speed:
        try:
            out["speed_pct"] = max(0.0, min(1.0, float(speed)))
        except ValueError:
            pass

    accel = elem.findtext("accel_time", default="").strip()
    if accel:
        try:
            out["accel_time"] = max(0.05, float(accel))
        except ValueError:
            pass

    profile = elem.findtext("profile", default="").strip().lower()
    if profile in ("trapezoidal", "scurve"):
        out["profile"] = profile

    jerk = elem.findtext("jerk_factor", default="").strip()
    if jerk:
        try:
            out["jerk_factor"] = max(0.5, float(jerk))
        except ValueError:
            pass

    control_mode = elem.findtext("control_mode", default="").strip().lower()
    if control_mode in ("pid", "pd"):
        out["control_mode"] = control_mode

    return out
