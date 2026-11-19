"""config/system.xml: the active robot / gripper / cell and the robot-independent policies — jog and RTI
defaults, maintenance (flashing park gate), friction-compensation policy, safety-monitor switches. Every
policy block falls back to its hardcoded default, with a warning, when it is missing or out of range.

Split out of the single-file config/xml_parser.py on 2026-10-03 (refactor Phase 4); definitions moved verbatim.
"""

from __future__ import annotations

import logging
import math
import os
logger = logging.getLogger("config")  # docs/logging.md "Logger names"
import xml.etree.ElementTree as ET
from pathlib import Path

from config.schema_versions import check_schema
from config.xml_parser.xml_access import get_active_gripper, get_joint_count, parse_robot_config
from utility.sound import DEFAULT_TUNES as _DEFAULT_TUNES, TUNES as _TUNES      # the sound library (<sound> tunes)

# <drive_watchdog_ms> (2026-10-13): every drive's CAN watchdog. 0 would switch it off in the drive firmware; the
# minimum is 2x the tick-50/150/300 configuration bursts (~200 frames ahead of that tick's commands: a drive went 40 ms
# without a fresh reply after the tick-50 burst on 2026-10-13); above 1 s a drive keeps running RTI's last speed or
# torque long after RTI is gone.
DRIVE_WATCHDOG_DEFAULT_MS = 100
DRIVE_WATCHDOG_MIN_MS = 80
DRIVE_WATCHDOG_MAX_MS = 1000

# <kt_tolerance> (2026-11-15, user: ±30 %): how far a drive's own Kt may be from the robot XML's before RTI keeps the
# XML value. A fraction. The drives reported up to +20 % on 2026-11-07 (J5 0.219 against 0.183); the band is for a
# value a reflash or recalibration made wild, not for those.
KT_TOLERANCE_DEFAULT = 0.3
KT_TOLERANCE_MIN = 0.05
KT_TOLERANCE_MAX = 1.0


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
    check_schema(root, "system", system_xml_path)   # a newer format than this runtime reads: refused (2026-10-21)
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
    # RCB_ACTIVE_GRIPPER (2026-11-18): the nets' reference tool, set by tools/run_nets.py for every net — so a test does
    # not depend on the tool this robot has fitted (with the Flange on, nine nets failed). The robot's processes never
    # set it, so they run with system.xml's <active_gripper>.
    if os.environ.get("RCB_ACTIVE_GRIPPER"):
        active_gripper = os.environ["RCB_ACTIVE_GRIPPER"]

    kt_source = root.findtext("kt_source", default="auto").strip().lower()
    if kt_source not in ("auto", "xml"):
        kt_source = "auto"
    # <kt_tolerance>: with kt_source auto, a drive's Kt outside the XML's ±this keeps the XML value (a warning names it).
    # Missing -> the default; not a number in [MIN, MAX] -> the default with a warning.
    _ktt_text = (root.findtext("kt_tolerance") or str(KT_TOLERANCE_DEFAULT)).strip()
    try:
        kt_tolerance = float(_ktt_text)
    except ValueError:
        kt_tolerance = float("nan")
    if not KT_TOLERANCE_MIN <= kt_tolerance <= KT_TOLERANCE_MAX:      # NaN fails it too
        logger.warning(f"system.xml <kt_tolerance>{_ktt_text!r} is not a fraction in "
                       f"{KT_TOLERANCE_MIN}..{KT_TOLERANCE_MAX} — using {KT_TOLERANCE_DEFAULT}")
        kt_tolerance = KT_TOLERANCE_DEFAULT

    jog_defaults = _parse_jog_defaults(root.find("jog_defaults"))
    rti_defaults = _parse_rti_defaults(root.find("rti_defaults"))
    maintenance_defaults = _parse_maintenance_defaults(root.find("maintenance_defaults"))
    safety_monitor = _parse_safety_monitor(root.find("safety_monitor"))
    self_collision = _parse_self_collision(root.find("self_collision"))
    friction_comp = _parse_friction_comp(root.find("friction_comp"))
    dynamics_feedforward = _parse_dynamics_feedforward(root.find("dynamics_feedforward"))
    sound = _parse_sound(root.find("sound"))
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
            logger.warning(f"system.xml <can_read_first>{_crf_text!r} is not 0/1 — ON (read first, send early)")
        can_read_first = 1
    # <drive_watchdog_ms>: every drive's CAN watchdog in ms (2026-10-13, user: "normal times" — it was 5000 per drive in
    # the robot / gripper XMLs, a bring-up value, no longer read). One value for every drive: it is about RTI's link,
    # not the arm. Missing -> the default; not a whole number in [MIN, MAX] -> the default with a warning.
    _wd_text = (root.findtext("drive_watchdog_ms") or str(DRIVE_WATCHDOG_DEFAULT_MS)).strip()
    try:
        drive_watchdog_ms = int(_wd_text)
    except ValueError:
        drive_watchdog_ms = None
    if drive_watchdog_ms is None or not DRIVE_WATCHDOG_MIN_MS <= drive_watchdog_ms <= DRIVE_WATCHDOG_MAX_MS:
        logger.warning(f"system.xml <drive_watchdog_ms>{_wd_text!r} is not a whole number of ms in "
                        f"{DRIVE_WATCHDOG_MIN_MS}..{DRIVE_WATCHDOG_MAX_MS} — using {DRIVE_WATCHDOG_DEFAULT_MS} ms")
        drive_watchdog_ms = DRIVE_WATCHDOG_DEFAULT_MS
    # <control_box><pcb_version>: which PCB is fitted (hardware/control_box.py has the capability table; the
    # first consumer is the E-stop channel policy). Missing or not a number -> 1 (the board the fleet has).
    _cb = root.find("control_box")
    _pcb_text = ((_cb.findtext("pcb_version") if _cb is not None else None) or "1").strip()
    try:
        pcb_version = int(float(_pcb_text))
    except ValueError:
        logger.warning(f"system.xml <pcb_version>{_pcb_text!r} is not a number — assuming PCB v1")
        pcb_version = 1

    # <display_timezone>: the zone times are SHOWN in (utility/display_time.py) — "system" (default) = the Pi's own zone;
    # a name the zone database does not know falls back to "system" with a warning (a display setting never stops a start)
    display_timezone = (root.findtext("display_timezone") or "system").strip() or "system"
    if display_timezone != "system":
        try:
            import zoneinfo
            zoneinfo.ZoneInfo(display_timezone)
        except (ValueError, zoneinfo.ZoneInfoNotFoundError):
            logger.warning(f"system.xml <display_timezone>{display_timezone!r} is not a known time zone — using the Pi's own")
            display_timezone = "system"

    return {
        "active_robot":   active_robot,
        "robot_xml_path": robot_xml_path,
        "joint_num":      joint_num,
        "active_gripper": active_gripper,
        "kt_source":      kt_source,
        "kt_tolerance":   kt_tolerance,
        "realtime_required": realtime_required,
        "can_read_first": can_read_first,
        "drive_watchdog_ms": drive_watchdog_ms,
        "pcb_version":    pcb_version,
        "jog_defaults":   jog_defaults,
        "rti_defaults":   rti_defaults,
        "maintenance_defaults": maintenance_defaults,
        "safety_monitor": safety_monitor,
        "self_collision": self_collision,
        "friction_comp":  friction_comp,
        "dynamics_feedforward": dynamics_feedforward,
        "display_timezone": display_timezone,
        "sound":          sound,
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
            logger.warning("system.xml <maintenance_defaults><%s> is not a number "
                            "(%r) — using %s", key, text, default)
            continue
        lo, hi = _MAINTENANCE_DEFAULT_BOUNDS[key]
        if not (lo <= val <= hi):
            logger.warning("system.xml <maintenance_defaults><%s> = %s is outside "
                            "[%s, %s] — using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    return out


# Friction compensation POLICY (utility/friction_comp.py; TODO/friction_compensation_plan.md) — robot-independent.
# The measured friction and gravity error per joint are the ARM's: <friction_coulomb_nm> (+ <friction_load_per_nm>;
# the older <friction_sliding_nm> when missing) / <gravity_error_nm> (and the breakaway <friction_static_nm>) in the
# robot XML, api_tests/77. The operator switch (dashboard / set_friction_comp) starts OFF at every RTI start.
_FRICTION_COMP_FALLBACK = {
    "fraction":       0.5,     # of (sliding friction - gravity error) given back: below friction -> still passive
    "sliding_ratio":  0.7,     # a joint with no measured sliding friction: sliding friction = this x its breakaway
    # the share of the friction MODEL (the typical friction, api_tests/77) freedrive relies on: the friction measured
    # went down to 0.69x the model (J4 at 0 deg; J6 0.84, J5 0.88, J3 0.90, J1 / J2 0.93) — with 0.7 a joint released
    # while moving still stops wherever its real friction is at least 0.7x the model, at its worst gravity error
    "model_share":    0.7,
    "band_dps":       2.0,     # deg/s: full compensation beyond this speed, 0 at rest (never breaks a joint loose)
    "fade_start_dps": 20.0,    # deg/s: fading out above this — friction matters most for SLOW motion (fine
    "fade_end_dps":   40.0,    # positioning, stick-slip); gone at this: caps how fast a joint could glide with help
}


_FRICTION_COMP_BOUNDS = {
    "fraction":       (0.0, 0.9),
    "sliding_ratio":  (0.3, 1.0),
    "model_share":    (0.3, 1.0),
    "band_dps":       (0.5, 30.0),
    "fade_start_dps": (5.0, 360.0),
    "fade_end_dps":   (10.0, 720.0),
}


def _parse_dynamics_feedforward(elem: ET.Element | None) -> dict:
    """<dynamics_feedforward><inertia>1</inertia><coriolis>1</coriolis></dynamics_feedforward> — the START state of the two
    switches (2026-10-10, user: both ON by default; the dashboard / set_inertia_comp / set_coriolis_comp still switch
    them at runtime). Missing block, missing key or a value that is not 0/1 -> ON (with a warning for the bad value).
    <friction> (2026-10-17): the friction feedforward's start state — OFF when missing or not 0/1 (not yet measured on
    the arm with api_tests/66); <friction_fraction>: the share of the friction model it gives back, [0, 1], default 1."""
    out: dict[str, bool | float] = {"inertia": True, "coriolis": True}
    if elem is None:
        out.update(friction=False, friction_fraction=1.0)
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
            logger.warning("system.xml <dynamics_feedforward><%s> is not 0/1 (%r) — ON", key, text)
    text = (elem.findtext("friction") or "").strip().lower()
    out["friction"] = text in ("1", "true", "on", "yes")
    if text and text not in ("1", "true", "on", "yes", "0", "false", "off", "no"):
        logger.warning("system.xml <dynamics_feedforward><friction> is not 0/1 (%r) — OFF", text)
    text = (elem.findtext("friction_fraction") or "").strip()
    out["friction_fraction"] = 1.0
    if text:
        try:
            val = float(text)
        except ValueError:
            val = None
        if val is None or not 0.0 <= val <= 1.0:
            logger.warning("system.xml <dynamics_feedforward><friction_fraction> %r is not in [0, 1] — using 1", text)
        else:
            out["friction_fraction"] = val
    return out


# <sound> (2026-11-17, utility/sound.py): each use switched on (1) / off (0), the axis, the loudness, the drive that plays,
# the sound's voltage cap. A missing block / value = these; an unreadable or out-of-range value = its default + a warning.
_SOUND_FALLBACK = {"boot": 1, "homing_done": 1, "error": 1, "identify": 1, "programs": 1, "axis": "q",
                   "current_ma": 290.0, "node": 1, "limit_mv": 3600.0,
                   **{f"{e}_tune": t for e, t in _DEFAULT_TUNES.items()}}
_SOUND_BOUNDS = {"current_ma": (10.0, 1000.0), "node": (0, 15), "limit_mv": (500.0, 6000.0)}
SOUND_SWITCHES = ("boot", "homing_done", "error", "identify", "programs")


def _parse_sound(elem: ET.Element | None) -> dict:
    """<sound> in system.xml (see _SOUND_FALLBACK)."""
    out = dict(_SOUND_FALLBACK)
    if elem is None:
        return out
    for key, default in _SOUND_FALLBACK.items():
        text = elem.findtext(key, default="").strip().lower()
        if not text:
            continue
        if key in SOUND_SWITCHES:
            if text in ("0", "1"):
                out[key] = int(text)
            else:
                logger.warning("system.xml <sound><%s> is %r, not 0 / 1 — using %s", key, text, default)
        elif key == "axis":
            if text in ("d", "q"):
                out[key] = text
            else:
                logger.warning("system.xml <sound><axis> is %r, not d / q — using %s", text, default)
        elif key.endswith("_tune"):
            if text in _TUNES:
                out[key] = text
            else:
                logger.warning("system.xml <sound><%s> is %r, not a tune of the library (utility/sound.TUNES) — "
                               "using %s", key, text, default)
        else:
            try:
                val = float(text)
            except ValueError:
                logger.warning("system.xml <sound><%s> is not a number (%r) — using %s", key, text, default)
                continue
            lo, hi = _SOUND_BOUNDS[key]
            if not lo <= val <= hi:
                logger.warning("system.xml <sound><%s> = %s is outside [%s, %s] — using %s", key, val, lo, hi, default)
                continue
            out[key] = int(val) if key == "node" else val
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
            logger.warning("system.xml <friction_comp><%s> is not a number (%r) — using %s", key, text, default)
            continue
        lo, hi = _FRICTION_COMP_BOUNDS[key]
        if not (lo <= val <= hi):
            logger.warning("system.xml <friction_comp><%s> = %s is outside [%s, %s] — using %s", key, val, lo, hi,
                            default)
            continue
        out[key] = val
    if out["fade_end_dps"] <= out["fade_start_dps"]:
        logger.warning("system.xml <friction_comp> fade_end_dps must be above fade_start_dps — using %s / %s",
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
    # The command check (2026-10-27): what the robot COMMANDS in EXEC / JOG / RTI / SOFT_RTI against the robot XML's
    # speed / acceleration ceilings x margin and the soft limits — a jump only a bug of ours makes. Shipped warn (user:
    # "warn first, stop later").
    "command_check":        "warn",
    "command_check_margin": 1.2,
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
    # Collision detection (2026-10-16): the external torque estimate (momentum observer) minus the baseline taken at rest
    # before each move, against the robot XML's per-joint <contact_torque_nm> / <collision_torque_nm>; EXEC / JOG / RTI /
    # SOFT_RTI only. Shipped warn (user: warn-only first, then the normal safety stop); a PC program can switch it.
    "collision":            "warn",
    "collision_sustain_s":  0.012,
    "collision_settle_s":   0.5,
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
    "command_check_margin": (1.0, 3.0),      # below 1.0 a move planned at a ceiling would trip
    "zones":                (0, 1),
    "zone_tolerance":       (0.0, 0.05),     # metres; 0 trips on the surface itself
    "hold_oscillation_hz":  (5.0, 100.0),    # under 5 Hz a hand's push and the hold's settling count; 125 Hz is Nyquist
    # under 0.5 s the kick a hold gives as it engages (0.25-0.5 s measured) would warn
    "hold_oscillation_sustain_s":  (0.5, 10.0),
    "hold_still_dps":       (0.05, 5.0),
    "collision_sustain_s":  (0.001, 0.1),    # the observer already averages over 20 ms; past 0.1 s a collision pushes on
    "collision_settle_s":   (0.0, 5.0),      # how long after a move the check still runs (the arm settling)
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
                logger.warning("system.xml <self_collision><jaws> must be swept or measured (got %r) — using %s", text, default)
                continue
            out[key] = text.lower()
            continue
        if isinstance(default, str):
            mode = {"0": "off", "1": "stop"}.get(text, text.lower())
            if mode not in ("off", "warn", "stop"):
                logger.warning("system.xml <self_collision><%s> must be off, warn or stop (got %r) — using %s", key, text, default)
                continue
            out[key] = mode
            continue
        try:
            val = int(text) if isinstance(default, int) else float(text)
        except ValueError:
            logger.warning("system.xml <self_collision><%s> is not a number (%r) — using %s", key, text, default)
            continue
        lo, hi = _SELF_COLLISION_BOUNDS[key]
        if not (lo <= val <= hi):
            logger.warning("system.xml <self_collision><%s> = %s is outside [%s, %s] — using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    if out["monitor_margin_mm"] > out["planner_margin_mm"]:
        logger.warning("system.xml <self_collision> monitor_margin_mm (%s) > planner_margin_mm (%s): a legal program could "
                        "be stopped mid-move — using the planner margin for the monitor", out["monitor_margin_mm"], out["planner_margin_mm"])
        out["monitor_margin_mm"] = out["planner_margin_mm"]
    # the jog margin sits between the two: above the monitor's (a jog must come to rest before the trip line — at the
    # monitor margin itself a jog ended exactly ON it, 2026-10-07) and at most the planner's
    if out["jog_margin_mm"] <= out["monitor_margin_mm"]:
        logger.warning("system.xml <self_collision> jog_margin_mm (%s) <= monitor_margin_mm (%s): a jog would come to rest on "
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
                logger.warning("system.xml <safety_monitor><%s> must be off, warn or stop (got %r) — using %s",
                                key, text, default)
                continue
            out[key] = mode
            continue
        try:
            val = int(text) if isinstance(default, int) else float(text)
        except ValueError:
            logger.warning("system.xml <safety_monitor><%s> is not a number (%r) — using %s",
                            key, text, default)
            continue
        lo, hi = _SAFETY_MONITOR_BOUNDS[key]
        if not (lo <= val <= hi):
            logger.warning("system.xml <safety_monitor><%s> = %s is outside [%s, %s] — using %s",
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
            logger.warning("system.xml <rti_defaults><%s> is not a number (%r) — "
                            "using %s", key, text, default)
            continue
        lo, hi = _RTI_DEFAULT_BOUNDS[key]
        if not (lo <= val <= hi):
            logger.warning("system.xml <rti_defaults><%s> = %s is outside [%s, %s] — "
                            "using %s", key, val, lo, hi, default)
            continue
        out[key] = val
    if out["success_bad"] > out["success_warn"]:
        logger.warning("system.xml success_bad (%s) > success_warn (%s) — the 'bad' "
                        "threshold would fire first. Swapping them.",
                        out["success_bad"], out["success_warn"])
        out["success_warn"], out["success_bad"] = out["success_bad"], out["success_warn"]
    return out


def _parse_jog_defaults(elem: ET.Element | None) -> dict:
    """Read the optional <jog_defaults> section from system.xml.

    Missing fields fall back to _JOG_DEFAULTS_FALLBACK so older system.xml files (without the section) still load
    cleanly. A field that is present but unusable — not a number, not finite, out of its range, an unknown word — also
    gets the default, with a WARNING naming the field (2026-11-19, review K2: speed_pct 20, as every API speed since
    protocol 2, or nan was clamped to 1.0 = FULL jog speed without a word, accel_time nan gave the hardest ramp).
    """
    if elem is None:
        return dict(_JOG_DEFAULTS_FALLBACK)

    out = dict(_JOG_DEFAULTS_FALLBACK)

    def number(tag: str, ok, rule: str) -> None:
        text = elem.findtext(tag, default="").strip()
        if not text:
            return
        try:
            value = float(text)
        except ValueError:
            value = math.nan
        if math.isfinite(value) and ok(value):
            out[tag] = value
        else:
            logger.warning("system.xml <jog_defaults><%s>%s</%s>: must be %s — using %s", tag, text, tag, rule,
                           _JOG_DEFAULTS_FALLBACK[tag])

    def word(tag: str, allowed: tuple[str, ...]) -> None:
        text = elem.findtext(tag, default="").strip().lower()
        if not text:
            return
        if text in allowed:
            out[tag] = text
        else:
            logger.warning("system.xml <jog_defaults><%s>%s</%s>: must be %s — using %s", tag, text, tag,
                           " or ".join(allowed), _JOG_DEFAULTS_FALLBACK[tag])

    number("speed_pct", lambda v: 0.0 < v <= 1.0, "a fraction above 0 and at most 1.0 (0.20 = 20 %)")
    number("accel_time", lambda v: v >= 0.05, "a number of at least 0.05 s")
    word("profile", ("trapezoidal", "scurve"))
    number("jerk_factor", lambda v: v >= 0.5, "a number of at least 0.5")
    word("control_mode", ("pid", "pd"))
    return out
