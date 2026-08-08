"""Parse the selectable-filter XML into per-signal, per-joint filter banks.

A filter config is a small XML file (hot-reloadable — see the manager below) that assigns
a filter to each derived signal, optionally overriding individual joints:

    <filters>
        <speed  type="ema"  tau="0.0246"/>
        <accel  type="none"/>                <!-- inherit speed's pole, no extra filter -->
        <jerk   type="ema2" tau="0.03">
            <joint idx="4" type="median" window="5"/>   <!-- J4 has a bad encoder -->
        </jerk>
        <torque type="ema"  tau="0.02"/>
        <gripper_speed type="ema" tau="0.02"/>
    </filters>

Per the design decision (2026-07-17) each joint may use a fully independent filter TYPE and
params; a <joint idx=N> override inherits the signal's default type/params and replaces only
the attributes it names. tau is in SECONDS (delta_t-independent — see signal_filters.py).

This module only PARSES and BUILDS; it holds no shared-memory or RTI knowledge, so it stays
unit-testable in isolation. Validation is delegated to signal_filters.create_filter, which
raises on a bad kind / missing tau / even window etc. — so a broken config fails LOUDLY at
load, never silently at runtime.
"""

import logging
import xml.etree.ElementTree as ET
from pathlib import Path

from utility.signal_filters import FILTER_KINDS, create_filter_bank

logger = logging.getLogger(__name__)

# Signals grouped by channel count: per-joint (joint_num), the 6-DOF Cartesian signals, and
# the single-channel gripper.
JOINT_SIGNALS = ("speed", "accel", "jerk", "torque")
TCP_SIGNALS = ("tcp_accel",)          # 6 channels (Cartesian twist/accel DOF)
GRIPPER_SIGNALS = ("gripper_speed",)  # 1 channel
ALL_SIGNALS = JOINT_SIGNALS + TCP_SIGNALS + GRIPPER_SIGNALS


def _channels(signal, joint_num):
    """Channel count for a signal: 6 for Cartesian, 1 for the gripper, else joint_num."""
    if signal in TCP_SIGNALS:
        return 6
    if signal in GRIPPER_SIGNALS:
        return 1
    return joint_num


def default_filter_config():
    """The starting recommendation, used when no filter XML is present.

    Grounded in the 2026-07-17 rest-noise measurement + the poles==differentiations rule:
    speed is measured (0 diffs) so 1 pole; accel is 1 diff off filtered speed so it needs
    NO extra pole (a 2nd was measured to make it worse); jerk is 2 diffs so it needs 2 poles
    (ema2); torque is measured (0 diffs) so 1 pole. tau values are STARTING points — refine
    on live motion. These are NOT "today's behaviour": jerk_filtered/torque_filtered are new
    fields, so there is nothing to stay bit-compatible with.
    """
    return {
        "speed":         {"kind": "ema",  "tau": 0.0246},   # == the old hardcoded alpha=0.15
        "accel":         {"kind": "none"},                  # inherits speed's pole
        "jerk":          {"kind": "ema2", "tau": 0.03},     # 2 poles for 2 differentiations
        "torque":        {"kind": "ema",  "tau": 0.02},
        "tcp_accel":     {"kind": "ema",  "tau": 0.0246},   # == the old hardcoded TCP-accel EMA
        "gripper_speed": {"kind": "ema",  "tau": 0.02},
    }


def _spec_from_attrs(el, keys=("type", "tau", "window", "poly")):
    """Pull only the filter attributes present on an element into a spec dict.

    'type' maps to 'kind'. Absent attributes are omitted (so a per-joint override inherits
    them from the signal default) rather than defaulted here.
    """
    spec = {}
    for k in keys:
        v = el.get(k)
        if v is None:
            continue
        if k == "type":
            spec["kind"] = v
        elif k == "poly" or k == "window":
            spec[k] = int(v)
        else:
            spec[k] = float(v)
    return spec


def _parse_signal(el, channels):
    """Return a list of `channels` spec dicts for one signal element.

    The element's own attributes are the default for every channel; each <joint idx=N>
    child inherits that default and overrides only the attributes it specifies.
    """
    default = _spec_from_attrs(el)
    default.setdefault("kind", "none")
    if default["kind"] not in FILTER_KINDS:
        raise ValueError(f"<{el.tag}> has unknown filter type '{default['kind']}'")

    specs = [dict(default) for _ in range(channels)]
    for j in el.findall("joint"):
        raw_idx = j.get("idx")
        if raw_idx is None:
            raise ValueError(f"<{el.tag}> has a <joint> with no idx")
        idx = int(raw_idx)
        if not 0 <= idx < channels:
            raise ValueError(f"<{el.tag}> joint idx={idx} out of range 0..{channels - 1}")
        override = dict(default)
        override.update(_spec_from_attrs(j))
        specs[idx] = override
    return specs


def parse_filter_config(source, joint_num):
    """Parse a <filters> config into {signal: [spec per channel]}.

    `source` may be a path (str/Path) or an already-parsed ElementTree Element. Any signal
    absent from the file falls back to default_filter_config() so a partial file is legal.
    Raises ValueError on a malformed file — callers fall back to the default and log.
    """
    if isinstance(source, (str, Path)):
        root = ET.parse(Path(source)).getroot()
    else:
        root = source
    if root.tag != "filters":
        raise ValueError(f"filter config root must be <filters>, got <{root.tag}>")

    defaults = default_filter_config()
    config = {}
    for sig in ALL_SIGNALS:
        channels = _channels(sig, joint_num)
        el = root.find(sig)
        if el is None:
            config[sig] = [dict(defaults[sig]) for _ in range(channels)]
        else:
            config[sig] = _parse_signal(el, channels)
    return config


def build_filter_banks(config, joint_num, dt):
    """Turn a parsed config into ready-to-run banks: {signal: bank}. STARTUP/RELOAD only.

    Delegates every spec to create_filter_bank, so an invalid combination (median with an
    even window, ema without tau, ...) raises here, at load time.
    """
    banks = {}
    for sig, specs in config.items():
        channels = _channels(sig, joint_num)
        banks[sig] = create_filter_bank(specs, channels, dt)
    return banks


def load_filter_banks(path, joint_num, dt):
    """Best-effort load: parse `path` if it exists and is valid, else fall back to defaults.

    Never raises — a missing or broken filter file must not stop the robot from booting; it
    degrades to the built-in default and logs loudly. Returns (banks, source_str).
    """
    if path is not None and Path(path).exists():
        try:
            config = parse_filter_config(path, joint_num)
            banks = build_filter_banks(config, joint_num, dt)
            return banks, str(path)
        except (ET.ParseError, ValueError, OSError) as e:
            logger.error(f"[filters] '{path}' invalid ({e}) — falling back to default filters")
    else:
        logger.info("[filters] no filter config file — using default filters")
    banks = build_filter_banks(
        {s: [dict(default_filter_config()[s])] * _channels(s, joint_num)
         for s in ALL_SIGNALS},
        joint_num, dt,
    )
    return banks, "default"
