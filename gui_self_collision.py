"""The dashboard's self-collision readout (2026-10-07, TODO/self_collision_plan.md S3g).

One status-bar badge, on every tab: is the collision worker watching, and how close is the arm to where a jog would stop?
  SELF-COLLISION: OFF / STALE / not homed     — nothing is being checked (STALE = the worker's heartbeat stopped)
  SC: upper arm ↔ tool 12 mm (jog stops at 5) — the pair nearest its OWN jog margin, when within <readout_mm> of it
  SC: clear                                    — nothing that close
  SC: STOP LINE upper arm ↔ tool 1.6 mm         — inside the monitor margin (RTI stops an active mode there)
The tooltip carries the margins in force, where the jaws are taken to be (measured / swept) and the worker's slowest
recent cycle. Reads the worker's shared-memory fields only (docs/variable_reference.md); refreshes 5x per second and
sends the browser something only when the text changes.
"""
from __future__ import annotations

import math
import time

from utility.self_collision import PAR6_NAMES

STALE_S = 0.5            # the monitor's SELF_COLLISION_STALE time: a heartbeat this old means nothing is watching
NEAR_WARN_MM = 10.0      # the badge turns orange when the nearest pair is this close to its jog margin


def _pair(a: float, b: float) -> str:
    ia, ib = int(a), int(b)
    if 0 <= ia < len(PAR6_NAMES) and 0 <= ib < len(PAR6_NAMES):
        return f"{PAR6_NAMES[ia]} ↔ {PAR6_NAMES[ib]}"
    return f"frames {ia} ↔ {ib}"


def readout(rti: dict, policy: dict, alive: bool, homed: bool) -> tuple[str, str, str]:
    """(badge text, quasar colour, tooltip) from the worker's fields — pure, for the nets."""
    margins = (f"Margins: planner {policy['planner_margin_mm']:g} / jog {policy['jog_margin_mm']:g} / monitor "
               f"{policy['monitor_margin_mm']:g} mm (config/system.xml <self_collision>; pairs close by design have their own).")
    if not policy.get("enabled", 1):
        return "SELF-COLLISION: OFF", "grey", "Self-collision checking is OFF in config/system.xml <self_collision><enabled>."
    jaw = float(rti["sc_jaw_closed"][0])
    jaws = (f"Jaws: where the gripper reports them, {100 * jaw:.0f} % closed (± {policy.get('jaw_band_mm', 0):g} mm)." if jaw >= 0
            else "Jaws: swept over their whole stroke" + (" (the gripper's position is not trustworthy now)."
                                                          if policy.get("jaws") == "measured" else "."))
    worker = f"Worker: slowest recent cycle {float(rti['sc_cycle_ms'][0]):.1f} ms (period {1000 / float(policy['worker_hz']):.0f} ms)."
    tip = " ".join((margins, jaws, worker))
    if not alive:
        return ("SELF-COLLISION: STALE", "negative",
                f"The collision worker's heartbeat has not moved for {STALE_S} s — NOTHING is watching self-collisions "
                f"(RTI reports SELF_COLLISION_STALE). See logs/self_collision.log. " + margins)
    if not homed:
        return "SELF-COLLISION: not homed", "grey", "Nothing is checked until every joint is homed (the angles mean nothing). " + tip
    if float(rti["sc_hit"][0]) >= 0.5:
        return (f"SC: STOP LINE {_pair(rti['sc_pair_a'][0], rti['sc_pair_b'][0])} {1e3 * float(rti['sc_min_gap'][0]):.1f} mm",
                "negative", "Two links are inside the MONITOR margin: an active mode is stopped (SELF_COLLISION); only "
                            "moving them apart is allowed. " + tip)
    gap = float(rti["sc_near_gap"][0])
    if float(rti["sc_near_a"][0]) >= 0 and math.isfinite(gap):
        m = 1e3 * float(rti["sc_near_margin"][0])
        text = f"SC: {_pair(rti['sc_near_a'][0], rti['sc_near_b'][0])} {1e3 * gap:.0f} mm (jog stops at {m:g})"
        return text, ("orange" if 1e3 * gap - m < NEAR_WARN_MM else "primary"), \
            "The pair nearest its own jog margin (where a jog toward it comes to rest). " + tip
    return ("SC: clear", "positive",
            f"No pair within {policy.get('readout_mm', 30):g} mm of its jog margin. " + tip)


def add_self_collision_badge(shared_data, policy: dict, joint_num: int):
    """Put the badge in the current row; returns (badge, refresh) — refresh for the nets."""
    from nicegui import ui
    badge = ui.badge("SC: ---").props("color=grey outline")
    with badge:
        tip = ui.tooltip("")
    state = {"hb": None, "t": time.monotonic(), "shown": None}

    def refresh():
        rti = shared_data.rti
        now = time.monotonic()
        hb = float(rti["sc_heartbeat"][0])
        if hb != state["hb"]:
            state["hb"], state["t"] = hb, now
        alive = now - state["t"] < STALE_S
        homed = bool((rti["joint_homed"][:joint_num] == 1).all())
        text, colour, tooltip = readout(rti, policy, alive, homed)
        if (text, colour, tooltip) == state["shown"]:      # send the browser something only when it changed
            return
        state["shown"] = (text, colour, tooltip)
        badge.text = text
        badge.props(f"color={colour} outline")
        tip.text = tooltip

    ui.timer(0.2, refresh)
    return badge, refresh
