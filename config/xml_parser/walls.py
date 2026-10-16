"""Per-joint freedrive walls (<freedrive_wall> in the robot XML): resolve one spec, read them all, write
them back (the dashboard's Walls tab).

Split out of the single-file config/xml_parser.py on 2026-10-03 (refactor Phase 4); definitions moved verbatim.
"""

from __future__ import annotations

from collections.abc import Sequence
import logging
logger = logging.getLogger("config")  # docs/logging.md "Logger names"
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from config.xml_parser.zones import HAPTIC_STABLE


# ---------------------------------------------------------------------------------------- freedrive walls
# Per-joint freedrive walls (utility/haptic_walls.py, docs/safety_zones.md §2b). One element per <joint>:
#   <freedrive_wall at="soft" frequency_hz="auto" damping_ratio="auto" filter_ms="auto" max_torque_nm="auto"
#                   breakthrough_deg="auto" freedrive="warn" scaling="auto"/>
# Missing element or "auto" = the default. The wall is set by its natural FREQUENCY and DAMPING RATIO — the same at
# every pose (user decision 2026-10-02); its stiffness and damping in Nm/deg follow the joint's inertia at each
# pose (utility/haptic_walls.py JOINT WALLS). filter_ms = the joint's own speed filter for the damping (auto = the
# <haptic_walls><velocity_tau>; a stiffer wall needs less filtering — the stable range follows it, HAPTIC_STABLE).
# scaling = adaptive (auto: stiffness/damping x the joint's inertia at the pose / its lightest) or fixed (the
# lightest pose's Nm/deg everywhere — softer where the joint carries more; user request 2026-10-02, to compare).
# Tuned by feel in the dashboard's Walls tab, which writes them here.
WALL_NUMBERS = ("frequency_hz", "damping_ratio", "filter_ms", "max_torque_nm", "breakthrough_deg")


WALL_SPEC_AUTO = {"at": None, "frequency_hz": None, "damping_ratio": None, "filter_ms": None, "max_torque_nm": None,
                  "breakthrough_deg": None, "freedrive": "warn", "scaling": None}


WALL_SCALINGS = ("adaptive", "fixed")


WALL_FILTER_MAX_MS = float(max(HAPTIC_STABLE))      # the stability table's last column


_WALL_FIELDS_BEFORE_2026_10_02 = ("stiffness_nm_per_deg", "damping_nm_s_per_deg")


def wall_spec(raw: dict | None, where: str = "freedrive_wall") -> dict:
    """Validate one wall spec (a dict of strings/numbers, or None) -> a clean spec dict. Bad values fall back to
    auto with a warning: a wall setting only changes how a limit FEELS; the safety monitor still watches."""
    spec = dict(WALL_SPEC_AUTO)
    if not raw:
        return spec
    at = str(raw.get("at") or "").strip().lower()
    if at in ("soft", "hard"):
        spec["at"] = at
    elif at not in ("", "auto", "none"):
        logger.warning("%s: at=%r must be soft or hard — using the default", where, raw.get("at"))
    fd = str(raw.get("freedrive") or "warn").strip().lower()
    if fd in ("warn", "stop"):
        spec["freedrive"] = fd
    else:
        logger.warning("%s: freedrive=%r must be warn or stop — using warn", where, raw.get("freedrive"))
    sc = str(raw.get("scaling") or "").strip().lower()
    if sc in WALL_SCALINGS:
        spec["scaling"] = sc
    elif sc not in ("", "auto", "none"):
        logger.warning("%s: scaling=%r must be adaptive or fixed — using auto (adaptive)", where, raw.get("scaling"))
    for old in _WALL_FIELDS_BEFORE_2026_10_02:
        val = raw.get(old)
        if val is not None and str(val).strip().lower() not in ("", "auto", "none"):
            logger.warning("%s: %s=%r is no longer used — the wall is set by frequency_hz and damping_ratio "
                            "(2026-10-02); using auto. Re-tune it in the Walls tab.", where, old, val)
    for key in WALL_NUMBERS:
        val = raw.get(key)
        if val is None or str(val).strip().lower() in ("", "auto", "none"):
            continue
        try:
            v = float(val)
        except (TypeError, ValueError):
            logger.warning("%s: %s=%r is not a number — using auto", where, key, val)
            continue
        if not (np.isfinite(v) and v >= 0.0):
            logger.warning("%s: %s=%r must be >= 0 — using auto", where, key, val)
            continue
        if key == "filter_ms" and v > WALL_FILTER_MAX_MS:
            logger.warning("%s: filter_ms=%r is past the stability table (%g ms) — using auto", where, val,
                            WALL_FILTER_MAX_MS)
            continue
        spec[key] = v
    return spec


def get_freedrive_walls(joints: ET.Element) -> list[dict]:
    """Per-joint wall specs from the robot XML's <joint> blocks (see wall_spec)."""
    out = []
    for i, joint in enumerate(joints.findall("joint")):
        el = joint.find("freedrive_wall")
        out.append(wall_spec(dict(el.attrib) if el is not None else None, f"robot XML joint {i} <freedrive_wall>"))
    return out


def _wall_attr(v: float | None) -> str:
    return "auto" if v is None else f"{v:g}"


def write_freedrive_walls(xml_path: str | Path, specs: Sequence[dict | None]) -> int:
    """Write the per-joint wall specs into the robot XML, replacing each joint's <freedrive_wall .../> or adding one
    before </joint>. Text-level, so comments and formatting elsewhere are untouched. Returns the joints written."""
    import re
    path = Path(xml_path)
    text = path.read_text()
    for i, spec in enumerate(specs):
        spec = wall_spec(spec)
        m = re.search(rf'<joint id="{i}">(.*?)</joint>', text, flags=re.S)
        if m is None:
            raise ValueError(f"{path.name}: no <joint id=\"{i}\"> block")
        element = (f'<freedrive_wall at="{spec["at"] or "auto"}" frequency_hz="{_wall_attr(spec["frequency_hz"])}" '
                   f'damping_ratio="{_wall_attr(spec["damping_ratio"])}" filter_ms="{_wall_attr(spec["filter_ms"])}" '
                   f'max_torque_nm="{_wall_attr(spec["max_torque_nm"])}" breakthrough_deg="{_wall_attr(spec["breakthrough_deg"])}" '
                   f'freedrive="{spec["freedrive"]}" scaling="{spec["scaling"] or "auto"}"/>')
        body = m.group(1)
        if re.search(r"<freedrive_wall\b[^>]*/>", body):
            body = re.sub(r"<freedrive_wall\b[^>]*/>", element, body, count=1)
        else:
            indent = re.search(r"\n(\s*)<tracking_limit_deg>", body)
            pad = indent.group(1) if indent else "            "
            content = body.rstrip()           # insert after the last element; keep the whitespace (and blank lines) after it
            body = (content + f"\n{pad}{element} <!-- freedrive wall (docs/safety_zones.md §2b); the dashboard's "
                    f"Walls tab writes this -->" + body[len(content):])
        text = text[:m.start(1)] + body + text[m.end(1):]
    ET.fromstring(text)                    # never write a file that no longer parses
    path.write_text(text)
    return len(specs)
