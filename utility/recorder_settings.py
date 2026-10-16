"""The recorder's settings (2026-10-16): config/system.xml `<recorder .../>`, changed from the Recordings tab.

    <recorder before_s="5" after_s="2" incident_keep="20" min_interval_s="10" capture_before_s="5" capture_keep="10"/>

  incidents (automatic, on a hard error):
  before_s          seconds kept BEFORE the fault
  after_s           seconds an incident dump waits for and keeps AFTER the fault
  incident_keep     incident files kept in temp_data/incidents (the oldest deleted first)
  min_interval_s    at most one incident dump per this many seconds (the faults in between are counted, not saved)
  captures (on request):
  capture_before_s  seconds kept before the request (2026-10-16, user: captures and incidents are separate things;
                    until then before_s served both)
  capture_keep      capture files kept in temp_data/captures

The recorder (recorder.py) reads them at its start and again within ~1 s of the file changing; the dashboard writes
them (`write`, one element, the file's comments kept — as utility/display_time.save_to_system_xml does for the time
zone). A missing element or attribute is its default; a value out of its bounds is refused by `write` and, read from
the file, replaced by its default with a warning.

The bounds keep memory and the SD card bounded: the recorder's history is the longer of before_s + after_s and
capture_before_s, + 3 s, of rows (at most 43 s, ~20 MB); `disk_estimate_mb` gives the worst case on the card for a set of values (a file is ~105 KB per second
of recording, measured 2026-10-16: 65 s -> 6.8 MB).
"""
from __future__ import annotations

import os
import re
import xml.etree.ElementTree as ET  # nosec B405 — our own config/system.xml, read as config/xml_parser reads it

from utility.incident_dump import (CAPTURE_KEEP, CAPTURE_MAX_S, INCIDENT_AFTER_S, INCIDENT_BEFORE_S, INCIDENT_KEEP,
                                   INCIDENT_MIN_INTERVAL_S)

DEFAULTS = {"before_s": float(INCIDENT_BEFORE_S), "after_s": float(INCIDENT_AFTER_S), "incident_keep": int(INCIDENT_KEEP),
            "min_interval_s": float(INCIDENT_MIN_INTERVAL_S), "capture_before_s": float(INCIDENT_BEFORE_S),
            "capture_keep": int(CAPTURE_KEEP)}
BOUNDS = {"before_s": (1.0, 30.0), "after_s": (0.5, 10.0), "incident_keep": (1, 50), "min_interval_s": (1.0, 600.0),
          "capture_before_s": (0.0, 30.0), "capture_keep": (1, 20)}
INTS = frozenset({"incident_keep", "capture_keep"})
LABELS = {"before_s": "seconds before a fault", "after_s": "seconds after a fault", "incident_keep": "incident files kept",
          "min_interval_s": "seconds between incident files (at least)",
          "capture_before_s": "seconds before the request", "capture_keep": "capture files kept"}
INCIDENT_KEYS = ("before_s", "after_s", "incident_keep", "min_interval_s")
CAPTURE_KEYS = ("capture_before_s", "capture_keep")
FILE_KB_PER_S = 105.0              # a recording's file size per second of it (65 s -> 6.8 MB on 2026-10-16)
HISTORY_EXTRA_S = 3.0              # the recorder keeps before_s + after_s + this


def _number(key, raw):
    v = float(raw)
    if v != v or v in (float("inf"), float("-inf")):
        raise ValueError("not a number")
    if key in INTS:
        if v != int(v):
            raise ValueError("not a whole number")
        return int(v)
    return v


def validate(values: dict) -> tuple[dict, list]:
    """(the settings, problems): every key from `values` that is known and in its bounds; any other known key its
    default. Unknown keys and bad values are problems ("before_s: 99 is outside 1-30")."""
    out, problems = dict(DEFAULTS), []
    for key, raw in (values or {}).items():
        if key not in DEFAULTS:
            problems.append(f"{key}: unknown setting")
            continue
        try:
            v = _number(key, raw)
        except (TypeError, ValueError) as e:
            problems.append(f"{key}: {raw!r} is {e}")
            continue
        lo, hi = BOUNDS[key]
        if not lo <= v <= hi:
            problems.append(f"{key}: {raw!r} is outside {lo:g}-{hi:g}")
            continue
        out[key] = v
    return out, problems


def parse_text(text: str) -> tuple[dict, list]:
    """The settings in a system.xml text, and the problems found (none when there is no <recorder> element)."""
    try:
        root = ET.fromstring(text)  # nosec B314 — our own config file, not data from the network
    except ET.ParseError as e:
        return dict(DEFAULTS), [f"system.xml does not parse: {e}"]
    elem = root.find("recorder")
    return validate(dict(elem.attrib)) if elem is not None else (dict(DEFAULTS), [])


def read(path) -> tuple[dict, list]:
    try:
        with open(path, encoding="utf-8") as fh:
            return parse_text(fh.read())
    except OSError as e:
        return dict(DEFAULTS), [f"cannot read {path}: {e}"]


def element(values: dict) -> str:
    def fmt(k):
        return str(int(values[k])) if k in INTS else f"{float(values[k]):g}"
    return "<recorder " + " ".join(f'{k}="{fmt(k)}"' for k in DEFAULTS) + "/>"


def write(path, values: dict) -> dict:
    """Write the settings into system.xml (all five; raises ValueError listing the problems and changes nothing when
    any value is unknown or out of bounds). Replaces the one <recorder .../> element, or adds it before </system>."""
    clean, problems = validate(values)
    if problems:
        raise ValueError("; ".join(problems))
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    new_elem = element(clean)
    found = re.findall(r"<recorder\b[^>]*/>", text)
    if len(found) > 1:
        raise ValueError(f"{path} has {len(found)} <recorder> elements (expected at most 1)")
    if found:
        new = text.replace(found[0], new_elem)
    else:
        if text.count("</system>") != 1:
            raise ValueError(f"{path} has no single </system> to add <recorder> before")
        new = text.replace("</system>", "    <!-- 2026-10-16: the recorder's windows and how many files it keeps "
                                         "(utility/recorder_settings.py; the Recordings tab writes it) -->\n"
                                         f"    {new_elem}\n</system>")
    parsed, problems = parse_text(new)
    if problems or parsed != clean:
        raise ValueError(f"the edited file would not read back as written: {problems or parsed}")
    tmp = f"{path}.tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        fh.write(new)
    os.replace(tmp, path)
    return clean


def history_s(values: dict) -> float:
    """The seconds of rows the recorder keeps for these settings (at least 10)."""
    longest = max(float(values["before_s"]) + float(values["after_s"]), float(values["capture_before_s"]))
    return max(10.0, longest + HISTORY_EXTRA_S)


def disk_estimate_mb(values: dict) -> float:
    """The worst case on the SD card: every incident file full, every capture at the longest length."""
    incidents = values["incident_keep"] * (values["before_s"] + values["after_s"])
    captures = values["capture_keep"] * (values["capture_before_s"] + CAPTURE_MAX_S)
    return (incidents + captures) * FILE_KB_PER_S / 1024.0
