"""The camera server's settings (2026-11-23, TODO/camera_plan.md P1): config/system.xml `<camera .../>`, changed from
the dashboard's Camera tab.

    <camera enabled="1" device="auto" width="1280" height="720" fps="30" preview_fps="15"
            controls="exposure_dynamic_framerate=0"/>

  enabled       1 = stream when a camera is found (the user's D3: on whenever present), 0 = the camera is left closed
  device        "auto" = the camera the ACTIVE gripper's XML declares (grippers/<gripper>.xml
                <camera usb_id="vvvv:pppp"/>; a gripper without one = no camera); "usb:vvvv:pppp" = the first camera
                with that USB id; a path (/dev/v4l/by-id/... or /dev/videoN) = that node
  width, height, fps
                the MJPEG mode asked for; the camera takes the nearest it has (the Camera tab lists them)
  preview_fps   the dashboard preview's rate (the user's D4: 15, "maybe even less if it will hinder performance");
                the raw stream always carries every frame
  controls      the camera's own controls by key, "key=value;key=value" — keys as hardware/uvc_camera.control_key
                makes them (auto_exposure, exposure_time_absolute, brightness, ...). Applied at every open
                and whenever they change here, the automatic modes first. The default turns the camera's dynamic
                frame rate OFF: it dropped to 16.6 fps in the dark (2026-11-23), which a recording must not do
                silently.

The camera server reads them at its start and again within ~1 s of the file changing (its mtime), like the recorder
(utility/recorder_settings.py). A missing element or attribute is its default; a value out of its bounds is refused by
`write` and, read from the file, replaced by its default with a warning.
"""
from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET  # nosec B405 — our own config/system.xml, read as config/xml_parser reads it

from utility.config_write import edit_lock, write_atomic

DEFAULTS = {"enabled": 1, "device": "auto", "width": 1280, "height": 720, "fps": 30.0, "preview_fps": 15.0,
            "controls": "exposure_dynamic_framerate=0"}
BOUNDS = {"enabled": (0, 1), "width": (16, 4096), "height": (16, 4096), "fps": (1.0, 240.0),
          "preview_fps": (1.0, 30.0)}
INTS = frozenset({"enabled", "width", "height"})
LABELS = {"enabled": "camera on", "device": "which camera", "width": "width (px)", "height": "height (px)",
          "fps": "frames per second", "preview_fps": "dashboard preview (frames per second)",
          "controls": "the camera's controls"}
_KEY = re.compile(r"^[a-z0-9_]+$")
_DEVICE = re.compile(r"^(auto|usb:[0-9a-f]{4}:[0-9a-f]{4}|/dev/[A-Za-z0-9_./:-]+)$")


def parse_controls(text: str) -> list[tuple[str, int]]:
    """"a=1;b=-2" -> [("a", 1), ("b", -2)], in the written order. Raises ValueError for a malformed entry or a key
    given twice."""
    out, seen = [], set()
    for part in (text or "").replace(",", ";").split(";"):
        part = part.strip()
        if not part:
            continue
        key, sep, raw = part.partition("=")
        key, raw = key.strip(), raw.strip()
        if not sep or not _KEY.match(key):
            raise ValueError(f"{part!r} is not key=value")
        if key in seen:
            raise ValueError(f"{key} is given twice")
        try:
            value = int(raw)
        except ValueError:
            raise ValueError(f"{key}: {raw!r} is not a whole number") from None
        seen.add(key)
        out.append((key, value))
    return out


def format_controls(pairs) -> str:
    return ";".join(f"{k}={int(v)}" for k, v in pairs)


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
    """(the settings, problems): every key from `values` that is known and valid; any other known key its default.
    Unknown keys and bad values are problems ("fps: 999 is outside 1-240")."""
    out, problems = dict(DEFAULTS), []
    for key, raw in (values or {}).items():
        if key not in DEFAULTS:
            problems.append(f"{key}: unknown setting")
            continue
        if key == "device":
            v = str(raw).strip()
            if not _DEVICE.match(v):
                problems.append(f"device: {raw!r} is not auto, usb:vvvv:pppp or a /dev path")
                continue
            out[key] = v
            continue
        if key == "controls":
            try:
                out[key] = format_controls(parse_controls(str(raw)))
            except ValueError as e:
                problems.append(f"controls: {e}")
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
    """The settings in a system.xml text, and the problems found (none when there is no <camera> element)."""
    try:
        root = ET.fromstring(text)  # nosec B314 — our own config file, not data from the network
    except ET.ParseError as e:
        return dict(DEFAULTS), [f"system.xml does not parse: {e}"]
    elem = root.find("camera")
    return validate(dict(elem.attrib)) if elem is not None else (dict(DEFAULTS), [])


def read(path) -> tuple[dict, list]:
    try:
        with open(path, encoding="utf-8") as fh:
            return parse_text(fh.read())
    except OSError as e:
        return dict(DEFAULTS), [f"cannot read {path}: {e}"]


def element(values: dict) -> str:
    def fmt(k):
        if k in ("device", "controls"):
            return str(values[k])
        return str(int(values[k])) if k in INTS else f"{float(values[k]):g}"
    return "<camera " + " ".join(f'{k}="{fmt(k)}"' for k in DEFAULTS) + "/>"


def _elements(text: str) -> list[tuple[int, int]]:
    """(start, end) of every <camera .../> element OUTSIDE comments (a comment may quote one: system.xml's and the
    gripper XMLs' explanations do — the first net run of 2026-11-24 caught the writer counting a comment's)."""
    comments = [m.span() for m in re.finditer(r"<!--.*?-->", text, re.S)]
    return [m.span() for m in re.finditer(r"<camera\b[^>]*/>", text)
            if not any(a <= m.start() < b for a, b in comments)]


def write(path, values: dict) -> dict:
    """Write the settings into system.xml (raises ValueError listing the problems and changes nothing when any value
    is unknown or invalid). Replaces the one <camera .../> element, or adds it before </system>."""
    clean, problems = validate(values)
    if problems:
        raise ValueError("; ".join(problems))
    with edit_lock(path):                           # one edit of system.xml at a time across processes
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
        new_elem = element(clean)
        found = _elements(text)
        if len(found) > 1:
            raise ValueError(f"{path} has {len(found)} <camera> elements (expected at most 1)")
        if found:
            start, end = found[0]
            new = text[:start] + new_elem + text[end:]
        else:
            if text.count("</system>") != 1:
                raise ValueError(f"{path} has no single </system> to add <camera> before")
            new = text.replace("</system>", "    <!-- the camera server's settings (utility/camera_settings.py; the "
                                             "Camera tab writes it) -->\n"
                                             f"    {new_elem}\n</system>")
        parsed, problems = parse_text(new)
        if problems or parsed != clean:
            raise ValueError(f"the edited file would not read back as written: {problems or parsed}")
        write_atomic(path, new)
    return clean


def gripper_camera(system_xml, grippers_dir) -> tuple[dict | None, str]:
    """(the active gripper's <camera> attributes or None, the gripper's name) — read straight from the files (no
    robot model). A gripper file that is missing or unreadable reads as no camera."""
    try:
        root = ET.parse(str(system_xml)).getroot()  # nosec B314 — our own config file
    except (OSError, ET.ParseError):
        return None, ""
    name = (root.findtext("active_gripper") or "").strip()
    if name.lower() in ("", "none", "unknown"):
        return None, name
    try:
        groot = ET.parse(f"{grippers_dir}/{name}.xml").getroot()  # nosec B314 — our own gripper file
    except (OSError, ET.ParseError):
        return None, name
    elem = groot.find("camera")
    return (dict(elem.attrib) if elem is not None else None), name


# ---------------------------------------------------------------- the timing calibration (P3, 2026-11-24)
# api_tests/80_camera_timing.py measures, per camera mode, how far the image's moment (the middle of the exposure of
# the image's centre, as the joint data sees it) lies from the camera's own frame time (its PTS on the robot's clock),
# and writes the result as attributes of the gripper XML's <camera> element:
#   timing_mode="1280x720@30"   the mode it was measured in (another mode: not calibrated)
#   timing_auto_us="..."        the offset under AUTOMATIC exposure. Valid while the camera exposes for the whole frame,
#                               which it does indoors: in fixed-exposure mode this camera drops its gain, so a fixed
#                               33 ms gives a black picture under the lab's lamps (0.8 / 255; automatic at the same
#                               30 fps: 137, measured 2026-11-24) — the automatic mode is at its longest exposure plus
#                               gain there. In bright light it may shorten the exposure: up to half a frame off then.
#   timing_offset_us="..."      the offset at a FIXED exposure of setting 0 ...
#   timing_us_per_unit="..."    ... and its change per unit of exposure_time_absolute (100 us per unit, the UVC
#                               standard, confirmed 2026-11-24 by the frame rate: 1000 -> 9.92 fps, 4000 -> 2.49 fps):
#                               +50 when the PTS marks the exposure's start, -50 when its end. Only measured where
#                               fixed exposures are bright enough at the full frame rate. A fixed exposure is
#                               calibrated only with BOTH numbers (one alone: not calibrated, 2026-11-25).
#   timing_readout_us="..."     the rolling shutter: the time the camera takes to read its rows out, top to bottom
#                               (30.8 ms for 720 rows, measured 2026-11-24); the offsets above are the image's CENTRE
#                               row's, a row's own moment is (row - rows / 2) x readout / rows from it
#   timing_run="..."            the run it came from (temp_data/camera_timing/<run>)
TIMING_KEYS = ("timing_mode", "timing_auto_us", "timing_offset_us", "timing_us_per_unit", "timing_readout_us",
               "timing_run")
EXPOSURE_UNIT_US = 100          # exposure_time_absolute's unit (UVC; confirmed on the MSG camera 2026-11-24)


def mode_key(width, height, fps) -> str:
    """"1280x720@30": a camera mode as the timing calibration names it."""
    return f"{int(width)}x{int(height)}@{float(fps):g}"


def _timing_number(attrs: dict, key: str) -> float | None:
    try:
        v = float(attrs[key])
    except (KeyError, ValueError):
        return None
    return v if math.isfinite(v) else None


def camera_timing(gripper_cam: dict | None, mode: str) -> dict | None:
    """The timing calibration of the gripper's camera for `mode` (mode_key), or None: {"auto_us" (or None),
    "offset_us" (or None), "us_per_unit" (or None), "readout_us" (or None), "run"}."""
    if not gripper_cam or gripper_cam.get("timing_mode") != mode:
        return None
    out = {"auto_us": _timing_number(gripper_cam, "timing_auto_us"),
           "offset_us": _timing_number(gripper_cam, "timing_offset_us"),
           "us_per_unit": _timing_number(gripper_cam, "timing_us_per_unit"),
           "readout_us": _timing_number(gripper_cam, "timing_readout_us"),
           "run": gripper_cam.get("timing_run", "")}
    return out if out["auto_us"] is not None or out["offset_us"] is not None else None


def timing_offset_s(timing: dict | None, exposure_setting) -> float | None:
    """Seconds to add to a frame's camera time to reach its image's moment: at a fixed exposure `exposure_setting`
    (exposure_time_absolute) from the fixed-exposure line, under automatic exposure (None) the automatic offset; None
    when that one is not calibrated. The fixed-exposure line needs both its numbers: a missing change per unit used
    to count as 0 and the frame was still flagged calibrated (review 2026-11-25)."""
    if timing is None:
        return None
    if exposure_setting is None:
        return None if timing["auto_us"] is None else timing["auto_us"] * 1e-6
    if timing["offset_us"] is None or timing.get("us_per_unit") is None:
        return None
    return (timing["offset_us"] + timing["us_per_unit"] * float(exposure_setting)) * 1e-6
