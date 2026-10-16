"""Times shown to people, in the robot box's chosen time zone (2026-10-09).

User: "it is annoying that this shows UTC while I am in Europe/Zagreb — could users select it? And without internet?"
`<display_timezone>` in config/system.xml: an IANA name (Europe/Zagreb, UTC, ...) or "system" (the Pi's own zone). The
zone database is installed on the Pi (`zoneinfo`, /usr/share/zoneinfo — 599 zones), so choosing and converting needs no
internet; what internet gives is a CORRECT CLOCK (NTP) — without it the Pi runs on its real-time clock, which keeps time
through a power cut only with its coin battery fitted (`clock_status()` says which). Everything stored keeps UTC
(the logs' epoch timestamps, the homing table's "...utc" fields): unambiguous, and safe across the summer-time change
when one local hour happens twice. Only what is SHOWN is converted, here.

One setting per robot box: the dashboard is one shared page for every browser, so it cannot follow each viewer's zone
(the Blockly page, drawn in the browser, does show the browser's own time).
"""
from __future__ import annotations

import calendar
import math
import datetime as _dt
import os
import re
import subprocess
import time

_zone = None          # a zoneinfo.ZoneInfo, or None = the Pi's own zone
_name = "system"


def set_zone(name: str | None) -> str:
    """Use this zone for every time shown from now on ("system"/None/"" = the Pi's own). An unknown name raises
    ValueError and changes nothing. Returns the name in force."""
    global _zone, _name
    name = (name or "system").strip() or "system"
    if name == "system":
        _zone, _name = None, "system"
        return _name
    import zoneinfo
    try:
        z = zoneinfo.ZoneInfo(name)
    except (ValueError, zoneinfo.ZoneInfoNotFoundError) as exc:
        raise ValueError(f"unknown time zone {name!r}") from exc
    _zone, _name = z, name
    return _name


def zone_name() -> str:
    """The setting ("system" or an IANA name)."""
    return _name


def system_zone_name() -> str:
    """The Pi's own zone (what "system" means), e.g. 'Etc/UTC'."""
    try:
        return os.path.realpath("/etc/localtime").split("zoneinfo/", 1)[1]
    except IndexError:
        return time.tzname[0]


def all_zones() -> list[str]:
    import zoneinfo
    return sorted(zoneinfo.available_timezones())


def _dt_local(ts: float) -> _dt.datetime:
    return _dt.datetime.fromtimestamp(ts, _zone) if _zone is not None else _dt.datetime.fromtimestamp(ts).astimezone()


def fmt(ts: float, pattern: str = "%Y-%m-%d %H:%M:%S") -> str:
    """An epoch time (as the logs store it) in the chosen zone. `%Z` gives the zone's abbreviation (CEST, UTC)."""
    return _dt_local(float(ts)).strftime(pattern)


# THE way the dashboard shows an event's time (2026-10-15, user: "be consistent"): the Errors, Activity Log and Logs
# tabs all use it. Date and time in the chosen zone, to the microsecond — the resolution the files keep (the text
# logs' ISO stamps, the activity log's and the error list's epoch floats).
PRECISE_PATTERN = "%Y-%m-%d %H:%M:%S"


def fmt_precise(ts: float) -> str:
    """"2026-10-15 19:19:10.292626" in the chosen zone."""
    ts = float(ts)
    whole = math.floor(ts)                 # the second and its fraction apart: datetime ROUNDS a fraction (1.9999996
    micro = min(int((ts - whole) * 1_000_000), 999_999)   # -> 2.0), which put the seconds one ahead of it
    return f"{fmt(whole, PRECISE_PATTERN)}.{micro:06d}"   # truncated, as the log files' formatter does


def utc_text_to_ts(text: str) -> float:
    """'YYYY-MM-DD HH:MM:SS' written in UTC (the homing table's "...utc" fields) -> epoch seconds."""
    return float(calendar.timegm(time.strptime(text.strip(), "%Y-%m-%d %H:%M:%S")))


def fmt_utc_text(text: str | None, pattern: str = "%Y-%m-%d %H:%M:%S") -> str:
    """A stored UTC text shown in the chosen zone ('' stays '', something unreadable is shown as it is)."""
    if not text:
        return ""
    try:
        return fmt(utc_text_to_ts(text), pattern)
    except ValueError:
        return text


def parse_local(text: str) -> float:
    """A time a PERSON typed ('YYYY-MM-DD[ HH:MM[:SS]]'), read in the chosen zone -> epoch seconds (the tools' --since)."""
    for pattern in ("%Y-%m-%d %H:%M:%S", "%Y-%m-%d %H:%M", "%Y-%m-%d"):
        try:
            naive = _dt.datetime.strptime(text.strip(), pattern)
            break
        except ValueError:
            continue
    else:
        raise ValueError(f"cannot read the time {text!r} (YYYY-MM-DD [HH:MM[:SS]])")
    return (naive.replace(tzinfo=_zone) if _zone is not None else naive.astimezone()).timestamp()


def clock_status() -> dict:
    """{"synced": True/False/None (NTP; None = unknown), "rtc": bool (a real-time clock device exists)} — slow (a
    subprocess): call it now and then, not per frame."""
    synced = None
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], capture_output=True, text=True,
                             timeout=2).stdout.strip()
        synced = {"yes": True, "no": False}.get(out)
    except (OSError, subprocess.SubprocessError):
        pass
    return {"synced": synced, "rtc": os.path.exists("/sys/class/rtc/rtc0")}


def save_to_system_xml(path, name: str) -> None:
    """Write `<display_timezone>` into config/system.xml, changing that one element only (its comments stay)."""
    name = set_zone(name)                               # validates (raises ValueError) and applies
    with open(path) as fh:
        text = fh.read()
    new, n = re.subn(r"<display_timezone>[^<]*</display_timezone>", f"<display_timezone>{name}</display_timezone>", text)
    if n != 1:
        raise ValueError(f"{path} has {n} <display_timezone> elements (expected 1)")
    tmp = f"{path}.tmp"
    with open(tmp, "w") as fh:
        fh.write(new)
    os.replace(tmp, path)
