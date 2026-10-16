"""The recorder's logic: incident dumps and captures from RTI's shared-memory trace (2026-10-16).

recorder.py (its own process, normal priority on the general cores) runs this; RTI only writes the trace
(utility/trace_ring.py). Everything here works on rows of numbers — the trace's rows are already arrays, so writing a
file is slicing columns, not walking per-tick Python objects. Pure: the process feeds it rows, the error list and the
capture request; it writes the files (utility/incident_dump.save) and logs.

  * History: the newest `history_s` (10 s) of rows, in the recorder's own memory.
  * Incidents: a key new in the error list with level "error" (a HARD error; warnings never dump) -> after
    INCIDENT_AFTER_S of rows has arrived, the rows from INCIDENT_BEFORE_S before the fault's tick to INCIDENT_AFTER_S
    after it -> temp_data/incidents/. At most one per INCIDENT_MIN_INTERVAL_S (the rest counted in `dropped`); none
    with fewer than INCIDENT_MIN_HISTORY rows before the fault (`too_early`). Errors already latched when the recorder
    starts are not dumped. If the rows stop coming (RTI stopped), it is written with what there is after
    INCIDENT_AFTER_S + 3 s.
  * Captures: a request (the executor writes it to shared memory: seq, seconds, name, request_tick, id, origin) ->
    the rows from capture_before_s (its own setting since 2026-10-16) before request_tick to `seconds` after it ->
    temp_data/captures/. One at a time
    (CAPTURE_REFUSED otherwise); rows lost between two polls are counted in "gaps" (CAPTURE_SAVED is then a warning).
    The request present when the recorder starts is not replayed. A request with "action": "stop" ends the running
    capture early and saves what it has (`stop_capture`); with none running it is refused.
  * Settings (2026-10-16, utility/recorder_settings.py — system.xml <recorder>, the Recordings tab): the windows, the
    files kept and the incidents' rate limit, applied live by `apply_settings`; the history follows the windows.
  * `status()`: what the dashboard shows — the settings, the running capture's progress, the counters.
"""
from __future__ import annotations

import logging
import math
import time

import numpy as np

from utility.incident_dump import (CAPTURE_KEEP, CAPTURE_MAX_S, CAPTURE_MIN_S, INCIDENT_AFTER_S, INCIDENT_BEFORE_S,
                                   INCIDENT_KEEP, INCIDENT_MIN_HISTORY, INCIDENT_MIN_INTERVAL_S, prune, safe_capture_name,
                                   save)
from utility.recorder_settings import HISTORY_EXTRA_S

logger = logging.getLogger("rec")  # docs/logging.md "Logger names"
STALL_EXTRA_S = 3.0          # an incident's after-window waits this much longer for rows before it is written anyway


class Recorder:
    def __init__(self, layout, incidents_dir, captures_dir, activity=None, history_s=10.0,
                 min_interval_s=INCIDENT_MIN_INTERVAL_S, min_history=INCIDENT_MIN_HISTORY, before_s=INCIDENT_BEFORE_S,
                 after_s=INCIDENT_AFTER_S, keep=INCIDENT_KEEP, capture_keep=CAPTURE_KEEP, capture_before_s=None):
        self.layout = layout
        self.width = int(layout["width"])
        self.tick_s = float(layout["tick_s"])
        self.cols = {name: col for name, col in layout["scalars"]}
        self.tick_col = self.cols["tick"]
        self.modes = list(layout.get("modes", []))
        self.incidents_dir, self.captures_dir = incidents_dir, captures_dir
        self.activity = activity            # (level, code, message, id, origin) -> None, or None
        self.min_history = int(min_history)
        self._min_history_s = float(history_s)
        self._cap_rows = 0
        self._buf = np.zeros((0, self.width))
        self._n = 0                          # rows ever stored (the buffer holds the newest _cap_rows)
        self.apply_settings({"before_s": before_s, "after_s": after_s, "incident_keep": keep,
                             "capture_keep": capture_keep, "min_interval_s": min_interval_s,
                             "capture_before_s": before_s if capture_before_s is None else capture_before_s})
        self.incidents_total = self.captures_total = 0
        self._known = None                   # the hard error keys latched at the last look (None: not looked yet)
        self._last_incident = -math.inf
        self.pending: list = []
        self.dropped = self.too_early = 0
        self.written: list = []              # incident paths written (newest last; the newest `keep` remembered)
        self._last_seq = None
        self.capture = None                  # the running capture, or None
        self.captured: list = []

    def apply_settings(self, values) -> None:
        """The windows, the files kept and the rate limit (utility/recorder_settings keys). The history grows or
        shrinks to before + after + 3 s (at least `history_s`), keeping its newest rows; a running capture keeps its
        own window, a pending incident is written with the new after-window. Fewer files kept: the oldest go now."""
        self.settings = {k: values[k] for k in ("before_s", "after_s", "incident_keep", "min_interval_s",
                                                "capture_before_s", "capture_keep")}
        self.before = int(round(float(values["before_s"]) / self.tick_s))
        self.capture_before = int(round(float(values["capture_before_s"]) / self.tick_s))
        self.after = int(round(float(values["after_s"]) / self.tick_s))
        self.after_s = float(values["after_s"])
        self.min_interval_s = float(values["min_interval_s"])
        self.keep, self.capture_keep = int(values["incident_keep"]), int(values["capture_keep"])
        longest = max(float(values["before_s"]) + float(values["after_s"]), float(values["capture_before_s"]))
        want_s = max(self._min_history_s, longest + HISTORY_EXTRA_S)
        rows = max(int(round(want_s / self.tick_s)), self.before + self.after + 1, self.capture_before + 1)
        if rows != self._cap_rows:
            keep = self.history()[-rows:]
            self._buf = np.zeros((rows, self.width))
            self._buf[:len(keep)] = keep
            self._cap_rows, self._n = rows, len(keep)
        prune(self.incidents_dir, self.keep)
        prune(self.captures_dir, self.capture_keep)

    def status(self) -> dict:
        """What the dashboard shows: the settings in force, the running capture (its progress), the counters."""
        c = self.capture
        cap = None
        if c is not None:
            span = max(1, c["end_tick"] - c["request_tick"])
            done = max(0, c["last"] - c["request_tick"])
            cap = {"name": c["name"], "seconds": c["seconds"], "done_s": round(done * self.tick_s, 2),
                   "fraction": round(min(1.0, done / span), 3), "gaps": c["gaps"], "started": c["wall"],
                   "id": c["id"], "origin": c["origin"]}
        return {"settings": dict(self.settings), "history_s": round(self._cap_rows * self.tick_s, 2), "capture": cap,
                "pending_incidents": len(self.pending), "incidents_written": self.incidents_total,
                "dropped": self.dropped, "too_early": self.too_early, "captures_written": self.captures_total,
                "last_incident": self.written[-1].name if self.written else None,
                "last_capture": self.captured[-1].name if self.captured else None}

    # ------------------------------------------------------------------ rows
    def history(self) -> np.ndarray:
        """The stored rows, oldest first (a copy)."""
        n = min(self._n, self._cap_rows)
        if self._n <= self._cap_rows:
            return self._buf[:n].copy()
        k = self._n % self._cap_rows
        return np.concatenate((self._buf[k:], self._buf[:k]))

    def newest_tick(self):
        if not self._n:
            return None
        return int(self._buf[(self._n - 1) % self._cap_rows, self.tick_col])

    def feed(self, block, lost=0, now=None) -> None:
        """New rows from the trace (oldest first) and how many were lost before them."""
        now = time.monotonic() if now is None else now
        if lost and self.capture is not None:
            self.capture["gaps"] += 1
        for row in block:
            self._buf[self._n % self._cap_rows] = row
            self._n += 1
        if self.capture is not None and len(block):
            self._capture_take(block)
        newest = self.newest_tick()
        for p in list(self.pending):
            if (newest is not None and newest >= p["tick"] + self.after) or now - p["mono"] > self.after_s + STALL_EXTRA_S:
                self.pending.remove(p)
                self._write_incident(p)

    # ------------------------------------------------------------------ incidents
    def on_errors(self, entries, now=None, wall=None) -> list:
        """The error list (shared memory's error_state JSON). Returns the new hard keys it saw."""
        now = time.monotonic() if now is None else now
        wall = time.time() if wall is None else wall
        hard = {e["key"]: e for e in entries if isinstance(e, dict) and "key" in e and e.get("level", "error") != "warning"} \
            if isinstance(entries, list) else {}
        if self._known is None:
            self._known = set(hard)
            return []
        new = sorted(k for k in hard if k not in self._known)
        self._known = set(hard)
        if not new:
            return []
        ticks = [int(hard[k]["tick"]) for k in new if hard[k].get("tick") is not None]
        tick = min(ticks) if ticks else self.newest_tick()
        if tick is None:
            return new
        if now - self._last_incident < self.min_interval_s:
            self.dropped += 1
            return new
        h = self.history()
        before = int((h[:, self.tick_col] <= tick).sum()) if len(h) else 0
        if before < self.min_history:
            self.too_early += 1
            logger.info(f"{', '.join(new)} at tick {tick}: not saved — only {before} tick(s) of history "
                        f"(< {self.min_history})")
            return new
        self._last_incident = now
        self.pending.append({"keys": new, "tick": tick, "wall": wall, "mono": now})
        return new

    def _write_incident(self, p) -> None:
        h = self.history()
        t = h[:, self.tick_col]
        rows = h[(t > p["tick"] - self.before) & (t <= p["tick"] + self.after)]
        before = int((rows[:, self.tick_col] <= p["tick"]).sum())
        path = save(self.incidents_dir, p["wall"], p["tick"], p["keys"], self.channels(rows), len(rows), "incident",
                    self.keep, {"before_ticks": before, "after_ticks": len(rows) - before, "after_s": self.after_s})
        self.written = (self.written + [path])[-self.keep:]
        self.incidents_total += 1
        logger.info(f"incident {', '.join(p['keys'])} at tick {p['tick']}: {before} ticks before it, "
                    f"{len(rows) - before} after -> {path.name}")

    # ------------------------------------------------------------------ captures
    def on_request(self, req, now=None, wall=None) -> None:
        """The capture request (shared memory's capture_request JSON: {seq, seconds, name, request_tick, id, origin})."""
        wall = time.time() if wall is None else wall
        if not isinstance(req, dict) or "seq" not in req:
            if self._last_seq is None:
                self._last_seq = ""
            return
        if self._last_seq is None:          # the request present at the recorder's start: never replayed
            self._last_seq = req["seq"]
        if req["seq"] == self._last_seq:
            return
        self._last_seq = req["seq"]
        cid, origin = req.get("id"), req.get("origin")
        if req.get("action") == "stop":
            if self.capture is None:
                self._activity("warning", "CAPTURE_REFUSED", "stop: no capture is running", cid, origin)
                return
            self.capture["stopped"] = True
            self._finish_capture()
            return
        name = safe_capture_name(req.get("name"))
        try:
            seconds = float(req["seconds"])
        except (KeyError, TypeError, ValueError):
            seconds = float("nan")
        if self.capture is not None:
            self._activity("warning", "CAPTURE_REFUSED", f"{name}: a capture is already running", cid, origin)
            return
        if not CAPTURE_MIN_S <= seconds <= CAPTURE_MAX_S:
            self._activity("warning", "CAPTURE_REFUSED", f"{name}: the length must be {CAPTURE_MIN_S:g} to "
                           f"{CAPTURE_MAX_S:g} s, not {req.get('seconds')!r}", cid, origin)
            return
        rq = req.get("request_tick")
        rq = int(rq) if rq is not None else self.newest_tick()
        if rq is None:
            self._activity("warning", "CAPTURE_REFUSED", f"{name}: nothing to capture (RTI writes no trace)", cid, origin)
            return
        end = rq + int(round(seconds / self.tick_s))
        self.capture = {"name": name, "seconds": seconds, "request_tick": rq, "end_tick": end, "blocks": [],
                        "last": rq - self.capture_before, "gaps": 0, "id": cid, "origin": origin, "wall": wall,
                        "stopped": False}
        self._capture_take(self.history())
        self._activity("info", "CAPTURE_STARTED", f"{name}: {seconds:g} s", cid, origin)

    def _capture_take(self, block) -> None:
        c = self.capture
        t = block[:, self.tick_col]
        part = block[(t > c["last"]) & (t <= c["end_tick"])]
        if len(part):
            c["blocks"].append(part.copy())
            c["last"] = int(part[-1, self.tick_col])
        if c["last"] >= c["end_tick"]:
            self._finish_capture()

    def capture_overdue(self, now_wall=None) -> None:
        """Finish a capture whose rows stopped coming (RTI stopped) STALL_EXTRA_S after its end."""
        c = self.capture
        if c is None:
            return
        now_wall = time.time() if now_wall is None else now_wall
        if now_wall - c["wall"] > c["seconds"] + STALL_EXTRA_S:
            c["gaps"] += 1
            self._finish_capture()

    def _finish_capture(self) -> None:
        c, self.capture = self.capture, None
        rows = np.concatenate(c["blocks"]) if c["blocks"] else np.empty((0, self.width))
        if not len(rows):
            self._activity("warning", "CAPTURE_REFUSED", f"{c['name']}: nothing captured ({c['seconds']:g} s)",
                           c["id"], c["origin"])
            return
        before = int((rows[:, self.tick_col] <= c["request_tick"]).sum())
        path = save(self.captures_dir, c["wall"], int(rows[-1, self.tick_col]), (c["name"],), self.channels(rows),
                    len(rows), "capture", self.capture_keep,
                    {"seconds": c["seconds"], "request_tick": c["request_tick"], "gaps": c["gaps"],
                     "before_ticks": before, "after_ticks": len(rows) - before})
        self.captured = (self.captured + [path])[-self.capture_keep:]
        self.captures_total += 1
        lost = f"; {c['gaps']} gap(s): ticks were lost" if c["gaps"] else ""
        early = f"; stopped early after {(len(rows) - before) * self.tick_s:.1f} s" if c.get("stopped") else ""
        self._activity("warning" if c["gaps"] else "info", "CAPTURE_SAVED",
                       f"{c['name']}: {len(rows)} ticks ({before} before the request, {len(rows) - before} after; "
                       f"{c['seconds']:g} s asked{early}{lost}) in {path.name}", c["id"], c["origin"])

    # ------------------------------------------------------------------ files
    def channels(self, rows) -> dict:
        """{channel: array}: each traced array as (ticks, n), each scalar as (ticks,), the mode as strings."""
        out = {}
        for name, start, size in self.layout["arrays"]:
            out[name] = rows[:, start:start + size].copy()
        for name, col in self.layout["scalars"]:
            if name == "mode":
                codes = rows[:, col].astype(int)
                out[name] = np.array([self.modes[c] if 0 <= c < len(self.modes) else "" for c in codes])
            else:
                out[name] = rows[:, col].copy()
        return out

    def _activity(self, level, code, message, cid=None, origin=None) -> None:
        logger.log(logging.WARNING if level == "warning" else logging.INFO, f"{code}: {message}")
        if self.activity is not None:
            self.activity(level, code, message, cid, origin)
