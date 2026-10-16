"""Program run records — one JSON-lines file per block-program run (2026-10-15).

The dashboard's event list (blockly/runs.py) holds the CURRENT run only, in memory, and forgets it at the next
start; the block log (block_programs.log) has the runners' stderr. What was missing is a durable record per run,
the way a robot cell keeps a job history: which program ran, from which file (its hash), on which code, who started
it, every event in order, how it ended. `<LOGS_DIR>/block_runs/<UTC>_<program>.jsonl`:

    {"schema": 1, "type": "run_started", "time": ..., "time_utc": "...", "program": ..., "sha256": ..., "code": ...,
     "origin": ...}
    {"seq": 1, "t": ..., "type": "state", ...}        every event the runner reported, as runs._record saw it
    ...
    {"type": "run_finished", "time": ..., "exit": 0, "state": "done", "error": null, "blocks": 12, "duration_s": 4.2}

A record is capped at MAX_BYTES (a `truncated` line says so, the final line still comes); KEEP files are kept.
`history(limit)` reads the first and last line of each file — the Runs card.
"""
from __future__ import annotations

import hashlib
import json
import os
import time

from project_paths import LOGS_DIR, PROJECT_ROOT

SCHEMA = 1
KEEP = 200
MAX_BYTES = 4_000_000

_settings = {"dir": os.path.join(str(LOGS_DIR), "block_runs")}


def configure(record_dir=None):
    """Where the records go (the nets and the simulator give their own folder; the default follows LOGS_DIR)."""
    if record_dir is not None:
        _settings["dir"] = str(record_dir)


def record_dir() -> str:
    return _settings["dir"]


def _code_version():
    try:
        from utility.code_version import describe
        return describe(PROJECT_ROOT)
    except Exception:
        return "unknown"


def _sha256(path):
    try:
        with open(path, "rb") as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return None


def _safe(name):
    return "".join(ch if ch.isalnum() or ch in "-_" else "-" for ch in str(name))[:48] or "program"


def begin(program, path, origin="blocks", now=None) -> dict:
    """Open a record for a run that starts now. Returns the record handle (a dict); never raises — a record that
    cannot be opened returns a handle that ignores every write (the run must not fail for its record)."""
    now = time.time() if now is None else now
    stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime(now))
    rec = {"path": None, "bytes": 0, "events": 0, "blocks": 0, "started": now, "truncated": False, "program": program}
    try:
        os.makedirs(_settings["dir"], exist_ok=True)
        rec["path"] = os.path.join(_settings["dir"], f"{stamp}_{_safe(program)}.jsonl")
        _write(rec, {"schema": SCHEMA, "type": "run_started", "time": round(now, 3),
                     "time_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now)), "program": program,
                     "file": os.path.basename(str(path)) if path else None, "sha256": _sha256(path) if path else None,
                     "code": _code_version(), "origin": origin}, force=True)
        _prune()
    except OSError:
        rec["path"] = None
    return rec


def append(rec, event) -> None:
    """One event (runs._record's dict) into the record; past MAX_BYTES only a `truncated` marker, once."""
    if not rec or not rec.get("path"):
        return
    if event.get("type") == "block":
        rec["blocks"] += 1
    rec["events"] += 1
    if rec["bytes"] >= MAX_BYTES:
        if not rec["truncated"]:
            rec["truncated"] = True
            _write(rec, {"type": "truncated", "after_events": rec["events"] - 1, "time": round(time.time(), 3)}, force=True)
        return
    _write(rec, event)


def finish(rec, exit_code, state, error, now=None) -> None:
    """The final line: how the run ended."""
    if not rec or not rec.get("path"):
        return
    now = time.time() if now is None else now
    _write(rec, {"type": "run_finished", "time": round(now, 3), "exit": exit_code, "state": state, "error": error,
                 "blocks": rec["blocks"], "events": rec["events"], "duration_s": round(now - rec["started"], 3),
                 "truncated": rec["truncated"]}, force=True)


def _write(rec, obj, force=False):
    line = json.dumps(obj, separators=(",", ":")) + "\n"
    try:
        with open(rec["path"], "a", encoding="utf-8") as f:
            f.write(line)
        rec["bytes"] += len(line)
    except OSError:
        if force:
            rec["path"] = None


def _prune(keep=None):
    keep = KEEP if keep is None else int(keep)
    try:
        files = sorted(f for f in os.listdir(_settings["dir"]) if f.endswith(".jsonl"))
    except OSError:
        return
    for old in files[:-keep] if keep > 0 else []:
        try:
            os.unlink(os.path.join(_settings["dir"], old))
        except OSError:
            pass


def _first_last(path):
    first = last = None
    with open(path, "rb") as f:
        head = f.readline()
        if not head:
            return None, None
        try:
            first = json.loads(head)
        except ValueError:
            return None, None
        f.seek(0, os.SEEK_END)
        size = f.tell()
        f.seek(max(0, size - 4096))
        tail = f.read().splitlines()
        for raw in reversed(tail):
            try:
                last = json.loads(raw)
                break
            except ValueError:
                continue
    return first, last


def history(limit=50) -> list[dict]:
    """The newest `limit` runs: [{file, program, started, finished, result, exit, blocks, duration_s, error, origin,
    code}] newest first; `result` = "done" / "failed" / "stopped" / "refused" / "died" / "running" (no final line)."""
    try:
        files = sorted((f for f in os.listdir(_settings["dir"]) if f.endswith(".jsonl")), reverse=True)[:limit]
    except OSError:
        return []
    out = []
    for name in files:
        path = os.path.join(_settings["dir"], name)
        try:
            first, last = _first_last(path)
        except OSError:
            continue
        if not first or first.get("type") != "run_started":
            continue
        done = last if last and last.get("type") == "run_finished" else None
        out.append({"file": name, "program": first.get("program"), "started": first.get("time"),
                    "finished": done.get("time") if done else None, "result": _result(done),
                    "exit": done.get("exit") if done else None, "blocks": done.get("blocks") if done else None,
                    "duration_s": done.get("duration_s") if done else None, "error": done.get("error") if done else None,
                    "origin": first.get("origin"), "code": first.get("code")})
    return out


def _result(done):
    if done is None:
        return "running"
    code = done.get("exit")
    return {0: "done", 1: "failed", 2: "stopped", 3: "refused"}.get(code, "died")


def read(name) -> list[dict]:
    """Every line of one record (by file name), for a detailed view."""
    path = os.path.join(_settings["dir"], os.path.basename(name))
    out = []
    with open(path, encoding="utf-8") as f:
        for raw in f:
            try:
                out.append(json.loads(raw))
            except ValueError:
                continue
    return out
