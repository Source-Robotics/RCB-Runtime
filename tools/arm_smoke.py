#!/usr/bin/env python3
"""The real-arm smoke routine (2026-10-13): after a change to RTI (or anything it runs), ONE command runs a fixed, safe
set of api_tests on the arm and saves the results with the commit they ran on.

    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/arm_smoke.py          # asks before anything moves
        --yes          no question (you are at the arm anyway)
        --list         what it runs, then exit
        --only a,b     just these (names from --list)

Before: the arm ENABLED, homed, error-free, in IDLE or EXEC (the dashboard); the workspace clear, the E-stop in reach.
What moves: J1 by +1 deg and back, J1 5 deg stopped mid-move and back, J5 small moves + a refused limit + a jog
(api_tests 02 / 12 / 45). Everything else moves nothing (00 / 49 / 42 / 60 / 25).

It also tells you when the robot runs OLDER code than what is on disk: RTI's commit (its `[RTI] code:` line in
RTI.log, logged at every start since 2026-10-13) against HEAD, and tracked files changed after RTI started.

Results: temp_data/smoke/<UTC time>_<commit>/ (each test's output + summary.json) and one line per run in
temp_data/smoke/history.tsv. Exit 0 = all passed, 1 = a test failed, 2 = refused before running anything.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from project_paths import RTI_LOG, TEMP_DATA_DIR                          # noqa: E402
from utility.code_version import git_commit                               # noqa: E402

PY = os.path.join(ROOT, ".venv", "bin", "python")
OUT = os.path.join(str(TEMP_DATA_DIR), "smoke")
TIMEOUT_S = 240

# (name, script, extra args, what moves (None = nothing), the mode it must start in (None = IDLE or EXEC))
TESTS = [
    ("status",         "00_status.py",         [],                                        None, None),
    ("is_reachable",   "49_is_reachable.py",   [],                                        None, None),
    ("arbitration",    "42_arbitration.py",    [],                                        None, None),
    ("self_collision", "60_self_collision.py", ["--go"],                                  None, None),
    ("rti_link",       "25_rti_comm.py",       [],                                        None, "IDLE"),   # RTI only from IDLE
    ("joint_nudge",    "02_joint.py",          ["--go", "--assume-ready", "--speed", "10"], "J1 +1 deg and back", "EXEC"),
    ("stop",           "12_stop.py",           ["--go", "--assume-ready", "--speed", "10"],
     "J1 5 deg, stopped mid-move, back", "EXEC"),
    ("exec_abort",     "45_exec_abort.py",     ["--go"],
     "J5 small moves, a refused limit, a jog", None),
]


def _client():
    from robotclient.client import RobotClient
    r = RobotClient(sync=True)
    r.connect("127.0.0.1", 29000)
    return r


def _status(robot):
    s = robot.get_status()
    return s if isinstance(s, dict) else {}


def _ready_problems(s):
    p = []
    if s.get("state") != "ENABLED":
        p.append(f"state {s.get('state')!r} (enable it)")
    if s.get("homed") is not True:
        p.append("not homed")
    if s.get("err") or s.get("error_active"):
        p.append(f"an error is active ({s.get('err') or s.get('error_active')})")
    if s.get("mode") not in ("IDLE", "EXEC"):
        p.append(f"mode {s.get('mode')!r} (IDLE or EXEC needed)")
    return p


def _wait_control_free(robot, timeout=15.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if not robot.control_holder():
            return True
        time.sleep(0.5)
    return False


def _rti_code_line():
    """(UTC epoch, 'branch @ sha12') of RTI's last start, from RTI.log; (None, None) when not logged yet."""
    try:
        with open(RTI_LOG, errors="replace") as fh:
            lines = fh.readlines()[-20000:]
    except OSError:
        return None, None
    for ln in reversed(lines):
        if "[RTI] code:" in ln:
            m = re.search(r"\|\s*([0-9.]+)\s*\|.*\[RTI\] code:\s*(.+)$", ln)
            if m:
                return float(m.group(1)), m.group(2).strip()
    return None, None


def _changed_since(epoch):
    """Tracked .py / .xml files modified after `epoch` (the code RTI may not have loaded)."""
    try:
        files = subprocess.run(["git", "ls-files", "*.py", "*.xml"], cwd=ROOT, capture_output=True, text=True,
                               timeout=20).stdout.split()
    except (OSError, subprocess.SubprocessError):
        return []
    out = []
    for f in files:
        if f.startswith(("api_tests/", "tools/", "blockly/web/")):
            continue
        try:
            if os.path.getmtime(os.path.join(ROOT, f)) > epoch:
                out.append(f)
        except OSError:
            pass
    return out


def _dirty_files():
    try:
        return [ln[3:] for ln in subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=ROOT,
                                               capture_output=True, text=True, timeout=20).stdout.splitlines()]
    except (OSError, subprocess.SubprocessError):
        return []


def _first_failure_line(text):
    for ln in text.splitlines():
        if re.search(r"\bFAIL|REJECTED|TIMEOUT|ROBOT ERROR|Traceback|Error:", ln):
            return ln.strip()[:160]
    return ""


def main():
    ap = argparse.ArgumentParser(description="the real-arm smoke routine (api_tests on the arm)")
    ap.add_argument("--yes", action="store_true", help="do not ask before starting")
    ap.add_argument("--list", action="store_true", help="print the tests and exit")
    ap.add_argument("--only", default="", help="comma-separated names (see --list)")
    args = ap.parse_args()

    tests = TESTS
    if args.only:
        want = {w.strip() for w in args.only.split(",") if w.strip()}
        unknown = want - {t[0] for t in TESTS}
        if unknown:
            print(f"unknown test(s): {sorted(unknown)} — see --list")
            return 2
        tests = [t for t in TESTS if t[0] in want]
    if args.list:
        for name, script, extra, moves, _ in TESTS:
            print(f"  {name:15s} {'api_tests/' + script:32s} {' '.join(extra):34s} {moves or 'nothing moves'}")
        return 0

    branch, sha = git_commit(ROOT)
    dirty = _dirty_files()
    rti_t, rti_code = _rti_code_line()
    warnings = []
    if rti_code is None:
        warnings.append("RTI.log has no '[RTI] code:' line — RTI started before 2026-10-13's change; restart it")
    elif sha and not rti_code.endswith(sha[:12]):
        warnings.append(f"RTI runs {rti_code}, HEAD is {sha[:12]} — restart RTI (the service) to test this code")
    if rti_t is not None:
        newer = _changed_since(rti_t)
        if newer:
            warnings.append(f"{len(newer)} tracked file(s) changed after RTI started — a restart may be needed: "
                            + ", ".join(newer[:8]) + (" ..." if len(newer) > 8 else ""))

    print(f"code on disk: {branch} @ {(sha or '?')[:12]}" + (f" + {len(dirty)} uncommitted file(s)" if dirty else ""))
    print(f"RTI runs:     {rti_code or 'unknown'}")
    for w in warnings:
        print("WARNING: " + w)
    try:
        robot = _client()
    except Exception as e:                                          # noqa: BLE001 — any failure to connect
        print(f"REFUSED: no executor answering on 127.0.0.1:29000 ({e})")
        return 2
    s = _status(robot)
    problems = _ready_problems(s)
    if problems:
        print("REFUSED: the arm is not ready — " + "; ".join(problems) + ". Prepare it in the dashboard first.")
        return 2

    print("\nWill run, in this order:")
    for name, script, extra, moves, _ in tests:
        print(f"  {name:15s} {moves or 'nothing moves'}")
    if not args.yes:
        try:
            input("\nWorkspace clear, E-stop in reach? Enter to start, Ctrl-C to cancel: ")
        except (KeyboardInterrupt, EOFError):
            print("\ncancelled")
            return 2

    stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime())
    run_dir = os.path.join(OUT, f"{stamp}_{(sha or 'nogit')[:8]}")
    os.makedirs(run_dir, exist_ok=True)
    results = []
    aborted = None
    for name, script, extra, moves, mode in tests:
        s = _status(robot)
        problems = _ready_problems(s)
        if problems:
            aborted = f"before '{name}': " + "; ".join(problems)
            break
        if mode == "EXEC" and s.get("mode") != "EXEC":
            robot.enter_exec_mode(wait=True)
        elif mode == "IDLE" and s.get("mode") != "IDLE":
            robot.enter_idle_mode(wait=True)
        try:
            robot.release_control()
        except Exception:                                           # noqa: BLE001 — not holding it is fine
            pass
        _wait_control_free(robot)
        cmd = [PY, os.path.join("api_tests", script), "--host", "127.0.0.1"] + extra
        print(f"\n--- {name}: {' '.join(cmd[1:])}", flush=True)
        t0 = time.monotonic()
        try:
            p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=TIMEOUT_S,
                               env=dict(os.environ, PYTHONUNBUFFERED="1"))
            code, text = p.returncode, p.stdout + ("\n--- stderr ---\n" + p.stderr if p.stderr.strip() else "")
        except subprocess.TimeoutExpired as e:
            code, text = -1, (e.stdout or "") + f"\n--- TIMEOUT after {TIMEOUT_S} s ---"
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
        dt = time.monotonic() - t0
        with open(os.path.join(run_dir, f"{name}.log"), "w") as fh:
            fh.write(text)
        ok = code == 0
        note = "" if ok else _first_failure_line(text)
        results.append({"name": name, "script": script, "args": extra, "exit": code, "ok": ok,
                        "seconds": round(dt, 1), "note": note})
        print(f"    {'PASS' if ok else 'FAIL'}  {dt:6.1f} s" + (f"   exit {code}: {note}" if not ok else ""), flush=True)

    passed = sum(r["ok"] for r in results)
    ok_all = aborted is None and passed == len(tests)
    summary = {"utc": stamp, "branch": branch, "commit": sha, "uncommitted": dirty, "rti_code": rti_code,
               "warnings": warnings, "results": results, "aborted": aborted, "passed": passed, "total": len(tests),
               "final_status": {k: _status(robot).get(k) for k in ("mode", "state", "homed", "err")}}
    with open(os.path.join(run_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, indent=1)
    hist = os.path.join(OUT, "history.tsv")
    new = not os.path.exists(hist)
    with open(hist, "a") as fh:
        if new:
            fh.write("utc\tcommit\tuncommitted\trti_code\tpassed\ttotal\tfailed\twarnings\n")
        fh.write(f"{stamp}\t{(sha or '')[:12]}\t{len(dirty)}\t{rti_code or ''}\t{passed}\t{len(tests)}\t"
                 f"{','.join(r['name'] for r in results if not r['ok'])}{(' ABORTED ' + aborted) if aborted else ''}"
                 f"\t{len(warnings)}\n")

    print(f"\nSMOKE: {passed}/{len(tests)} passed" + (f" — ABORTED {aborted}" if aborted else "")
          + (f" — {len(warnings)} warning(s) above" if warnings else ""))
    print(f"results: {os.path.relpath(run_dir, ROOT)}  (history: {os.path.relpath(hist, ROOT)})")
    print("RESULT:", "ALL PASSED" if ok_all else "FAILED")
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
