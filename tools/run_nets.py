#!/usr/bin/env python3
"""Run every offline test net in dummy_test_scripts/ with ONE command, and say pass or fail.

    .venv/bin/python tools/run_nets.py                 # everything (refuses while the robot is ENABLED)
    .venv/bin/python tools/run_nets.py --only haptic   # nets whose file name contains "haptic"
    .venv/bin/python tools/run_nets.py --list          # what would run, and which source modules no net imports
    .venv/bin/python tools/run_nets.py --update-ruff-baseline   # accept today's ruff findings as the baseline

WHY (2026-10-02, TODO/refactor_plan.md Phase 0): 58 nets existed and nothing ran them together, so "nothing
breaks" could not be shown. This runner is the proof every refactor step needs: the same command, before and after.

RULES IT ENFORCES
  1. It refuses to run while the robot is ENABLED (asked over the executor's TCP port). Nets load the CPU, and on
     this Pi memory-bandwidth contention makes RTI's ticks slow (measured 2026-10-01). --allow-enabled overrides.
  2. Every net runs with RCB_SHM_PREFIX set to a unique value, so a net that calls shared_data.setup() creates its
     OWN segments and can never attach to the running robot's memory (test_kt_source.py used to write into it).
     The segments are unlinked after each net.
  3. Each net runs at nice 19 / idle I/O priority, serially, with a timeout, its output captured to
     temp_data/nets/<net>.log. A net FAILS on a non-zero exit code, a timeout, or a failure marker in its output
     (the nets are not uniform: some sys.exit, some assert, some only print "N FAILED").
  4. ruff (F-rules + syntax) runs over the source; any finding not in temp_data/ruff_baseline.txt fails. The
     baseline is "what the tree had on 2026-10-02" — the rule is NO NEW findings, not zero.
  5. The summary names the source modules no net imports (static scan) — refactoring one of those is riskier.

Exit code 0 only when every net and the ruff check pass. The summary is also written to temp_data/nets/last_run.txt.
"""
import argparse
import ast
import glob
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NETS_DIR = os.path.join(ROOT, "dummy_test_scripts")
OUT_DIR = os.path.join(ROOT, "temp_data", "nets")
RUFF_BASELINE = os.path.join(ROOT, "temp_data", "ruff_baseline.txt")
SOURCE = ["RTI.py", "command_executor.py", "supervisor.py", "nicegui_test.py", "gui_command_sender.py",
          "gui_walls_tab.py", "project_paths.py", "communication", "config", "data", "hardware", "motion",
          "robotics", "robots", "utility", "supervisor", "robotclient"]
SOURCE_PKGS = ("communication", "config", "data", "hardware", "motion", "robotics", "robots", "utility",
               "supervisor", "robotclient")
# A net passed when its exit code is 0 AND no line STARTS with one of these (anchored: a PASS line whose text
# mentions "FAILED" must not count). Tracebacks in the output are NOT a failure signal: a crash gives a non-zero
# exit anyway, while several nets print tracebacks on purpose (a handler raising "boom" to test the intake guard)
# or at exit (CPython's resource_tracker complaining about a segment the net unlinked itself).
FAIL_MARKERS = re.compile(r"(?m)^\s*(?:(?!0 )\d+ (?:failed|FAILED|FAILURE)|FAILED\b|FAILURES:|\[FAIL\]|FAIL\s{2,}|"
                          r"PLAN ERROR|RESULT: (?!ALL)\S*FAIL)")


def robot_state(timeout=2.0):
    """'ENABLED' / 'DISABLED' / ... from the executor, or None when nothing answers (then no robot is running)."""
    try:
        with socket.create_connection(("127.0.0.1", 29000), timeout=timeout) as s:
            s.settimeout(timeout)
            s.sendall((json.dumps({"id": "nets", "cmd": "get_status"}) + "\n").encode())
            buf = b""
            while b"\n" not in buf:
                chunk = s.recv(65536)
                if not chunk:
                    break
                buf += chunk
            reply = json.loads(buf.split(b"\n")[0].decode())
            data = reply.get("data", reply)
            return data.get("state") or data.get("robot_state")
    except (OSError, ValueError):
        return None


def list_nets(only):
    nets = sorted(glob.glob(os.path.join(NETS_DIR, "test_*.py")))
    if only:
        nets = [n for n in nets if only in os.path.basename(n)]
    return nets


def unlink_prefixed(prefix):
    n = 0
    for p in glob.glob(f"/dev/shm/{prefix}*"):
        try:
            os.unlink(p)
            n += 1
        except OSError:
            pass
    return n


def run_one(path, index, timeout):
    name = os.path.basename(path)[:-3]
    prefix = f"nets{os.getpid()}_{index}_"
    env = dict(os.environ, RCB_SHM_PREFIX=prefix, PYTHONUNBUFFERED="1", PYTHONPATH=ROOT)
    cmd = [sys.executable, path]
    if shutil.which("ionice"):
        cmd = ["ionice", "-c3"] + cmd
    log = os.path.join(OUT_DIR, name + ".log")
    t0 = time.perf_counter()
    with open(log, "w") as fh:
        try:
            proc = subprocess.run(cmd, cwd=ROOT, env=env, stdout=fh, stderr=subprocess.STDOUT, timeout=timeout,
                                  preexec_fn=lambda: os.nice(19))
            rc = proc.returncode
            why = ""
        except subprocess.TimeoutExpired:
            rc, why = -1, f"TIMEOUT after {timeout:.0f} s"
    dt = time.perf_counter() - t0
    leaked = unlink_prefixed(prefix)
    out = open(log, errors="replace").read()
    marker = FAIL_MARKERS.search(out)
    if rc == 0 and marker:
        rc, why = 1, f"output says: {marker.group(0).strip()[:60]!r}"
    elif rc not in (0, -1):
        why = f"exit {rc}"
        m = re.findall(r"(?m)^(?:\w+Error|\w+Exception|AssertionError)[^\n]*$", out)
        if m:
            why += ": " + m[-1][:80]
    return {"net": name, "ok": rc == 0, "seconds": round(dt, 1), "why": why, "log": os.path.relpath(log, ROOT),
            "shm_segments_cleaned": leaked}


def ruff_findings():
    """{(path, code, message)} from ruff over the source — line numbers dropped so edits above a finding don't
    make it 'new'."""
    ruff = os.path.join(ROOT, ".venv", "bin", "ruff")
    if not os.path.exists(ruff):
        return None, "ruff not installed in .venv (pip install ruff)"
    r = subprocess.run([ruff, "check", *SOURCE, "--output-format", "concise", "--no-fix"], cwd=ROOT,
                       capture_output=True, text=True)
    found = set()
    for line in r.stdout.splitlines():
        m = re.match(r"^(.+?):\d+:\d+: ([A-Z]+\d+) (.*)$", line)
        if m:
            found.add((m.group(1), m.group(2), _norm(m.group(3))))
    return found, None


def _norm(msg):
    """Line numbers inside a message (F811: 'Redefinition of unused `x` from line 382') shift with every edit
    above them; they are not part of the finding's identity."""
    return re.sub(r"\bline \d+", "line N", msg)


def ruff_check(update):
    found, err = ruff_findings()
    if found is None:
        return {"ok": False, "why": err, "new": [], "total": 0}
    if update or not os.path.exists(RUFF_BASELINE):
        with open(RUFF_BASELINE, "w") as fh:
            fh.write(f"# ruff baseline written {time.strftime('%Y-%m-%d %H:%M')} — the rule is NO NEW findings\n")
            for p, c, msg in sorted(found):
                fh.write(f"{p}\t{c}\t{msg}\n")
        return {"ok": True, "why": f"baseline written ({len(found)} findings)", "new": [], "total": len(found)}
    base = set()
    for line in open(RUFF_BASELINE):
        if line.startswith("#") or not line.strip():
            continue
        p, c, msg = line.rstrip("\n").split("\t", 2)
        base.add((p, c, _norm(msg)))
    new = sorted(found - base)
    return {"ok": not new, "why": f"{len(new)} new finding(s) vs baseline of {len(base)}", "new": new,
            "total": len(found)}


def coverage_map(nets):
    """Source modules that no net imports (static) — informational."""
    imported = set()
    for n in nets:
        try:
            t = ast.parse(open(n).read())
        except SyntaxError:
            continue
        for x in ast.walk(t):
            if isinstance(x, ast.Import):
                imported.update(a.name for a in x.names)
            elif isinstance(x, ast.ImportFrom) and x.module:
                imported.add(x.module)
                imported.update(f"{x.module}.{a.name}" for a in x.names)
    mods = []
    for pkg in SOURCE_PKGS:
        for f in sorted(os.listdir(os.path.join(ROOT, pkg))):
            if f.endswith(".py") and f != "__init__.py":
                mods.append(f"{pkg}.{f[:-3]}")
    return [m for m in mods if not any(i == m or i.startswith(m + ".") for i in imported)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", help="run only nets whose file name contains this")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--timeout", type=float, default=1200.0, help="seconds per net (default 1200)")
    ap.add_argument("--no-ruff", action="store_true")
    ap.add_argument("--update-ruff-baseline", action="store_true")
    ap.add_argument("--allow-enabled", action="store_true", help="run even while the robot is ENABLED (not advised)")
    args = ap.parse_args()
    nets = list_nets(args.only)
    all_nets = list_nets(None)
    if args.list:
        for n in nets:
            print("  " + os.path.basename(n))
        unc = coverage_map(all_nets)
        print(f"\n{len(nets)} nets. Source modules NO net imports ({len(unc)}): " + ", ".join(unc))
        return 0
    state = robot_state()
    if state == "ENABLED" and not args.allow_enabled:
        print("REFUSED: the robot is ENABLED. Disable it first (dashboard Disable, or robot.disable()) — nets load "
              "the CPU and slow RTI's ticks. --allow-enabled overrides.")
        return 3
    os.makedirs(OUT_DIR, exist_ok=True)
    print(f"robot: {state or 'no executor answering'} — running {len(nets)} nets serially at nice 19, "
          f"RCB_SHM_PREFIX set per net, timeout {args.timeout:.0f} s each\n")
    results = []
    t_all = time.perf_counter()
    for i, n in enumerate(nets):
        r = run_one(n, i, args.timeout)
        results.append(r)
        print(f"  {'PASS' if r['ok'] else 'FAIL'}  {r['seconds']:7.1f} s  {r['net']}" + (f"   [{r['why']}]" if r["why"] else "")
              + (f"   (cleaned {r['shm_segments_cleaned']} shm segments)" if r["shm_segments_cleaned"] else ""), flush=True)
    ruff = None if args.no_ruff else ruff_check(args.update_ruff_baseline)
    failed = [r for r in results if not r["ok"]]
    lines = [f"nets: {len(results) - len(failed)}/{len(results)} passed in {time.perf_counter() - t_all:.0f} s"]
    for r in failed:
        lines.append(f"  FAIL {r['net']}: {r['why']}  (see {r['log']})")
    if ruff is not None:
        lines.append(f"ruff: {'OK' if ruff['ok'] else 'FAIL'} — {ruff['why']}")
        for p, c, msg in ruff["new"][:30]:
            lines.append(f"  NEW {p}: {c} {msg}")
    unc = coverage_map(all_nets)
    lines.append(f"source modules no net imports ({len(unc)}): " + ", ".join(unc))
    summary = "\n".join(lines)
    print("\n" + summary)
    with open(os.path.join(OUT_DIR, "last_run.txt"), "w") as fh:
        fh.write(time.strftime("%Y-%m-%d %H:%M:%S") + "\n" + summary + "\n")
    with open(os.path.join(OUT_DIR, "last_run.json"), "w") as fh:
        json.dump({"when": time.time(), "results": results, "ruff": ruff}, fh, indent=1)
    ok = not failed and (ruff is None or ruff["ok"])
    print("\nRESULT:", "ALL GREEN" if ok else "RED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
