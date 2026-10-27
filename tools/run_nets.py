#!/usr/bin/env python3
"""Run every offline test net in dummy_test_scripts/ with ONE command, and say pass or fail.

    .venv/bin/python tools/run_nets.py                 # everything (refuses while the robot is ENABLED)
    .venv/bin/python tools/run_nets.py --only haptic   # nets whose file name contains "haptic"
    .venv/bin/python tools/run_nets.py --list          # what would run, and which source modules no net imports
    .venv/bin/python tools/run_nets.py --changed       # only the nets the uncommitted changes touch (see 8.)
    .venv/bin/python tools/run_nets.py --changed v1.1.1 --list   # ...those since a tag / commit, listed with why
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
  6. (2026-10-13, tools/quality.py) mypy, bandit, vulture and radon run over the runtime, each against its own
     baseline (temp_data/quality/): NO NEW findings. --coverage measures line + branch coverage of the nets and
     ratchets it (temp_data/coverage/html). A failed net is rerun once: a pass then is reported FLAKY (and logged in
     temp_data/quality/flaky_history.tsv). A full run regenerates docs/test_catalog.md. --audit adds the known-
     vulnerability audit (report only). The policy for new features: docs/testing.md "Testing policy".
  7. (2026-10-16) Every process it starts is capped at MEM_CAP_MB of address space (RLIMIT_AS, set on the runner and
     inherited): a net that allocates without bound gets a MemoryError instead of running the Pi out of memory
     (that day a mutant allocated ~3 GB; the Pi — 4 GB, no swap — stalled RTI 194 s). --mem-cap-mb 0 = no cap.
  8. (2026-10-27, user: "we are testing and simulating waaaay too much") --changed [REF] runs only the nets a change
     touches: the .py files changed against REF (default HEAD: the uncommitted changes; untracked files included),
     then every module that imports one of them, directly or through other modules (the import graph of the tree),
     and the nets that import any of those or read a changed file's source by its path or file name (the nets that
     lift RTI.py's functions, for one). The full suite still runs once at push (the pre-push hook). The nets
     themselves are not tracked by git: run a net you changed with --only.

Exit code 0 only when every net and every check pass. The summary is also written to temp_data/nets/last_run.txt.
"""
import argparse
import ast
import glob
import json
import os
import re
import resource
import shutil
import socket
import subprocess
import sys
import time

import quality                                     # tools/quality.py (same folder)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NETS_DIR = os.path.join(ROOT, "dummy_test_scripts")
OUT_DIR = os.path.join(ROOT, "temp_data", "nets")
NET_SITE = os.path.join(ROOT, "tools", "net_site")     # its sitecustomize.py points tempfile at the net's folder
RUFF_BASELINE = os.path.join(ROOT, "temp_data", "ruff_baseline.txt")
# The runtime source: ONE list for ruff, the static checks and the import map (tools/quality.py) — ruff covered
# less until 2026-10-13 (not blockly/, sim/, the self-collision worker, six gui_*.py: 0 findings there).
SOURCE = quality.RUNTIME_FILES + quality.RUNTIME_PKGS
SOURCE_PKGS = tuple(quality.RUNTIME_PKGS)
# A net passed when its exit code is 0 AND no line STARTS with one of these (anchored: a PASS line whose text
# mentions "FAILED" must not count). Tracebacks in the output are NOT a failure signal: a crash gives a non-zero
# exit anyway, while several nets print tracebacks on purpose (a handler raising "boom" to test the intake guard)
# or at exit (CPython's resource_tracker complaining about a segment the net unlinked itself).
# The address-space cap per process (rule 7). The whole suite passed under 1.5 GB on 2026-10-16.
MEM_CAP_MB = 1500
FAIL_MARKERS = re.compile(r"(?m)^\s*(?:(?!0 )\d+ (?:failed|FAILED|FAILURE)|FAILED\b|FAILURES:|\[FAIL\]|FAIL\s{2,}|"
                          r"PLAN ERROR|RESULT: (?!ALL)\S*FAIL)")
# A CPU-time budget skips itself only under coverage (dummy_test_scripts/_timing.py); in a plain run that line means
# the budget silently stopped being judged — a failure.
COVERAGE_SKIP = re.compile(r"(?m)^SKIP .*\(coverage run:")


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


def run_one(path, index, timeout, coverage=False):
    name = os.path.basename(path)[:-3]
    prefix = f"nets{os.getpid()}_{index}_"
    # RCB_LOGS_DIR (project_paths): the test's logs in a folder of its own, never the robot's logs/ (2026-10-13)
    logs = os.path.join(OUT_DIR, "logs", name)
    shutil.rmtree(logs, ignore_errors=True)
    # The net's temporary files (tempfile.mkdtemp & co.) in a folder of its own, deleted when the net passes and kept
    # until its next run when it fails — they piled up in /tmp, on the SD card, before (2026-10-15). RCB_TMPDIR, read
    # by tools/net_site/sitecustomize.py (first on PYTHONPATH): python3 here has a file capability, so the C library
    # drops TMPDIR from its environment at start. TMPDIR is set as well, for whatever else a net runs.
    tmp = os.path.join(OUT_DIR, "tmp", name)
    shutil.rmtree(tmp, ignore_errors=True)
    os.makedirs(tmp, exist_ok=True)
    env = dict(os.environ, RCB_SHM_PREFIX=prefix, PYTHONUNBUFFERED="1", PYTHONPATH=os.pathsep.join((NET_SITE, ROOT)),
               RCB_LOGS_DIR=logs, RCB_TMPDIR=tmp, TMPDIR=tmp)
    cmd = [sys.executable, path]
    if coverage:                                       # measured: lines + branches, its subprocesses too
        cmd = quality.coverage_command(path)
        env.update(quality.coverage_env())
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
    if rc == 0:
        shutil.rmtree(tmp, ignore_errors=True)       # a failed net keeps its temporary files for a look
    out = open(log, errors="replace").read()
    marker = FAIL_MARKERS.search(out)
    if rc == 0 and marker:
        rc, why = 1, f"output says: {marker.group(0).strip()[:60]!r}"
    elif rc == 0 and not coverage and COVERAGE_SKIP.search(out):
        rc, why = 1, "a CPU-time budget skipped itself outside a coverage run (dummy_test_scripts/_timing.py)"
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


_SKIP_DIRS = {".venv", ".git", "temp_data", "logs", "dummy_test_scripts", "__pycache__", "node_modules"}


def changed_sources(ref="HEAD"):
    """The .py files (repo-relative) changed against `ref` — the index and the working tree — plus untracked ones."""
    def git(*a):
        r = subprocess.run(["git", "-C", ROOT, *a], capture_output=True, text=True)
        if r.returncode:
            raise SystemExit(f"git {' '.join(a)}: {r.stderr.strip()}")
        return r.stdout.split()
    files = set(git("diff", "--name-only", ref)) | set(git("ls-files", "--others", "--exclude-standard"))
    return sorted(f for f in files if f.endswith(".py"))


def _module_of(rel):
    """'utility/safety_monitor.py' -> 'utility.safety_monitor'; a package's __init__.py -> the package."""
    mod = rel[:-3].replace(os.sep, ".")
    return mod[:-len(".__init__")] if mod.endswith(".__init__") else mod


def _imports_of(path):
    """Every module name a file imports (static): `import a.b` -> a.b; `from a import b` -> a and a.b."""
    try:
        tree = ast.parse(open(path, encoding="utf-8").read())
    except (SyntaxError, UnicodeDecodeError, OSError):
        return set()
    out = set()
    for x in ast.walk(tree):
        if isinstance(x, ast.Import):
            out.update(a.name for a in x.names)
        elif isinstance(x, ast.ImportFrom) and x.module:
            out.add(x.module)
            out.update(f"{x.module}.{a.name}" for a in x.names)
    return out


def _code_strings(path):
    """The string constants a file's CODE uses — its docstrings left out ("Run: tools/run_nets.py --only x" in every
    net's docstring is no read of run_nets.py), comments are not in the tree anyway."""
    try:
        tree = ast.parse(open(path, encoding="utf-8").read())
    except (SyntaxError, UnicodeDecodeError, OSError):
        return []
    docs = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and node.body \
                and isinstance(node.body[0], ast.Expr) and isinstance(node.body[0].value, ast.Constant):
            docs.add(id(node.body[0].value))
    return [x.value for x in ast.walk(tree)
            if isinstance(x, ast.Constant) and isinstance(x.value, str) and id(x) not in docs]


def affected_nets(changed, nets):
    """The nets `changed` (repo-relative .py paths) can affect -> [(net, why)]. A changed module, every module that
    imports one (transitively), and the nets importing any of those or naming a changed file (its path, or its file
    name unless that is __init__.py). A changed tools/ module also matches its bare name (tools put tools/ on
    sys.path and `import name`)."""
    graph = {}
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in _SKIP_DIRS and not d.startswith(".")]
        for f in filenames:
            if f.endswith(".py"):
                rel = os.path.relpath(os.path.join(dirpath, f), ROOT)
                graph[_module_of(rel)] = _imports_of(os.path.join(dirpath, f))

    def names(mods):
        return set(mods) | {m.rsplit(".", 1)[-1] for m in mods if m.startswith("tools.")}

    hit = names(_module_of(c) for c in changed)
    grew = True
    while grew:                                           # who imports a hit module, transitively
        grew = False
        for mod, imps in graph.items():
            if mod not in hit and imps & hit:
                hit.add(mod)
                grew = True
    texts = [(c, os.path.basename(c)) for c in changed]
    out = []
    for n in nets:
        strings = _code_strings(n)
        via = sorted(_imports_of(n) & hit)
        named = [c for c, base in texts
                 if any(c in s or (base != "__init__.py" and base in s) for s in strings)]
        if via or named:
            out.append((n, ("imports " + ", ".join(via[:3]) + (" ..." if len(via) > 3 else "")) if via
                        else "reads " + ", ".join(named[:3])))
    return out


def apply_memory_cap(mb):
    """Cap this process's address space at `mb` MB (0 / None: no cap); every child inherits it. Returns the cap in
    bytes, or None."""
    if not mb:
        return None
    _soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    cap = int(mb) * 1024 * 1024
    if hard != resource.RLIM_INFINITY:
        cap = min(cap, hard)
    resource.setrlimit(resource.RLIMIT_AS, (cap, hard))
    return cap


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", help="run only nets whose file name contains this")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--changed", nargs="?", const="HEAD", metavar="REF",
                    help="run only the nets the .py files changed against REF touch (default HEAD: the uncommitted "
                         "changes); with --list, which and why")
    ap.add_argument("--timeout", type=float, default=1200.0, help="seconds per net (default 1200)")
    ap.add_argument("--no-ruff", action="store_true")
    ap.add_argument("--update-ruff-baseline", action="store_true")
    ap.add_argument("--allow-enabled", action="store_true", help="run even while the robot is ENABLED (not advised)")
    ap.add_argument("--coverage", action="store_true", help="measure line + branch coverage (slower); a ratchet")
    ap.add_argument("--no-static", action="store_true", help="skip mypy / bandit / vulture / radon")
    ap.add_argument("--audit", action="store_true", help="also the known-vulnerability audit (internet; report only)")
    ap.add_argument("--no-rerun", action="store_true", help="do not rerun a failed net once (flaky detection)")
    ap.add_argument("--mem-cap-mb", type=int, default=MEM_CAP_MB,
                    help=f"address-space cap per process in MB (default {MEM_CAP_MB}; 0 = no cap)")
    ap.add_argument("--update-baselines", action="store_true",
                    help="accept today's ruff / static / coverage state as the baselines (after review only)")
    args = ap.parse_args()
    if args.update_baselines:
        args.update_ruff_baseline = True
    nets = list_nets(args.only)
    all_nets = list_nets(None)
    partial = bool(args.only) or args.changed is not None     # no catalog, no coverage ratchet for a partial run
    if args.changed is not None:
        changed = changed_sources(args.changed)
        picked = affected_nets(changed, nets)
        print(f"--changed {args.changed}: {len(changed)} changed .py file(s): {', '.join(changed) or 'none'}")
        for n, why in picked:
            print(f"  {os.path.basename(n):40s} {why}")
        print(f"-> {len(picked)} of {len(all_nets)} nets\n")
        nets = [n for n, _ in picked]
        if args.list:
            return 0
        if not nets:
            print("nothing to run")
            return 0
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
    cap = apply_memory_cap(args.mem_cap_mb)
    print(f"robot: {state or 'no executor answering'} — running {len(nets)} nets serially at nice 19, "
          f"RCB_SHM_PREFIX set per net, timeout {args.timeout:.0f} s each, "
          f"{f'memory capped at {cap // 1048576} MB per process' if cap else 'NO memory cap'}\n")
    results = []
    t_all = time.perf_counter()
    if args.coverage:
        quality.coverage_prepare()
    for i, n in enumerate(nets):
        r = run_one(n, i, args.timeout, args.coverage)
        if not r["ok"] and not args.no_rerun and not r["why"].startswith("TIMEOUT"):
            first_why = r["why"]                          # flaky detection: one rerun; a pass then is FLAKY, not green
            r = run_one(n, i, args.timeout, args.coverage)
            if r["ok"]:
                r["flaky"], r["why"] = True, f"FLAKY — failed once ({first_why}), passed on the rerun"
        results.append(r)
        tag = "FLAKY" if r.get("flaky") else ("PASS" if r["ok"] else "FAIL")
        print(f"  {tag:5s} {r['seconds']:7.1f} s  {r['net']}" + (f"   [{r['why']}]" if r["why"] else "")
              + (f"   (cleaned {r['shm_segments_cleaned']} shm segments)" if r["shm_segments_cleaned"] else ""), flush=True)
    t_nets = time.perf_counter() - t_all
    ruff = None if args.no_ruff else ruff_check(args.update_ruff_baseline)
    static = None if args.no_static else quality.static_checks(args.update_baselines)
    cov = quality.coverage_finish(args.update_baselines, ratchet=not partial) if args.coverage else None
    aud = quality.audit() if args.audit else None
    failed = [r for r in results if not r["ok"]]
    flaky = [r["net"] for r in results if r.get("flaky")]
    quality.record_flaky(flaky)
    lines = [f"nets: {len(results) - len(failed)}/{len(results)} passed in {t_nets:.0f} s"
             + (f" ({len(flaky)} FLAKY: {', '.join(flaky)})" if flaky else "")]
    for r in failed:
        lines.append(f"  FAIL {r['net']}: {r['why']}  (see {r['log']})")
    if ruff is not None:
        lines.append(f"ruff: {'OK' if ruff['ok'] else 'FAIL'} — {ruff['why']}")
        for p, c, msg in ruff["new"][:30]:
            lines.append(f"  NEW {p}: {c} {msg}")
    for name, v in (static or {}).items():
        lines.append(f"{name}: {'OK' if v['ok'] else 'FAIL'} — {v['why']}")
        for f in v["new"][:30]:
            lines.append("  NEW " + " | ".join(f))
    if cov is not None:
        lines.append(f"coverage: {'OK' if cov['ok'] else 'FAIL'} — {cov['why']} (temp_data/coverage/html/index.html)")
        for pth, was, now in cov.get("drops", [])[:20]:
            lines.append(f"  DROP {pth}: {was} % -> {now} %")
    if aud is not None:
        lines.append(f"audit: {'OK' if aud['ok'] else 'WARN'} — {aud['why']} (report only)")
    if not partial:
        lines.append("catalog: " + os.path.relpath(quality.write_catalog(results, static, cov, ruff), ROOT))
    unc = coverage_map(all_nets)
    lines.append(f"source modules no net imports ({len(unc)}): " + ", ".join(unc))
    summary = "\n".join(lines)
    print("\n" + summary)
    with open(os.path.join(OUT_DIR, "last_run.txt"), "w") as fh:
        fh.write(time.strftime("%Y-%m-%d %H:%M:%S") + "\n" + summary + "\n")
    with open(os.path.join(OUT_DIR, "last_run.json"), "w") as fh:
        json.dump({"when": time.time(), "results": results, "ruff": ruff, "static": static,
                   "coverage": None if cov is None else {k: v for k, v in cov.items() if k != "files"}}, fh, indent=1)
    ok = not failed and (ruff is None or ruff["ok"]) and all(v["ok"] for v in (static or {}).values()) \
        and (cov is None or cov["ok"])
    print("\nRESULT:", "ALL GREEN" if ok else "RED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
