#!/usr/bin/env python3
"""The test runner's quality checks (2026-10-13, user: "everything industry standard"; docs/testing.md): static type
checking (mypy), security lint (bandit), dead code (vulture), complexity (radon), line + branch coverage of the nets
(coverage), the known-vulnerability audit of the installed packages (pip-audit), the flaky-test record and the
generated test catalog (docs/test_catalog.md). tools/run_nets.py calls these; by hand:

    .venv/bin/python tools/quality.py            # the static checks alone (~25 s)
    .venv/bin/python tools/quality.py --audit    # + the vulnerability audit (needs the internet)

THE RULE, as for ruff since 2026-10-02: each check has a BASELINE (temp_data/quality/<tool>_baseline.txt — what the
tree had when the baseline was written) and a run fails only on a NEW finding. A finding's identity has no line
number, so editing above it does not make it new. Coverage has a ratchet instead: the total may not drop more than
COVERAGE_TOTAL_SLACK points, no file more than COVERAGE_FILE_SLACK. `--update-baselines` (run_nets) accepts today's
state — only after fixing what is real or consciously accepting it.
"""
from __future__ import annotations

import argparse
import ast
import json
import os
import re
import site
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
QDIR = os.path.join(ROOT, "temp_data", "quality")
COVDIR = os.path.join(ROOT, "temp_data", "coverage")
RCFILE = os.path.join(ROOT, ".coveragerc")
os.environ.setdefault("RCB_ROOT", ROOT)   # .coveragerc's ${RCB_ROOT-.}: every child process measures THIS tree
VENV_BIN = os.path.join(ROOT, ".venv", "bin")
CATALOG = os.path.join(ROOT, "docs", "test_catalog.md")
NETS_DIR = os.path.join(ROOT, "dummy_test_scripts")

# THE runtime source — what the robot runs (not tests, tools or archives). One list for every check.
RUNTIME_FILES = ["RTI.py", "command_executor.py", "supervisor.py", "nicegui_test.py", "project_paths.py",
                 "self_collision_worker.py", "gui_command_sender.py", "gui_hall_check.py", "gui_homing_arms.py",
                 "gui_jaw_view.py", "gui_processes.py", "gui_self_collision.py", "gui_time_card.py", "gui_walls_tab.py"]
RUNTIME_PKGS = ["blockly", "communication", "config", "data", "hardware", "motion", "robotclient", "robotics",
                "robots", "sim", "supervisor", "utility"]
RADON_MAX_CC = 30            # a function above this cyclomatic complexity (radon rank E / F) is a finding
COVERAGE_TOTAL_SLACK = 0.3   # percentage points the total may drop (thread timing moves a few lines)
COVERAGE_FILE_SLACK = 3.0    # ... and one file
HOOK_NAME = "zz_rcb_coverage_subprocess.pth"


def _norm(msg):
    """No line numbers in a finding's identity (they move with every edit above it)."""
    return re.sub(r"\bline \d+", "line N", re.sub(r"\s+", " ", msg)).strip()


def _run(cmd, timeout=900):
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True, timeout=timeout)


def _tool(name):
    p = os.path.join(VENV_BIN, name)
    return p if os.path.exists(p) else None


# ------------------------------------------------------------------------------------------------ the static checks
def mypy_findings():
    """{(path, code, message)} — two passes: supervisor.py (the script) and the supervisor/ package share a name."""
    exe = _tool("mypy")
    if exe is None:
        return None, "mypy not installed (pip install -r requirements-dev.txt)"
    rest = [f for f in RUNTIME_FILES if f != "supervisor.py"] + RUNTIME_PKGS
    found = set()
    for files in (rest, ["supervisor.py"]):
        r = _run([exe, "--explicit-package-bases", *files])
        if "errors prevented further checking" in r.stdout:
            return None, "mypy could not check: " + r.stdout.strip().splitlines()[0][:160]
        for line in r.stdout.splitlines():
            m = re.match(r"^(.+?):\d+: error: (.*?)\s+\[([a-z-]+)\]$", line)
            if m:
                found.add((m.group(1), m.group(3), _norm(m.group(2))))
    return found, None


def bandit_findings():
    """{(path, test id, issue)} — B101 (assert) skipped: asserts here are test-style invariants, not input checks."""
    exe = _tool("bandit")
    if exe is None:
        return None, "bandit not installed"
    r = _run([exe, "-r", *RUNTIME_FILES, *RUNTIME_PKGS, "-f", "json", "-q", "-s", "B101", "-x", "./blockly/web"])
    try:
        res = json.loads(r.stdout)["results"]
    except (ValueError, KeyError):
        return None, "bandit gave no report: " + (r.stderr or r.stdout)[:160]
    return {(os.path.relpath(x["filename"], ROOT), x["test_id"], _norm(x["issue_text"])) for x in res}, None


def vulture_findings():
    """{(path, message)} — dead code, confidence >= 80 %."""
    exe = _tool("vulture")
    if exe is None:
        return None, "vulture not installed"
    r = _run([exe, *RUNTIME_FILES, *RUNTIME_PKGS, "--min-confidence", "80", "--exclude", "blockly/web"])
    found = set()
    for line in r.stdout.splitlines():
        m = re.match(r"^(.+?):\d+: (.*?) \(\d+% confidence\)$", line)
        if m:
            found.add((m.group(1), _norm(m.group(2))))
    return found, None


def radon_findings():
    """{(path, function)} with cyclomatic complexity above RADON_MAX_CC (radon rank E / F)."""
    exe = _tool("radon")
    if exe is None:
        return None, "radon not installed"
    r = _run([exe, "cc", "-j", *RUNTIME_FILES, *RUNTIME_PKGS])
    try:
        data = json.loads(r.stdout)
    except ValueError:
        return None, "radon gave no report"
    found = set()
    for path, blocks in data.items():
        if not isinstance(blocks, list):
            continue
        for b in blocks:
            if b.get("complexity", 0) > RADON_MAX_CC:
                name = f"{b.get('classname')}.{b['name']}" if b.get("classname") else b["name"]
                found.add((path, name))
    return found, None


CHECKS = {"mypy": mypy_findings, "bandit": bandit_findings, "vulture": vulture_findings, "radon": radon_findings}


def baseline_check(name, found, update):
    """The ratchet: NEW findings (not in temp_data/quality/<name>_baseline.txt) fail."""
    os.makedirs(QDIR, exist_ok=True)
    path = os.path.join(QDIR, f"{name}_baseline.txt")
    if update or not os.path.exists(path):
        with open(path, "w") as fh:
            fh.write(f"# {name} baseline written {time.strftime('%Y-%m-%d %H:%M')} — the rule is NO NEW findings\n")
            for f in sorted(found):
                fh.write("\t".join(f) + "\n")
        return {"ok": True, "why": f"baseline written ({len(found)} findings)", "new": [], "total": len(found),
                "fixed": 0}
    base = set()
    for line in open(path):
        if line.strip() and not line.startswith("#"):
            base.add(tuple(line.rstrip("\n").split("\t")))
    new = sorted(found - base)
    return {"ok": not new, "why": f"{len(new)} new finding(s) vs baseline of {len(base)} ({len(base - found)} fixed)",
            "new": new, "total": len(found), "fixed": len(base - found)}


def static_checks(update=False):
    """{tool: result} for every static check."""
    out = {}
    for name, fn in CHECKS.items():
        t0 = time.perf_counter()
        try:
            found, err = fn()
        except (OSError, subprocess.SubprocessError) as e:
            found, err = None, f"{type(e).__name__}: {e}"
        if found is None:
            out[name] = {"ok": False, "why": err, "new": [], "total": 0, "fixed": 0}
        else:
            out[name] = baseline_check(name, found, update)
        out[name]["seconds"] = round(time.perf_counter() - t0, 1)
    return out


# ------------------------------------------------------------------------------------------------ coverage
def coverage_prepare():
    """Fresh data; the .venv startup hook that makes every Python process with COVERAGE_PROCESS_START measure itself
    (a net's runner subprocess, the block programs, ...). The hook does nothing without that variable."""
    os.makedirs(COVDIR, exist_ok=True)
    for f in os.listdir(COVDIR):
        if f.startswith(".coverage"):
            os.unlink(os.path.join(COVDIR, f))
    sp = [p for p in site.getsitepackages() if p.startswith(os.path.join(ROOT, ".venv"))]
    if sp:
        hook = os.path.join(sp[0], HOOK_NAME)
        if not os.path.exists(hook):
            with open(hook, "w") as fh:
                fh.write("import os; os.environ.get('COVERAGE_PROCESS_START') and "
                         "__import__('coverage').process_startup()\n")


def coverage_command(path):
    return [sys.executable, "-m", "coverage", "run", f"--rcfile={RCFILE}", path]


def coverage_env():
    # RCB_COVERAGE: CPU-time budget checks skip themselves (dummy_test_scripts/_timing.py — the tracer is 2-4x slower)
    return {"COVERAGE_PROCESS_START": RCFILE, "RCB_ROOT": ROOT, "RCB_COVERAGE": "1"}


def coverage_finish(update=False, ratchet=True):
    """Combine, report (text / JSON / HTML) and apply the ratchet. Returns the result dict. `ratchet=False` (a partial
    run, --only): report only — a few nets' coverage must neither become the baseline nor be judged against it."""
    cov = [sys.executable, "-m", "coverage"]
    _run(cov + ["combine", f"--rcfile={RCFILE}"], timeout=600)
    rep = _run(cov + ["report", f"--rcfile={RCFILE}"], timeout=600)
    _run(cov + ["json", f"--rcfile={RCFILE}"], timeout=600)
    _run(cov + ["html", f"--rcfile={RCFILE}"], timeout=900)
    with open(os.path.join(COVDIR, "report.txt"), "w") as fh:
        fh.write(rep.stdout)
    try:
        data = json.load(open(os.path.join(COVDIR, "coverage.json")))
    except (OSError, ValueError):
        return {"ok": False, "why": "no coverage data (coverage combine found nothing)", "total": None, "files": {}}
    files = {p: round(v["summary"]["percent_covered"], 1) for p, v in data["files"].items()}
    total = round(data["totals"]["percent_covered"], 1)
    res = {"total": total, "files": files, "lines": data["totals"]["num_statements"],
           "covered_lines": data["totals"]["covered_lines"], "branches": data["totals"].get("num_branches", 0),
           "covered_branches": data["totals"].get("covered_branches", 0)}
    if not ratchet:
        res.update(ok=True, why=f"{total} % (lines + branches; a partial run — no ratchet)", drops=[])
        return res
    base_path = os.path.join(QDIR, "coverage_baseline.json")
    os.makedirs(QDIR, exist_ok=True)
    if update or not os.path.exists(base_path):
        json.dump({"written": time.strftime("%Y-%m-%d %H:%M"), "total": total, "files": files},
                  open(base_path, "w"), indent=1, sort_keys=True)
        res.update(ok=True, why=f"{total} % (lines + branches); baseline written", drops=[])
        return res
    base = json.load(open(base_path))
    drops = [(p, base["files"][p], v) for p, v in files.items()
             if p in base["files"] and v < base["files"][p] - COVERAGE_FILE_SLACK]
    total_drop = total < base["total"] - COVERAGE_TOTAL_SLACK
    res.update(ok=not drops and not total_drop, drops=drops,
               why=f"{total} % (baseline {base['total']} %)" + (" — the TOTAL DROPPED" if total_drop else "")
               + (f"; {len(drops)} file(s) dropped" if drops else ""))
    return res


def coverage_by_package(files):
    """{package: (statement-weighted %)} from coverage.json — for the catalog."""
    try:
        data = json.load(open(os.path.join(COVDIR, "coverage.json")))
    except (OSError, ValueError):
        return {}
    agg = {}
    for p, v in data["files"].items():
        pkg = p.split("/", 1)[0] if "/" in p else "(top level)"
        s = v["summary"]
        a = agg.setdefault(pkg, [0, 0])
        a[0] += s["covered_lines"] + s.get("covered_branches", 0)
        a[1] += s["num_statements"] + s.get("num_branches", 0)
    return {k: round(100.0 * c / n, 1) if n else 100.0 for k, (c, n) in sorted(agg.items())}


# ------------------------------------------------------------------------------------------------ the audit
def audit():
    """Known vulnerabilities (pip-audit: the PyPI / OSV advisories) in the .venv packages AND the system interpreter's
    (RTI runs on system python3 + ~/.local). Report only — a vendor library cannot always be upgraded at once."""
    exe = _tool("pip-audit")
    if exe is None:
        return {"ok": False, "why": "pip-audit not installed", "report": ""}
    out = []
    r = _run([exe, "--progress-spinner", "off"], timeout=900)
    out.append("== .venv ==\n" + (r.stdout or r.stderr))
    freeze = _run(["python3", "-m", "pip", "freeze", "--user"], timeout=120).stdout
    req = os.path.join(QDIR, "system_freeze.txt")
    os.makedirs(QDIR, exist_ok=True)
    open(req, "w").write("\n".join(ln for ln in freeze.splitlines() if "==" in ln and not ln.startswith("-e")))
    r2 = _run([exe, "-r", req, "--no-deps", "--disable-pip", "--progress-spinner", "off"], timeout=900)
    out.append("== system python3 (~/.local, RTI's interpreter) ==\n" + (r2.stdout or r2.stderr))
    text = "\n".join(out)
    # pip-audit prints an advisory once per alias it matched: count each (interpreter, package, advisory) once
    found = {}
    for sec in re.split(r"(?m)^== ", text)[1:]:
        label = sec.split(" ==", 1)[0]
        for name, ver, vid in re.findall(r"(?m)^(\S+)\s+(\S+)\s+((?:GHSA|PYSEC|CVE)-\S+)", sec):
            found.setdefault(label, {}).setdefault(f"{name} {ver}", set()).add(vid)
    vulns = sum(len(ids) for pk in found.values() for ids in pk.values())
    summary = "\n".join(f"{label}: " + ", ".join(f"{pkg} ({len(ids)})" for pkg, ids in sorted(pk.items()))
                        for label, pk in found.items())
    open(os.path.join(QDIR, "audit.txt"), "w").write("SUMMARY (unique advisories per package)\n" + summary + "\n\n" + text)
    npk = sum(len(pk) for pk in found.values())
    return {"ok": vulns == 0, "why": f"{vulns} known vulnerabilit{'y' if vulns == 1 else 'ies'} in {npk} package(s) "
            f"(temp_data/quality/audit.txt)", "report": text, "summary": summary}


# ------------------------------------------------------------------------------------------------ flaky + catalog
def record_flaky(names):
    if not names:
        return
    os.makedirs(QDIR, exist_ok=True)
    with open(os.path.join(QDIR, "flaky_history.tsv"), "a") as fh:
        for n in names:
            fh.write(f"{time.strftime('%Y-%m-%d %H:%M')}\t{n}\n")


def _net_summary(path):
    try:
        doc = ast.get_docstring(ast.parse(open(path).read())) or ""
    except (OSError, SyntaxError):
        return "", []
    first = " ".join(doc.split("\n\n", 1)[0].split())
    mods = set()
    try:
        t = ast.parse(open(path).read())
    except SyntaxError:
        t = None
    for x in ast.walk(t) if t is not None else []:
        if isinstance(x, ast.ImportFrom) and x.module:
            mods.add(x.module)
        elif isinstance(x, ast.Import):
            mods.update(a.name for a in x.names)
    top = tuple(RUNTIME_PKGS) + tuple(f[:-3] for f in RUNTIME_FILES)
    return first, sorted(m for m in mods if m.split(".")[0] in top)


def _checks_in_log(net):
    try:
        out = open(os.path.join(ROOT, "temp_data", "nets", net + ".log"), errors="replace").read()
    except OSError:
        return None
    return len(re.findall(r"(?m)^\s*(?:PASS\b|\[PASS\]|ok\b|✓)", out))


def write_catalog(results, static=None, coverage=None, ruff=None):
    """docs/test_catalog.md — every net: what it covers (its docstring's first paragraph), the runtime modules it
    imports, its checks and time, the last result; the coverage by package and the static checks' state."""
    rows = []
    for r in results:
        summary, mods = _net_summary(os.path.join(NETS_DIR, r["net"] + ".py"))
        rows.append((r, summary, mods, _checks_in_log(r["net"])))
    lines = [
        "# Test catalog (generated)",
        "",
        f"Written by `tools/run_nets.py` on {time.strftime('%Y-%m-%d %H:%M')} — do not edit by hand; the policy is in",
        "`docs/testing.md` (\"Testing policy\"). Each net's text is the first paragraph of its docstring.",
        "",
        f"**{len(rows)} nets**, {sum(c or 0 for _, _, _, c in rows)} checks counted in their logs, "
        f"{sum(r['seconds'] for r, *_ in rows):.0f} s; "
        f"{sum(1 for r, *_ in rows if r['ok'])} passed"
        + (f", {sum(1 for r, *_ in rows if r.get('flaky'))} flaky" if any(r.get('flaky') for r, *_ in rows) else "")
        + ".",
        "",
    ]
    if coverage and coverage.get("total") is not None:
        lines += [f"## Coverage — {coverage['total']} % (lines + branches)", "",
                  f"{coverage['covered_lines']} of {coverage['lines']} lines, {coverage['covered_branches']} of "
                  f"{coverage['branches']} branches. Line by line: `temp_data/coverage/html/index.html`.", "",
                  "| package | covered |", "|---|---|"]
        lines += [f"| {k} | {v} % |" for k, v in coverage_by_package(coverage.get("files", {})).items()]
        lines.append("")
    if static or ruff:
        lines += ["## Static checks (findings now / new vs the baseline)", "", "| check | findings | new | fixed |",
                  "|---|---|---|---|"]
        if ruff:
            lines.append(f"| ruff | {ruff.get('total', '?')} | {len(ruff.get('new', []))} | — |")
        for k, v in (static or {}).items():
            lines.append(f"| {k} | {v.get('total', '?')} | {len(v.get('new', []))} | {v.get('fixed', 0)} |")
        lines.append("")
    lines += ["## Nets", "", "| net | covers | runtime modules it imports | checks | s | last |", "|---|---|---|---|---|---|"]
    for r, summary, mods, checks in rows:
        status = "FLAKY" if r.get("flaky") else ("pass" if r["ok"] else "FAIL")
        cov = ((summary[:220] + "…") if len(summary) > 220 else summary) or "**no docstring — add one** (docs/testing.md)"
        lines.append(f"| `{r['net']}` | {cov.replace('|', '/')} | {', '.join(mods[:8])}{' …' if len(mods) > 8 else ''} | "
                     f"{checks if checks is not None else '—'} | {r['seconds']} | {status} |")
    os.makedirs(os.path.dirname(CATALOG), exist_ok=True)
    with open(CATALOG, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return CATALOG


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--update-baselines", action="store_true", help="accept today's findings as the baselines")
    ap.add_argument("--audit", action="store_true", help="also the known-vulnerability audit (internet)")
    args = ap.parse_args()
    res = static_checks(args.update_baselines)
    ok = True
    for k, v in res.items():
        print(f"{k:8s} {'OK  ' if v['ok'] else 'FAIL'} {v['why']} ({v['seconds']} s)")
        for f in v["new"][:20]:
            print("    NEW " + " | ".join(f))
        ok = ok and v["ok"]
    if args.audit:
        a = audit()
        print(f"audit    {'OK  ' if a['ok'] else 'WARN'} {a['why']}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
