#!/usr/bin/env python3
"""What on the SD card can go, and (with --apply) deleting it — the caches nothing else limits (2026-10-15).

The robot's own files are bounded where they are written (docs/logging.md "Rotation and limits"; the test
recordings' 500 MB budget is kept by the supervisor). Two things on this box grow with nobody limiting them:

  * VS Code's remote server (~/.vscode-server): every VS Code update downloads a new server build (~600 MB) and new
    extension versions; the old ones stay. VS Code decides what it needs, so this tool deletes only what no process
    uses and VS Code no longer lists:
      - server builds (cli/servers/Stable-<commit>) that no process runs, except the newest in cli/servers/lru.json
        (the one VS Code switches to next);
      - CLI binaries (code-<commit>) no process runs, except the newest; their .cli.<commit>.log;
      - extension folders VS Code's extensions.json does not list, or lists in .obsolete, that no process uses
        (versions it still lists are reported, never deleted);
      - the installer cache (data/CachedExtensionVSIXs) and log folders older than a week (data/logs).
  * /tmp (on the SD card here): temporary folders our tests and test scripts made and left — the ones named with a
    prefix our code uses (found in the code: tempfile.mkdtemp(prefix=...)), older than --min-age-h. tools/run_nets.py
    gives every net its own temporary folder since 2026-10-15, so new ones no longer pile up there.

It also reports (never deletes) logs/, the test recordings against their budget, the system journal and syslog,
other Claude sessions' scratchpads, and the free space.

    tools/disk_cleanup.py                 # the report: what would go and why; nothing deleted
    tools/disk_cleanup.py --apply         # delete it
    tools/disk_cleanup.py --only vscode   # or --only tmp

"In use" = some process's command line or working folder contains the path (/proc). Run it with VS Code connected
or not: the server build and extensions a connected VS Code runs are in use and kept.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

LOG_AGE_DAYS = 7
PREFIX_RE = re.compile(r"""(?:mkdtemp|TemporaryDirectory)\(\s*prefix\s*=\s*["']([A-Za-z0-9_.-]+)["']""")
CODE_DIRS = ("dummy_test_scripts", "api_tests", "tools", "blockly")


# ---------------------------------------------------------------- helpers
def size_of(path) -> int:
    """Bytes under `path` (a file or a folder); symbolic links not followed."""
    try:
        if os.path.islink(path):
            return 0
        if os.path.isfile(path):
            return os.path.getsize(path)
    except OSError:
        return 0
    total = 0
    for base, _dirs, files in os.walk(path):
        for f in files:
            p = os.path.join(base, f)
            try:
                if not os.path.islink(p):
                    total += os.path.getsize(p)
            except OSError:
                pass
    return total


def process_paths(proc="/proc") -> list:
    """Every process's command line and working folder (the ones this user may read)."""
    out = []
    for pid in os.listdir(proc):
        if not pid.isdigit():
            continue
        try:
            with open(os.path.join(proc, pid, "cmdline"), "rb") as f:
                out.append(f.read().replace(b"\0", b" ").decode(errors="replace"))
        except OSError:
            pass
        try:
            out.append(os.readlink(os.path.join(proc, pid, "cwd")))
        except OSError:
            pass
    return out


def in_use(path, procs) -> bool:
    path = os.path.normpath(path)
    return any(path in p for p in procs)


def item(category, path, action, why, nbytes=None) -> dict:
    return {"category": category, "path": path, "action": action, "why": why,
            "bytes": size_of(path) if nbytes is None else nbytes}


# ---------------------------------------------------------------- VS Code
def vscode_plan(vs_dir, procs, now=None) -> list:
    """The VS Code server's leftovers: [item] with action "delete" or "keep"."""
    now = time.time() if now is None else now
    plan = []
    servers = os.path.join(vs_dir, "cli", "servers")
    try:
        lru = json.load(open(os.path.join(servers, "lru.json")))
    except (OSError, ValueError):
        lru = []
    newest = lru[0] if lru else None
    if os.path.isdir(servers):
        for name in sorted(os.listdir(servers)):
            path = os.path.join(servers, name)
            if not (name.startswith("Stable-") and os.path.isdir(path)):
                continue
            if in_use(path, procs):
                plan.append(item("vscode server", path, "keep", "running"))
            elif name == newest:
                plan.append(item("vscode server", path, "keep", "the newest (VS Code switches to it next)"))
            else:
                plan.append(item("vscode server", path, "delete", "no process runs it; not the newest"))
    clis = sorted((n for n in os.listdir(vs_dir) if n.startswith("code-")) if os.path.isdir(vs_dir) else [],
                  key=lambda n: os.path.getmtime(os.path.join(vs_dir, n)))
    for i, name in enumerate(clis):
        path = os.path.join(vs_dir, name)
        log = os.path.join(vs_dir, f".cli.{name[5:]}.log")
        if in_use(path, procs):
            plan.append(item("vscode cli", path, "keep", "running"))
        elif i == len(clis) - 1:
            plan.append(item("vscode cli", path, "keep", "the newest"))
        else:
            plan.append(item("vscode cli", path, "delete", "no process runs it; not the newest"))
            if os.path.exists(log):
                plan.append(item("vscode cli", log, "delete", "its log"))
    ext = os.path.join(vs_dir, "extensions")
    try:
        listed = {e.get("relativeLocation") for e in json.load(open(os.path.join(ext, "extensions.json")))}
    except (OSError, ValueError):
        listed = None                                 # unreadable: delete no extension at all
    try:
        obsolete = set(json.load(open(os.path.join(ext, ".obsolete"))))
    except (OSError, ValueError):
        obsolete = set()
    if os.path.isdir(ext) and listed is not None:
        by_id: dict = {}
        for name in sorted(os.listdir(ext)):
            path = os.path.join(ext, name)
            if name.startswith(".") or not os.path.isdir(path):
                continue
            if in_use(path, procs):
                plan.append(item("vscode extension", path, "keep", "in use"))
            elif name in obsolete or name not in listed:
                why = "marked obsolete by VS Code" if name in obsolete else "not listed in extensions.json"
                plan.append(item("vscode extension", path, "delete", why + "; no process uses it"))
            else:
                ident = re.sub(r"-\d[\w.]*(-linux-[a-z0-9]+)?$", "", name)
                by_id.setdefault(ident, []).append(name)
                plan.append(item("vscode extension", path, "keep", "listed in extensions.json"))
        for ident, names in by_id.items():                # several listed versions: say so, delete none
            if len(names) > 1:
                for p in plan:
                    if os.path.basename(p["path"]) in names[:-1] and p["action"] == "keep":
                        p["why"] = "listed, but a newer version is too (VS Code's to remove)"
    cache = os.path.join(vs_dir, "data", "CachedExtensionVSIXs")
    if os.path.isdir(cache):
        for name in os.listdir(cache):
            plan.append(item("vscode installer cache", os.path.join(cache, name), "delete",
                             "an installer; the extension stays installed"))
    logs = os.path.join(vs_dir, "data", "logs")
    if os.path.isdir(logs):
        for name in os.listdir(logs):
            path = os.path.join(logs, name)
            if now - os.path.getmtime(path) > LOG_AGE_DAYS * 86400:
                plan.append(item("vscode logs", path, "delete", f"older than {LOG_AGE_DAYS} days"))
    return plan


# ---------------------------------------------------------------- /tmp
def our_prefixes(root=ROOT) -> list:
    """The temporary-folder prefixes our code uses (tempfile.mkdtemp(prefix=...) / TemporaryDirectory(prefix=...))."""
    found = set()
    for d in CODE_DIRS:
        folder = os.path.join(root, d)
        if not os.path.isdir(folder):
            continue
        for name in os.listdir(folder):
            if name.endswith(".py"):
                try:
                    found.update(PREFIX_RE.findall(open(os.path.join(folder, name), encoding="utf-8").read()))
                except OSError:
                    pass
    return sorted(found)


def tmp_plan(tmp_dir, prefixes, procs, min_age_h=24.0, now=None, uid=None) -> list:
    """Our leftover temporary folders in `tmp_dir`: named with one of `prefixes`, this user's, older than min_age_h,
    no process inside. Everything else in /tmp is left alone (other Claude sessions' scratchpads are reported)."""
    now = time.time() if now is None else now
    uid = os.getuid() if uid is None else uid
    plan = []
    try:
        names = sorted(os.listdir(tmp_dir))
    except OSError:
        return plan
    for name in names:
        path = os.path.join(tmp_dir, name)
        try:
            st = os.lstat(path)
        except OSError:
            continue
        if not os.path.isdir(path) or os.path.islink(path) or st.st_uid != uid:
            continue
        if any(name.startswith(p) for p in prefixes):
            age_h = (now - st.st_mtime) / 3600.0
            if in_use(path, procs):
                plan.append(item("tmp", path, "keep", "a process works in it"))
            elif age_h < min_age_h:
                plan.append(item("tmp", path, "keep", f"younger than {min_age_h:g} h"))
            else:
                plan.append(item("tmp", path, "delete", "a test's leftover temporary folder"))
    return plan


# ---------------------------------------------------------------- report-only facts
def report_facts(root=ROOT) -> list:
    """(label, text) lines about what this tool never deletes."""
    facts = []
    try:
        from utility.housekeeping import RECORDINGS_BUDGET_BYTES, recording_runs
        from project_paths import LOGS_DIR, TEMP_DATA_DIR
        runs = recording_runs(TEMP_DATA_DIR)
        facts.append(("test recordings", f"{sum(r['bytes'] for r in runs) / 1e6:.0f} MB in {len(runs)} runs, budget "
                      f"{RECORDINGS_BUDGET_BYTES / 1e6:.0f} MB (the supervisor deletes the oldest hourly)"))
        facts.append(("logs/", f"{size_of(str(LOGS_DIR)) / 1e6:.0f} MB (every file rotates at its own cap)"))
    except Exception as exc:                               # a report line must never stop the tool
        facts.append(("robot files", f"not read: {exc}"))
    try:
        out = subprocess.run(["journalctl", "--disk-usage"], capture_output=True, text=True, timeout=10).stdout.strip()
        facts.append(("system journal", out or "unknown"))
    except (OSError, subprocess.SubprocessError):
        pass
    syslog = sum(size_of(os.path.join("/var/log", n)) for n in os.listdir("/var/log") if n.startswith("syslog"))
    facts.append(("syslog", f"{syslog / 1e6:.0f} MB (/var/log/syslog*)"))
    others = os.path.join("/tmp", f"claude-{os.getuid()}")
    if os.path.isdir(others):
        facts.append(("Claude scratchpads", f"{size_of(others) / 1e6:.0f} MB in {others} (other sessions'; not "
                      f"deleted here)"))
    du = shutil.disk_usage("/")
    facts.append(("free", f"{du.free / 1e9:.2f} GB of {du.total / 1e9:.1f} GB"))
    return facts


# ---------------------------------------------------------------- main
def report_lines(plan) -> list:
    """One line per item, except /tmp: one summary line per action (there can be hundreds)."""
    lines = []
    for it in plan:
        if it["category"] != "tmp":
            lines.append(f"  {it['action']:6s} {it['bytes'] / 1e6:8.1f} MB  {it['category']:22s} {it['path']}  — {it['why']}")
    for action in ("delete", "keep"):
        tmp = [it for it in plan if it["category"] == "tmp" and it["action"] == action]
        if tmp:
            why = sorted({it["why"] for it in tmp})
            lines.append(f"  {action:6s} {sum(it['bytes'] for it in tmp) / 1e6:8.1f} MB  {'tmp':22s} {len(tmp)} test "
                         f"temporary folder(s) in /tmp  — {'; '.join(why)}")
    return lines


def apply(plan) -> int:
    """Delete every "delete" item; returns the bytes freed."""
    freed = 0
    for it in plan:
        if it["action"] != "delete":
            continue
        p = it["path"]
        try:
            if os.path.isdir(p) and not os.path.islink(p):
                shutil.rmtree(p)
            else:
                os.unlink(p)
            freed += it["bytes"]
        except OSError as exc:
            it["why"] += f" (NOT deleted: {exc})"
    return freed


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--apply", action="store_true", help="delete what the report marks 'delete'")
    ap.add_argument("--only", choices=("vscode", "tmp"), help="one part only")
    ap.add_argument("--vscode-dir", default=os.path.expanduser("~/.vscode-server"))
    ap.add_argument("--tmp-dir", default="/tmp")
    ap.add_argument("--min-age-h", type=float, default=24.0, help="a /tmp folder must be this old (default 24)")
    args = ap.parse_args(argv)
    procs = process_paths()
    plan = []
    if args.only in (None, "vscode"):
        plan += vscode_plan(args.vscode_dir, procs)
    if args.only in (None, "tmp"):
        plan += tmp_plan(args.tmp_dir, our_prefixes(), procs, args.min_age_h)
    gone = [p for p in plan if p["action"] == "delete"]
    for line in report_lines(plan):
        print(line)
    total = sum(p["bytes"] for p in gone)
    if args.apply:
        freed = apply(plan)
        print(f"\ndeleted {len(gone)} item(s), {freed / 1e6:.0f} MB freed")
    else:
        print(f"\n{len(gone)} item(s), {total / 1e6:.0f} MB can go — run with --apply to delete them")
    print()
    for label, text in report_facts():
        print(f"  {label:20s} {text}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
