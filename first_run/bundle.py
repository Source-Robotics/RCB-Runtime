"""The support bundle in the first-run report (2026-10-27, TODO/first_run_plan.md P4): one zip — this report, the
robot's logs, its incident recordings, its configuration files and info.txt (every part's version, the robot's
state) — what a builder sends when asking for help. The zip is the Logs tab's support bundle (gui_logs_tab.py: the
same files, the same refusals) with the report added, written into the report's folder.

    offer(ctx, ui, folder, mode) -> path | None     mode "ask" (the user is asked), "yes", "no"
    prune(root, keep=KEEP_BUNDLES)                   only the newest `keep` reports keep their zip

Refused, like the Logs tab's button, while the arm is ENABLED — reading ~60 MB slows the real-time loop while the arm
is powered: the user is asked to disable it and the step waits (never disables it itself) — and with under 200 MB of
free disk. A bundle that is not built never fails the run: the report says why, and the Logs tab builds one later.
"""
from __future__ import annotations

import glob
import os
import shutil

from first_run.core import DISABLE_WAIT_S, Aborted, Cancelled
from gui_logs_tab import BUNDLE_KEEP, build_bundle, bundle_refusal, info_text
from project_paths import PROJECT_ROOT

# A bundle is tens of MB and a report a few KB: the report folders stay (first_run.report.KEEP_REPORTS), but only the
# newest few keep their zip — as many as the Logs tab keeps of its own.
KEEP_BUNDLES = BUNDLE_KEEP
REPORT_FILES = ("report.txt", "report.json")


def _state(robot) -> str:
    try:
        return str((robot.get_status() or {}).get("state", "")) if robot is not None else ""
    except Exception:   # a lost connection: no state (the bundle is still built — it is what to send then)
        return ""


def snapshot(robot) -> dict:
    """info.txt's robot part: the state, the mode, homed, the latched errors, every part's version (the robot's own
    get_versions(), the drives' included). Whatever cannot be read is left out."""
    if robot is None:
        return {"robot": "not connected"}
    snap: dict = {}
    try:
        s = robot.get_status() or {}
        snap.update(robot_state=s.get("state"), robot_mode=s.get("mode"), homed=s.get("homed"))
    except Exception as e:
        snap["status"] = f"unavailable ({e})"
    for key, call in (("errors", "errors"), ("versions", "get_versions")):
        try:
            snap[key] = getattr(robot, call)()
        except Exception:   # an older executor, a lost connection: info_text falls back to this Pi's own versions
            pass
    return snap


def prune(root, keep=KEEP_BUNDLES, spare=None):
    """Delete the support zips of all but the newest `keep` report folders (their names are UTC stamps) — never
    `spare`, the zip just built (an older folder's, when the clock went back)."""
    zips = sorted(glob.glob(os.path.join(str(root), "*", "support_*.zip")))
    for p in zips[:-keep] if keep > 0 else zips:
        if spare is not None and os.path.abspath(p) == os.path.abspath(spare):
            continue
        try:
            os.remove(p)
        except OSError:
            pass


def offer(ctx, ui, folder, mode="ask", disk_free=None):
    """Build the bundle into `folder` (the report's) when mode says so; returns its path, or None (and says why)."""
    if mode == "no":
        return None
    if mode == "ask":
        try:
            if not ui.confirm("Add the support bundle to the report? One zip of this report, the robot's logs, its "
                              "incident recordings and configuration files — what to send when asking for help "
                              "(tens of MB)."):
                return None
        except Aborted:
            return None
    robot = ctx.get("robot")
    if _state(robot) == "ENABLED":
        try:
            got = ui.wait("The support bundle reads ~60 MB, which can slow the real-time loop while the arm is "
                          "powered. Support the arm (or park it), then press Disable in the dashboard — waiting",
                          lambda: True if _state(robot) != "ENABLED" else None, DISABLE_WAIT_S, 0.5)
        except Cancelled:
            got = None
        if got is None:
            ui.say("Support bundle not built: the arm stayed enabled (the dashboard's Logs tab builds one later).")
            return None
    free = shutil.disk_usage(str(PROJECT_ROOT)).free if disk_free is None else disk_free
    why = bundle_refusal(_state(robot), free)
    if why:
        ui.say(f"Support bundle not built: {why}")
        return None
    extra = [(os.path.join(folder, n), os.path.join("first_run", n)) for n in REPORT_FILES
             if os.path.isfile(os.path.join(folder, n))]
    try:
        path = build_bundle(info_text(snapshot(robot)), out_dir=folder, keep=1, extra=extra)
    except OSError as e:
        ui.say(f"Support bundle not built: {e}")
        return None
    prune(os.path.dirname(os.path.abspath(folder)), spare=path)
    ui.say(f"Support bundle: {path} ({os.path.getsize(path) / 1e6:.1f} MB)")
    return path
