"""Size and count limits for the files the robot writes outside Python's logging (2026-10-15, docs/logging.md
"Rotation"). Python's own log files rotate in their RotatingFileHandler; these do not:

  * rotate_by_size — a file nobody holds open: <name> -> <name>.1 -> ... -> <name>.<keep>, the oldest dropped, when it
    is over max_bytes. The supervisor's <NAME>.out.log at a (re)start, the Blockly runner's block_programs.log per run.
  * copy_truncate — a file a RUNNING process appends to (the supervisor's <NAME>.out.log: the child holds it open with
    O_APPEND, so it cannot be renamed under it): copy it to <name>.1 (rotating the older ones), then truncate it to
    zero; the child's next write lands at the new end. Lines written between the copy and the truncation are lost
    (milliseconds — acceptable for a file that holds crashes and thread dumps).
  * prune_folders — keep the newest `keep` run folders of a tool (tools/sim_run.py, tools/arm_smoke.py).
  * enforce_budget — the test recordings in temp_data (RECORDING_DIRS: what the api_tests measurement scripts save,
    one file or folder per run) together at most RECORDINGS_BUDGET_BYTES (500 MB, see there): the oldest runs
    deleted first, a run's files together (the same "YYYYMMDD_HHMMSS" name start). The supervisor checks once an
    hour. Only these folders: reference data (calibration, the self-collision fits, motor-tuning baselines, the arm
    table, the firmware cache) is never in the list. dummy_test_scripts/test_disk_limits.py keeps every
    api_test that writes into temp_data either in RECORDING_DIRS or in KEPT_DIRS.
"""
from __future__ import annotations

import os
import re
import shutil
import time

# The folders of temp_data the api_tests measurement scripts write their runs into (recordings, re-creatable by
# running the test again). Nothing else is ever deleted by enforce_budget.
RECORDING_DIRS = ("tracking", "can_gaps", "rti_ff", "exec_brake", "wall_delay", "resonance", "haptic_wall",
                  "hold_noise", "idle_release", "virtual_box", "friction", "j6_sampling", "monitor_baseline",
                  "external_torque",       # api_tests/76: the external torque's capture copies (2026-10-16)
                  "friction_id",           # api_tests/77: friction identification runs (2026-10-16)
                  "weight_calibration",    # api_tests/78: torque scale with hung weights + force check (2026-10-17)
                  "camera_timing")         # api_tests/80: the camera timing calibration's frames + captures (2026-11-24)
# temp_data folders a script writes that are NOT recordings: kept whole (reference data, their own limits, caches).
KEPT_DIRS = {"calibration": "the arms' calibration-screw readings (api_tests/65): reference data",
             "self_collision": "the geometry tool's fits and reports: reference data",
             "motor_tuning": "tuning baselines (tools/tune_motor_loops.py)",
             "quiet_check": "per-arm idle signatures (tools/joint_quiet_check.py): reference data",
             "rt_benchmarks": "the RT loop baseline (tools/benchmark_rt_loop.py)",
             "firmware_soak": "flasher soak results (tools/soak_flash_test.py), a few KB each",
             "firmware": "downloaded firmware releases (the flasher's cache)",
             "smoke": "tools/arm_smoke.py: its own limit (prune_folders, newest 30) + history.tsv",
             "sim": "tools/sim_run.py: its own limit (newest 30 runs) and rotating logs",
             "incidents": "utility/incident_dump.py: newest 20",
             "captures": "utility/incident_dump.py: newest 10",
             "drive_captures": "utility/drive_capture.py: a drive's own recording of its fault, newest 30, ~10 KB each",
             "support": "gui_logs_tab.py: newest 3 bundles",
             "first_run": "tools/first_run.py: the first-run check's reports, a few KB each, newest 50",
             "nets": "tools/run_nets.py: one log per net, overwritten each run",
             "quality": "tools/quality.py: baselines, overwritten",
             "coverage": "tools/quality.py: the last coverage report, overwritten"}
# 500 MB since 2026-10-25 (user: the 15 GB SD card filled up); 1 GB from 2026-10-15. On a Pi with a larger card
# (32 GB or more) set it back to 1_000_000_000 — the only place the number lives (supervisor, disk_cleanup read it).
RECORDINGS_BUDGET_BYTES = 500_000_000
# The SD card nearly full (review I6, 2026-11-20): under DISK_LOW_BYTES free the supervisor raises DISK_LOW_WARNING
# (the health check's WARN line too); it clears above DISK_OK_BYTES (no flicker at the line). The card ran full on
# 2026-10-24 and was at 99 % on 2026-11-20, with nothing saying so unless the health card was open.
DISK_LOW_BYTES = 500_000_000
DISK_OK_BYTES = 550_000_000
# logs/ together (review I6, the user 2026-11-21: "(b)", a total budget like the recordings'): each log rotates on its
# own size (Python's handlers, copy_truncate, rotate_by_size), but nothing bounded the folder — 9 rotating files of up
# to 5 x 10 MB each. Over it, the supervisor deletes the oldest ROTATED copies (<name>.N) first, never a live log.
LOGS_BUDGET_BYTES = 300_000_000
_ROTATED_COPY = re.compile(r"\.log\.\d+$")
_RUN_STAMP = re.compile(r"^(?:[a-z]+_)?(\d{8}_\d{6})")

def _shift(path, keep):
    """<path>.<keep-1> -> .<keep> ... <path>.1 -> .2 (the oldest overwritten)."""
    for i in range(keep - 1, 0, -1):
        older = f"{path}.{i}"
        if os.path.exists(older):
            os.replace(older, f"{path}.{i + 1}")


def rotate_by_size(path, max_bytes, keep=3) -> bool:
    """Rotate `path` when it is over max_bytes; True if it was. Never raises (OSError -> False)."""
    try:
        if not os.path.exists(path) or os.path.getsize(path) <= max_bytes:
            return False
        _shift(path, keep)
        os.replace(path, f"{path}.1")
        return True
    except OSError:
        return False


COPY_RETRY_S = 600.0                     # a copy that failed (a full card) is tried again after this, not every pass
_copy_failed_until: dict = {}


def copy_truncate(path, max_bytes, keep=3, now=None) -> bool:
    """For a file a running process appends to: when over max_bytes, copy it to <path>.1 and truncate it. True if done.

    Review I6 (2026-11-20): the copy is made FIRST (<path>.1.tmp), and only once it is complete are the older copies
    shifted and it renamed to .1 — a copy that fails (a full SD card) changes nothing, and is tried again only after
    COPY_RETRY_S. Before, the shift came first: each failed attempt pushed one more old copy out (.1 - .3 gone within
    6 s on a full card) and retried a 20 MB copy every pass."""
    now = time.monotonic() if now is None else now
    if now < _copy_failed_until.get(path, 0.0):
        return False
    try:
        if not os.path.exists(path) or os.path.getsize(path) <= max_bytes:
            return False
        tmp = f"{path}.1.tmp"
        try:
            shutil.copyfile(path, tmp)
        except OSError:
            _copy_failed_until[path] = now + COPY_RETRY_S
            try:
                os.unlink(tmp)
            except OSError:  # nosec B110 — never made; the copy's failure is what counts
                pass
            return False
        _shift(path, keep)
        os.replace(tmp, f"{path}.1")
        os.truncate(path, 0)
        return True
    except OSError:
        return False


def _size(path) -> int:
    """Bytes of a file, or of every file under a folder; symbolic links are not followed (counted as 0)."""
    try:
        if os.path.islink(path):
            return 0
        if os.path.isfile(path):
            return os.path.getsize(path)
        total = 0
        for base, _dirs, files in os.walk(path):
            for f in files:
                p = os.path.join(base, f)
                if not os.path.islink(p):
                    try:
                        total += os.path.getsize(p)
                    except OSError:
                        pass
        return total
    except OSError:
        return 0


def recording_runs(root, names=RECORDING_DIRS) -> list:
    """[{"key", "paths", "bytes", "mtime"}] — one per run in root/<name> (a run = the entries whose name starts with
    the same "YYYYMMDD_HHMMSS" stamp; an entry without one is a run of its own), oldest first."""
    runs: dict = {}
    for name in names:
        folder = os.path.join(str(root), name)
        try:
            entries = list(os.scandir(folder))
        except OSError:
            continue
        for e in entries:
            if e.is_symlink():
                continue
            m = _RUN_STAMP.match(e.name)
            key = (name, m.group(1) if m else e.name)
            run = runs.setdefault(key, {"key": f"{name}/{key[1]}", "paths": [], "bytes": 0, "mtime": 0.0})
            run["paths"].append(e.path)
            run["bytes"] += _size(e.path)
            try:
                run["mtime"] = max(run["mtime"], os.path.getmtime(e.path))
            except OSError:
                pass
    return sorted(runs.values(), key=lambda r: r["mtime"])


def enforce_budget(root, budget_bytes=RECORDINGS_BUDGET_BYTES, names=RECORDING_DIRS) -> tuple:
    """Delete the oldest recording runs until the recording folders together use at most `budget_bytes`. Returns
    (bytes before, bytes after, [deleted run keys]). Never raises."""
    runs = recording_runs(root, names)
    before = total = sum(r["bytes"] for r in runs)
    deleted = []
    for run in runs:
        if total <= budget_bytes:
            break
        for p in run["paths"]:
            try:
                if os.path.isdir(p) and not os.path.islink(p):
                    shutil.rmtree(p, ignore_errors=True)
                else:
                    os.unlink(p)
            except OSError:
                pass
        total -= run["bytes"]
        deleted.append(run["key"])
    return before, total, deleted


def enforce_logs_budget(logs_dir, budget_bytes=LOGS_BUDGET_BYTES) -> tuple:
    """Delete the oldest rotated log copies (<name>.log.N, by modification time) until `logs_dir`'s files together use
    at most `budget_bytes`. A live log (no .N) is never deleted: when the live ones alone are over, it stops at the
    last copy. Returns (bytes before, bytes after, [deleted file names]). Never raises."""
    try:
        entries = [e for e in os.scandir(logs_dir) if e.is_file(follow_symlinks=False)]
    except OSError:
        return 0, 0, []
    sizes, copies = {}, []
    for e in entries:
        try:
            st = e.stat(follow_symlinks=False)
        except OSError:
            continue
        sizes[e.path] = st.st_size
        if _ROTATED_COPY.search(e.name):
            copies.append((st.st_mtime, e.path, e.name))
    before = total = sum(sizes.values())
    deleted = []
    for _, path, name in sorted(copies):
        if total <= budget_bytes:
            break
        try:
            os.unlink(path)
        except OSError:
            continue
        total -= sizes[path]
        deleted.append(name)
    return before, total, deleted


def prune_folders(parent, keep) -> list:
    """Delete all but the newest `keep` sub-folders of `parent` (newest = latest modification time); return the
    deleted paths. Files in `parent` are left alone."""
    try:
        subs = [os.path.join(parent, d) for d in os.listdir(parent) if os.path.isdir(os.path.join(parent, d))]
    except OSError:
        return []
    subs.sort(key=lambda d: os.path.getmtime(d))
    gone = []
    for d in subs[:-keep] if keep > 0 else subs:
        shutil.rmtree(d, ignore_errors=True)
        gone.append(d)
    return gone
