#!/usr/bin/env python3
"""The real-arm smoke routine (2026-10-13): after a change to RTI (or anything it runs), ONE command runs a fixed, safe
set of api_tests on the arm and saves the results with the commit they ran on.

    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/arm_smoke.py          # asks before anything moves
        --yes          no question (you are at the arm anyway)
        --list         what it runs, then exit
        --only a,b     just these (names from --list; a+b too — inside sim_run.py --tests, whose list splits on ',')
        --suite sim    the SIMULATED suite (refused on the real arm): these tests + 30 more api_tests that move the
                       arm far (blends, Cartesian lines and curves, G-code, pause / stop mid-chain, SOFT_RTI, block
                       programs ...) — run by tools/sim_run.py --tests suite (2026-10-13)

Before: the arm ENABLED, homed, error-free, in IDLE or EXEC (the dashboard); the workspace clear, the E-stop in reach.
What moves: every joint to the api_tests' start pose at 10 % (2026-10-13), then J1 by +1 deg and back, J1 5 deg
stopped mid-move and back, J5 small moves + a refused limit + a jog (api_tests 02 / 12 / 45). Everything else moves nothing (00 / 49 / 42 / 60 / 25).

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
from utility import sim_mode                                              # noqa: E402
sys.path.insert(0, os.path.join(ROOT, "api_tests"))
from _common import START_POSE_DEG                                        # noqa: E402  (the api_tests' one start pose)

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
    # 2026-10-13 (user): every run's motion tests start from the same pose — on the arm 45's J5 approach was refused
    # (SELF_COLLISION, correctly) from where the arm had been left by hand; the simulator starts here after homing
    ("start_pose",     None,                   [],
     "every joint to the api_tests' start pose " + str(START_POSE_DEG) + " deg at 10 %", "EXEC"),
    ("joint_nudge",    "02_joint.py",          ["--go", "--assume-ready", "--speed", "10"], "J1 +1 deg and back", "EXEC"),
    ("stop",           "12_stop.py",           ["--go", "--assume-ready", "--speed", "10"],
     "J1 5 deg, stopped mid-move, back", "EXEC"),
    ("exec_abort",     "45_exec_abort.py",     ["--go"],
     "J5 small moves, a refused limit, a jog", None),
]

# The simulated suite (--suite sim, 2026-10-13, TODO/simulated_robot_plan.md S3): TESTS + these. They move the arm far
# and fast, so they run ONLY against the simulated robot; there each EXEC test starts from the start pose and a test
# that leaves an error or an abort latch behind gets a note, and the next one starts clean.
SIM_EXTRA = [
    ("blend",           "03_blend.py",            ["--go", "--assume-ready"], "a blended joint chain", "EXEC"),
    ("cart_dry",        "04_cart_dry.py",         ["--go", "--assume-ready"], None, "EXEC"),
    ("cart",            "05_cart.py",             ["--go", "--assume-ready", "--cartesian-hardware-ack"],
     "TCP lines in WRF and TRF", "EXEC"),
    ("io",              "06_io.py",               ["--all"], None, None),
    ("gripper",         "06_io.py",               ["--go", "--assume-ready", "--gripper-all"], "the gripper suite", "EXEC"),
    ("errors",          "07_errors.py",           [], None, None),
    ("callbacks",       "08_callbacks.py",        ["--go", "--assume-ready"], "J1", "EXEC"),
    ("completion",      "09_completion.py",       ["--go", "--assume-ready"], "J1 + settle policies", "EXEC"),
    ("state",           "10_state.py",            ["--go", "--disable-cycle", "--disable-hardware-ack"],
     "a disable / re-enable cycle", None),
    ("pause_resume",    "11_pause_resume.py",     ["--go", "--assume-ready"], "J1 paused mid-move", "EXEC"),
    ("pose",            "13_pose.py",             ["--go", "--assume-ready", "--pose-hardware-ack"], "move_pose",
     "EXEC"),
    ("motion_failure",  "16_motion_failure.py",   ["--go", "--assume-ready"], "a burst with a failing move", "EXEC"),
    ("rolling_buffer",  "17_rolling_buffer.py",   ["--go", "--assume-ready"], "commands spliced in flight", "EXEC"),
    ("gcode",           "18_gcode.py",            ["--go", "--assume-ready"], "a G-code toolpath", "EXEC"),
    ("ruckig_corner",   "20_ruckig_corner.py",    ["--go", "--assume-ready"], "jerk-limited corners", "EXEC"),
    ("blend_interrupt", "21_blend_interrupt.py",  ["--go", "--assume-ready"], "pause / stop mid-chain", "EXEC"),
    ("cart_curves",     "22_cartesian_curves.py", ["--go", "--assume-ready", "--curve-hardware-ack"],
     "arcs and curves", "EXEC"),
    ("dry_run_preview", "23_dry_run_preview.py",  [], None, None),
    ("soft_rti",        "27_soft_rti.py",         ["--move", "--seconds", "5"], "J1 streamed", "IDLE"),
    ("soft_rti_safety", "34_soft_rti_safety.py",  ["--go"], None, "IDLE"),
    ("block_program",   "57_block_program.py",    ["--go"], "J5 block programs", None),
    # 2026-10-13, the second pass (user: "item 7"): zone blending, multi-joint and TCP streaming, a read-only glitch
    # hunt, a block Check's load, the profile demo; and every setting command once (72). Not 26 (pick and place):
    # it asks the operator to place a part (input()), so it cannot run unattended.
    ("zone_blend",      "14_zone_blend.py",       ["--go", "--assume-ready"], "a zone-blended burst", "EXEC"),
    ("multi_joint",     "32_multi_joint.py",      ["--go", "--freqs", "0.5", "--rates", "25", "--seconds", "3"],
     "J1 / J4 / J5 / J6 streamed together (SOFT_RTI)", "IDLE"),
    ("transport",       "41_transport.py",        ["--go", "--seconds", "6"], "SOFT_RTI over UDP, then TCP", "IDLE"),
    ("glitch_hunt",     "43_glitch_hunt.py",      ["--seconds", "10"], None, None),
    ("check_load",      "58_block_check_load.py", ["--enabled-ok", "--checks", "1", "--baseline", "2"], None, None),
    ("profile_demo",    "67_profile_demo.py",     ["--go", "--auto", "--steps", "0,1"],
     "the default profile, then ruckig vs trapezoid", "EXEC"),
    ("command_sweep",   "72_command_sweep.py",    [], None, "IDLE"),
    ("rti_brake",       "71_rti_brake.py",        ["--go"], "J1 at 60 deg/s, braked three ways (hard RTI)", "IDLE"),
    ("exec_brake",      "73_exec_brake.py",       ["--go"], "J1 + J5 at speed, paused and stopped along the path (EXEC)",
     "EXEC"),
]
SIM_POSE_SPEED = 30          # the simulated suite's moves to the start pose (%)
NO_HOST = {"23_dry_run_preview.py", "43_glitch_hunt.py"}     # offline / shared-memory-only tests: no --host


def _to_start_pose(robot, speed):
    """Every joint to START_POSE_DEG; (0, "reached") or (1, why)."""
    try:
        robot.move_joints(*START_POSE_DEG, speed=speed)
        f = robot.wait_idle(timeout=TIMEOUT_S, raise_on_error=False)
        return (0, "reached") if not f else (1, f"FAIL  {f}")
    except Exception as e:                          # noqa: BLE001 — a refused or failed move fails the step
        return 1, f"FAIL  {type(e).__name__}: {e}"


def _wait_executor_idle(robot, timeout=30.0):
    """Until the executor runs nothing: a test can exit with a queued move still in flight (its failure — and the abort
    latch — then land after the test; 16_motion_failure's last move did, 2026-10-13)."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        s = _status(robot)
        if s.get("active_command") is None and s.get("executing_seq") == s.get("completed_seq") \
                and not s.get("queue_depth"):
            return True
        time.sleep(0.2)
    return False


def _client():
    from robotclient.client import RobotClient
    r = RobotClient(sync=True)
    r.connect("127.0.0.1", 29000)
    return r


def _status(robot):
    s = robot.get_status()
    if not isinstance(s, dict):
        return {}
    # get_status nests the planner's fields — motion_aborted, executing / completed seq, the active command, the queue
    # depth — under "planner" (the event stream's status() has them at the top). Until 2026-10-13 this read the top
    # level only, so the stale-abort-latch check never saw a latch (found by the simulated suite).
    return {**(s.get("planner") or {}), **s}


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
    ap.add_argument("--suite", default="arm", choices=("arm", "sim"),
                    help="arm = the smoke routine; sim = + 30 api_tests, the SIMULATED robot only")
    args = ap.parse_args()

    simulated_suite = args.suite == "sim"
    if simulated_suite and not sim_mode.enabled():
        print("REFUSED: --suite sim moves the arm far and fast — it runs against the SIMULATED robot only "
              "(tools/sim_run.py --tests suite)")
        return 2
    all_tests = TESTS + (SIM_EXTRA if simulated_suite else [])
    tests = all_tests
    if args.only:
        want = {w.strip() for w in args.only.replace("+", ",").split(",") if w.strip()}   # '+' too: sim_run --tests splits on ','
        unknown = want - {t[0] for t in all_tests}
        if unknown:
            print(f"unknown test(s): {sorted(unknown)} — see --list")
            return 2
        tests = [t for t in all_tests if t[0] in want]
    if args.list:
        for name, script, extra, moves, _ in all_tests:
            print(f"  {name:15s} {('api_tests/' + script) if script else '(this tool)':32s} {' '.join(extra):34s} "
                  f"{moves or 'nothing moves'}")
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
    if s.get("motion_aborted"):
        # a previous program's abort latch (e.g. a refused move of an earlier smoke run): motion stays refused until
        # cleared — a fresh run clears it (real errors refused above), and says so
        print(f"note: a previous program's abort latch is set ({s['motion_aborted'].get('code')}) — clearing it")
        robot.clear_errors(wait=True)
        time.sleep(0.3)

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
        if simulated_suite and results:
            _wait_executor_idle(robot)
            time.sleep(0.3)                                 # a failure event right at the end reaches the latch
        s = _status(robot)
        if simulated_suite and results and (s.get("err") or s.get("motion_aborted") or s.get("state") != "ENABLED"):
            # the previous test left the robot unusable: say so on ITS result, start the next one clean (simulation)
            left = (f"left behind: err={s.get('err')} motion_aborted={bool(s.get('motion_aborted'))} "
                    f"state={s.get('state')} — cleared for the next test")
            results[-1]["note"] = (results[-1]["note"] + "; " if results[-1]["note"] else "") + left
            print(f"    note: {left}", flush=True)
            robot.clear_errors(wait=False)
            time.sleep(0.5)
            robot.enable(wait=True)
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
        if script is None:                                  # the start pose: this tool moves the arm itself
            print(f"\n--- {name}: move_joints{tuple(START_POSE_DEG)} at 10 %", flush=True)
            t0 = time.monotonic()
            code, text = _to_start_pose(robot, 10)
            try:
                robot.release_control()
            except Exception:                               # noqa: BLE001
                pass
            dt = time.monotonic() - t0
            with open(os.path.join(run_dir, f"{name}.log"), "w") as fh:
                fh.write(text + "\n")
            ok = code == 0
            results.append({"name": name, "script": None, "args": [], "exit": code, "ok": ok, "seconds": round(dt, 1),
                            "note": "" if ok else text[:160]})
            print(f"    {'PASS' if ok else 'FAIL'}  {dt:6.1f} s" + ("" if ok else f"   {text[:150]}"), flush=True)
            if not ok:
                aborted = "the start pose was not reached — the motion tests were not run"
                break
            continue
        if simulated_suite and mode == "EXEC" and name not in {t[0] for t in TESTS}:
            pc, ptext = _to_start_pose(robot, SIM_POSE_SPEED)       # each simulated EXEC test from the same pose
            try:
                robot.release_control()
            except Exception:                               # noqa: BLE001
                pass
            if pc:
                print(f"\n--- {name}: the start pose was not reached ({ptext[:120]}) — the test runs from here", flush=True)
        cmd = [PY, os.path.join("api_tests", script)] + ([] if script in NO_HOST else ["--host", "127.0.0.1"]) + extra
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
