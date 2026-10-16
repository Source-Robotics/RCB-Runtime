#!/usr/bin/env python3
"""A simulated run (TODO/simulated_robot_plan.md, S1 2026-10-13): the simulated drives on vcan0 + the REAL runtime
(RTI, the executor, the self-collision worker, the GUI with --gui) in simulation mode (RCB_SIM=1), on this Pi with the robot service stopped.

    sudo systemctl stop autorun.service          # first — the arm goes limp: park or support it
    sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up    # once per boot
    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/sim_run.py                # boots, reports, waits for Ctrl-C
        --seconds N      stop by itself after N s
        --gui            also the dashboard (port 8081; it shows the SIMULATED robot)
        --faults all|a,b fault injection after the boot (sim/faults.py): node_lost, drive_fault, drive_flags,
                         reply_loss, drive_reboot, hand_push, push_through, runaway, tracking_lag, hold_buzz (the
                         S6 disturbances), homing_false_stall (J2 blocked while homing), executor_lost (homes
                         first), estop (the simulated E-stop button), rti_lost (ends the run)
        --tests a,b,..   after the boot, run these api_tests in order — the file name without .py, plus its
                         arguments: --tests "00_status,01_mode --go,02_joint --go --assume-ready"
                         'smoke' = tools/arm_smoke.py against the simulated robot (home it first):
                         --tests "01_mode --go,smoke"
                         'suite' = the SIMULATED suite: the smoke routine + 30 more api_tests that move the arm far
                         (tools/arm_smoke.py --suite sim): --tests "01_mode --go,suite"
        --coverage       measure the runtime (RTI, the executor, the worker, the drives, the GUI) with coverage: run
                         under the venv's Python, data + report in temp_data/sim/coverage/ and, with the nets' last
                         coverage run, their union in temp_data/sim/coverage/union/ (docs/testing.md). A process
                         killed with SIGKILL (the executor_lost / rti_lost faults) loses its data: run those without it.
    sudo systemctl start autorun.service         # afterwards; then re-home the arm

Everything the runtime writes goes under temp_data/sim/ (project_paths.SIMULATION); each run's process output under
temp_data/sim/runs/<UTC time>/. It starts and stops only its own processes. Exit 0 = booted (and the tests passed),
1 = boot or a test failed, 2 = refused.
"""
from __future__ import annotations

import argparse
import os
import shlex
import socket
import signal
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from utility import sim_mode                                              # noqa: E402
import quality                                                            # noqa: E402  tools/quality.py

SIM_DIR = os.path.join(ROOT, "temp_data", "sim")
SIM_COVDIR = os.path.join(SIM_DIR, "coverage")
NETS_COVERAGE = os.path.join(quality.COVDIR, ".coverage")      # the nets' combined data (run_nets.py --coverage)
SIM_RTI_LOG = os.path.join(SIM_DIR, "logs", "RTI.log")
SHM_MODE = "/dev/shm/robot_mode"
BOOT_TIMEOUT_S = 40.0
VENV_PY = os.path.join(ROOT, ".venv", "bin", "python")
EXECUTOR_PORT = 29000
ARGV = {"drives": ["python3", "-m", "sim.drives"], "RTI": ["python3", "RTI.py"],
        "executor": ["python3", "command_executor.py"], "worker": ["python3", "self_collision_worker.py"],
        "recorder": ["python3", "recorder.py"], "GUI": ["python3", "nicegui_test.py"]}


def _argv(name, coverage=False):
    """The process's command line; with coverage the same program under the venv's Python and `coverage run`."""
    argv = ARGV[name]
    if not coverage:
        return argv
    head = [VENV_PY, "-m", "coverage", "run", f"--rcfile={quality.RCFILE}"]
    return head + (["-m", argv[2]] if argv[1] == "-m" else argv[1:])


def _service_active(unit="autorun.service"):
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", unit], timeout=10).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def _port_open(port, host="127.0.0.1"):
    try:
        with socket.create_connection((host, port), timeout=0.5):
            return True
    except OSError:
        return False


def _read_shm_string(path):
    """A SharedString segment read as a FILE (never attached): 4-byte length + text."""
    try:
        with open(path, "rb") as fh:
            raw = fh.read()
        n = int.from_bytes(raw[:4], "little")
        return raw[4:4 + n].decode(errors="replace") if 0 <= n <= len(raw) - 4 else ""
    except OSError:
        return ""


def _log_lines_since(path, offset):
    try:
        with open(path, errors="replace") as fh:
            fh.seek(offset)
            return fh.read().splitlines()
    except OSError:
        return []


# Simulation has no real-time core: every process shares the general cores. RTI and the simulated drives run at
# normal priority; the dashboard, the executor and the collision worker are niced, so a busy dashboard cannot starve
# the drives — it did once (2026-10-13: every node went silent in the same ms mid-homing, a CAN_LOST on all of them).
NICE = {"GUI": 10, "executor": 5, "worker": 5, "recorder": 5}


def _start(name, argv, run_dir, env):
    out = open(os.path.join(run_dir, f"{name}.out.log"), "w")
    nice = NICE.get(name, 0)
    p = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=subprocess.STDOUT, start_new_session=True,
                         preexec_fn=(lambda: os.nice(nice)) if nice else None)
    print(f"  started {name} (pid {p.pid})", flush=True)
    return p, out


def _stop(name, proc, timeout=15.0):
    if proc.poll() is not None:
        return proc.returncode
    proc.send_signal(signal.SIGTERM)
    try:
        return proc.wait(timeout)
    except subprocess.TimeoutExpired:
        print(f"  {name} did not stop in {timeout:.0f} s — killing it", flush=True)
        proc.kill()
        return proc.wait(5)


def main():
    ap = argparse.ArgumentParser(description="a simulated run: fake drives on vcan0 + the real runtime")
    ap.add_argument("--seconds", type=float, default=0.0, help="stop after this long (0 = wait for Ctrl-C)")
    ap.add_argument("--gui", action="store_true", help="also start the dashboard")
    ap.add_argument("--tests", default="", help="api_tests to run after the boot, e.g. 00_status,49_is_reachable")
    ap.add_argument("--faults", default="", help="fault scenarios after the boot (and the tests): all, or names")
    ap.add_argument("--coverage", action="store_true", help="measure the runtime with coverage (temp_data/sim/coverage)")
    args = ap.parse_args()

    if _service_active():
        print("REFUSED: the robot service is running. Stop it first: sudo systemctl stop autorun.service "
              "(the arm goes limp: park or support it).")
        return 2
    others = sim_mode.other_rti_pids()
    if others:
        print(f"REFUSED: an RTI is running (pid {others}).")
        return 2
    left = sim_mode.other_runtime_pids()
    if left:
        print("REFUSED: the runtime's programs are still running from an earlier run (they share the robot's shared "
              "memory and the simulator's port): " + ", ".join(f"{what} (pid {pid})" for pid, what in left.items())
              + f". Stop them: kill {' '.join(str(p) for p in left)}")
        return 2
    if not sim_mode.interface_up(sim_mode.SIM_CAN_INTERFACE):
        print(f"REFUSED: {sim_mode.SIM_CAN_INTERFACE} is not up — {sim_mode.VCAN_HELP}")
        return 2

    run_dir = os.path.join(SIM_DIR, "runs", time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
    os.makedirs(run_dir, exist_ok=True)
    from utility.housekeeping import prune_folders
    prune_folders(os.path.join(SIM_DIR, "runs"), 30)      # the newest 30 runs (2026-10-15)
    env = dict(os.environ, RCB_SIM="1", PYTHONUNBUFFERED="1")
    selfcov = None
    if args.coverage:
        quality.coverage_prepare(SIM_COVDIR)
        env.update(COVERAGE_FILE=os.path.join(SIM_COVDIR, ".coverage"), RCB_ROOT=ROOT,
                   COVERAGE_PROCESS_START=quality.RCFILE)          # the venv children (a block program) measure too
        print("COVERAGE: the runtime runs under the venv's Python + coverage (slower; timing is not what is tested)")
        import coverage
        selfcov = coverage.Coverage(config_file=quality.RCFILE, data_file=os.path.join(SIM_COVDIR, ".coverage"))
        selfcov.start()                    # this process too: the fault scenarios (sim/faults.py) run in it
    log_offset = os.path.getsize(SIM_RTI_LOG) if os.path.exists(SIM_RTI_LOG) else 0
    print(f"SIMULATED RUN — the arm is not involved. Output: {os.path.relpath(run_dir, ROOT)}", flush=True)
    procs = []
    ok = True
    try:
        procs.append(("drives", *_start("drives", _argv("drives", args.coverage), run_dir, env)))
        time.sleep(1.0)
        for name in ("RTI", "executor", "worker", "recorder") + (("GUI",) if args.gui else ()):
            procs.append((name, *_start(name, _argv(name, args.coverage), run_dir, env)))

        t0 = time.monotonic()
        mode = ""
        while time.monotonic() - t0 < BOOT_TIMEOUT_S:
            dead = [n for n, p, _ in procs if p.poll() is not None]
            if dead:
                print(f"FAILED: {', '.join(dead)} exited during the boot — see {os.path.relpath(run_dir, ROOT)}")
                ok = False
                break
            mode = _read_shm_string(SHM_MODE)
            lines = _log_lines_since(SIM_RTI_LOG, log_offset)
            if mode == "IDLE" and any("| rti.selfcheck" in ln and ("PASSED" in ln or "FAILED" in ln) for ln in lines) \
                    and _port_open(EXECUTOR_PORT):          # the executor loads its models for several seconds
                break
            time.sleep(0.5)
        lines = _log_lines_since(SIM_RTI_LOG, log_offset)
        keys = ("SIMULATION", "| code:", "CAN scan", "| ✓", "| ✗", "FAILED", "ERROR", "CRITICAL",
                "Kt =", "start-up clear", "drive watchdog")
        print(f"\nboot: robot_mode={mode or '?'}, executor {'up' if _port_open(EXECUTOR_PORT) else 'NOT up'}, after "
              f"{time.monotonic() - t0:.1f} s; RTI.log (simulated):")
        for ln in lines:
            if any(k in ln for k in keys):
                print("  " + ln[:200])
        if ok and mode != "IDLE":
            print("FAILED: RTI did not reach IDLE")
            ok = False
        if ok and not any("| rti.selfcheck" in ln and "| ✓ PASSED" in ln for ln in lines):
            print("FAILED: the start-up self-check did not pass")
            ok = False
        booted = ok                  # a failed TEST does not stop the fault scenarios (2026-10-13): report both

        if ok and args.tests:
            for spec in [x.strip() for x in args.tests.split(",") if x.strip()]:
                name, *extra = shlex.split(spec)
                limit = 900
                if name == "smoke":          # the real-arm smoke routine, against the simulated robot
                    script, argv = "tools/arm_smoke.py", [VENV_PY, "tools/arm_smoke.py", "--yes"] + extra
                elif name == "suite":        # the smoke routine + the api_tests that run only against the simulator
                    script, argv = "tools/arm_smoke.py", [VENV_PY, "tools/arm_smoke.py", "--yes", "--suite", "sim"] + extra
                    limit = 3600
                else:
                    script = os.path.join("api_tests", name if name.endswith(".py") else name + ".py")
                    argv = [VENV_PY, script, "--host", "127.0.0.1"] + extra
                print(f"\n--- {script} {' '.join(extra)}", flush=True)
                with open(os.path.join(run_dir, f"test_{os.path.basename(script)}.log"), "w") as fh:
                    r = subprocess.run(argv, cwd=ROOT, env=env, stdout=fh, stderr=subprocess.STDOUT, timeout=limit)
                print(f"    {'PASS' if r.returncode == 0 else 'FAIL'} (exit {r.returncode})", flush=True)
                ok = ok and r.returncode == 0
        if booted and args.faults:
            from sim import faults as fl

            def _kill(name, sig):
                for n, p, _ in procs:
                    if n == name and p.poll() is None:          # only this run's own process, by its handle
                        p.send_signal(sig)
                        p.wait(10)

            def _restart(name):
                for k, (n, p, out) in enumerate(procs):
                    if n == name:
                        out.close()
                        procs[k] = (name, *_start(name, _argv(name, args.coverage), run_dir, env))

            def _wait_executor(timeout):
                t1 = time.monotonic()
                while time.monotonic() - t1 < timeout:
                    if _port_open(EXECUTOR_PORT):
                        return True
                    time.sleep(0.5)
                return False

            ctx = fl.Context(ROOT, _kill, _restart, _wait_executor)
            names = list(fl.ORDER) if args.faults.strip() == "all" else \
                [x.strip() for x in args.faults.split(",") if x.strip()]
            names = sorted(names, key=lambda x: fl.ORDER.index(x) if x in fl.ORDER else 99)
            for name in names:
                if name not in fl.SCENARIOS:
                    print(f"\n--- fault {name}: unknown (one of {', '.join(fl.ORDER)})")
                    ok = False
                    continue
                print(f"\n--- fault {name}: {fl.SCENARIOS[name].__doc__.split(chr(10))[0]}", flush=True)
                notes = []
                t1 = time.monotonic()
                try:
                    good = fl.SCENARIOS[name](ctx, notes)
                except Exception as e:                       # noqa: BLE001 — a scenario that crashes failed
                    good = False
                    notes.append(f"crashed: {type(e).__name__}: {e}")
                for ln in notes:
                    print("    " + ln, flush=True)
                print(f"    {'PASS' if good else 'FAIL'} ({time.monotonic() - t1:.1f} s)", flush=True)
                ok = ok and good
        elif booted and not args.tests and not args.faults:
            print("\nThe simulated robot is up" + (f" for {args.seconds:.0f} s" if args.seconds else
                                                   " — Ctrl-C to stop") + ".", flush=True)
            end = time.monotonic() + args.seconds if args.seconds else None
            while end is None or time.monotonic() < end:
                if any(p.poll() is not None for _, p, _ in procs):
                    print("a process exited — stopping")
                    ok = False
                    break
                time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nCtrl-C — stopping")
    finally:
        # the clients first, then RTI, then what RTI watches (the worker: a worker stopped under a live RTI in EXEC
        # raises SELF_COLLISION_STALE), the simulated drives last
        order = {"GUI": 0, "executor": 1, "recorder": 1, "RTI": 2, "worker": 3, "drives": 4}
        for name, p, out in sorted(procs, key=lambda t: order.get(t[0], 9)):
            code = _stop(name, p)
            out.close()
            print(f"  stopped {name} (exit {code})", flush=True)
    if args.coverage:
        if selfcov is not None:
            selfcov.stop()
            selfcov.save()
        rep = quality.coverage_report(SIM_COVDIR, union_with=NETS_COVERAGE)
        f = rep["files"]
        print(f"\ncoverage of this simulated run: {rep['total']} % (lines + branches); RTI.py {f.get('RTI.py')} %, "
              f"command_executor.py {f.get('command_executor.py')} %, self_collision_worker.py "
              f"{f.get('self_collision_worker.py')} % — temp_data/sim/coverage/html/index.html")
        if rep["union_total"] is not None:
            print(f"the nets AND this run together: {rep['union_total']} % — temp_data/sim/coverage/union/html/index.html")
    print("RESULT:", "OK" if ok else "FAILED", f"— output in {os.path.relpath(run_dir, ROOT)}")
    print("Afterwards: sudo systemctl start autorun.service, then re-home the arm.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
