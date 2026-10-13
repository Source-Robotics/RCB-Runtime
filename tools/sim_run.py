#!/usr/bin/env python3
"""A simulated run (TODO/simulated_robot_plan.md, S1 2026-10-13): the simulated drives on vcan0 + the REAL runtime
(RTI, the executor, the self-collision worker, the GUI with --gui) in simulation mode (RCB_SIM=1), on this Pi with the robot service stopped.

    sudo systemctl stop autorun.service          # first — the arm goes limp: park or support it
    sudo modprobe vcan && sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up    # once per boot
    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/sim_run.py                # boots, reports, waits for Ctrl-C
        --seconds N      stop by itself after N s
        --gui            also the dashboard (port 8081; it shows the SIMULATED robot)
        --tests a,b,..   after the boot, run these api_tests in order — the file name without .py, plus its
                         arguments: --tests "00_status,01_mode --go,02_joint --go --assume-ready"
                         'smoke' = tools/arm_smoke.py against the simulated robot (home it first):
                         --tests "01_mode --go,smoke"
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

SIM_DIR = os.path.join(ROOT, "temp_data", "sim")
SIM_RTI_LOG = os.path.join(SIM_DIR, "logs", "RTI.log")
SHM_MODE = "/dev/shm/robot_mode"
BOOT_TIMEOUT_S = 40.0
VENV_PY = os.path.join(ROOT, ".venv", "bin", "python")
EXECUTOR_PORT = 29000


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


def _start(name, argv, run_dir, env):
    out = open(os.path.join(run_dir, f"{name}.out.log"), "w")
    p = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=out, stderr=subprocess.STDOUT, start_new_session=True)
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
    args = ap.parse_args()

    if _service_active():
        print("REFUSED: the robot service is running. Stop it first: sudo systemctl stop autorun.service "
              "(the arm goes limp: park or support it).")
        return 2
    others = sim_mode.other_rti_pids()
    if others:
        print(f"REFUSED: an RTI is running (pid {others}).")
        return 2
    if not sim_mode.interface_up(sim_mode.SIM_CAN_INTERFACE):
        print(f"REFUSED: {sim_mode.SIM_CAN_INTERFACE} is not up — {sim_mode.VCAN_HELP}")
        return 2

    run_dir = os.path.join(SIM_DIR, "runs", time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
    os.makedirs(run_dir, exist_ok=True)
    env = dict(os.environ, RCB_SIM="1", PYTHONUNBUFFERED="1")
    log_offset = os.path.getsize(SIM_RTI_LOG) if os.path.exists(SIM_RTI_LOG) else 0
    print(f"SIMULATED RUN — the arm is not involved. Output: {os.path.relpath(run_dir, ROOT)}", flush=True)
    procs = []
    ok = True
    try:
        procs.append(("drives", *_start("drives", ["python3", "-m", "sim.drives"], run_dir, env)))
        time.sleep(1.0)
        procs.append(("RTI", *_start("RTI", ["python3", "RTI.py"], run_dir, env)))
        procs.append(("executor", *_start("executor", ["python3", "command_executor.py"], run_dir, env)))
        procs.append(("worker", *_start("worker", ["python3", "self_collision_worker.py"], run_dir, env)))
        if args.gui:
            procs.append(("GUI", *_start("GUI", ["python3", "nicegui_test.py"], run_dir, env)))

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
            if mode == "IDLE" and any("[Selfcheck]" in ln and ("PASSED" in ln or "FAILED" in ln) for ln in lines) \
                    and _port_open(EXECUTOR_PORT):          # the executor loads its models for several seconds
                break
            time.sleep(0.5)
        lines = _log_lines_since(SIM_RTI_LOG, log_offset)
        keys = ("[SIM]", "[RTI] code:", "CAN scan", "[Selfcheck] ✓", "[Selfcheck] ✗", "FAILED", "ERROR", "CRITICAL",
                "Kt =", "start-up clear", "drive watchdog")
        print(f"\nboot: robot_mode={mode or '?'}, executor {'up' if _port_open(EXECUTOR_PORT) else 'NOT up'}, after "
              f"{time.monotonic() - t0:.1f} s; RTI.log (simulated):")
        for ln in lines:
            if any(k in ln for k in keys):
                print("  " + ln[:200])
        if ok and mode != "IDLE":
            print("FAILED: RTI did not reach IDLE")
            ok = False
        if ok and not any("[Selfcheck] ✓ PASSED" in ln for ln in lines):
            print("FAILED: the start-up self-check did not pass")
            ok = False

        if ok and args.tests:
            for spec in [x.strip() for x in args.tests.split(",") if x.strip()]:
                name, *extra = shlex.split(spec)
                if name == "smoke":          # the real-arm smoke routine, against the simulated robot
                    script, argv = "tools/arm_smoke.py", [VENV_PY, "tools/arm_smoke.py", "--yes"] + extra
                else:
                    script = os.path.join("api_tests", name if name.endswith(".py") else name + ".py")
                    argv = [VENV_PY, script, "--host", "127.0.0.1"] + extra
                print(f"\n--- {script} {' '.join(extra)}", flush=True)
                with open(os.path.join(run_dir, f"test_{os.path.basename(script)}.log"), "w") as fh:
                    r = subprocess.run(argv, cwd=ROOT, env=env, stdout=fh, stderr=subprocess.STDOUT, timeout=900)
                print(f"    {'PASS' if r.returncode == 0 else 'FAIL'} (exit {r.returncode})", flush=True)
                ok = ok and r.returncode == 0
        elif ok:
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
        order = {"GUI": 0, "executor": 1, "RTI": 2, "worker": 3, "drives": 4}
        for name, p, out in sorted(procs, key=lambda t: order.get(t[0], 9)):
            code = _stop(name, p)
            out.close()
            print(f"  stopped {name} (exit {code})", flush=True)
    print("RESULT:", "OK" if ok else "FAILED", f"— output in {os.path.relpath(run_dir, ROOT)}")
    print("Afterwards: sudo systemctl start autorun.service, then re-home the arm.")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
