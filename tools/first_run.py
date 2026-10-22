#!/usr/bin/env python3
"""The first-run check from the command line (2026-10-18, TODO/first_run_plan.md P1): a guided verification that a
robot was built and wired correctly. Stages 0-2 move nothing; stage 3 moves each joint a few degrees.

    S0  the Pi and the configuration     the Pi's setup, the robot / gripper XMLs, the PCB version, the robot service
    S1  power and CAN                    every drive answers, the CAN bus, firmware, supply voltage, temperatures, faults
    S2  E-stop, buttons, I/O, Hall       you press the E-stop and the box's buttons, optionally switch the isolated
                                         I/O, and turn the Hall-sensor joint by hand
    S3  small test moves                 the arm ENABLED, homed or not: each joint moves a few degrees and back,
                                         slowly, its force limited; you say which way it went (--by-hand: you turn
                                         each joint yourself instead, one at a time limp)

    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/first_run.py                 # every stage
    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/first_run.py --stages 3
    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/first_run.py --list          # the steps

Run it on the robot's Pi (stage 0 reads its files) with the arm DISABLED for stage 2 and ENABLED for stage 3 (each
waits for it; nothing is homed). Ctrl-C while it
waits for you skips that step; Ctrl-C at a question stops the run (the report keeps what was done). The report:
temp_data/first_run/<UTC>/report.txt + report.json. Exit 0 = no FAIL, 1 = a FAIL, 2 = stopped early.
"""
from __future__ import annotations

import argparse
import os
import socket
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from first_run import core, report, stage0                               # noqa: E402
from first_run.steps import BY_HAND, STEPS                                # noqa: E402
from utility.code_version import describe as describe_code                # noqa: E402

_COLOURS = {core.PASS: "32", core.WARN: "33", core.FAIL: "31", core.SKIP: "90", core.INFO: "36"}


class ConsoleUI:
    """The UI contract of first_run.core on a terminal: print + input()."""

    def __init__(self, out=sys.stdout, ask=input, monotonic=time.monotonic, sleep=time.sleep, colour=None):
        self.out, self.ask, self.monotonic, self.sleep = out, ask, monotonic, sleep
        self.colour = out.isatty() if colour is None else colour

    def _p(self, text=""):
        print(text, file=self.out, flush=True)

    def _c(self, code, text):
        return f"\033[{code}m{text}\033[0m" if self.colour else text

    def stage(self, number, title, about):
        self._p()
        self._p(self._c("1", f"=== Stage {number}: {title} ==="))
        if about:
            self._p(f"    {about}")

    def say(self, text):
        self._p(f"    {text}")

    def _input(self, prompt):
        try:
            return str(self.ask(prompt)).strip().lower()
        except (KeyboardInterrupt, EOFError):
            self._p()
            raise core.Aborted() from None

    def confirm(self, question) -> bool:
        while True:
            a = self._input(f"  ? {question} [y/n] ")
            if a in ("y", "yes"):
                return True
            if a in ("n", "no"):
                return False

    def choose(self, question, options) -> str:
        menu = "  ".join(f"[{i + 1}] {o}" for i, o in enumerate(options))
        while True:
            a = self._input(f"  ? {question}\n    {menu}: ")
            if a.isdigit() and 1 <= int(a) <= len(options):
                return options[int(a) - 1]
            for o in options:
                if a == o.lower():
                    return o

    def wait(self, prompt, poll, timeout_s, interval_s=0.1):
        self._p(f"  > {prompt}  (Ctrl-C skips)")
        end = self.monotonic() + float(timeout_s)
        shown = None
        try:
            while True:
                value = poll()
                if value is not None:
                    if shown is not None:
                        self.out.write("\r" + " " * 24 + "\r")
                    return value
                left = end - self.monotonic()
                if left <= 0:
                    self._p("\r    timed out" + " " * 12)
                    return None
                if int(left) != shown:
                    shown = int(left)
                    self.out.write(f"\r    waiting ... {shown:3d} s ")
                    self.out.flush()
                self.sleep(interval_s)
        except KeyboardInterrupt:
            self._p("\r    skipped" + " " * 16)
            raise core.Cancelled() from None

    def result(self, res):
        v = res["verdict"]
        self._p(f"  {self._c(_COLOURS.get(v, '0'), f'{v:<4}')}  {res['title']}: {res['summary']}")
        if v in (core.WARN, core.FAIL):
            for h in res.get("hints", []):
                self._p(f"        check: {h}")


def build_ctx(host):
    from robotclient import RobotClient

    def connect():
        return RobotClient(name="first_run").connect(host)
    return {"connect": connect, "sleep": time.sleep, "monotonic": time.monotonic, "run": subprocess.run}


def select_steps(stages, by_hand=False):
    """The steps to run. Without stage 0 its config + service steps still run first (the later stages need them);
    the setup and PCB questions do not. by_hand: stage 3 = the by-hand check instead of the powered test moves."""
    base = [s for s in STEPS if s["stage"] != 3] + (BY_HAND if by_hand else [s for s in STEPS if s["stage"] == 3])
    steps = [s for s in base if s["stage"] in stages]
    if 0 not in stages:
        steps = [dict(stage0.CONFIG), dict(stage0.SERVICE)] + steps
        for s in steps[:2]:
            s["stage"] = min(stages)
    return steps


def meta_of(ctx, completed, t):
    cfg = ctx.get("config") or {}
    return {"time": t, "completed": completed, "host": socket.gethostname(), "code": describe_code(ROOT),
            "robot": cfg.get("robot"), "gripper": cfg.get("gripper"), "joint_num": cfg.get("joint_num"),
            "pcb_version": cfg.get("pcb_version"), "hardware": ctx.get("hardware")}


def main(argv=None, ui=None, ctx=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="the robot's executor (default: this Pi)")
    ap.add_argument("--stages", default="0,1,2,3", help="comma list of stages to run (default 0,1,2,3)")
    ap.add_argument("--list", action="store_true", help="list the steps and exit")
    ap.add_argument("--by-hand", action="store_true",
                    help="stage 3: turn each joint by hand (one at a time limp) instead of the powered test moves")
    args = ap.parse_args(argv)
    if args.list:
        for s in STEPS + BY_HAND:
            print(f"  {s['id']:<16} S{s['stage']}  {s['title']} — {s['about']}"
                  + ("  (--by-hand)" if s in BY_HAND else ""))
        return 0
    try:
        stages = sorted({int(x) for x in args.stages.split(",") if x.strip()})
    except ValueError:
        ap.error("--stages takes numbers, e.g. 0,1,2")
    if not stages or any(s not in core.STAGE_TITLES for s in stages):
        ap.error(f"--stages: one or more of {sorted(core.STAGE_TITLES)}")
    t0 = time.time()
    ui = ui or ConsoleUI()
    ctx = ctx if ctx is not None else build_ctx(args.host)
    print("RCB first-run check — every step says what it checks; only stage 3 moves the arm (a few degrees).")
    try:
        results, completed = core.run(select_steps(stages, args.by_hand), ctx, ui, stages)
    finally:
        robot = ctx.get("robot")
        if robot is not None:
            for fn in (robot.release_control, robot.disconnect):
                try:
                    fn()
                except Exception:   # the run's result matters more than a tidy goodbye
                    pass
    folder = report.write(results, meta_of(ctx, completed, t0), root=report.REPORTS_DIR)
    c = core.counts(results)
    print()
    print(f"Result: {core.worst(results)} — " + ", ".join(f"{c[v]} {v}" for v in c if c[v])
          + ("" if completed else " (stopped before the end)"))
    print(f"Report: {os.path.join(folder, 'report.txt')}")
    if not completed:
        return 2
    return 1 if core.worst(results) == core.FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
