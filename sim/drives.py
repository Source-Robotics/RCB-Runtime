#!/usr/bin/env python3
"""The simulated drives on the virtual CAN bus (TODO/simulated_robot_plan.md S1, 2026-10-13): one SimDrive per node
of the active robot + gripper (the same XMLs RTI loads), answering RTI's frames on vcan0 and stepping the joint models
at ~1 kHz. Normally started by tools/sim_run.py; by hand:

    cd ~/Desktop/RCB-Runtime && RCB_SIM=1 python3 -m sim.drives

Runs at normal priority on the general cores; SIGTERM / Ctrl-C stops it.
"""
from __future__ import annotations

import os
import signal
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from sim.drive_sim import SimDrive, split_id                             # noqa: E402
from utility import sim_mode                                              # noqa: E402

STEP_S = 0.001
REPORT_S = 10.0


def build_drives():
    """{node: SimDrive} for the active robot's joints (+ its CAN gripper), Kt from the robot / gripper XMLs."""
    import math
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    n = int(sysc["joint_num"])
    drives = {i: SimDrive(i, kt=float(cfg["motor_kt"][i])) for i in range(n)}
    raw = cfg.get("gripper_CAN_gripper")
    if raw is not None and not (isinstance(raw, float) and math.isnan(raw)) and int(raw):
        drives[n] = SimDrive(n, gripper=True, kt=float(cfg["gripper_kt"]))
    return drives


def main():
    if not sim_mode.interface_up(sim_mode.SIM_CAN_INTERFACE):
        print(f"[sim] {sim_mode.SIM_CAN_INTERFACE} is not up — {sim_mode.VCAN_HELP}", file=sys.stderr)
        return 2
    import can
    drives = build_drives()
    bus = can.interface.Bus(channel=sim_mode.SIM_CAN_INTERFACE, interface="socketcan")
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    print(f"[sim] {len(drives)} simulated drive(s) on {sim_mode.SIM_CAN_INTERFACE}: nodes {sorted(drives)}"
          f"{' (gripper ' + str(max(drives)) + ')' if any(d.gripper for d in drives.values()) else ''}", flush=True)
    t0 = last_step = last_report = time.monotonic()
    frames = replies = 0
    try:
        while not stop:
            msg = bus.recv(timeout=0.0005)
            now = time.monotonic()
            ms = int((now - t0) * 1000)
            if msg is not None and not msg.is_error_frame and not msg.is_extended_id:
                node, cmd, _err = split_id(msg.arbitration_id)
                d = drives.get(node)
                if d is not None:
                    frames += 1
                    for arb, data in d.handle(cmd, msg.data, msg.is_remote_frame, ms):
                        bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=False))
                        replies += 1
            if now - last_step >= STEP_S:
                for d in drives.values():
                    d.step(now - last_step, ms)
                last_step = now
            if now - last_report >= REPORT_S:
                errs = [n for n, d in drives.items() if d.error]
                print(f"[sim] {frames} frames in, {replies} replies out; drives in error: {errs or 'none'}", flush=True)
                last_report = now
    finally:
        bus.shutdown()
        print(f"[sim] stopped after {time.monotonic() - t0:.1f} s: {frames} frames in, {replies} replies out",
              flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
