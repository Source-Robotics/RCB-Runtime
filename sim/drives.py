#!/usr/bin/env python3
"""The simulated drives on the virtual CAN bus (TODO/simulated_robot_plan.md S1, 2026-10-13): one SimDrive per node
of the active robot + gripper (the same XMLs RTI loads), answering RTI's frames on vcan0 and stepping the joint models
at ~1 kHz. Normally started by tools/sim_run.py; by hand:

    cd ~/Desktop/RCB-Runtime && RCB_SIM=1 python3 -m sim.drives

Runs at normal priority on the general cores; SIGTERM / Ctrl-C stops it. S4 (2026-10-13): a control channel on
udp 127.0.0.1:SIMULATOR_PORT (config/networking.py) — JSON requests: {"cmd": "state"}, {"cmd": "silence" | "reboot",
"node": n, "ms": t}, {"cmd": "fault", "node": n, "kind": FAULTS | "clear"}, {"cmd": "drop", "node": n, "every": k,
"ms": t}; request() sends one (sim/faults.py). S6: {"cmd": "physics", "node": n, "kind": "push" | "runaway" (speed,
ticks/s) | "lag" (tau, s) | "buzz" (ma, hz) | "block" (dir: +1 / -1, default the endstop's side) | "clear", "ms": t}
— the disturbances of sim/drive_sim.py.
"""
from __future__ import annotations

import json
import math
import os
import signal
import socket
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from config.networking import SIMULATOR_PORT                              # noqa: E402
from sim.drive_sim import FAULTS, SimDrive, split_id                     # noqa: E402
from utility import sim_mode                                              # noqa: E402

STEP_S = 0.001
REPORT_S = 10.0


REACH_S = 3.0          # an endstop / the magnet sits this long (at the homing speed) from the start, on the homing side


def _num(v, default):
    try:
        v = float(v)
    except (TypeError, ValueError):
        return float(default)
    return float(default) if math.isnan(v) else v


def homing_layout(speed, direction, timeout_s, hall):
    """(stop, stop_dir, hall_centre) for one actuator starting at 0 ticks: its endstop (or, with a Hall sensor, its
    magnet and no endstop) REACH_S at its homing speed along its homing direction (0 = positive), at most 40 % of its
    homing timeout away. The joints are ideal (no gravity), so where the arm starts does not matter: homing defines
    the angles from these stops, as on the arm."""
    s = 1 if int(_num(direction, 1)) == 0 else -1
    dist = abs(_num(speed, 5000.0)) * min(REACH_S, 0.4 * _num(timeout_s, 6.0))
    if hall:
        return None, s, s * dist
    return s * dist, s, None


def build_drives():
    """{node: SimDrive} for the active robot's joints (+ its CAN gripper): Kt from the robot / gripper XMLs, the
    endstops and J6's magnet placed from the homing fields (the same defaults as robotics/homing/config.py)."""
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    n = int(sysc["joint_num"])
    drives = {}
    for i in range(n):
        stop, s, hall = homing_layout(cfg["homing_speed"][i], cfg["homing_direction"][i], cfg["homing_timeout_s"][i],
                                      int(_num(cfg["has_hall_sensor"][i], 0)) == 1)
        drives[i] = SimDrive(i, kt=float(cfg["motor_kt"][i]), stop=stop, stop_dir=s, hall=hall)
    raw = cfg.get("gripper_CAN_gripper")
    if raw is not None and not (isinstance(raw, float) and math.isnan(raw)) and int(raw):
        stop, s, hall = homing_layout(cfg.get("gripper_homing_speed"), cfg.get("gripper_homing_direction"),
                                      cfg.get("gripper_homing_timeout_s"), False)
        drives[n] = SimDrive(n, gripper=True, kt=float(cfg["gripper_kt"]), stop=stop, stop_dir=s)
    return drives


class Faults:
    """The injected faults of one node (S4, the control channel): silent (neither receives nor answers: a cable pulled,
    a drive without power) until silent_until_ms, then rebooted if reboot; every drop_every-th reply dropped until
    drop_until_ms."""
    def __init__(self):
        self.silent_until_ms = -1
        self.reboot = False
        self.drop_every = 0
        self.drop_until_ms = -1
        self.dropped = 0
        self._n = 0

    def silent(self, ms):
        return ms < self.silent_until_ms

    def drop_reply(self, ms):
        if self.drop_every <= 0 or ms >= self.drop_until_ms:
            return False
        self._n += 1
        if self._n % self.drop_every == 0:
            self.dropped += 1
            return True
        return False


def drive_state(d, f, ms):
    return {"position": round(d.position, 1), "speed": round(d.speed, 1), "current": round(d.current, 1),
            "mode": d.mode, "error": bool(d.error), "watchdog_error": bool(d.watchdog_error), "fault": d.fault,
            "estop_error": bool(d.estop_error), "wd_time_ms": d.wd_time_ms, "last_cmd_ms": d.last_cmd_ms,
            "trip_ms": d.trip_ms, "current_limit": d.current_limit, "velocity_limit": d.velocity_limit,
            "silent": f.silent(ms), "dropped": f.dropped, "frames": d.frames, "gripper": d.gripper, "stop": d.stop,
            "g_target": d.g_target, "g_position": d.g_position, "g_activated": d.g_activated, "g_action": d.g_action,
            "g_calibrated": d.g_calibrated,
            "boots": d.boots, "wd_at_boot": d.wd_at_boot, "wd_writes": d.wd_writes, "limit_writes": d.limit_writes,
            "physics": {k: v for k, v in (("push", d.push_until_ms > ms), ("runaway", d.runaway_until_ms > ms),
                                          ("lag", d.lag_until_ms > ms), ("buzz", d.buzz_until_ms > ms),
                                          ("block", d.block_until_ms > ms)) if v}}


def control(drives, faults, msg, ms):
    """One control-channel request (a dict) -> the reply (a dict). The S4 fault injection, pure (no sockets)."""
    cmd = msg.get("cmd")
    if cmd == "state":
        return {"ok": True, "now_ms": ms, "nodes": {str(n): drive_state(d, faults[n], ms) for n, d in drives.items()}}
    try:
        node = int(msg["node"])
    except (KeyError, TypeError, ValueError):
        return {"ok": False, "error": f"{cmd!r} needs a node"}
    if node not in drives:
        return {"ok": False, "error": f"no simulated drive {node}"}
    d, f = drives[node], faults[node]
    if cmd in ("silence", "reboot"):
        f.silent_until_ms = ms + int(msg.get("ms", 500))
        f.reboot = cmd == "reboot"
    elif cmd == "fault":
        kind = msg.get("kind")
        if kind in (None, "clear", ""):
            d.fault = None
        elif kind in FAULTS:
            d.fault = kind
        else:
            return {"ok": False, "error": f"fault kind {kind!r} not in {FAULTS}"}
    elif cmd == "drop":
        f.drop_every, f.drop_until_ms, f.dropped = int(msg.get("every", 2)), ms + int(msg.get("ms", 1000)), 0
    elif cmd == "physics":                               # S6 disturbances
        kind, until = msg.get("kind"), ms + int(_num(msg.get("ms"), 1000))
        if kind == "push":
            d.push_speed, d.push_until_ms = _num(msg.get("speed"), 0.0), until
        elif kind == "runaway":
            d.runaway_speed, d.runaway_until_ms = _num(msg.get("speed"), 0.0), until
        elif kind == "lag":
            d.lag_tau, d.lag_until_ms = max(0.0, _num(msg.get("tau"), 0.0)), until
        elif kind == "buzz":
            d.buzz_ma, d.buzz_hz, d.buzz_until_ms = _num(msg.get("ma"), 0.0), _num(msg.get("hz"), 60.0), until
        elif kind == "block":                            # an obstacle where the joint is now, on one side
            side = msg.get("dir")
            d.block_at, d.block_until_ms = d.position, until
            d.block_dir = (1 if side > 0 else -1) if isinstance(side, (int, float)) and side else d.stop_dir
        elif kind in (None, "clear", ""):
            d.push_until_ms = d.runaway_until_ms = d.lag_until_ms = d.buzz_until_ms = d.block_until_ms = -1
        else:
            return {"ok": False, "error": f"physics kind {kind!r} not in push, runaway, lag, buzz, block, clear"}
    else:
        return {"ok": False, "error": f"unknown command {cmd!r}"}
    return {"ok": True, "now_ms": ms}


def main():
    if not sim_mode.interface_up(sim_mode.SIM_CAN_INTERFACE):
        print(f"[sim] {sim_mode.SIM_CAN_INTERFACE} is not up — {sim_mode.VCAN_HELP}", file=sys.stderr)
        return 2
    import can
    drives = build_drives()
    faults = {n: Faults() for n in drives}
    bus = can.interface.Bus(channel=sim_mode.SIM_CAN_INTERFACE, interface="socketcan")
    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)       # the S4 control channel (127.0.0.1 only)
    ctl.bind(("127.0.0.1", SIMULATOR_PORT))
    ctl.setblocking(False)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    print(f"[sim] {len(drives)} simulated drive(s) on {sim_mode.SIM_CAN_INTERFACE}: nodes {sorted(drives)}"
          f"{' (gripper ' + str(max(drives)) + ')' if any(d.gripper for d in drives.values()) else ''}; control "
          f"channel udp 127.0.0.1:{SIMULATOR_PORT}", flush=True)
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
                if d is not None and not faults[node].silent(ms):
                    frames += 1
                    for arb, data in d.handle(cmd, msg.data, msg.is_remote_frame, ms):
                        if faults[node].drop_reply(ms):
                            continue
                        bus.send(can.Message(arbitration_id=arb, data=data, is_extended_id=False))
                        replies += 1
            try:
                req, addr = ctl.recvfrom(4096)
            except BlockingIOError:
                req = None
            if req:
                try:
                    reply = control(drives, faults, json.loads(req), ms)
                except (ValueError, TypeError) as e:
                    reply = {"ok": False, "error": f"bad request: {e}"}
                if reply.get("ok") and json.loads(req).get("cmd") != "state":
                    print(f"[sim] {ms} ms control: {req.decode(errors='replace')}", flush=True)
                ctl.sendto(json.dumps(reply).encode(), addr)
            for n, f in faults.items():
                if f.reboot and not f.silent(ms):
                    drives[n].reboot()
                    f.reboot = False
                    print(f"[sim] {ms} ms node {n} rebooted (its config lost)", flush=True)
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
        ctl.close()
        print(f"[sim] stopped after {time.monotonic() - t0:.1f} s: {frames} frames in, {replies} replies out",
              flush=True)
    return 0


def request(msg, timeout=1.0):
    """Send one control request to a running simulator (127.0.0.1) and return its reply — for tests and sim/faults.py."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(timeout)
        s.sendto(json.dumps(msg).encode(), ("127.0.0.1", SIMULATOR_PORT))
        data, _ = s.recvfrom(65536)
    return json.loads(data)


if __name__ == "__main__":
    sys.exit(main())
