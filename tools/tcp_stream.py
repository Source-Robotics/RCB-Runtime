#!/usr/bin/env python3
"""Stream the arm's TCP pose to another PC over UDP (2026-10-19): the real PAR6, hand-guided in freedrive, as the input
device of an IK / TCP-control simulator. Moves NOTHING and commands nothing — it only reads what RTI computes every tick.

Home the arm, enable it and drag it in freedrive (dashboard: IDLE with gravity compensation ON). This sends, --rate
times a second, one JSON datagram: the TCP pose (RTI's own forward kinematics of the measured joints — the active
gripper's TCP in the robot base frame) and the six joint angles in both conventions.

    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/tcp_stream.py --host <sim PC IP>
    cd ~/Desktop/RCB-Runtime && .venv/bin/python tools/tcp_stream.py --host 192.168.1.50 --port 5005 --rate 100

Frames (checked 2026-10-19 over 300 random poses each for the MSG, SSG48 and flange: <= 0.0053 mm / 0.0013 deg): the
pose IS the PAR6 description package's `tcp` frame relative to `base_link` (par6_<tool>.urdf) and its MJCF `tcp` site,
with the package's joint values = ours + URDF_OFFSET_DEG (tools/self_collision_capsules.py).

The packet format and a receiver for the simulator's side: tools/tcp_stream_kit/ (README.md + par6_tcp_receiver.py —
hand that folder over). Reads shared memory as FILES (/dev/shm/<name>, as api_tests/64 does): no model, no live segment.
"""
import argparse
import json
import os
import secrets
import socket
import struct
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.join(_ROOT, "tools"))

PROTOCOL_VERSION = 1
STALE_S = 0.1            # RTI's tick counter not advancing for this long = the robot's data is not live
SHM = "/dev/shm/" + os.environ.get("RCB_SHM_PREFIX", "")
# the description package's variant per gripper (the URDF / MJCF file the simulator loads); others: no model
MODEL_OF_TOOL = (("MSG", "par6_msg"), ("SSG48", "par6_ssg48"), ("Flange", "par6_flange"))


def f64(name, n=1):
    with open(SHM + name, "rb") as fh:
        return struct.unpack(f"<{n}d", fh.read(8 * n))


def text(name):
    with open(SHM + name, "rb") as fh:
        b = fh.read(260)
    k = struct.unpack("<I", b[:4])[0]
    return b[4:4 + k].decode(errors="replace")


def main():
    p = argparse.ArgumentParser(description="Stream the TCP pose to a simulator over UDP (read-only).")
    p.add_argument("--host", required=True, help="the simulator PC's IP address (or name)")
    p.add_argument("--port", type=int, default=5005, help="its UDP port (default 5005)")
    p.add_argument("--rate", type=float, default=50.0, help="datagrams per second, 1..250 (default 50; RTI ticks at 250)")
    p.add_argument("--print-every", type=float, default=1.0, help="seconds between console lines (0 = state changes only)")
    args = p.parse_args()
    if not 1.0 <= args.rate <= 250.0:
        p.error("--rate must be 1..250")

    import config.xml_parser as xp
    from project_paths import ROBOTS_DIR, SYSTEM_XML
    from self_collision_capsules import URDF_OFFSET_DEG, URDF_SIGN
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    tool = str(sysc["active_gripper"])
    robot = os.path.splitext(os.path.basename(str(sysc["robot_xml_path"])))[0]
    model = next((m for prefix, m in MODEL_OF_TOOL if tool.startswith(prefix)), None) if robot == "PAR6" else None
    sign, offset = [float(v) for v in URDF_SIGN], [float(v) for v in URDF_OFFSET_DEG]

    addr = socket.getaddrinfo(args.host, args.port, type=socket.SOCK_DGRAM)[0]
    sock = socket.socket(addr[0], socket.SOCK_DGRAM)
    target = addr[4]
    os.nice(10)              # a guest on the robot's Pi: never ahead of anything the robot runs
    session = secrets.token_hex(4)

    print(f"Streaming {robot} / {tool} TCP pose to {target[0]}:{target[1]} UDP at {args.rate:g} Hz (session {session}).")
    print(f"Simulator model: {model + '.urdf / .xml, frame tcp' if model else 'NONE for this robot/tool — the pose is still sent'}")
    print("Read-only: home + enable the arm, gravity compensation ON, drag it. Ctrl-C to stop.\n")

    period = 1.0 / args.rate
    next_t = time.monotonic()
    seq = 0
    last_tick, tick_changed_at = None, None
    last_flags, last_print, send_error = None, 0.0, None
    try:
        while True:
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()   # fell behind (a busy Pi): restart the schedule, never a burst

            now = time.monotonic()
            tick = int(f64("loop_tick")[0])
            if tick != last_tick:
                last_tick, tick_changed_at = tick, now
            m = f64("robot_pose_matrix", 16)                    # row-major 4x4, base -> TCP, metres
            quat = f64("robot_pose_quaternion", 4)              # (w, x, y, z), w >= 0
            q_rad = f64("joint_position", 6)
            mode, state, homed_s = text("robot_mode"), text("robot_state"), text("homed_state")
            gravity = f64("gravity_comp_enabled")[0] > 0.5

            fresh = tick_changed_at is not None and now - tick_changed_at < STALE_S
            homed = homed_s == "HOMED"
            freedrive = gravity and mode == "IDLE" and state == "ENABLED"
            valid = fresh and homed and mode != "ACTIVE_ERROR"
            q_deg = [v * 57.29577951308232 for v in q_rad]
            seq += 1
            pkt = {
                "type": "par6_tcp", "version": PROTOCOL_VERSION, "session": session, "seq": seq,
                "t": round(time.time(), 6), "tick": tick,
                "valid": valid, "fresh": fresh, "homed": homed, "freedrive": freedrive, "mode": mode, "state": state,
                "tool": tool, "model": model,
                "pos": [round(m[3], 7), round(m[7], 7), round(m[11], 7)],
                "quat_wxyz": [round(v, 9) for v in quat],
                "rot": [[round(v, 9) for v in m[0:3]], [round(v, 9) for v in m[4:7]], [round(v, 9) for v in m[8:11]]],
                "q_deg": [round(v, 6) for v in q_deg],
                "q_model_rad": [round((s * v + o) * 0.017453292519943295, 9) for v, s, o in zip(q_deg, sign, offset)],
            }
            try:
                sock.sendto(json.dumps(pkt, separators=(",", ":")).encode(), target)
                if send_error is not None:
                    print("sending again", flush=True)
                send_error = None
            except OSError as exc:                               # no route / network down: say it once, keep trying
                if send_error != str(exc):
                    print(f"send failed: {exc}", flush=True)
                send_error = str(exc)

            flags = (valid, fresh, homed, freedrive, mode, state)
            if flags != last_flags:
                why = [] if valid else [w for w, bad in (("RTI not ticking", not fresh), ("not homed", not homed),
                                                          ("error latched", mode == "ACTIVE_ERROR")) if bad]
                print(f"[{seq}] {'VALID' if valid else 'NOT VALID (' + ', '.join(why) + ')'}  mode={mode} state={state}"
                      f"  freedrive={'ON' if freedrive else 'off'}", flush=True)
                last_flags = flags
            if args.print_every > 0 and now - last_print >= args.print_every:
                last_print = now
                print(f"  seq {seq:7d}  tcp [mm] {1e3 * m[3]:8.2f} {1e3 * m[7]:8.2f} {1e3 * m[11]:8.2f}"
                      f"   q [deg] {' '.join(f'{v:7.2f}' for v in q_deg)}", flush=True)
    except KeyboardInterrupt:
        pass
    print(f"\n{seq} datagram(s) sent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
