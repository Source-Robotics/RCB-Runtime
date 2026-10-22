#!/usr/bin/env python3
"""Receive the real PAR6's TCP pose stream — the simulator side of RCB-Runtime's tools/tcp_stream.py.

The robot is hand-guided in gravity compensation; it sends one JSON object per UDP datagram (~50 Hz) with its TCP
pose and joint angles. The packet format and the frames are in README.md next to this file.

Standard library only (Python 3.8+). Use it as a module:

    from par6_tcp_receiver import Par6PoseReceiver

    rx = Par6PoseReceiver(port=5005)
    while sim_running:
        p = rx.poll()                     # the newest sample, or None while nothing live arrives
        if p is not None and p.valid:
            set_ik_target(p.pos, p.quat_wxyz)        # metres, (w, x, y, z): the `tcp` frame in `base_link`
        step_sim()

or run it:

    python par6_tcp_receiver.py                # print what arrives on UDP 5005
    python par6_tcp_receiver.py --fake         # also send a synthetic stream to itself (no robot needed)
"""
import argparse
import json
import math
import secrets
import socket
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

PROTOCOL_VERSION = 1
DEFAULT_PORT = 5005


@dataclass
class Par6Pose:
    """One sample from the robot. Lengths in metres, angles in radians unless the name says _deg."""
    session: str                    # random per sender start; a new value = the sender restarted (seq starts at 1)
    seq: int                        # +1 per datagram within a session
    t_sent: float                   # sender's wall clock (Unix s) — not synchronised with this PC
    tick: int                       # the robot's control-loop tick the sample came from (250 Hz)
    valid: bool                     # homed + live + no error: safe to use as a target
    fresh: bool                     # the robot's control loop is running
    homed: bool                     # joint angles are referenced (meaningless before homing)
    freedrive: bool                 # gravity compensation in IDLE, enabled: the arm is being hand-guided
    mode: str                       # robot mode, e.g. IDLE, EXEC, JOG, ACTIVE_ERROR
    state: str                      # ENABLED / DISABLED
    tool: str                       # the gripper configured on the robot
    model: Optional[str]            # the description-package variant that matches: par6_msg / par6_ssg48 / par6_flange
    pos: List[float]                # [x, y, z] of `tcp` in `base_link`
    quat_wxyz: List[float]          # orientation of `tcp` in `base_link`, scalar FIRST, w >= 0
    rot: List[List[float]]          # the same orientation as a 3x3 matrix, rows; columns = the tcp axes in base coords
    q_deg: List[float]              # joint angles in the robot's own convention (what its dashboard shows)
    q_model_rad: List[float]        # joint angles for the PAR6 URDF / MJCF (base_joint .. wrist_3_joint)
    received_at: float = field(default=0.0)   # time.monotonic() on THIS PC when it arrived

    @property
    def age(self) -> float:
        """Seconds since this sample arrived."""
        return time.monotonic() - self.received_at

    @property
    def quat_xyzw(self) -> List[float]:
        """The orientation in (x, y, z, w) order (scipy, ROS, Eigen's coeffs(), MuJoCo uses w-first like quat_wxyz)."""
        w, x, y, z = self.quat_wxyz
        return [x, y, z, w]

    @property
    def matrix4(self) -> List[List[float]]:
        """4x4 homogeneous transform base_link -> tcp, rows."""
        r = self.rot
        return [r[0] + [self.pos[0]], r[1] + [self.pos[1]], r[2] + [self.pos[2]], [0.0, 0.0, 0.0, 1.0]]


def parse_packet(data: bytes) -> Par6Pose:
    """One datagram -> Par6Pose. Raises ValueError on anything that is not a version-1 PAR6 packet."""
    try:
        d = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"not JSON: {exc}") from None
    if not isinstance(d, dict) or d.get("type") != "par6_tcp":
        raise ValueError("not a par6_tcp packet")
    if d.get("version") != PROTOCOL_VERSION:
        raise ValueError(f"protocol version {d.get('version')!r}, this receiver reads {PROTOCOL_VERSION}")
    try:
        return Par6Pose(
            session=str(d["session"]), seq=int(d["seq"]), t_sent=float(d["t"]), tick=int(d["tick"]),
            valid=bool(d["valid"]), fresh=bool(d["fresh"]), homed=bool(d["homed"]), freedrive=bool(d["freedrive"]),
            mode=str(d["mode"]), state=str(d["state"]), tool=str(d["tool"]), model=d.get("model"),
            pos=[float(v) for v in d["pos"]], quat_wxyz=[float(v) for v in d["quat_wxyz"]],
            rot=[[float(v) for v in row] for row in d["rot"]],
            q_deg=[float(v) for v in d["q_deg"]], q_model_rad=[float(v) for v in d["q_model_rad"]],
            received_at=time.monotonic())
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"malformed packet: {exc!r}") from None


class Par6PoseReceiver:
    """Non-blocking UDP receiver that keeps only the NEWEST sample (a pose stream is latest-wins: an old pose is
    worth nothing once a newer one exists). Call poll() once per simulation step; it never blocks."""

    def __init__(self, port: int = DEFAULT_PORT, host: str = "0.0.0.0", stale_after: float = 0.25):
        self.stale_after = stale_after          # poll() returns None when the newest sample is older than this
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host, port))
        self.sock.setblocking(False)
        self.latest: Optional[Par6Pose] = None
        self.received = self.dropped_old = self.bad = 0
        self.last_error = ""

    def poll(self) -> Optional[Par6Pose]:
        """Drain every datagram waiting, keep the newest. The newest sample while it is younger than stale_after,
        else None (the robot side stopped, the network dropped, or nothing arrived yet)."""
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as exc:              # e.g. Windows' WSAECONNRESET after an ICMP "port unreachable"
                self.last_error = str(exc)
                break
            try:
                p = parse_packet(data)
            except ValueError as exc:
                self.bad += 1
                self.last_error = str(exc)
                continue
            self.received += 1
            cur = self.latest
            if cur is not None and p.session == cur.session and p.seq <= cur.seq:
                self.dropped_old += 1           # reordered or duplicated by the network
                continue
            self.latest = p
        if self.latest is None or self.latest.age > self.stale_after:
            return None
        return self.latest

    def close(self) -> None:
        self.sock.close()


# ----------------------------------------------------------------------------------------------------------------------
# --fake: a synthetic stream with the robot's exact packet format, so the simulator can be developed without the robot.
# The forward kinematics below is the robot's own DH model (robots/PAR6.py + the MSG gripper's tool link); it matches
# par6_msg.urdf's `tcp` frame and the MJCF `tcp` site to 0.005 mm. Handy as a cross-check of the simulator's model too.
# ----------------------------------------------------------------------------------------------------------------------
# standard DH per joint (d, a, alpha), theta = the robot's own joint angle (no offsets)
PAR6_MSG_DH = [(0.1105, 0.02342, -math.pi / 2), (0.0, 0.180, math.pi), (0.0, -0.0435, math.pi / 2),
               (-0.177, 0.0, -math.pi / 2), (0.0, 0.0, math.pi / 2), (-0.1409, -0.02835, math.pi)]
# model (URDF / MJCF) joint value = robot joint value + this, degrees
MODEL_OFFSET_DEG = [0.0, 90.0, -180.0, 0.0, 0.0, -180.0]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def par6_fk(q_deg):
    """Robot joint angles (deg, robot convention) -> 4x4 base_link -> tcp (MSG gripper)."""
    t = [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]
    for (d, a, alpha), q in zip(PAR6_MSG_DH, q_deg):
        th = math.radians(q)
        ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(alpha), math.sin(alpha)
        t = _matmul(t, [[ct, -st * ca, st * sa, a * ct], [st, ct * ca, -ct * sa, a * st], [0.0, sa, ca, d],
                        [0.0, 0.0, 0.0, 1.0]])
    return t


def rot_to_quat_wxyz(r):
    """3x3 rotation -> unit quaternion (w, x, y, z) with w >= 0."""
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = [0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s]
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2])
        q = [(r[2][1] - r[1][2]) / s, 0.25 * s, (r[0][1] + r[1][0]) / s, (r[0][2] + r[2][0]) / s]
    elif r[1][1] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2])
        q = [(r[0][2] - r[2][0]) / s, (r[0][1] + r[1][0]) / s, 0.25 * s, (r[1][2] + r[2][1]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1])
        q = [(r[1][0] - r[0][1]) / s, (r[0][2] + r[2][0]) / s, (r[1][2] + r[2][1]) / s, 0.25 * s]
    n = math.sqrt(sum(v * v for v in q))
    q = [v / n for v in q]
    return [-v for v in q] if q[0] < 0 else q


def fake_sender(port: int, rate: float = 50.0, stop: Optional[threading.Event] = None) -> None:
    """Send the robot's packet format to 127.0.0.1:port: slow sinusoids around a safe pose, NOT VALID for the first
    2 s (as before homing), then valid."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session, seq, t0 = secrets.token_hex(4), 0, time.monotonic()
    home = [90.0, -105.0, 165.0, 0.0, -30.0, 180.0]          # the robot's usual start pose, robot convention
    amp = [20.0, 10.0, 10.0, 30.0, 20.0, 45.0]
    while stop is None or not stop.is_set():
        t = time.monotonic() - t0
        q = [h + a * math.sin(0.4 * t + i) for i, (h, a) in enumerate(zip(home, amp))]
        m = par6_fk(q)
        rot = [row[:3] for row in m[:3]]
        seq += 1
        valid = t > 2.0
        pkt = {"type": "par6_tcp", "version": PROTOCOL_VERSION, "session": session, "seq": seq, "t": time.time(),
               "tick": int(t * 250), "valid": valid, "fresh": True, "homed": valid, "freedrive": valid,
               "mode": "IDLE", "state": "ENABLED", "tool": "FAKE (MSG geometry)", "model": "par6_msg",
               "pos": [m[0][3], m[1][3], m[2][3]], "quat_wxyz": rot_to_quat_wxyz(rot), "rot": rot, "q_deg": q,
               "q_model_rad": [math.radians(v + o) for v, o in zip(q, MODEL_OFFSET_DEG)]}
        sock.sendto(json.dumps(pkt).encode(), ("127.0.0.1", port))
        time.sleep(1.0 / rate)


def main() -> int:
    ap = argparse.ArgumentParser(description="Print the PAR6 TCP stream (UDP, JSON).")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--fake", action="store_true", help="also send a synthetic stream to this port (no robot needed)")
    args = ap.parse_args()
    rx = Par6PoseReceiver(port=args.port)
    stop = threading.Event()
    if args.fake:
        threading.Thread(target=fake_sender, args=(args.port,), kwargs={"stop": stop}, daemon=True).start()
    print(f"Listening on UDP {args.port}{' (+ fake sender)' if args.fake else ''}. Ctrl-C to stop.")
    try:
        while True:
            time.sleep(0.1)
            p = rx.poll()
            if p is None:
                print(f"  no live stream   (received {rx.received}, bad {rx.bad}{', ' + rx.last_error if rx.last_error else ''})")
                continue
            x, y, z = (1e3 * v for v in p.pos)
            print(f"  seq {p.seq:6d} {'VALID' if p.valid else 'not valid'} {'freedrive' if p.freedrive else p.mode:9s}"
                  f"  tcp [mm] {x:8.2f} {y:8.2f} {z:8.2f}  quat wxyz {' '.join(f'{v:+.4f}' for v in p.quat_wxyz)}"
                  f"  age {1e3 * p.age:4.0f} ms")
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        rx.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
