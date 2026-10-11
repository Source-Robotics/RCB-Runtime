"""Shared by the per-tick telemetry tests (api_tests/66_tracking.py, 68_rti_feedforward.py; moved verbatim from 66 on
2026-10-11): the UDP telemetry capture, packets -> arrays, "is the arm enabled?", and the best time shift."""
import socket
import threading

import numpy as np

from robotclient.rti_client import parse_state_packet

TELEMETRY_PORT = 30003            # config/networking.py TELEMETRY_PORT (the import would pull in zmq setup)
NEED = ("j_p", "j_pc", "j_s", "j_sc", "j_ac", "j_t", "j_tc", "tick")
OPTIONAL = ("j_t_inertia", "j_t_coriolis",      # 2026-10-10; zeros in runs recorded before them
            "j_pt")                             # the target (RTI: the PC's raw request), 2026-10-11; zeros before
DELAY_MAX_S, DELAY_STEP_S, DELAY_COARSE_S = 0.020, 0.00025, 0.001   # the time shift searched for each move


class Capture:
    """Receives the robot's per-tick telemetry broadcast on this machine."""
    def __init__(self, port=TELEMETRY_PORT):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.2)
        self.packets = []
        self._run = True
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while self._run:
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            self.packets.append(data)

    def last_tick(self):
        """The newest tick received (scans back past packets of another recipe)."""
        for raw in reversed(self.packets[-50:]):
            try:
                r = parse_state_packet(raw.decode())
            except Exception:
                continue
            if "tick" in r and "j_ac" in r:
                return int(r["tick"])
        return None

    def stop(self):
        self._run = False
        self._t.join(timeout=1.0)
        self._sock.close()


def to_arrays(packets, joint_num=6):
    """Raw packets -> arrays ordered by tick, duplicates and packets of another recipe dropped."""
    rows = {}
    for raw in packets:
        try:
            r = parse_state_packet(raw.decode() if isinstance(raw, bytes) else raw)
        except Exception:
            continue
        if all(k in r for k in NEED):
            rows[int(r["tick"])] = r
    ticks = sorted(rows)
    out = {"tick": np.array(ticks, dtype=np.int64),
           "mode": np.array([str(rows[t].get("mode", "")) for t in ticks])}
    for k in NEED[:-1] + OPTIONAL:
        out[k] = np.array([np.resize(np.asarray(rows[t].get(k, 0.0), float), joint_num) for t in ticks]) if ticks \
            else np.zeros((0, joint_num))
    return out


def best_delay(t, cmd, meas, mask):
    """The shift D (0..DELAY_MAX_S) minimising the RMS of cmd(t - D) - meas(t) over `mask`, and that residual (deg).
    Coarse 1 ms grid, then the 0.25 ms grid within +/-0.75 ms of the coarse best: 28 evaluations instead of 81 (the
    full grid made the analysis heavy enough to slow RTI's loop, 2026-10-10)."""
    tm, mm = t[mask], meas[mask]
    best = [np.inf, 0.0, None]

    def try_at(D):
        r = np.degrees(np.interp(tm - D, t, cmd) - mm)
        e = float(np.mean(r ** 2))
        if e < best[0]:
            best[:] = [e, float(D), r]
    for D in np.arange(0.0, DELAY_MAX_S + 1e-12, DELAY_COARSE_S):
        try_at(D)
    c = best[1]
    for D in np.arange(max(0.0, c - 0.00075), min(DELAY_MAX_S, c + 0.00075) + 1e-12, DELAY_STEP_S):
        try_at(D)
    return best[1], best[2]


def robot_enabled(host, port):
    """True / False from the executor's status(); None when it cannot be reached (an analysis on another machine)."""
    try:
        from robotclient import RobotClient
        r = RobotClient(sync=False)
        r.connect(host, port)
        try:
            return r.status().get("state") == "ENABLED"
        finally:
            r.disconnect()
    except Exception:
        return None
