"""The control loop's delay per joint, from a CAN log and RTI's CAN read times (2026-10-25; the analysis came from
api_tests/69_can_gaps.py, which imports it from here — one definition).

The delay: a joint's CAN reply (its position, speed, current) arriving -> the CAN command RTI computed from it leaving.
~3.8 ms in option B's tick order (`can_read_first`: read first, send early), ~7.8 ms in the old order. It matters where
OUR software closes a loop on the measured state (the haptic walls, a PC's force control in hard RTI); the drives close
their own position loops on their encoders.

Measured ON REQUEST only (tools/loop_delay.py; the dashboard's RT Diagnostics "Measure" button), never by RTI: from
2026-10-12 to 2026-10-25 RTI sampled it live (utility/loop_delay_meter.py) by switching a CAN socket's receive filter
inside its tick — setsockopt(CAN_RAW_FILTER) takes the kernel's global network lock (rtnl_lock), and whenever another
process held that lock (a network namespace torn down: the hostname service a Wi-Fi reconnect starts, docker, network
changes) RTI's tick waited for it: 8-44 ms single-tick stalls, LOOP_CRITICAL, the arm disabled (kernel trace
2026-10-25: 15 of 15 long stalls in rtnl_lock <- raw_setsockopt). The measurement now runs in its own process
(`candump`, its own socket) and reads RTI's CAN read times from the shared-memory trace; RTI does nothing for it.

How the pairing works (loop_delay_per_joint): every joint gets a command and answers every tick, so by time alone — the
newest reply before RTI's read in tick k -> the first command to that joint after tick k+lag's read (lag 0 in option
B, 1 in the old order). Times are CLOCK_REALTIME: candump's kernel stamps and RTI's `can_read_t`.
"""
from __future__ import annotations

import numpy as np

CMD_DATA_PACK_1 = 2               # RTI -> drive (Spectral_BLDC SEND_DATA_PACK_1_ID)
CMD_DATA_PACK_PD = 4              # RTI -> drive (SEND_DATA_PACK_PD_ID)
RESP_DATA_PACK_1 = 3              # drive -> RTI: position 24 b, speed 24 b, current 16 b, big-endian signed
JOINT_NODES = 6                   # CAN nodes 0..5 = J1..J6


def parse_candump(lines):
    """candump -L lines -> sorted arrays: t (s, CLOCK_REALTIME — when our CAN driver handled the frame), node, cmd,
    pos (the 24-bit position of a data pack or its reply; NaN otherwise)."""
    t, node, cmd, pos = [], [], [], []
    for ln in lines:
        ln = ln.strip() if isinstance(ln, str) else ln.decode(errors="replace").strip()
        if not ln.startswith("("):
            continue
        try:
            ts, _iface, frame = ln.split()
            cid_hex, data_hex = frame.split("#", 1)
            cid = int(cid_hex, 16)
            data = b"" if data_hex.startswith("R") else bytes.fromhex(data_hex)
            tt = float(ts.strip("()"))
        except ValueError:
            continue
        if cid > 0x7FF:
            continue
        c = (cid >> 1) & 0x3F
        p = np.nan
        if c in (CMD_DATA_PACK_1, CMD_DATA_PACK_PD, RESP_DATA_PACK_1) and len(data) == 8:
            p = int.from_bytes(data[0:3], "big", signed=True)
        t.append(tt)
        node.append((cid >> 7) & 0xF)
        cmd.append(c)
        pos.append(p)
    o = np.argsort(np.asarray(t), kind="stable")
    return {"t": np.asarray(t, float)[o], "node": np.asarray(node, int)[o], "cmd": np.asarray(cmd, int)[o],
            "pos": np.asarray(pos, float)[o]}


def node_frames(c, node, cmds):
    m = (c["node"] == node) & np.isin(c["cmd"], cmds)
    return c["t"][m], c["pos"][m]


def _stats(x):
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {"n": 0}
    return {"n": int(x.size), "med": float(np.median(x)), "p99": float(np.percentile(x, 99)), "max": float(np.max(x)),
            "min": float(np.min(x))}


def candump_gaps(c, w0, w1, min_ms=8.0):
    """[(t0, t1)]: stretches of the window with no frame at all in the log. The bus is never that quiet while RTI
    runs (a frame every ~0.1-0.3 ms) — such a gap is candump missing frames, not the bus: excluded from the counts."""
    t = c["t"][(c["t"] >= w0) & (c["t"] < w1)]
    g = np.diff(t)
    i = np.flatnonzero(g > min_ms * 1e-3)
    return [(float(t[k]), float(t[k + 1])) for k in i]


def tick_anchor(read_t, ts):
    """Where each row's commands are looked for from: its CAN read (`can_read_t`) where recorded, else its clock reading.
    A tick's commands leave after its own read and before the next one in both orders; the clock reading (the history
    entry's stamp) does not: with option B's send-early order (2026-10-12, §9.10) the entry is made AFTER the commands
    left, so J1's / J2's went out before it and the first command after it was the NEXT tick's (J1 / J2 read 100 %
    late and 7.8 ms of loop delay, while by content J1's was 3.8)."""
    return np.where(np.isfinite(read_t) & (read_t > 0), read_t, ts)


def loop_delay_per_joint(c, ts, read_t, tick, lag, gaps=(), joints=JOINT_NODES):
    """Per joint, by TIME alone (no motion needed — every joint gets a command and answers every tick): the newest
    reply before RTI's read in row k -> the first command to that joint after row k+lag's READ (the tick's own command:
    the frames leave after it in both orders — at RTI's sleep in the old order, after dispatch in option B; the clock
    reading is no anchor since option B makes the history entry after the send, 2026-10-12). lag 1 in the old order (row k's reply feeds tick k+1's command), 0 in option B. A pair
    is used only when the ticks are consecutive and that command came before the following row's stamp (it is really
    that tick's). ts / read_t in CLOCK_REALTIME. -> {"J1": stats, ...} in ms."""
    ok = np.isfinite(read_t) & (read_t > 0) & np.isfinite(ts)
    for g0, g1 in gaps:                       # a read or a command inside a candump gap cannot be judged
        ok &= ~((read_t > g0) & (read_t < g1 + 0.005)) & ~((ts > g0 - 0.005) & (ts < g1 + 0.005))
    anchor = tick_anchor(read_t, ts)          # the tick's read (see tick_anchor)
    n = tick.size
    out = {}
    for j in range(joints):
        tc = node_frames(c, j, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))[0]
        tr = node_frames(c, j, (RESP_DATA_PACK_1,))[0]
        if tc.size == 0 or tr.size == 0 or n <= lag + 1:
            continue
        ir = np.searchsorted(tr, read_t, "right") - 1             # the newest reply before each read
        ic = np.searchsorted(tc, anchor, "right")                 # the first command after each tick's read
        rep = np.where(ir >= 0, tr[np.clip(ir, 0, None)], np.nan)
        cmd = np.where(ic < tc.size, tc[np.clip(ic, None, tc.size - 1)], np.nan)
        k = np.arange(n - lag - 1)
        good = ok[k] & ok[k + lag] & ((tick[k + lag] - tick[k]) == lag) & (cmd[k + lag] < anchor[k + lag + 1])
        out[f"J{j + 1}"] = _stats((cmd[k + lag][good] - rep[k][good]) * 1e3)
    return out


SKIP_MODES = ("HOMING", "FLASHING")    # they send their own frames, not a command to every joint each tick: not judged


def report(candump_lines, tick, read_t, modes, lag, joints=JOINT_NODES):
    """The on-request measurement's result (tools/loop_delay.py) from what it recorded: the `candump -L` lines, and per
    RTI tick its number, its CAN read time (`can_read_t`, CLOCK_REALTIME) and its mode's name. Not judged: ticks in
    SKIP_MODES, reads outside the log (their reply or command may be missing from it) and reads in a stretch candump
    missed (candump_gaps). The read time stands in for the tick's clock reading too (loop_delay_per_joint's `ts`, used
    only to drop commands inside a gap). lag: 0 in option B's order, 1 in the old one.
    -> {"joints": {"J1": {"n", "med", "p99", "max", "min"} in ms (only {"n": 0} when none judged), ...},
        "ticks": the ticks judged, "frames": the frames in the log, "skipped_mode": ticks skipped for their mode,
        "gaps": the stretches candump missed, [(t0, t1)]}"""
    c = parse_candump(candump_lines)
    tick = np.asarray(tick, float)
    read_t = np.array(read_t, float)
    skip = np.isin(np.asarray(modes, dtype=object), SKIP_MODES)
    read_t[skip] = np.nan
    gaps = []
    if c["t"].size:
        w0, w1 = float(c["t"][0]), float(c["t"][-1])
        read_t[(read_t < w0) | (read_t > w1)] = np.nan
        gaps = candump_gaps(c, w0, w1)
    else:
        read_t[:] = np.nan
    joints_out = loop_delay_per_joint(c, read_t, read_t, tick, lag, gaps, joints)
    for j in range(joints):
        joints_out.setdefault(f"J{j + 1}", {"n": 0})
    return {"joints": joints_out, "ticks": int(np.isfinite(read_t).sum()), "frames": int(c["t"].size),
            "skipped_mode": int(skip.sum()), "gaps": gaps}
