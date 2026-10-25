#!/usr/bin/env python3
"""69: why does J1 sometimes get no fresh reply for 1-3 ticks? CAN frames, CAN error counters and RTI's loop timing,
recorded together (2026-10-12).

Seen in api_tests/68 since the 2026-10-11 ~23:15 restart (none in ~230 s of J1 motion before it, then 2 in 86 s and
5 in 34 s): J1's measured position, speed and torque all repeat on 1-3 consecutive ticks while it moves — RTI got no
new reply from J1's drive — and the drive brakes meanwhile (negative torque, 49 -> 13 deg/s) although the command
keeps going, then catches up with +1.5 Nm: up to 1.2 deg off the path for ~80 ms, with the feedforward on or off,
and nothing in the logs (no `[CAN] dropped`, no stale or loop warning). A drive that brakes while it is told to move
did not GET the commands in time; a missing reply could be the drive, the bus, or our side. Measure before changing
anything (user, 2026-10-12): this records

  can0.log      every frame on can0 (`candump -L`, read-only: it sends nothing) — RTI's commands (their echo, stamped
                when our CAN driver finished sending them) and every reply (stamped when the driver read it)
  telemetry     every RTI tick (recipe `can_gaps`): J1's raw encoder position as RTI last received it (repeats = no new
                reply), the commanded and measured joint state, the loop's execution time (each packet carries the
                PREVIOUS tick's) and period, and RTI's own clock read just after its CAN read (lines the ticks up with
                the CAN log to the microsecond)
  counters      `ip -s -s -d link show can0` before and after every block: RX overruns (the CAN chip's 2 receive
                buffers were full — a frame lost on OUR side), bus errors, error-passive, bus-off, TX errors / drops;
                and the SPI / CAN-chip interrupt counts; a snapshot of the SPI / CAN kernel threads (class, priority, core)

around two kinds of J1 motion, each twice, in the old tick order and with option B (`_rf` blocks: RTI reads CAN first
and sends early, TODO/can_tx_timing_plan.md §9; see DEFAULT_ORDER), after 3 s of IDLE as a baseline:

  passthrough   hard RTI, 250 Hz, with send times: the pass-through (the hard default since 2026-10-11; the old planner
                block — 101-byte packets — is gone: hard RTI refuses a position without a send time since 2026-10-12)
  exec          EXEC moves J1 +/-15 deg back and forth at 80 % (~57 deg/s peak, ruckig, the default profile)
  The RTI blocks run 68's J1 sine (+/-15 deg, 1.5 s period, ~63 deg/s peak, faded in / out, 8 s); nothing else changes
  (generator / pacing are each contract's defaults, the feedforward as the robot has it).

WHAT IS REPORTED, per block:
  stale runs    runs of consecutive ticks with NO new J1 reply while J1 is commanded faster than 20 deg/s (a fresh
                reply moves the encoder by 20+ counts per tick at that speed), per minute of motion; the longest; the
                largest departure of J1's tracking error from its own running median around each run (deg)
  loop          RTI's execution time per tick (median / p99 / max, ticks over 3 ms), the longest period
  bus           per joint node: commands, replies, replies missing (commands - replies), reply intervals over 6 ms;
                J1's response time (its reply after its command); the tick's exchange (J1's command to the last joint
                reply before the next tick's) and the longest pause INSIDE one (a host stall shows as a pause after which
                the waiting frames come in a bunch)
  timing        J1's command echo and the reply RTI used, relative to RTI's clock reading after its read (ms): whether
                our frames go out while RTI still runs (negative) or only after it (positive)
  loop delay    J1's reply -> the command computed from it (its echo), ms: in the old order the reply read in tick k
                feeds tick k+1's command, with option B tick k's — the delay the control loop adds
  counters      what changed during the block (RX overruns above all)
and every stale run on its own line: when, how long, how late the reply that would have refreshed J1 came, when its
command went out, the drive's response time, missing replies, the longest pause inside its tick's exchange, RTI's
execution time on the ticks around it, and a FIRST-SORT label (the numbers decide):
  reply lost    a command to J1 got no reply at all (the drive did not answer, or our chip overran — see the counters)
  host stall    the exchange that carried J1's reply paused > 1.5 ms and then came in a bunch: the frames waited in
                our CAN path (the SPI / CAN driver threads; candump's stamps are the driver's, not the wire's)
  command late  J1's command went out > 1 ms later than usual in its tick
  drive slow    the reply came > 1 ms after the command, alone
  read early    none of these: the reply came as usual and RTI's read was earlier than usual

The robot must be ENABLED and homed, in IDLE or EXEC; J1 needs +/-25 deg of room (the start pose of the other tests:
J1 90 deg). The script takes control without forcing, switches the telemetry recipe for the run and restores it,
and leaves the arm in IDLE. Ctrl+C stops the arm. Nothing is analysed on the Pi while the arm is enabled (the
analysis would slow RTI's loop): the raw recording is saved, then `--reanalyse` with the arm disabled.

    python3 api_tests/69_can_gaps.py --selftest      # the analysis on a synthetic recording with known events
    python3 api_tests/69_can_gaps.py                 # dry run: the plan
    python3 api_tests/69_can_gaps.py --go            # ~2 min
    python3 api_tests/69_can_gaps.py --go --order passthrough,exec,passthrough,exec
    python3 api_tests/69_can_gaps.py --reanalyse temp_data/can_gaps/<time>    # with the arm DISABLED

Results: temp_data/can_gaps/<time>/ (can0.log, telemetry.raw, run.json; report.json after the analysis).
"""
import argparse
import importlib
import json
import math
import os
import random
import re
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _tracking_common import Capture, robot_enabled  # noqa: E402
from config.cpu_cores import GENERAL_CPU_CORES  # noqa: E402
from utility.loop_delay import (CMD_DATA_PACK_1, CMD_DATA_PACK_PD, JOINT_NODES, RESP_DATA_PACK_1,  # noqa: E402
                                _stats, candump_gaps, loop_delay_per_joint, node_frames, parse_candump,
                                tick_anchor)

RECIPE = "can_gaps"
KINDS = ("passthrough", "exec", "passthrough_rf", "exec_rf")   # 'planner' (101 bytes) is refused since 2026-10-12;
#   '_rf': option B (read first, send early) ON for the block; option A's '_wait' blocks retired 2026-10-12
DEFAULT_ORDER = "passthrough,passthrough_rf,exec,exec_rf,passthrough,passthrough_rf,exec,exec_rf"
LOAD_EVERY_S = 2.0                # --load: a Python interpreter (numpy + scipy imports) started this often per block
IDLE_S = 3.0                      # the baseline before the first block
AMP_DEG, PERIOD_S, SINE_S = 15.0, 1.5, 8.0     # the RTI blocks: 68's J1 sine
EXEC_SPEED, EXEC_MOVES = 80.0, 8                # % of the EXEC limits; 30 deg each (~1.04 s, ~57 deg/s peak)
CLEARANCE_DEG = 10.0
MOVING_DPS = 20.0                 # J1 commanded speed above which a fresh reply always moves the encoder
GAP_MS = 6.0                      # a reply interval over 1.5 ticks
STALL_MS = 1.5                    # a pause inside one tick's exchange (normal frame spacing there: < 0.6 ms)
BUNCH_MS = 0.5                    # after a host stall the waiting frames come within this of the pause's end
LATE_MS = 1.0                     # 'command late' / 'drive slow': this much over the usual
MATCH_S = 0.015                   # how far from a tick's clock reading its frames are looked for
EXC_W = 25                        # half width (ticks) of the running median the excursions are measured against


# ----------------------------------------------------------------------------- reading the recordings (pure)
def to_rows(packets, joint_num=6):
    """Telemetry packets of the `can_gaps` recipe -> arrays ordered by tick (other recipes' packets dropped)."""
    from robotclient.rti_client import parse_state_packet
    need = ("tick", "m_p", "j_pc", "j_p", "timestamp", "exec_t")
    rows = {}
    for raw in packets:
        try:
            r = parse_state_packet(raw.decode() if isinstance(raw, bytes) else raw)
        except Exception:
            continue
        if all(k in r for k in need):
            rows[int(r["tick"])] = r
    ticks = sorted(rows)
    d = {"tick": np.array(ticks, dtype=np.int64),
         "ts": np.array([float(rows[k]["timestamp"]) for k in ticks]),
         "exec_ms": np.array([float(rows[k]["exec_t"]) * 1e3 for k in ticks]),
         "period_ms": np.array([float(rows[k].get("period_ms", 0.0)) for k in ticks]),
         "read_t": np.array([float(rows[k].get("can_read_t", np.nan)) for k in ticks]),
         "mode": np.array([str(rows[k].get("mode", "")) for k in ticks])}
    for key in ("j_p", "j_pc", "j_s", "j_t", "m_p"):
        d[key] = np.array([np.resize(np.asarray(rows[k].get(key, np.nan), float), joint_num) for k in ticks]) \
            if ticks else np.zeros((0, joint_num))
    return d


def counters_from_ip_json(text):
    """`ip -j -s -s -d link show <can>` -> the numbers that say whether frames were lost on our side."""
    d = json.loads(text)[0]
    s = d.get("stats64", {})
    rx, tx = s.get("rx", {}), s.get("tx", {})
    li = d.get("linkinfo", {})
    x = li.get("info_xstats", {})
    out = {"state": li.get("info_data", {}).get("state", "?")}
    for k in ("packets", "errors", "dropped", "over_errors", "missed_errors", "fifo_errors", "crc_errors",
              "frame_errors"):
        out["rx_" + k] = int(rx.get(k, 0))
    for k in ("packets", "errors", "dropped", "aborted_errors", "fifo_errors"):
        out["tx_" + k] = int(tx.get(k, 0))
    for k in ("restarts", "bus_error", "arbitration_lost", "error_warning", "error_passive", "bus_off"):
        out[k] = int(x.get(k, 0))
    return out


def counter_delta(before, after):
    if not before or not after:
        return {}
    return {k: after[k] - before[k] for k in after if k != "state" and isinstance(after.get(k), int)
            and isinstance(before.get(k), int)}


def irq_lines(text):
    """/proc/interrupts -> {name: count over all cores} for the SPI controllers and the CAN chip."""
    out = {}
    for ln in text.splitlines():
        if not re.search(r"spi|mcp|can", ln, re.I) or ":" not in ln:
            continue
        head, rest = ln.split(":", 1)
        parts = rest.split()
        nums = []
        for p in parts:
            if p.isdigit():
                nums.append(int(p))
            else:
                break
        out[f"{head.strip()} {parts[-1] if parts else ''}"] = sum(nums)
    return out


# ----------------------------------------------------------------------------- analysis (pure)
def fit_ticks(j_p0, m_p0):
    """J1's motor counts = a x joint rad + b (RTI derives one from the other), least squares over every row; returns
    (a, b, worst residual in counts) or None when J1 never moved enough to tell."""
    ok = np.isfinite(j_p0) & np.isfinite(m_p0)
    if ok.sum() < 50 or np.ptp(j_p0[ok]) < 0.02:
        return None
    A = np.vstack([j_p0[ok], np.ones(int(ok.sum()))]).T
    (a, b), *_ = np.linalg.lstsq(A, m_p0[ok], rcond=None)
    return float(a), float(b), float(np.max(np.abs(A @ np.array([a, b]) - m_p0[ok])))


def moving_mask(d, dt, joint=0):
    """Rows where the joint is commanded faster than MOVING_DPS (5-tick mean of |speed|, from consecutive ticks)."""
    n = d["tick"].size
    v = np.zeros(n)
    if n > 1:
        step = np.diff(d["tick"]).astype(float)
        dv = np.abs(np.degrees(np.diff(d["j_pc"][:, joint])) / (np.maximum(step, 1.0) * dt))
        dv[step != 1] = 0.0
        v[1:] = dv
    sm = np.convolve(v, np.ones(5) / 5.0, mode="same")
    return sm > MOVING_DPS


def stale_runs(d, dt, joint=0):
    """[(row, length)]: runs of consecutive ticks on which RTI got NO new reply from the joint's drive while it moved —
    the drive's encoder position, speed and torque all exactly as on the tick before."""
    if d["tick"].size < 2:
        return []
    mov = moving_mask(d, dt, joint)
    same = ((d["m_p"][1:, joint] == d["m_p"][:-1, joint]) & (d["j_s"][1:, joint] == d["j_s"][:-1, joint])
            & (d["j_t"][1:, joint] == d["j_t"][:-1, joint]) & (np.diff(d["tick"]) == 1) & mov[1:])
    runs, k = [], 0
    idx = np.flatnonzero(same) + 1
    while k < idx.size:
        start, ln = idx[k], 1
        while k + ln < idx.size and idx[k + ln] == start + ln:
            ln += 1
        runs.append((int(start), int(ln)))
        k += ln
    return runs


def running_median(x, w=EXC_W):
    if x.size == 0:
        return x
    pad = np.pad(x, (w, w), mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(pad, 2 * w + 1), axis=1)


def burst_at(c, starts, i):
    """Tick i's exchange: from J1's command starts[i] to the last JOINT reply before J1's next command (the polls after
    the joints are left out: a temperature reply may come 3 ms later by design). Returns (start, end, the longest pause
    inside it in s, frames within BUNCH_MS after the frame that ended the pause — the rest of the tick's frames)."""
    t = c["t"]
    s = starts[i]
    nxt = starts[i + 1] if i + 1 < starts.size else s + 0.004
    lo, hi = np.searchsorted(t, s, "left"), np.searchsorted(t, nxt, "left")
    jr = np.flatnonzero((c["cmd"][lo:hi] == RESP_DATA_PACK_1) & (c["node"][lo:hi] < JOINT_NODES))
    if jr.size == 0:
        return s, s, 0.0, 0
    tt = t[lo:lo + jr[-1] + 1]
    if tt.size < 2:
        return s, tt[-1], 0.0, 0
    g = np.diff(tt)
    j = int(np.argmax(g))
    pend = tt[j + 1]
    return s, tt[-1], float(g[j]), int(np.sum((t[lo + j + 2:hi] - pend) <= BUNCH_MS * 1e-3))


def bursts(c):
    """Every tick's exchange in `c` (see burst_at)."""
    starts = node_frames(c, 0, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))[0]
    return [burst_at(c, starts, i) for i in range(starts.size)]


def missing_replies(tc, tr, w0, w1):
    """Replies missing for one node's commands in [w0, w1): each command owns the replies from 2 ms before its echo
    (our driver may stamp the echo after the reply) to 2 ms before the next command; an empty window is a reply short,
    a window with two is a late reply found — so missing = empty windows - extra replies. Replies are looked up in the
    whole log: no edge effects."""
    sel = np.flatnonzero((tc >= w0) & (tc < w1))
    if sel.size == 0:
        return 0
    lo = tc[sel] - 0.002
    hi = np.append(tc[1:], tc[-1] + 0.02)[sel] - 0.002
    n_in = np.searchsorted(tr, hi) - np.searchsorted(tr, lo)
    return max(int(np.sum(n_in == 0)) - int(np.sum(np.maximum(n_in - 1, 0))), 0)


def align(d, c, fit, offset, moving):
    """Per row (moving rows only): J1's command of that tick (matched by content, nearest in time) and the reply RTI
    used (the reply whose position equals the row's m_p). Times in CLOCK_REALTIME; NaN where not found."""
    n = d["tick"].size
    ts = d["ts"] + offset
    tc, pc = node_frames(c, 0, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))
    tr, pr = node_frames(c, 0, (RESP_DATA_PACK_1,))
    tx_t, rx_t = np.full(n, np.nan), np.full(n, np.nan)
    if fit is None:
        return ts, tx_t, rx_t
    a, b, _ = fit
    for k in np.flatnonzero(moving):
        pred = a * d["j_pc"][k, 0] + b
        lo, hi = np.searchsorted(tc, ts[k] - 0.004), np.searchsorted(tc, ts[k] + 0.012)
        cand = np.flatnonzero(np.abs(pc[lo:hi] - pred) <= 2.5)
        if cand.size:
            tx_t[k] = tc[lo + cand[np.argmin(np.abs(tc[lo + cand] - ts[k]))]]
        lo, hi = np.searchsorted(tr, ts[k] - MATCH_S), np.searchsorted(tr, ts[k] + 0.001)
        cand = np.flatnonzero(pr[lo:hi] == d["m_p"][k, 0])
        if cand.size:
            rx_t[k] = tr[lo + cand[-1]]
    return ts, tx_t, rx_t


def classify(runs, d, c, ts, tx_t, rx_t, dt):
    """Each stale run, with the numbers that tell its causes apart, and a first-sort label."""
    tc, _ = node_frames(c, 0, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))
    tr, _ = node_frames(c, 0, (RESP_DATA_PACK_1,))
    tx_rel = (tx_t - ts) * 1e3
    usual_tx = float(np.nanmedian(tx_rel)) if np.isfinite(tx_rel).any() else np.nan
    sender = {}
    for k in np.flatnonzero(np.isfinite(tx_t)):
        sender[float(tx_t[k])] = int(k)
    e = np.degrees(d["j_pc"][:, 0] - d["j_p"][:, 0])
    dev = np.abs(e - running_median(e))
    out = []
    for k0, ln in runs:
        r = {"row": k0, "tick": int(d["tick"][k0]), "len": ln, "t_mono": float(d["ts"][k0]),
             # a row carries the PREVIOUS tick's execution time (RTI computes it after the packet is built): rows
             # k0-1 .. k0+ln = ticks k0-2 .. k0+ln-1, the tick that should have got the reply out and those that missed it
             "exec_max_ms": float(np.max(d["exec_ms"][max(k0 - 1, 0):k0 + ln + 1])),
             "excursion_deg": float(np.max(dev[max(k0 - 2, 0):min(k0 + ln + 40, dev.size)])),
             "late_ms": np.nan, "tx_ms": np.nan, "resp_ms": np.nan, "missing": 0, "pause_ms": np.nan, "bunch": 0}
        t_prev = rx_t[k0 - 1] if k0 >= 1 else np.nan
        if not np.isfinite(t_prev):
            r["label"] = "unmatched"
            out.append(r)
            continue
        j = np.searchsorted(tr, t_prev, "right")
        if j >= tr.size or tr[j] - t_prev > 0.2:
            r["label"] = "reply lost"
            r["missing"] = int(np.sum((tc > t_prev) & (tc < t_prev + 0.2)))
            out.append(r)
            continue
        t_next = tr[j]
        cmds = tc[(tc > t_prev) & (tc < t_next)]
        r["missing"] = max(int(cmds.size) - 1, 0)
        r["late_ms"] = (t_next - ts[k0]) * 1e3
        if cmds.size:
            c_next = cmds[-1]
            r["resp_ms"] = (t_next - c_next) * 1e3
            ks = sender.get(float(c_next))
            if ks is not None:
                r["tx_ms"] = (c_next - ts[ks]) * 1e3
            _, _, pause, bunch = burst_at(c, tc, int(np.searchsorted(tc, c_next)))
            r["pause_ms"], r["bunch"] = pause * 1e3, bunch
        if r["missing"] > 0:
            r["label"] = "reply lost"
        elif r["pause_ms"] > STALL_MS and r["bunch"] >= 2:
            r["label"] = "host stall"
        elif r["tx_ms"] > usual_tx + LATE_MS:
            r["label"] = "command late"
        elif r["resp_ms"] > LATE_MS:
            r["label"] = "drive slow"
        else:
            r["label"] = "read early"
        out.append(r)
    return out


def can_metrics(c, w0, w1):
    """The bus in [w0, w1) (CLOCK_REALTIME)."""
    m = (c["t"] >= w0) & (c["t"] < w1)
    cc = {k: v[m] for k, v in c.items()}
    nodes = {}
    for j in range(JOINT_NODES):
        tc_all = node_frames(c, j, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))[0]
        tr_all = node_frames(c, j, (RESP_DATA_PACK_1,))[0]
        tcj = node_frames(cc, j, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))[0]
        trj = node_frames(cc, j, (RESP_DATA_PACK_1,))[0]
        if tcj.size == 0 and trj.size == 0:
            continue
        gaps = np.diff(trj) * 1e3
        nodes[f"J{j + 1}"] = {"cmd": int(tcj.size), "rep": int(trj.size),
                              "missing": missing_replies(tc_all, tr_all, w0, w1),
                              "rep_gaps": int(np.sum(gaps > GAP_MS)),
                              "rep_gap_max_ms": float(gaps.max()) if gaps.size else 0.0}
    tc = node_frames(cc, 0, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))[0]
    tr = node_frames(cc, 0, (RESP_DATA_PACK_1,))[0]
    resp = []
    for t in tr:
        i = np.searchsorted(tc, t) - 1
        if i >= 0 and t - tc[i] < 0.02:
            resp.append((t - tc[i]) * 1e3)
    bs = bursts(cc)
    pauses = np.array([b[2] for b in bs]) * 1e3
    stalls = int(sum(1 for b in bs if b[2] * 1e3 > STALL_MS and b[3] >= 2))
    return {"frames": int(m.sum()), "nodes": nodes, "j1_resp_ms": _stats(resp),
            "exchange_ms": _stats([(b[1] - b[0]) * 1e3 for b in bs]), "pause_ms": _stats(pauses),
            "pauses_over": int(np.sum(pauses > STALL_MS)), "host_stalls": stalls}


def tick_commands(tc, ts):
    """Each row's own command among a node's command times tc. A row owns the commands within +/-2 ms of where its
    command usually leaves (its clock reading + the median offset of the first command after a reading, this block):
    the windows tile the time axis, so a frame that leaves a little EARLY (before the reading — seen in the old order
    when RTI's thread yields mid-tick) or late still lands in its own row. A row with none whose next row holds two gets
    the first of them (its own, sent more than 2 ms late). Local: one missing frame shifts nothing else (a chained rule
    did, on the arm's log). NaN where a row has none."""
    n = ts.size
    cmd = np.full(n, np.nan)
    fin = np.isfinite(ts)
    if tc.size == 0 or not fin.any():
        return cmd
    first = np.searchsorted(tc, ts[fin], "right")
    ok = first < tc.size
    if not ok.any():
        return cmd
    off = float(np.median(tc[first[ok]] - ts[fin][ok]))
    e = np.where(fin, ts + off, np.nan)
    lo = np.searchsorted(tc, np.where(fin, e - 0.002, np.inf), "left")
    hi = np.searchsorted(tc, np.where(fin, e + 0.002, np.inf), "left")
    cnt = np.where(fin, hi - lo, 0)
    for k in range(n):
        if cnt[k] == 0:
            if k + 1 < n and cnt[k + 1] >= 2:
                cmd[k] = tc[lo[k + 1]]               # its own, late — the next row keeps the one nearest its time
            continue
        if cnt[k] == 1:
            cmd[k] = tc[lo[k]]
        else:
            cand = tc[lo[k]:hi[k]]
            if k > 0 and cnt[k - 1] == 0:
                cand = cand[1:]                      # the first went to the previous row
            cmd[k] = cand[np.argmin(np.abs(cand - e[k]))]
    return cmd


def reply_freshness(c, read_t, tick, ts, gaps=()):
    """Per joint, from the CAN log, when each of RTI's CAN reads ran (read_t) and each row's clock reading (ts; both
    CLOCK_REALTIME, one per row): missed = reads at which the reply to the PREVIOUS tick's command (the joint's first
    command around that row's read — the tick's own in both orders, tick_anchor) was not in yet, or never came: that tick
    the joint's data is a tick old. (Until 2026-10-12 this counted reads that found NO NEW reply at all, which misses
    lateness that lasts: each read then finds the previous tick's late reply — option B under load read J6 12.8 %
    that way, 40 % this way. The old order's numbers hardly change: its lateness never lasted.) margin = the reply
    minus the read, median (ms; negative = in time); age = how old the newest reply was at the read (ms)."""
    ok = np.isfinite(read_t) & (read_t > 0)
    for g0, g1 in gaps:                      # a read inside or right after a candump gap cannot be judged
        ok &= ~((read_t > g0) & (read_t < g1 + 0.005))
    pair = (np.diff(tick) == 1) & ok[1:] & ok[:-1]
    out = {}
    for j in range(JOINT_NODES):
        tr = node_frames(c, j, (RESP_DATA_PACK_1,))[0]
        tc = node_frames(c, j, (CMD_DATA_PACK_1, CMD_DATA_PACK_PD))[0]
        if tr.size == 0 or tc.size == 0 or not ok.any():
            continue
        prev = tick_commands(tc, tick_anchor(read_t, ts))[:-1]                    # the previous row's own command
        valid = pair & np.isfinite(prev)
        ir = np.searchsorted(tr, np.where(np.isfinite(prev), prev, 0.0), "right")
        rep = np.where(ir < tr.size, tr[np.clip(ir, None, tr.size - 1)], np.inf)
        rep = np.where(rep - prev < 0.02, rep, np.inf)                              # no reply within 20 ms: lost
        late = valid & (rep > read_t[1:])
        missed = int(late.sum())
        rel = (rep - read_t[1:])[valid & np.isfinite(rep)] * 1e3
        idx = np.searchsorted(tr, np.where(ok, read_t, 0.0), "right")
        good = ok & (idx > 0)
        age = (read_t[good] - tr[idx[good] - 1]) * 1e3
        out[f"J{j + 1}"] = {"reads": int(valid.sum()), "missed": missed,
                            "missed_pct": 100.0 * missed / valid.sum() if valid.sum() else np.nan,
                            "margin_ms": float(np.median(rel)) if rel.size else np.nan,
                            "age_ms": _stats(age), "age_p5": float(np.percentile(age, 5)) if age.size else np.nan}
    return out


def analyse(run, d, c):
    dt = float(run.get("delta_t", 0.004))
    offset = float(np.mean([o for o in run.get("clock_offsets", []) if o is not None] or [0.0]))
    fit = fit_ticks(d["j_p"][:, 0], d["m_p"][:, 0]) if d["tick"].size else None
    blocks = []
    for blk in run["blocks"]:
        sel = (d["tick"] >= (blk.get("tick0") or 0)) & (d["tick"] <= (blk.get("tick1") or -1))
        dd = {k: v[sel] for k, v in d.items()}
        mov = moving_mask(dd, dt) if dd["tick"].size else np.zeros(0, bool)
        runs = stale_runs(dd, dt) if blk["kind"] != "idle" else []
        ts, tx_t, rx_t = align(dd, c, fit, offset, mov)
        detail = classify(runs, dd, c, ts, tx_t, rx_t, dt)
        gaps = candump_gaps(c, blk["t_start"], blk["t_end"])
        for r in detail:                     # a stale run in a candump gap is not judged from the log
            t_run = r["t_mono"] + offset
            if any(g0 - 0.004 < t_run < g1 + 0.004 for g0, g1 in gaps):
                r["label"] = "candump gap"
        missing_ticks = int(np.sum(np.diff(dd["tick"]) - 1)) if dd["tick"].size > 1 else 0
        # J1's loop delay: the reply a row read -> the command computed from it. Each row pairs the tick's measurement
        # with the tick's command in both orders, and the command computed from row k's measurement is row k+1's in
        # the old order, row k's with option B. Option A's old '_wait' blocks fit neither: not judged.
        lag = 0 if blk.get("read_first") else 1
        loop = []
        if not blk["kind"].endswith("_wait") and dd["tick"].size > lag:
            k = np.arange(dd["tick"].size - lag)
            okk = ((dd["tick"][k + lag] - dd["tick"][k]) == lag) & np.isfinite(rx_t[k]) & np.isfinite(tx_t[k + lag])
            loop = (tx_t[k + lag][okk] - rx_t[k][okk]) * 1e3
        moving_s = float(mov.sum()) * dt
        res = {"name": blk["name"], "kind": blk["kind"], "ok": blk.get("ok", True), "rows": int(dd["tick"].size),
               "fresh": reply_freshness(c, dd["read_t"], dd["tick"], dd["ts"] + offset, gaps),
               "telemetry_missing": missing_ticks, "moving_s": moving_s, "stale_runs": len(runs),
               "stale_per_min": (len(runs) / moving_s * 60.0) if moving_s > 0 else np.nan,
               "stale_longest": max((r[1] for r in runs), default=0),
               "excursion_max_deg": max((r["excursion_deg"] for r in detail), default=0.0),
               "labels": {}, "runs": detail,
               "exec_ms": _stats(dd["exec_ms"]), "exec_over_3ms": int(np.sum(dd["exec_ms"] > 3.0)),
               "period_max_ms": float(np.max(dd["period_ms"])) if dd["tick"].size else np.nan,
               "tx_rel_ms": _stats((tx_t - ts) * 1e3), "rx_rel_ms": _stats((rx_t - ts) * 1e3),
               "matched": int(np.isfinite(tx_t).sum()), "moving_rows": int(mov.sum()),
               "read_first": bool(blk.get("read_first")), "loop_ms": _stats(loop),
               "loop_joints": ({} if blk["kind"].endswith("_wait") else
                               loop_delay_per_joint(c, dd["ts"] + offset, dd["read_t"], dd["tick"], lag, gaps)),
               "can": can_metrics(c, blk["t_start"], blk["t_end"]),
               "read_rel_ms": _stats((dd["read_t"] - ts) * 1e3),
               "tx_read_ms": _stats((tx_t - dd["read_t"]) * 1e3),          # J1's command after the tick's read
               "candump_gaps": candump_gaps(c, blk["t_start"], blk["t_end"]),
               "counters": counter_delta(blk.get("can_before"), blk.get("can_after")),
               "irq": {k: blk["irq_after"].get(k, 0) - v for k, v in blk.get("irq_before", {}).items()}
               if blk.get("irq_after") else {}}
        for r in detail:
            res["labels"][r["label"]] = res["labels"].get(r["label"], 0) + 1
        blocks.append(res)
    return {"fit": fit, "offset": offset, "blocks": blocks}


def _f(v, nd=2):
    return "-" if v is None or (isinstance(v, float) and not math.isfinite(v)) else f"{v:.{nd}f}"


def print_report(rep, title=""):
    if title:
        print(title)
    fit = rep["fit"]
    print(f"J1: {fit[0]:.1f} counts per rad, worst residual {fit[2]:.2f} counts (lines the frames up with the ticks)"
          if fit else "J1 never moved enough to line the frames up with the ticks: the timing columns stay empty.")
    print("\nPer block. Line 1 — J1 with no new reply while moving (stale runs), RTI's loop. Line 2 — the bus: J1's\n"
          "command echo and the reply RTI used, relative to RTI's clock reading after its CAN read (ms, median); J1's\n"
          "response time; one tick's exchange; pauses inside it; replies missing per joint; the counters' change.")
    for b in rep["blocks"]:
        cm, ex, cnt = b["can"], b["exec_ms"], b["counters"]
        print(f"\n{b['name']}{'' if b['ok'] else '  (FAILED part-way)'}")
        lost = f"; {b['telemetry_missing']} telemetry packets missing" if b["telemetry_missing"] else ""
        print(f"  stale: {b['stale_runs']} run(s) in {b['moving_s']:.1f} s of motion ({_f(b['stale_per_min'], 1)}/min), "
              f"longest {b['stale_longest']} tick(s), worst excursion {_f(b['excursion_max_deg'], 3)} deg "
              f"{b['labels'] or ''}{lost} | loop exec {_f(ex.get('med'))} / {_f(ex.get('p99'))} / {_f(ex.get('max'))} ms "
              f"(med / p99 / max), {b['exec_over_3ms']} tick(s) over 3 ms, longest period {_f(b['period_max_ms'])} ms")
        miss = " ".join(f"J{j + 1}:{cm['nodes'][f'J{j + 1}']['missing']}" for j in range(JOINT_NODES)
                        if f"J{j + 1}" in cm["nodes"])
        print(f"  bus: J1 command {_f(b['tx_rel_ms'].get('med'))} (p99 {_f(b['tx_rel_ms'].get('p99'))}), reply used "
              f"{_f(b['rx_rel_ms'].get('med'))} ms ({b['matched']}/{b['moving_rows']} moving ticks lined up) | response "
              f"{_f(cm['j1_resp_ms'].get('med'))} / {_f(cm['j1_resp_ms'].get('max'))} ms | exchange "
              f"{_f(cm['exchange_ms'].get('med'))} / {_f(cm['exchange_ms'].get('p99'))} ms (med / p99), pauses > "
              f"{STALL_MS:g} ms {cm['pauses_over']}, host stalls {cm['host_stalls']} | missing {miss} | RX overruns "
              f"{cnt.get('rx_over_errors', '-')}, bus errors {cnt.get('bus_error', '-')}, TX errors "
              f"{cnt.get('tx_errors', '-')}, error-passive {cnt.get('error_passive', '-')}, bus-off {cnt.get('bus_off', '-')}")
        lj = b.get("loop_joints") or {}
        lp = b.get("loop_ms") or {}
        if lj:
            print(f"  loop delay ({'option B, read first' if b.get('read_first') else 'old order'}; a joint's reply -> "
                  f"the command computed from it, median / p99 ms): "
                  + "  ".join(f"{k} {_f(v.get('med'))}/{_f(v.get('p99'))}" for k, v in lj.items())
                  + (f"  | J1 by content (moving ticks): {_f(lp.get('med'))}/{_f(lp.get('p99'))}" if lp.get("n") else ""))
        fr = b.get("fresh") or {}
        cg = b.get("candump_gaps") or []
        if cg:
            print(f"  ({len(cg)} candump gap(s), {sum(1e3 * (g1 - g0) for g0, g1 in cg):.0f} ms in all — candump missed "
                  f"frames there; those reads and stale runs are not judged)")
        if fr:
            print(f"  reads: RTI's CAN read {_f(b['read_rel_ms'].get('med'))} ms from its clock reading, J1's command "
                  f"{_f((b.get('tx_read_ms') or {}).get('med'))} ms after the read | per joint, "
                  f"replies missed / age of the newest reply at the read (median, p5..p99 ms): "
                  + "  ".join(f"{k} {v['missed']} / {_f(v['age_ms'].get('med'), 1)} ({_f(v['age_p5'], 1)}.."
                              f"{_f(v['age_ms'].get('p99'), 1)})" for k, v in fr.items()))
    print("\nStale runs (the first 60): late = the refreshing reply after RTI's clock reading; tx = its command after the "
          "sending tick's reading; resp = reply after command; pause = longest inside that tick's exchange (ms)")
    print(f"  {'block':15s} {'tick':>9s} {'len':>4s} {'exc_deg':>8s} {'late':>6s} {'tx':>6s} {'resp':>6s} "
          f"{'missing':>7s} {'pause':>6s} {'bunch':>5s} {'exec_max':>8s}  label")
    n = 0
    for b in rep["blocks"]:
        for r in b["runs"]:
            if n >= 60:
                break
            n += 1
            print(f"  {b['name']:15s} {r['tick']:9d} {r['len']:4d} {_f(r['excursion_deg'], 3):>8s} {_f(r['late_ms']):>6s} "
                  f"{_f(r['tx_ms']):>6s} {_f(r['resp_ms']):>6s} {r['missing']:7d} {_f(r['pause_ms']):>6s} "
                  f"{r['bunch']:5d} {_f(r['exec_max_ms']):>8s}  {r['label']}")
    if n == 0:
        print("  none")
    print("\nPer kind (all its blocks):")
    for kind in ("idle",) + KINDS:
        bb = [b for b in rep["blocks"] if b["kind"] == kind]
        if not bb:
            continue
        mv = sum(b["moving_s"] for b in bb)
        nr = sum(b["stale_runs"] for b in bb)
        labels = {}
        for b in bb:
            for k, v in b["labels"].items():
                labels[k] = labels.get(k, 0) + v
        over = sum(b["counters"].get("rx_over_errors", 0) for b in bb)
        stalls = sum(b["can"]["host_stalls"] for b in bb)
        frames = sum(b["can"]["frames"] for b in bb)
        rate = f" ({nr / mv * 60:.1f}/min)" if mv > 0 else ""
        print(f"  {kind:12s} {len(bb)} block(s), {mv:5.1f} s moving: {nr} stale run(s){rate} {labels or ''}; "
              f"RX overruns {over}, host stalls {stalls} in {frames} frames")
        miss = {}
        for b in bb:
            for k, v in (b.get("fresh") or {}).items():
                m = miss.setdefault(k, [0, 0])
                m[0] += v["missed"]
                m[1] += v["reads"]
        if miss:
            print(f"  {'':12s} replies missed at the read: " + "  ".join(
                f"{k} {m[0]}/{m[1]}" for k, m in miss.items()))
        lj = {}
        for b in bb:
            for k, v in (b.get("loop_joints") or {}).items():
                if v.get("n"):
                    lj.setdefault(k, []).append(v["med"])
        if lj:
            print(f"  {'':12s} loop delay, median per joint (ms): " + "  ".join(
                f"{k} {np.median(v):.2f}" for k, v in lj.items()))


# ----------------------------------------------------------------------------- self-test
def _synth(seed=1, read_first=False, stamp_after_send=False):
    """A recording with known events, built the way RTI and the bus work: each tick RTI sends its commands, reads what
    arrived, then reads its clock (the telemetry stamp); the commands go out after it and the replies come back
    ~0.23 ms after each; the reply RTI uses at tick k is the newest one that arrived before its read.
    Every 10th tick a temperature poll after the joints, answered 2.5 ms later. Events: tick 199 a fresh reply whose speed and current equal the one before (not stale: the position moved), 300 a host stall (J1's reply and every later frame of that exchange 3 ms late, in a bunch), 600 the
    whole exchange 2.8 ms late (command late), 900 J1's reply lost, 1100 J1's reply alone 2.8 ms late (drive slow).
    read_first: option B's timing — the read 0.1 ms into the tick, the clock reading 0.6, the commands from 1.0 (the old
    order: 0.4 / 0.9 / 1.5). stamp_after_send: option B as built on 2026-10-12 evening (§9.10) — the history entry,
    and so the clock reading, after the commands left (1.8 ms)."""
    t_read, t_stamp, t_send = (0.0001, 0.0006, 0.0010) if read_first else (0.0004, 0.0009, 0.0015)
    if stamp_after_send:
        t_stamp = 0.0018
    rng = np.random.default_rng(seed)
    dt, n, nodes = 0.004, 1300, 3
    mono0, offset = 5000.0, 1.7e9
    a, b = -16500.0, 40000.0
    q = lambda t: 0.5 * math.sin(2 * math.pi * t / 1.5)          # noqa: E731
    frames = []                                                   # (t realtime, node, cmd, pos, speed, current)
    reply = {}                                                    # tick -> (t realtime, pos, speed, current)
    for k in range(n):
        T = mono0 + k * dt + offset
        base = T + t_send
        delays = [0.0] * (2 * nodes)
        if k == 600:
            delays = [0.0028] * (2 * nodes)
        if k == 300:
            delays = [0.0] + [0.003] * (2 * nodes - 1)
        for j in range(nodes):
            tcmd = base + 0.00031 * j + delays[2 * j]
            pos = round(a * q(k * dt) + b) if j == 0 else 1000 * j
            frames.append((tcmd, j, CMD_DATA_PACK_1, pos, 0, 0))
            if j == 0 and k == 900:
                continue
            trep = base + 0.00031 * j + 0.00023 + delays[2 * j + 1] + (0.0028 if (j == 0 and k == 1100) else 0.0)
            rp = round(a * q(k * dt - 0.006) + b) if j == 0 else 1000 * j + k
            sp = k - 1 if k == 199 else k        # 199: a FRESH reply whose speed and current repeat 198's
            frames.append((trep, j, RESP_DATA_PACK_1, rp, sp, 2 * sp))
            if j == 0:
                reply[k] = (trep, rp, sp, 2 * sp)
        if k % 10 == 5:                   # a temperature poll after the joints, answered 2.5 ms later (as on the arm)
            tp = base + 0.00031 * nodes + 0.0001
            frames.append((tp, k % nodes, 23, None, 0, 0))
            frames.append((tp + 0.0025, k % nodes, 23, 0x26, 0, 0))
    lines = []
    for t, j, cm, p, s, cur in sorted(frames, key=lambda f: f[0]):
        if cm == 23:
            lines.append(f"({t:.6f}) can0 {(j << 7) | (cm << 1):03X}#{'R' if p is None else f'{p:04X}'}")
            continue
        data = (int(p) & 0xFFFFFF).to_bytes(3, "big") + (int(s) & 0xFFFFFF).to_bytes(3, "big") \
            + (int(cur) & 0xFFFF).to_bytes(2, "big")
        lines.append(f"({t:.6f}) can0 {(j << 7) | (cm << 1):03X}#{data.hex().upper()}")
    rows = {k: [] for k in ("tick", "ts", "exec_ms", "period_ms", "mode", "j_p", "j_pc", "j_s", "j_t", "m_p", "read_t")}
    got = sorted(reply.values())
    rt = np.array([g[0] for g in got])
    for k in range(1, n):
        T = mono0 + k * dt
        i = np.searchsorted(rt, T + offset + t_read) - 1             # the read
        _, rp, sp, cu = got[i]
        rows["tick"].append(10000 + k)
        rows["ts"].append(round(T + t_stamp, 6))
        rows["read_t"].append(T + offset + t_read)
        rows["exec_ms"].append(round(1.8 + rng.uniform(0, 0.3), 3))
        rows["period_ms"].append(4.0)
        rows["mode"].append("RTI")
        rows["j_pc"].append([round(q(k * dt), 5)] + [0.0] * 5)
        rows["j_p"].append([round((rp - b) / a, 5)] + [0.0] * 5)
        rows["j_s"].append([sp * 1e-3] + [0.0] * 5)
        rows["j_t"].append([cu * 1e-3] + [0.0] * 5)
        rows["m_p"].append([float(rp)] + [0.0] * 5)
    d = {k: np.asarray(v) for k, v in rows.items()}
    return lines, d, offset, (a, b)


def selftest():
    ok_all = True

    def chk(name, cond, detail=""):
        nonlocal ok_all
        ok_all &= bool(cond)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + ("" if cond else f"  [{detail}]"))

    lines, d, offset, (a, b) = _synth()
    c = parse_candump(lines + ["(1.0) can0 0AE#R", "garbage", "(2.0) can0 12345678#00"])
    chk("candump: every synthetic frame parsed, the RTR frame kept, the extended / bad lines dropped",
        c["t"].size == len(lines) + 1, c["t"].size)
    neg = parse_candump(["(3.0) can0 006#FFFFFE00000A0005", "(3.1) can0 084#0000640000000000"])
    chk("candump: the 24-bit position decoded signed (J1's reply -2, J2's command 100), node and command from the id",
        list(neg["pos"]) == [-2.0, 100.0] and list(neg["node"]) == [0, 1]
        and list(neg["cmd"]) == [RESP_DATA_PACK_1, CMD_DATA_PACK_1], neg)
    fit = fit_ticks(d["j_p"][:, 0], d["m_p"][:, 0])
    chk("the counts-per-rad fit recovers the drive's scale", fit and abs(fit[0] - a) < 1.0 and fit[2] < 1.0, fit)
    runs = stale_runs(d, 0.004)
    rows = {int(d["tick"][k]) - 10000: ln for k, ln in runs}
    chk("stale runs exactly at the four events (the tick after each), one tick long; a fresh reply repeating only "
        "speed and current is not one",
        rows == {301: 1, 601: 1, 901: 1, 1101: 1}, rows)
    run = {"delta_t": 0.004, "clock_offsets": [offset], "blocks": [
        {"name": "synthetic", "kind": "passthrough", "t_start": -np.inf, "t_end": np.inf, "tick0": 0, "tick1": 10 ** 9}]}
    rep = analyse(run, d, c)
    blk = rep["blocks"][0]
    lab = {r["tick"] - 10000: r["label"] for r in blk["runs"]}
    chk("labels: 300 host stall, 600 command late, 900 reply lost, 1100 drive slow",
        lab == {301: "host stall", 601: "command late", 901: "reply lost", 1101: "drive slow"}, lab)
    chk("every moving row's command and reply found", blk["matched"] >= blk["moving_rows"] - 2,
        (blk["matched"], blk["moving_rows"]))
    chk("J1's command goes out 0.6 ms after RTI's clock reading (median)",
        abs(blk["tx_rel_ms"]["med"] - 0.6) < 0.01, blk["tx_rel_ms"])
    chk("the reply RTI used came 3.17 ms before it (median)", abs(blk["rx_rel_ms"]["med"] + 3.17) < 0.01,
        blk["rx_rel_ms"])
    cm = blk["can"]
    chk("bus: one J1 reply missing, none on J2 / J3",
        cm["nodes"]["J1"]["missing"] == 1 and cm["nodes"]["J2"]["missing"] == 0 and cm["nodes"]["J3"]["missing"] == 0,
        cm["nodes"])
    chk("bus: exactly one host stall (300), J1's lone late reply (1100) is not one", cm["host_stalls"] == 1,
        cm["host_stalls"])
    chk("bus: pauses over 1.5 ms inside the joints' exchange only at 300 and 1100 (the polls' slow replies after the "
        "joints are not counted)", cm["pauses_over"] == 2, cm["pauses_over"])
    chk("bus: J1's usual response time 0.23 ms", abs(cm["j1_resp_ms"]["med"] - 0.23) < 0.01, cm["j1_resp_ms"])
    r300 = [r for r in blk["runs"] if r["tick"] == 10301][0]
    chk("the stall's pause (~3.2 ms) and its bunch measured", 3.0 < r300["pause_ms"] < 3.4 and r300["bunch"] >= 2,
        r300)
    fr = blk["fresh"]
    chk("per joint, reads the previous tick's reply had not reached: J1 4 (300 / 600 / 900 / 1100), J2 and J3 2 (the stall and the late "
        "exchange delayed theirs too)", (fr["J1"]["missed"], fr["J2"]["missed"], fr["J3"]["missed"]) == (4, 2, 2),
        {k: v["missed"] for k, v in fr.items()})
    sync = []                                  # 20 ticks of J1 alone; ticks 5-14: every reply 0.2 ms after the next read
    for k in range(20):
        T = 100.0 + 0.004 * k
        tr_ = T + 0.0015 + 0.00023 if not 5 <= k <= 14 else T + 0.004 + 0.0006
        sync += [f"({T + 0.0015:.6f}) can0 004#0000000000000000", f"({tr_:.6f}) can0 006#0000000000000000"]
    cs_ = parse_candump(sorted(sync, key=lambda ln: float(ln[1:ln.index(")")])))
    ks = np.arange(20)
    fr_s = reply_freshness(cs_, 100.0 + 0.004 * ks + 0.0004, ks, 100.0 + 0.004 * ks + 0.0009)
    chk("late replies that LAST (ticks 5-14 each 0.2 ms after the read): 10 stale reads, not 1 (each read found the "
        "previous tick's late reply — the old count's blind spot)", fr_s["J1"]["missed"] == 10, fr_s["J1"])
    early = []                                 # 20 ticks of J1; tick 10's command leaves 0.7 ms BEFORE its clock reading
    for k in range(20):
        T = 100.0 + 0.004 * k
        tc_ = T + 0.0015 if k != 10 else T + 0.0002
        early += [f"({tc_:.6f}) can0 004#0000000000000000", f"({tc_ + 0.00023:.6f}) can0 006#0000000000000000"]
    ce_ = parse_candump(sorted(early, key=lambda ln: float(ln[1:ln.index(")")])))
    fr_e = reply_freshness(ce_, 100.0 + 0.004 * ks + 0.0004, ks, 100.0 + 0.004 * ks + 0.0009)
    chk("a command that leaves BEFORE its tick's clock reading (RTI yielding mid-tick, seen in the old order) is still its "
        "tick's: no false late reply", fr_e["J1"]["missed"] == 0 and fr_e["J1"]["reads"] == 19, fr_e["J1"])
    chk("the newest J1 reply is 2.67 ms old at the read (median)", abs(fr["J1"]["age_ms"]["med"] - 2.67) < 0.01,
        fr["J1"]["age_ms"])
    chk("RTI's read sits 0.5 ms before its clock reading", abs(blk["read_rel_ms"]["med"] + 0.5) < 0.01,
        blk["read_rel_ms"])
    chk("loop delay, old order: J1's reply (2.27 ms before the tick) -> the NEXT tick's command (1.5 ms into it) = "
        "7.77 ms", abs(blk["loop_ms"]["med"] - 7.77) < 0.01 and not blk["read_first"], blk["loop_ms"])
    lines_b, d_b, off_b, _ = _synth(read_first=True)
    run_b = {"delta_t": 0.004, "clock_offsets": [off_b], "blocks": [
        {"name": "synthetic_rf", "kind": "passthrough_rf", "read_first": True, "t_start": -np.inf, "t_end": np.inf,
         "tick0": 0, "tick1": 10 ** 9}]}
    blk_b = analyse(run_b, d_b, parse_candump(lines_b))["blocks"][0]
    chk("loop delay, option B: J1's reply (2.77 ms before the tick) -> the SAME tick's command (1.0 ms into it) = "
        "3.77 ms", abs(blk_b["loop_ms"]["med"] - 3.77) < 0.01 and blk_b["read_first"], blk_b["loop_ms"])
    lj, lj_b = blk["loop_joints"], blk_b["loop_joints"]
    chk("loop delay by time, every joint: 7.77 ms old order, 3.77 ms option B (J1..J3 of the synthetic bus)",
        set(lj) == {"J1", "J2", "J3"} and all(abs(v["med"] - 7.77) < 0.01 for v in lj.values())
        and all(abs(v["med"] - 3.77) < 0.01 for v in lj_b.values()), ({k: v.get("med") for k, v in lj.items()},
                                                                       {k: v.get("med") for k, v in lj_b.items()}))
    chk("...J1 by time == J1 by content (the two methods agree)", abs(lj["J1"]["med"] - blk["loop_ms"]["med"]) < 0.01
        and abs(lj_b["J1"]["med"] - blk_b["loop_ms"]["med"]) < 0.01, (lj["J1"], blk["loop_ms"]))
    lines_s, d_s, off_s, _ = _synth(read_first=True, stamp_after_send=True)
    run_s = {"delta_t": 0.004, "clock_offsets": [off_s], "blocks": [dict(run_b["blocks"][0], name="synthetic_rf_late")]}
    blk_s = analyse(run_s, d_s, parse_candump(lines_s))["blocks"][0]
    same_fresh = {k: v["missed"] for k, v in blk_s["fresh"].items()} == {k: v["missed"] for k, v in blk_b["fresh"].items()}
    chk("option B with the clock reading AFTER the commands (the send-early order): the same late replies and the same "
        "3.77 ms on every joint — the tick's read anchors its commands, not the stamp", same_fresh
        and all(abs(v["med"] - 3.77) < 0.01 for v in blk_s["loop_joints"].values())
        and abs(blk_s["loop_ms"]["med"] - 3.77) < 0.01,
        ({k: v["missed"] for k, v in blk_s["fresh"].items()}, {k: v.get("med") for k, v in blk_s["loop_joints"].items()}))
    chk("...and J1's command 0.9 ms after the read in both option B timings",
        abs(blk_s["tx_read_ms"]["med"] - 0.9) < 0.01 and abs(blk_b["tx_read_ms"]["med"] - 0.9) < 0.01,
        (blk_s["tx_read_ms"], blk_b["tx_read_ms"]))
    run_x = {"delta_t": 0.004, "clock_offsets": [offset], "blocks": [dict(run["blocks"][0], read_first=True)]}
    blk_x = analyse(run_x, d, c)["blocks"][0]
    chk("...a block's order matters: the old-order recording judged as option B pairs each reply with the command of "
        "its OWN tick (1.5 + 2.27 = 3.77 ms, not 7.77)", abs(blk_x["loop_ms"]["med"] - 3.77) < 0.01, blk_x["loop_ms"])
    src = open(os.path.abspath(__file__)).read()
    code = [ln.strip() for ln in src.splitlines()]   # whole code lines: this check's own strings never equal one
    chk("the session runner records each block's order, sets it for EVERY block (option B for '_rf', else the old "
        "one — B is on at start since 2026-10-12) and puts the starting order back at the end — the analysis pairs reply "
        "and command by it",
        '"read_first": base != kind}' in code and 'base = kind[:-len("_rf")] if kind.endswith("_rf") else kind' in code
        and any(ln.startswith("robot.set_can_read_first(base != kind)") for ln in code)
        and any(ln.startswith("robot.set_can_read_first(rf_start)") for ln in code)
        and not any(ln.startswith("robot.set_can_read_first(True)") or ln == "robot.set_can_read_first(False)"
                    for ln in code))
    r600 = [r for r in blk["runs"] if r["tick"] == 10601][0]
    chk("the late command: 3.4 ms after RTI's clock reading (usual 0.6)", abs(r600["tx_ms"] - 3.4) < 0.01, r600)
    ipj = json.dumps([{"linkinfo": {"info_data": {"state": "ERROR-ACTIVE"}, "info_xstats": {"bus_error": 2}},
                       "stats64": {"rx": {"packets": 10, "errors": 3, "over_errors": 3}, "tx": {"packets": 9}}}])
    c1 = counters_from_ip_json(ipj)
    c2 = dict(c1, rx_over_errors=5, rx_packets=30)
    chk("ip counters parsed and differenced", c1["rx_over_errors"] == 3 and c1["bus_error"] == 2
        and counter_delta(c1, c2)["rx_over_errors"] == 2 and counter_delta(c1, c2)["rx_packets"] == 20,
        (c1, counter_delta(c1, c2)))
    irq = irq_lines("           CPU0  CPU1  CPU2  CPU3\n122:  100  0  5  0  rp1_irq_chip 19 Level 1f00050000.spi\n"
                    "170:  7 0 0 1  pinctrl-rp1  12 Level     spi0.0\n 23: 9 9 9 9 GICv2 30 Level arch_timer\n")
    chk("interrupt counts summed over the cores, only SPI / CAN lines",
        irq == {"122 1f00050000.spi": 105, "170 spi0.0": 8}, irq)
    print("selftest:", "ALL PASSED" if ok_all else "FAILED")
    return 0 if ok_all else 1


# ----------------------------------------------------------------------------- robot session
def clock_offset(n=50):
    """CLOCK_REALTIME - CLOCK_MONOTONIC (s): candump stamps frames with the first, RTI its telemetry with the second
    (time.perf_counter). The pair of readings taken closest together wins."""
    best = None
    for _ in range(n):
        a = time.perf_counter()
        r = time.time()
        b = time.perf_counter()
        if best is None or b - a < best[0]:
            best = (b - a, r - 0.5 * (a + b))
    return best[1]


def read_counters(iface):
    try:
        out = subprocess.run(["ip", "-j", "-s", "-s", "-d", "link", "show", iface], capture_output=True, text=True,
                             timeout=2.0)
        return counters_from_ip_json(out.stdout)
    except Exception:
        return None


def read_irqs():
    try:
        with open("/proc/interrupts") as fh:
            return irq_lines(fh.read())
    except OSError:
        return {}


def thread_snapshot():
    """The threads that carry CAN traffic (SPI pump, CAN-chip interrupt, workqueues on RTI's core) and every real-time
    or core-3 Python thread (RTI's): class, priority, core."""
    try:
        out = subprocess.run(["ps", "-eLo", "pid,tid,cls,rtprio,psr,comm"], capture_output=True, text=True,
                             timeout=2.0).stdout.splitlines()
    except Exception:
        return []
    keep = out[:1]
    for ln in out[1:]:
        f = ln.split(None, 5)
        if len(f) < 6:
            continue
        comm = f[5]
        if re.search(r"spi|mcp|kworker/3|kworker/R|irq/.*(spi|can)", comm) or \
                (comm.startswith("python") and (f[2] in ("FF", "RR") or f[4] == "3")):
            keep.append(ln)
    return keep


def newest_tick(capture):
    from robotclient.rti_client import parse_state_packet
    for raw in reversed(capture.packets[-50:]):
        try:
            r = parse_state_packet(raw.decode())
        except Exception:
            continue
        if "tick" in r and "m_p" in r:
            return int(r["tick"])
    return None


class Load:
    """--load: starts a Python interpreter (numpy + scipy imports — what made the late replies on 2026-10-12: a
    program starting) every LOAD_EVERY_S on cores 0-2 at nice 19, until stop()."""
    def __init__(self, cores="0,1"):
        import threading
        self.cores = cores
        self._stop = threading.Event()
        self._procs = []
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop.is_set():
            self._procs.append(subprocess.Popen(["nice", "-n", "19", "taskset", "-c", self.cores, sys.executable, "-c",
                                                 "import numpy, scipy.signal"], stdout=subprocess.DEVNULL,
                                                stderr=subprocess.DEVNULL))
            self._stop.wait(LOAD_EVERY_S)

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2.0)
        for p in self._procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()


def rti_block(robot, ff68, sc, capture):
    """One RTI block: a hard-RTI session at 250 Hz streaming 68's J1 sine; back to IDLE after."""
    from robotclient import RTIClient
    rti = RTIClient(robot, clock="pc", timestamps=True)
    try:
        rti.connect(rate_hz=None)
        try:
            robot.wait(lambda s: s.get("mode") == "RTI", 3.0, "mode RTI")
        except Exception:
            raise RuntimeError(f"the robot is in {robot.status().get('mode')}, not RTI — refusing the block") from None
        time.sleep(0.4)
        sc.apply_config(robot, {"gen": "passthrough"})       # the hard contract's generator
        rti.read_once(timeout=2.0)
        rti.claim()
        q0 = list(rti.read_once(timeout=1.0)["j_p"])
        ff68.stream_block(rti, q0, [0], math.radians(AMP_DEG), PERIOD_S, 250.0, 250.0, SINE_S, random.Random(7))
        sc.keep_alive(rti, q0, 250.0, 0.3)
        rti.release()
        time.sleep(0.6)
        robot.rti_disconnect(wait=True)
    finally:
        try:
            rti.close()
        except Exception:
            pass


def exec_block(robot):
    """EXEC: J1 +15, then -15 / +15 ... (EXEC_MOVES moves of 30 deg), then back, at EXEC_SPEED %; back to IDLE after."""
    robot.enter_exec_mode(wait=True)
    q0 = [float(x) for x in robot.get_joints()]
    offsets = [AMP_DEG] + [(-AMP_DEG if i % 2 == 0 else AMP_DEG) for i in range(EXEC_MOVES)] + [0.0]
    for off in offsets:
        q = list(q0)
        q[0] = q0[0] + off
        robot.move_joints(*q, speed=EXEC_SPEED)
    fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
    time.sleep(0.3)
    robot.enter_idle_mode(wait=True)
    time.sleep(0.4)
    if fails:
        raise RuntimeError(f"EXEC move failed: {fails}")


def run_session(args, cfg, order, out_dir):
    from robotclient import RobotClient, ControlLost
    from api_tests import _servo_common as sc
    ff68 = importlib.import_module("68_rti_feedforward")     # the J1 sine + its sender, the control claim
    lo_lim = np.degrees(np.asarray(cfg["software_joint_limit_negative"], float))
    hi_lim = np.degrees(np.asarray(cfg["software_joint_limit_positive"], float))
    robot = RobotClient()
    robot.connect(args.host, args.port)
    capture = dump = dump_fh = dump_path = None
    rf_start = None                                    # the tick order at the start, put back at the end
    run = {"order": order, "args": vars(args), "delta_t": float(cfg.get("delta_t", 0.004)), "blocks": [],
           "clock_offsets": [], "threads": [], "analysed": False}
    prev_recipe, claimed, ok = None, False, True
    try:
        why = sc.preflight(robot)
        if why:
            print(f"Not ready: {why}. Nothing moved.")
            return 2, None
        st = robot.status()
        if st.get("mode") not in ("IDLE", "EXEC"):
            print(f"The robot must be in IDLE or EXEC (it is in {st.get('mode')}). Nothing moved.")
            return 2, None
        q_now = np.asarray(robot.get_joints(), float)
        room = min(q_now[0] - (lo_lim[0] + CLEARANCE_DEG), (hi_lim[0] - CLEARANCE_DEG) - q_now[0])
        if room < AMP_DEG:
            print(f"J1 at {q_now[0]:.1f} deg has only {room:.1f} deg of room for +/-{AMP_DEG:g} deg (soft limits "
                  f"{lo_lim[0]:.1f} .. {hi_lim[0]:.1f}, {CLEARANCE_DEG:g} deg clearance). Nothing moved.")
            return 2, None
        ff68.claim_control(robot)
        claimed = True
        # RTI's tick order at the start (status() since 2026-10-13; an older executor: the shipped default, ON) — every
        # block sets its own order and the run puts this one back at the end (RTI starts in system.xml's order).
        rf_start = bool(st.get("can_read_first", True))
        run["read_first_at_start"] = rf_start
        if st.get("motion_aborted"):
            robot.clear_errors(wait=True)
        if st.get("mode") == "EXEC":
            print("Leaving EXEC for IDLE (the blocks start from IDLE).")
            robot.enter_idle_mode(wait=True)
            time.sleep(0.5)
        os.makedirs(out_dir, exist_ok=True)
        # candump writes to RAM (tmpfs) while the arm moves, moved to out_dir at the end: on the SD card a slow write
        # stalled it and it dropped frames — a 221 ms "bus silence" in which RTI read 55 fresh replies (2026-10-12).
        # -d: candump reports its own drops ("DROPCOUNT" lines).
        ram = f"/run/user/{os.getuid()}"
        dump_path = os.path.join(ram if os.access(ram, os.W_OK) else out_dir, f"rcb69_{os.getpid()}_can0.log")
        dump_fh = open(dump_path, "w")
        dump = subprocess.Popen(["candump", "-L", "-d", args.iface], stdout=dump_fh, stderr=subprocess.DEVNULL)
        capture = Capture()
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(RECIPE)
        time.sleep(0.5)
        if newest_tick(capture) is None:
            print(f"No telemetry of the {RECIPE} recipe is arriving on UDP 30003. Nothing moved.")
            return 2, None
        run["threads"] = thread_snapshot()
        run["clock_offsets"].append(clock_offset())
        for name in ["idle"] + [f"{k}_{i + 1}" for i, k in enumerate(order)]:
            kind = name.rsplit("_", 1)[0] if name != "idle" else "idle"
            base = kind[:-len("_rf")] if kind.endswith("_rf") else kind
            blk = {"name": name, "kind": kind, "can_before": read_counters(args.iface), "irq_before": read_irqs(),
                   "t_start": time.time(), "tick0": newest_tick(capture), "ok": True, "load": bool(args.load),
                   "read_first": base != kind}
            print(f"  {name:15s} ... ", end="", flush=True)
            load = Load(args.load_cores) if args.load and kind != "idle" else None
            try:
                robot.set_can_read_first(base != kind)     # this block's order: option B for '_rf', else the old one
                time.sleep(0.2)
                if kind == "idle":
                    time.sleep(IDLE_S)
                elif base == "exec":
                    exec_block(robot)
                else:
                    rti_block(robot, ff68, sc, capture)
                print("done")
            except (ControlLost, Exception) as exc:             # any failure ends the run; the recording is kept
                print(f"FAILED: {type(exc).__name__}: {exc}")
                blk["ok"] = ok = False
                try:
                    robot.rti_disconnect()
                except Exception:
                    pass
            finally:
                if load is not None:
                    load.stop()
            blk.update({"t_end": time.time(), "tick1": newest_tick(capture), "can_after": read_counters(args.iface),
                        "irq_after": read_irqs()})
            run["blocks"].append(blk)
            if not ok:
                break
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        ok = False
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        try:
            sc.clear_config(robot)
            robot.enter_idle_mode()
        except Exception as exc:
            print(f"(cleanup: {exc})")
        if rf_start is not None:
            try:
                robot.set_can_read_first(rf_start)         # the order the robot had before the run
            except Exception as exc:
                print(f"!!! could not restore the tick order ({exc}) — RTI starts in system.xml's at its next start")
        if capture is not None:
            capture.stop()
        if dump is not None:
            dump.send_signal(signal.SIGINT)
            try:
                dump.wait(timeout=3)
            except subprocess.TimeoutExpired:
                dump.kill()
        if dump_fh is not None:
            dump_fh.close()
            if dump_path != os.path.join(out_dir, "can0.log"):
                shutil.move(dump_path, os.path.join(out_dir, "can0.log"))
        run["clock_offsets"].append(clock_offset())
        try:
            if prev_recipe and prev_recipe != RECIPE:
                robot.set_recipe(prev_recipe)
        except Exception as exc:
            print(f"Could not restore recipe {prev_recipe!r}: {exc}")
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()
    if capture is None or not run["blocks"]:
        return (0 if ok else 1), None
    return (0 if ok else 1), (capture.packets, run)


def _clean(o):
    """For JSON: NaN / inf -> null, numpy numbers -> Python numbers, tuples -> lists."""
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (float, np.floating)):
        return float(o) if math.isfinite(o) else None
    if isinstance(o, np.integer):
        return int(o)
    return o


def analyse_saved(folder):
    with open(os.path.join(folder, "run.json")) as fh:
        run = json.load(fh)
    with open(os.path.join(folder, "telemetry.raw"), "rb") as fh:
        d = to_rows(fh.read().split(b"\n"))
    with open(os.path.join(folder, "can0.log")) as fh:
        c = parse_candump(fh)
    rep = analyse(run, d, c)
    with open(os.path.join(folder, "can0.log")) as fh:
        drops = [ln.strip() for ln in fh if ln.startswith("DROPCOUNT")]
    rep["candump_drops"] = drops
    print_report(rep, f"=== {folder} ===")
    if drops:
        print(f"\ncandump reported dropped frames {len(drops)} time(s); the last: {drops[-1]}")
    if run.get("threads"):
        print("\nthreads at the start (class, priority, core):")
        for ln in run["threads"]:
            print("  " + ln)
    with open(os.path.join(folder, "report.json"), "w") as fh:
        json.dump(_clean(rep), fh, indent=1)
    print(f"\nSaved {os.path.join(folder, 'report.json')}")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move the arm")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--order", default=DEFAULT_ORDER, help=f"blocks in order, of {list(KINDS)}")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--load", action="store_true", help="start a Python interpreter every 2 s during each block "
                    "(nice 19): the load that made replies late on 2026-10-12")
    ap.add_argument("--load-cores", default=",".join(str(c) for c in GENERAL_CPU_CORES),
                    help="the cores --load runs on (taskset list; default the general cores of config/cpu_cores.py)")
    ap.add_argument("--reanalyse", metavar="FOLDER")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if args.reanalyse:
        if robot_enabled(args.host, args.port):
            print("The arm is ENABLED: the analysis is heavy for the Pi and would slow RTI's loop. Disable it first.")
            return 2
        return analyse_saved(args.reanalyse.rstrip("/"))
    order = [s.strip() for s in args.order.split(",") if s.strip()]
    bad = [s for s in order if s not in KINDS]
    if bad or not order:
        print(f"Refused: unknown blocks {bad} (of {list(KINDS)}). Nothing moved.")
        return 2
    rti_s = SINE_S + 2 * 1.0 + 1.5
    exec_s = EXEC_MOVES * 1.04 + 2.0
    total = IDLE_S + sum(exec_s if k.startswith("exec") else rti_s for k in order)
    print(f"Plan: {IDLE_S:g} s IDLE (baseline), then {order}: RTI blocks J1 sine +/-{AMP_DEG:g} deg, {PERIOD_S:g} s period "
          f"(~63 deg/s), {SINE_S:g} s each; EXEC blocks {EXEC_MOVES} moves of {2 * AMP_DEG:g} deg at {EXEC_SPEED:g} % "
          f"(~57 deg/s). candump on {args.iface} (read-only) + RTI telemetry ({RECIPE}) the whole time, ~{total / 60:.1f} min."
          + (" '_rf' blocks: option B ON — RTI reads CAN first, sends early (TODO/can_tx_timing_plan.md §9)." if any(
              k.endswith("_rf") for k in order) else "")
          + (f" --load: a Python interpreter started every {LOAD_EVERY_S:g} s in each block, on cores "
             f"{args.load_cores}." if args.load else ""))
    if not args.go:
        print("DRY RUN: re-run with --go to move the arm. Nothing moved.")
        return 0
    if shutil.which("candump") is None:
        print("candump not found (apt package can-utils). Nothing moved.")
        return 2
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    out_dir = os.path.join(_ROOT, "temp_data", "can_gaps", time.strftime("%Y%m%d_%H%M%S"))
    rc, got = run_session(args, cfg, order, out_dir)
    if got is None:
        return rc
    packets, run = got
    with open(os.path.join(out_dir, "telemetry.raw"), "wb") as fh:
        fh.write(b"\n".join(packets))
    with open(os.path.join(out_dir, "run.json"), "w") as fh:
        json.dump(_clean(run), fh, indent=1)
    rel = os.path.relpath(out_dir, _ROOT)
    print(f"\nRaw recording saved: {rel}/ (can0.log, telemetry.raw, run.json; {len(packets)} packets).")
    if robot_enabled(args.host, args.port) is not False:
        print("Not analysed yet (heavy for the Pi while the arm is enabled). Disable the arm, then:\n"
              f"  cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/69_can_gaps.py --reanalyse {rel}")
        return rc
    return analyse_saved(out_dir) or rc


if __name__ == "__main__":
    sys.exit(main())
