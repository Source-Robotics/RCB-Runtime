"""The control loop's delay, measured live per joint (2026-10-12): a joint's CAN reply (its position / speed / current)
arriving -> the CAN command RTI computed from it leaving. The GUI's RT Diagnostics shows it.

What it is: ~7.8 ms in the normal tick order — the reply arrives while RTI sleeps, is read in the next tick, used by the
tick after that, whose command leaves only when RTI sleeps again — and ~3.8 ms with option B (`can_read_first`: read
first, send early; TODO/can_tx_timing_plan.md §9), measured offline by api_tests/69. It matters where OUR software closes
a loop on the measured state (the haptic walls, a PC's force control in hard RTI); the drives close their own position
loops on their encoders, so tracking does not depend on it.

How: the reply times come with RTI's CAN read (every frame carries its kernel arrival time). The command times need the
frames' own departure, which only a second socket sees (the kernel hands every socket a frame once it has been SENT —
how candump sees ours): a raw socket filtered to every joint's data commands, SO_TIMESTAMPNS on. It receives NOTHING
between samples (an empty filter, so nothing queues); every `every` ticks it samples:
  arm, right after the read of tick m:  the reply times of that read; the filter on
  finish, before the dispatch of tick m+1+lag (lag 1 in the old order: read m feeds tick m+1's command; 0 with B: tick
  m's own): the echoes since arming, per joint — exactly 1+lag of them expected (old: tick m's and m+1's, the second
  is the one; B: tick m's) — else that joint is skipped; delay = that echo - the reply time; the filter off.
The median and the maximum of the last `window` samples per joint go to shared memory (rti['loop_delay'],
rti['loop_delay_max'], seconds). A sample is dropped when the tick order or the mode changes in between, or in
HOMING / FLASHING (they send their own way). Per tick: two integer comparisons; the sampling itself (~0.1 ms, a few
small allocations) runs on one tick in `every`.
"""
import logging
import math
import socket
import struct

import numpy as np

logger = logging.getLogger("rti.can")  # docs/logging.md "Logger names"

_CAN_EFF_FLAG, _CAN_RTR_FLAG, _CAN_SFF_MASK = 0x80000000, 0x40000000, 0x7FF
SO_TIMESTAMPNS = getattr(socket, "SO_TIMESTAMPNS", 35)         # Linux; also the SCM type of the ancillary data
DATA_COMMANDS = (2, 4)            # data pack 1, data pack PD: the per-tick command to a joint
_TS = struct.Struct("@ll")        # struct timespec: seconds, nanoseconds


def data_command_ids(joint_num):
    """Every joint's data command CAN ids: (node << 7) | (command << 1), error bit 0."""
    return tuple((j << 7) | (c << 1) for j in range(joint_num) for c in DATA_COMMANDS)


def filter_bytes(can_ids):
    mask = _CAN_SFF_MASK | _CAN_EFF_FLAG | _CAN_RTR_FLAG
    return b"".join(struct.pack("=II", i, mask) for i in can_ids)


def open_echo_socket(interface="can0"):
    """A raw CAN socket with kernel timestamps, receiving nothing until its filter is set."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b"")
    sock.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
    sock.setblocking(False)
    sock.bind((interface,))
    return sock


class LoopDelayMeter:
    def __init__(self, joint_num, every=50, window=25, interface="can0", open_socket=open_echo_socket):
        self.n = joint_num
        self.every = every
        self._on = filter_bytes(data_command_ids(joint_num))
        self._ids = {cid: j for j in range(joint_num) for cid in ((j << 7) | (c << 1) for c in DATA_COMMANDS)}
        self._rep = np.full(joint_num, np.nan)
        self._ring = np.full((window, joint_num), np.nan)
        self._k = 0
        self.median = np.full(joint_num, np.nan)
        self.maximum = np.full(joint_num, np.nan)
        self.samples = 0
        self._armed_at = -1               # the tick the filter went on (-1: idle)
        self._finish_at = -1
        self._lag = 1
        try:
            self._sock = open_socket(interface)
        except OSError as e:
            self._sock = None
            logger.error(f"loop-delay meter: cannot open a socket on {interface}: {e} — no live loop delay")

    # ------------------------------------------------------------------ per tick (cheap)
    def on_read(self, tick, reply_times, read_first, sending):
        """Right after RTI's CAN read. Arms a sample on every `every`-th tick (when idle and the mode sends)."""
        if self._armed_at < 0 and tick % self.every == 0 and sending and self._sock is not None:
            self._arm(tick, reply_times, read_first)

    def before_dispatch(self, tick, read_first, sending):
        """Right before dispatch. Finishes an armed sample at its tick; drops it if the order or the mode changed."""
        if self._armed_at >= 0 and tick >= self._finish_at:
            self._finish(read_first == (self._lag == 0) and sending)

    # ------------------------------------------------------------------ the sample (cold: one tick in `every`)
    def _arm(self, tick, reply_times, read_first):
        self._rep[:] = reply_times
        self._lag = 0 if read_first else 1
        self._armed_at = tick
        self._finish_at = tick + 1 + self._lag
        try:
            self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, self._on)
        except OSError:
            self._armed_at = -1

    def _finish(self, valid):
        echoes = [[] for _ in range(self.n)]
        try:
            self._sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b"")
            while True:                                  # what came in while the filter was on (bounded: one tick's)
                try:
                    data, anc, _, _ = self._sock.recvmsg(16, socket.CMSG_SPACE(_TS.size))
                except BlockingIOError:
                    break
                cid = struct.unpack_from("=I", data, 0)[0] & _CAN_SFF_MASK
                j = self._ids.get(cid)
                for level, kind, raw in anc:
                    if j is not None and level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS:
                        sec, nsec = _TS.unpack(raw[:_TS.size])
                        echoes[j].append(sec + nsec * 1e-9)
        except OSError:
            valid = False
        finally:
            self._armed_at = -1
        if not valid:
            return
        row = self._ring[self._k]
        for j in range(self.n):
            e = echoes[j]
            ok = len(e) == 1 + self._lag and math.isfinite(self._rep[j]) and e[self._lag] > self._rep[j]
            row[j] = e[self._lag] - self._rep[j] if ok else np.nan
        self._k = (self._k + 1) % self._ring.shape[0]
        self.samples += 1
        with np.errstate(all="ignore"):
            for j in range(self.n):
                col = self._ring[:, j]
                col = col[np.isfinite(col)]
                self.median[j] = float(np.median(col)) if col.size else np.nan
                self.maximum[j] = float(col.max()) if col.size else np.nan

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
