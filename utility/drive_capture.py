"""A drive's own recording of a fault (2026-11-17, user: "auto-capture faults"; TODO/driver_110_rtb15_plan.md).

Firmware 109+ records three signals inside its own control loop (6250 Hz on a STEPFOC), 1024 samples, and can be armed
to trigger when one of its errors rises. RTI arms every drive at start (and again after Clear Errors and after reading
a capture): the motor current, its speed (/16) and the position error, every 4th loop (1562 Hz: 0.66 s, 800 samples
— 0.51 s — before the fault). When a drive's live fault bit rises (rti['node_error_bits'], from every reply), RTI
waits 0.5 s (the drive finishes the samples after the trigger), asks how many samples and where the trigger is, reads
the samples back and publishes them; recorder.py saves the file in temp_data/drive_captures/. Read 2 samples per
request, one request every other free status-poll slot — never in one request: the drive answers a buffer read with
one frame per sample "as fast as the bus takes them", ~130 ms of solid bus for 1024 samples, which would starve the
higher-numbered drives' data replies for ~30 ticks. ~4 s per capture.

Shared memory (data/shared_data.py): rti['drive_capture'] float64 [3 * 1024] the samples (channel-major, oldest first,
in the drive's units: mA, ticks/s / 16, ticks); rti['drive_capture_meta'] float64 [12]: seq (raised last, when the rest
is written), node, count, trigger index, divider, loop rate Hz, signal ids A / B / C, the fault's Unix time, its tick,
samples lost on the way.
"""
from __future__ import annotations

import contextlib
import logging
import time
from collections import deque

import numpy as np

SIGNALS = ("iq", "velocity_16", "position_error")
SIGNAL_IDS = (1, 5, 7)                    # the library's SIGNALS numbers of the three
UNITS = ("mA", "ticks/s", "ticks")        # velocity_16 is shown x16
SCALE = (1.0, 16.0, 1.0)
DIVIDER, PRETRIGGER, LENGTH = 4, 800, 1024
CHUNK = 2                                 # samples per buffer-read request
SETTLE_TICKS = 125                        # 0.5 s after the fault before reading: the 224 samples after it are recorded
WAIT_TICKS = 6                            # an answer is in a few ticks after asking
LOOP_RATES = {1: 6250.0, 2: 5000.0}       # STEPFOC, Spectral Micro (product id)
# the capture's parameters (the library's PARAM_CAPTURE_*)
P_SOURCE_A, P_SOURCE_B, P_SOURCE_C, P_DIVIDER, P_TRIGGER, P_LEVEL, P_PRETRIGGER = 24, 25, 26, 27, 28, 29, 30
P_STATE, P_COUNT, P_TRIGGER_INDEX = 31, 32, 33
TRIGGER_ERROR = 2
BUFFER_CAPTURE = 0
META = 12


class DriveCapture:
    """RTI's side (see the module docstring). step() and send_next() run in the poll slot; nothing waits."""

    def __init__(self, samples, meta, error_bits, nodes, clock=time.time):
        self.samples, self.meta, self.err = samples, meta, error_bits
        self.nodes = list(nodes)
        self.clock = clock
        self.queue: deque = deque()          # ("set", node, number, value) | ("get", node, number) | ("buf", node, first, n)
        self.prev = [0] * (max(self.nodes) + 1 if self.nodes else 0)
        self.armed: set = set()
        self.pending: deque = deque()        # (node, fault tick, fault Unix time)
        self.phase = None                    # None | "meta" | "data"
        self.node = -1
        self.count = 0
        self.fault = (0, 0.0)
        self.tick = 0
        self.last_send = 0
        self.tries = 0

    def watch(self, node) -> None:
        """Watch one more node from now on (2026-11-18, P2b: a gripper with a drive applied live after RTI started with
        a tool without one); arm it with arm()."""
        node = int(node)
        if node not in self.nodes:
            self.nodes.append(node)
            self.prev.extend([0] * (node + 1 - len(self.prev)))

    def arm(self, nodes) -> None:
        """Arm `nodes`: the three signals, the divider, the samples kept before the trigger, trigger on an error."""
        for n in nodes:
            for number, value in ((P_SOURCE_A, SIGNAL_IDS[0]), (P_SOURCE_B, SIGNAL_IDS[1]), (P_SOURCE_C, SIGNAL_IDS[2]),
                                  (P_DIVIDER, DIVIDER), (P_PRETRIGGER, PRETRIGGER), (P_LEVEL, 0),
                                  (P_TRIGGER, TRIGGER_ERROR)):
                self.queue.append(("set", n, number, value))
            self.armed.add(n)

    def rearm(self, nodes) -> None:
        """Arm `nodes` again (Clear Errors) — except a drive whose capture is waiting to be read or being read: arming
        restarts its recording and would overwrite the samples on the way."""
        busy = {p[0] for p in self.pending}
        if self.phase is not None:
            busy.add(self.node)
        self.arm([n for n in nodes if n not in busy])

    def send_next(self, motors) -> None:
        """Send the next request (one frame)."""
        if not self.queue:
            return
        item = self.queue.popleft()
        m = motors[item[1]]
        if item[0] == "set":
            m.Send_Parameter(item[2], item[3])
        elif item[0] == "get":
            params = getattr(m, "parameters", None)
            if params is not None:
                params.pop(item[2], None)
            m.Send_Get_Parameter(item[2])
        else:
            m.Send_Buffer_Read(BUFFER_CAPTURE, item[2], item[3])
        self.last_send = self.tick

    def step(self, motors) -> None:
        """Every poll tick: a rising fault bit on an armed drive is a capture to read; the reading's next step."""
        self.tick += 1
        err = self.err
        for n in self.nodes:
            e = 1 if int(err[n]) == 1 else 0
            if e and not self.prev[n] and n in self.armed:
                self.armed.discard(n)
                self.pending.append((n, self.tick, self.clock()))
                self._drop_arming(n)
            self.prev[n] = e
        if self.queue or self.tick - self.last_send < WAIT_TICKS:
            return
        if self.phase is None:
            if self.pending and self.tick - self.pending[0][1] >= SETTLE_TICKS:
                self._start(motors)
        elif self.phase == "meta":
            self._after_meta(motors)
        else:
            self._after_data(motors)

    def _drop_arming(self, node) -> None:
        """On `node`'s fault edge: its arming frames not sent yet are dropped (review C10, 2026-11-21). A fault right
        after Clear Errors, while they waited, was followed by them — they restarted the drive's recording over it. Off
        the per-tick path (a fault edge only)."""
        keep = [q for q in self.queue if not (q[0] == "set" and q[1] == node)]
        self.queue.clear()
        self.queue.extend(keep)

    def _start(self, motors) -> None:
        n, t, wall = self.pending.popleft()
        self.node, self.fault, self.phase, self.tries = n, (t, wall), "meta", 0
        for number in (P_STATE, P_COUNT, P_TRIGGER_INDEX):
            self.queue.append(("get", n, number))

    def _after_meta(self, motors) -> None:
        params = getattr(motors[self.node], "parameters", None) or {}
        count = params.get(P_COUNT)
        if count is None:                     # no answer (a drive that lost power at an E-stop): once more, then give up
            self.tries += 1
            if self.tries <= 1:
                for number in (P_STATE, P_COUNT, P_TRIGGER_INDEX):
                    self.queue.append(("get", self.node, number))
            else:
                self.phase = None             # nothing to read; Clear Errors arms it again
            return
        self.count = max(0, min(int(count), LENGTH))
        if self.count == 0:
            self.phase = None
            self.arm([self.node])
            return
        cap = getattr(motors[self.node], "capture", None)
        if cap is not None:
            cap.clear()
        for first in range(0, self.count, CHUNK):
            self.queue.append(("buf", self.node, first, min(CHUNK, self.count - first)))
        self.phase = "data"

    def _after_data(self, motors) -> None:
        m = motors[self.node]
        cap = getattr(m, "capture", None) or {}
        params = getattr(m, "parameters", None) or {}
        # One list, one array, three slice copies (review D16, 2026-11-22): value by value into shared memory it cost
        # ~0.8 ms of RTI's tick for 1024 samples, now ~0.5 ms (measured off the RT core; the same numbers).
        lost, last, rows = 0, (0, 0, 0), []
        get = cap.get
        for i in range(self.count):
            row = get(i)
            if row is None:
                lost += 1
                row = last
            last = row
            rows.append(row)
        if rows:
            arr = np.asarray(rows, dtype=np.float64)
            for k in range(3):
                self.samples[k * LENGTH:k * LENGTH + self.count] = arr[:, k]
        meta = self.meta
        meta[1], meta[2] = float(self.node), float(self.count)
        trig = params.get(P_TRIGGER_INDEX)
        meta[3] = float(trig) if trig is not None else -1.0
        meta[4] = float(DIVIDER)
        meta[5] = LOOP_RATES.get(getattr(m, "product_id", None) or 1, 6250.0)
        meta[6], meta[7], meta[8] = (float(s) for s in SIGNAL_IDS)
        meta[9], meta[10], meta[11] = float(self.fault[1]), float(self.fault[0]), float(lost)
        meta[0] += 1.0                        # last: the recorder saves when it sees a new seq
        self.phase = None
        self.arm([self.node])


# ---- the recorder's side ----------------------------------------------------------------------------------------------
def capture_arrays(samples, meta):
    """(meta dict, three channels in their units) of a published capture."""
    count = int(meta[2])
    channels = [[float(samples[k * LENGTH + i]) * SCALE[k] for i in range(count)] for k in range(3)]
    info = {"seq": int(meta[0]), "node": int(meta[1]), "count": count, "trigger_index": int(meta[3]),
            "divider": int(meta[4]), "loop_rate_hz": float(meta[5]), "signals": list(SIGNALS), "units": list(UNITS),
            "fault_time": float(meta[9]), "fault_tick": int(meta[10]), "lost": int(meta[11])}
    info["sample_s"] = info["divider"] / info["loop_rate_hz"] if info["loop_rate_hz"] else None
    return info, channels


def node_label(node: int, joint_num: int) -> str:
    return f"J{node + 1}" if node < joint_num else "gripper"


class CaptureSaver:
    """recorder.py's side: a new seq in rti['drive_capture_meta'] -> temp_data/drive_captures/<UTC>_<J3>.npz."""

    KEEP = 30

    def __init__(self, samples, meta, folder, joint_num, activity=None):
        self.samples, self.meta, self.folder = samples, meta, folder
        self.joint_num = joint_num
        self.activity = activity
        self.seen = int(meta[0])              # what was there before the recorder started is not saved again

    RETRY_S = 30.0                        # a failed save (a full card) is tried again this long after

    def poll(self, now=None):
        """Save a new capture; returns its path or None. A save that fails is tried again RETRY_S later, until it
        succeeds or a newer capture replaces it (review C10, 2026-11-21: it was marked saved first, so a failure lost it
        for good); one warning per capture."""
        import json
        import numpy as np
        seq = int(self.meta[0])
        if seq == self.seen:
            return None
        now = time.monotonic() if now is None else now
        if getattr(self, "_failed", None) == seq and now < self._retry_at:
            return None
        info, channels = capture_arrays(self.samples, self.meta)
        label = node_label(info["node"], self.joint_num)
        stamp = time.strftime("%Y%m%d_%H%M%S", time.gmtime(info["fault_time"] or time.time()))
        path = self.folder / f"{stamp}_{label}_drive_capture.npz"
        try:
            self.folder.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(path, meta=json.dumps(info), **{s: np.asarray(c) for s, c in zip(SIGNALS, channels)})
        except OSError as e:
            if getattr(self, "_failed", None) != seq:
                logging.getLogger("rec").warning("drive capture %s could not be saved (%s) — tried again every %g s",
                                                 path.name, e, self.RETRY_S)
            self._failed, self._retry_at = seq, now + self.RETRY_S
            with contextlib.suppress(OSError):
                path.unlink()                 # a half-written file is not a capture
            return None
        self.seen = seq
        for old in sorted(self.folder.glob("*_drive_capture.npz"))[:-self.KEEP]:
            old.unlink()
        if self.activity is not None:
            self.activity("info", "DRIVE_CAPTURE_SAVED", f"{label}: the drive's own recording of its fault, "
                          f"{info['count']} samples at {1 / info['sample_s']:.0f} Hz -> {path.name}", None, None)
        return path
