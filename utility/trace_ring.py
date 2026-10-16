"""RTI's per-tick trace in shared memory, read by the recorder in ANOTHER process (2026-10-16).

Incident dumps and telemetry captures used to be taken inside RTI's process: a thread copied RTI's per-tick history
(Python dicts) and turned it into arrays. Measured on the arm (api_tests/75, 2026-10-16): a 60 s capture made 37 ticks
late and raised LOOP_DEGRADED every 3 s — that thread held the interpreter lock RTI's loop needs. Any way of saving the
history from inside RTI's process walks Python objects under that lock (pickling it raw: 253 ms in one call). User:
"I don't want them to impact my loop performance in any way".

So RTI only WRITES: once per tick, `TraceWriter.write()` copies the tick's values into the next row of a ring in shared
memory — one `np.concatenate(..., out=row)` of RTI's own persistent arrays (the same ones the history entry copies)
plus ten scalars, ~9 us, the same every tick, nothing allocated. The recorder process (recorder.py,
utility/trace_recorder.py) reads the rows with its own interpreter and does the incidents, the captures, the
conversion and the files; RTI never waits for it.

Layout (shared memory, RCB_SHM_PREFIX-aware through data.shared_data's helpers):
  trace_head    float64[8]: COUNT (rows written so far), GENERATION (RTI's start, epoch s), WIDTH, ROWS, TICK_S,
                REC_HEARTBEAT (the recorder's counter), REC_TIME (the recorder's last pass, epoch s), 1 spare
  trace_rows    float64[ROWS x WIDTH]: column 0 = the row's sequence number (COUNT after it was written; -1 while
                being written), then every array source flat, in order, then SCALARS
  trace_layout  a JSON string: {"generation", "rows", "width", "tick_s", "arrays": [[name, start, size], ...],
                "scalars": [[name, column], ...], "modes": [...]} — written once at RTI's start

A reader takes only rows at least READ_LAG behind COUNT (the writer's stores to a row are long visible by then on a
weakly-ordered CPU) and drops a row whose sequence number changed while it copied it (the writer wrapped onto it).
"""
from __future__ import annotations

import json
import time

import numpy as np

ROWS = 1250                  # 5 s at 250 Hz: the reader polls every ~50 ms, so it is never near a wrap
READ_LAG = 2                 # rows a reader leaves behind COUNT
SCALARS = ("timestamp", "exec_t", "tick", "rti_seq", "rti_ok", "rti_miss", "rti_state", "mode", "period_ms",
           "can_read_t")
H_COUNT, H_GENERATION, H_WIDTH, H_ROWS, H_TICK_S, H_REC_HEARTBEAT, H_REC_TIME = range(7)
REC_ALIVE_S = 1.0            # the recorder passes 20 times a second; older than this = not running
HEAD_SIZE = 8
LAYOUT_BYTES = 16384


def mode_names() -> tuple:
    """The robot modes in a fixed order: a row stores a mode as its index here (-1 = unknown)."""
    from utility.state_machine import RobotMode
    return tuple(m.value for m in RobotMode)


def _segment(name, count):
    from data.shared_data import _attach_or_create
    return _attach_or_create(name, np.float64, count)


def _layout_string():
    from data.shared_data import SharedString
    return SharedString("trace_layout", LAYOUT_BYTES)


class TraceWriter:
    """RTI's side. `sources`: [(name, array)] — RTI's own arrays that keep their identity for the whole run (shared
    memory views, the motor error buffer); a source replaced by a new object would be traced stale. Built once, before
    the loop."""

    def __init__(self, sources, tick_s: float, rows: int = ROWS, generation: float | None = None):
        self.names = [str(n) for n, _ in sources]
        self._src = tuple(np.asarray(a) for _, a in sources)
        sizes = [int(a.size) for a in self._src]
        n_data = sum(sizes)
        self.rows = int(rows)
        self.width = 1 + n_data + len(SCALARS)
        self._s0 = 1 + n_data                                 # the first scalar's column
        self._head_shm, self._head = _segment("trace_head", HEAD_SIZE)
        self._rows_shm, flat = _segment("trace_rows", self.rows * self.width)
        self._ring = flat.reshape(self.rows, self.width)
        # one view per row (and per row's data part), made here: write() then allocates nothing
        self._row_views = [self._ring[k] for k in range(self.rows)]
        self._data_views = [self._ring[k, 1:1 + n_data] for k in range(self.rows)]
        self._modes = {m: float(i) for i, m in enumerate(mode_names())}
        self._n = 0
        generation = time.time() if generation is None else float(generation)
        arrays, start = [], 1
        for name, size in zip(self.names, sizes):
            arrays.append([name, start, size])
            start += size
        self.layout = {"generation": generation, "rows": self.rows, "width": self.width, "tick_s": float(tick_s),
                       "arrays": arrays, "scalars": [[s, self._s0 + i] for i, s in enumerate(SCALARS)],
                       "modes": list(self._modes)}
        self._ring[:, 0] = 0.0
        self._head[H_COUNT] = 0.0
        self._head[H_WIDTH] = self.width
        self._head[H_ROWS] = self.rows
        self._head[H_TICK_S] = float(tick_s)
        _layout_string().set(json.dumps(self.layout))
        self._head[H_GENERATION] = generation                 # last: a reader re-attaches when it changes

    def write(self, timestamp, exec_t, tick, rti_seq, rti_ok, rti_miss, rti_state, mode, period_ms, can_read_t):
        """This tick's row: called once per tick, after the history entry. Allocates nothing."""
        n = self._n + 1
        k = (n - 1) % self.rows
        row = self._row_views[k]
        row[0] = -1.0                                         # being written
        np.concatenate(self._src, out=self._data_views[k])
        s = self._s0
        row[s] = timestamp
        row[s + 1] = exec_t
        row[s + 2] = tick
        row[s + 3] = rti_seq
        row[s + 4] = rti_ok
        row[s + 5] = rti_miss
        row[s + 6] = rti_state
        row[s + 7] = self._modes.get(mode, -1.0)
        row[s + 8] = period_ms
        row[s + 9] = can_read_t
        row[0] = n
        self._head[H_COUNT] = n
        self._n = n


class TraceReader:
    """The recorder's side. attach() returns None until RTI has written a layout. poll() -> (rows, lost)."""

    def __init__(self, head, ring, layout, rows_shm=None, head_shm=None):
        self._head, self._ring, self.layout = head, ring, layout
        self._shms = (rows_shm, head_shm)                     # keep the mappings alive
        self.generation = float(layout["generation"])
        self.rows, self.width = int(layout["rows"]), int(layout["width"])
        self.last = int(head[H_COUNT])                        # start at the present: no old rows replayed

    @classmethod
    def attach(cls):
        head_shm, head = _segment("trace_head", HEAD_SIZE)
        try:
            layout = json.loads(_layout_string().get() or "null")
        except ValueError:
            layout = None
        if not layout or float(layout.get("generation", -1)) != float(head[H_GENERATION]) or not head[H_WIDTH]:
            return None
        rows_shm, flat = _segment("trace_rows", int(layout["rows"]) * int(layout["width"]))
        return cls(head, flat.reshape(int(layout["rows"]), int(layout["width"])), layout, rows_shm, head_shm)

    def stale(self) -> bool:
        """RTI restarted (a new generation): attach() again."""
        return float(self._head[H_GENERATION]) != self.generation

    def poll(self, lag: int = READ_LAG):
        """The rows written since the last poll (oldest first, a copy) and how many were lost (overwritten before they
        were read, or changed while copied)."""
        upto = int(self._head[H_COUNT]) - lag
        if upto <= self.last:
            return np.empty((0, self.width)), 0
        lost = 0
        first = self.last + 1
        if upto - first + 1 > self.rows - lag:                # the writer lapped us: the oldest are gone
            new_first = upto - (self.rows - lag) + 1
            lost += new_first - first
            first = new_first
        seqs = np.arange(first, upto + 1)
        block = self._ring[(seqs - 1) % self.rows].copy()
        ok = block[:, 0] == seqs                              # a row the writer reached while we copied it: dropped
        lost += int((~ok).sum())
        self.last = upto
        return block[ok], lost

    def heartbeat(self, now=None) -> None:
        self._head[H_REC_HEARTBEAT] += 1.0
        self._head[H_REC_TIME] = time.time() if now is None else now


def read_head():
    """The head as a dict (the health summary, the dashboard), or None when RTI never wrote one."""
    try:
        _shm, head = _segment("trace_head", HEAD_SIZE)
    except Exception:
        return None
    return {"count": float(head[H_COUNT]), "generation": float(head[H_GENERATION]),
            "rec_heartbeat": float(head[H_REC_HEARTBEAT]), "rec_time": float(head[H_REC_TIME])}


def recorder_alive(max_age_s: float = REC_ALIVE_S, now=None) -> bool:
    """True when the recorder passed within `max_age_s` (its time in the trace's head)."""
    h = read_head()
    now = time.time() if now is None else now
    return bool(h) and 0.0 <= now - h["rec_time"] <= max_age_s
