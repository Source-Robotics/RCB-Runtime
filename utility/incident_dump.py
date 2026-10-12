#!/usr/bin/env python3
"""The last two seconds before a hard error, saved to a file (2026-10-03).

RTI already keeps 500 ticks of every joint / TCP channel in the `Joint_data` deque (utility/data_init.py) for
the telemetry recipes. When a HARD error latches (E-stop, CAN_LOST, LOOP_CRITICAL, a safety-monitor stop, a motor
fault, ...) that history is exactly the evidence every "what happened on that tick" question has needed — and it
was being overwritten 500 ticks later. Industrial controllers commonly attach the same thing (the last state
records) to a control error. This saves it to
`temp_data/incidents/<UTC time>_<tick>_<key>.npz` with every channel as a (ticks, n) array.

Cost on the real-time thread: a shallow copy of the deque (500 references, a few microseconds) on the one tick
the error appears, nothing otherwise. The arrays are stacked and written by a worker thread that yields between
channels, so it never holds the interpreter for more than one channel — and that thread moves itself OFF the
real-time core to a normal scheduling class first (`_leave_the_rt_core`): started after `setup_realtime_process`
it inherits SCHED_FIFO 99 on RTI's core, where a same-priority thread is NOT preempted by the loop's wake-up, and
the zlib compress of one dump is 36-50 ms of pure CPU (measured on the real 0.84 MB dump). On the arm, 2026-10-07,
the first real hard error (SELF_COLLISION) was followed 80 ms later by a 31 ms tick and a LOOP_CRITICAL on top of
it — this. Off the RT core the compress costs the loop nothing (the GIL is released inside zlib); the short Python
bursts between (one channel each, a sleep after) are the only interpreter time it takes.
At most one dump per `min_interval_s`, the newest `keep` files kept; none with less than `min_history` ticks of
history (an error at the very start of a run has nothing to show).

    from utility.incident_dump import IncidentDumper
    dumper = IncidentDumper(TEMP_DATA_DIR / "incidents")
    dumper.start()                                # after setup_realtime_process, like the log listener
    ... process_errors_tick(..., on_hard_error=dumper.hook(Joint_data))

    meta, channels = IncidentDumper.load(path)    # channels["j_p"].shape == (500, 6)
"""
from __future__ import annotations

import json
import logging
import os
import queue
import threading
import time
from pathlib import Path

import numpy as np

_META_KEY = "__meta__"
_log = logging.getLogger(__name__)


from config.cpu_cores import GENERAL_CPU_CORES  # noqa: E402
OTHER_CORES = tuple(GENERAL_CPU_CORES)   # never RTI's core (3) nor the CAN core (2); see the module docstring


def _leave_the_rt_core(cores=OTHER_CORES) -> tuple[int, set] | None:
    """Called by the writer THREAD on itself (pid 0 = the calling thread on Linux): normal scheduling, off RTI's core.
    Returns (policy, affinity) as set, None when the platform cannot."""
    try:
        wanted = set(cores) & os.sched_getaffinity(0) if hasattr(os, "sched_getaffinity") else set()
        if not wanted:                                    # a 1-3 core box: at least drop the priority
            wanted = os.sched_getaffinity(0)
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
        os.sched_setaffinity(0, wanted)
        return os.sched_getscheduler(0), os.sched_getaffinity(0)
    except (AttributeError, OSError) as exc:                   # pragma: no cover - not Linux, or refused
        logging.getLogger(__name__).warning("[incident] the dump thread stays at the RT priority: %s", exc)
        return None


class IncidentDumper:
    def __init__(self, directory, keep: int = 20, min_interval_s: float = 10.0, chunk_sleep_s: float = 0.001,
                 min_history: int = 50):
        self.directory = Path(directory)
        self.keep = int(keep)
        self.min_interval_s = float(min_interval_s)
        self.chunk_sleep_s = float(chunk_sleep_s)
        self._q = queue.SimpleQueue()
        self._thread = None
        self._last_request = float("-inf")
        self.written = []            # paths written this run (newest last) — for tests and the dashboard
        self.requested = 0
        self.dropped = 0             # requests refused by the rate limit
        # An error with less history than this is not saved (2026-10-05): a fault the drivers carried over from the
        # last RTI latched at TICK 1 of every start — one tick of history, nothing to see — and each such file took
        # one of the `keep` slots from a real incident.
        self.min_history = int(min_history)
        self.too_early = 0           # requests refused for too little history

    # ------------------------------------------------------------------ real-time side
    def request(self, keys, tick, joint_data) -> bool:
        """Snapshot `joint_data` (a deque of per-tick dicts) for `keys` at `tick`. Returns False when rate-limited or
        when there is less than `min_history` ticks of history (checked first: it does not use up the rate limit)."""
        if len(joint_data) < self.min_history:
            self.too_early += 1
            _log.info(f"[incident] {', '.join(str(k) for k in keys)} at tick {tick}: not saved — only "
                      f"{len(joint_data)} tick(s) of history (< {self.min_history})")
            return False
        now = time.monotonic()
        if now - self._last_request < self.min_interval_s:
            self.dropped += 1
            return False
        self._last_request = now
        self.requested += 1
        self._q.put((time.time(), int(tick), tuple(str(k) for k in keys), list(joint_data)))
        return True

    def hook(self, joint_data):
        """The callable process_errors_tick(on_hard_error=...) wants: (new_hard_keys, tick) -> request()."""
        def on_hard_error(keys, tick):
            self.request(keys, tick, joint_data)
        return on_hard_error

    # ------------------------------------------------------------------ worker side
    def start(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._run, name="incident-dump", daemon=True)
            self._thread.start()

    def stop(self, timeout: float = 5.0) -> None:
        """Write whatever is queued, then return (atexit)."""
        if self._thread is None:
            return
        self._q.put(None)
        self._thread.join(timeout)
        self._thread = None

    def _run(self) -> None:
        _leave_the_rt_core()
        while True:
            item = self._q.get()
            if item is None:
                return
            try:
                self._write(*item)
            except Exception as exc:                      # a dump must never take anything else down
                import logging
                logging.getLogger(__name__).warning("[incident] dump failed: %s", exc)

    def _write(self, wall_time, tick, keys, snapshot) -> None:
        if not snapshot:
            return
        channels = {}
        sample = snapshot[0]
        for name in sample:
            values = [d.get(name) for d in snapshot]
            if isinstance(sample[name], np.ndarray):
                channels[name] = np.stack([np.asarray(v, dtype=np.float64) for v in values])
            elif isinstance(sample[name], str):
                channels[name] = np.array([str(v) for v in values])
            else:
                channels[name] = np.asarray(values, dtype=np.float64)
            time.sleep(self.chunk_sleep_s)                # yield: never hold the interpreter for long
        meta = {"time": wall_time, "time_utc": time.strftime("%Y-%m-%d %H:%M:%S", time.gmtime(wall_time)),
                "tick": tick, "keys": list(keys), "ticks": len(snapshot), "channels": sorted(channels)}
        self.directory.mkdir(parents=True, exist_ok=True)
        safe = keys[0].replace(":", "-").replace("/", "-") if keys else "error"
        path = self.directory / f"{time.strftime('%Y%m%d_%H%M%S', time.gmtime(wall_time))}_{tick}_{safe}.npz"
        tmp = path.with_suffix(".tmp.npz")
        np.savez_compressed(tmp, **{_META_KEY: np.array(json.dumps(meta))}, **channels)
        os.replace(tmp, path)
        self.written.append(path)
        self._prune()

    def _prune(self) -> None:
        files = sorted(self.directory.glob("*.npz"), key=lambda p: p.stat().st_mtime)
        for old in files[:-self.keep] if self.keep > 0 else []:
            try:
                old.unlink()
            except OSError:
                pass

    # ------------------------------------------------------------------ reading
    @staticmethod
    def load(path):
        """(meta dict, {channel: array}) of one dump."""
        with np.load(path, allow_pickle=False) as z:
            meta = json.loads(str(z[_META_KEY]))
            channels = {k: z[k] for k in z.files if k != _META_KEY}
        return meta, channels
