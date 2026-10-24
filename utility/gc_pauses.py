"""Python's garbage collector and RTI's loop (2026-10-24).

Why: every service restart + homing since 2026-10-22 latched LOOP_CRITICAL 5-9 s after homing ended — ONE tick of
~70 ms, all of it RTI's own work, in IDLE, the loop normal around it (temp_data/incidents/*_LOOP_CRITICAL.npz). RTI
never touched `gc`, so CPython's automatic collector could run a FULL collection inside a tick: it walks every
container object the process holds, and RTI holds every object its imports and start-up made (the robot model, the
toolbox, numpy, our modules).

Two parts, both used by RTI.py:

  freeze_startup(log)   once, just before the loop: time one full collection, `gc.freeze()` every object start-up
                        made (CPython's permanent generation: never scanned again), time one more, log both. The two
                        numbers on the arm's own heap show what a full collection cost the loop before and after.
  GcPauseMonitor        a `gc.callbacks` timer for every collection while the loop runs. The callback only stores
                        numbers (it runs INSIDE the collector, in RTI's thread: no logging there — the queue handler's
                        lock could be held by the very code the collection interrupted). The loop calls flush() when
                        `pending` is set, outside the collector, which logs each pause over the threshold as a
                        WARNING with its generation.

Frozen objects are never collected: a reference cycle made before the freeze and dropped later stays in memory.
Start-up objects live as long as RTI anyway.
"""
from __future__ import annotations

import gc
import logging
import time

logger = logging.getLogger("rti.startup")       # docs/logging.md "Logger names"

GC_PAUSE_LOG_S = 0.005      # s: a collection longer than this is logged (a tick is 4 ms on PAR6)
_SLOTS = 8                  # pauses kept between two flushes (more are counted, not kept)


def full_collection_ms() -> float:
    """One full collection, in ms."""
    t0 = time.perf_counter()
    gc.collect()
    return (time.perf_counter() - t0) * 1000.0


def freeze_startup(log: logging.Logger = logger) -> tuple[float, float, int]:
    """Time a full collection, freeze every object start-up made, time one again; log the three numbers.
    Returns (ms before, ms after, objects frozen). Called once, before the loop starts — never inside it."""
    before = full_collection_ms()
    gc.freeze()
    after = full_collection_ms()
    frozen = gc.get_freeze_count()
    log.info("garbage collection: a full collection took %.1f ms over the start-up objects; %d objects frozen "
             "(never scanned again); a full collection now takes %.1f ms", before, frozen, after)
    return before, after, frozen


class GcPauseMonitor:
    """Times every garbage collection while installed; keeps the ones over `threshold_s` for flush()."""

    def __init__(self, threshold_s: float = GC_PAUSE_LOG_S, log: logging.Logger | None = None):
        self.threshold_s = float(threshold_s)
        self.log = log if log is not None else logging.getLogger("rti.loop")
        self.pending = 0                     # pauses over the threshold since the last flush (the loop reads this)
        self.collections = 0                 # every collection seen
        self.longest_ms = 0.0
        self._t0 = 0.0
        self._gen = [0] * _SLOTS
        self._ms = [0.0] * _SLOTS
        self._installed = False

    def on_gc(self, phase, info):
        """gc.callbacks: phase "start" / "stop", info {"generation", "collected", "uncollectable"}. Runs inside the
        collector, in the collecting thread: it stores numbers only."""
        if phase == "start":
            self._t0 = time.perf_counter()
            return
        dt = time.perf_counter() - self._t0
        self.collections += 1
        if dt > self.threshold_s:
            if self.pending < _SLOTS:
                self._gen[self.pending] = info["generation"]
                self._ms[self.pending] = dt * 1000.0
            self.pending += 1

    def install(self) -> None:
        if not self._installed:
            gc.callbacks.append(self.on_gc)
            self._installed = True

    def uninstall(self) -> None:
        if self._installed:
            gc.callbacks.remove(self.on_gc)
            self._installed = False

    def flush(self, tick: int) -> None:
        """Log the pauses kept since the last flush (call it when `pending` is set, outside the collector)."""
        n = self.pending
        self.pending = 0
        for i in range(min(n, _SLOTS)):
            self.longest_ms = max(self.longest_ms, self._ms[i])
            self.log.warning("garbage collection (generation %d) paused the loop %.1f ms (by tick %d)",
                             self._gen[i], self._ms[i], tick)
        if n > _SLOTS:
            self.log.warning("... and %d more collections over %.0f ms", n - _SLOTS, self.threshold_s * 1000.0)
