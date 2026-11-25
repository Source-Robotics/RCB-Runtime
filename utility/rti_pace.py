"""Pacing for heavy picture work on the robot's Pi, watched on RTI's own trace (2026-11-25).

Decoding recorded camera frames beside RTI slowed its loop: LOOP_CRITICAL latched twice on 2026-11-24 and once on
2026-11-25 (the arm disabled each time; nothing moved, but Clear Errors was needed), even at the lowest priority —
the work competes with RTI for memory bandwidth, not for a core. So a tool that decodes many frames calls a Pace after
each piece of work: it pauses for twice that work (at most a third of one core), and reads the rows RTI wrote since.
A tick longer than PERIOD_MS, or a tick's own work over EXEC_MS, pauses 2 s and halves the pace; the STRIKES-th such
tick stops the work with RuntimeError. RTI's normal tick: period 4.000 ms (p99 4.003), work 1.7 ms (max 2.7),
measured 2026-11-25; both limits sit far below RTI's own LOOP_DEGRADED.

Without RTI's trace (a PC, a net, RTI not running) a Pace does nothing. A strike is reported through `say` (the
caller's console, e.g. print; None = silent): this module is part of the runtime tree, which prints nothing.
"""
from __future__ import annotations

import time
from typing import Callable, Optional

PERIOD_MS = 4.5
EXEC_MS = 3.5
STRIKES = 3


class Pace:
    def __init__(self, on: bool = True, say: Optional[Callable[[str], None]] = None):
        self.tr = None
        self.say = say
        self.strikes, self.duty = 0, 2.0
        if on:
            try:
                from utility.trace_ring import TraceReader
                self.tr = TraceReader.attach()
            except Exception:  # noqa: BLE001 — no shared memory: not on the robot
                self.tr = None
        if self.tr is not None:
            cols = {n: c for n, c in self.tr.layout["scalars"]}
            self.cp, self.ce = cols["period_ms"], cols["exec_t"]
            self.tr.poll()

    def __call__(self, seconds: float) -> None:
        """After `seconds` of work: pause, then look at RTI's ticks since the last call."""
        if self.tr is None:
            return
        time.sleep(max(0.005, self.duty * seconds))
        rows, _ = self.tr.poll()
        if len(rows) and (rows[:, self.cp].max() > PERIOD_MS or rows[:, self.ce].max() * 1e3 > EXEC_MS):
            self.strikes += 1
            self.duty *= 2.0
            if self.say is not None:
                self.say(f"  (RTI's loop slowed during the picture work: pausing, strike {self.strikes} of {STRIKES})")
            if self.strikes >= STRIKES:
                raise RuntimeError(f"RTI's loop slowed {STRIKES} times during the picture work: run the analysis "
                                   f"later (--reanalyse) or on a PC")
            time.sleep(2.0)
            self.tr.poll()
