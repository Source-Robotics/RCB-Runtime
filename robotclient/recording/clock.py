"""One robot's clock against this PC's, from the packets the robot sends (the recorder's one time base).

Inside one robot, a camera frame and the state rows are matched by RTI's own clock and tick, exactly. Several robots
(several Pis) and this PC each have their own clock, and a robot on an isolated network has no time server, so the
recorder puts every robot's times on THIS PC's clock (`time.perf_counter_ns`, high resolution on Windows too):

* every packet carries the robot's time (the `timestamp` tag: RTI's `time.perf_counter()` taken just before the
  packet is built and sent) and is stamped here the moment it is read;
* arrival - sent = the difference between the clocks + the delay on the way (the rest of RTI's tick, the network,
  this program's reading); the smallest of it over a window is the difference + the SHORTEST delay, which hardly
  varies (a fraction of a millisecond on a cable);
* the window is the last `buckets` buckets of `bucket_s` seconds each (4 x 0.5 s), so a slow drift between the two
  clocks (a crystal's few tens of parts per million) is followed.

The constant shortest delay shifts one robot's times as a whole; across robots it is the error between them, to be
measured (TODO/p4_recording_plan.md 3.1). Standard library only.
"""
from __future__ import annotations

import time
from collections import deque


def unix_minus_perf_ns() -> int:
    """This PC's Unix time minus its `time.perf_counter_ns()`, taken now: added to a perf_counter time it gives
    nanoseconds since 1970 (what an MCAP file's times are). Taken once per session, so a step of the wall clock
    (a time server's correction) never moves the times inside a recording."""
    return time.time_ns() - time.perf_counter_ns()


class RobotClock:
    """Maps one robot's clock (seconds of its `time.perf_counter()`) onto this PC's `time.perf_counter_ns()`.

    `add(sent_s, arrival_ns)` for every packet; `to_pc_ns(robot_s)` for any time on the robot's clock (a packet's, a
    camera frame's). `offset_ns` = the current estimate (PC - robot), None before the first packet; `samples` =
    packets seen; `spread_ns` = the largest minus the smallest bucket minimum in the window (how steady it is).
    """

    def __init__(self, bucket_s: float = 0.5, buckets: int = 4) -> None:
        self.bucket_ns = int(bucket_s * 1e9)
        self._done: deque = deque(maxlen=max(1, int(buckets) - 1))   # finished buckets' minima
        self._cur_min: int | None = None
        self._cur_start: int | None = None
        self.offset_ns: int | None = None
        self.spread_ns = 0
        self.samples = 0

    def add(self, sent_s: float, arrival_ns: int) -> bool:
        """One packet: its robot time `sent_s` (seconds) and its arrival `arrival_ns` (this PC's perf_counter_ns).
        Returns True when a bucket closed (a new settled estimate, worth storing)."""
        diff = int(arrival_ns) - round(float(sent_s) * 1e9)
        self.samples += 1
        closed = False
        if self._cur_start is None:
            self._cur_start = arrival_ns
        elif arrival_ns - self._cur_start >= self.bucket_ns and self._cur_min is not None:
            self._done.append(self._cur_min)
            self._cur_min = None
            self._cur_start = arrival_ns
            closed = True
        if self._cur_min is None or diff < self._cur_min:
            self._cur_min = diff
        mins = list(self._done) + [self._cur_min]
        self.offset_ns = min(mins)
        self.spread_ns = max(mins) - self.offset_ns
        return closed

    def to_pc_ns(self, robot_s: float) -> int | None:
        """A time on the robot's clock (seconds) on this PC's perf_counter_ns; None before the first packet."""
        if self.offset_ns is None:
            return None
        return round(float(robot_s) * 1e9) + self.offset_ns
