"""J{i}:CAN_REPLY_MISSED_WARNING — a joint's drive reply missed RTI's read too often (2026-10-12).

Every tick RTI sends each joint a data command, and the drive answers it during RTI's sleep; the next tick's CAN read
picks the answer up. A reply that comes after that read (or never) leaves the joint's measured state a tick old. Until
2026-10-12 this went unnoticed: the CAN chip's interrupt thread ran on RTI's core while RTI slept and froze all CAN
traffic ~10 times a second in hard RTI (api_tests/69_can_gaps.py, TODO/servo_mode_options.md §17.23) — gaps far too
short for CAN_STALE (10 ticks) or a disconnect (200 ms). This counts them.

Per joint, a run of ticks without a reply that ENDS (a reply arrives) and was shorter than CAN_STALE's 10 ticks is added
to a 1 s window — a longer run is CAN_STALE's / CAN_LOST's to report. A joint with MISSED_WARN (5, user) or more in its
window gets a live warning (never a stop); it clears below half that, and RTI.log gets the peak and the total since the
start. Counted only on ticks whose reads should hold a reply: the previous tick sent every joint a data command (not
HOMING / FLASHING — those send their own way or nothing) and RTI is past its start-up reconfiguration bursts (ticks 50 /
150 / 300 in RTI.py put ~170 frames on the bus each and delay replies on purpose).

Per tick: a few integer operations per joint, nothing allocated; the warning and its log line are the cold path.
"""
import logging

CAN_REPLY_MISSED_WARNING = "CAN_REPLY_MISSED_WARNING"
WINDOW_S = 1.0           # the window the misses are counted over
MISSED_WARN = 5          # misses in the window that raise the warning (2 % of ticks at 250 Hz; user, 2026-10-12)
STALE_TICKS = 10         # a run this long or longer is CAN_STALE's (communication/can_message_handlers.py)
START_TICK = 400         # after the start-up reconfiguration at ticks 50 / 150 / 300 (RTI.py) has drained


class CanReplyMonitor:
    def __init__(self, joint_num, delta_t, missed_warn=MISSED_WARN, window_s=WINDOW_S, stale_ticks=STALE_TICKS,
                 start_tick=START_TICK):
        self.n = joint_num
        self.window = max(int(round(window_s / delta_t)), 1)
        self.missed_warn = missed_warn
        self.clear_below = missed_warn / 2.0
        self.stale_ticks = stale_ticks
        self.start_tick = start_tick
        self._ring = [[0] * joint_num for _ in range(self.window)]   # misses added on each tick of the window
        self._i = 0
        self.in_window = [0] * joint_num
        self._run = [0] * joint_num          # ticks without a reply so far, per joint
        self.total = [0] * joint_num         # misses counted since the start
        self._peak = [0] * joint_num
        self._keys = [f"J{j + 1}:{CAN_REPLY_MISSED_WARNING}" for j in range(joint_num)]
        self.warnings = set()

    def step(self, replies, tick, expected):
        """replies: data replies RTI's CAN read took this tick, per joint. expected: the previous tick sent every
        joint a data command (so this read should hold one reply each)."""
        row = self._ring[self._i]
        self._i += 1
        if self._i == self.window:
            self._i = 0
        hot = False
        count = expected and tick >= self.start_tick
        for j in range(self.n):
            if row[j]:
                self.in_window[j] -= row[j]
                row[j] = 0
            if not count:
                self._run[j] = 0
            elif replies[j] == 0:
                self._run[j] += 1
            elif self._run[j]:
                run = self._run[j]
                self._run[j] = 0
                if run < self.stale_ticks:
                    row[j] = run
                    self.in_window[j] += run
                    self.total[j] += run
            if self.in_window[j] >= self.missed_warn:
                hot = True
        if hot or self.warnings:
            self._update_warnings()

    def _update_warnings(self):
        """Cold path: raise, track the peak of and clear the per-joint warnings."""
        for j in range(self.n):
            key = self._keys[j]
            c = self.in_window[j]
            if key in self.warnings:
                if c > self._peak[j]:
                    self._peak[j] = c
                if c < self.clear_below:
                    self.warnings.discard(key)
                    logging.info(f"[CAN] J{j + 1}: replies back on time; the gap peaked at {self._peak[j]} missed in "
                                 f"{self.window} ticks ({self.total[j]} missed since the start)")
            elif c >= self.missed_warn:
                self.warnings.add(key)
                self._peak[j] = c
                logging.warning(f"[CAN] J{j + 1}: {c} of its last {self.window} replies missed RTI's read — its "
                                f"measured state was a tick old (TODO/servo_mode_options.md §17.23)")
