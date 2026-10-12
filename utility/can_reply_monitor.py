"""J{i}:CAN_REPLY_MISSED_WARNING — a joint's drive reply missed RTI's read too often (2026-10-12).

Every tick RTI sends each joint a data command, and the drive answers it during RTI's sleep; the next tick's CAN read
picks the answer up. A reply that comes after that read (or never) leaves the joint's measured state a tick old. Until
2026-10-12 this went unnoticed: the CAN chip's interrupt thread ran on RTI's core while RTI slept and froze all CAN
traffic ~10 times a second in hard RTI (api_tests/69_can_gaps.py, TODO/servo_mode_options.md §17.23) — gaps far too
short for CAN_STALE (10 ticks) or a disconnect (200 ms). This counts them.

Per joint, a run of ticks without a reply that ENDS (a reply arrives) and was shorter than CAN_STALE's 10 ticks is added
to a 1 s window — a longer run is CAN_STALE's / CAN_LOST's to report. MISSED_WARN (5, user) or more in the window starts
a BURST, which lasts until the window is under half that. A burst that lasts SUSTAIN_S (3 s, user) becomes a live
warning (never a stop); RTI.log gets its peak, how long it lasted and the total since the start when it ends. A shorter
burst is one INFO line when it ends: on the arm (2026-10-12, after the fix) such bursts come with LOAD — another program
starting or a heavy job slows RTI's tick (memory traffic), its frames leave later (when it sleeps) and the last joints'
replies come a tick late for ~1 s; the freezes this was built for were continuous (~10 a second through a whole
stream). Counted only on ticks whose reads should hold a reply: the previous tick sent every joint a data command (not
HOMING / FLASHING — those send their own way or nothing) and RTI is past its start-up reconfiguration bursts (ticks 50 /
150 / 300 in RTI.py put ~170 frames on the bus each and delay replies on purpose).

Per tick: a few integer operations per joint, nothing allocated; the warning and its log line are the cold path.
"""
import logging

CAN_REPLY_MISSED_WARNING = "CAN_REPLY_MISSED_WARNING"
WINDOW_S = 1.0           # the window the misses are counted over
MISSED_WARN = 5          # misses in the window that start a burst (2 % of ticks at 250 Hz; user, 2026-10-12)
SUSTAIN_S = 3.0          # a burst this long becomes the live warning (user, 2026-10-12: shorter ones are load)
STALE_TICKS = 10         # a run this long or longer is CAN_STALE's (communication/can_message_handlers.py)
START_TICK = 400         # after the start-up reconfiguration at ticks 50 / 150 / 300 (RTI.py) has drained


class CanReplyMonitor:
    def __init__(self, joint_num, delta_t, missed_warn=MISSED_WARN, window_s=WINDOW_S, stale_ticks=STALE_TICKS,
                 start_tick=START_TICK, sustain_s=SUSTAIN_S):
        self.n = joint_num
        self.window = max(int(round(window_s / delta_t)), 1)
        self.missed_warn = missed_warn
        self.clear_below = missed_warn / 2.0
        self.delta_t = delta_t
        self.sustain_ticks = max(int(round(sustain_s / delta_t)), 1)
        self.stale_ticks = stale_ticks
        self.start_tick = start_tick
        self._ring = [[0] * joint_num for _ in range(self.window)]   # misses added on each tick of the window
        self._i = 0
        self.in_window = [0] * joint_num
        self._run = [0] * joint_num          # ticks without a reply so far, per joint
        self.total = [0] * joint_num         # misses counted since the start
        self._peak = [0] * joint_num
        self._burst = [0] * joint_num        # ticks the joint's burst has lasted (0 = none)
        self._bursting = 0                   # joints in a burst
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
        if hot or self._bursting:
            self._update_bursts()

    def _update_bursts(self):
        """Cold path: per joint, start / time / end a burst; a burst that lasts sustain_ticks is the live warning."""
        for j in range(self.n):
            c = self.in_window[j]
            if not self._burst[j]:
                if c >= self.missed_warn:
                    self._burst[j] = 1
                    self._bursting += 1
                    self._peak[j] = c
                continue
            self._burst[j] += 1
            if c > self._peak[j]:
                self._peak[j] = c
            key = self._keys[j]
            if c < self.clear_below:
                lasted = self._burst[j] * self.delta_t
                if key in self.warnings:
                    self.warnings.discard(key)
                    logging.info(f"[CAN] J{j + 1}: replies back on time after {lasted:.1f} s; the gap peaked at "
                                 f"{self._peak[j]} missed in {self.window} ticks ({self.total[j]} missed since the start)")
                else:
                    logging.info(f"[CAN] J{j + 1}: a {lasted:.1f} s burst of late replies (peak {self._peak[j]} missed in "
                                 f"{self.window} ticks; {self.total[j]} since the start) — not sustained, no warning "
                                 f"(usually load on the Pi)")
                self._burst[j] = 0
                self._bursting -= 1
            elif key not in self.warnings and self._burst[j] >= self.sustain_ticks:
                self.warnings.add(key)
                logging.warning(f"[CAN] J{j + 1}: its replies have kept missing RTI's read for "
                                f"{self._burst[j] * self.delta_t:.1f} s ({c} of the last {self.window}) — its measured "
                                f"state is a tick old (TODO/servo_mode_options.md §17.23)")
