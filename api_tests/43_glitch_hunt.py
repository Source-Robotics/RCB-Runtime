#!/usr/bin/env python3
"""43: is the robot's own position reading glitching? READ-ONLY — no commands, no motion.

WHY THIS EXISTS. The off-robot suite (api_tests/external) found single state packets whose
`j_p` was wrong by 20-99 deg on several joints at once, about once per 10 s, sandwiched between
perfectly smooth neighbours. The arm cannot move 99 deg in 4 ms, so the ARM was not doing that:
the DATA was. It made tracking error read 1-3 deg on ~5 runs in 6, and the one clean run had no
glitch in it. Sender, network, transport and generator were all ruled out first.

RTI.py:2059 builds the packet's j_p by copying shared memory's `joint_position`, and
update_shared_data.py:89 rebuilds that array every tick from `motor_position`. So the glitch is
either

  (A) already in shared memory  -> the robot's OWN state is glitching, and the dashboard, the
      planner (`read_current_joints`), gravity compensation and a claim's start pose all read it;
  (B) added after that write, on the way into the packet  -> telemetry only.

ROOT CAUSE, found the same day: the OLED "Simulate" setting made supervisor.py write random
numbers into the live motor arrays every blink period (TODO/position_glitch.md). The feature has
been REMOVED. Verified on the arm after switching it off: 0 events in 60 s (was ~15/min). This
script is now the check that nothing similar comes back — expect 0 events.

This tells them apart by watching /dev/shm directly (plain file reads, nothing attached, nothing
written) at ~2 kHz for a while, arm at rest, and looking for single-tick jumps in BOTH
`joint_position` and `motor_position`. A glitch that lives for a whole 4 ms tick is seen by a
2 kHz sampler essentially every time.

    python3 api_tests/43_glitch_hunt.py --seconds 120
"""
import argparse
import math
import mmap
import os
import statistics as st
import struct
import sys
import time

SHM = "/dev/shm"
N = 6
JOINT_THRESH_DEG = 3.0


class Shm:
    """A read-only mapping of one shared-memory array of doubles."""

    def __init__(self, name, n, code="d"):
        fd = os.open(f"{SHM}/{name}", os.O_RDONLY)
        self.m = mmap.mmap(fd, 0, access=mmap.ACCESS_READ)
        os.close(fd)
        self.fmt = f"{n}{code}"

    def read(self):
        return struct.unpack_from(self.fmt, self.m, 0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--period-ms", type=float, default=0.5, help="sampler sleep (~2 kHz)")
    a = ap.parse_args()

    try:
        # motor_position is int32 encoder ticks (data/shared_data.py:250); the others are float64.
        jp, mp, lt = Shm("joint_position", N), Shm("motor_position", N, "i"), Shm("loop_tick", 1)
    except OSError as exc:
        print(f"cannot open shared memory ({exc}) — is RTI.py running?")
        return 2

    print("=" * 84)
    print(f"43: GLITCH HUNT — reading /dev/shm at ~{1000 / a.period_ms:.0f} Hz for {a.seconds:g} s. READ-ONLY.")
    print("=" * 84)
    print("  keep the arm still; no test needs to be running.\n")

    S = []                                  # (t, tick, jp[6], mp[6])
    t_end = time.perf_counter() + a.seconds
    dt = a.period_ms / 1e3
    while True:
        now = time.perf_counter()
        if now >= t_end:
            break
        S.append((now, lt.read()[0], jp.read(), mp.read()))
        time.sleep(dt)

    n = len(S)
    T0 = S[0][0]
    ticks = [int(s[1]) for s in S]
    print(f"  {n} samples in {S[-1][0] - T0:.1f} s = {n / (S[-1][0] - T0):.0f} Hz; loop ticks "
          f"{ticks[0]} -> {ticks[-1]} ({ticks[-1] - ticks[0]} ticks, "
          f"{(ticks[-1] - ticks[0]) / (S[-1][0] - T0):.1f}/s)")

    # --- spikes: deviation from the median of the 3 samples each side ---------------------
    def spikes(idx, thresh, conv):
        found = []
        for i in range(3, n - 3):
            dev = []
            for k in range(N):
                nb = [S[i + d][idx][k] for d in (-3, -2, -1, 1, 2, 3)]
                dev.append(abs(S[i][idx][k] - st.median(nb)) * conv)
            big = [k for k in range(N) if dev[k] > thresh]
            if big:
                found.append((i, big, dev))
        return found

    jsp = spikes(2, JOINT_THRESH_DEG, 180.0 / math.pi)
    # motor_position is in encoder ticks: use its own scale — the median step at rest is ~0,
    # so call it a spike when it moves more than 1000 ticks against its neighbours.
    msp = spikes(3, 1000.0, 1.0)

    def group(found):
        """Merge consecutive samples of one glitch. Returns [(first_idx, last_idx, joints, peak)]."""
        out = []
        for i, big, dev in found:
            if out and i - out[-1][1] <= 2:
                out[-1][1] = i
                out[-1][2] |= set(big)
                out[-1][3] = max(out[-1][3], max(dev))
            else:
                out.append([i, i, set(big), max(dev)])
        return out

    jg, mg = group(jsp), group(msp)
    print(f"\n  joint_position (shared memory): {len(jg)} glitch event(s) over {S[-1][0] - T0:.0f} s")
    for i0, i1, js, pk in jg[:15]:
        dur = (S[i1][0] - S[i0][0]) * 1e3 + a.period_ms
        print(f"    t={S[i0][0] - T0:8.3f} s  tick {int(S[i0][1])}  joints {sorted(k + 1 for k in js)}"
              f"  peak {pk:6.1f} deg  lasted ~{dur:.1f} ms ({i1 - i0 + 1} samples)")
    print(f"  motor_position (shared memory): {len(mg)} glitch event(s)")
    for i0, i1, js, pk in mg[:15]:
        dur = (S[i1][0] - S[i0][0]) * 1e3 + a.period_ms
        print(f"    t={S[i0][0] - T0:8.3f} s  tick {int(S[i0][1])}  joints {sorted(k + 1 for k in js)}"
              f"  peak {pk:9.0f} ticks  lasted ~{dur:.1f} ms")

    # the raw samples around the first few events, so the shape of a glitch can be SEEN
    for label, grp, idx in (("motor_position", mg, 3), ("joint_position", jg, 2)):
        for i0, i1, js, pk in grp[:2]:
            print(f"\n  raw {label} around t={S[i0][0] - T0:.3f} s "
                  f"(one row per sample, {a.period_ms:g} ms apart):")
            for i in range(max(0, i0 - 2), min(n, i1 + 3)):
                vals = S[i][idx]
                cell = ("  ".join(f"{v:10d}" for v in vals) if idx == 3
                        else "  ".join(f"{math.degrees(v):9.3f}" for v in vals))
                print(f"    {(S[i][0] - T0) * 1e3:10.2f} ms  tick {int(S[i][1])}  {cell}")

    if len(mg) >= 4:
        gaps = [mg[k + 1][0] - mg[k][0] for k in range(len(mg) - 1)]
        tk = [int(S[mg[k + 1][0]][1]) - int(S[mg[k][0]][1]) for k in range(len(mg) - 1)]
        print(f"\n  motor_position events: interval median {st.median(tk):.0f} ticks "
              f"({st.median(tk) * 4:.0f} ms), min {min(tk)}, max {max(tk)} — "
              f"{'PERIODIC' if max(tk) - min(tk) <= 4 else 'irregular'}")

    # how far apart in ticks, and do the two arrays glitch together?
    both = sum(1 for a_ in jg for b_ in mg if abs(a_[0] - b_[0]) <= 4)
    print(f"\n  events seen in BOTH arrays at the same moment: {both}")
    rate = len(jg) / (S[-1][0] - T0)
    print(f"  glitch rate in joint_position: {rate * 60:.1f} per minute "
          f"(the packet stream showed ~{9 / (6 * 16.0) * 60:.1f} per minute)")

    print("\n" + "-" * 84)
    if len(jg) == 0:
        print("  VERDICT: NO glitch in shared memory in this window.")
        print("    If the packets still glitch, it is added AFTER shared memory (telemetry only) —")
        print("    or it only happens in RTI mode; rerun with the arm paired/claimed in RTI.")
    else:
        print("  VERDICT: shared memory ITSELF glitches — the robot's own position reading, not")
        print("    telemetry. Everything that reads joint_position sees it.")
        print("  The 2026-09-30 cause (the OLED Simulate setting) was removed. Something ELSE is")
        print("    writing motor[]: run dummy_test_scripts/test_motor_array_writers.py, and record")
        print("    the CAN bus (candump) at the same moment to split bus from host.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
