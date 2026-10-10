"""Test whether Python's GC can explain RTI's rare multi-ms exec spikes.

Run: python3 tools/check_gc_pauses.py            (offline, does not touch RTI)

RTI showed 211 overruns / 7.269 ms max exec over 73 min — 0.019% of ticks, p99 a healthy
1.844 ms, i.e. rare outliers rather than systemic cost. The suspect is the generational
collector: process_joint_data does 32 numpy .copy() per tick (~8,000 allocations/sec at
250 Hz) into a 500-deep deque of dicts, and RTI never calls gc.disable/freeze/set_threshold.
CLAUDE.md's hot-path rule names this exact failure: "a GC pause doesn't respect SCHED_FIFO —
it happens inside your own process".

This drives the REAL process_joint_data at the real rate and times every collection via
gc.callbacks, so the pause distribution is measured rather than assumed.

IMPORTANT — this is a LOWER BOUND. A gen2 pass walks every tracked object in the process, and
RTI holds far more of them (robot model, pinocchio, shared memory wrappers, handlers) than this
script does. Whatever pause shows up here, RTI's will be at least that long.
"""

import argparse
import gc
import statistics
import sys
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from utility.data_init import get_default_joint_data, process_joint_data

JN = 6
DT = 0.004


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60.0,
                    help="how long to run (default 60; RTI saw ~1 spike per 21 s)")
    ap.add_argument("--freeze", action="store_true",
                    help="apply the candidate fix (gc.freeze + higher thresholds) and compare")
    args = ap.parse_args()

    if args.freeze:
        # Candidate mitigation: promote everything alive now out of GC's way, then make gen0
        # collections much rarer. Does not disable GC (cycles still get collected eventually).
        gc.collect()
        gc.freeze()
        gc.set_threshold(50_000, 50, 50)
        print("MITIGATION ACTIVE: gc.freeze() + thresholds (50000, 50, 50)\n")

    # ---- time every collection ----
    pauses = {0: [], 1: [], 2: []}
    _t0 = [0.0]

    def _cb(phase, info):
        if phase == "start":
            _t0[0] = time.perf_counter()
        else:
            pauses[info["generation"]].append((time.perf_counter() - _t0[0]) * 1e3)

    gc.callbacks.append(_cb)

    # ---- replicate the real per-tick allocation pattern ----
    Joint_data = deque([get_default_joint_data(JN)], maxlen=500)
    z6, zj, z16, z4 = np.zeros(6), np.zeros(JN), np.zeros(16), np.zeros(4)
    args_tuple = (zj, zj, zj, zj, zj, zj, zj, zj, zj,
                  z6, z16, z4, z6, zj, z6, z6, z6, z6, z6,
                  zj, zj, zj, zj, z6, z6, zj, zj, zj, zj, z6, z6,
                  1.0, 0.001, 0, zj)

    print(f"driving the real process_joint_data at {1/DT:.0f} Hz for {args.seconds:.0f}s "
          f"({32} array copies/tick)...\n")

    ticks = 0
    worst_tick_ms = 0.0
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        t_tick = time.perf_counter()
        process_joint_data(Joint_data, *args_tuple)
        el = (time.perf_counter() - t_tick) * 1e3
        if el > worst_tick_ms:
            worst_tick_ms = el
        ticks += 1
        time.sleep(DT)

    gc.callbacks.remove(_cb)

    # ---- report ----
    print(f"ticks driven          : {ticks:,}")
    print(f"worst single tick     : {worst_tick_ms:.3f} ms  (includes any GC that landed on it)\n")
    total = 0
    for gen in (0, 1, 2):
        v = pauses[gen]
        total += len(v)
        if not v:
            print(f"  gen{gen}: none")
            continue
        print(f"  gen{gen}: {len(v):>6} collections   "
              f"median {statistics.median(v):6.3f} ms   "
              f"max {max(v):7.3f} ms   "
              f"total {sum(v):8.1f} ms")
    rate = total / args.seconds
    print(f"\n  {total:,} collections in {args.seconds:.0f}s = {rate:.1f}/s")

    worst = max((max(v) for v in pauses.values() if v), default=0.0)
    print()
    if worst >= 4.0:
        print(f"VERDICT: a single GC pause reached {worst:.2f} ms — LONGER THAN THE 4 ms TICK.")
        print("  This alone can produce RTI's rare overruns. And it is a lower bound: RTI holds")
        print("  far more tracked objects, so its gen2 passes will be longer than these.")
    elif worst >= 1.0:
        print(f"VERDICT: worst pause {worst:.2f} ms — significant but under the 4 ms tick here.")
        print("  RTI tracks many more objects, so scale this up before dismissing it; a gen2")
        print("  pass there could still cross 4 ms.")
    else:
        print(f"VERDICT: worst pause only {worst:.2f} ms. GC does NOT look like the cause;")
        print("  the 7.269 ms spikes are more likely I/O (logging), the scheduler, or a")
        print("  periodic task. Look elsewhere.")


if __name__ == "__main__":
    main()
