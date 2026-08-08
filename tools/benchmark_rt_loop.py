"""Benchmark RTI's real-time behaviour, so tuning decisions are measured not argued.

Typical use — establish a clean baseline BEFORE changing anything:

    # GUI closed, nobody touching the Pi, no other tooling running
    python3 tools/benchmark_rt_loop.py --label baseline --seconds 600

Then reproduce the contention you actually care about:

    python3 tools/benchmark_rt_loop.py --label baseline-loaded --seconds 600 --load 3

Change the boot flags, reboot, repeat with --label isolcpus / isolcpus-loaded, then:

    python3 tools/benchmark_rt_loop.py --compare baseline isolcpus

WHY THE --load FLAG MATTERS. isolcpus/nohz_full/rcu_nocbs only help when something else is
competing for the machine. An idle-system A/B will show no difference and tell you nothing.
--load N pins N busy processes to the NON-RTI cores, which is the honest test: it reproduces
"GUI + executor + OS are busy" without you having to drive the GUI by hand. Contention for the
shared L3 and memory bus crosses cores even though the load never touches core 3 — which is
precisely the effect under investigation.

Read-only with respect to RTI. The only thing it writes is loop_stats_reset_signal, the same
one-shot the GUI's "Reset Loop Stats" button uses.
"""

import argparse
import json
import multiprocessing as mp
import os
import statistics
import struct
import sys
import time
from pathlib import Path

RESULTS = Path(__file__).resolve().parent.parent / "temp_data" / "rt_benchmarks"
SHM = Path("/dev/shm")


def _rd(name):
    with open(SHM / name, "rb") as f:
        return struct.unpack("<d", f.read(8))[0]


def _rti_alive():
    a = _rd("loop_tick")
    time.sleep(0.25)
    return _rd("loop_tick") > a


def _burn(stop):
    """CPU burner with a memory-touching inner loop, so it competes for cache/bandwidth and
    not just for CPU time — the latter cannot affect a SCHED_FIFO 99 task anyway."""
    buf = bytearray(4 * 1024 * 1024)
    i = 0
    while not stop.is_set():
        for j in range(0, len(buf), 64):     # stride the cache lines
            buf[j] = (buf[j] + 1) & 0xFF
        i += 1


def _spawn_load(n):
    """Pin burners to every core EXCEPT RTI's (core 3)."""
    others = [c for c in range(os.cpu_count() or 4) if c != 3]
    stop = mp.Event()
    procs = []
    for k in range(n):
        p = mp.Process(target=_burn, args=(stop,), daemon=True)
        p.start()
        try:
            os.sched_setaffinity(p.pid, {others[k % len(others)]})
        except OSError:
            pass
        procs.append(p)
    return stop, procs


def measure(seconds, label, load):
    if not _rti_alive():
        sys.exit("RTI is not running (loop_tick is not advancing) — start it first.")

    stop = procs = None
    if load:
        stop, procs = _spawn_load(load)
        print(f"  spawned {load} load process(es) on the non-RTI cores")

    # Zero the since-start peaks so the window is clean (same signal the GUI button uses).
    with open(SHM / "loop_stats_reset_signal", "r+b") as f:
        f.write(b"1\x00")
    time.sleep(0.5)

    t0 = time.time()
    tick0 = _rd("loop_tick")
    execs, ages, jits = [], [], []
    print(f"  sampling for {seconds}s...")
    try:
        while time.time() - t0 < seconds:
            execs.append(_rd("loop_execution_time") * 1e3)
            ages.append(_rd("can_frame_age") * 1e3)
            jits.append(_rd("jitter") * 1e6)
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("  interrupted — reporting what was collected")
    finally:
        if stop:
            stop.set()
            for p in procs:
                p.join(timeout=1)

    elapsed = time.time() - t0
    ticks = _rd("loop_tick") - tick0
    execs.sort()
    res = {
        "label": label, "seconds": round(elapsed, 1), "load": load,
        "ticks": int(ticks), "hz": round(ticks / elapsed, 2),
        "overruns": int(_rd("overrun_count")),
        "overrun_pct": round(_rd("overrun_count") / max(ticks, 1) * 100, 4),
        "exec_avg_ms": round(_rd("avg_loop_execution_time") * 1e3, 3),
        "exec_p99_ms": round(_rd("p99_loop_execution_time") * 1e3, 3),
        "exec_max_ms": round(_rd("max_loop_execution_time") * 1e3, 3),
        "exec_sampled_p99_ms": round(execs[int(len(execs) * 0.99)], 3) if execs else 0,
        "jitter_avg_us": round(_rd("avg_jitter") * 1e6, 1),
        "jitter_max_us": round(_rd("max_jitter") * 1e6, 1),
        "can_age_ms": round(statistics.median(ages), 3) if ages else 0,
        "can_age_peak_ms": round(_rd("can_frame_age_peak") * 1e3, 3),
    }
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / f"{label}.json").write_text(json.dumps(res, indent=2))
    _print_one(res)
    print(f"\n  saved -> {RESULTS / (label + '.json')}")
    return res


_FIELDS = [
    ("overruns", "overruns", "lower"), ("overrun_pct", "overrun %", "lower"),
    ("exec_avg_ms", "exec avg (ms)", "lower"), ("exec_p99_ms", "exec p99 (ms)", "lower"),
    ("exec_max_ms", "exec MAX (ms)", "lower"), ("jitter_avg_us", "jitter avg (us)", "lower"),
    ("jitter_max_us", "jitter MAX (us)", "lower"), ("can_age_ms", "CAN age (ms)", "lower"),
    ("hz", "rate (Hz)", "same"),
]


def _print_one(r):
    print(f"\n  {r['label']}  ({r['seconds']}s, {r['ticks']:,} ticks, load={r['load']})")
    for key, name, _ in _FIELDS:
        print(f"    {name:18} {r[key]}")


def compare(a, b):
    ra = json.loads((RESULTS / f"{a}.json").read_text())
    rb = json.loads((RESULTS / f"{b}.json").read_text())
    print(f"\n{'metric':20}{a:>16}{b:>16}{'change':>14}")
    print("-" * 66)
    for key, name, better in _FIELDS:
        va, vb = ra[key], rb[key]
        if va == 0:
            chg = "n/a" if vb == 0 else "+inf"
        else:
            pct = (vb - va) / abs(va) * 100
            chg = f"{pct:+.1f}%"
            if better == "lower" and abs(pct) > 5:
                chg += " BETTER" if pct < 0 else " WORSE"
        print(f"{name:20}{va:>16}{vb:>16}{chg:>14}")
    if ra["load"] != rb["load"]:
        print(f"\n  ⚠ different load levels ({ra['load']} vs {rb['load']}) — not comparable.")
    if ra["seconds"] < 120 or rb["seconds"] < 120:
        print("\n  ⚠ short run: overruns are rare (~0.02% of ticks), so a window under a few")
        print("    minutes cannot distinguish a real change from noise.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", help="name this run (saved for later comparison)")
    ap.add_argument("--seconds", type=float, default=600)
    ap.add_argument("--load", type=int, default=0,
                    help="busy processes pinned to the NON-RTI cores (contention test)")
    ap.add_argument("--compare", nargs=2, metavar=("A", "B"))
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    if a.list:
        for f in sorted(RESULTS.glob("*.json")):
            _print_one(json.loads(f.read_text()))
        return
    if a.compare:
        compare(*a.compare)
        return
    if not a.label:
        ap.error("--label is required (or use --compare / --list)")
    measure(a.seconds, a.label, a.load)


if __name__ == "__main__":
    main()
