#!/usr/bin/env python3
"""82: RTI's packets in text and in binary, over the recorder link — and what each costs RTI. NOTHING MOVES.

The binary packets and the recorder link (2026-11-26, TODO/p4_recording_plan.md 2B) end to end on the running robot:

  1. the `record` recipe (every tag but the gains) is chosen;
  2. for each format, text then binary: the format is set, this program connects to the robot's TCP port 30004 (the
     recorder link) and reads `--seconds` of packets: every one in the asked format, every one decoded
     (robotclient.telemetry), no tick missing; in binary every tag of the recipe present;
  3. RTI's own loop execution time per tick (`exec_t`) in each format: the median, p99 and maximum, and the
     difference — what the text costs RTI against the binary copy. Read from RTI's trace in shared memory (every tick
     of the window, whatever the recipe) when this runs on the robot; from the packets otherwise (the recipe must then
     carry exec_t).

The recipe and the format are put back as they were at the end (also on Ctrl-C). Any mode, enabled or not; it takes
no control. Refused when another program holds the recorder link.

    .venv/bin/python api_tests/82_telemetry_formats.py            # dry run: prints the plan
    .venv/bin/python api_tests/82_telemetry_formats.py --go
    .venv/bin/python api_tests/82_telemetry_formats.py --go --seconds 5
    .venv/bin/python api_tests/82_telemetry_formats.py --go --recipe standard   # the everyday recipe's cost
"""
import argparse
import math
import os
import socket
import statistics
import sys
import time

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)

from robotclient import RobotClient                                     # noqa: E402
from robotclient.telemetry import TelemetryClient, decode_packet, is_binary  # noqa: E402


def read_format(host, fmt, seconds, record_tags, reader=None):
    """`seconds` of packets over the recorder link -> a result dict (the checks' inputs). `reader`: RTI's trace
    (utility.trace_ring.TraceReader) for exec_t of every tick in the window, or None."""
    if reader is not None:
        reader.poll()                                    # the rows before the window: not counted
    out = {"fmt": fmt, "packets": 0, "wrong_format": 0, "undecoded": 0, "gaps": 0, "lost": 0, "missing_tags": set(),
           "bytes": [], "exec_us": [], "first_tick": None, "last_tick": None, "t_first": None, "t_last": None}
    last_tick = None
    with TelemetryClient(host) as tel:
        end = time.monotonic() + seconds
        first = True
        while time.monotonic() < end:
            raw = tel.read_raw(timeout=2.0)
            if first:                                    # the packets already queued may be of the old format
                first = False
                if is_binary(raw) != (fmt == "binary"):
                    continue
            out["packets"] += 1
            out["bytes"].append(len(raw))
            now = time.monotonic()
            out["t_first"] = now if out["t_first"] is None else out["t_first"]
            out["t_last"] = now
            if is_binary(raw) != (fmt == "binary"):
                out["wrong_format"] += 1
                continue
            try:
                st = decode_packet(raw)
            except ValueError:
                out["undecoded"] += 1
                continue
            if fmt == "binary":
                out["missing_tags"] |= set(record_tags) - set(st)
            tick = st.get("tick")
            if isinstance(tick, float) and not math.isnan(tick):
                if last_tick is not None and tick != last_tick + 1:
                    out["gaps"] += 1
                    out["lost"] += int(max(0, tick - last_tick - 1))
                last_tick = tick
                if out["first_tick"] is None:
                    out["first_tick"] = tick
                out["last_tick"] = tick
            ex = st.get("exec_t")
            if reader is None and isinstance(ex, float) and not math.isnan(ex):
                out["exec_us"].append(ex * 1e6)
    if reader is not None:
        rows, _lost = reader.poll()
        col = dict((n, c) for n, c in reader.layout["scalars"])["exec_t"]
        out["exec_us"] = [v * 1e6 for v in rows[:, col].tolist() if not math.isnan(v)]
    return out


def stats(xs):
    if not xs:
        return "no values"
    s = sorted(xs)
    return (f"median {statistics.median(s):7.1f} us   p99 {s[int(len(s) * 0.99)]:7.1f} us   max {s[-1]:7.1f} us   "
            f"({len(s)} ticks)")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--seconds", type=float, default=2.0, help="packets read in each format (default 2 s)")
    ap.add_argument("--recipe", default="record", help="the recipe used (default record: every tag but the gains)")
    ap.add_argument("--go", action="store_true")
    args = ap.parse_args()
    from data.data_recipe import load_recipe_from_file
    rec = load_recipe_from_file(os.path.join(_ROOT, "data_recipe", f"{args.recipe}.xml"))
    if rec is None:
        print(f"REFUSED: no recipe data_recipe/{args.recipe}.xml")
        return 2
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    st: dict = {}
    end = time.monotonic() + 3.0                         # the status the robot pushes (get_status() is another dict)
    while time.monotonic() < end and "recipe" not in st:
        st = robot.latest_status() or {}
        time.sleep(0.05)
    tele = st.get("telemetry") or {}
    recipe0, fmt0 = st.get("recipe"), tele.get("format")
    print(f"Now: recipe {recipe0}, format {fmt0}, UDP to {tele.get('destination')}:{tele.get('port')}, "
          f"recorder link TCP {tele.get('tcp_port')} ({tele.get('tcp_peer') or 'free'})")
    print(f"Plan: recipe {args.recipe!r} ({len(rec.fields)} tags); {args.seconds:g} s of packets over TCP 30004 as text, then "
          f"as binary; RTI's exec_t in each; then recipe {recipe0!r} and format {fmt0!r} again. Nothing moves.")
    if not tele:
        print("REFUSED: the status has no 'telemetry' field — RTI / the executor run code from before 2026-11-26 "
              "(restart the robot service).")
        return 2
    if tele.get("tcp_peer"):
        print(f"REFUSED: another program holds the recorder link ({tele['tcp_peer']}).")
        return 2
    if not args.go:
        print("DRY RUN: re-run with --go. Nothing done.")
        return 0
    reader = None
    if args.host in ("127.0.0.1", "localhost"):          # on the robot: RTI's own exec_t of every tick
        try:
            from utility.trace_ring import TraceReader
            reader = TraceReader.attach()
        except Exception as e:  # noqa: BLE001
            print(f"(RTI's trace not readable here: {e}; exec_t from the packets)")
    print(f"exec_t from {'RTI trace (every tick)' if reader is not None else 'the packets'}")
    results = []
    ok = True
    try:
        robot.set_recipe(args.recipe)
        for fmt in ("text", "binary"):
            robot.set_telemetry_format(fmt)
            time.sleep(0.1)
            try:
                results.append(read_format(args.host, fmt, args.seconds, rec.fields, reader))
            except (OSError, ConnectionError) as e:
                print(f"FAIL  {fmt}: the recorder link: {e}")
                ok = False
            time.sleep(0.2)                              # the link is free again for the next connect (25 ticks)
    finally:
        try:
            if fmt0:
                robot.set_telemetry_format(fmt0)
            if recipe0:
                robot.set_recipe(recipe0)
        finally:
            robot.disconnect()
    for r in results:
        sizes = f"{statistics.median(r['bytes']):.0f} B" if r["bytes"] else "-"
        print(f"\n== {r['fmt']}: {r['packets']} packets, {sizes} each ==")
        span = (int(r["last_tick"] - r["first_tick"] + 1) if r["first_tick"] is not None else 0)
        # every tick from the first to the last, each once — not a fixed count: an RTI that runs slow (the simulator:
        # no real-time core) runs fewer ticks in the same seconds, and that is no loss. The rate is taken from the first
        # packet to the last (the read window also holds the wait for RTI to accept the link: up to 25 ticks)
        dt = (r["t_last"] - r["t_first"]) if r["t_first"] is not None else 0.0
        rate = f"RTI ran {(span - 1) / dt:.0f} ticks a second" if dt > 0 else "no rate"
        print(f"  {span} ticks in {dt:.2f} s from the first packet to the last: {rate}")
        checks = [
            (f"{r['fmt']}: every tick from the first to the last arrived, each once ({r['packets'] - r['wrong_format']}"
             f" packets, ticks {r['first_tick']}..{r['last_tick']})",
             span > 0 and r["packets"] - r["wrong_format"] - r["undecoded"] == span),
            (f"{r['fmt']}: every packet in the asked format", r["wrong_format"] == 0),
            (f"{r['fmt']}: every packet decoded", r["undecoded"] == 0),
            (f"{r['fmt']}: no tick missing over TCP ({r['gaps']} gaps, {r['lost']} ticks lost)", r["gaps"] == 0),
        ]
        if r["fmt"] == "binary":
            checks.append((f"binary: every tag of the recipe present (missing: {sorted(r['missing_tags'])})",
                           not r["missing_tags"]))
        for name, good in checks:
            print(("  PASS  " if good else "  FAIL  ") + name)
            ok &= bool(good)
        print(f"  RTI's exec_t: {stats(r['exec_us'])}")
    if len(results) == 2 and results[0]["exec_us"] and results[1]["exec_us"]:
        d = statistics.median(results[0]["exec_us"]) - statistics.median(results[1]["exec_us"])
        print(f"\nText costs RTI {d:+.1f} us a tick more than binary (median exec_t, the {args.recipe} recipe).")
    print("\nRESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\nstopped (the recipe and the format were put back)")
        sys.exit(1)
    except socket.timeout:
        print("FAIL: no packet for 2 s on the recorder link")
        sys.exit(1)
