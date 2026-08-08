"""Soak-test the CAN firmware flasher against a bench stack of driver boards.

Answers the question the single-shot flasher cannot: **how much margin is
there?** A flash that succeeds after 40 chunk retries and one that succeeds with
zero both print "OK". Only the retry counts tell them apart, and only repetition
tells you whether the 1-in-50 failure exists.

The specific unknown this exists to measure: the MCP2515 has a small controller
FIFO, and the flasher blasts 16 back-to-back stream frames per chunk. The retry
ladder (5 chunk retries inside 10 page retries) should absorb any losses — but
"should" is an assumption until it has a number. This turns
`DEFAULT_STREAM_CHUNK_FRAMES = 16` from a value copied out of a reference
implementation into a measured choice.

MEANT FOR A BENCH RIG, NOT A ROBOT. Flashing the same board dozens of times in a
row is fine for bare daisy-chained boards and a bad idea on an assembled arm.

    # 30 flashes of node 2 at the default chunk size
    python3 tools/soak_flash_test.py --node 2 --file fw.bin --iterations 30

    # sweep chunk sizes to find where it breaks (or prove it doesn't)
    python3 tools/soak_flash_test.py --node 2 --file fw.bin \
        --iterations 20 --chunk-frames 32,16,8,4,1

    # exercise every board in the stack, round-robin
    python3 tools/soak_flash_test.py --node 0,1,2,3,4,5,6 --file fw.bin --iterations 5

Flash the version the boards ALREADY RUN. A byte-identical image means any
failure is the tool's fault rather than the firmware's, and it leaves the rig in
the state you found it.

Results are written to temp_data/firmware_soak/ as JSON, alongside a summary on
the terminal. Ctrl-C reports whatever completed rather than throwing it away.
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import flash_motor_firmware as F
from project_paths import TEMP_DATA_DIR

RESULTS_DIR = TEMP_DATA_DIR / "firmware_soak"


def parse_int_list(text):
    return [int(part) for part in str(text).split(",") if str(part).strip()]


def run_one(bus, node, firmware, chunk_frames, erase, frame_gap):
    """One flash. Returns a result dict; never raises for a flash failure."""
    stats = F.new_stats()
    started = time.perf_counter()
    ok, error = True, None
    try:
        F.flash_firmware(
            bus, node, firmware,
            erase_first=erase,
            stream_chunk_frames=chunk_frames,
            stream_frame_gap_s=frame_gap,
            on_log=None, on_progress=None,
            stats=stats,
        )
    except (F.BootloaderError, F.FlashCancelled, ValueError) as exc:
        ok, error = False, str(exc)
    except Exception as exc:
        # Anything else (a full TX buffer when nothing is ACKing, a socket
        # error) must not destroy a soak dataset that took an hour to collect.
        # Record it as a failed iteration and keep going.
        ok, error = False, f"{type(exc).__name__}: {exc}"
    elapsed = time.perf_counter() - started
    return {"node": node, "chunk_frames": chunk_frames, "ok": ok,
            "seconds": round(elapsed, 2), "error": error, **stats}


def summarise(runs):
    """Per-(chunk_frames) aggregate. Retries are the headline, not just pass rate."""
    groups = {}
    for run in runs:
        groups.setdefault(run["chunk_frames"], []).append(run)

    rows = []
    for chunk in sorted(groups, reverse=True):
        group = groups[chunk]
        good = [r for r in group if r["ok"]]
        times = [r["seconds"] for r in good]
        rows.append({
            "chunk_frames": chunk,
            "runs": len(group),
            "ok": len(good),
            "failed": len(group) - len(good),
            "pct": 100.0 * len(good) / len(group) if group else 0.0,
            "page_retries": sum(r["page_retries"] for r in group),
            "chunk_retries": sum(r["chunk_retries"] for r in group),
            # Only SUCCESSFUL zero-retry runs are "clean". A run that failed
            # without retrying (e.g. the board never answered a ping) has zero
            # retries too, and counting it here would report a broken bus as
            # pristine — the exact opposite of what this column is for.
            "clean_runs": sum(1 for r in good
                              if r["page_retries"] == 0 and r["chunk_retries"] == 0),
            "median_s": round(statistics.median(times), 2) if times else None,
            "min_s": round(min(times), 2) if times else None,
            "max_s": round(max(times), 2) if times else None,
        })
    return rows


def print_report(rows, runs):
    print("\n" + "=" * 78)
    print("SOAK RESULTS")
    print("=" * 78)
    header = (f"{'chunk':>5} {'runs':>5} {'ok':>4} {'fail':>5} {'pass%':>7} "
              f"{'clean':>6} {'pgRtry':>7} {'chRtry':>7} "
              f"{'min':>6} {'med':>6} {'max':>6}")
    print(header)
    print("-" * len(header))
    for row in rows:
        print(f"{row['chunk_frames']:>5} {row['runs']:>5} {row['ok']:>4} "
              f"{row['failed']:>5} {row['pct']:>6.1f}% {row['clean_runs']:>6} "
              f"{row['page_retries']:>7} {row['chunk_retries']:>7} "
              f"{_fmt(row['min_s']):>6} {_fmt(row['median_s']):>6} "
              f"{_fmt(row['max_s']):>6}")

    print("\n'clean' = runs that needed ZERO retries. That column is the real "
          "margin —\na 100% pass rate carried by the retry ladder is not the "
          "same as a healthy bus.")

    failures = [r for r in runs if not r["ok"]]
    if failures:
        print(f"\n{len(failures)} FAILURE(S):")
        seen = {}
        for run in failures:
            seen.setdefault(run["error"], []).append(run)
        for error, group in seen.items():
            nodes = sorted({r['node'] for r in group})
            chunks = sorted({r['chunk_frames'] for r in group})
            print(f"  x{len(group):<3} node(s) {nodes} chunk(s) {chunks}: {error}")
        print("\nA board that failed mid-flash is sitting in its bootloader. "
              "Recover with:\n  python3 tools/flash_motor_firmware.py scan --no-reset")
    else:
        print("\nNo failures.")


def _fmt(value):
    return "-" if value is None else f"{value:.1f}"


def main():
    parser = argparse.ArgumentParser(
        description="Repeat-flash a bench stack to measure flasher reliability. "
                    "NOT for an assembled robot.")
    parser.add_argument("--node", required=True,
                        help="board id, or a comma list to round-robin (e.g. 0,1,2)")
    parser.add_argument("--file", required=True, help="firmware .bin")
    parser.add_argument("--iterations", type=int, default=20,
                        help="flashes per node per chunk size (default 20)")
    parser.add_argument("--chunk-frames", default=str(F.DEFAULT_STREAM_CHUNK_FRAMES),
                        help="chunk size, or a comma list to sweep (e.g. 32,16,8,4)")
    parser.add_argument("--frame-gap", type=float, default=0.0,
                        help="pause between stream frames, seconds (default 0)")
    parser.add_argument("--no-erase", action="store_true",
                        help="skip ERASE_APP each iteration (faster, less realistic)")
    parser.add_argument("--stop-on-fail", action="store_true",
                        help="halt at the first failure instead of continuing")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--yes", action="store_true", help="skip the confirmation")
    args = parser.parse_args()

    nodes = parse_int_list(args.node)
    chunk_sizes = parse_int_list(args.chunk_frames)
    for node in nodes:
        if not 0 <= node <= F.BL_BOARD_ID_MAX:
            sys.exit(f"Invalid board id {node} — must be 0..{F.BL_BOARD_ID_MAX}")
    for chunk in chunk_sizes:
        if not 1 <= chunk <= F.CAN_STREAM_FRAME_COUNT:
            sys.exit(f"Invalid chunk size {chunk} — must be 1..{F.CAN_STREAM_FRAME_COUNT}")

    path = Path(args.file)
    if not path.is_file():
        sys.exit(f"No such firmware file: {path}")
    firmware = path.read_bytes()
    validation = F.validate_firmware_bytes(firmware)
    print(F.format_validation(validation))
    if not validation.ok:
        sys.exit("Refusing to soak-test with an image that fails validation.")

    ok, reason = F.bus_available()
    if not ok:
        sys.exit(reason)
    print(f"  {reason}")

    total = len(nodes) * len(chunk_sizes) * args.iterations
    print(f"\nPlan: {len(nodes)} node(s) x {len(chunk_sizes)} chunk size(s) "
          f"x {args.iterations} iterations = {total} flashes")
    if not args.yes:
        print("\nThis reflashes the same boards over and over. Intended for a BENCH "
              "STACK,\nnot an assembled robot.")
        if input("Type 'yes' to start: ").strip().lower() != "yes":
            sys.exit("Aborted.")

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    runs = []
    started = time.perf_counter()
    stamp = time.strftime("%Y%m%d_%H%M%S")
    results_path = RESULTS_DIR / f"soak_{stamp}.json"

    def save_results():
        """Write the whole result file. Called after EVERY run, not just at the end.

        A sweep is ~47 minutes and Ctrl-C is not the only way one dies: a killed
        terminal, an SSH drop or a box crash takes the process out without
        running the handler, and an end-of-run-only save turns 100 flashes into
        zero data. That happened (2026-08-04). Rewriting a small JSON once per
        ~28 s flash costs nothing next to the flash itself, and caps the loss at
        one iteration.
        """
        results_path.write_text(json.dumps({
            "when": stamp,
            "firmware": str(Path(args.file).resolve()),
            "firmware_bytes": len(firmware),
            "pages": validation.pages,
            "nodes": nodes,
            "chunk_sizes": chunk_sizes,
            "iterations": args.iterations,
            "erase_each_time": not args.no_erase,
            "frame_gap_s": args.frame_gap,
            "planned_flashes": total,
            "complete": len(runs) == total,
            "elapsed_s": round(time.perf_counter() - started, 1),
            "summary": summarise(runs),
            "runs": runs,
        }, indent=2))

    bus = F.open_bus(args.channel)
    try:
        index = 0
        for chunk in chunk_sizes:
            for iteration in range(args.iterations):
                for node in nodes:
                    index += 1
                    print(f"[{index}/{total}] node {node} chunk {chunk} "
                          f"iter {iteration + 1}/{args.iterations} ... ",
                          end="", flush=True)
                    result = run_one(bus, node, firmware, chunk,
                                     not args.no_erase, args.frame_gap)
                    runs.append(result)
                    save_results()
                    if result["ok"]:
                        extra = ""
                        if result["page_retries"] or result["chunk_retries"]:
                            extra = (f"  (retries: page {result['page_retries']}, "
                                     f"chunk {result['chunk_retries']})")
                        print(f"OK {result['seconds']:.1f}s{extra}")
                    else:
                        print(f"FAIL — {result['error']}")
                        if args.stop_on_fail:
                            raise KeyboardInterrupt
                    # Let the board finish rebooting into the app before the next
                    # pass, so each iteration starts from the same state (the
                    # bootloader needs ~3 s of silence to hand over).
                    time.sleep(3.5)
    except KeyboardInterrupt:
        print("\nInterrupted — reporting what completed.")
    finally:
        F.close_bus(bus)

    if not runs:
        sys.exit("No runs completed.")

    rows = summarise(runs)
    print_report(rows, runs)

    save_results()
    if len(runs) < total:
        print(f"\nINCOMPLETE: {len(runs)}/{total} flashes — the file records this "
              f"as complete=false.")
    print(f"\nSaved {results_path}")

    return 0 if all(r["ok"] for r in runs) else 1


if __name__ == "__main__":
    sys.exit(main())
