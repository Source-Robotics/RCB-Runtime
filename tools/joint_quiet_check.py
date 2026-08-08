"""Per-joint idle-noise signature — the A/B measurement for "is this joint quiet?".

Written 2026-08-06 to compare one arm against another after J5 on the first PAR6
was found oscillating (sd 164 mA, 148 encoder ticks peak-to-peak) while every
other joint sat within a few mA on IDENTICAL gains.

Read-only: samples shared memory, commands nothing. Whatever mode the robot is
in is what gets measured, so record it — an IDLE gravity-comp hold and an IDLE
velocity-0 hold are different tests.

    python3 tools/joint_quiet_check.py --seconds 30 --label robotA
    python3 tools/joint_quiet_check.py --seconds 30 --label robotB
    python3 tools/joint_quiet_check.py --compare robotA robotB

The number that matters is CURRENT SD. A quiet joint holds a steady current;
an oscillating one swings while going nowhere. Position peak-to-peak confirms
whether the joint is physically moving or just fighting electrically.
"""

import argparse
import json
import statistics
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

RESULTS_DIR = Path(__file__).resolve().parent.parent / "temp_data" / "quiet_check"


def capture(seconds, rate_hz):
    from data import shared_data
    m, r, j = shared_data.motor, shared_data.rti, shared_data.joint
    n = int(shared_data.joint_num) if hasattr(shared_data, "joint_num") else 6

    meta = {
        "mode": shared_data.robot_mode.get(),
        "state": shared_data.robot_state.get(),
        "homed": shared_data.homed_state.get(),
        "gravity_comp": float(r["gravity_comp_enabled"][0]),
    }
    cur = {i: [] for i in range(n)}
    pos = {i: [] for i in range(n)}
    period = 1.0 / rate_hz
    end = time.time() + seconds
    while time.time() < end:
        for i in range(n):
            cur[i].append(float(m["motor_current"][i]))
            pos[i].append(float(m["motor_position"][i]))
        time.sleep(period)

    joints = {}
    for i in range(n):
        c, p = cur[i], pos[i]
        joints[f"J{i + 1}"] = {
            "mean_mA": round(statistics.mean(c), 1),
            "sd_mA": round(statistics.pstdev(c), 1),
            "min_mA": round(min(c), 0),
            "max_mA": round(max(c), 0),
            "pos_p2p_ticks": round(max(p) - min(p), 0),
            "temp_C": round(float(m["motor_temperature"][i]), 1),
            "kt_can": round(float(m["motor_Kt"][i]), 4),
        }
    return {"meta": meta, "samples": len(cur[0]), "seconds": seconds,
            "rate_hz": rate_hz, "joints": joints}


def show(result, title):
    meta = result["meta"]
    print(f"\n=== {title} ===")
    print(f"mode={meta['mode']} state={meta['state']} homed={meta['homed']} "
          f"gravcomp={meta['gravity_comp']}  "
          f"({result['samples']} samples / {result['seconds']}s)")
    print(f"{'J':<4}{'mean mA':>9}{'sd mA':>8}{'min':>7}{'max':>7}"
          f"{'pos p2p':>9}{'temp C':>8}{'Kt':>8}   verdict")
    for name, d in result["joints"].items():
        # A joint holding station should sit within a few mA. Ten times that is
        # not noise, it is a limit cycle — that was the J5 signature.
        verdict = "QUIET" if d["sd_mA"] < 15 else (
            "NOISY" if d["sd_mA"] < 50 else "*** OSCILLATING ***")
        print(f"{name:<4}{d['mean_mA']:9.1f}{d['sd_mA']:8.1f}{d['min_mA']:7.0f}"
              f"{d['max_mA']:7.0f}{d['pos_p2p_ticks']:9.0f}{d['temp_C']:8.1f}"
              f"{d['kt_can']:8.4f}   {verdict}")


def compare(a, b, label_a, label_b):
    print(f"\n=== {label_a}  vs  {label_b} ===")
    print(f"{'J':<4}{'sd A':>9}{'sd B':>9}{'ratio':>8}   {'p2p A':>7}{'p2p B':>7}")
    for name in a["joints"]:
        if name not in b["joints"]:
            continue
        sa, sb = a["joints"][name]["sd_mA"], b["joints"][name]["sd_mA"]
        ratio = f"{sa / sb:.1f}x" if sb > 0.01 else "-"
        flag = "  <-- differs" if sb > 0.01 and (sa / sb > 3 or sb / sa > 3) else ""
        print(f"{name:<4}{sa:9.1f}{sb:9.1f}{ratio:>8}   "
              f"{a['joints'][name]['pos_p2p_ticks']:7.0f}"
              f"{b['joints'][name]['pos_p2p_ticks']:7.0f}{flag}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--rate", type=float, default=100.0, help="sample rate Hz")
    ap.add_argument("--label", help="save the capture under this name")
    ap.add_argument("--compare", nargs=2, metavar=("A", "B"),
                    help="compare two saved captures instead of capturing")
    args = ap.parse_args()

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    if args.compare:
        a_path = RESULTS_DIR / f"{args.compare[0]}.json"
        b_path = RESULTS_DIR / f"{args.compare[1]}.json"
        for path in (a_path, b_path):
            if not path.is_file():
                sys.exit(f"No such capture: {path}")
        a = json.loads(a_path.read_text())
        b = json.loads(b_path.read_text())
        show(a, args.compare[0])
        show(b, args.compare[1])
        compare(a, b, *args.compare)
        return

    print(f"Sampling {args.seconds:.0f}s at {args.rate:.0f}Hz — read-only, "
          f"nothing is commanded.")
    result = capture(args.seconds, args.rate)
    show(result, args.label or "capture")
    if args.label:
        path = RESULTS_DIR / f"{args.label}.json"
        path.write_text(json.dumps(result, indent=2))
        print(f"\nsaved {path}")


if __name__ == "__main__":
    main()
