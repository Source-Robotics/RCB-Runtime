#!/usr/bin/env python3
"""47: why does J6's measured position advance unevenly? Record the CAN bus and RTI's telemetry together.

api_tests/46 found J6's measured position advancing in a repeating 3-tick pattern at constant commanded
speed (0.42, 0.25, 0.78 deg per 4 ms tick, averaging exactly the command), which made its raw speed read
1.2x its ceiling. J4 and J5 did not do it. Two very different causes would look the same in telemetry:

  DRIVE-SIDE   the J6 drive itself reports positions sampled unevenly (its CAN replies already
               carry the pattern) — firmware / encoder / control-loop timing on that board.
  RTI-SIDE     the drive reports evenly, but RTI sometimes uses a stale or early reply for J6 —
               CAN scheduling / receive timing on the Pi.

CLAUDE.md's rule for exactly this: record the CAN bus and the host side at the same moment before
theorising. So this runs `candump` on can0 (read-only — it sends nothing) while recording RTI's per-tick
telemetry, moves J4, J5 and J6 together at constant speed (J4/J5 are the controls), and compares, per
joint, the position steps in the drive's own replies with the steps RTI published.

    python3 api_tests/47_j6_sampling.py --selftest   # check the analysis on synthetic data
    python3 api_tests/47_j6_sampling.py              # dry run: prints the plan, moves nothing
    python3 api_tests/47_j6_sampling.py --go

Moves only J4, J5, J6 (+amp then back, 50 % EXEC speed by default). The robot must be ENABLED and
homed; the script enters EXEC itself (no motion), takes control without forcing, switches the telemetry
recipe for the run and restores it. Results: printed + temp_data/j6_sampling/<time>/.
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _ROOT)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

CMD_DATA_PACK_1 = 2          # RTI -> drive (Spectral_BLDC SEND_DATA_PACK_1_ID)
RESP_DATA_PACK_1 = 3         # drive -> RTI: position(24b) speed(24b) current(16b), big-endian signed
JOINT_NODES = 6              # CAN nodes 0..5 = J1..J6


# ----------------------------------------------------------------------------- CAN log
def parse_candump(lines):
    """candump -L lines -> (t, node, cmd, pos, spd) arrays. Only standard-ID frames."""
    t, node, cmd, pos, spd = [], [], [], [], []
    for ln in lines:
        ln = ln.strip()
        if not ln.startswith("("):
            continue
        try:
            ts, _iface, frame = ln.split()
            cid_hex, data_hex = frame.split("#", 1)
            cid = int(cid_hex, 16)
            data = bytes.fromhex(data_hex) if data_hex and not data_hex.startswith("R") else b""
        except ValueError:
            continue
        n, c = (cid >> 7) & 0xF, (cid >> 1) & 0x3F
        p = s = np.nan
        if c == RESP_DATA_PACK_1 and len(data) == 8:
            p = int.from_bytes(data[0:3], "big", signed=True)
            s = int.from_bytes(data[3:6], "big", signed=True)
        t.append(float(ts.strip("()")))
        node.append(n)
        cmd.append(c)
        pos.append(p)
        spd.append(s)
    return (np.asarray(t), np.asarray(node, int), np.asarray(cmd, int), np.asarray(pos, float),
            np.asarray(spd, float))


def per_tick(t, node, cmd, pos, spd, joints0):
    """Group the bus into RTI ticks, marked by RTI's command to node 0. For each tick and joint: the
    FIRST position reply that arrived before the next tick's marker, its latency after that joint's own
    command, and how many replies came."""
    marks = np.sort(t[(node == 0) & (cmd == CMD_DATA_PACK_1)])
    n = max(marks.size - 1, 0)
    out = {}
    for j in joints0:
        cmd_t = np.sort(t[(node == j) & (cmd == CMD_DATA_PACK_1)])
        r = (node == j) & (cmd == RESP_DATA_PACK_1)
        rt, rp, rs = t[r], pos[r], spd[r]
        order = np.argsort(rt)
        rt, rp, rs = rt[order], rp[order], rs[order]
        p = np.full(n, np.nan); s = np.full(n, np.nan); lat = np.full(n, np.nan); cnt = np.zeros(n, int)
        lo = np.searchsorted(rt, marks[:-1])
        hi = np.searchsorted(rt, marks[1:])
        clo = np.searchsorted(cmd_t, marks[:-1])
        for i in range(n):
            cnt[i] = hi[i] - lo[i]
            if cnt[i]:
                k = lo[i]
                p[i], s[i] = rp[k], rs[k]
                if clo[i] < cmd_t.size and cmd_t[clo[i]] < marks[i + 1]:
                    lat[i] = rt[k] - cmd_t[clo[i]]
        out[j] = {"pos": p, "spd": s, "lat": lat, "count": cnt}
    gaps = np.diff(marks)
    return out, marks, gaps


# ----------------------------------------------------------------------------- unevenness
def unevenness(series):
    """Step irregularity of a position series during CONSTANT-speed motion.

    Takes the per-tick increments, keeps the cruise (increments within 50 %..150 % of the local median
    AND the local median well away from zero), and returns (cv, period): the coefficient of variation of
    the increments (0 = perfectly even) and the dominant repeat period in ticks (autocorrelation peak,
    2..8), or None when there is no clear repetition."""
    x = np.asarray(series, float)
    d = np.diff(x)
    d = d[np.isfinite(d)]
    if d.size < 30:
        return None, None
    med = np.median(np.abs(d))
    if med <= 0:
        return None, None
    # cruise = where a 9-tick moving median of the steps is within 20 % of the overall cruise step
    k = 9
    pad = np.pad(d, (k // 2, k // 2), mode="edge")
    local = np.median(np.lib.stride_tricks.sliding_window_view(pad, k), axis=1)
    top = np.percentile(np.abs(local), 90)
    cruise = np.abs(local) > 0.8 * top
    if cruise.sum() < 24:
        return None, None
    # Each step RELATIVE TO ITS LOCAL MEDIAN: removes the slow trend at the ends of the cruise, which
    # otherwise correlates with itself at every lag and reads as a spurious period.
    z = d[cruise] / local[cruise] - 1.0
    cv = float(np.std(z))
    z = z - z.mean()
    if not np.any(z):
        return cv, None
    ac = np.array([np.dot(z[:-L], z[L:]) / np.dot(z, z) for L in range(1, 9)])
    lag = int(np.argmax(ac[1:]) + 2)                       # periods 2..8
    period = lag if ac[lag - 1] > 0.3 else None
    return cv, period


def best_lag(a, b):
    """Shift of b against a (ticks) that best matches their step series, and |correlation|.

    Searches the WHOLE recording: the CAN log starts seconds before the telemetry capture, so a
    +/-10-tick window (the first version) could never find the match and reported noise. The sign of
    the correlation is dropped because these motors count opposite to the joint (negative gear ratio)."""
    da, db = np.nan_to_num(np.diff(np.asarray(a, float))), np.nan_to_num(np.diff(np.asarray(b, float)))
    if da.size < 30 or db.size < 30:
        return None, None
    za = (da - da.mean()) / (da.std() or 1.0)
    zb = (db - db.mean()) / (db.std() or 1.0)
    xc = np.correlate(za, zb, mode="full")
    lag = int(np.argmax(np.abs(xc))) - (zb.size - 1)
    # Score the match over the OVERLAP only; the non-overlapping ends would dilute it.
    if lag >= 0:
        x, y = da[lag:], db
    else:
        x, y = da, db[-lag:]
    n = min(x.size, y.size)
    if n < 30 or np.std(x[:n]) == 0 or np.std(y[:n]) == 0:
        return lag, 0.0
    return lag, float(abs(np.corrcoef(x[:n], y[:n])[0, 1]))


def verdict(cv_can, cv_rti, even=0.15):
    if cv_can is None or cv_rti is None:
        return "no cruise data"
    if cv_can > even and cv_rti > even:
        return "DRIVE-SIDE: the drive's own CAN replies are already uneven"
    if cv_can <= even < cv_rti:
        return "RTI-SIDE: the drive reports evenly, RTI's published steps are uneven"
    if cv_can <= even and cv_rti <= even:
        return "even in both"
    return "CAN uneven but RTI even (unexpected)"


# ----------------------------------------------------------------------------- self-test
def selftest():
    ok_all = True

    def chk(name, cond, detail=""):
        nonlocal ok_all
        ok_all &= bool(cond)
        print(f"  {'PASS' if cond else 'FAIL'}  {name}" + ("" if cond else f"  [{detail}]"))

    rng = np.random.default_rng(1)
    n = 600
    ramp = np.concatenate([np.linspace(0, 1, 50), np.ones(n - 100), np.linspace(1, 0, 50)])
    even = np.cumsum(ramp * 100.0)
    pattern = np.tile([0.9, 0.5, 1.6], n // 3 + 1)[:n]     # the 3-tick shape seen on J6
    uneven = np.cumsum(ramp * 100.0 * pattern)
    cv_e, per_e = unevenness(even + rng.normal(0, 0.5, n))
    cv_u, per_u = unevenness(uneven)
    chk("an even series reads even", cv_e is not None and cv_e < 0.05, cv_e)
    chk("the J6 pattern reads uneven", cv_u is not None and cv_u > 0.3, cv_u)
    chk("...with period 3", per_u == 3, per_u)
    chk("even series has no strong period", per_e is None, per_e)
    # Alignment needs a NON-periodic series: a perfectly periodic one matches equally well at shifts
    # one period apart (real data has ramps and speed changes, which make the match unique).
    rough = np.cumsum(ramp * 100.0 * rng.uniform(0.5, 1.5, n))
    lag, r = best_lag(rough, np.concatenate([rough[2:], [np.nan, np.nan]]))
    chk("alignment finds a 2-tick shift", lag == 2 and r > 0.95, (lag, r))
    far = np.concatenate([np.zeros(300), rough])
    lag, r = best_lag(far, -rough)            # 300 ticks apart AND sign-flipped, like the real capture
    chk("alignment finds a 300-tick, sign-flipped match", lag == 300 and r > 0.95, (lag, r))
    chk("verdict drive-side", verdict(cv_u, cv_u).startswith("DRIVE"))
    chk("verdict RTI-side", verdict(cv_e, cv_u).startswith("RTI"))

    # candump parsing + per-tick grouping: 3 ticks, node 5 replies 1 ms after its command
    lines = []
    for i in range(40):
        t0 = 1000.0 + i * 0.004
        for j in range(6):
            cid = (j << 7) | (CMD_DATA_PACK_1 << 1)
            lines.append(f"({t0 + j * 1e-4:.6f}) can0 {cid:03X}#0000000000000000")
            rid = (j << 7) | (RESP_DATA_PACK_1 << 1)
            posv = (i * 10 + j) & 0xFFFFFF
            data = posv.to_bytes(3, "big") + (5).to_bytes(3, "big") + (0).to_bytes(2, "big")
            lines.append(f"({t0 + j * 1e-4 + 0.001:.6f}) can0 {rid:03X}#{data.hex().upper()}")
    t, node, cmd, pos, spd = parse_candump(lines)
    chk("candump parsed (40 ticks x 12 frames)", t.size == 480, t.size)
    ticks, marks, gaps = per_tick(t, node, cmd, pos, spd, [5])
    j6 = ticks[5]
    chk("one reply per tick", np.all(j6["count"] == 1), np.unique(j6["count"]))
    chk("positions decoded", np.allclose(j6["pos"][:3], [5, 15, 25]), j6["pos"][:3])
    chk("latency 1.0 ms", np.allclose(j6["lat"], 0.001, atol=1e-6), j6["lat"][:3])
    print("selftest:", "ALL PASSED" if ok_all else "FAILED")
    return 0 if ok_all else 1


# ----------------------------------------------------------------------------- session
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--speed", type=float, default=50.0, help="EXEC speed percent (constant-speed cruise)")
    ap.add_argument("--iface", default="can0")
    args = ap.parse_args()
    if args.selftest:
        return selftest()

    import importlib
    base = importlib.import_module("46_monitor_baseline")      # telemetry capture + recipe helpers
    from robotclient import RobotClient
    from robotclient.rti_client import parse_state_packet

    amps = {3: 30.0, 4: 30.0, 5: 60.0}                           # 0-based J4, J5, J6
    robot = RobotClient(sync=False)
    robot.connect(args.host, args.port)
    capture = dump = None
    prev_recipe = None
    claimed = False
    out_dir = os.path.join(_ROOT, "temp_data", "j6_sampling", time.strftime("%Y%m%d_%H%M%S"))
    try:
        st = robot.status()
        q0 = np.asarray(robot.get_joints(), float)
        plus = q0.copy()
        for j, a in amps.items():
            plus[j] = q0[j] + a
        print(f"Joints now (deg): {np.round(q0, 2).tolist()}")
        print(f"Plan: J4 +30, J5 +30, J6 +60 deg together at {args.speed:.0f}%, then back; candump on "
              f"{args.iface} (read-only) + RTI telemetry the whole time.")
        if not args.go:
            print("DRY RUN: re-run with --go to execute. Nothing moved.")
            return 0
        if shutil.which("candump") is None:
            print("candump not found (apt package can-utils). Nothing moved.")
            return 2
        if "motion_aborted" not in st or st.get("state") != "ENABLED" or st.get("homed") is not True \
                or st.get("mode") not in ("IDLE", "EXEC"):
            print(f"Robot must be ENABLED + homed in IDLE/EXEC with the current executor (got mode="
                  f"{st.get('mode')} state={st.get('state')} homed={st.get('homed')}). Nothing moved.")
            return 2

        os.makedirs(out_dir, exist_ok=True)
        dump_path = os.path.join(out_dir, "can0.log")
        dump_file = open(dump_path, "w")
        dump = subprocess.Popen(["candump", "-L", args.iface], stdout=dump_file, stderr=subprocess.DEVNULL)
        capture = base.Capture()
        base.claim_control(robot)
        claimed = True
        if st.get("mode") != "EXEC":
            robot.enter_exec_mode(wait=True)
        prev_recipe = st.get("recipe")
        robot.reload_recipes()
        robot.set_recipe(base.RECIPE)
        time.sleep(1.0)
        capture.packets.clear()
        time.sleep(1.0)                                            # standstill reference
        for label, target in (("out", plus), ("back", q0)):
            print(f"  moving {label}")
            robot.move_joints(*[float(x) for x in target], speed=args.speed)
            fails = robot.wait_idle(timeout=60.0, raise_on_error=False)
            if fails:
                print(f"Move failed: {fails}")
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        try:
            robot.stop()
        except Exception:
            pass
    finally:
        if dump is not None:
            dump.send_signal(signal.SIGINT)
            try:
                dump.wait(timeout=3)
            except subprocess.TimeoutExpired:
                dump.kill()
        if capture is not None:
            capture.stop()
        try:
            if prev_recipe and prev_recipe != base.RECIPE:
                robot.set_recipe(prev_recipe)
        except Exception as exc:
            print(f"Could not restore recipe {prev_recipe!r}: {exc}")
        if claimed:
            try:
                robot.release_control()
            except Exception:
                pass
        robot.disconnect()

    if capture is None:
        return 0
    with open(os.path.join(out_dir, "can0.log")) as fh:
        t, node, cmd, pos, spd = parse_candump(fh)
    tele = base.to_arrays(capture.packets)
    report = analyse_session(t, node, cmd, pos, spd, tele)
    np.savez_compressed(os.path.join(out_dir, "telemetry.npz"), **tele)
    with open(os.path.join(out_dir, "report.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    print(f"\nSaved {os.path.relpath(out_dir, _ROOT)}/ (can0.log, telemetry.npz, report.json)")
    return 0


def analyse_session(t, node, cmd, pos, spd, tele):
    joints0 = [3, 4, 5]
    ticks, marks, gaps = per_tick(t, node, cmd, pos, spd, joints0)
    print("\n" + "=" * 78)
    print(f"CAN: {t.size} frames, {marks.size} RTI ticks, tick period median {np.median(gaps) * 1e3:.3f} ms, "
          f"max {gaps.max() * 1e3:.2f} ms" if gaps.size else "CAN: no ticks found")
    rep = {"frames": int(t.size), "ticks": int(marks.size)}
    print(f"{'':4s} {'replies/tick':>14s} {'latency p50':>12s} {'p99':>7s} {'max':>7s}  "
          f"{'CAN steps cv':>12s} {'per':>4s}  {'RTI steps cv':>12s} {'per':>4s}  {'align':>14s}  verdict")
    for j in joints0:
        d = ticks[j]
        counts = {int(k): int(v) for k, v in zip(*np.unique(d["count"], return_counts=True))}
        lat = d["lat"][np.isfinite(d["lat"])] * 1e3
        cv_c, per_c = unevenness(d["pos"])
        rti = tele["j_p"][:, j] if tele["tick"].size else np.array([])
        cv_r, per_r = unevenness(rti) if rti.size else (None, None)
        lag, r = best_lag(d["pos"], rti) if rti.size else (None, None)
        v = verdict(cv_c, cv_r)
        f = lambda x, k=2: "-" if x is None else f"{x:.{k}f}"
        print(f"J{j + 1:<3d} {str(counts):>14s} {f(np.median(lat) if lat.size else None):>12s} "
              f"{f(np.percentile(lat, 99) if lat.size else None):>7s} {f(lat.max() if lat.size else None):>7s}  "
              f"{f(cv_c, 3):>12s} {str(per_c):>4s}  {f(cv_r, 3):>12s} {str(per_r):>4s}  "
              f"{(f'{lag:+d} |r|={r:.2f}' if lag is not None else '-'):>14s}  {v}")
        rep[f"J{j + 1}"] = {"replies_per_tick": counts,
                             "latency_ms_p50": float(np.median(lat)) if lat.size else None,
                             "latency_ms_p99": float(np.percentile(lat, 99)) if lat.size else None,
                             "can_step_cv": cv_c, "can_period": per_c, "rti_step_cv": cv_r, "rti_period": per_r,
                             "align_lag_ticks": lag, "align_r": r, "verdict": v}
    print("\ncv = spread of the per-tick position steps during constant-speed cruise (0 = perfectly even);"
          "\nper = repeat period in ticks; align = shift between the CAN and RTI step series and their match.")
    return rep


if __name__ == "__main__":
    sys.exit(main())
