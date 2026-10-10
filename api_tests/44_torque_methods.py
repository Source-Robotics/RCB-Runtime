#!/usr/bin/env python3
"""44: does ruckig_advancing really cost more vibration than ruckig? THREE METHODS, ONE RUN.

MOVES THE ARM (J4/J5/J6, +-15 deg triangle at 15 deg/s, 50 Hz). Run on the ROBOT.

THE QUESTION. Two measurements of the same thing disagree, reproducibly, and only for the
advancing family (robot-local, worst joint, t_rms Nm):

                       40_jitter (shm sampler)    e3 (state-packet stream)
    ruckig_advancing         0.0978                     0.0765     <- +24% vs -4% against ruckig
    ruckig                   0.0788                     0.0794     <- these two AGREE

Both read the same raw joint_torque. So one of them is measuring something the other is not, and
"advancing costs 30% more torque" (servo_mode_options.md 15.3) cannot be quoted until that is
settled. This records the SAME blocks three ways at once:

  packets  the robot's state packet, one per tick, atomic (what e3 uses)
  shm      /dev/shm/joint_torque sampled every ~2 ms with the identical filter (what 40_jitter uses)
  bus      each motor's Iq current straight off the CAN bus via `candump` — independent of
           shared memory, of telemetry and of every line of this runtime. The referee.

Current and torque are proportional (torque = current x Kt x gear), so the ratio
advancing/ruckig is directly comparable between methods, whatever their units.

ABAB order (ruckig, advancing, ruckig, advancing) so a slow drift in the arm — temperature,
friction settling — shows up as A1 != A2 instead of masquerading as a config effect.

    python3 api_tests/44_torque_methods.py --go
"""
import argparse
import math
import os
import subprocess
import sys
import tempfile
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

W = 13                     # moving-average half-window: the SAME 27-sample filter as 40_jitter / e3
JOINTS = (3, 4, 5)
SHM = "/dev/shm"


# --------------------------------------------------------------------------------------------
# Pure analysis — unit-tested in dummy_test_scripts/test_torque_methods.py
# --------------------------------------------------------------------------------------------
def hf_rms(series, w=W):
    """RMS of (x - centred moving average of width 2w+1). The metric all three methods share."""
    n = len(series)
    if n <= w * 3:
        return float("nan")
    hf = [series[i] - sum(series[i - w:i + w + 1]) / (2 * w + 1) for i in range(w, n - w)]
    return math.sqrt(sum(x * x for x in hf) / len(hf))


def parse_candump(path, off, t_lo, t_hi, nodes=JOINTS):
    """{node: [current_mA, ...]} for DATA_PACK replies (command 3) inside [t_lo, t_hi].

    `candump -L` stamps CLOCK_REALTIME; `off` = time.time() - time.monotonic() converts it to the
    monotonic clock the rest of this script uses. Frame layout (Spectral_BLDC.py:218):
    position = bytes 0-2, speed = bytes 3-5, current = bytes 6-7, signed, big-endian.
    """
    out = {n: [] for n in nodes}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 3 or "#" not in p[2]:
                continue
            cid_s, data_s = p[2].split("#")
            if data_s.startswith("R") or len(data_s) != 16:      # remote frames / wrong DLC
                continue
            cid = int(cid_s, 16)
            if ((cid >> 1) & 0x3F) != 3:                         # RESPOND_DATA_PACK_1
                continue
            node = (cid >> 7) & 0xF
            if node not in out:
                continue
            t = float(p[0].strip("()")) - off
            if not (t_lo <= t <= t_hi):
                continue
            cur = int(data_s[12:16], 16)
            out[node].append(cur - 65536 if cur >= 32768 else cur)
    return out


def trim(rows, t_start, seconds, edge=0.5, tcol=0):
    """Rows within [t_start+edge, t_start+seconds-edge] — identical trimming for every method."""
    return [r for r in rows if t_start + edge <= r[tcol] <= t_start + seconds - edge]


# --------------------------------------------------------------------------------------------
# shm sampler — a faithful copy of 40_jitter's, so the number is the number 40_jitter prints
# --------------------------------------------------------------------------------------------
def _rd(name, n=6):
    import struct
    with open(f"{SHM}/{name}", "rb") as f:
        return list(struct.unpack(f"{n}d", f.read(8 * n)))


class Sampler(threading.Thread):
    FIELDS = ("joint_position", "joint_torque")
    MAX_ROWS = 20000

    def __init__(self):
        super().__init__(daemon=True)
        self.rows, self.error = [], None
        self._stop_evt, self._on = threading.Event(), False

    def run(self):
        while not self._stop_evt.is_set():
            if self._on:
                try:
                    self.rows.append((time.monotonic(),) + tuple(_rd(f) for f in self.FIELDS))
                    if len(self.rows) > self.MAX_ROWS:
                        del self.rows[:len(self.rows) - self.MAX_ROWS]
                except Exception as exc:
                    self.error = self.error or repr(exc)
            time.sleep(0.002)

    def start_window(self):
        self.rows = []
        self._on = True

    def end_window(self):
        self._on = False
        return list(self.rows)

    def stop(self):
        self._stop_evt.set()


# --------------------------------------------------------------------------------------------
def run_block(name, cfg, a, ec, robot, RTIClient, ControlLost, sampler):
    """One config, one block. Returns dict of per-method worst-joint values, or None."""
    rti = RTIClient(robot, clock="pc", transport="udp")
    log = tempfile.NamedTemporaryFile(prefix="candump_", suffix=".log", delete=False)
    dump = None
    try:
        # Start recording BEFORE anything is claimed. Once claimed there is no idle state: the
        # SOFT_RTI watchdog is 3 command intervals (60 ms at 50 Hz), and the first version of this
        # script spawned candump and slept 0.15 s AFTER the claim — RTI latched RTI_LINK_LOST,
        # disabled the robot and the block measured an arm at rest (2026-09-30). Between claim()
        # and release() only ec.keep_alive / ec.stream may run; the tripwire in
        # dummy_test_scripts/test_torque_methods.py enforces it.
        dump = subprocess.Popen(["candump", "-L", "can0"], stdout=log, stderr=subprocess.DEVNULL)
        time.sleep(0.3)                                   # not claimed yet: sleeping is fine here
        rti.connect(rate_hz=a.rate, transport="udp")
        time.sleep(0.5)
        ec.apply_config(robot, cfg)
        robot.rti_set_state_rate(0.0)
        first = rti.read_once(timeout=5.0)
        if not isinstance(first.get("j_t"), list):
            print("recipe lacks j_t — run e1_link.py")
            os.unlink(log.name)
            return None
        rti.claim()
        q0 = rti.read_once(timeout=2.0)["j_p"]
        ec.keep_alive(rti, q0, a.rate, 1.0)

        sampler.start_window()
        t_start = time.monotonic()
        cap, t0, chz = ec.stream(rti, q0, JOINTS, math.radians(a.amp), a.period, a.rate, a.seconds)
        rows_shm = sampler.end_window()
        ec.keep_alive(rti, q0, a.rate, 1.0)
        rti.release()
        rti.close()
        dump.terminate()                                  # released: waiting is fine again
        dump.wait(timeout=3)
    except ControlLost as exc:
        print(f"CONTROL LOST: {exc}")
        try:
            robot.enter_idle_mode()
        except Exception:
            pass
        try:
            os.unlink(log.name)
        except OSError:
            pass
        return None
    finally:
        if dump is not None and dump.poll() is None:
            dump.kill()
        log.close()

    # ---------- analysis, after release ----------
    off = time.time() - time.monotonic()
    rows_pkt = ec.parse(cap)
    lo, hi = t_start + 0.5, t_start + a.seconds - 0.5

    # A) packets: j_t per tick, trimmed like the others
    pk = [(ts, s["j_t"]) for ts, s in rows_pkt if isinstance(s.get("j_t"), list) and lo <= ts <= hi]
    A = {j: hf_rms([r[1][j] for r in pk]) for j in JOINTS}

    # B) shm: EXACTLY 40_jitter's rows -> hf_rms over its 2 ms samples
    sh = [r for r in rows_shm if lo <= r[0] <= hi]
    B = {j: hf_rms([r[2][j] for r in sh]) for j in JOINTS}

    # C) bus: Iq current (mA), DATA_PACK replies, one per motor per tick
    bus = parse_candump(log.name, off, lo, hi)
    C = {j: hf_rms(bus[j]) for j in JOINTS}
    try:
        os.unlink(log.name)
    except OSError:
        pass
    series = dict(A={j: [r[1][j] for r in pk] for j in JOINTS},
                  B={j: [r[2][j] for r in sh] for j in JOINTS},
                  C={j: list(bus[j]) for j in JOINTS})
    return dict(A=A, B=B, C=C, series=series, n_pkt=len(pk), n_shm=len(sh),
                n_bus=len(bus[JOINTS[0]]), trk=None, chz=chz)


# Windows to sweep, as (label, half-window at 250 Hz, half-window at the shm sampler's ~500 Hz).
# The metric is a moving-average window counted in SAMPLES, so it spans half the TIME at 500 Hz —
# 40_jitter's 13 samples are 54 ms, e3's are 108 ms. Sweeping matched TIMES separates "the
# sampler is wrong" from "the two metrics are different bands". Offline, on real advancing torque,
# w=6 at 250 Hz alone took J6 from 0.0763 to 0.0909 (40_jitter measured 0.0927-0.0968).
WINDOWS = ((3, 6, "28 ms"), (6, 13, "52 ms"), (13, 26, "108 ms"), (26, 52, "212 ms"))


def ratios_by_window(base_blocks, adv_blocks):
    """{ 'A'|'B'|'C': [(label, ratio advancing/ruckig) ...] } — worst joint, time-matched windows."""
    out = {}
    for m in ("A", "B", "C"):
        rows = []
        for w250, w500, label in WINDOWS:
            w = w500 if m == "B" else w250

            def avg(blocks):
                v = [max(hf_rms(b["series"][m][j], w) for j in JOINTS) for b in blocks]
                return sum(v) / len(v)
            rows.append((label, avg(adv_blocks) / avg(base_blocks)))
        out[m] = rows
    return out


def worst(d):
    v = [x for x in d.values() if x == x]
    return max(v) if v else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true", help="required: this MOVES the arm")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--order", default="ruckig,advancing,ruckig,advancing")
    ap.add_argument("--rate", type=float, default=50.0)
    ap.add_argument("--amp", type=float, default=15.0)
    ap.add_argument("--period", type=float, default=4.0)
    ap.add_argument("--seconds", type=float, default=16.0)
    a = ap.parse_args()
    if not a.go:
        print(__doc__)
        print("Refusing to move the arm without --go.")
        return 2
    if a.host not in ("127.0.0.1", "localhost"):
        print("This needs /dev/shm and the CAN bus: run it ON the robot with --host 127.0.0.1.")
        return 2
    for tool in ("candump",):
        if subprocess.call(["which", tool], stdout=subprocess.DEVNULL) != 0:
            print(f"{tool} not found (apt install can-utils)")
            return 2
    from api_tests.external import _ext_common as ec
    from robotclient import RobotClient, RTIClient, ControlLost

    names = [n.strip() for n in a.order.split(",") if n.strip()]
    for n in names:
        if n not in ec.CONFIGS:
            print(f"unknown config {n!r}; known: {list(ec.CONFIGS)}")
            return 2

    print("=" * 96)
    print("44: TORQUE — THREE METHODS ON THE SAME RUN.  MOVES THE ARM (J4/J5/J6).")
    print("=" * 96)
    print(f"  {a.order}   {a.rate:g} Hz   +-{a.amp:g} deg / {a.period:g} s triangle   {a.seconds:g} s per block")
    print(f"  packets = e3's method | shm = 40_jitter's | bus = CAN Iq current via candump (the referee)\n")

    robot = RobotClient()
    robot.connect(a.host)
    sampler = Sampler()
    sampler.start()
    results = []
    try:
        why = ec.preflight(robot)
        if why:
            print(f"ABORT: {why}")
            return 1
        try:
            robot.reload_recipes()
            time.sleep(0.3)
        except Exception:
            pass
        robot.set_recipe(ec.RECIPE)
        for i, name in enumerate(names):
            print(f"  block {i + 1}/{len(names)}  {name:10s} ... ", end="", flush=True)
            r = run_block(name, ec.CONFIGS[name], a, ec, robot, RTIClient, ControlLost, sampler)
            if r is None:
                results.append((name, None))
                continue
            results.append((name, r))
            print(f"packets {worst(r['A']):.4f} Nm | shm {worst(r['B']):.4f} Nm | "
                  f"bus {worst(r['C']):7.2f} mA   (n: pkt {r['n_pkt']}, shm {r['n_shm']}, bus {r['n_bus']})")
        ec.clear_config(robot)
        robot.enter_idle_mode()
    finally:
        sampler.stop()
        try:
            ec.clear_config(robot)
            robot.enter_idle_mode()
            robot.disconnect()
        except Exception:
            pass
    if sampler.error:
        print(f"\n  sampler error (first): {sampler.error}")

    good = [(n, r) for n, r in results if r]
    base = [r for n, r in good if n == "ruckig"]
    adv = [r for n, r in good if n == "advancing"]
    if not base or not adv:
        print("\nNot enough completed blocks to compare (need a ruckig and an advancing block).")
        return 1

    def mean(rs, m):
        return sum(worst(r[m]) for r in rs) / len(rs)

    print("\n" + "=" * 96)
    print("RESULT — worst joint, mean over the ABAB repeats")
    print("=" * 96)
    print(f"  {'method':10s} {'ruckig':>12s} {'advancing':>12s} {'adv/ruckig':>12s}   unit")
    ratios = {}
    for m, label, unit in (("A", "packets", "Nm"), ("B", "shm", "Nm"), ("C", "bus", "mA")):
        rb, ra = mean(base, m), mean(adv, m)
        ratios[m] = ra / rb
        print(f"  {label:10s} {rb:12.4f} {ra:12.4f} {ratios[m]:11.3f}x   {unit}")

    def spread(rs, m):
        v = [worst(r[m]) for r in rs]
        return (max(v) - min(v)) / (sum(v) / len(v)) * 100 if len(v) > 1 else 0.0
    print(f"\n  repeat-to-repeat spread (drift check): ruckig {spread(base, 'C'):.1f}% / "
          f"advancing {spread(adv, 'C'):.1f}%  (bus)")

    print("\n  advancing / ruckig at MATCHED filter windows (worst joint) — is the answer a property of")
    print("  the arm, or of which band the metric happens to look at?")
    rw = ratios_by_window(base, adv)
    print(f"  {'window':>8}  {'packets':>9}  {'shm':>9}  {'bus':>9}")
    for k, (label, _) in enumerate(rw["A"]):
        print(f"  {label:>8}  {rw['A'][k][1]:8.3f}x  {rw['B'][k][1]:8.3f}x  {rw['C'][k][1]:8.3f}x")

    print("\n" + "-" * 96)
    ref = ratios["C"]
    agree = [lab for m, lab in (("A", "packets"), ("B", "shm")) if abs(ratios[m] - ref) <= 0.05]
    print(f"  The bus says advancing is {ref:.2f}x ruckig ({(ref - 1) * 100:+.0f}%).")
    if len(agree) == 2:
        print("  BOTH other methods agree — the earlier disagreement did not reproduce here.")
    elif agree:
        print(f"  Only '{agree[0]}' agrees with the bus; the other is measuring something else "
              f"({'shm' if agree[0] == 'packets' else 'packets'}).")
    else:
        print("  NEITHER method agrees with the bus — look at the raw numbers before trusting any.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
