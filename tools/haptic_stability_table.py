#!/usr/bin/env python3
"""Generate the haptic-wall stability table (config/xml_parser.HAPTIC_STABLE): for each velocity filter (ms) and
stiffness row (N/m at the 0.1 kg reference mass), the integer damping window (N.s/m, at least MIN_WIDTH wide) in which a mass of 0.1-5 kg
bouncing on the wall never gains energy, with the state 1 tick old as in RTI.

The force law is the one in utility/haptic_walls.py (two-sided damping on the wall's own EMA of the raw speed,
clamped so it never pulls); dummy_test_scripts/test_haptic_walls.py has the same model ("the fast table-checker and
the real class agree") and re-checks every cell of the table this prints. Pure Python, one core; run it while the
arm is idle (memory traffic on the Pi slows RTI).

Run: .venv/bin/python tools/haptic_stability_table.py [--taus 0 1 2 ...] [--rows 100 200 ...]
"""
import argparse
import math

DT = 0.004
LIGHT = (0.1, 0.13, 0.2, 0.5, 1.0, 5.0)          # kg; 0.13 = the model's TCP mass along J5's travel
ROWS = (100, 200, 300, 400, 500, 600, 750, 1000)
D_MAX = 30
MIN_WIDTH = 2      # N.s/m: a narrower window has no margin (1000 N/m at 0 ms is passive at exactly 11 and nothing else)


def light_bounce(k, d, m, tau_s, delay=1, v0=0.3, sub=20):
    """Speed out / speed in for a mass hitting the wall (> 1 = the wall added energy). Same as test_haptic_walls."""
    a = 1.0 if tau_s <= 0 else 1 - math.exp(-DT / tau_s)
    x, v, vf = -0.02, v0, v0
    hist, entered = [(x, v)] * (delay + 2), False
    for _ in range(4000):
        xs, vs = hist[-1 - delay]
        vf += a * (vs - vf)
        F = -max(k * xs + d * vf, 0.0) if xs > 0 else 0.0
        for _ in range(sub):
            v += F / m * (DT / sub)
            x += v * (DT / sub)
        hist.append((x, v))
        hist = hist[-(delay + 3):]
        entered = entered or x > 0
        if entered and x < 0 and v < 0:
            return abs(v) / v0
    return 0.0


def window(k, tau_ms):
    """The integer damping window where every mass is passive, or None. Refuses a window with a hole in it."""
    ok = [d for d in range(0, D_MAX + 1) if max(light_bounce(k, d, m, tau_ms / 1000.0) for m in LIGHT) <= 1.0]
    if not ok:
        return None
    if ok != list(range(ok[0], ok[-1] + 1)):
        raise RuntimeError(f"k={k} tau={tau_ms} ms: the passive damping values are not one range: {ok}")
    if ok[-1] == D_MAX:
        raise RuntimeError(f"k={k} tau={tau_ms} ms: passive up to the sweep's end ({D_MAX}) — widen D_MAX")
    return (ok[0], ok[-1]) if ok[-1] - ok[0] >= MIN_WIDTH else None


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--taus", type=int, nargs="+", default=list(range(0, 9)))
    ap.add_argument("--rows", type=int, nargs="+", default=list(ROWS))
    a = ap.parse_args()
    print("HAPTIC_STABLE = {")
    for tau in a.taus:
        cells = {}
        for k in a.rows:
            w = window(k, tau)
            if w is None:
                break                      # stiffer rows only shrink the window
            cells[k] = w
        print(f"    {tau}: {{" + ", ".join(f"{k}: ({lo}, {hi})" for k, (lo, hi) in cells.items()) + "},", flush=True)
    print("}")
