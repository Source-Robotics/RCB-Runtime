#!/usr/bin/env python3
"""The camera's own timing, checked with an LED (2026-11-24, TODO/camera_plan.md P3, method A — the cross-check of
api_tests/80_camera_timing.py). Nothing moves.

One of the box's isolated outputs drives an LED (the user wired one to each output) that the wrist camera sees. RTI
notes the moment it switches an output (rti['output_switch'], hardware/gpio_handler._note_switch: CLOCK_MONOTONIC right
after the pin's write — the clock of the camera's frame times). With a SHORT fixed exposure the LED shows clearly even
in a dark room, and the camera's rolling shutter turns a switch into an edge: rows read out before it show the old
level, rows read out after it the new one. That edge row, against the switch's time, gives

  * the moment each row is read out (its exposure ends) relative to the frame's camera time:
        t_switch - frame time = a + (edge row - centre row) x row time - exposure / 2
    fitted over ~100 switches at random moments: a (the centre row's readout) and the row time (the readout over the
    whole frame, which api_tests/80 also measured: 31.8 ms on the MSG camera) — the camera alone, no joint data;
  * with api_tests/80's result for automatic exposure (the image's moment as the JOINT DATA sees it) and automatic
    exposure's length: how late the joint data RTI stamps with a tick is. That length is ASSUMED to be the whole
    frame; the camera does not report it, and at 30 ms (its likely ceiling at 30 fps with 50 Hz anti-flicker) the
    result moves by 1.7 ms (review 2026-11-25: to be measured with this check under automatic exposure).

    .venv/bin/python api_tests/81_camera_led_timing.py              # dry run: the checks
    .venv/bin/python api_tests/81_camera_led_timing.py --go         # find the LED, then ~120 switches (~40 s)
    .venv/bin/python api_tests/81_camera_led_timing.py --go --output 2
    .venv/bin/python api_tests/81_camera_led_timing.py --reanalyse temp_data/camera_timing/led_RUN
    .venv/bin/python api_tests/81_camera_led_timing.py --selftest   # synthetic frames, no robot
    .venv/bin/python api_tests/81_camera_led_timing.py --go --auto  # automatic exposure: its length and moment (E1)
    .venv/bin/python api_tests/81_camera_led_timing.py --apply temp_data/camera_timing/led_RUN   # its moment: the calibration

--apply (2026-11-25, true timing — TODO/camera_review_plan.md option A, the user's choice): an --auto run's image moment
becomes the calibration under automatic exposure (timing_auto_us in every gripper XML declaring the run's camera; the
camera server takes it within ~1 s). It reads the run's result.json, made by --reanalyse here or, kinder to RTI, by
the PC kit's run_analysis.py (copy result.json back into the run's folder). The joint data are paired on their own
measurement times (RTI's trace column j_rx, utility/joint_times.py), so the picture's moment is the whole camera side.

--auto (2026-11-25, TODO/camera_review_plan.md E1, Q1): the LED is found as above, then the camera goes back to
AUTOMATIC exposure (the user's controls, as for recording) and the LED switches ~300 times. Around a switch each row
of the LED's spot shows a SHARE of the light: the share of that row's exposure after the switch (on) or before it
(off). Over all switches that share against the switch's moment is a ramp: its width is the exposure's length E, its
end the row's readout. Fitted: a (the centre row's readout after the frame's camera time) and E; the image's moment of
the centre row = a - E / 2. Each pixel's grey level is made linear in light first, (grey / 255) ^ p, with the camera's
power p fitted too (the ramp is straight only in light, not in grey levels: with a power on whole rows instead, the
self-test's 30 ms came out 29.3). Only pixels of the spot that are not saturated when on are used, each scaled by its
own off and on levels (under 30 ms + gain the LED's core saturates; its soft edge does not). The room's light as when
recording: this measures the exposure the camera picks in THAT light.

Setup: aim the camera at the LED of the chosen output, 5-15 cm away — a large, out-of-focus spot covers more rows, so
more switches land on it. The output switches on and off ~120 times; it is left OFF. The camera's settings are put
back after. Results: temp_data/camera_timing/led_<UTC>/ (run.json, frames.npz, switches, report.txt).
"""
from __future__ import annotations

import glob
import io
import json
import os
import random
import sys
import threading
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)

from utility import camera_settings as cs  # noqa: E402
from utility.rti_pace import Pace  # noqa: E402  (the picture work paced on RTI's own trace)

OUTPUT = 1
EVENTS = 120
GAP_S = (0.15, 0.30)               # between switches: every switch on its own frames (30 fps = 33 ms)
EXPOSURES_TRY = (2, 5, 10, 20, 50)  # fixed settings tried, shortest first (100 us each: 0.2 .. 5 ms)
MIN_LED_CONTRAST = 25.0            # grey levels between the LED on and off, at its brightest (half-size) pixel
MIN_LED_ROWS = 8                   # rows the LED's spot must cover (full resolution)
MIN_EDGES = 10                     # switches that must show an edge on the spot for the line (else: the bounds);
                                   # 19 edges gave the readout to 0.045 ms scatter (2026-11-24)
OLD_BELOW, NEW_ABOVE = 0.15, 0.85  # the spot's level between off (0) and on (1): old / new / in between
LOCATE_LEVELS = (1, 0, 1, 0, 1, 0)  # finding the LED: from off, every step a CHANGE (RTI notes only a change), ends off
AUTO_EVENTS = 300                  # --auto: switches (~70 s)
AUTO_MARGIN = 60                   # --auto: px around the spot found at the short exposure (its soft edge grows)
AUTO_SETTLE_S = 3.0                # --auto: automatic exposure settles after the switch back
AUTO_MIN_CONTRAST = 20.0           # a pixel used: on minus off at least this (grey levels) ...
AUTO_MAX_ON = 240.0                # ... and never this bright when on (saturated: its share is lost)
AUTO_MAX_HALVES_MS = 0.2          # --apply: the run's two halves must agree this well
AUTO_MIN_PIXELS = 3                # a row of the spot used with this many such pixels
AUTO_MIN_SWITCHES = 40             # switches whose frames lie on the ramp
SYSTEM_XML = os.path.join(_ROOT, "config", "system.xml")
GRIPPERS_DIR = os.path.join(_ROOT, "grippers")


# ------------------------------------------------------------------------------------------------------- analysis
def gray(jpeg: bytes, half: bool = False) -> np.ndarray:
    """The JPEG in grey; half: decoded at half size by the decoder itself (finding the LED)."""
    from PIL import Image
    im = Image.open(io.BytesIO(jpeg))
    if half:
        im.draft("L", (im.size[0] // 2, im.size[1] // 2))
    return np.asarray(im.convert("L"), dtype=np.float32)


def spot_crop(jpeg: bytes, spot: dict) -> np.ndarray:
    """The LED spot's pixels of a frame, full resolution: the whole picture is decoded but only the crop kept (the
    first run kept ~70 full pictures as floats while the arm was ENABLED: RTI's loop slowed — LOOP_CRITICAL,
    2026-11-24 21:04)."""
    from PIL import Image
    (y0, y1), (x0, x1) = spot["rows"], spot["cols"]
    full = np.asarray(Image.open(io.BytesIO(jpeg)).convert("L"))
    return full[y0:y1, x0:x1].astype(np.float32)


def find_led(diff: np.ndarray, scale: int = 1) -> dict | None:
    """The LED's spot from the mean picture with it on minus the mean with it off (at 1 / scale of full size):
    {"rows": (y0, y1), "cols": (x0, x1) at full size, "contrast"} or None when no spot stands out."""
    peak = float(diff.max())
    if peak < MIN_LED_CONTRAST:
        return None
    ys, xs = np.nonzero(diff > 0.5 * peak)
    y0, y1 = int(np.percentile(ys, 2)) * scale, (int(np.percentile(ys, 98)) + 1) * scale
    x0, x1 = int(np.percentile(xs, 2)) * scale, (int(np.percentile(xs, 98)) + 1) * scale
    if y1 - y0 < MIN_LED_ROWS:
        return None
    return {"rows": (y0, y1), "cols": (x0, x1), "contrast": peak}




def edge_row(profile: np.ndarray, on_ref: np.ndarray, off_ref: np.ndarray, rising: bool) -> float | None:
    """The row (within the spot) where the level crosses half-way between off and on in the switch's direction, to a
    fraction of a row; None when the frame shows no edge on the spot (all old, all new, or more than one crossing)."""
    span = on_ref - off_ref
    ok = np.abs(span) > 10.0
    if ok.sum() < 4:
        return None
    n = np.full(len(profile), np.nan)
    n[ok] = (profile[ok] - off_ref[ok]) / span[ok]
    rows = np.nonzero(ok)[0]
    level = n[rows] if rising else 1.0 - n[rows]          # 0 = the old level, 1 = the new one, later rows newer
    above = level >= 0.5
    if above.all() or not above.any():
        return None
    first = int(np.argmax(above))
    if not above[first:].all() or first == 0:             # one clean crossing, old above it, new below
        return None
    r0, r1 = rows[first - 1], rows[first]
    v0, v1 = level[first - 1], level[first]
    return float(r0 + (0.5 - v0) / (v1 - v0) * (r1 - r0)) if v1 != v0 else float(r1)


def spot_crop_half(jpeg: bytes, spot: dict) -> np.ndarray:
    """The spot's pixels decoded at HALF size by the JPEG decoder (a quarter of the work); row r of the crop is the
    full-size rows spot_y0 + 2r and + 2r + 1."""
    (y0, y1), (x0, x1) = spot["rows"], spot["cols"]
    g = gray(jpeg, half=True)
    return g[y0 // 2:(y1 + 1) // 2, x0 // 2:(x1 + 1) // 2]


AUTO_POWERS = np.arange(0.8, 3.01, 0.1)    # the camera's tone curve, grey -> light: (grey / 255) ^ p, p tried


def _auto_model(p, delta, light_on):
    """The share of a spot row's exposure that was lit (0..1). delta = switch time - the frame's camera time - (row -
    centre) x row time; p = (a, E) in s."""
    a, e = p
    s = np.clip((a - delta) / e, 0.0, 1.0)                # the share of the exposure after the switch
    return np.where(light_on, s, 1.0 - s)


def _auto_fit(delta, light_on, f):
    from scipy.optimize import least_squares
    best = None
    for a0 in (0.010, 0.0167, 0.023):
        for e0 in (0.008, 0.018, 0.030):
            r = least_squares(lambda p: _auto_model(p, delta, light_on) - f, (a0, e0),
                              bounds=((0.0, 0.0005), (0.05, 0.04)), loss="soft_l1", f_scale=0.1)
            if best is None or r.cost < best.cost:
                best = r
    return best


def _row_shares(crop_, on_ref, off_ref, use, rows_ok, p):
    """Each usable row's lit share in one frame: every used pixel made linear in light, scaled by its own off / on."""
    lin_off, lin_on = (off_ref / 255.0) ** p, (on_ref / 255.0) ** p
    span = np.where(use, lin_on - lin_off, 1.0)
    n = np.where(use, ((np.clip(crop_, 0, 255) / 255.0) ** p - lin_off) / span, np.nan)
    return np.nanmean(n[rows_ok], axis=1)


def analyse_auto(frames: dict, switches: list, spot: dict, full_rows: int, row_time_s: float,
                 calib_offset_s: float = 0.0, pace=None) -> dict:
    """E1: the exposure's length and the centre row's moment under AUTOMATIC exposure (the module's header). frames:
    exposure_time, flags, jpegs; switches: [(time, level)]; spot: the crop (full-size rows / cols); calib_offset_s: a
    calibration active while recording (taken back off flagged frames); pace: called with each decode's seconds."""
    from communication import camera_protocol as cp
    flags = np.asarray(frames["flags"])
    times = np.asarray(frames["exposure_time"], float) - np.where((flags & cp.FLAG_CALIBRATED) > 0, calib_offset_s, 0.0)
    ok = (flags & cp.FLAG_CAMERA_CLOCK) > 0
    sw = sorted(switches)
    steady = {1: [], 0: []}
    late = {1: [], 0: []}                                 # steady frames >= 150 ms after the switch (the drift check)
    near = []
    for i, (ts, lv) in enumerate(sw):
        nxt = sw[i + 1][0] if i + 1 < len(sw) else ts + 1.0
        for k in np.nonzero(ok & (times > ts + 0.07) & (times < nxt - 0.05))[0][:3]:
            steady[lv].append(int(k))
            if times[k] > ts + 0.15:
                late[lv].append(int(k))
        for k in np.nonzero(ok & (times > ts - 0.09) & (times < ts + 0.06))[0]:
            near.append((int(k), ts, lv))
    if len(steady[1]) < 10 or len(steady[0]) < 10:
        return {"ok": False, "why": "too few steady frames with the LED on and off"}
    crops = {}

    def crop(k):
        if k not in crops:
            t0 = time.perf_counter()
            crops[k] = spot_crop_half(frames["jpegs"][k], spot)
            if pace is not None:
                pace(time.perf_counter() - t0)
        return crops[k]

    on_stack = np.array([crop(k) for k in steady[1]])
    off_stack = np.array([crop(k) for k in steady[0]])
    on_ref, off_ref = np.median(on_stack, axis=0), np.median(off_stack, axis=0)
    use = ((on_ref - off_ref) >= AUTO_MIN_CONTRAST) & (on_stack.max(axis=0) < AUTO_MAX_ON)
    rows_ok = use.sum(axis=1) >= AUTO_MIN_PIXELS
    n_pix = int(use[rows_ok].sum())
    if rows_ok.sum() < 2:
        return {"ok": False, "why": f"no part of the LED's spot is bright enough and unsaturated when on ({n_pix} px): "
                                    f"move the LED a little further away or out of focus"}
    y_full = spot["rows"][0] // 2 * 2 + 2.0 * np.arange(on_ref.shape[0]) + 0.5       # each crop row's full-size row
    deltas, lights, swid, near_crops = [], [], [], []
    for k, ts, lv in near:
        d = ts - times[k] - (y_full[rows_ok] - full_rows / 2.0) * row_time_s
        near_crops.append(crop(k))
        deltas.append(d)
        lights.append(np.full(len(d), bool(lv)))
        swid.append(np.full(len(d), ts))
    delta, light_on, sid = (np.concatenate(x) for x in (deltas, lights, swid))

    def shares(p):
        return np.concatenate([_row_shares(c, on_ref, off_ref, use, rows_ok, p) for c in near_crops])

    f = shares(1.0)
    ramp = (f > 0.05) & (f < 0.95)
    n_sw = len(np.unique(sid[ramp]))
    if n_sw < AUTO_MIN_SWITCHES:
        return {"ok": False, "why": f"only {n_sw} switches fell on the ramp (needs {AUTO_MIN_SWITCHES}): more switches "
                                    f"(--events) or a larger spot"}
    tried = []
    for p in AUTO_POWERS:                                 # the camera's power: the one whose ramp is straightest
        fp = shares(p)
        tried.append((_auto_fit(delta, light_on, fp).cost, float(p), fp))
    _cost, g, f = min(tried, key=lambda x: x[0])
    fit = _auto_fit(delta, light_on, f)
    a, e = fit.x
    resid = _auto_model(fit.x, delta, light_on) - f
    halves = []
    order = np.argsort(np.unique(sid))
    cut = np.unique(sid)[order[len(order) // 2]]
    for sel in (sid < cut, sid >= cut):
        h = _auto_fit(delta[sel], light_on[sel], f[sel]).x
        halves.append((h[0] - h[1] / 2) * 1e3)
    drift = {}
    for lv, name in ((1, "on"), (0, "off")):
        ks = late[lv]
        if len(ks) >= 20:
            third = len(ks) // 3
            lvl = [float(np.mean(crop(k)[use])) for k in ks]
            drift[name] = float(np.mean(lvl[-third:]) - np.mean(lvl[:third]))
    early_on = [float(np.mean(crop(k)[use])) for k in steady[1] if k not in late[1]]
    late_on = [float(np.mean(crop(k)[use])) for k in late[1]]
    return {"ok": True, "method": "auto", "a_ms": a * 1e3, "exposure_ms": e * 1e3, "power": g,
            "moment_ms": (a - e / 2) * 1e3, "halves_ms": halves, "half_diff_ms": abs(halves[0] - halves[1]),
            "rms": float(np.sqrt(np.mean(resid ** 2))), "points": int(len(f)), "ramp_points": int(ramp.sum()),
            "switches_on_ramp": n_sw, "pixels": n_pix, "rows": int(rows_ok.sum()),
            "level_drift": drift,
            "on_early_minus_late": (float(np.mean(early_on) - np.mean(late_on)) if early_on and late_on else None),
            "row_time_us": row_time_s * 1e6}


def report_auto(res: dict, timing: dict | None) -> list:
    if not res.get("ok"):
        return [f"  no result: {res.get('why')}"]
    lines = [f"  automatic exposure: the exposure lasts {res['exposure_ms']:.2f} ms; the centre row is read out "
             f"{res['a_ms']:+.2f} ms after the frame's camera time; the middle of its exposure (the image's moment) "
             f"{res['moment_ms']:+.2f} ms after it (the two halves of the run: "
             f"{res['halves_ms'][0]:+.2f} / {res['halves_ms'][1]:+.2f} ms)",
             f"  fit: {res['switches_on_ramp']} switches on the ramp, {res['ramp_points']} of {res['points']} row "
             f"readings on it, {res['pixels']} unsaturated pixels in {res['rows']} rows, rms {res['rms']:.3f} of the "
             f"on-off step, the camera's tone curve: light = (grey / 255) ^ {res['power']:.1f}"]
    d = res.get("level_drift") or {}
    if d or res.get("on_early_minus_late") is not None:
        lines.append(f"  steadiness: the steady levels drifted {d.get('on', float('nan')):+.1f} (on) / "
                     f"{d.get('off', float('nan')):+.1f} (off) grey levels from the first to the last third; on-frames "
                     f"70-150 ms after a switch minus later ones {res.get('on_early_minus_late') or 0.0:+.1f} (the "
                     f"automatic exposure answering the LED itself, when large)")
    in_use = (f"the calibration in use: {timing['auto_us'] / 1e3:+.2f} ms (run {timing.get('run')})"
              if timing and timing.get("auto_us") is not None else "no calibration in use for this mode")
    lines.append(f"  {in_use}; this run's image moment {res['moment_ms']:+.2f} ms: "
                 + ("--apply FOLDER makes it the calibration (true timing: each joint is paired with the picture on its "
                    "own measurement time, RTI's j_rx)" if res["half_diff_ms"] <= AUTO_MAX_HALVES_MS else
                    f"its halves differ by more than {AUTO_MAX_HALVES_MS:g} ms, so --apply refuses it"))
    return lines


def apply_result(folder: str, grippers_dir: str = GRIPPERS_DIR, system_xml: str = SYSTEM_XML) -> int:
    """--apply: an --auto run's image moment (its result.json) written as the calibration under automatic exposure
    (timing_auto_us) into every gripper XML declaring the run's camera, for the run's mode; 0 when written."""
    try:
        with open(os.path.join(folder, "run.json"), encoding="utf-8") as f:
            run_ = json.load(f)
        with open(os.path.join(folder, "result.json"), encoding="utf-8") as f:
            res = json.load(f)
    except (OSError, ValueError) as e:
        print(f"--apply: {folder} has no readable run.json and result.json ({e}); analyse it first (--reanalyse, or "
              f"the PC kit's run_analysis.py, then copy result.json into the folder)")
        return 1
    if not run_.get("auto"):
        print("--apply: only an --auto run's image moment is a calibration (a fixed exposure is black indoors)")
        return 1
    if not res.get("ok"):
        print(f"--apply: the run has no result ({res.get('why')})")
        return 1
    if res.get("half_diff_ms", float("inf")) > AUTO_MAX_HALVES_MS:
        print(f"--apply: the run's halves differ by {res.get('half_diff_ms')} ms (more than {AUTO_MAX_HALVES_MS:g}): "
              f"not written")
        return 1
    usb_id, mode = run_.get("usb_id"), run_.get("mode")
    if not (usb_id and mode):
        # recorded before 2026-11-25 (no camera or mode in run.json): its stored timing was the gripper camera's for
        # the run's mode, so that camera and its mode
        gcam, _ = cs.gripper_camera(system_xml, grippers_dir)
        if not (run_.get("timing") and gcam and gcam.get("usb_id") and gcam.get("timing_mode")):
            print("--apply: the run names no camera or mode, and the gripper's camera had no timing for it")
            return 1
        usb_id, mode = gcam["usb_id"], gcam["timing_mode"]
    value = f"{res['moment_ms'] * 1e3:.1f}"
    written = [os.path.basename(p) for p in sorted(glob.glob(os.path.join(grippers_dir, "*.xml")))
               if cs.write_timing(p, usb_id, mode, {"timing_auto_us": value}, os.path.basename(folder.rstrip("/")),
                                  os.path.join(folder, "backup"))]
    print(f"--apply: the image's moment {res['moment_ms']:+.2f} ms (timing_auto_us=\"{value}\", camera {usb_id}, "
          f"mode {mode}) written into {len(written)} gripper XML(s): {', '.join(written) or 'none'}; the camera server "
          f"takes it within ~1 s (backups in {os.path.join(folder, 'backup')})")
    return 0 if written else 1


def switch_bounds(sw, times, ok, level, spot, exposure_s, full_rows, row_time_s) -> dict | None:
    """Every switch bounds a, the centre row's readout after the frame's camera time (row y is read out at frame time
    + a + (y - centre) x row time, its exposure the E before): a frame whose spot still shows the OLD level read all
    the spot's rows out before the switch (a <= switch - frame time - (last row - centre) x row time); one showing the
    NEW level began all of them after it (a >= switch - frame time - (first row - centre) x row time + E); one in
    between did both partly. Each switch leaves an interval ~a frame wide, narrower when the spot was in between; the
    value most intervals share is a, its range shrinking with the number of switches (~a frame / their number). Works
    for a spot of a few rows, where an edge on it is rare. None without enough switches."""
    c = full_rows / 2.0
    y0, y1 = spot["rows"][0], spot["rows"][1] - 1
    ivals = []
    for ts, lv in sw:
        lo, hi = -np.inf, np.inf
        for k in np.nonzero(ok & (times > ts - 0.15) & (times < ts + 0.15))[0]:
            n = level(int(k))
            new = n if lv else 1.0 - n
            d = ts - times[k]
            # conservative: "old" = at most OLD_BELOW of the spot's earliest row's exposure after the switch, "new" = at
            # least NEW_ABOVE of its latest row's (2026-11-24: taking them as all-old / all-new put the selftest's truth
            # just outside the range)
            if new <= OLD_BELOW:
                hi = min(hi, d - (y0 - c) * row_time_s + OLD_BELOW * exposure_s)
            elif new >= NEW_ABOVE:
                lo = max(lo, d - (y1 - c) * row_time_s + NEW_ABOVE * exposure_s)
            else:
                lo = max(lo, d - (y1 - c) * row_time_s)
                hi = min(hi, d - (y0 - c) * row_time_s + exposure_s)
        if np.isfinite(lo) and np.isfinite(hi):
            ivals.append((lo, hi))
    if len(ivals) < 10:
        return None
    pts = sorted([(lo, 1) for lo, _ in ivals] + [(hi, -1) for _, hi in ivals], key=lambda p: (p[0], -p[1]))
    best, cur, at = -1, 0, 0
    for i, (x, d) in enumerate(pts):
        cur += d
        if cur > best:
            best, at = cur, i
    lo_b, hi_b = pts[at][0], pts[at + 1][0] if at + 1 < len(pts) else pts[at][0]
    return {"a_ms": (lo_b + hi_b) / 2 * 1e3, "a_range_ms": (hi_b - lo_b) * 1e3, "switches_used": len(ivals),
            "agree": best, "row_time_us": row_time_s * 1e6, "readout_ms": row_time_s * full_rows * 1e3,
            "narrow": int(sum(1 for lo, hi in ivals if hi - lo < 0.5 * (1.0 / 30)))}


def analyse(frames: dict, switches: list, spot: dict, exposure_s: float, full_rows: int,
            row_time_s: float = 31.8e-3 / 720, pace=None) -> dict:
    """The edges -> t_switch - frame time = a' + (edge row - centre) x row time; a = a' + exposure / 2 (the centre
    row's readout after the frame's camera time). Fewer than MIN_EDGES edges (a small spot): a from the bounds every
    switch sets (switch_bounds), with the row time given (api_tests/80's readout). frames: exposure_time, flags,
    jpegs (decoded here, the spot's crop only); switches: [(time, level)]."""
    from communication import camera_protocol as cp
    times = frames["exposure_time"]
    ok = (frames["flags"] & cp.FLAG_CAMERA_CLOCK) > 0
    calibrated = int(np.sum((frames["flags"] & cp.FLAG_CALIBRATED) > 0))
    if calibrated:
        return {"ok": False, "why": f"{calibrated} frames carry a calibrated time (a fixed-exposure calibration is "
                                    f"active): this check needs the camera's own time"}
    profiles = {}

    def prof(k):
        if k not in profiles:
            t0 = time.perf_counter()
            profiles[k] = spot_crop(frames["jpegs"][k], spot).mean(axis=1)
            if pace is not None:
                pace(time.perf_counter() - t0)
        return profiles[k]

    y0 = spot["rows"][0]
    sw = sorted(switches)
    # steady references: frames >= 2 frames after a switch and >= 1 frame before the next
    on_p, off_p = [], []
    for i, (ts, level) in enumerate(sw):
        nxt = sw[i + 1][0] if i + 1 < len(sw) else ts + 1.0
        for k in np.nonzero(ok & (times > ts + 0.07) & (times < nxt - 0.05))[0][:2]:
            (on_p if level else off_p).append(prof(int(k)))
    if len(on_p) < 5 or len(off_p) < 5:
        return {"ok": False, "why": "too few steady frames with the LED on and off"}
    on_ref, off_ref = np.median(on_p, axis=0), np.median(off_p, axis=0)
    level_of = {}

    def level(k):
        """The spot's level in frame k: 0 = off, 1 = on (its brightest rows, the steady frames as the scale)."""
        if k not in level_of:
            w = np.abs(on_ref - off_ref)
            level_of[k] = float(((prof(k) - off_ref) * w).sum() / ((on_ref - off_ref) * w).sum())
        return level_of[k]

    bounds = switch_bounds(sw, times, ok, level, spot, exposure_s, full_rows, row_time_s)
    xs, ys, kinds = [], [], []
    for ts, level in sw:
        for k in np.nonzero(ok & (times > ts - 0.08) & (times < ts + 0.05))[0]:
            e = edge_row(prof(int(k)), on_ref, off_ref, bool(level))
            if e is not None:
                xs.append(y0 + e - full_rows / 2.0)
                ys.append(ts - times[k])
                kinds.append(level)
                break
    n = len(xs)
    if n < MIN_EDGES:
        if bounds is None:
            return {"ok": False, "why": f"only {n} switches showed an edge on the LED's spot, and the bounds found "
                                        f"no common value (rows {spot['rows'][0]}-{spot['rows'][1]} of {full_rows})",
                    "edges": n}
        return dict(bounds, ok=True, method="bounds", edges_found=n, exposure_ms=exposure_s * 1e3)
    x, y = np.array(xs), np.array(ys)
    k1, a1 = np.polyfit(x, y, 1)
    r = y - (a1 + k1 * x)
    mad = float(np.median(np.abs(r - np.median(r)))) * 1.4826
    keep = np.abs(r) <= max(0.3e-3, 4 * mad)
    k1, a1 = np.polyfit(x[keep], y[keep], 1)
    r = y - (a1 + k1 * x)
    kinds_a = np.array(kinds)
    per = {}
    for lv, name in ((1, "on"), (0, "off")):
        m = keep & (kinds_a == lv)
        per[name] = float(np.mean(y[m] - k1 * x[m]) * 1e3) if m.sum() >= 3 else None
    return {"ok": True, "method": "edges", "bounds": bounds, "edges": int(keep.sum()), "edges_found": n,
            "row_time_us": k1 * 1e6,
            "readout_ms": k1 * full_rows * 1e3, "a_ms": (a1 + exposure_s / 2) * 1e3, "scatter_ms": float(np.std(r[keep]) * 1e3),
            "on_minus_off_ms": (per["on"] - per["off"]) if per["on"] is not None and per["off"] is not None else None,
            "exposure_ms": exposure_s * 1e3}


def report(res: dict, timing: dict | None, fps: float) -> list:
    if not res.get("ok"):
        return [f"  no result: {res.get('why')}"]
    if res["method"] == "bounds":
        lines = [f"  the LED's spot showed an edge in only {res['edges_found']} switches (a small spot): a from the "
                 f"bounds of {res['switches_used']} switches ({res['agree']} agree, {res['narrow']} narrow), the "
                 f"readout taken from api_tests/80 ({res['readout_ms']:.2f} ms), exposure {res['exposure_ms']:g} ms",
                 f"  the centre row's exposure ENDS (it is read out) {res['a_ms']:+.2f} ms (+-{res['a_range_ms'] / 2:.2f}) "
                 f"after the frame's camera time; its middle at an exposure E: {res['a_ms']:+.2f} ms - E / 2"]
    else:
        lines = [f"  {res['edges']} switches with an edge on the LED's spot ({res['edges_found']} found), scatter about "
                 f"the line {res['scatter_ms']:.3f} ms, exposure {res['exposure_ms']:g} ms",
                 f"  rows read out over {res['readout_ms']:.2f} ms ({res['row_time_us']:.2f} us per row)"
                 + (f"; api_tests/80 measured {timing['readout_us'] / 1e3:.2f} ms" if timing and timing.get('readout_us')
                    else ""),
                 f"  the centre row's exposure ENDS (it is read out) {res['a_ms']:+.2f} ms after the frame's camera time; "
                 f"its middle at an exposure E: {res['a_ms']:+.2f} ms - E / 2"]
        if res.get("bounds"):
            lines.append(f"  (the switches' bounds alone: {res['bounds']['a_ms']:+.2f} +-{res['bounds']['a_range_ms'] / 2:.2f} ms)")
    if res.get("on_minus_off_ms") is not None:
        lines.append(f"  switching on vs off: {res['on_minus_off_ms']:+.3f} ms apart (the LED's and the output's own "
                     f"delays)")
    if timing and timing.get("auto_us") is not None:
        e_auto = 1e3 / fps
        cam = res["a_ms"] - e_auto / 2
        lines.append(f"  automatic exposure, IF it runs for the whole frame (~{e_auto:.1f} ms; its real length is not "
                     f"measured, and at 30 ms these numbers move by 1.7 ms): the centre row's middle is {cam:+.2f} ms "
                     f"after the camera time; api_tests/80 found the joint data matching it at "
                     f"{timing['auto_us'] / 1e3:+.2f} ms -> the joint data RTI stamps with a tick would describe the "
                     f"arm ~{timing['auto_us'] / 1e3 - cam:.1f} ms earlier than its time")
    return lines


# ------------------------------------------------------------------------------------------------------- the box
def camera_status() -> dict:
    from communication import camera_protocol as cp
    from data.shared_data import SharedString
    return SharedString(cp.STATUS_NAME, cp.STATUS_SIZE).get_json() or {}


def switch_segment_exists() -> bool:
    return os.path.exists("/dev/shm/" + os.environ.get("RCB_SHM_PREFIX", "") + "output_switch")


def checks() -> tuple[dict, list]:
    problems = []
    if not switch_segment_exists():
        problems.append("RTI does not note its output switches yet (no shared memory 'output_switch'): the robot "
                        "service must restart with the code from 2026-11-24 (the simulator first)")
    st = camera_status()
    if st.get("state") != "streaming":
        problems.append(f"the camera is not streaming ({st.get('state')}: {st.get('message')})")
    clk = st.get("clock") or {}
    if "calibration_why" not in clk:
        problems.append("the camera server is older than this script: restart it")
    elif clk.get("source") != "camera":
        problems.append(f"the frames do not carry the camera's own clock ({clk.get('why')})")
    return st, problems


def set_controls(base: dict, setting) -> None:
    if setting is None:
        cs.write(SYSTEM_XML, base)
        return
    pairs = [(k, v) for k, v in cs.parse_controls(base["controls"]) if k not in ("auto_exposure",
                                                                                 "exposure_time_absolute")]
    cs.write(SYSTEM_XML, {**base, "controls": cs.format_controls([("auto_exposure", 1),
                                                                   ("exposure_time_absolute", setting)] + pairs)})
    end = time.monotonic() + 8.0
    while time.monotonic() < end:
        ctl = {c["key"]: c["value"] for c in camera_status().get("controls") or []}
        if ctl.get("auto_exposure") == 1 and ctl.get("exposure_time_absolute") == setting:
            time.sleep(0.5)
            return
        time.sleep(0.2)
    raise RuntimeError(f"the camera did not take exposure setting {setting} within 8 s")


class Frames(threading.Thread):
    def __init__(self, host):
        super().__init__(daemon=True)
        from robotclient import CameraClient
        self.cam = CameraClient().connect(host)
        self.frames: list = []
        self.stop_flag = False

    def run(self):
        try:
            for f in self.cam.frames(timeout=3.0):
                self.frames.append(f)
                if self.stop_flag:
                    break
        except Exception:  # noqa: BLE001 — the frames so far are what counts
            pass

    def finish(self):
        self.stop_flag = True
        self.join(5.0)
        self.cam.close()
        return self.frames


def output_off(robot, output: int) -> None:
    """The output off, with no note awaited (it may be off already: RTI writes, and notes, only a change)."""
    robot.set_digital_output(output, 0)
    time.sleep(0.1)


def switch(robot, output: int, level: int, sw, timeout=0.5):
    """Switch the output through the robot to a level it does NOT have now; (its time, its level) as RTI noted them.
    (2026-11-24, the first run: asking for the level it already had waited for a note that never comes.)"""
    count = sw[0]
    robot.set_digital_output(output, level)
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if sw[0] != count and int(sw[1]) == output:
            return float(sw[3]), int(sw[2])
        time.sleep(0.001)
    raise RuntimeError("RTI did not note the switch (does it run the code from 2026-11-24?)")


def as_arrays(frames: list) -> dict:
    return {"exposure_time": np.array([f.exposure_time for f in frames]),
            "flags": np.array([int(f.camera_clock) | int(f.calibrated) << 1 for f in frames], dtype=np.int64),
            "jpegs": [f.jpeg for f in frames]}


def locate(robot, output, sw, host, base) -> tuple[int, dict] | None:
    """The shortest exposure that shows the LED: (its setting, the spot)."""
    for setting in EXPOSURES_TRY:
        set_controls(base, setting)
        output_off(robot, output)
        col = Frames(host)
        col.start()
        marks = []
        for level in LOCATE_LEVELS:
            marks.append(switch(robot, output, level, sw))
            time.sleep(0.4)
        frames = col.finish()
        sums = {1: None, 0: None}                   # running sums at half size: never a list of pictures
        counts = {1: 0, 0: 0}
        for f in frames:
            before = [m for m in marks if m[0] < f.exposure_time - 0.07]
            after = [m for m in marks if m[0] > f.exposure_time - 0.07]
            if before and (not after or after[0][0] > f.exposure_time + 0.05):
                lv = before[-1][1]
                g = gray(f.jpeg, half=True)
                sums[lv] = g if sums[lv] is None else sums[lv] + g
                counts[lv] += 1
        spot = (find_led(sums[1] / counts[1] - sums[0] / counts[0], scale=2)
                if counts[1] >= 3 and counts[0] >= 3 else None)
        if spot:
            print(f"exposure {setting / 10:g} ms: the LED's spot at rows {spot['rows'][0]}-{spot['rows'][1]}, columns "
                  f"{spot['cols'][0]}-{spot['cols'][1]}, contrast {spot['contrast']:.0f}", flush=True)
            return setting, spot
        print(f"exposure {setting / 10:g} ms: no LED seen", flush=True)
    return None


def arm_enabled(args) -> bool:
    """True (and said why) while the arm is enabled: this check moves nothing, and its picture work must not compete
    with RTI's loop while the arm holds (2026-11-24: the first run, the arm enabled, latched LOOP_CRITICAL)."""
    from _common import connect
    probe = connect(args)
    try:
        state = probe.get_status().get("state")
    finally:
        probe.disconnect()
    if state == "ENABLED":
        print("The arm is ENABLED: disable it first (the dashboard's Disable), then run this again.")
        return True
    return False


def run(args) -> str | None:
    from data import shared_data
    from project_paths import TEMP_DATA_DIR
    from _common import connect
    st, problems = checks()
    if problems:
        print("NOT READY:\n  " + "\n  ".join(problems))
        return None
    if arm_enabled(args):
        return None
    sw = shared_data.rti["output_switch"]
    base, _ = cs.read(SYSTEM_XML)
    events = AUTO_EVENTS if args.auto and args.events == EVENTS else args.events
    spot_core = None
    fps = float(st["mode"]["fps"])
    full_rows = int(st["mode"]["height"])
    gcam, _ = cs.gripper_camera(SYSTEM_XML, GRIPPERS_DIR)
    out = os.path.join(str(TEMP_DATA_DIR), "camera_timing", "led_" + time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
    os.makedirs(out, exist_ok=True)
    robot = connect(args)
    try:
        try:
            found = locate(robot, args.output, sw, args.host, base)
            if found is None:
                print(f"The camera does not see output {args.output}'s LED at any exposure up to "
                      f"{EXPOSURES_TRY[-1] / 10:g} ms: aim it at the LED, 5-15 cm away, and run again.")
                return None
            setting, spot = found
            spot_core = spot
            if args.auto:
                spot = {"rows": (max(0, spot["rows"][0] - AUTO_MARGIN), min(full_rows, spot["rows"][1] + AUTO_MARGIN)),
                        "cols": (max(0, spot["cols"][0] - AUTO_MARGIN),
                                 min(int(st["mode"]["width"]), spot["cols"][1] + AUTO_MARGIN)),
                        "contrast": spot.get("contrast")}
                set_controls(base, None)                   # the user's controls: automatic exposure
                end = time.monotonic() + 8.0
                while time.monotonic() < end:
                    ctl = {c["key"]: c["value"] for c in camera_status().get("controls") or []}
                    if ctl.get("auto_exposure") not in (None, 1):
                        break
                    time.sleep(0.2)
                else:
                    raise RuntimeError("the camera did not go back to automatic exposure within 8 s")
                print(f"automatic exposure back on; {AUTO_SETTLE_S:g} s to settle; then {events} switches", flush=True)
                time.sleep(AUTO_SETTLE_S)
            col = Frames(args.host)
            col.start()
            time.sleep(0.5)
            switches = []
            level = 1
            for _ in range(events):
                time.sleep(random.uniform(*GAP_S))     # nosec B311 — a random phase, not security
                switches.append(switch(robot, args.output, level, sw))
                level = 1 - level
            time.sleep(0.3)
            frames = col.finish()
        finally:
            try:
                robot.set_digital_output(args.output, 0)
            finally:
                set_controls(base, None)
                print("output", args.output, "off; the camera's settings put back:", base["controls"])
        arr = as_arrays(frames)
        blob = b"".join(arr["jpegs"])
        np.savez(os.path.join(out, "frames.npz"), exposure_time=arr["exposure_time"], flags=arr["flags"],
                 jpeg_offsets=np.cumsum([0] + [len(j) for j in arr["jpegs"]]),
                 jpeg_bytes=np.frombuffer(blob, dtype=np.uint8))
        with open(os.path.join(out, "run.json"), "w", encoding="utf-8") as f:
            json.dump({"output": args.output, "setting": setting, "spot": spot, "switches": switches,
                       "auto": bool(args.auto), "spot_core": spot_core,
                       "usb_id": (st.get("device") or {}).get("usb_id"),
                       "mode": cs.mode_key(st["mode"]["width"], st["mode"]["height"], fps),
                       "full_rows": full_rows, "fps": fps, "timing": cs.camera_timing(gcam, cs.mode_key(
                           st["mode"]["width"], st["mode"]["height"], fps))}, f, indent=1)
        print(f"{len(switches)} switches, {len(frames)} frames: {os.path.relpath(out, _ROOT)}", flush=True)
    finally:
        robot.disconnect()
    return out


def reanalyse(folder: str) -> dict:
    with open(os.path.join(folder, "run.json"), encoding="utf-8") as f:
        run_ = json.load(f)
    z = np.load(os.path.join(folder, "frames.npz"))
    blob, offs = z["jpeg_bytes"].tobytes(), z["jpeg_offsets"]
    frames = {"exposure_time": z["exposure_time"], "flags": z["flags"],
              "jpegs": [blob[offs[i]:offs[i + 1]] for i in range(len(offs) - 1)]}
    spot = {"rows": tuple(run_["spot"]["rows"]), "cols": tuple(run_["spot"]["cols"])}
    timing = run_.get("timing") or {}
    readout_s = (timing.get("readout_us") or 31.8e3) * 1e-6
    if run_.get("auto"):
        res = analyse_auto(frames, [tuple(s) for s in run_["switches"]], spot, run_["full_rows"],
                           readout_s / run_["full_rows"], calib_offset_s=(timing.get("auto_us") or 0.0) * 1e-6,
                           pace=Pace(say=print))
        lines = [f"LED timing check under AUTOMATIC exposure, run {os.path.basename(folder)}: output {run_['output']}, "
                 f"the spot's crop rows {spot['rows'][0]}-{spot['rows'][1]}, {len(run_['switches'])} switches, the "
                 f"readout {readout_s * 1e3:.2f} ms (stored)"]
        lines += report_auto(res, run_.get("timing"))
    else:
        res = analyse(frames, [tuple(s) for s in run_["switches"]], spot, run_["setting"] * cs.EXPOSURE_UNIT_US * 1e-6,
                      run_["full_rows"], row_time_s=readout_s / run_["full_rows"], pace=Pace(say=print))
        lines = [f"LED timing check, run {os.path.basename(folder)}: output {run_['output']}, exposure "
                 f"{run_['setting'] / 10:g} ms, the spot at rows {spot['rows'][0]}-{spot['rows'][1]}"]
        lines += report(res, run_.get("timing"), run_["fps"])
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(folder, "report.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    with open(os.path.join(folder, "result.json"), "w", encoding="utf-8") as f:
        json.dump(res, f, indent=1, default=float)
    return res


# ------------------------------------------------------------------------------------------------------- selftest
def selftest() -> int:
    """Synthetic frames of a rolling-shutter camera watching an LED switched at known moments."""
    from PIL import Image
    fails = []

    def chk(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + (f"   [{detail}]" if detail and not cond else ""))
        if not cond:
            fails.append(name)

    rng = np.random.default_rng(7)
    h, w = 180, 320
    readout, a_true, exposure = 31.8e-3, 4.2e-3, 0.5e-3
    period = 1 / 30.0
    row_t = readout / h
    spot = {"rows": (40, 150), "cols": (120, 220)}
    sw, t = [], 0.4
    level = 1
    while t < 30.0:
        sw.append((t, level))
        level = 1 - level
        t += rng.uniform(*GAP_S)
    sw_t = np.array([s[0] for s in sw])
    sw_l = np.array([s[1] for s in sw])

    def led_level_fraction(t0, t1):
        """The share of [t0, t1] the LED was on."""
        pts = np.concatenate(([t0], sw_t[(sw_t > t0) & (sw_t < t1)], [t1]))
        on = 0.0
        for u0, u1 in zip(pts, pts[1:]):
            i = np.searchsorted(sw_t, (u0 + u1) / 2) - 1
            on += (u1 - u0) * (sw_l[i] if i >= 0 else 0)
        return on / (t1 - t0)

    times = 0.3 + np.arange(int(30.0 / period)) * period
    jpegs = []
    rows = np.arange(h)
    for tk in times:
        img = np.full((h, w), 30.0) + rng.normal(0, 2, (h, w))
        ends = tk + a_true + (rows - h / 2) * row_t
        for y in range(spot["rows"][0], spot["rows"][1]):
            img[y, spot["cols"][0]:spot["cols"][1]] += 180 * led_level_fraction(ends[y] - exposure, ends[y])
        buf = io.BytesIO()
        Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(buf, "JPEG", quality=90)
        jpegs.append(buf.getvalue())
    frames = {"exposure_time": times, "flags": np.full(len(times), 0x09), "jpegs": jpegs}
    on_imgs = [gray(jpegs[i]) for i in range(len(times)) if led_level_fraction(times[i] - 0.03, times[i] + 0.05) == 1]
    off_imgs = [gray(jpegs[i]) for i in range(len(times)) if led_level_fraction(times[i] - 0.03, times[i] + 0.05) == 0]
    found = find_led(np.mean(on_imgs[:10], axis=0) - np.mean(off_imgs[:10], axis=0))
    chk("A1 the LED's spot found from frames with it on and off", found is not None
        and abs(found["rows"][0] - 40) <= 3 and abs(found["rows"][1] - 150) <= 3, found)
    res = analyse(frames, sw, found or spot, exposure, h, row_time_s=row_t)
    chk(f"A2 the readout and the centre row's readout time recovered: {res.get('readout_ms', float('nan')):.2f} ms "
        f"(true {readout * 1e3:g}), a {res.get('a_ms', float('nan')):+.3f} ms (true {a_true * 1e3:+g}), "
        f"{res.get('edges')} edges", res.get("ok") and abs(res["readout_ms"] - readout * 1e3) < 0.6
        and abs(res["a_ms"] - a_true * 1e3) < 0.15, res)
    flat = dict(frames, flags=np.full(len(times), 0x0B))
    chk("A3 frames with a calibrated time are refused (the check needs the camera's own time)",
        not analyse(flat, sw, spot, exposure, h, row_time_s=row_t)["ok"])
    small = {"rows": (60, 64), "cols": (120, 220)}             # 4 rows: an edge on it is rare
    jp2 = []
    for tk in times:
        img = np.full((h, w), 30.0) + rng.normal(0, 2, (h, w))
        ends = tk + a_true + (rows - h / 2) * row_t
        for y in range(*small["rows"]):
            img[y, small["cols"][0]:small["cols"][1]] += 40 * led_level_fraction(ends[y] - exposure, ends[y])
        buf = io.BytesIO()
        Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(buf, "JPEG", quality=90)
        jp2.append(buf.getvalue())
    res2 = analyse(dict(frames, jpegs=jp2), sw, small, exposure, h, row_time_s=row_t)
    chk(f"A2b a 4-row, dim spot (contrast 40, as the first run's): the bounds of every switch give a within its "
        f"range (under 1 ms wide): {res2.get('a_ms', float('nan')):+.3f} +-{res2.get('a_range_ms', float('nan')) / 2:.3f} ms "
        f"(true {a_true * 1e3:+g}), {res2.get('switches_used')} switches, {res2.get('agree')} agree",
        res2.get("ok") and res2.get("method") == "bounds" and abs(res2["a_ms"] - a_true * 1e3) <= res2["a_range_ms"] / 2
        + 0.02 and res2["a_range_ms"] < 1.0, res2)
    prof = np.array([30.0] * 10 + [120.0] + [210.0] * 9)
    chk("A4 an edge to a fraction of a row; none when the spot shows one level",
        abs(edge_row(prof, np.full(20, 210.0), np.full(20, 30.0), True) - 10.0) < 0.01
        and edge_row(np.full(20, 210.0), np.full(20, 210.0), np.full(20, 30.0), True) is None)
    lines = report(dict(res, ok=True), {"auto_us": 1413.9, "readout_us": 31814.0}, 30.0)
    chk("A5 the report gives the joint data's lateness from api_tests/80's automatic-exposure result",
        any("joint data" in ln for ln in lines) and any("31.81" in ln for ln in lines), lines)
    print("B. --auto: automatic exposure's length and moment (E1)")
    e_auto, a_auto, ro_auto = 30e-3, 16.69e-3, 30.23e-3
    rt_auto = ro_auto / h
    cy, cx = 95.0, 170.0
    yy, xx = np.mgrid[0:h, 0:w]
    led = 3.0 * np.exp(-((yy - cy) ** 2 + (xx - cx) ** 2) / (2 * 22.0 ** 2))   # its core saturates, its edge does not
    bg = 0.0067                                                                  # off: ~26 grey levels
    jp3 = []
    box = {"rows": (45, 146), "cols": (110, 231)}
    for tk in times:
        ends = tk + a_auto + (rows - h / 2) * rt_auto
        share = np.array([led_level_fraction(ends[y] - e_auto, ends[y]) for y in range(h)])
        lin = bg + share[:, None] * led
        img = 255.0 * np.clip(lin, 0, 1) ** 0.45 + rng.normal(0, 1.5, (h, w))    # a camera's tone curve
        buf = io.BytesIO()
        Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).save(buf, "JPEG", quality=92)
        jp3.append(buf.getvalue())
    fr3 = dict(frames, jpegs=jp3, flags=np.full(len(times), 0x0B), exposure_time=times + 1.4139e-3)
    r3 = analyse_auto(fr3, sw, box, h, rt_auto, calib_offset_s=1.4139e-3)
    chk(f"B1 a 30 ms automatic exposure (the LED's core saturated, a tone curve, a calibration of +1.41 ms active "
        f"while recording): the exposure {r3.get('exposure_ms', float('nan')):.2f} ms (true 30), the readout "
        f"{r3.get('a_ms', float('nan')):+.2f} (true +16.69), the image's moment {r3.get('moment_ms', float('nan')):+.2f} "
        f"(true +1.69) — within 0.5 / 0.3 / 0.3 ms, the halves within 0.5", r3.get("ok")
        and abs(r3["exposure_ms"] - 30.0) < 0.5 and abs(r3["a_ms"] - 16.69) < 0.3 and abs(r3["moment_ms"] - 1.69) < 0.3
        and r3["half_diff_ms"] < 0.5 and r3["pixels"] > 50, {k: r3.get(k) for k in ("ok", "why", "exposure_ms", "a_ms",
                                                                                     "moment_ms", "half_diff_ms", "pixels",
                                                                                     "switches_on_ramp", "power")})
    lines3 = report_auto(r3, {"auto_us": 1413.9, "run": "R"})
    chk("B2 ...the report: the exposure, the image's moment, the calibration in use and that --apply makes the moment "
        "the calibration", any("the exposure lasts" in ln for ln in lines3)
        and any("the calibration in use: +1.41 ms" in ln and "--apply FOLDER makes it" in ln for ln in lines3), lines3)
    hot = {"rows": (88, 102), "cols": (163, 177)}                               # only the saturated core
    r4 = analyse_auto(fr3, sw, hot, h, rt_auto, calib_offset_s=1.4139e-3)
    chk("B3 a crop of only the saturated core: no result, and the reason says what to do", not r4.get("ok")
        and "unsaturated" in (r4.get("why") or ""), r4)
    r5 = analyse_auto(dict(fr3, flags=np.full(len(times), 0x09), exposure_time=times), sw, box, h, rt_auto,
                      calib_offset_s=1.4139e-3)
    chk("B4 frames recorded WITHOUT a calibration (the camera's own times, not flagged): the offset is not taken off "
        "them, the same answer", r5.get("ok") and abs(r5["moment_ms"] - 1.69) < 0.3, r5.get("moment_ms"))
    print("C. --apply (true timing, 2026-11-25)")
    import shutil
    import tempfile
    import xml.etree.ElementTree as ET  # nosec B405 — our own gripper XMLs
    tmp = tempfile.mkdtemp(prefix="cam81_")
    try:
        gdir = os.path.join(tmp, "grippers")
        shutil.copytree(GRIPPERS_DIR, gdir)

        def run_dir(name, run_, res_):
            d = os.path.join(tmp, name)
            os.makedirs(d)
            for fname, obj in (("run.json", run_), ("result.json", res_)):
                if obj is not None:
                    with open(os.path.join(d, fname), "w", encoding="utf-8") as f:
                        json.dump(obj, f)
            return d

        def auto_us(fname="MSG_small_motor_150mm_rail.xml"):
            cam = ET.parse(os.path.join(gdir, fname)).getroot().find("camera")  # nosec B314
            return dict(cam.attrib) if cam is not None else {}

        good = {"ok": True, "moment_ms": 0.8, "half_diff_ms": 0.01}
        with open(os.devnull, "w") as quiet:
            saved_out, sys.stdout = sys.stdout, quiet
            try:
                rc1 = apply_result(run_dir("led_A", {"auto": True, "usb_id": "0bda:5844", "mode": "1280x720@30"}, good),
                                   gdir)
                after1 = auto_us()
                other = auto_us("Flange.xml") if os.path.exists(os.path.join(gdir, "Flange.xml")) else {}
                rc2 = apply_result(run_dir("led_B", {"auto": False, "usb_id": "0bda:5844", "mode": "1280x720@30"}, good),
                                   gdir)
                rc3 = apply_result(run_dir("led_C", {"auto": True, "usb_id": "0bda:5844", "mode": "1280x720@30"},
                                           dict(good, half_diff_ms=0.5)), gdir)
                rc4 = apply_result(run_dir("led_D", {"auto": True, "usb_id": "0bda:5844", "mode": "1280x720@30"}, None),
                                   gdir)
                after4 = auto_us()
                rc5 = apply_result(run_dir("led_E", {"auto": True, "timing": {"auto_us": 800.0}},
                                           dict(good, moment_ms=0.75)), gdir)
                after5 = auto_us()
            finally:
                sys.stdout = saved_out
        chk("C1 an --auto run's moment written as timing_auto_us (us) with the run's name, the camera's other values "
            "kept; a gripper without that camera untouched", rc1 == 0 and after1.get("timing_auto_us") == "800.0"
            and after1.get("timing_run") == "led_A" and after1.get("timing_readout_us") == "30230"
            and "timing_auto_us" not in other, (rc1, after1, other))
        chk("C2 refused: a run that was not --auto, halves more than 0.2 ms apart, no result.json", rc2 == 1 and rc3 == 1
            and rc4 == 1 and after4.get("timing_run") == "led_A", (rc2, rc3, rc4, after4))
        chk("C3 a run from before 2026-11-25 (no camera or mode in run.json): the gripper's camera and its stored mode",
            rc5 == 0 and after5.get("timing_auto_us") == "750.0" and after5.get("timing_run") == "led_E"
            and after5.get("timing_mode") == "1280x720@30", (rc5, after5))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print()
    print(f"{len(fails)} failed" if fails else "ALL PASSED")
    return 1 if fails else 0


def main():
    from _common import parser
    ap = parser(__doc__.split("\n")[0])
    ap.add_argument("--output", type=int, default=OUTPUT, choices=(1, 2, 3), help="the output whose LED the camera sees")
    ap.add_argument("--events", type=int, default=EVENTS, help="switches")
    ap.add_argument("--reanalyse", metavar="FOLDER")
    ap.add_argument("--apply", metavar="FOLDER",
                    help="an --auto run's image moment (its result.json) as the calibration under automatic exposure")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--auto", action="store_true",
                    help="after finding the LED, switch it under AUTOMATIC exposure: the exposure's length and the "
                         "image's moment (E1)")
    args = ap.parse_args()
    os.nice(19)                                    # the picture work at the lowest priority, beside RTI
    if args.selftest:
        sys.exit(selftest())
    if args.apply:
        sys.exit(apply_result(os.path.join(_ROOT, args.apply) if not os.path.isabs(args.apply) else args.apply))
    if args.reanalyse:
        if arm_enabled(args):
            sys.exit(2)
        res = reanalyse(os.path.join(_ROOT, args.reanalyse) if not os.path.isabs(args.reanalyse) else args.reanalyse)
        sys.exit(0 if res.get("ok") else 1)
    st, problems = checks()
    events = AUTO_EVENTS if args.auto and args.events == EVENTS else args.events
    print(f"camera: {st.get('state')}, mode {st.get('mode')}; plan: find output {args.output}'s LED (exposures "
          f"{', '.join(f'{s / 10:g}' for s in EXPOSURES_TRY)} ms), then {events} switches at random moments "
          + ("under AUTOMATIC exposure (the room's light as when recording) " if args.auto else "")
          + f"(~{events * 0.225 + 10:.0f} s); nothing moves; the output left off, the camera's settings put back")
    if problems:
        print("NOT READY:\n  " + "\n  ".join(problems))
        sys.exit(2)
    if not args.go:
        print("DRY RUN: aim the camera at the LED (5-15 cm), then re-run with --go.")
        return
    out = run(args)
    if out:
        res = reanalyse(out)
        sys.exit(0 if res.get("ok") else 1)
    sys.exit(1)


if __name__ == "__main__":
    from _common import main_guard
    main_guard(main)
