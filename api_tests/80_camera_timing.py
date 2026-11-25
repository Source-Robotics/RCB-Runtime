#!/usr/bin/env python3
"""The wrist camera's timing calibration: which moment of the robot's joint data each image shows (2026-11-24,
TODO/camera_plan.md P3, method B: the arm checks itself).

After P2 every frame carries the camera's own time (its PTS on the robot's clock) and RTI's tick there, to a few
microseconds. What is left is ONE constant per camera mode and exposure: the image's moment (the middle of the
exposure of the image's centre rows) is not the PTS, and the joint data RTI stamps with a tick has its own delay. This
measures that whole chain at once — the number that matters for pairing images with joint data:

  1. The arm at the api_tests' start pose, the camera looking at a still scene with some texture (shelves, a cluttered
     desk; not a blank wall), the camera's exposure as asked: "auto" (the default) and / or fixed settings. Each one is
     checked first: the camera must keep its full frame rate (a fixed exposure longer than a frame slows it) and give
     a picture bright enough to measure.
  2. J5 swings +-AMPLITUDE_DEG a few times (a small wrist pitch: the picture shifts up and down) while the robot
     captures RTI's per-tick joint angles (robot.capture_telemetry) and this script receives every frame
     (robotclient.CameraClient). The camera's settings are put back afterwards.
  3. With the arm disabled (decoding a thousand JPEGs is CPU work), each pair of consecutive frames gives the
     picture's shift to a fraction of a pixel (phase correlation on a quarter-size grey image), and the shifts are
     fitted against J5's motion between the same two ticks, moved by a delay d: shift = c x (theta(tick + d) -
     theta(previous tick + d)). The d that fits best is the delay at that exposure. A frame's tick is worked out
     here, from its time and the capture's own rows; the tick in the frame's header (the camera server's) is only
     CHECKED against it, and the report says how far it was off (review 2026-11-25: it was ~2 ms rms off, and the
     first calibration was measured through it).
  4. Automatic exposure gives one offset. Indoors it holds: this camera drops its gain at a FIXED exposure (a fixed
     33 ms is black under the lab's lamps, 0.8 / 255, while automatic gives 137 at the same 30 fps — measured
     2026-11-24), so its automatic mode exposes for the whole frame and adds gain there; in bright light it may
     shorten the exposure, up to half a frame off then. Fixed settings (exposure_time_absolute, 100 us per unit —
     confirmed by the frame rate: 1000 -> 9.92 fps, 4000 -> 2.49 fps) give a line, d = offset + k x setting, with k
     = +50 us per unit when the camera's PTS marks the exposure's start and -50 when it marks its end; they need
     light bright enough for a fixed exposure shorter than a frame.

--apply writes the result into every gripper XML that declares this camera (<camera timing_*>,
utility/camera_settings.py; a calibration of the other kind for the same mode is kept); the camera server then adds it
to every frame's time and flags it (FLAG_CALIBRATED).

    .venv/bin/python api_tests/80_camera_timing.py                 # dry run: the checks, the plan; nothing moves
    .venv/bin/python api_tests/80_camera_timing.py --go            # record; analyse once the arm is disabled
    .venv/bin/python api_tests/80_camera_timing.py --go --apply    # ... and write a confident result
    .venv/bin/python api_tests/80_camera_timing.py --go --exposures auto,150,250   # + fixed settings (bright light)
    .venv/bin/python api_tests/80_camera_timing.py --reanalyse temp_data/camera_timing/RUN [--apply]
    .venv/bin/python api_tests/80_camera_timing.py --selftest      # the analysis on synthetic frames, no robot

Results: temp_data/camera_timing/<UTC>/ (run.json, frames_<setting>.npz, the captures' copies, report.txt).
"""
from __future__ import annotations

import glob
import io
import json
import math
import os
import shutil
import sys
import threading
import time
import xml.etree.ElementTree as ET  # nosec B405 — our own gripper XMLs

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.dirname(_HERE)
sys.path.insert(0, _ROOT)
sys.path.insert(0, _HERE)

from utility import camera_settings as cs  # noqa: E402

JOINT = 5                          # the wrist pitch: the camera's picture shifts, it does not turn
AMPLITUDE_DEG = 8.0
SWING_SPEED = 40.0                 # percent of the joint's EXEC limits
SWINGS = 8                         # moves per exposure, each to the other side
EXPOSURES = ("auto",)              # what works under the lab's lamps (a fixed exposure is black there)
# --verify (2026-11-24, the user: "confirm that these tests we did are really correct"): the same delay from other
# motions, nothing written. A pure timing delay does not change with the SPEED. On the MSG gripper J5's delay does
# (+1.0 to +2.3 ms at 17 to 37 deg/s) and the cause is not known: a gearbox gap and an acceleration term were fitted
# to the recordings and rejected (review 2026-11-25). Another joint (J6 turns the picture: the tiles see it move
# round) has another gearbox and another lever.
VERIFY_VARIANTS = ((5, 8.0, 20.0), (5, 8.0, 40.0), (5, 8.0, 60.0), (6, 15.0, 40.0))   # (joint, +-deg, speed %)
VERIFY_SPREAD_MS = 1.0             # the variants' delays must agree this well
SIZE = (640, 360)                  # the grey image the shifts are measured on (the JPEG decoded at 1/2 size)
TILES = (6, 4)                     # rows x columns of tiles, each fitted on its own (the rolling shutter: every row
                                   # band has its own moment; the gripper's jaws in view move WITH the camera)
MIN_PAIRS = 150                    # frame pairs a tile needs
SEARCH_TICKS = 40.0                # the delay is searched this far each way; a frame nearer than this to the
                                   # capture's first / last row is left out (np.interp holds the end value there)
MAX_TICK_RMS_MS = 0.2              # the header's rti_tick against the capture's rows: more is said in the report
MIN_TILE_R2 = 0.95                 # a tile whose shifts follow the joint less well is left out (the jaws: 0.1-0.6)
MAX_TILE_HALVES_MS = 3.0           # ... and one whose two halves disagree more
MIN_GOOD_TILES = 8
MAX_HALF_DIFF_MS = 1.0             # the centre row's delay from the two halves of a run must agree this well
MAX_LINE_RESIDUAL_MS = 1.0         # the fixed exposures' delays must lie on a line this well
MAX_UNIT_ERROR_US = 15.0           # ... with a slope within this of +-50 us per setting unit
MIN_TEXTURE = 2.0                  # mean gradient of the grey picture (0..255): below it the scene is too plain
MIN_BRIGHTNESS = 15.0              # mean grey (0..255) an exposure must give
MIN_FPS_SHARE = 0.9                # an exposure must keep this share of the mode's frame rate
DISABLE_WAIT_S = 900.0
TD = os.path.join(_ROOT, "temp_data", "camera_timing")
SYSTEM_XML = os.path.join(_ROOT, "config", "system.xml")
GRIPPERS_DIR = os.path.join(_ROOT, "grippers")


def parse_setting(text):
    """"auto" or a fixed exposure setting (exposure_time_absolute, an int)."""
    text = str(text).strip().lower()
    return "auto" if text == "auto" else int(text)


def setting_name(setting) -> str:
    return "automatic exposure" if setting == "auto" else (f"fixed {setting} "
                                                           f"({setting * cs.EXPOSURE_UNIT_US / 1000:g} ms)")


# ------------------------------------------------------------------------------------------------------- analysis
def decode_gray(jpeg: bytes, size=SIZE) -> np.ndarray:
    """The JPEG as a grey float image of `size` (width, height): decoded at a reduced scale by the JPEG decoder itself
    (draft), so a 1280x720 frame costs a few ms."""
    from PIL import Image
    im = Image.open(io.BytesIO(jpeg))
    im.draft("L", size)
    im = im.convert("L")
    if im.size != tuple(size):
        im = im.resize(size)
    return np.asarray(im, dtype=np.float32)


def texture(gray: np.ndarray) -> float:
    """The picture's mean gradient (0..255 per pixel): how much there is to measure a shift on."""
    return float(np.mean(np.abs(np.diff(gray, axis=0))) + np.mean(np.abs(np.diff(gray, axis=1))))


def _window(shape) -> np.ndarray:
    return np.outer(np.hanning(shape[0]), np.hanning(shape[1])).astype(np.float32)


def _spectrum(gray: np.ndarray, win: np.ndarray) -> np.ndarray:
    return np.fft.rfft2((gray - gray.mean()) * win)


PEAK_SIGMA_F = 0.08                # the cross-power's low-pass (cycles per pixel): the peak a ~2 px Gaussian


def _lowpass(shape) -> np.ndarray:
    fy, fx = np.fft.fftfreq(shape[0])[:, None], np.fft.rfftfreq(shape[1])[None, :]
    return np.exp(-(fy ** 2 + fx ** 2) / (2 * PEAK_SIGMA_F ** 2))


def _gauss_offset(r_minus, r0, r_plus) -> float:
    """The sub-pixel offset of a Gaussian peak from three samples (exact for a Gaussian)."""
    lm, l0, lp = (math.log(max(float(v), 1e-12)) for v in (r_minus, r0, r_plus))
    den = lm - 2.0 * l0 + lp
    return 0.0 if den == 0 else 0.5 * (lm - lp) / den


def phase_shift(fa: np.ndarray, fb: np.ndarray, shape, lowpass: np.ndarray) -> tuple[float, float, float]:
    """(dy, dx, peak): the shift of picture b against picture a from their spectra. Phase correlation with the
    whitened cross-power LOW-PASSED (the high frequencies are JPEG noise), so the peak is a Gaussian and three samples
    place it exactly: measured 2026-11-24 on the real scene shifted by known amounts, 0.010 px rms (the plain
    parabola fit: 0.077 px, and 0.95 px on a smooth scene). peak: how clearly one shift explains the two pictures."""
    cross = fa.conj() * fb
    cross = cross / (np.abs(cross) + 1e-12) * lowpass
    r = np.fft.irfft2(cross, s=shape)
    h, w = shape
    i, j = np.unravel_index(int(np.argmax(r)), r.shape)
    dy = i + _gauss_offset(r[(i - 1) % h, j], r[i, j], r[(i + 1) % h, j])
    dx = j + _gauss_offset(r[i, (j - 1) % w], r[i, j], r[i, (j + 1) % w])
    # the peak of a perfect match is the weights' sum over the full spectrum / its size (the half spectrum counts
    # twice): ~1 for identical pictures, near 0 for unrelated ones
    return (dy - h if dy > h / 2 else dy), (dx - w if dx > w / 2 else dx), float(r[i, j] * h * w / (2.0 * lowpass.sum()))


def shifts(grays: list) -> tuple[np.ndarray, np.ndarray]:
    """The shift (dy, dx) of every picture against the one before, and each one's peak."""
    shape = grays[0].shape
    win = _window(shape)
    lowpass = _lowpass(shape)
    specs = [_spectrum(g, win) for g in grays]
    out, peaks = [], []
    for a, b in zip(specs, specs[1:]):
        dy, dx, p = phase_shift(a, b, shape, lowpass)
        out.append((dy, dx))
        peaks.append(p)
    return np.array(out, dtype=float).reshape(-1, 2), np.array(peaks)


def _tile_fit(tiles: list, ticks: np.ndarray, cap_ticks: np.ndarray, theta: np.ndarray) -> dict | None:
    """One tile's delay: its shifts along their main direction, fitted against the joint (the whole run and each
    half); None when too few frame pairs show a clear shift."""
    d2, peaks = shifts(tiles)
    moving = np.hypot(d2[:, 0], d2[:, 1]) > 0.3
    u = np.linalg.svd(d2[moving] if moving.sum() >= 10 else d2, full_matrices=False)[2][0]
    disp = d2 @ u
    keep = np.nonzero(peaks > 0.05)[0]           # a weak correlation's shift is a guess: left out
    if len(keep) < MIN_PAIRS:
        return None
    half = len(keep) // 2
    fits = [fit_delay_pairs(ticks[sel], ticks[sel + 1], disp[sel], cap_ticks, theta)
            for sel in (keep, keep[:half], keep[half:])]
    return {"delay_ticks": fits[0]["delay_ticks"], "r2": fits[0]["r2"], "px_per_rad": fits[0]["px_per_unit"],
            "h1_ticks": fits[1]["delay_ticks"], "h2_ticks": fits[2]["delay_ticks"], "pairs": int(len(keep)),
            "at_edge": fits[0]["at_edge"]}


def _row_line(rows: np.ndarray, delays: np.ndarray) -> tuple[float, float, np.ndarray]:
    """(intercept, slope, kept) of delay = intercept + slope x row, a tile far off the first line (> max(1.5 ms, 3
    robust sigmas)) left out of the second."""
    k, a = np.polyfit(rows, delays, 1)
    r = delays - (a + k * rows)
    mad = float(np.median(np.abs(r - np.median(r)))) * 1.4826
    kept = np.abs(r) <= max(1.5, 3.0 * mad)
    if kept.sum() >= 3 and not kept.all():
        k, a = np.polyfit(rows[kept], delays[kept], 1)
    return float(a), float(k), kept


def analyse_exposure(frames: dict, capture: dict, joint: int, uncal_offset_s: float = 0.0, size=SIZE, tiles=TILES,
                     full_rows: int = 720) -> dict:
    """One exposure's delay at the image's CENTRE row, and the rolling shutter's readout. frames: the saved frames
    (exposure_time, rti_tick, flags, jpegs); capture: the robot's capture (tick, timestamp, j_p); uncal_offset_s: a
    calibration that was ACTIVE while recording (taken back off); full_rows: the camera mode's height.

    The picture is cut into tiles, each fitted on its own: a tile that does not follow the joint (the gripper's jaws
    in view, which move WITH the camera) is left out, and the rolling shutter shows as a delay that grows with the
    tile's row (measured 2026-11-24: -12 ms at the top, +13 ms at the bottom, 30.8 ms for 720 rows). A line through
    the good tiles' delays over their rows gives the delay at the centre row and the readout."""
    from communication import camera_protocol as cp
    cap_ticks = np.asarray(capture["tick"], dtype=float)
    cap_time = np.asarray(capture["timestamp"], dtype=float)
    tick_s = float(np.median(np.diff(cap_time) / np.diff(cap_ticks)))
    theta = np.asarray(capture["j_p"], dtype=float)[:, joint - 1]
    # Each frame's tick from its OWN time and the capture's rows (both on the robot's clock), never the header's
    # rti_tick: that is the camera server's work, which this test checks instead of resting on it.
    flags = np.asarray(frames["flags"])
    t_frame = np.asarray(frames["exposure_time"], dtype=float)
    own_tick = np.interp(t_frame - np.where((flags & cp.FLAG_CALIBRATED) > 0, uncal_offset_s, 0.0), cap_time, cap_ticks)
    clock = (flags & cp.FLAG_CAMERA_CLOCK) > 0
    inside = (own_tick >= cap_ticks[0] + SEARCH_TICKS) & (own_tick <= cap_ticks[-1] - SEARCH_TICKS)
    idx = np.nonzero(clock & inside)[0]
    outside = int((clock & ~inside).sum())
    if len(idx) < MIN_PAIRS + 1:
        return {"ok": False, "why": f"only {len(idx)} frames with the camera's clock inside the capture ({outside} at "
                                    f"or past its ends)"}
    ticks = own_tick[idx]
    stamped = clock & inside & np.isfinite(frames["rti_tick"])
    tick_check = None                      # the header's tick against the rows, at the time the header gives
    if stamped.sum() >= 2:
        err = (np.asarray(frames["rti_tick"], dtype=float)[stamped]
               - np.interp(t_frame[stamped], cap_time, cap_ticks)) * tick_s * 1e3
        tick_check = {"frames": int(stamped.sum()), "rms_ms": float(np.sqrt(np.mean(err ** 2))),
                      "mean_ms": float(err.mean()), "worst_ms": float(np.abs(err).max())}
    # 8-bit: a quarter of the memory (a 794-frame run kept as floats was ~730 MB on a Pi with ~1 GB free, 2026-11-24)
    grays = [decode_gray(frames["jpegs"][k], size).astype(np.uint8) for k in idx]
    h, w = grays[0].shape
    nr, nc = tiles
    th, tw = h // nr, w // nc
    results = []
    for r in range(nr):
        for c in range(nc):
            fit = _tile_fit([g[r * th:(r + 1) * th, c * tw:(c + 1) * tw].astype(np.float32) for g in grays], ticks,
                            cap_ticks, theta)
            if fit is not None:
                fit.update(row=(r + 0.5) * th * full_rows / h, tile=(r, c))
                results.append(fit)
    good = [t for t in results if t["r2"] >= MIN_TILE_R2 and not t["at_edge"]
            and abs(t["h1_ticks"] - t["h2_ticks"]) * tick_s * 1e3 <= MAX_TILE_HALVES_MS]
    fps = 1.0 / float(np.median(np.diff(frames["exposure_time"][idx])))
    base = {"frames": int(len(idx)), "frames_outside": outside, "tick_check": tick_check,
            "tick_s": tick_s, "fps": fps, "brightness": float(np.mean(grays[0])),
            "texture": texture(grays[0]), "tiles_total": nr * nc, "tiles_good": len(good),
            "tiles": [{k: (list(v) if k == "tile" else v) for k, v in t.items()} for t in results]}
    if len(good) < MIN_GOOD_TILES:
        return dict(base, ok=False, why=f"only {len(good)} of {nr * nc} tiles follow the joint well (texture, light, "
                                        f"the scene moving?)")
    rows = np.array([t["row"] for t in good])
    ms = tick_s * 1e3
    a, k, kept = _row_line(rows, np.array([t["delay_ticks"] for t in good]) * ms)
    a1, k1, _ = _row_line(rows, np.array([t["h1_ticks"] for t in good]) * ms)
    a2, k2, _ = _row_line(rows, np.array([t["h2_ticks"] for t in good]) * ms)
    centre = full_rows / 2.0
    resid = np.array([t["delay_ticks"] for t in good]) * ms - (a + k * rows)
    out = dict(base, ok=True, delay_ms=a + k * centre, readout_ms=k * full_rows,
               half_diff_ms=abs((a1 + k1 * centre) - (a2 + k2 * centre)),
               scatter_ms=float(np.std(resid[kept])), tiles_used=int(kept.sum()),
               r2=float(np.median([t["r2"] for t in good])), pairs=int(min(t["pairs"] for t in good)),
               row_span=float(rows.max() - rows.min()) / full_rows)
    frame_ms = 1e3 / fps
    out["confident"] = bool(out["tiles_used"] >= MIN_GOOD_TILES and out["row_span"] >= 0.5
                            and out["half_diff_ms"] <= MAX_HALF_DIFF_MS and 0.0 < out["readout_ms"] <= 1.2 * frame_ms)
    return out


def fit_delay_pairs(t0: np.ndarray, t1: np.ndarray, disp: np.ndarray, cap_ticks: np.ndarray, theta: np.ndarray,
                    lo: float = -SEARCH_TICKS, hi: float = SEARCH_TICKS) -> dict:
    """The delay d (ticks) for which the picture's shifts best follow the joint, over frame pairs (t0 -> t1, the
    pictures' shift disp along the motion): disp = c x (theta(t1 + d) - theta(t0 + d)), c fitted for each d (the
    pixels per unit of theta); a 0.25-tick grid over lo..hi, then a golden-section search to 0.001 tick."""
    def sse(d):
        x = np.interp(t1 + d, cap_ticks, theta) - np.interp(t0 + d, cap_ticks, theta)
        xx = float(x @ x)
        c = float(x @ disp) / xx if xx > 0 else 0.0
        r = disp - c * x
        return float(r @ r), c

    grid = np.arange(lo, hi + 1e-9, 0.25)
    vals = np.array([sse(d)[0] for d in grid])
    i = int(np.argmin(vals))
    a, b = grid[max(0, i - 1)], grid[min(len(grid) - 1, i + 1)]
    g = (math.sqrt(5) - 1) / 2
    x1, x2 = b - g * (b - a), a + g * (b - a)
    f1, f2 = sse(x1)[0], sse(x2)[0]
    while b - a > 1e-3:
        if f1 < f2:
            b, x2, f2 = x2, x1, f1
            x1 = b - g * (b - a)
            f1 = sse(x1)[0]
        else:
            a, x1, f1 = x1, x2, f2
            x2 = a + g * (b - a)
            f2 = sse(x2)[0]
    d = (a + b) / 2
    s, c = sse(d)
    total = float(disp @ disp)
    return {"delay_ticks": d, "px_per_unit": c, "r2": 1.0 - s / total if total > 0 else 0.0,
            "at_edge": bool(i in (0, len(grid) - 1))}


def exposure_line(results: dict) -> dict:
    """The FIXED exposures' delays -> delay_ms = offset_ms + k_ms x setting (two settings or more). With 100 us per
    setting unit the slope must be +-0.05 ms per unit: + when the PTS marks the exposure's start, - its end."""
    xs = np.array(sorted(results), dtype=float)
    ys = np.array([results[int(x)]["delay_ms"] for x in xs])
    k, off = np.polyfit(xs, ys, 1)
    resid = ys - (off + k * xs)
    half_unit_ms = cs.EXPOSURE_UNIT_US / 2e3
    return {"offset_ms": float(off), "k_ms": float(k), "residual_ms": float(np.max(np.abs(resid))) if len(xs) > 2 else 0.0,
            "unit_error_us": abs(abs(k) - half_unit_ms) * 1e3,
            "pts_marks": "the exposure's start" if k > 0 else "the exposure's end"}


# ------------------------------------------------------------------------------------------------------- --apply
def write_timing(xml_path: str, usb_id: str, mode: str, values: dict, run: str, backup_dir: str) -> bool:
    """The timing attributes `values` (timing_auto_us and / or timing_offset_us + timing_us_per_unit) into the gripper
    XML's <camera> element when it declares `usb_id`; the other kind's values kept when they are for the same mode;
    a backup first; comments and everything else kept. False when the file has no such camera."""
    with open(xml_path, encoding="utf-8") as f:
        text = f.read()
    found = cs._elements(text)
    if len(found) != 1:
        return False
    start, end = found[0]
    attrs = dict(ET.fromstring(text[start:end]).attrib)  # nosec B314 — our own gripper XML
    if attrs.get("usb_id", "").lower() != usb_id.lower():
        return False
    old = {k: v for k, v in attrs.items() if k in cs.TIMING_KEYS}
    attrs = {k: v for k, v in attrs.items() if k not in cs.TIMING_KEYS}
    timing = {"timing_mode": mode}
    if old.get("timing_mode") == mode:                    # the same mode: what this run did not measure stays
        timing.update({k: v for k, v in old.items() if k not in ("timing_mode", "timing_run")})
    timing.update(values)
    timing["timing_run"] = run
    attrs.update(timing)
    elem = "<camera " + " ".join(f'{k}="{v}"' for k, v in attrs.items()) + "/>"
    os.makedirs(backup_dir, exist_ok=True)
    shutil.copy2(xml_path, os.path.join(backup_dir, os.path.basename(xml_path)))
    from utility.config_write import write_atomic
    write_atomic(xml_path, text[:start] + elem + text[end:])
    return True


def apply(run_dir: str, summary: dict) -> list:
    """Write a confident result into every gripper XML declaring the run's camera; the files written."""
    written = []
    for path in sorted(glob.glob(os.path.join(GRIPPERS_DIR, "*.xml"))):
        if write_timing(path, summary["usb_id"], summary["mode"], summary["values"], os.path.basename(run_dir),
                        os.path.join(run_dir, "backup")):
            written.append(os.path.basename(path))
    return written


# ------------------------------------------------------------------------------------------------------- files
def save_frames(path: str, frames: list) -> None:
    blob = b"".join(f.jpeg for f in frames)
    offs = np.cumsum([0] + [len(f.jpeg) for f in frames])
    np.savez(path, exposure_time=np.array([f.exposure_time for f in frames]),
             rti_tick=np.array([np.nan if f.rti_tick is None else f.rti_tick for f in frames]),
             flags=np.array([int(f.camera_clock) | int(f.calibrated) << 1 | int(f.auto_exposure) << 2
                             | int(f.rti_tick is not None) << 3 for f in frames], dtype=np.int64),
             pts=np.array([f.pts for f in frames], dtype=np.int64), frame=np.array([f.frame for f in frames]),
             jpeg_offsets=offs, jpeg_bytes=np.frombuffer(blob, dtype=np.uint8))


def load_frames(path: str) -> dict:
    z = np.load(path)
    blob, offs = z["jpeg_bytes"].tobytes(), z["jpeg_offsets"]
    return {"exposure_time": z["exposure_time"], "rti_tick": z["rti_tick"], "flags": z["flags"], "pts": z["pts"],
            "frame": z["frame"], "jpegs": [blob[offs[i]:offs[i + 1]] for i in range(len(offs) - 1)]}


def summarise(run: dict, results: dict) -> tuple[dict, list]:
    """The values to write and the report lines, from the per-setting results."""
    lines, values = [], {}
    auto = results.get("auto")
    fixed = {s: r for s, r in results.items() if s != "auto" and r.get("ok") and r.get("confident")}
    confident = True
    readouts = [r["readout_ms"] for r in results.values() if r.get("ok") and r.get("confident")]
    if readouts:
        values["timing_readout_us"] = f"{float(np.mean(readouts)) * 1e3:.0f}"
        lines.append(f"  -> the rolling shutter: the image's rows are read out over {np.mean(readouts):.2f} ms, top to "
                     f"bottom (a row's own moment = the frame's time + (row - centre) x that / rows)")
    if auto is not None:
        if auto.get("ok") and auto.get("confident"):
            values["timing_auto_us"] = f"{auto['delay_ms'] * 1e3:.1f}"
            lines.append(f"  -> automatic exposure: image moment = camera time {auto['delay_ms']:+.2f} ms (holds while "
                         f"the camera exposes for the whole frame: indoors; in bright light up to half a frame off)")
        else:
            confident = False
    if len(fixed) >= 2:
        line = exposure_line(fixed)
        ok = line["residual_ms"] <= MAX_LINE_RESIDUAL_MS and line["unit_error_us"] <= MAX_UNIT_ERROR_US
        lines.append(f"  -> fixed exposures: image moment = camera time {line['offset_ms']:+.2f} ms "
                     f"{line['k_ms'] * 1e3:+.2f} us x setting (expected +-{cs.EXPOSURE_UNIT_US / 2:g}: the PTS marks "
                     f"{line['pts_marks']}); the settings' scatter about the line {line['residual_ms']:.2f} ms"
                     + ("" if ok else "  — NOT confident"))
        if ok:
            values["timing_offset_us"] = f"{line['offset_ms'] * 1e3:.1f}"
            values["timing_us_per_unit"] = f"{line['k_ms'] * 1e3:.4f}"
        confident = confident and ok
    elif fixed:
        lines.append("  fixed exposures: one confident setting is not a line (two or more are needed) — not written")
    if not (auto or fixed) or (auto is None and len(fixed) < 2):
        values.pop("timing_readout_us", None)            # a readout alone is no calibration
    tried = [s for s in results if s != "auto"]
    if tried and len(fixed) < len(tried):
        confident = False
    return values, lines + (["  CONFIDENT" if confident and values else "  NOT confident: nothing to write"
                             if not values else "  NOT confident: not every exposure gave a confident delay"]), \
        confident and bool(values)


def reanalyse(run_dir: str, do_apply: bool = False) -> dict:
    with open(os.path.join(run_dir, "run.json"), encoding="utf-8") as f:
        run = json.load(f)
    results = {}
    lines = [f"camera timing calibration, run {os.path.basename(run_dir)}: J{run['joint']} +-{run['amplitude_deg']} deg, "
             f"mode {run['mode']}, camera {run['usb_id']}"]
    for e in run["exposures"]:
        setting = parse_setting(e["setting"])
        frames = load_frames(os.path.join(run_dir, e["frames"]))
        capture = dict(np.load(os.path.join(run_dir, e["capture"])))
        active = cs.timing_offset_s(run.get("timing_active"), None if setting == "auto" else setting) or 0.0
        r = analyse_exposure(frames, capture, e.get("joint", run["joint"]), active,
                             full_rows=int(run["mode"].split("x")[1].split("@")[0]))
        r.update(joint=e.get("joint", run["joint"]), speed=e.get("speed", run.get("swing_speed")),
                 amplitude_deg=e.get("amplitude_deg", run["amplitude_deg"]))
        label = (f"J{r['joint']} +-{r['amplitude_deg']:g} deg at {r['speed']:g} %" if run.get("verify")
                 else setting_name(setting))
        results[e.get("tag", setting) if run.get("verify") else setting] = r
        if r["ok"]:
            lines.append(f"  {label:>22}: delay at the centre row {r['delay_ms']:+7.2f} ms (halves "
                         f"differ {r['half_diff_ms']:.2f} ms), readout {r['readout_ms']:.1f} ms, {r['tiles_used']} of "
                         f"{r['tiles_total']} tiles (scatter {r['scatter_ms']:.2f} ms, median R2 {r['r2']:.3f}), "
                         f"{r['fps']:.2f} fps, brightness {r['brightness']:.0f}/255"
                         + ("" if r["confident"] else "  — NOT confident"))
            lines.append("      tiles (delay ms, R2): " + "  ".join(
                f"[{t['tile'][0]},{t['tile'][1]}] {t['delay_ticks'] * r['tick_s'] * 1e3:+.1f} {t['r2']:.2f}"
                for t in r["tiles"]))
        else:
            lines.append(f"  {label:>22}: no result — {r['why']}")
        lines += tick_lines(r)
    if run.get("verify"):
        return _verify_report(run_dir, run, results, lines)
    values, more, confident = summarise(run, results)
    lines += more
    summary = {"usb_id": run["usb_id"], "mode": run["mode"], "results": {str(k): v for k, v in results.items()},
               "values": values, "confident": confident}
    if do_apply:
        if confident:
            written = apply(run_dir, summary)
            lines.append(f"  --apply: written into {len(written)} gripper XML(s): {', '.join(written) or 'none'}; the "
                         f"camera server takes it within ~1 s (backups in {os.path.relpath(run_dir, _ROOT)}/backup)")
        else:
            lines.append("  --apply: not confident — nothing written")
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(run_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    with open(os.path.join(run_dir, "result.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1, default=float)
    return summary


def tick_lines(r: dict) -> list:
    """The report's lines on a recording's ticks: the frames left out at the capture's ends, and how far the tick in
    the frames' headers (the camera server's) lay from the capture's rows."""
    lines = []
    if r.get("frames_outside"):
        lines.append(f"      {r['frames_outside']} frames at or past the capture's ends left out")
    c = r.get("tick_check")
    if c:
        lines.append(f"      the frames' own rti_tick against the capture's rows: {c['rms_ms']:.3f} ms rms, mean "
                     f"{c['mean_ms']:+.3f}, worst {c['worst_ms']:.2f} ms ({c['frames']} frames)"
                     + ("" if c["rms_ms"] <= MAX_TICK_RMS_MS else
                        f"  — MORE than {MAX_TICK_RMS_MS:g} ms: the camera server's tick is off (the delay above "
                        f"does not use it)"))
    return lines


def verify_verdict(results: dict, applied_ms: float | None) -> tuple[bool, list]:
    """The variants' delays side by side: consistent when every one is confident and they agree within
    VERIFY_SPREAD_MS (and with the applied calibration); the lines saying so, and how the delay moves with J5's
    speed (a trend = mechanics, not timing)."""
    good = {k: r for k, r in results.items() if r.get("ok") and r.get("confident")}
    lines = []
    if len(good) < 2:
        return False, ["  VERIFY: fewer than two confident variants — no verdict"]
    d = np.array([r["delay_ms"] for r in good.values()])
    spread = float(d.max() - d.min())
    j5 = sorted((r["speed"], r["delay_ms"]) for r in good.values() if r["joint"] == 5)
    if len(j5) >= 2:
        sp, dl = np.array(j5).T
        trend = float(np.polyfit(sp, dl, 1)[0]) * (sp.max() - sp.min()) if len(j5) > 1 else 0.0
        lines.append(f"  J5's delay from {sp.min():g} to {sp.max():g} % speed changes by {trend:+.2f} ms (a pure "
                     f"timing delay does not change with the speed)")
    lines.append(f"  the variants' delays: {', '.join(f'{x:+.2f}' for x in d)} ms -> spread {spread:.2f} ms (mean "
                 f"{d.mean():+.2f})" + (f"; the calibration in use: {applied_ms:+.2f} ms" if applied_ms is not None
                                        else ""))
    readouts = [r["readout_ms"] for r in good.values()]
    lines.append(f"  the readout from each: {', '.join(f'{x:.1f}' for x in readouts)} ms")
    ok = (len(good) == len(results) and spread <= VERIFY_SPREAD_MS
          and (applied_ms is None or abs(d.mean() - applied_ms) <= VERIFY_SPREAD_MS))
    lines.append("  VERIFIED: the same delay from every motion" if ok else
                 f"  NOT verified: the variants differ by more than {VERIFY_SPREAD_MS:g} ms, from the calibration, or "
                 f"not all are confident")
    return ok, lines


def _verify_report(run_dir: str, run: dict, results: dict, lines: list) -> dict:
    active = run.get("timing_active") or {}
    ok, more = verify_verdict(results, None if active.get("auto_us") is None else active["auto_us"] / 1e3)
    lines += more + ["  (--verify writes nothing)"]
    text = "\n".join(lines)
    print(text)
    with open(os.path.join(run_dir, "report.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    summary = {"verify": True, "verified": ok, "results": {str(k): v for k, v in results.items()}}
    with open(os.path.join(run_dir, "result.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1, default=float)
    return dict(summary, confident=ok)


# ------------------------------------------------------------------------------------------------------- the arm
def camera_status() -> dict:
    from communication import camera_protocol as cp
    from data.shared_data import SharedString
    return SharedString(cp.STATUS_NAME, cp.STATUS_SIZE).get_json() or {}


def set_exposure(base: dict, setting) -> None:
    """The user's controls with `setting` ("auto", a fixed setting; None: as they were)."""
    if setting is None:
        cs.write(SYSTEM_XML, base)
        return
    pairs = [(k, v) for k, v in cs.parse_controls(base["controls"]) if k not in ("auto_exposure",
                                                                                 "exposure_time_absolute")]
    head = [("auto_exposure", 3)] if setting == "auto" else [("auto_exposure", 1), ("exposure_time_absolute", setting)]
    cs.write(SYSTEM_XML, {**base, "controls": cs.format_controls(head + pairs)})


def wait_exposure(setting, timeout=8.0) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        ctl = {c["key"]: c["value"] for c in camera_status().get("controls") or []}
        if (ctl.get("auto_exposure") not in (None, 1) if setting == "auto" else
                ctl.get("auto_exposure") == 1 and ctl.get("exposure_time_absolute") == setting):
            time.sleep(1.5)                                   # the camera settles (automatic exposure converges)
            return True
        time.sleep(0.2)
    return False


def precheck(host: str, mode_fps: float, seconds: float = 1.5) -> tuple[float, float]:
    """(frames per second by the camera's clock, the picture's brightness) right now."""
    from robotclient import CameraClient
    cam = CameraClient().connect(host)
    try:
        frames = [cam.read(timeout=3.0)]
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            frames.append(cam.read(timeout=3.0))
    except Exception:  # noqa: BLE001 — too few frames: the frame rate says so
        pass
    finally:
        cam.close()
    if len(frames) < 2:
        return 0.0, 0.0
    fps = (len(frames) - 1) / max(1e-6, frames[-1].exposure_time - frames[0].exposure_time)
    return fps, float(np.mean(decode_gray(frames[-1].jpeg)))


class Collector(threading.Thread):
    """Every frame from the camera server while it runs."""

    def __init__(self, host):
        super().__init__(daemon=True)
        from robotclient import CameraClient
        self.cam = CameraClient().connect(host)
        self.frames: list = []
        self.stop_flag = False
        self.error = None

    def run(self):
        try:
            for f in self.cam.frames(timeout=3.0):
                self.frames.append(f)
                if self.stop_flag:
                    break
        except Exception as e:  # noqa: BLE001 — reported after the run
            self.error = e

    def finish(self):
        self.stop_flag = True
        self.join(5.0)
        self.cam.close()
        return self.frames


def _capture_file(name, before, timeout=20.0):
    from project_paths import TEMP_DATA_DIR
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        new = set(glob.glob(os.path.join(str(TEMP_DATA_DIR), "captures", f"*_{name}.npz"))) - before
        if new:
            time.sleep(0.5)                       # the file is renamed into place when complete
            return sorted(new)[-1]
        time.sleep(0.2)
    return None


def checks(args) -> tuple[dict, list]:
    """The camera server's state; the problems that stop a run."""
    st = camera_status()
    problems = []
    if st.get("state") != "streaming":
        problems.append(f"the camera is not streaming ({st.get('state')}: {st.get('message')})")
    clk = st.get("clock") or {}
    if "calibration_why" not in clk:
        problems.append("the camera server is older than this script: restart it (tools/restart_gui_and_executor.sh "
                        "--with-camera)")
    elif clk.get("source") != "camera":
        problems.append(f"the frames do not carry the camera's own clock ({clk.get('why')})")
    return st, problems


def record_setting(robot, args, setting, q0, out_dir, run, mode_fps, joint=None, amplitude=None, speed=None,
                   tag=None) -> None:
    """One recording at an exposure: check it, swing (args' joint / amplitude / speed unless given), keep the frames
    and the capture under `tag` (default: the setting)."""
    from project_paths import TEMP_DATA_DIR
    joint = args.joint if joint is None else joint
    amplitude = args.amplitude if amplitude is None else amplitude
    speed = args.swing_speed if speed is None else speed
    tag = str(setting) if tag is None else tag
    fps, bright = precheck(args.host, mode_fps)
    if fps < MIN_FPS_SHARE * mode_fps:
        print(f"{setting_name(setting)}: the camera runs at {fps:.1f} fps, not {mode_fps:g} (a fixed exposure longer "
              f"than a frame, {1000 / mode_fps:.1f} ms, slows it) — skipped")
        return
    if bright < MIN_BRIGHTNESS:
        print(f"{setting_name(setting)}: the picture is too dark ({bright:.1f}/255; this camera drops its gain at a "
              f"fixed exposure — more light, or automatic exposure) — skipped")
        return
    name = f"cam80_{tag}"
    before = set(glob.glob(os.path.join(str(TEMP_DATA_DIR), "captures", f"*_{name}.npz")))
    # long enough for a slow swing too (stop_capture ends it; at 20 % a capture sized for 40 % ended 0.4 s before
    # the frames, 2026-11-24); the recorder takes 60 s at most
    robot.capture_telemetry(min(60.0, (args.swings * 2.5 + 6.0) * max(1.0, SWING_SPEED / speed)), name)
    col = Collector(args.host)
    col.start()
    time.sleep(0.7)
    j = joint - 1
    for i in range(args.swings):
        q = list(q0)
        q[j] = q0[j] + (amplitude if i % 2 == 0 else -amplitude)
        robot.move_joints(q, speed=speed, timeout=args.timeout)
        robot.wait_idle(timeout=args.timeout)
    robot.move_joints(q0, speed=speed, timeout=args.timeout)
    robot.wait_idle(timeout=args.timeout)
    time.sleep(0.7)
    frames = col.finish()
    robot.stop_capture()
    cap = _capture_file(name, before)
    if cap is None or col.error is not None or len(frames) < MIN_PAIRS:
        print(f"{setting_name(setting)}: no usable recording (capture {cap}, {len(frames)} frames, {col.error})")
        return
    shutil.copy2(cap, os.path.join(out_dir, f"capture_{tag}.npz"))
    save_frames(os.path.join(out_dir, f"frames_{tag}.npz"), frames)
    run["exposures"].append({"setting": setting, "frames": f"frames_{tag}.npz", "capture": f"capture_{tag}.npz",
                             "n_frames": len(frames), "tag": tag, "joint": joint, "amplitude_deg": amplitude,
                             "speed": speed})
    print(f"{setting_name(setting)}, J{joint} +-{amplitude:g} deg at {speed:g} %: {len(frames)} frames at {fps:.1f} fps, "
          f"brightness {bright:.0f}/255, capture {os.path.basename(cap)}", flush=True)


def run_arm(args) -> str | None:
    from _common import connect, finish, goto_start, prepare_exec
    from project_paths import TEMP_DATA_DIR
    from robotclient import CameraClient
    st, problems = checks(args)
    if problems:
        print("NOT READY:\n  " + "\n  ".join(problems))
        return None
    base, _ = cs.read(SYSTEM_XML)
    mode = cs.mode_key(st["mode"]["width"], st["mode"]["height"], st["mode"]["fps"])
    mode_fps = float(st["mode"]["fps"])
    usb_id = (st.get("device") or {}).get("usb_id", "")
    gcam, _ = cs.gripper_camera(SYSTEM_XML, GRIPPERS_DIR)
    out_dir = os.path.join(str(TEMP_DATA_DIR), "camera_timing", time.strftime("%Y%m%d_%H%M%S", time.gmtime()))
    os.makedirs(out_dir, exist_ok=True)
    run = {"joint": args.joint, "amplitude_deg": args.amplitude, "swing_speed": args.swing_speed, "swings": args.swings,
           "mode": mode, "usb_id": usb_id, "controls_before": base["controls"],
           "timing_active": cs.camera_timing(gcam, mode), "exposures": []}
    robot = connect(args)
    try:
        prepare_exec(robot, args)
        goto_start(robot, args)
        q0 = list(robot.get_joints())
        run["start_deg"] = q0
        cam = CameraClient().connect(args.host)
        first = cam.read()
        cam.close()
        tex = texture(decode_gray(first.jpeg))
        print(f"the scene's texture {tex:.1f} (needs {MIN_TEXTURE})")
        if tex < MIN_TEXTURE:
            print("The camera sees too plain a scene: point it at something with detail (shelves, a cluttered desk) "
                  "with --no-start-pose, or place something in front of it.")
            return None
        try:
            if args.verify:
                run["verify"] = True
                set_exposure(base, "auto")
                if wait_exposure("auto"):
                    for joint, amplitude, speed in VERIFY_VARIANTS:
                        record_setting(robot, args, "auto", q0, out_dir, run, mode_fps, joint=joint,
                                       amplitude=amplitude, speed=speed, tag=f"auto_J{joint}_{speed:g}")
            for setting in ([] if args.verify else args.exposures):
                set_exposure(base, setting)
                if not wait_exposure(setting):
                    print(f"{setting_name(setting)}: the camera did not take it within 8 s — skipped")
                    continue
                record_setting(robot, args, setting, q0, out_dir, run, mode_fps)
        finally:
            set_exposure(base, None)
            print("the camera's settings put back:", base["controls"])
        with open(os.path.join(out_dir, "run.json"), "w", encoding="utf-8") as f:
            json.dump(run, f, indent=1)
        if not run["exposures"]:
            print("nothing recorded")
            return None
        print(f"\nRecorded: {os.path.relpath(out_dir, _ROOT)}. Disable the arm now (the dashboard's Disable): the "
              f"analysis decodes ~{sum(e['n_frames'] for e in run['exposures'])} frames and starts once it is "
              f"disabled. (Ctrl-C: later, with --reanalyse {os.path.relpath(out_dir, _ROOT)})", flush=True)
        end = time.monotonic() + DISABLE_WAIT_S
        while time.monotonic() < end and robot.get_status().get("state") == "ENABLED":
            time.sleep(0.5)
        if robot.get_status().get("state") == "ENABLED":
            print(f"still enabled after {DISABLE_WAIT_S:.0f} s: analyse later with --reanalyse")
            return None
    finally:
        finish(robot)
    return out_dir


# ------------------------------------------------------------------------------------------------------- selftest
def selftest() -> int:
    """The analysis on synthetic frames whose true delay is known, the summary, the XML writer."""
    from PIL import Image
    fails = []

    def chk(name, cond, detail=""):
        print(("  PASS  " if cond else "  FAIL  ") + name + (f"   [{detail}]" if detail and not cond else ""))
        if not cond:
            fails.append(name)

    rng = np.random.default_rng(3)
    size = (320, 180)                                         # the synthetic frames' size (kept small: rendering)
    h, w = size[1], size[0]
    k = np.fft.fftfreq(h)[:, None] ** 2 + np.fft.rfftfreq(w)[None, :] ** 2
    tex_f = np.fft.rfft2(rng.normal(size=(h, w))) * np.exp(-k / 0.004)   # a smooth random scene, periodic
    fy = np.fft.fftfreq(h)[:, None]

    def render(dy):                                           # the scene shifted dy rows (J5: up / down)
        return np.fft.irfft2(tex_f * np.exp(-2j * np.pi * fy * dy), s=(h, w))

    tick_s = 0.004
    ticks = np.arange(0, 3500, dtype=float)                   # 14 s
    t = ticks * tick_s
    theta = 0.14 * np.sin(2 * np.pi * t / 1.7) + 0.05 * np.sin(2 * np.pi * t / 0.61)   # rad
    px_per_rad = 160.0

    jaws = rng.normal(size=(h - 120, 160))                    # a static block: the gripper's jaws, bottom centre (the
                                                              # 2 x 2 tiles there: rows 4-5, columns 1-2)

    def synth(delay_s, exposure_s, n=300, readout_s=0.030, bands=15):
        """Frames whose centre row shows the joint at delay_s after the frame time; each row band read out
        readout_s x (its row - the centre) / rows later (the rolling shutter); blur over the exposure; the jaws."""
        tf = 0.3 + np.arange(n) / 30.0 + rng.normal(0, 5e-6, n)
        jpegs = []
        bh = h // bands
        for tk in tf:
            img = np.empty((h, w))
            for b in range(bands):
                t_row = tk + delay_s + ((b + 0.5) * bh - h / 2) / h * readout_s
                subs = np.linspace(-exposure_s / 2, exposure_s / 2, 3)        # motion blur over the exposure
                img[b * bh:(b + 1) * bh] = np.mean([render(px_per_rad * np.interp((t_row + s) / tick_s, ticks, theta)
                                                           )[b * bh:(b + 1) * bh] for s in subs], axis=0)
            img = 40 * img / img.std()
            img[120:, 80:240] = 30 * jaws                      # moves WITH the camera: never shifts
            img = np.clip(128 + img + rng.normal(0, 2, img.shape), 0, 255).astype(np.uint8)
            buf = io.BytesIO()
            Image.fromarray(img).save(buf, "JPEG", quality=85)
            jpegs.append(buf.getvalue())
        return {"exposure_time": tf, "rti_tick": tf / tick_s, "flags": np.full(n, 0x09), "pts": np.zeros(n, int),
                "frame": np.arange(n), "jpegs": jpegs}

    cap = {"tick": ticks, "timestamp": t, "j_p": np.repeat(theta[:, None], 6, axis=1)}
    def analyse(frames, **kw):
        return analyse_exposure(frames, cap, JOINT, size=size, full_rows=h, **kw)

    print("A. the delay on synthetic frames (a smooth scene, J5's motion, a rolling shutter, motion blur, the gripper's "
          "jaws in view, JPEG, noise)")
    for true_ms, exp_ms, ro_ms in ((7.3, 10.0, 30.0), (-4.1, 33.3, 25.0)):
        r = analyse(synth(true_ms * 1e-3, exp_ms * 1e-3, readout_s=ro_ms * 1e-3))
        jaw_tiles = [t for t in r.get("tiles", []) if t["tile"][0] >= 4 and t["tile"][1] in (1, 2)]
        chk(f"A1 a true centre-row delay of {true_ms:+.1f} ms, a {ro_ms:g} ms readout ({exp_ms:g} ms exposure) found "
            f"within 0.3 / 1.5 ms, confident: {r.get('delay_ms', float('nan')):+.2f} ms, readout "
            f"{r.get('readout_ms', float('nan')):.2f} ms, {r.get('tiles_used')} tiles, halves "
            f"{r.get('half_diff_ms', float('nan')):.2f} ms",
            r["ok"] and abs(r["delay_ms"] - true_ms) < 0.3 and abs(r["readout_ms"] - ro_ms) < 1.5 and r["confident"], r)
        chk("A1b ...the tiles on the jaws (they move with the camera) left out",
            jaw_tiles and all(t["r2"] < MIN_TILE_R2 for t in jaw_tiles), jaw_tiles)
    fr = synth(5.0e-3, 10e-3)
    fr["flags"] = np.full(len(fr["flags"]), 0x0B)              # recorded with a calibration of +2 ms active: the
    fr["exposure_time"] = fr["exposure_time"] + 2e-3           # server adds it to the frame's time and takes the
    fr["rti_tick"] = fr["rti_tick"] + 2e-3 / tick_s            # tick there
    r = analyse(fr, uncal_offset_s=2e-3)
    chk("A2 a calibration active while recording is taken back off", r["ok"] and abs(r["delay_ms"] - 5.0) < 0.3, r)
    chk("A2b ...the headers' ticks agree with the capture's rows: the tick check says ~0",
        r["ok"] and r["tick_check"]["frames"] == r["frames"] and r["tick_check"]["rms_ms"] < 1e-6, r.get("tick_check"))
    bad = dict(fr, rti_tick=fr["rti_tick"] + rng.normal(0, 2e-3 / tick_s, len(fr["rti_tick"])))
    bad["rti_tick"][::7] = np.nan
    r_bad = analyse(bad, uncal_offset_s=2e-3)
    chk("A4 headers whose rti_tick is 2 ms rms off, one in seven without a tick: the SAME delay (a frame's tick comes "
        "from its time and the capture's rows), and the tick check reports the 2 ms",
        r_bad["ok"] and abs(r_bad["delay_ms"] - r["delay_ms"]) < 1e-9 and r_bad["frames"] == r["frames"]
        and 1.5 < r_bad["tick_check"]["rms_ms"] < 2.5 and r_bad["tick_check"]["frames"] < r["frames"]
        and any("MORE than" in ln for ln in tick_lines(r_bad)) and not any("MORE than" in ln for ln in tick_lines(r)),
        (r_bad.get("delay_ms"), r.get("delay_ms"), r_bad.get("tick_check")))
    n_cap = 2000                                               # the capture ends at 8 s, the frames go on to 10.3 s
    short = {"tick": ticks[:n_cap], "timestamp": t[:n_cap], "j_p": cap["j_p"][:n_cap]}
    r_short = analyse_exposure(fr, short, JOINT, 2e-3, size=size, full_rows=h)
    want_out = int(np.sum(fr["exposure_time"] - 2e-3 > t[n_cap - 1] - SEARCH_TICKS * tick_s))
    chk(f"A5 a capture that ends before the frames do: the {want_out} frames at or past its end left out and counted, "
        f"the delay from the rest",
        r_short["ok"] and r_short["frames_outside"] == want_out and r_short["frames"] == len(fr["jpegs"]) - want_out
        and abs(r_short["delay_ms"] - 5.0) < 0.4 and any("left out" in ln for ln in tick_lines(r_short)),
        (r_short.get("frames_outside"), want_out, r_short.get("delay_ms"), r_short.get("why")))
    flat = synth(3e-3, 10e-3, n=200)
    flat["flags"] = np.zeros(200, int)
    chk("A3 frames without the camera's clock: no result, said why", not analyse(flat)["ok"])
    late = dict(fr, exposure_time=fr["exposure_time"] + 60.0)
    r_late = analyse(late, uncal_offset_s=2e-3)
    chk("A6 frames all past the capture: no result, said why",
        not r_late["ok"] and "inside the capture" in r_late["why"], r_late)
    print("B. the summary")
    good = {"ok": True, "confident": True}
    good = dict(good, readout_ms=30.8)
    vals, _, conf = summarise({}, {"auto": dict(good, delay_ms=12.34)})
    chk("B1 automatic exposure alone: its offset and the readout written (us), confident",
        conf and vals == {"timing_auto_us": "12340.0", "timing_readout_us": "30800"}, vals)
    fixed = {s: dict(good, delay_ms=3.0 + 0.05 * s) for s in (100, 200, 300)}
    vals, lines, conf = summarise({}, dict(fixed, auto=dict(good, delay_ms=9.0)))
    chk("B2 + fixed settings on a +50 us / unit line: the line written, the PTS at the exposure's start",
        conf and vals["timing_offset_us"] == "3000.0" and vals["timing_us_per_unit"] == "50.0000"
        and "timing_auto_us" in vals and any("exposure's start" in ln for ln in lines), (vals, lines))
    line = exposure_line({s: dict(good, delay_ms=3.0 - 0.05 * s) for s in (100, 200, 300)})
    chk("B3 ...falling: the PTS at the exposure's end", "end" in line["pts_marks"] and line["unit_error_us"] < 1e-6)
    vals, _, conf = summarise({}, {s: dict(good, delay_ms=3.0 + 0.01 * s) for s in (100, 200, 300)})
    chk("B4 a slope far from +-50 us per unit: not confident, nothing of the line written",
        not conf and "timing_offset_us" not in vals, vals)
    vals, _, conf = summarise({}, {"auto": {"ok": False, "why": "dark"}})
    chk("B5 nothing confident: nothing to write", not conf and not vals)
    v = {"J5_20": dict(good, joint=5, speed=20.0, delay_ms=1.30, readout_ms=31.8),
         "J5_60": dict(good, joint=5, speed=60.0, delay_ms=1.62, readout_ms=31.7),
         "J6_40": dict(good, joint=6, speed=40.0, delay_ms=1.48, readout_ms=31.9)}
    ok_v, lines_v = verify_verdict(v, 1.41)
    chk("B6 --verify: three motions within 1 ms of each other and of the calibration: verified", ok_v, lines_v)
    v2 = dict(v, J5_20=dict(v["J5_20"], delay_ms=0.40), J5_60=dict(v["J5_60"], delay_ms=2.40))
    ok_v2, lines_v2 = verify_verdict(v2, 1.41)
    chk("B7 ...a delay that moves 2 ms with J5's speed (not a pure timing delay): not verified, the trend shown",
        not ok_v2 and any("+2.00 ms" in ln for ln in lines_v2), lines_v2)
    ok_v3, _ = verify_verdict(dict(v, J6_40={"ok": False, "why": "dark"}), 1.41)
    chk("B8 ...a variant without a confident delay: not verified", not ok_v3)
    print("C. --apply's writer")
    import tempfile
    tmp = tempfile.mkdtemp(prefix="cam80_")
    try:
        dst = os.path.join(tmp, "MSG.xml")
        shutil.copy2(os.path.join(GRIPPERS_DIR, "MSG_small_motor_150mm_rail.xml"), dst)
        before = open(dst, encoding="utf-8").read()
        ok = write_timing(dst, "0bda:5844", "1280x720@30", {"timing_auto_us": "12340.0"}, "RUN1",
                          os.path.join(tmp, "backup"))
        after = open(dst, encoding="utf-8").read()
        root = ET.parse(dst).getroot()  # nosec B314
        timing = cs.camera_timing(dict(root.find("camera").attrib), "1280x720@30")
        chk("C1 written: the camera's attributes kept, the automatic offset added, every comment kept, a backup",
            ok and timing is not None and timing["auto_us"] == 12340.0 and timing["offset_us"] is None
            and root.find("camera").attrib["usb_id"] == "0bda:5844" and after.count("<!--") == before.count("<!--")
            and os.path.exists(os.path.join(tmp, "backup", "MSG.xml")), timing)
        write_timing(dst, "0bda:5844", "1280x720@30", {"timing_offset_us": "3000.0", "timing_us_per_unit": "50.0"},
                     "RUN2", os.path.join(tmp, "backup2"))
        two = cs.camera_timing(dict(ET.parse(dst).getroot().find("camera").attrib), "1280x720@30")  # nosec B314
        again = open(dst, encoding="utf-8").read()
        chk("C2 a later run of the other kind for the same mode: both kept, the run replaced, each key once",
            two["auto_us"] == 12340.0 and two["offset_us"] == 3000.0 and two["run"] == "RUN2"
            and again.count("timing_mode=") == 1, two)
        write_timing(dst, "0bda:5844", "640x480@30", {"timing_auto_us": "1.0"}, "RUN3", os.path.join(tmp, "b3"))
        three = dict(ET.parse(dst).getroot().find("camera").attrib)  # nosec B314
        chk("C3 another mode: the old mode's values go", "timing_offset_us" not in three and three["timing_mode"]
            == "640x480@30", three)
        chk("C4 another camera's file is not touched", not write_timing(dst, "046d:0825", "1280x720@30",
                                                                        {"timing_auto_us": "1"}, "X",
                                                                        os.path.join(tmp, "b4")))
        chk("C5 the server's formula: automatic -> its offset, fixed -> offset + k x setting, missing -> None",
            abs(cs.timing_offset_s(two, None) - 12.34e-3) < 1e-12
            and abs(cs.timing_offset_s(two, 200) - (3000.0 + 50.0 * 200) * 1e-6) < 1e-12
            and cs.timing_offset_s(timing, 200) is None and cs.camera_timing({"timing_mode": "1280x720@30"},
                                                                             "1280x720@30") is None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    print()
    print(f"{len(fails)} failed" if fails else "ALL PASSED")
    return 1 if fails else 0


# ------------------------------------------------------------------------------------------------------- main
def main():
    from _common import parser
    ap = parser(__doc__.split("\n")[0])
    ap.add_argument("--joint", type=int, default=JOINT, help="the joint that swings (default J5)")
    ap.add_argument("--amplitude", type=float, default=AMPLITUDE_DEG, help="degrees each way")
    ap.add_argument("--swing-speed", type=float, default=SWING_SPEED, help="percent of the joint's limits")
    ap.add_argument("--swings", type=int, default=SWINGS, help="moves per exposure")
    ap.add_argument("--exposures", default=",".join(map(str, EXPOSURES)),
                    help="auto and / or fixed settings (exposure_time_absolute, 100 us each), comma-separated")
    ap.add_argument("--reanalyse", metavar="FOLDER")
    ap.add_argument("--apply", action="store_true", help="write a confident result into the gripper XMLs")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--verify", action="store_true",
                    help="record J5 at 20 / 40 / 60 %% and J6 +-15 deg under automatic exposure; compare; write nothing")
    args = ap.parse_args()
    args.exposures = [parse_setting(x) for x in str(args.exposures).split(",") if x.strip()]
    if args.verify and args.apply:
        sys.exit("--verify writes nothing: leave --apply out")
    if args.selftest:
        sys.exit(selftest())
    if args.reanalyse:
        summary = reanalyse(os.path.join(_ROOT, args.reanalyse) if not os.path.isabs(args.reanalyse)
                            else args.reanalyse, args.apply)
        sys.exit(0 if summary.get("confident") else 1)
    st, problems = checks(args)
    print(f"camera: {st.get('state')}, {(st.get('device') or {}).get('usb_id')}, mode {st.get('mode')}; frame times: "
          f"{(st.get('clock') or {}).get('source')}")
    if args.verify:
        print(f"plan (--verify): the start pose, then under automatic exposure "
              f"{'; '.join(f'J{j} +-{a:g} deg at {v:g} %' for j, a, v in VERIFY_VARIANTS)}, {args.swings} moves each "
              f"(~{len(VERIFY_VARIANTS) * (args.swings * 2.5 + 8):.0f} s); compared, nothing written; the analysis once "
              f"the arm is disabled")
    else:
        print(f"plan: the start pose, then J{args.joint} +-{args.amplitude:g} deg x {args.swings} at {args.swing_speed:g} % "
              f"for each of {', '.join(setting_name(s) for s in args.exposures)} "
              f"(~{len(args.exposures) * (args.swings * 2.5 + 10):.0f} s); the camera's settings put back after; the "
              f"analysis once the arm is disabled")
    if problems:
        print("NOT READY:\n  " + "\n  ".join(problems))
        sys.exit(2)
    if not args.go:
        print("DRY RUN: nothing moved. The camera must see a still scene with some detail; clear the space around the "
              "wrist; then re-run with --go.")
        return
    out_dir = run_arm(args)
    if out_dir:
        summary = reanalyse(out_dir, args.apply)
        sys.exit(0 if summary.get("confident") else 1)
    sys.exit(1)


if __name__ == "__main__":
    from _common import main_guard
    main_guard(main)
