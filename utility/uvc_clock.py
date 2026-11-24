"""The camera's own clock on the robot's clock (2026-11-24, TODO/camera_plan.md P2).

A UVC camera stamps every frame with its PTS (presentation time stamp: when it began the frame, in its own clock —
15 MHz on the MSG camera) and every packet header with an SCR (its clock, sampled at the USB microframe the packet
went out in). For each header the driver records CLOCK_MONOTONIC and the host's USB frame number at the moment it
read it, which is when the USB transfer that carried the packet completed (hardware/uvc_camera.py: the metadata
node, parse_meta). Linux 6.8 does not turn the PTS into a timestamp itself (uvcvideo hwtimestamps=0); this does:

  1. Pairs. A block that is the LAST of its transfer (the next block completed later) and that the next block follows
     by exactly one microframe of the camera's clock (no idle microframe between: the transfer ended with it) gives
     the pair (its STC, its ns): the STC was sampled at the start of its microframe, the transfer completed at the
     microframe's end, and the driver read the time right then — plus a constant latency (the interrupt and the
     driver, tens of us). Measured on the MSG camera (2026-11-24, 2338 transfers): +-8 us around the fit, the worst
     31 us. (The plain estimate per packet — ns minus the USB frames between the host's and the camera's frame
     numbers — scatters +-0.5 ms and carries a bias of up to 1 ms that changes with every start of the stream.)
  2. A straight line through the pairs of the last WINDOW_S seconds, CLOCK_MONOTONIC = a + b * STC (b: the camera's
     clock period; 14.99918 MHz measured, 55 ppm slow against the Pi's clock), refitted every REFIT_S; late pairs (a
     delayed interrupt) are left out of a second fit.
  3. time_of(pts): the line at the PTS, minus one microframe (the pairs mark a microframe's END, the STC its start).

What stays unknown is a constant: that latency, and where in the exposure the camera puts its PTS. P3's calibration
measures it. On the recorded capture (dummy_test_scripts/fixtures/uvc_meta_20261124.npz) the frame intervals came out
at 33.3295 ms +- 2 us (the driver's timestamps: 32 or 36 ms, +-1.9 ms), and the driver's timestamp 1.2-5.1 ms after
the PTS (the frame's first packet leaves ~1.1 ms after it, then up to one 4 ms transfer).

Vectorised (numpy): ~0.07 ms per metadata buffer (242 blocks) on the Pi 5, plus a refit a second (~1 ms).
"""
from __future__ import annotations

import numpy as np

WINDOW_S = 4.0                    # the pairs fitted (~230 a second on the MSG camera)
REFIT_S = 1.0                      # the camera's clock drifts by ppm: a line a second is plenty
MIN_PAIRS = 40                    # fewer: no camera time yet (~0.2 s after the stream starts)
SAME_TRANSFER_NS = 300_000        # blocks read within this of each other came in one transfer (they are ~1 us apart;
                                  # transfers 1-4 ms)
STEP_MIN = 20                     # STC steps inside transfers needed to know one microframe (a buffer has ~240)
STEP_EVERY = 300                  # ... re-checked every this many buffers (~10 s)
OUTLIER_S = 50e-6                 # a pair further than this (or 5 robust sigmas) from the median residual is left out
MAX_RESIDUAL_S = 200e-6           # a fit scattering more is not used (the camera's clock jumped, or no clean pairs)
UVC_PTS, UVC_SCR = 0x04, 0x08
_U32, _HALF = 1 << 32, 1 << 31
# A block with PTS and SCR (struct uvc_meta_buf + the header's PTS and SCR): every block of the MSG camera.
_FULL = np.dtype([("ns", "<u8"), ("sof", "<u2"), ("length", "u1"), ("flags", "u1"), ("pts", "<u4"), ("stc", "<u4"),
                  ("dsof", "<u2")])
assert _FULL.itemsize == 22


def blocks_of(data) -> tuple[np.ndarray, np.ndarray, int | None]:
    """(ns, STC, the frame's PTS or None) of one metadata buffer's blocks that carry an SCR, in arrival order: the
    numpy reader when every block carries PTS and SCR, hardware/uvc_camera.parse_meta otherwise."""
    data = bytes(data)
    n = len(data)
    if n and n % 22 == 0:
        a = np.frombuffer(data, dtype=_FULL)
        if np.all(a["flags"] & (UVC_PTS | UVC_SCR) == UVC_PTS | UVC_SCR):
            return a["ns"].astype(np.int64), a["stc"].astype(np.int64), int(a["pts"][0])
    from hardware.uvc_camera import parse_meta
    blocks = parse_meta(data)
    pts = next((b[3] for b in blocks if b[3] is not None), None)
    with_scr = [(b[0], b[4]) for b in blocks if b[4] is not None]
    if not with_scr:
        return np.zeros(0, np.int64), np.zeros(0, np.int64), pts
    arr = np.array(with_scr, dtype=np.int64)
    return arr[:, 0], arr[:, 1], pts


class CameraClock:
    """The camera's clock -> CLOCK_MONOTONIC for one stream (reset() at every open of the camera)."""

    def __init__(self, window_s: float = WINDOW_S):
        self.window_s = window_s
        self.reset()

    def reset(self) -> None:
        self.px = np.zeros(0, np.int64)        # the pairs: STC unwrapped ...
        self.pt = np.zeros(0)                  # ... and its microframe's end, CLOCK_MONOTONIC s
        self.last_raw: int | None = None
        self.unw = 0                           # the last STC, unwrapped
        self.prev: tuple | None = None         # (ns, STC unwrapped) of the previous block
        self.buffers = 0                       # metadata buffers taken
        self.step: int | None = None           # one microframe in the camera's clock (1875 at 15 MHz)
        self.fit: tuple | None = None          # (STC at the reference, time there, seconds per tick)
        self.residual: float | None = None
        self.rejected = 0                      # pairs the last fit left out
        self.restarts = 0                      # fits thrown away (the clock jumped)
        self.last_fit_t: float | None = None   # the newest pair's time at the last fit
        self.fit_span = 0.0                    # the pairs' time span at the last fit
        self.why = "no camera clock pairs yet"

    def add(self, data) -> int | None:
        """Take one metadata buffer (its bytes); return its frame's PTS (raw, 32 bits), or None when it has none."""
        ns, stc, pts = blocks_of(data)
        if not len(stc):
            self.prev = None
            return pts
        d = np.empty_like(stc)
        d[0] = 0 if self.last_raw is None else stc[0] - self.last_raw
        d[1:] = stc[1:] - stc[:-1]
        d = (d + _HALF) % _U32 - _HALF                         # each step as a signed 32-bit difference
        unw_all = (self.unw if self.last_raw is not None else int(stc[0])) + np.cumsum(d)
        ns_all = ns
        if self.prev is not None:
            pns = np.concatenate(([self.prev[0]], ns_all[:-1]))
            px = np.concatenate(([self.prev[1]], unw_all[:-1]))
            unw = unw_all
        else:                                                  # the first block has no predecessor
            pns, px, ns, unw = ns_all[:-1], unw_all[:-1], ns_all[1:], unw_all[1:]
        dx = unw - px
        same = (ns - pns) < SAME_TRANSFER_NS
        self.buffers += 1
        if self.step is None or self.buffers % STEP_EVERY == 0:        # one microframe: found once, then re-checked
            inside = dx[same & (dx > 0)]
            if len(inside) >= STEP_MIN:
                self.step = int(np.median(inside))
        if self.step:
            ends = ~same & (np.abs(dx - self.step) <= max(2, self.step // 500))
            if ends.any():
                self.px = np.concatenate((self.px, px[ends]))
                self.pt = np.concatenate((self.pt, pns[ends] * 1e-9))
        self.prev = (int(ns_all[-1]), int(unw_all[-1]))
        self.unw = int(unw_all[-1])
        self.last_raw = int(stc[-1])
        return pts

    def refit(self) -> None:
        if len(self.pt):
            keep_from = int(np.searchsorted(self.pt, self.pt[-1] - self.window_s))
            self.px, self.pt = self.px[keep_from:], self.pt[keep_from:]
        n = len(self.pt)
        if n < MIN_PAIRS:
            self.fit = None
            self.why = f"{n} clock pairs so far (needs {MIN_PAIRS})"
            return
        x0, t0 = int(self.px[-1]), float(self.pt[-1])
        x = (self.px - x0).astype(float)
        t = self.pt - t0
        b, a = _line(x, t)
        keep = np.ones(n, bool)
        for _ in range(2):            # late pairs out: by their distance from the MEDIAN residual (a delayed interrupt
            r = t - (a + b * x)       # is only ever late, so it pulls the plain line up; the median stays with the rest)
            med = float(np.median(r[keep]))
            mad = float(np.median(np.abs(r[keep] - med)))
            inliers = np.abs(r - med) <= max(OUTLIER_S, 5.0 * 1.4826 * mad)
            if inliers.sum() < max(MIN_PAIRS // 2, n // 2):     # never fewer than half the pairs
                break
            keep = inliers
            b, a = _line(x[keep], t[keep])
        r = t - (a + b * x)
        self.rejected = int(n - keep.sum())
        self.residual = float(np.std(r[keep]))
        self.last_fit_t = t0
        self.fit_span = float(self.pt[-1] - self.pt[0])
        if not (b > 0 and 1e3 <= 1.0 / b <= 1e9) or self.residual > MAX_RESIDUAL_S:
            self.why = (f"the camera's clock does not fit the robot's (scatter {self.residual * 1e6:.0f} us, "
                        f"{1.0 / b / 1e6 if b > 0 else 0:.3f} MHz): starting over")
            self.fit = None
            self.px, self.pt = self.px[:0], self.pt[:0]
            self.restarts += 1
            return
        self.fit = (x0, t0 + a, b)
        self.why = ""

    def time_of(self, pts: int) -> float | None:
        """CLOCK_MONOTONIC (s) of the camera's timestamp `pts` (its raw 32-bit value, from the frame being read now),
        or None while the clock is not known."""
        if len(self.pt) and self._refit_due():
            self.refit()
        if self.fit is None or self.last_raw is None or not self.step:
            return None
        x0, t0, b = self.fit
        x = self.unw + ((pts - self.last_raw + _HALF) % _U32 - _HALF) - self.step
        return t0 + b * (x - x0)

    def _refit_due(self) -> bool:
        """A new line: none yet; at every new pair while the window is still filling (a line through a short span has
        an uncertain slope — used for a whole REFIT_S it put up to 450 us into the first second, found by the net's
        synthetic streams 2026-11-24); then every REFIT_S."""
        if self.fit is None or self.last_fit_t is None:
            return True
        every = REFIT_S if self.fit_span >= 0.9 * self.window_s else 0.0
        return float(self.pt[-1]) - self.last_fit_t > every

    def health(self) -> dict:
        """For the status: ok, why not, the pairs, the fit's scatter (us), the camera's clock (MHz)."""
        f = self.fit
        return {"ok": f is not None, "why": self.why, "pairs": int(len(self.pt)),
                "fit_us": round(self.residual * 1e6, 1) if f is not None and self.residual is not None else None,
                "device_mhz": round(1.0 / f[2] / 1e6, 6) if f is not None else None,
                "step": self.step, "rejected": self.rejected, "restarts": self.restarts}


def _line(x: np.ndarray, t: np.ndarray) -> tuple[float, float]:
    """(slope, intercept) of the least-squares line t = intercept + slope * x."""
    xm, tm = float(x.mean()), float(t.mean())
    dx = x - xm
    sxx = float(dx @ dx)
    b = float(dx @ (t - tm)) / sxx if sxx > 0 else 0.0
    return b, tm - b * xm
