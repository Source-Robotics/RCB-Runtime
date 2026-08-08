"""Selectable per-signal filters for the RT loop.

Two-call design, no classes (per CLAUDE.md's preference):

    f = create_filter("ema", joint_num, dt, tau=0.0246)   # STARTUP — allocates
    apply_filter(f, x, out)                               # EVERY TICK — allocates nothing

`create_filter` returns a plain dict holding every buffer the filter will ever need.
`apply_filter` only ever writes into those buffers and into `out`, so it is safe on the
4 ms hot path (see CLAUDE.md "Hot-path allocation discipline").

WHY A TIME CONSTANT AND NOT ALPHA
---------------------------------
alpha is a PER-TICK coefficient, so its meaning depends on delta_t. A single hardcoded
alpha=0.15 means a 26.7 ms filter on PAR6 (250 Hz) and a 66.7 ms filter on INDIGO/SKAR
(100 Hz) — the same number, two different filters. Config therefore specifies `tau` in
SECONDS and alpha is derived per robot: alpha = 1 - exp(-dt/tau). For reference, the
alpha=0.15 hardcoded before this module existed is tau = 24.6 ms at 250 Hz.

CHOOSING A FILTER — the rule that actually matters
--------------------------------------------------
Each differentiation multiplies high-frequency noise by ~2/dt (+20 dB/decade). Each
1-pole filter stage removes -20 dB/decade. So a signal needs AS MANY POLES AS IT HAS
DIFFERENTIATIONS, and no amount of tuning substitutes for a missing pole:

    joint_speed  0 diffs (measured on CAN)  -> "none" or 1 pole
    joint_accel  1 diff                     -> 1 pole  (ema / sma / butter2 / savgol)
    joint_jerk   2 diffs                    -> 2 poles (ema2 / butter2 / savgol)

Measured on a simulated 0.5 Hz sweep: jerk through a single ema is 92% error at ANY
alpha (best case 31% at alpha=0.04, and it gets WORSE if you tighten further — a 1-pole
roll-off simply cannot catch a 2-differentiation noise climb). Two poles: 15%.

    kind      poles / phase        state    notes
    ------    -----------------    -----    ---------------------------------------
    none      -                    0        passthrough. Also the honest choice when
                                            the signal is already clean.
    ema       1 pole,  -20 dB/dec  1        cheapest. What this project used before.
    ema2      2 poles, -40 dB/dec  2        cascaded ema. The jerk fix.
    dema      lag-reduced, ~1 pole 2        2*EMA - EMA(EMA). NOT ema2 — see below.
    sma       FIR, LINEAR phase    N        constant, correctable lag.
    butter2   2 poles, -40 dB/dec  4        flatter passband than ema2 for equal lag.
    savgol    FIR, LINEAR phase    N        fits a polynomial; built for derivatives.
    median    non-linear           N        the ONLY one that removes a lone encoder
                                            spike instead of smearing it.

⚠️ dema IS NOT ema2. They are near-opposites. `dema = 2*EMA(x) - EMA(EMA(x))` exists to
   REDUCE lag, and pays for it with overshoot and MORE high-frequency gain. It still
   rolls off at roughly -20 dB/dec, so it will NOT fix jerk. Reach for dema when lag
   hurts you; reach for ema2/butter2/savgol when noise does.

⚠️ ORDER OF FILTER AND DERIVATIVE IS IRRELEVANT. Every linear filter here and the
   backward difference are both LTI, so they commute: d(EMA(x)) and EMA(d(x)) agree to
   ~1e-13. Only the NUMBER of poles matters. (`median` is non-linear and does NOT
   commute — order matters for that one alone.)
"""

import math

import numpy as np

# Kinds that are valid in a <filters> XML block. "none" is always available.
FILTER_KINDS = ("none", "ema", "ema2", "dema", "sma", "butter2", "savgol", "median")

# Kinds parameterised by a time constant (seconds) rather than a sample window.
_TAU_KINDS = ("ema", "ema2", "dema", "butter2")
# Kinds parameterised by a sample window. They accept `tau` too and derive N from it.
_WINDOW_KINDS = ("sma", "savgol", "median")


def alpha_from_tau(tau: float, dt: float) -> float:
    """Exact 1-pole smoothing factor for a time constant: alpha = 1 - exp(-dt/tau).

    Not the dt/tau approximation — that only agrees when tau >> dt, and drifts badly for
    the short constants a 250 Hz loop actually uses.
    """
    if tau is None or tau <= 0.0:
        return 1.0          # degenerate -> passthrough
    return 1.0 - math.exp(-dt / tau)


def _resolve_tau(tau, joint_num):
    """Broadcast tau to a length-joint_num float64 array.

    Accepts a scalar (uniform across joints) or a per-joint sequence. Per-joint tau exists
    for one real reason: encoder LSB dithering makes joint noise wildly non-uniform (a rest
    sample showed one wrist joint at 0.6 rad/s^2 while the shoulder read exactly 0). WHICH
    joint dithers is not stable across power cycles, so the sane default is a uniform tau
    sized for the worst joint, with a per-joint override reserved for a known-bad encoder.
    """
    arr = np.atleast_1d(np.asarray(tau, dtype=np.float64))
    if arr.size == 1:
        return np.full(joint_num, float(arr[0]), dtype=np.float64)
    if arr.size != joint_num:
        raise ValueError(f"per-joint tau must have {joint_num} values, got {arr.size}")
    return arr.astype(np.float64)


def window_from_tau(tau: float, dt: float) -> int:
    """Odd sample count whose averaging time roughly matches a tau-second 1-pole lag.

    An N-sample boxcar has mean lag (N-1)/2 samples, so matching a tau-second lag wants
    N ~ 2*tau/dt + 1. Forced odd so FIR windows stay symmetric about a real sample.
    """
    n = int(round(2.0 * tau / dt)) + 1
    n = max(3, n)
    return n if n % 2 else n + 1


def create_filter(kind, joint_num, dt, tau=None, window=None, poly=2):
    """Preallocate one filter's complete state. STARTUP ONLY — this allocates.

    Args:
        kind:      one of FILTER_KINDS
        joint_num: signals filtered in parallel (one filter, N independent channels)
        dt:        loop period (s) — alpha/cutoff are derived from it, never hardcoded
        tau:       time constant (s). Required for ema/ema2/dema/butter2; for the FIR
                   kinds it is only used to derive `window` when that is not given.
        window:    sample count for sma/savgol/median. Derived from tau if omitted.
        poly:      polynomial order, savgol only. Must be < window.

    Returns:
        dict — pass it to apply_filter(). Treat as opaque.
    """
    if kind not in FILTER_KINDS:
        raise ValueError(f"unknown filter kind '{kind}' — valid: {', '.join(FILTER_KINDS)}")

    f = {"kind": kind, "n": joint_num, "dt": dt, "init": False}

    if kind == "none":
        return f

    if kind in _TAU_KINDS:
        if tau is None:
            raise ValueError(f"filter '{kind}' needs tau > 0 (seconds)")
        # ema/ema2/dema support per-joint tau (alpha becomes a vector — cheap, the recurrence
        # already loops per channel). butter2 does not: per-joint tau would need per-joint
        # biquad coefficients, so it is uniform-only and rejects a sequence loudly.
        if kind == "butter2" and np.atleast_1d(np.asarray(tau)).size > 1:
            raise ValueError("butter2 supports only a uniform (scalar) tau, not per-joint")
        f["tau"] = tau

    if kind in _WINDOW_KINDS:
        if window is None:
            if tau is None or tau <= 0.0:
                raise ValueError(f"filter '{kind}' needs window (samples) or tau (seconds)")
            window = window_from_tau(tau, dt)
        window = int(window)
        if window < 3 or window % 2 == 0:
            raise ValueError(f"filter '{kind}' needs an odd window >= 3, got {window}")
        f["w"] = window
        # (channels, window) — newest sample at the far right. Shifted left each tick so
        # the buffer is always in time order and the FIR is a plain matrix-vector product.
        f["buf"] = np.zeros((joint_num, window), dtype=np.float64)
        if kind == "median":
            # Scratch for the in-place partition. The partition REORDERS what it touches,
            # so it must not run on buf itself — that would destroy the time ordering the
            # window depends on.
            f["scratch"] = np.zeros((joint_num, window), dtype=np.float64)

    if kind in ("ema", "ema2", "dema"):
        # Per-joint alpha (a length-joint_num array even when tau is scalar). The step
        # functions index a[i], so uniform and per-joint share one code path.
        tau_arr = _resolve_tau(tau, joint_num)
        if np.any(tau_arr <= 0.0):
            raise ValueError(f"filter '{kind}' needs tau > 0 on every joint")
        f["a"] = np.array([alpha_from_tau(float(t), dt) for t in tau_arr], dtype=np.float64)
        f["y1"] = np.zeros(joint_num, dtype=np.float64)
        if kind in ("ema2", "dema"):
            f["y2"] = np.zeros(joint_num, dtype=np.float64)

    elif kind == "butter2":
        # 2nd-order Butterworth via bilinear transform. Cutoff from tau: fc = 1/(2*pi*tau),
        # clamped below Nyquist — a cutoff at/above Nyquist is not a filter, it is a bug.
        from scipy.signal import butter
        if tau <= 0.0:
            raise ValueError("butter2 needs tau > 0 (seconds)")
        nyq = 0.5 / dt
        fc = 1.0 / (2.0 * math.pi * tau)
        if fc >= nyq:
            raise ValueError(
                f"butter2 tau={tau}s -> cutoff {fc:.1f} Hz is at/above Nyquist ({nyq:.1f} Hz); "
                f"tau must exceed {1.0 / (2.0 * math.pi * nyq):.5f}s at dt={dt}"
            )
        b, a = butter(2, fc / nyq, btype="low")
        f["b"], f["a_coef"] = np.asarray(b), np.asarray(a)
        f["fc"] = fc
        # Direct-form-II transposed state (2 per channel).
        f["z1"] = np.zeros(joint_num, dtype=np.float64)
        f["z2"] = np.zeros(joint_num, dtype=np.float64)

    elif kind == "savgol":
        from scipy.signal import savgol_coeffs
        if poly >= f["w"]:
            raise ValueError(f"savgol needs poly < window, got poly={poly} window={f['w']}")
        f["poly"] = int(poly)
        # deriv=0: smooth only. pos=w-1 evaluates the fit at the NEWEST sample rather than
        # the window centre, so the filter is causal — a centred fit would need future
        # samples we do not have on an RT loop.
        f["coeffs"] = np.asarray(
            savgol_coeffs(f["w"], poly, deriv=0, delta=dt, pos=f["w"] - 1, use="dot")
        )

    return f


def apply_filter(f, x, out):
    """Run one tick of filter `f` over `x`, writing into `out`. Allocates nothing.

    `x` and `out` are (joint_num,) float64. `out` may alias `x`.
    """
    _STEP[f["kind"]](f, x, out)


# ---------------------------------------------------------------------------
# Per-kind step functions.
#
# Scalar Python loops for the IIR kinds: joint_num is ~6, where a numpy ufunc's
# call overhead (~1 us) costs more than the whole loop, and every ufunc without an
# out= would allocate a temporary anyway. The FIR kinds do use numpy — a
# matrix-vector product over the window, with out=, which stays allocation-free.
# ---------------------------------------------------------------------------

def _step_none(f, x, out):
    out[:] = x


def _step_ema(f, x, out):
    y1, a = f["y1"], f["a"]             # a is a per-joint array
    if not f["init"]:
        y1[:] = x                       # seed on the first sample; never ramp from zero
        f["init"] = True
    else:
        for i in range(f["n"]):
            y1[i] += a[i] * (x[i] - y1[i])
    out[:] = y1


def _step_ema2(f, x, out):
    y1, y2, a = f["y1"], f["y2"], f["a"]
    if not f["init"]:
        y1[:] = x
        y2[:] = x
        f["init"] = True
    else:
        for i in range(f["n"]):
            y1[i] += a[i] * (x[i] - y1[i])
            y2[i] += a[i] * (y1[i] - y2[i])
    out[:] = y2


def _step_dema(f, x, out):
    # DEMA = 2*EMA(x) - EMA(EMA(x)). Lag-reduced, NOT a 2-pole low-pass.
    y1, y2, a = f["y1"], f["y2"], f["a"]
    if not f["init"]:
        y1[:] = x
        y2[:] = x
        f["init"] = True
    else:
        for i in range(f["n"]):
            y1[i] += a[i] * (x[i] - y1[i])
            y2[i] += a[i] * (y1[i] - y2[i])
    for i in range(f["n"]):
        out[i] = 2.0 * y1[i] - y2[i]


def _push(f, x):
    """Shift the FIR window left and append x. Keeps the buffer in time order, so the
    filter stays a plain dot product (a circular buffer would need a roll or an
    index-dependent kernel)."""
    buf = f["buf"]
    if not f["init"]:
        buf[:, :] = np.asarray(x)[:, None]   # fill, so the window is never zero-padded
        f["init"] = True
    else:
        buf[:, :-1] = buf[:, 1:]
        buf[:, -1] = x


def _step_sma(f, x, out):
    _push(f, x)
    np.mean(f["buf"], axis=1, out=out)


def _step_savgol(f, x, out):
    _push(f, x)
    np.dot(f["buf"], f["coeffs"], out=out)


def _step_median(f, x, out):
    _push(f, x)
    # np.median() and np.partition() both allocate a result array. ndarray.partition() is
    # in-place, so copy into scratch and partition that — only the k-th element has to be
    # correct, so a full sort would be wasted work anyway.
    k = f["w"] // 2
    scratch = f["scratch"]
    scratch[:] = f["buf"]
    scratch.partition(k, axis=1)
    out[:] = scratch[:, k]


def _step_butter2(f, x, out):
    # Direct-form II transposed biquad, per channel.
    b, a = f["b"], f["a_coef"]
    z1, z2 = f["z1"], f["z2"]
    if not f["init"]:
        # Seed the state to the steady-state response of a constant x, otherwise the
        # filter audibly rings its way up from zero over the first ~window.
        for i in range(f["n"]):
            z1[i] = x[i] * (b[1] + b[2] - a[1] - a[2])
            z2[i] = x[i] * (b[2] - a[2])
        f["init"] = True
    for i in range(f["n"]):
        y = b[0] * x[i] + z1[i]
        z1[i] = b[1] * x[i] - a[1] * y + z2[i]
        z2[i] = b[2] * x[i] - a[2] * y
        out[i] = y


_STEP = {
    "none": _step_none,
    "ema": _step_ema,
    "ema2": _step_ema2,
    "dema": _step_dema,
    "sma": _step_sma,
    "savgol": _step_savgol,
    "median": _step_median,
    "butter2": _step_butter2,
}


def _tau_ms(tau) -> str:
    """Format a scalar or per-joint tau in ms for the startup log."""
    arr = np.atleast_1d(np.asarray(tau, dtype=np.float64))
    if arr.size == 1:
        return f"{arr[0] * 1e3:.1f}ms"
    return f"{arr.min() * 1e3:.1f}..{arr.max() * 1e3:.1f}ms"


# ===========================================================================
# Filter bank — one fully independent filter per joint.
#
# A bank filters one SIGNAL (e.g. jerk) across all joints, where each joint may use a
# DIFFERENT filter kind and params. It is a list of single-channel filters plus two
# preallocated length-1 scratch arrays so apply stays allocation-free (slicing x[i:i+1]
# would mint a fresh view object every call — 28 signals*joints/tick of garbage).
#
# The uniform case (all joints identical) still builds N single-channel filters rather
# than one N-channel filter. That costs N python calls instead of 1 (~a microsecond each,
# ~30 us/tick total across every signal) in exchange for one code path that never
# special-cases "are all joints the same". Revisit only if profiling flags it.
# ===========================================================================

def create_filter_bank(specs, joint_num, dt):
    """Build one independent filter per joint. STARTUP ONLY.

    Args:
        specs: either ONE spec dict (applied to every joint) or a list of `joint_num`
               spec dicts. A spec is {"kind": str, optional "tau"/"window"/"poly"}.
        joint_num, dt: as create_filter.

    Returns:
        an opaque bank dict — pass to apply_filter_bank().
    """
    if isinstance(specs, dict):
        specs = [specs] * joint_num
    if len(specs) != joint_num:
        raise ValueError(f"filter bank needs 1 or {joint_num} specs, got {len(specs)}")

    filters = []
    for s in specs:
        kw = {k: s[k] for k in ("tau", "window", "poly") if k in s}
        filters.append(create_filter(s["kind"], 1, dt, **kw))

    return {
        "filters": filters,
        "n": joint_num,
        "si": np.zeros(1, dtype=np.float64),   # scratch in
        "so": np.zeros(1, dtype=np.float64),   # scratch out
        "last": np.zeros(joint_num, dtype=np.float64),  # last finite output per joint (NaN guard)
        "specs": specs,
    }


def apply_filter_bank(bank, x, out):
    """Run one tick of a bank over x (joint_num,), writing into out. Allocates nothing.

    A non-finite input (NaN/inf) is NOT fed to the filter — a single NaN would otherwise
    poison a recursive filter's state permanently (y += a*(NaN-y) makes y NaN forever). On a
    bad sample the last finite output is held instead, so a transient glitch degrades to a
    one-tick freeze rather than a stuck-NaN channel.
    """
    si, so, filters, last = bank["si"], bank["so"], bank["filters"], bank["last"]
    for i in range(bank["n"]):
        xi = x[i]
        if math.isfinite(xi):
            si[0] = xi
            apply_filter(filters[i], si, so)
            last[i] = so[0]
        out[i] = last[i]


def describe_bank(bank) -> str:
    """Per-joint summary for the startup log. Collapses to one line if all joints match."""
    descs = [describe(f) for f in bank["filters"]]
    if all(d == descs[0] for d in descs):
        return f"all joints: {descs[0]}"
    return " | ".join(f"J{i}:{d}" for i, d in enumerate(descs))


def describe(f) -> str:
    """One-line human summary — for the startup log, so the ACTIVE config is on record."""
    k = f["kind"]
    if k == "none":
        return "none (passthrough)"
    if k in ("ema", "ema2", "dema"):
        a = f["a"]
        if np.allclose(a, a[0]):
            return f"{k} (tau={_tau_ms(f['tau'])}, alpha={a[0]:.4f})"
        return f"{k} (per-joint alpha {a.min():.4f}..{a.max():.4f})"
    if k == "butter2":
        return f"butter2 (tau={f['tau'] * 1e3:.1f}ms, cutoff={f['fc']:.2f}Hz)"
    if k == "savgol":
        return f"savgol (window={f['w']}, poly={f['poly']})"
    return f"{k} (window={f['w']})"
