"""Signal processing helpers: EMA timing, angle unwrap/normalise, moving-average filters."""

from __future__ import annotations

import numpy as np

# Magic number constants
ANGLE_CLAMP_LIMIT = np.pi * 2 - 0.2


def update_rt_loop_stats(time_now: float, prev_time: float, loop_execution_time: float, delta_t: float,
                         avg_dur: float, avg_exec: float, avg_jitter: float,
                         max_loop_period: float, max_exec_time: float, max_jitter: float,
                         overrun_count: float, alpha: float = 0.05) -> tuple[float, ...]:
    """
    Compute all RT loop timing statistics for one tick.

    Tracks per-tick loop period, execution budget, jitter (|period - delta_t|),
    peak values for all three, and cumulative overrun count.

    Args:
        time_now:              time.perf_counter() at start of current tick
        prev_time:             time.perf_counter() at start of previous tick
        loop_execution_time:   seconds spent in computation this tick (measured at end)
        delta_t:               nominal loop period (e.g. 0.01 s)
        avg_dur:               current EMA of loop period
        avg_exec:              current EMA of execution time
        avg_jitter:            current EMA of jitter
        max_loop_period:       running peak loop period
        max_exec_time:         running peak execution time
        max_jitter:            running peak jitter
        overrun_count:         cumulative overrun ticks so far
        alpha:                 EMA smoothing factor (smaller = smoother, more lag)

    Returns:
        new_prev_time, loop_period, avg_dur, avg_exec,
        jitter, avg_jitter, max_loop_period, max_exec_time, max_jitter, overrun_count
    """
    loop_period = time_now - prev_time
    jitter      = abs(loop_period - delta_t)

    if avg_dur == 0.0:                  # first tick — no history yet
        avg_dur    = loop_period
        avg_exec   = loop_execution_time
        avg_jitter = jitter
    else:
        avg_dur    = alpha * loop_period         + (1 - alpha) * avg_dur
        avg_exec   = alpha * loop_execution_time + (1 - alpha) * avg_exec
        avg_jitter = alpha * jitter              + (1 - alpha) * avg_jitter

    if loop_period         > max_loop_period: max_loop_period = loop_period
    if loop_execution_time > max_exec_time:   max_exec_time   = loop_execution_time
    if jitter              > max_jitter:      max_jitter      = jitter

    overrun_count += 1 if loop_execution_time > delta_t else 0

    return (time_now, loop_period, avg_dur, avg_exec,
            jitter, avg_jitter, max_loop_period, max_exec_time, max_jitter, overrun_count)


def update_rolling_percentiles(period_buf: np.ndarray, exec_buf: np.ndarray, buf_pos: int, buf_count: int,
                               loop_period: float, loop_execution_time: float, loop_tick: float,
                               p95_period: float, p99_period: float, p95_exec: float, p99_exec: float,
                               stats_interval: int = 50) -> tuple[int, int, float, float, float, float]:
    """
    Maintain a fixed-size circular buffer of recent loop_period/loop_execution_time
    samples and periodically recompute p95/p99 from it.

    Why this exists alongside max_loop_period/max_jitter/overrun_count: those are
    cumulative since the shared-memory segment was created (not since this RTI
    process started — shared memory outlives process restarts), so a single
    historical spike hours ago silently pollutes them forever unless explicitly
    reset (see the loop_stats_reset_signal convention). These percentiles are
    always computed fresh from only the last len(period_buf) ticks, so they
    reflect genuinely *current* behavior even if nobody ever resets anything.

    Args:
        period_buf, exec_buf: preallocated float64 ndarrays, same fixed length —
            circular buffer storage, mutated in place (no per-tick allocation).
        buf_pos:    next write index into the circular buffer.
        buf_count:  number of valid samples written so far (caps at buffer length;
            used so percentiles aren't computed over stale zero-initialized slots
            before the buffer fills for the first time).
        loop_period, loop_execution_time: this tick's fresh samples.
        loop_tick:  current tick counter, used only to gate recompute cadence.
        p95_period, p99_period, p95_exec, p99_exec: previous values, returned
            unchanged on ticks where recompute doesn't fire.
        stats_interval: recompute every N ticks. A single recompute is two
            np.partition calls (period + exec, both percentile indices at once)
            over at most len(period_buf) elements — ~10us for a 500-sample
            buffer, negligible even on the tick it fires, but still gated rather
            than run every tick since there's no reason to pay it 250x/sec.

    Returns:
        (buf_pos, buf_count, p95_period, p99_period, p95_exec, p99_exec)
    """
    buf_size = period_buf.shape[0]
    period_buf[buf_pos] = loop_period
    exec_buf[buf_pos] = loop_execution_time
    buf_pos = (buf_pos + 1) % buf_size
    buf_count = min(buf_count + 1, buf_size)

    # Require a handful of samples before trusting a percentile at all (avoids a
    # noisy "p99" computed from e.g. 2 data points right after startup/reset).
    if buf_count >= 10 and int(loop_tick) % stats_interval == 0:
        idx95 = int(round(0.95 * (buf_count - 1)))
        idx99 = int(round(0.99 * (buf_count - 1)))
        period_part = np.partition(period_buf[:buf_count], [idx95, idx99])
        exec_part = np.partition(exec_buf[:buf_count], [idx95, idx99])
        p95_period = float(period_part[idx95])
        p99_period = float(period_part[idx99])
        p95_exec = float(exec_part[idx95])
        p99_exec = float(exec_part[idx99])

    return buf_pos, buf_count, p95_period, p99_period, p95_exec, p99_exec


def reset_rolling_stats() -> tuple[int, int]:
    """Discard the rolling-percentile window (e.g. after loop_stats_reset_signal).

    Must reset BOTH buf_pos and buf_count to 0, not buf_count alone:
    update_rolling_percentiles always reads period_buf[:buf_count] (i.e. it
    assumes the buffer fills starting at index 0), which only holds if writes
    also resume at index 0. Resetting buf_count without buf_pos would read stale
    pre-reset data at index 0 while fresh samples land wherever buf_pos was —
    exactly the kind of silent-pollution bug this feature exists to fix.

    Percentiles won't be trusted again until buf_count >= 10 (same startup guard
    as update_rolling_percentiles).

    Returns:
        (buf_pos, buf_count) = (0, 0)
    """
    return 0, 0


def pace_deadline(deadline: float, now: float, delta_t: float) -> tuple[float, float]:
    """Absolute-deadline pacing for the RT loop (2026-10-04): the next tick is due at the previous DEADLINE plus
    delta_t, not at "now + delta_t - work". Returns (new_deadline, sleep_s).

    Why: sleeping delta_t minus the work time, measured from when the work started, adds the wake-up latency and
    the sleep's own error to every tick and never corrects them — the loop ran at ~249 Hz (period 4.024 ms), and
    a PC pacing on the state stream inherited the drift. With an absolute deadline the average period is exactly
    delta_t; the per-tick jitter is unchanged (it is the kernel's and time.sleep's).

    The overrun rule, explicit: a tick that finishes past its deadline sleeps 0 (the next tick starts at once —
    one late tick is absorbed); more than one period behind is a STALL, and the deadline is re-based to `now`:
    the lost time is lost, a burst of back-to-back catch-up ticks is the wrong reaction to a stall.
    """
    deadline += delta_t
    sleep_s = deadline - now
    if sleep_s >= 0.0:
        return deadline, sleep_s
    if -sleep_s > delta_t:
        return now, 0.0
    return deadline, 0.0

