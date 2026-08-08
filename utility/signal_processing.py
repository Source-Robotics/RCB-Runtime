"""Signal processing helpers: EMA timing, angle unwrap/normalise, moving-average filters."""

import numpy as np
import time

# Magic number constants
ANGLE_CLAMP_LIMIT = np.pi * 2 - 0.2


def update_loop_timing_ema(time_now, prev_time, loop_execution_time,
                           avg_dur, avg_exec, alpha=0.1):
    """Update loop duration and execution time using exponential moving average."""
    loop_duration = time_now - prev_time
    if avg_dur is None:
        avg_dur  = loop_duration
        avg_exec = loop_execution_time
    else:
        avg_dur  = alpha * loop_duration       + (1 - alpha) * avg_dur
        avg_exec = alpha * loop_execution_time + (1 - alpha) * avg_exec
    return time_now, avg_dur, avg_exec


def update_rt_loop_stats(time_now, prev_time, loop_execution_time, delta_t,
                         avg_dur, avg_exec, avg_jitter,
                         max_loop_period, max_exec_time, max_jitter,
                         overrun_count, alpha=0.05):
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


def update_rolling_percentiles(period_buf, exec_buf, buf_pos, buf_count,
                               loop_period, loop_execution_time, loop_tick,
                               p95_period, p99_period, p95_exec, p99_exec,
                               stats_interval=50):
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


def reset_rolling_stats():
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


def unwrap_angle(angle, previous_angle, clamp_limit=np.pi * 2 - 0.2):
    """Unwrap angle to prevent discontinuities, then clamp to ±clamp_limit."""
    while angle - previous_angle >  np.pi:
        angle -= 2 * np.pi
    while angle - previous_angle < -np.pi:
        angle += 2 * np.pi
    return max(min(angle, clamp_limit), -clamp_limit)


def normalize_angle(angle):
    """Normalise angle to [-pi, pi]."""
    while angle >  np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def unwrap_angles(q_solution, q_current):
    """Unwrap all joint angles in q_solution to be closest to q_current."""
    q_unwrapped = q_solution.copy()
    for i in range(len(q_solution)):
        diff = q_solution[i] - q_current[i]
        if diff >  np.pi:
            q_unwrapped[i] = q_solution[i] - 2 * np.pi
        elif diff < -np.pi:
            q_unwrapped[i] = q_solution[i] + 2 * np.pi
    return q_unwrapped


def cartesian_moving_average(buffer):
    """Simple moving average (mean) over a buffer of Cartesian samples."""
    return np.mean(buffer, axis=0)


def cartesian_moving_average_2(buffer, cart_window_size):
    """Moving average via convolution — better for large data sets."""
    window = np.ones(cart_window_size) / cart_window_size
    return np.array([np.convolve(buffer[:, i], window, mode='valid')[-1]
                     for i in range(6)])


def cartesian_moving_average_EMA(new_data, last_ema, alpha=0.8):
    """Exponential Moving Average for 6-DOF Cartesian data."""
    if last_ema is None:
        return new_data
    return alpha * np.array(new_data) + (1 - alpha) * last_ema
