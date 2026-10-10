"""CAN link-layer health — the counters only the kernel keeps.

Exists because automatic bus-off recovery makes a bus-off **invisible** to the
rest of the system. `CAN_BUS_OFF_RESTART_MS = 100` (can_hardware.py) brings the
controller back ~100 ms after it drops off the bus, which is past the 80 ms
`old_data_threshold` but well short of the 200 ms `disconnect_threshold` in
`check_data_freshness`. So RTI logs a transient stale-data warning and carries
on — indistinguishable from a dropped frame, a slow board, or a marginal
connector. Nothing in the runtime reads the interface state at all.

That is the cost of auto-recovery: a loud permanent failure became a silent
transient one. A bus that is quietly flapping (restart -> bus-off -> restart) is
a real wiring or termination fault being masked by the very feature keeping the
arm alive through it, and the kernel's counters are the ONLY record that it
happened. This module reads them so the fault leaves evidence.

**Not for the RT loop.** It shells out to `ip`; supervisor.py polls it at 1 Hz.

    stats = read_can_link_stats('can0')
    # {'state': 'ERROR-ACTIVE', 'restart_ms': 100, 'restarts': 0,
    #  'bus_errors': 0, 'arbitration_lost': 0, 'error_warning': 0,
    #  'error_passive': 0, 'bus_off': 0}
"""

import logging
import subprocess

logger = logging.getLogger(__name__)

# Header emitted by `ip -details -statistics link show`, in order. The value line
# that follows carries these six counters first, then unrelated queue fields.
_COUNTER_HEADER = ("re-started", "bus-errors", "arbit-lost",
                   "error-warn", "error-pass", "bus-off")

_COUNTER_KEYS = ("restarts", "bus_errors", "arbitration_lost",
                 "error_warning", "error_passive", "bus_off")

# Counters worth shouting about. bus_errors ticks up on ordinary noise and would
# drown the signal, so it is read but not alarmed on.
_ALARM_KEYS = ("bus_off", "restarts", "error_passive")

_LABELS = {
    "bus_off":          "bus-off events",
    "restarts":         "automatic restarts",
    "error_passive":    "error-passive transitions",
    "error_warning":    "error-warning transitions",
    "arbitration_lost": "arbitration losses",
    "bus_errors":       "bus errors",
}


def read_can_link_stats(interface="can0", timeout_s=2.0):
    """Kernel CAN counters for `interface`, or None if they cannot be read.

    Never raises: a missing interface or an `ip` that formats its output
    differently must degrade to "no data", not take the supervisor down.
    """
    try:
        completed = subprocess.run(
            ["ip", "-details", "-statistics", "link", "show", interface],
            capture_output=True, text=True, timeout=timeout_s)
    except (OSError, subprocess.SubprocessError):
        return None
    if completed.returncode != 0:
        return None

    lines = completed.stdout.splitlines()
    stats = {"interface": interface, "state": None, "restart_ms": None}

    for index, line in enumerate(lines):
        tokens = line.split()

        # "can state ERROR-ACTIVE restart-ms 100"
        if len(tokens) >= 3 and tokens[0] == "can" and tokens[1] == "state":
            stats["state"] = tokens[2]
            if "restart-ms" in tokens:
                position = tokens.index("restart-ms")
                if position + 1 < len(tokens):
                    stats["restart_ms"] = _to_int(tokens[position + 1])

        # The counter header, whose values are the first six fields of the NEXT
        # line (trailed by numtxqueues/... which we ignore).
        if tuple(tokens[:len(_COUNTER_HEADER)]) == _COUNTER_HEADER:
            if index + 1 >= len(lines):
                continue
            values = lines[index + 1].split()[:len(_COUNTER_KEYS)]
            if len(values) != len(_COUNTER_KEYS):
                continue
            parsed = [_to_int(value) for value in values]
            if any(value is None for value in parsed):
                continue
            stats.update(dict(zip(_COUNTER_KEYS, parsed)))

    if stats["state"] is None and "bus_off" not in stats:
        return None       # not a CAN interface, or output we do not understand
    return stats


def _to_int(text):
    try:
        return int(text)
    except (TypeError, ValueError):
        return None


def describe_can_link_change(previous, current):
    """Human-readable summary of what changed, or None if nothing did.

    Returns None on the first call (previous is None) — the caller logs its own
    baseline. A counter that DECREASED means the interface was taken down and
    back up, which zeroes the kernel's counters; that is reported as a re-base
    rather than as a nonsensical negative delta.
    """
    if not current:
        return None
    if not previous:
        return None

    if any(_counter(current, key) < _counter(previous, key) for key in _COUNTER_KEYS):
        return (f"{current['interface']} counters reset "
                f"(interface was brought down and up) — "
                f"{format_can_link_stats(current)}")

    parts = []
    for key in _ALARM_KEYS:
        delta = _counter(current, key) - _counter(previous, key)
        if delta > 0:
            parts.append(f"+{delta} {_LABELS[key]}")

    state_changed = current.get("state") != previous.get("state")
    if state_changed:
        parts.append(f"state {previous.get('state')} -> {current.get('state')}")

    if not parts:
        return None
    return (f"{current['interface']}: " + ", ".join(parts) +
            f" (totals: {format_can_link_stats(current)})")


def _counter(stats, key):
    value = stats.get(key)
    return 0 if value is None else int(value)


def format_can_link_stats(stats):
    """One-line summary for logs."""
    if not stats:
        return "unavailable"
    return (f"state={stats.get('state')} restart-ms={stats.get('restart_ms')} "
            f"bus-off={_counter(stats, 'bus_off')} "
            f"restarts={_counter(stats, 'restarts')} "
            f"error-passive={_counter(stats, 'error_passive')} "
            f"error-warning={_counter(stats, 'error_warning')} "
            f"bus-errors={_counter(stats, 'bus_errors')} "
            f"arbitration-lost={_counter(stats, 'arbitration_lost')}")

