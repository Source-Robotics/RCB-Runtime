"""The robot's health summary — one level per component, like a diagnostics panel (2026-10-15).

What an operator (the System Info tab's Health card), a program (`robot.get_health()`) or the supervisor can read at
a glance before digging into the Errors, Activity or Logs tabs: every part of the robot judged OK / WARN / ERROR /
STALE with one line saying why, plus the numbers it was judged on. STALE is the state of a part whose heartbeat has
stopped (RTI's tick, the executor's or the self-collision worker's counter): "no data" is a finding of its own, not
OK and not an error of the robot.

    assess(shared_data, joint_num, delta_t, has_gripper) -> {"level", "time", "components": [{name, level, summary,
                                                             values}]}

Pure rules over shared memory and a few system readings (disk, memory, the CPU temperature, the clock) — nothing
here commands the robot. ~60 ms: it watches the counters advance over WATCH_S. The clock check (a subprocess) is
cached for CLOCK_CACHE_S. Thresholds are module constants, named below with their reason.
"""
from __future__ import annotations

import shutil
import time

import psutil

from project_paths import PROJECT_ROOT
from utility.error_catalog import entry as catalog_entry

LEVELS = ("OK", "WARN", "ERROR", "STALE")
_RANK = {"OK": 0, "WARN": 1, "ERROR": 2, "STALE": 3}

WATCH_S = 0.05                 # the window the counters must advance in: 12 RTI ticks, 2.5 executor beats, 5 worker cycles
LOOP_WARN_FACTOR = 1.05        # p99 period over this x nominal: WARN (LOOP_DEGRADED's own band)
LOOP_ERROR_FACTOR = 1.5        # ...over this: ERROR (LOOP_CRITICAL's)
DRIVE_TEMP_WARN_C = 60         # a drive this hot is working hard; the drives fault at their own limit
SC_CYCLE_WARN_MS = 10.0        # the self-collision worker's cycle at 100 Hz should stay under its period
DISK_WARN_BYTES = 500_000_000  # logs rotate, but recordings and bundles need room
DISK_ERROR_BYTES = 200_000_000 # the support bundle refuses under this
MEMORY_WARN_PCT = 85.0
MEMORY_ERROR_PCT = 95.0
CPU_TEMP_WARN_C = 70.0         # the Pi throttles at 80 C
CPU_TEMP_ERROR_C = 80.0
CLOCK_CACHE_S = 60.0
# display name -> the command-line token that identifies the process (the dashboard's process card uses the same)
PROCESSES = (("RTI", "RTI.py"), ("executor", "command_executor.py"), ("dashboard", "nicegui_test.py"),
             ("supervisor", "supervisor.py"), ("self-collision worker", "self_collision_worker.py"),
             ("recorder", "recorder.py"))

_clock_cache = {"time": 0.0, "status": None}


def worst(levels) -> str:
    """The most serious of `levels` (OK < WARN < ERROR < STALE); OK for none."""
    best = "OK"
    for lv in levels:
        if _RANK.get(lv, 0) > _RANK[best]:
            best = lv
    return best


def component(name, level, summary, values=None) -> dict:
    return {"name": name, "level": level, "summary": summary, "values": dict(values or {})}


def _counters(rti):
    return (int(rti["loop_tick"][0]), float(rti["exec_heartbeat"][0]), float(rti["sc_heartbeat"][0]))


def assess(shared_data, joint_num, delta_t, has_gripper=True, watch_s=WATCH_S, system=True, sleep=time.sleep,
           head_fn=None) -> dict:
    """The summary. `system` False skips the disk / memory / temperature / clock / process readings (the nets)."""
    rti, motor, gripper = shared_data.rti, shared_data.motor, shared_data.gripper
    if head_fn is None:
        from utility.trace_ring import read_head as head_fn
    n = int(joint_num)
    before = _counters(rti)
    sleep(watch_s)
    after = _counters(rti)
    rti_alive = after[0] != before[0]
    comps = [_loop(rti, delta_t, rti_alive)]
    comps.append(_memory_lock(rti) if rti_alive else component("memory lock", "STALE", "RTI is not running"))
    stale = "RTI is not running: its tick does not advance"
    comps.append(_can(rti, n, has_gripper) if rti_alive else component("CAN", "STALE", stale))
    comps.append(_drives(motor, n) if rti_alive else component("drives", "STALE", stale))
    if has_gripper:
        comps.append(_gripper(rti, gripper, n) if rti_alive else component("gripper", "STALE", stale))
    comps.append(_errors(shared_data))
    comps.append(_homing(shared_data))
    comps.append(component("executor", "OK" if after[1] != before[1] else "STALE",
                           "command executor alive" if after[1] != before[1] else "the executor's heartbeat does not advance",
                           {"heartbeat": after[1]}))
    comps.append(_worker(rti, after[2] != before[2]))
    comps.append(_recorder(head_fn()))
    if system:
        comps.append(_processes())
        comps.append(_disk())
        comps.append(_memory())
        comps.append(_cpu_temperature())
        comps.append(_clock())
    return {"level": worst(c["level"] for c in comps), "time": time.time(), "components": comps}


# ---------------------------------------------------------------------------------------------------------------
def _loop(rti, delta_t, alive):
    p99 = float(rti["p99_loop_period"][0])
    overruns = int(rti["overrun_count"][0])
    values = {"tick": int(rti["loop_tick"][0]), "p99_period_ms": round(p99 * 1e3, 3),
              "nominal_ms": round(float(delta_t) * 1e3, 3), "overruns": overruns,
              "exec_time_ms": round(float(rti["avg_loop_execution_time"][0]) * 1e3, 3)}
    if not alive:
        return component("control loop", "STALE", "RTI is not running: its tick does not advance", values)
    if p99 > LOOP_ERROR_FACTOR * delta_t:
        return component("control loop", "ERROR", f"p99 period {p99 * 1e3:.2f} ms, over {LOOP_ERROR_FACTOR:g}x the "
                         f"{delta_t * 1e3:.1f} ms tick", values)
    if p99 > LOOP_WARN_FACTOR * delta_t:
        return component("control loop", "WARN", f"p99 period {p99 * 1e3:.2f} ms, over {LOOP_WARN_FACTOR:g}x the "
                         f"{delta_t * 1e3:.1f} ms tick", values)
    return component("control loop", "OK", f"{1.0 / delta_t:.0f} Hz, p99 period {p99 * 1e3:.2f} ms, "
                     f"{overruns} overruns since start", values)


def _can(rti, n, has_gripper):
    nodes = n + (1 if has_gripper else 0)
    lost = [i for i in range(nodes) if rti["CAN_node_connection_error"][i]]
    stale = [i for i in range(nodes) if rti["old_data_warning"][i] and i not in lost]
    name = lambda i: "gripper" if i == n else f"J{i + 1}"
    values = {"lost": [name(i) for i in lost], "stale": [name(i) for i in stale],
              "connected": bool(rti["robot_can_connected"][0] > 0.5)}
    if lost:
        return component("CAN", "ERROR", "no replies from " + ", ".join(values["lost"]), values)
    if stale:
        return component("CAN", "WARN", "old data from " + ", ".join(values["stale"]), values)
    return component("CAN", "OK", f"every node answering ({nodes})", values)


def _drives(motor, n):
    temps = [int(motor["motor_temperature"][i]) for i in range(n)]
    faulted = [f"J{i + 1}" for i in range(n) if int(motor["global_motor_error"][i]) != 0]
    hot = [f"J{i + 1} {temps[i]} C" for i in range(n) if temps[i] >= DRIVE_TEMP_WARN_C]
    values = {"temperature_c": temps, "faulted": faulted,
              "voltage_v": [round(float(motor["motor_voltage"][i]) / 1000.0, 1) for i in range(n)]}
    if faulted:
        return component("drives", "ERROR", "drive fault on " + ", ".join(faulted), values)
    if hot:
        return component("drives", "WARN", "hot: " + ", ".join(hot), values)
    return component("drives", "OK", f"no drive fault, hottest {max(temps) if temps else 0} C", values)


def _gripper(rti, gripper, n):
    values = {"connected": bool(rti["CAN_gripper_connected"][0] > 0.5), "fault": int(rti["node_error_bits"][n]) != 0,
              "homed": bool(gripper["gripper_homed"][0] > 0.5)}
    if not values["connected"]:
        return component("gripper", "ERROR", "gripper not answering", values)
    if values["fault"]:
        return component("gripper", "ERROR", "gripper drive fault", values)
    if not values["homed"]:
        return component("gripper", "WARN", "gripper not homed", values)
    return component("gripper", "OK", "gripper answering, homed", values)


def _errors(shared_data):
    try:
        data = shared_data.error_state.get_json()
    except Exception:
        data = None
    items = [e for e in data if isinstance(e, dict) and "key" in e] if isinstance(data, list) else []
    errors = [e for e in items if e.get("level", "error") != "warning"]
    warnings = [e for e in items if e.get("level") == "warning"]
    label = lambda e: f"{(catalog_entry(e['key']) or {}).get('number', '')} {e['key']}".strip()
    values = {"errors": [label(e) for e in errors], "warnings": [label(e) for e in warnings]}
    if errors:
        return component("errors", "ERROR", f"{len(errors)} latched: " + ", ".join(values["errors"][:4]), values)
    if warnings:
        return component("errors", "WARN", f"{len(warnings)} warning(s): " + ", ".join(values["warnings"][:4]), values)
    return component("errors", "OK", "none latched", values)


def _homing(shared_data):
    try:
        homed = shared_data.homed_state.get()
    except Exception:
        homed = None
    mode = shared_data.robot_mode.get() if shared_data.robot_mode is not None else None
    values = {"homed": homed == "HOMED", "mode": mode}
    if mode == "HOMING":
        return component("homing", "OK", "homing in progress", values)
    if homed != "HOMED":
        return component("homing", "WARN", "not homed", values)
    return component("homing", "OK", "homed", values)


def _worker(rti, alive):
    cycle = float(rti["sc_cycle_ms"][0])
    values = {"heartbeat": float(rti["sc_heartbeat"][0]), "slowest_cycle_ms": round(cycle, 2)}
    if not alive:
        return component("self-collision worker", "STALE", "the worker's heartbeat does not advance", values)
    if cycle > SC_CYCLE_WARN_MS:
        return component("self-collision worker", "WARN", f"slowest cycle {cycle:.1f} ms", values)
    return component("self-collision worker", "OK", f"alive, slowest cycle {cycle:.1f} ms", values)


def _memory_lock(rti):
    """RTI's locked memory (config/system_config.lock_memory, 2026-10-16): without it a memory shortage on the Pi can
    stall the loop (it did for 194 s on 2026-10-16). rti['memory_locked_mb']: MB, NaN = locked but not measured,
    0 = not locked, -1 = simulation (not tried)."""
    v = float(rti["memory_locked_mb"][0])
    if v != v:
        return component("memory lock", "OK", "RTI's memory is locked in RAM", {"locked_mb": None})
    values = {"locked_mb": round(v, 1)}
    if v < 0:
        return component("memory lock", "OK", "simulation: RTI's memory is not locked (not needed)", values)
    if v == 0:
        return component("memory lock", "WARN", "RTI's memory is NOT locked: a memory shortage can stall the loop — "
                         "the service needs LimitMEMLOCK=infinity (docs/rti_loop_timing.md)", values)
    return component("memory lock", "OK", f"{v:.0f} MB of RTI's memory locked in RAM", values)


def _recorder(head, now=None):
    """The recorder (recorder.py): its last pass, from the trace's head. STALE when it is older than a second (no
    incident dumps or captures then); no head = RTI never wrote its trace."""
    now = time.time() if now is None else now
    if not head or not head.get("generation"):
        return component("recorder", "STALE", "RTI's trace is not there (RTI not running?)", {})
    age = now - head.get("rec_time", 0.0)
    values = {"last_pass_s_ago": round(age, 2), "rows_written": int(head.get("count", 0))}
    if not 0.0 <= age <= 1.0:
        return component("recorder", "STALE", "the recorder is not running: no incident dumps or captures", values)
    return component("recorder", "OK", "recording (incidents, captures)", values)


def _processes():
    from utility.process_utils import process_info
    rows = {r["name"]: r for r in process_info(dict(PROCESSES))}
    missing = [name for name, _ in PROCESSES if name not in rows]
    values = {name: (f"pid {rows[name]['pid']}" if name in rows else "missing") for name, _ in PROCESSES}
    if missing:
        return component("processes", "ERROR", "not running: " + ", ".join(missing), values)
    return component("processes", "OK", f"all {len(PROCESSES)} running", values)


def _disk():
    usage = shutil.disk_usage(str(PROJECT_ROOT))
    values = {"free_mb": usage.free // 1_000_000, "total_mb": usage.total // 1_000_000}
    if usage.free < DISK_ERROR_BYTES:
        return component("disk", "ERROR", f"{values['free_mb']} MB free", values)
    if usage.free < DISK_WARN_BYTES:
        return component("disk", "WARN", f"{values['free_mb']} MB free", values)
    return component("disk", "OK", f"{values['free_mb']} MB free", values)


def _memory():
    vm = psutil.virtual_memory()
    values = {"used_pct": round(vm.percent, 1), "available_mb": vm.available // 1_000_000}
    level = "ERROR" if vm.percent > MEMORY_ERROR_PCT else "WARN" if vm.percent > MEMORY_WARN_PCT else "OK"
    return component("memory", level, f"{vm.percent:.0f} % used, {values['available_mb']} MB available", values)


def _cpu_temperature():
    from utility.system_temperature import get_raspberry_pi_temperature
    t = get_raspberry_pi_temperature()
    if t is None:
        return component("CPU temperature", "WARN", "not readable", {})
    level = "ERROR" if t > CPU_TEMP_ERROR_C else "WARN" if t > CPU_TEMP_WARN_C else "OK"
    return component("CPU temperature", level, f"{t:.0f} C", {"celsius": round(float(t), 1)})


def _clock():
    from utility.display_time import clock_status
    now = time.monotonic()
    if _clock_cache["status"] is None or now - _clock_cache["time"] > CLOCK_CACHE_S:
        _clock_cache["status"] = clock_status()
        _clock_cache["time"] = now
    st = _clock_cache["status"]
    values = {"ntp_synced": st.get("synced"), "rtc": st.get("rtc"), "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
    if st.get("synced") is True:
        return component("clock", "OK", "synchronised (NTP)", values)
    if st.get("rtc"):
        return component("clock", "OK" if st.get("synced") is None else "WARN", "a real-time clock, no NTP", values)
    return component("clock", "WARN", "not synchronised, no real-time clock: times may be wrong", values)
