"""S1 — power and CAN: the drives powered, nothing moves (2026-10-18, TODO/first_run_plan.md).

    nodes        every declared drive (and a CAN gripper) answers; ids answering that the robot does not use
    can_bus      the CAN interface's state and error counters, watched for CAN_WATCH_S
    firmware     the drives' firmware versions agree, and this runtime supports them (V4, hardware/firmware_support.py)
    voltage      each drive's supply voltage and the spread between them
    temperature  the drives' temperatures
    faults       the drives' own fault flags

Everything is read through `robot.get_hardware()` (utility/hardware_report.py). The reference numbers are PAR6's
(measured 2026-10-18: supply 23.0-23.8 V on a 24 V supply, spread ~0.7 V, firmware 103 everywhere, 36-42 C); P5 of
the plan moves them into the robot XML's <first_run> block.
"""
from __future__ import annotations

from first_run.core import FAIL, INFO, PASS, WARN, result, step

SUPPLY_NOMINAL_V = 24.0     # PAR6's supply
SUPPLY_WARN_FRAC = 0.10     # outside +-10 %: WARN (the PSU's setting, or a voltage drop on the way)
SUPPLY_FAIL_FRAC = 0.20     # outside +-20 %: FAIL (a wrong supply, or a drive barely powered)
SPREAD_WARN_V = 1.5         # between the drives (PAR6 0.7 V): a thin / long wire or a bad terminal on one branch
TEMP_WARN_C = 60            # utility/health.py's DRIVE_TEMP_WARN_C
TEMP_SPREAD_WARN_C = 15     # one drive this much hotter than the others' median while nothing moves
CAN_WATCH_S = 3.0
BOOTLOADER_ID = 14          # a drive in its bootloader answers on 0x700 ids, which read as node 14
TIMING_ID = 13              # reserved for RTI's timing messages: no device should answer there

MISSING_HINTS = [
    "its power: is the drive's LED lit? Follow the power wires to it (with a daisy-chained supply, the connector "
    "before it)",
    "its CAN cable: both connectors fully seated; CAN-H and CAN-L not swapped at any connector",
    "its CAN ID: the drive must be set to the ID shown — a drive on a wrong ID appears under 'other IDs answering'",
    "termination: one 120 ohm resistor at each END of the bus — with all power off, about 60 ohm between CAN-H and "
    "CAN-L at any connector (120 = one missing, ~40 = one too many)",
]
NONE_HINTS = [
    "the drives' power: the control box's power button / relay; is any drive's LED lit?",
    "the cable from the Pi's CAN connector to the first drive",
    "termination (120 ohm at each end of the bus: ~60 ohm between CAN-H and CAN-L with the power off)",
    "every drive set to 1 Mbit/s",
]
FLAG_HINTS = {
    "temperature": "over-temperature: the drive's heatsink and airflow; a motor working against a load",
    "encoder": "the encoder: the magnet on the motor shaft (loose, missing, off-centre) or the encoder board's "
               "distance to it",
    "supply_voltage": "the supply voltage is outside the drive's range: the power supply and its wiring",
    "driver": "the motor driver stage: the motor's phase wires (loose or touching), the supply",
    "velocity": "over-speed: usually after a fall or a hand spin; clear the errors and watch whether it returns",
    "current": "over-current: a shorted motor phase, or a joint that is blocked",
    "estop": "the drive's E-stop input is active: the E-stop line to the drives, or the E-stop pressed",
    "watchdog": "the drive stopped hearing from the Pi for 100 ms: normal after a restart or a lost connection; "
                "Clear Errors — if it returns, that drive's CAN connection",
}


def _hw(ctx):
    hw = ctx["robot"].get_hardware()
    ctx["hardware"] = hw
    return hw


def _nodes(hw):
    return list(hw["drives"]) + ([hw["gripper"]] if hw.get("gripper") else [])


def check_nodes(ctx, ui):
    st = NODES
    hw = _hw(ctx)
    nodes = _nodes(hw)
    silent = [d for d in nodes if not d["answering"]]
    late = [d["name"] for d in nodes if d["answering"] and not d["answered_scan"]]
    dropped = [d["name"] for d in silent if d["answered_scan"]]
    extra = list(hw.get("scan_unexpected_ids", []))
    out = []
    value = {d["name"]: d["answering"] for d in nodes}
    if silent and len(silent) == len(nodes):
        out.append(result(st, FAIL, "no drive answers", value=value, expected="all answering", hints=NONE_HINTS))
    elif silent:
        names = ", ".join(f"{d['name']} (ID {d['can_id']})" for d in silent)
        hints = list(MISSING_HINTS)
        if dropped:
            hints.insert(0, f"{', '.join(dropped)} answered when RTI started and stopped since: a loose connector, a "
                            "power dip, or a drive that resets (its supply)")
        if len(silent) > 1:
            hints.append("if every drive after one point of the cable chain is silent, the break is at that point")
        if extra:
            hints.insert(0, f"other IDs answered at the start ({', '.join(map(str, extra))}): probably the missing "
                            "drive set to a wrong ID")
        out.append(result(st, FAIL, f"not answering: {names}", value=value, expected="all answering", hints=hints))
    else:
        note = f" ({', '.join(late)} came after RTI's start-up scan: powered later?)" if late else ""
        out.append(result(st, PASS, f"all {len(nodes)} drives answer{note}", value=value, expected="all answering"))
    for k in extra:
        if k == BOOTLOADER_ID:
            out.append(result(st, WARN, "a drive is waiting in its bootloader (it answers on the bootloader's IDs)",
                              value=k, hints=["an interrupted firmware update: flash it again — the dashboard's "
                                              "Firmware tab, 'Scan' finds it"]))
        elif k == TIMING_ID:
            out.append(result(st, WARN, f"a device answers on ID {k}, which is reserved", value=k,
                              hints=["ID 13 is reserved for RTI's timing messages: set that device to its own ID"]))
        else:
            out.append(result(st, WARN, f"a device answers on ID {k}, which this robot does not use", value=k,
                              hints=["a drive set to the wrong ID — one of the missing ones?",
                                     "set it to its joint's ID (J1 = 0 ... J6 = 5, a CAN gripper = 6)"]))
    return out


LINK_HINTS = [
    "termination: 120 ohm at each END of the bus (~60 ohm between CAN-H and CAN-L with the power off)",
    "a connector with CAN-H / CAN-L swapped, or not fully seated",
    "two drives set to the SAME ID (both answer at once and their replies collide)",
    "every drive at 1 Mbit/s; the cable's length and its ground / shield",
]


def check_can_bus(ctx, ui):
    st = CAN_BUS
    first = (ctx.get("hardware") or _hw(ctx)).get("can_link")
    if not first:
        return result(st, INFO, "the CAN interface's counters cannot be read here (simulation, or not on the robot's "
                                "Pi)")
    ui.say(f"Watching the CAN bus for {CAN_WATCH_S:.0f} s ...")
    ctx["sleep"](CAN_WATCH_S)
    last = _hw(ctx).get("can_link") or first
    keys = ("bus_off", "restarts", "error_passive", "error_warning", "bus_errors", "arbitration_lost")
    rise = {k: int(last.get(k) or 0) - int(first.get(k) or 0) for k in keys}
    total = {k: int(last.get(k) or 0) for k in keys}
    state = str(last.get("state") or "?")
    details = {"first": first, "last": last, "rise": rise}
    value = {"state": state, **{f"{k}_rise": v for k, v in rise.items() if v}}
    if state in ("ERROR-PASSIVE", "BUS-OFF", "STOPPED") or any(rise[k] > 0 for k in ("bus_off", "restarts",
                                                                                     "error_passive")):
        return result(st, FAIL, f"the CAN bus is in trouble: state {state}, while watching "
                      + ", ".join(f"+{v} {k}" for k, v in rise.items() if v), value=value,
                      expected="ERROR-ACTIVE, no new errors", hints=LINK_HINTS, details=details)
    if state == "ERROR-WARNING" or rise["bus_errors"] > 0:
        return result(st, WARN, f"state {state}, +{rise['bus_errors']} bus errors in {CAN_WATCH_S:.0f} s",
                      value=value, expected="ERROR-ACTIVE, no new errors", hints=LINK_HINTS, details=details)
    if any(total[k] > 0 for k in ("bus_off", "restarts", "error_passive")):
        return result(st, WARN, "clean now, but since RTI started: " + ", ".join(
                          f"{total[k]} {k}" for k in ("bus_off", "restarts", "error_passive") if total[k]),
                      value=value, hints=["a drive powered up or down, or a cable moved, while RTI ran — if the "
                                          "counters keep rising, check:"] + LINK_HINTS, details=details)
    return result(st, PASS, f"state {state}, no errors in {CAN_WATCH_S:.0f} s nor since RTI started", value=value,
                  expected="ERROR-ACTIVE, no new errors", details=details)


def check_firmware(ctx, ui):
    st = FIRMWARE
    hw = ctx.get("hardware") or _hw(ctx)
    joints = [d for d in hw["drives"] if d["answering"]]
    if not joints:
        return result(st, INFO, "no drive answers (see 'Every drive answers')")
    fw = {d["name"]: d["firmware"] for d in joints}
    unread = [n for n, v in fw.items() if not v]
    known = sorted({v for v in fw.values() if v})
    grip = hw.get("gripper")
    extra = f"; gripper {grip['firmware']}" if grip and grip["answering"] else ""
    value = dict(fw, **({"gripper": grip["firmware"]} if grip else {}))
    # Outside what this runtime supports (versioning V4, hardware/firmware_support.py): a FAIL here, a warning at run
    # time. An executor from before V4 sends no verdict: nothing is judged then.
    outside = [d for d in joints + ([grip] if grip and grip["answering"] else [])
               if d.get("firmware_supported") in ("too_old", "too_new")]
    if outside:
        return result(st, FAIL, "firmware this runtime does not support: " + ", ".join(
                          f"{d['name']} {d['firmware']} ({d.get('driver_type')}: {d.get('supported_range')})"
                          for d in outside),
                      value=value, expected="a supported version on every drive",
                      hints=["update the drives: the dashboard's Firmware tab, 'Update robot' (it flashes every drive "
                             "with the release its driver type needs)"])
    if len(known) > 1:
        return result(st, WARN, "the joints run different firmware: " + ", ".join(f"{n} {v}" for n, v in fw.items()),
                      value=value, expected="the same version on every joint",
                      hints=["update every drive to the same release: the dashboard's Firmware tab, 'Update robot'"])
    if unread:
        return result(st, WARN, f"firmware not read for {', '.join(unread)}", value=value,
                      hints=["RTI reads the versions at start: restart the service once every drive is powered"])
    return result(st, PASS, f"every joint on firmware {known[0]}{extra}", value=value,
                  expected="the same version on every joint")


def check_voltage(ctx, ui):
    st = VOLTAGE
    hw = ctx.get("hardware") or _hw(ctx)
    nodes = [d for d in _nodes(hw) if d["answering"]]
    volts = {d["name"]: d["voltage_v"] for d in nodes if d["voltage_v"] > 0}
    if not volts:
        return result(st, INFO, "no supply voltage read yet (no drive answers, or RTI has not polled them)")
    lo_name = min(volts, key=volts.get)
    lo, hi = volts[lo_name], max(volts.values())
    spread = hi - lo
    value = {"volts": volts, "spread_v": round(spread, 2)}
    expected = f"{SUPPLY_NOMINAL_V:.0f} V +-{SUPPLY_WARN_FRAC:.0%}, spread under {SPREAD_WARN_V} V"
    far = {n: v for n, v in volts.items() if abs(v - SUPPLY_NOMINAL_V) > SUPPLY_FAIL_FRAC * SUPPLY_NOMINAL_V}
    off = {n: v for n, v in volts.items() if abs(v - SUPPLY_NOMINAL_V) > SUPPLY_WARN_FRAC * SUPPLY_NOMINAL_V}
    supply_hints = [f"the power supply's output (this robot expects {SUPPLY_NOMINAL_V:.0f} V)",
                    "the power wiring to the drives: wire size, screw terminals, connectors"]
    if far:
        return result(st, FAIL, "supply voltage far off: " + ", ".join(f"{n} {v:.1f} V" for n, v in far.items()),
                      value=value, expected=expected, hints=supply_hints)
    if off:
        return result(st, WARN, "supply voltage off: " + ", ".join(f"{n} {v:.1f} V" for n, v in off.items()),
                      value=value, expected=expected, hints=supply_hints)
    if spread > SPREAD_WARN_V:
        return result(st, WARN, f"{lo_name} gets {spread:.1f} V less than the best-supplied drive ({lo:.1f} V)",
                      value=value, expected=expected,
                      hints=[f"the power path to {lo_name}: a thin or long wire, a loose screw terminal, a worn "
                             "connector on its branch of the supply"])
    return result(st, PASS, f"{lo:.1f}-{hi:.1f} V on every drive (spread {spread:.1f} V)", value=value,
                  expected=expected)


def check_temperature(ctx, ui):
    st = TEMPERATURE
    hw = ctx.get("hardware") or _hw(ctx)
    temps = {d["name"]: d["temperature_c"] for d in _nodes(hw) if d["answering"]}
    if not temps:
        return result(st, INFO, "no temperature read (no drive answers)")
    ordered = sorted(temps.values())
    median = ordered[len(ordered) // 2]
    hot = {n: t for n, t in temps.items() if t >= TEMP_WARN_C}
    odd = {n: t for n, t in temps.items() if t - median > TEMP_SPREAD_WARN_C}
    value = temps
    if hot or odd:
        bad = dict(odd, **hot)
        return result(st, WARN, "warm: " + ", ".join(f"{n} {t} C" for n, t in bad.items()) + f" (others ~{median} C)",
                      value=value, expected=f"under {TEMP_WARN_C} C, near the others",
                      hints=["while nothing moves a drive should stay near the others: its heatsink and airflow; a "
                             "motor held against a load (a joint resting on an endstop); a drive that is faulty"])
    return result(st, PASS, f"{ordered[0]}-{ordered[-1]} C", value=value, expected=f"under {TEMP_WARN_C} C")


def check_faults(ctx, ui):
    st = FAULTS
    hw = ctx.get("hardware") or _hw(ctx)
    out = []
    for d in _nodes(hw):
        if not d["answering"]:
            continue
        flags = [k for k, v in d["flags"].items() if v]
        if not d["calibrated"]:
            out.append(result(st, WARN, f"{d['name']} reports it is not calibrated", value=d["name"],
                              hints=["the drive's motor / encoder calibration has to be done once (the drive's "
                                     "configuration tool) before it can run the motor"]))
        if not flags and not d["fault"]:
            continue
        verdict = WARN if flags == ["watchdog"] else FAIL
        what = ", ".join(flags) if flags else "a fault (its flags not read yet)"
        out.append(result(st, verdict, f"{d['name']} reports {what}", value={d["name"]: flags},
                          hints=[FLAG_HINTS[k] for k in flags] or ["Clear Errors and read again; logs/RTI.log names "
                                                                   "the fault"]))
    if not out:
        return result(st, PASS, "no drive reports a fault", expected="no fault flags")
    return out


NODES = step("S1.nodes", 1, "Every drive answers", check_nodes, needs=("robot",),
             about="every declared drive (and a CAN gripper) answers on its CAN ID")
CAN_BUS = step("S1.can_bus", 1, "The CAN bus is clean", check_can_bus, needs=("robot",),
               about="the CAN interface's state and error counters, watched for a few seconds")
FIRMWARE = step("S1.firmware", 1, "Firmware versions", check_firmware, needs=("robot",),
                about="every joint's drive runs the same firmware, one this runtime supports")
VOLTAGE = step("S1.voltage", 1, "Supply voltage", check_voltage, needs=("robot",),
               about="each drive's supply voltage, and the difference between them")
TEMPERATURE = step("S1.temperature", 1, "Drive temperatures", check_temperature, needs=("robot",),
                   about="no drive hot while nothing moves")
FAULTS = step("S1.faults", 1, "Drive fault flags", check_faults, needs=("robot",),
              about="the drives' own fault flags (encoder, driver, supply, ...) and calibration")
STEPS = [NODES, CAN_BUS, FIRMWARE, VOLTAGE, TEMPERATURE, FAULTS]
