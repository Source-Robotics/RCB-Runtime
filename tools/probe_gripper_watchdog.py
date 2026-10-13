#!/usr/bin/env python3
"""Standalone CAN watchdog A/B probe: does a node's watchdog trip when unfed?

Background (currently_doing.md 2026-07-15): after every RTI kill/restart the
arm joints latch a WATCHDOG error but the gripper (node = joint_num) never
does, despite receiving the identical Send_Watchdog config. Every observation
so far was made THROUGH an RTI restart, so RTI's own boot/tick traffic sat
between a potential trip and the reading — the gripper is the only node that
receives gripper-specific frames (Send_gripper_data_pack with activate=1,
every tick, in the default firmware mode), and if any of those clears a
latched error in firmware, the evidence is wiped ~4 ms after boot while the
round-robin error poll only reaches node 6 after ~84 ms. This probe takes RTI
out of the loop entirely and answers the question at ground truth.

Phases:
  0  read-only   Report each node's currently latched error state. Meaningful
                 when the stack ran and was killed >= 15 s ago: that run armed
                 a 5000 ms watchdog on every node (verified live 2026-07-15),
                 so a node whose watchdog works MUST show watchdog_error=1
                 here. Sends only remote-frame requests.
  A  controlled  Clear errors, verify clean, arm a short watchdog on every
                 probed node with byte-identical frames, stay silent past the
                 timeout, read back who latched. Valid from ANY starting state
                 (arms its own watchdog, trusts no history).
  B  gripper     Only if the gripper latched in phase A: replay each per-tick
                 frame RTI can send to node 6 (firmware-mode idle pack with
                 activate=1 / motor-mode Send_data_pack_1(Current=0) / empty
                 calibrate keep-alive pack) against a freshly latched error,
                 and report which of them, if any, CLEARS it. Whichever frame
                 clears the latch explains the original boot symptom.

Safety / preconditions:
  * RTI.py and supervisor.py must NOT be running (the script refuses).
  * Do NOT run while the gripper is holding a part — a tripping watchdog
    drops the motor to its configured action (limp for 'Idle').
  * The arm must already be at rest; it always is once RTI has been dead a
    few seconds (the joints' own watchdogs have fired by then).
  * The script never commands motion: polls are remote frames; the only data
    frames are Send_Clear_Error, Send_Watchdog, and (phase B, gripper only)
    idle packs with current=0.

Result of the 2026-07-16 hardware run (phases 0+A+B): the gripper watchdog
TRIPS when armed by a lone Send_Watchdog (phase A), NO per-tick frame clears a
latched error (phase B), yet after a real stack run it sits unlatched (phase 0).
So the remaining suspect is RTI's BOOT ARMING — the watchdog frame rides a
28-frame burst to the same node and is followed by 250 Hz tick traffic.
`--replay-boot` chases exactly that: C0 replays the REAL load_gripper_configs
with the REAL XML values, then either bisects which config message kills the
arming, or (if C0 arms fine) adds the tick traffic and splits it.

Examples:
    python3 tools/probe_gripper_watchdog.py                  # phases 0+A(+B)
    python3 tools/probe_gripper_watchdog.py --read-only      # phase 0 only
    python3 tools/probe_gripper_watchdog.py --replay-boot    # phases C/D
    python3 tools/probe_gripper_watchdog.py --setup-can      # bring up can0 first
"""

import argparse
import subprocess
import sys
import time
from pathlib import Path

import Spectral_BLDC as Spectral

# Repo root on sys.path so --replay-boot can import the REAL config parser and
# gripper config loader (the point is to replay them, not to copy them).
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

VERBOSE = False

ERROR_FIELDS = (
    "error", "watchdog_error", "estop_error", "temperature_error",
    "vbus_error", "driver_error", "velocity_error", "current_error",
    "calibrated", "activated",
)


def ensure_stack_stopped():
    """Refuse to touch the bus while RTI (or its supervisor) could be using it."""
    for proc in ("RTI.py", "supervisor.py"):
        result = subprocess.run(["pgrep", "-f", proc], capture_output=True, text=True)
        if result.returncode == 0:
            pids = " ".join(result.stdout.split())
            sys.exit(f"REFUSING to run: {proc} is alive (pid {pids}). "
                     f"Stop the robot stack first — two CAN readers steal each other's replies.")


def setup_can_interface(channel, bitrate):
    subprocess.run(["sudo", "ip", "link", "set", channel, "down"], check=False)
    subprocess.run(
        ["sudo", "ip", "link", "set", channel, "up", "type", "can", "bitrate", str(bitrate)],
        check=True,
    )
    subprocess.run(["sudo", "ifconfig", channel, "txqueuelen", "1000"], check=False)


def drain(comm):
    """Discard anything sitting in the socket buffer so reads are fresh."""
    while True:
        message, _ = comm.receive_can_messages(timeout=0.005)
        if message is None:
            return


def poll_node(comm, motor, node_id, timeout=0.6):
    """Request error state (cmd 26) + temperature from one node.

    Returns a dict of decoded fields plus 'agg' (the error bit that rides the
    CAN ID of the cmd-26 reply), or None if the node never answered.

    Trusts nothing about the bus: socketcan delivers BUS ERROR frames whose
    arbitration id is a bitmask of error classes — 0x34 (ACK|TRX|CRTL) decodes
    as node 0 / cmd 26 under the Spectral id packing, and the library's
    receive path does not filter them. A valid cmd-26 reply must be a data
    frame with dlc == 2; anything else is reported, never decoded.
    """
    drain(comm)
    motor.Send_Respond_State_of_Errors()
    motor.Send_Respond_Temperature()

    fields = None
    temp_value = None
    agg_bit = None
    bus_error_frames = 0
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and not (fields is not None and temp_value is not None):
        message, msg_id = comm.receive_can_messages(timeout=0.05)
        if message is None or msg_id is None:
            continue
        if getattr(message, "is_error_frame", False):
            bus_error_frames += 1
            if VERBOSE:
                print(f"      !! bus ERROR frame 0x{message.arbitration_id:08X} "
                      f"data=[{message.data.hex(' ') if message.data else ''}]")
            continue
        if message.is_remote_frame or msg_id.node_id != node_id:
            continue
        if VERBOSE:
            print(f"      rx node={msg_id.node_id} cmd={msg_id.command_id} "
                  f"err={msg_id.error_bit} dlc={message.dlc} "
                  f"data=[{message.data.hex(' ') if message.data else ''}]")
        if msg_id.command_id == motor.SEND_RESPOND_STATE_OF_ERRORS_ID:
            if message.dlc != 2:
                print(f"      !! node {node_id}: cmd-26 frame with unexpected "
                      f"dlc={message.dlc} — ignored (bus error alias or firmware oddity)")
                continue
            motor.UnpackData(message, msg_id)
            # Snapshot IMMEDIATELY: the library's UnpackData starts with
            # Set_2_none(), which wipes every decoded field on the NEXT call —
            # and watchdog_error is missing from that reset list (upstream bug),
            # so reading after the loop mixes live and stale values.
            fields = {name: getattr(motor, name) for name in ERROR_FIELDS}
            agg_bit = msg_id.error_bit
        elif msg_id.command_id == motor.SEND_RESPOND_TEMPERATURE_ID and message.dlc == 2:
            motor.UnpackData(message, msg_id)
            temp_value = motor.temperature

    if bus_error_frames:
        print(f"      !! node {node_id}: {bus_error_frames} bus ERROR frame(s) during poll — "
              f"if the node also shows 'NO' below, nothing on the bus is ACKing "
              f"(motor power off? wrong channel?)")
    if fields is None:
        return None
    state = dict(fields)
    state["agg"] = agg_bit
    state["temperature"] = temp_value
    return state


def _c(value):
    """Render a possibly-None decoded field — the table must never crash."""
    return "?" if value is None else str(value)


def print_state_table(states, gripper_node):
    header = f"{'node':>4}  {'live':>4}  {'agg':>3}  {'err':>3}  {'WDOG':>4}  " \
             f"{'estop':>5}  {'temp_e':>6}  {'vbus_e':>6}  {'calib':>5}  {'activ':>5}  {'temp':>5}"
    print(header)
    print("-" * len(header))
    for node_id in sorted(states):
        s = states[node_id]
        tag = " (gripper)" if node_id == gripper_node else ""
        if s is None:
            print(f"{node_id:>4}  {'NO':>4}  {'-':>3}  {'-':>3}  {'-':>4}  "
                  f"{'-':>5}  {'-':>6}  {'-':>6}  {'-':>5}  {'-':>5}  {'-':>5}{tag}")
            continue
        print(f"{node_id:>4}  {'yes':>4}  {_c(s['agg']):>3}  {_c(s['error']):>3}  "
              f"{_c(s['watchdog_error']):>4}  {_c(s['estop_error']):>5}  "
              f"{_c(s['temperature_error']):>6}  {_c(s['vbus_error']):>6}  "
              f"{_c(s['calibrated']):>5}  {_c(s['activated']):>5}  "
              f"{_c(s['temperature']):>5}{tag}")


def clear_errors(comm, motors, nodes, repeats=3):
    for _ in range(repeats):
        for node_id in nodes:
            motors[node_id].Send_Clear_Error()
        time.sleep(0.05)
    time.sleep(0.2)
    drain(comm)


def feed_watchdogs(motors, nodes, gripper_node):
    """Reset every node's watchdog counter once, right after a clear.

    Needed because the PREVIOUS stack run may have left a 5000 ms watchdog armed
    and starving — without a feed, a joint re-latches the instant it is cleared
    and the 'verify clean' step reads garbage. Joints get the exact IDLE tick
    frame (zero torque). The gripper gets the firmware idle pack instead, so its
    Controller_mode stays 11 (receiving data_pack_1 would flip it to a motor
    mode and phase A would measure the wrong state).
    """
    for node_id in nodes:
        if node_id == gripper_node:
            motors[node_id].Send_gripper_data_pack(
                position_=0, speed_=0, current_=0,
                activate_=1, action_=0, ESTOP_=0, Release_dir_=0)
        else:
            motors[node_id].Send_data_pack_1(Position=None, Speed=None, Current=0)
    time.sleep(0.1)


def arm_watchdogs(motors, nodes, rate_ms):
    for node_id in nodes:
        motors[node_id].Send_Watchdog(_rate=int(rate_ms), _action="Idle")


def silent_wait(wait_s, rate_ms):
    print(f"  bus SILENT for {wait_s:.0f}s (watchdog rate {rate_ms} ms) ...", flush=True)
    time.sleep(wait_s)


def relatch_gripper(comm, motors, gripper_node, rate_ms, wait_s):
    """Re-arm the gripper watchdog and let it trip again. Returns latched state."""
    clear_errors(comm, motors, [gripper_node])
    arm_watchdogs(motors, [gripper_node], rate_ms)
    silent_wait(wait_s, rate_ms)
    state = poll_node(comm, motors[gripper_node], gripper_node)
    if state is None or state["watchdog_error"] != 1:
        print("  !! could not re-latch the gripper watchdog — phase B aborted here")
        return None
    return state


def phase_b(comm, motors, gripper_node, rate_ms, wait_s):
    """Which per-tick RTI frame, if any, clears a latched gripper error?

    Precondition: gripper watchdog error is latched right now (phase A result).
    """
    gripper = motors[gripper_node]
    frames = [
        ("firmware-mode idle pack (activate=1, current=0 — the boot-default tick frame)",
         lambda: gripper.Send_gripper_data_pack(
             position_=0, speed_=0, current_=0,
             activate_=1, action_=0, ESTOP_=0, Release_dir_=0)),
        ("motor-mode tick frame Send_data_pack_1(Current=0)",
         lambda: gripper.Send_data_pack_1(Position=None, Speed=None, Current=0)),
        ("empty gripper pack (calibrate keep-alive)",
         lambda: gripper.Send_gripper_data_pack()),
    ]

    print("\n=== PHASE B: which per-tick frame clears a latched gripper error? ===")
    results = []
    latched = True  # phase A just proved it
    for label, send in frames:
        if not latched:
            state = relatch_gripper(comm, motors, gripper_node, rate_ms, wait_s)
            if state is None:
                results.append((label, "untested (re-latch failed)"))
                break
            latched = True
        print(f"\n  sending: {label}")
        send()
        time.sleep(0.3)
        state = poll_node(comm, gripper, gripper_node)
        if state is None:
            results.append((label, "no reply"))
            latched = False  # unknown, force re-latch
            continue
        cleared = state["watchdog_error"] == 0
        results.append((label, "CLEARED the latch" if cleared else "latch survived"))
        latched = not cleared

    print("\n  Phase B result matrix:")
    for label, outcome in results:
        print(f"    {outcome:<28} <- {label}")
    print("\n  A frame that clears the latch explains the boot symptom: RTI sends it")
    print("  every 4 ms from tick 1, the round-robin reads node errors only after ~84 ms.")


def _load_rti_gripper_cfg():
    """Parse the same XMLs RTI parses at boot and return its gripper config."""
    from config.xml_parser import load_system_config, load_full_robot_config
    from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR
    _sys = load_system_config(SYSTEM_XML, ROBOTS_DIR)
    cfg = load_full_robot_config(str(_sys["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                 active_gripper=_sys["active_gripper"])
    # RTI arms every drive with system.xml <drive_watchdog_ms> since 2026-10-13 (the gripper XML's value is not read)
    cfg["gripper_watchdog_rate"] = _sys["drive_watchdog_ms"]
    return _sys, cfg


def _replay_config_burst(motors, gripper_node, cfg):
    """Call the REAL load_gripper_configs with RTI's exact boot arguments."""
    from communication.can_message_handlers import load_gripper_configs
    load_gripper_configs(
        motors,
        joint_num=gripper_node,      # used only as the index of the gripper motor
        velocity_limit=cfg["gripper_velocity_limit"],
        current_limit=cfg["gripper_Ilim"],
        KP=cfg["gripper_KP"], KD=cfg["gripper_KD"],
        KPIQ=cfg["gripper_KPIQ"], KIIQ=cfg["gripper_KIIQ"],
        KPV=cfg["gripper_KPV"], KIV=cfg["gripper_KIV"],
        KPP=cfg["gripper_KPP"],
        watchdog_rate=int(cfg["gripper_watchdog_rate"]),   # RTI.py:425 int-cast
        watchdog_action="Idle",
        num_repeats=4,                                     # RTI's _reload_gripper_config default
        voltage_limit=cfg["gripper_voltage_limit"],
    )


def _traffic_then_silence(comm, motors, gripper_node, traffic_s, silence_s,
                          fw_packs=True, polling=True):
    """Emulate RTI's per-tick gripper traffic, then go silent (like a pkill)."""
    gripper = motors[gripper_node]
    poll_cycle = (gripper.Send_Respond_Temperature,
                  gripper.Send_Respond_Voltage,
                  gripper.Send_Respond_State_of_Errors)
    what = " + ".join(p for p, on in (("250 Hz fw packs", fw_packs),
                                      ("~84 ms polling", polling)) if on) or "nothing"
    print(f"    ticking {what} for {traffic_s:.0f}s, then silent {silence_s:.0f}s ...")
    end = time.monotonic() + traffic_s
    tick = 0
    while time.monotonic() < end:
        if fw_packs:
            gripper.Send_gripper_data_pack(
                position_=0, speed_=0, current_=0,
                activate_=1, action_=0, ESTOP_=0, Release_dir_=0)
        if polling and tick % 21 == 0:                     # node 6 cadence in the round-robin
            poll_cycle[(tick // 21) % 3]()
        for _ in range(8):                                 # keep the socket drained
            message, _ = comm.receive_can_messages(timeout=0)
            if message is None:
                break
        tick += 1
        time.sleep(0.004)
    time.sleep(silence_s)


def _latched(comm, motors, gripper_node):
    state = poll_node(comm, motors[gripper_node], gripper_node)
    if state is None:
        print("    !! gripper did not answer the readback")
        return None
    print(f"    -> WDOG={_c(state['watchdog_error'])}  agg={_c(state['agg'])}")
    return state["watchdog_error"] == 1


def replay_boot(comm, motors, gripper_node):
    """Phase C/D: reproduce RTI's boot arming of node 6, then bisect the failure.

    Phase A proved a LONE Send_Watchdog arms this device. RTI's boot instead
    sends it inside a 28-frame burst to the same node and then talks to it at
    250 Hz until the process dies — and after a real stack run the gripper is
    NOT latched. Something in that traffic must break the arming; this replays
    it piecewise.
    """
    _sys, cfg = _load_rti_gripper_cfg()
    rate_ms = int(cfg["gripper_watchdog_rate"])
    silence_s = rate_ms * 2 / 1000.0 + 2.0
    print(f"\n=== PHASE C: replay RTI's boot config burst "
          f"(gripper '{_sys['active_gripper']}', watchdog {rate_ms} ms) ===")

    print("  C0: clear + feed, then the REAL load_gripper_configs (7 msgs x 4 repeats)")
    clear_errors(comm, motors, [gripper_node])
    feed_watchdogs(motors, [gripper_node], gripper_node)
    _replay_config_burst(motors, gripper_node, cfg)
    time.sleep(silence_s)
    tripped = _latched(comm, motors, gripper_node)
    if tripped is None:
        return

    if not tripped:
        print("\n>> REPRODUCED AT C0: the config burst itself breaks the arming.")
        print(">> Bisecting which message kills it (lone watchdog + one candidate each):")
        gripper = motors[gripper_node]
        candidates = (
            ("Send_Limits",         lambda: gripper.Send_Limits(
                Velocity_limit=int(cfg["gripper_velocity_limit"]),
                Current_limit=int(cfg["gripper_Ilim"]))),
            ("Send_Voltage_Limit",  lambda: gripper.Send_Voltage_Limit(
                Voltage_limit=int(cfg["gripper_voltage_limit"]))),
            ("Send_PD_Gains",       lambda: gripper.Send_PD_Gains(
                KP=float(cfg["gripper_KP"]), KD=float(cfg["gripper_KD"]))),
            ("Send_Current_Gains",  lambda: gripper.Send_Current_Gains(
                KPIQ=float(cfg["gripper_KPIQ"]), KIIQ=float(cfg["gripper_KIIQ"]))),
            ("Send_Velocity_Gains", lambda: gripper.Send_Velocity_Gains(
                KPV=float(cfg["gripper_KPV"]), KIV=float(cfg["gripper_KIV"]))),
            ("Send_Position_Gains", lambda: gripper.Send_Position_Gains(
                KPP=float(cfg["gripper_KPP"]))),
        )
        matrix = []
        for name, send in candidates:
            print(f"  bisect: watchdog then {name}")
            clear_errors(comm, motors, [gripper_node])
            feed_watchdogs(motors, [gripper_node], gripper_node)
            motors[gripper_node].Send_Watchdog(_rate=rate_ms, _action="Idle")
            time.sleep(0.02)
            send()
            time.sleep(silence_s)
            tripped = _latched(comm, motors, gripper_node)
            matrix.append((name, tripped))
        print("\n  Bisect matrix (False = this message KILLED the arming):")
        for name, tripped in matrix:
            verdict = "?" if tripped is None else ("armed OK" if tripped else "KILLED ARMING")
            print(f"    {verdict:<14} <- {name}")
        return

    print("\n  C0 tripped — the config burst arms fine. PHASE D: add the tick traffic.")
    print("  D0: config burst, then 10 s of full tick traffic, then silence (the pkill scenario)")
    clear_errors(comm, motors, [gripper_node])
    feed_watchdogs(motors, [gripper_node], gripper_node)
    _replay_config_burst(motors, gripper_node, cfg)
    _traffic_then_silence(comm, motors, gripper_node, 10.0, silence_s)
    tripped = _latched(comm, motors, gripper_node)
    if tripped is None:
        return
    if tripped:
        print("\n>> D0 tripped too: boot burst + tick traffic + kill CANNOT reproduce the")
        print(">> symptom here. Next stop is the real thing: run  candump -ta can0  during an")
        print(">> actual RTI boot + kill and diff node-6 traffic against this replay.")
        return

    print("\n>> REPRODUCED AT D0: the tick traffic breaks the arming. Splitting it:")
    for label, fw, poll in (("D1: fw packs only", True, False),
                            ("D2: polling only", False, True)):
        print(f"  {label}")
        clear_errors(comm, motors, [gripper_node])
        feed_watchdogs(motors, [gripper_node], gripper_node)
        _replay_config_burst(motors, gripper_node, cfg)
        _traffic_then_silence(comm, motors, gripper_node, 10.0, silence_s,
                              fw_packs=fw, polling=poll)
        tripped = _latched(comm, motors, gripper_node)
        verdict = "?" if tripped is None else ("still trips" if tripped else "BREAKS ARMING")
        print(f"    {label}: {verdict}")


def _hold_packs(comm, gripper, seconds, empty):
    """Hold the bus at ~250 Hz with gripper packs (empty or full idle packs)."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if empty:
            gripper.Send_gripper_data_pack()
        else:
            # Full idle pack. current=0 keeps it torqueless — the watchdog is fed
            # by frame RECEIPT (firmware sets watchdog_reset on the handler), not
            # by the field values, so this is faithful where it matters.
            gripper.Send_gripper_data_pack(
                position_=0, speed_=0, current_=0,
                activate_=1, action_=0, ESTOP_=0, Release_dir_=0)
        for _ in range(8):
            message, _ = comm.receive_can_messages(timeout=0)
            if message is None:
                break
        time.sleep(0.004)


def _calibrate_scenario(comm, motors, gripper_node, rate_ms, hold_s,
                        send_calibrate, resume_full):
    """One arm->calibrate?->hold->resume?->silence->read cycle. True = tripped."""
    gripper = motors[gripper_node]
    clear_errors(comm, motors, [gripper_node])
    feed_watchdogs(motors, [gripper_node], gripper_node)
    _hold_packs(comm, gripper, 0.5, empty=False)          # normal running traffic
    gripper.Send_Watchdog(_rate=int(rate_ms), _action="Idle")
    _hold_packs(comm, gripper, 0.5, empty=False)          # armed inside traffic (D0-proven)
    if send_calibrate:
        print("    >>> Send_gripper_calibrate — GRIPPER SWEEPS NOW")
        gripper.Send_gripper_calibrate()
    print(f"    holding EMPTY packs {hold_s:.0f}s (RTI's calibrate hold) ...")
    _hold_packs(comm, gripper, hold_s, empty=True)
    if resume_full:
        print("    resuming FULL packs 2s (the user-clicked-a-button step) ...")
        _hold_packs(comm, gripper, 2.0, empty=False)
    silence_s = rate_ms * 2 / 1000.0 + 2.0
    print(f"    bus SILENT {silence_s:.0f}s (the pkill) ...")
    time.sleep(silence_s)
    return _latched(comm, motors, gripper_node)


def test_calibrate(comm, motors, gripper_node, rate_ms, hold_s):
    """Phase E: does the firmware CALIBRATE routine break the armed watchdog?

    This is the one gripper-only frame every REAL run sends (homing step 2 /
    the GUI calibrate button) that no probe phase has sent. RTI's own comment
    says the calibrate hold uses empty packs to keep the watchdog fed — a
    belief, never a measurement.
    """
    print(f"\n=== PHASE E: calibrate vs watchdog (rate {rate_ms} ms) ===")
    print("  ⚠ The gripper will do its FULL calibration sweep, same as during homing.")
    print("  ⚠ Nothing may be in the jaws. Ctrl+C now to abort.")
    for i in (3, 2, 1):
        print(f"    starting in {i} ...")
        time.sleep(1.0)

    print("  E0: arm -> CALIBRATE -> empty hold -> full resume -> silence")
    tripped = _calibrate_scenario(comm, motors, gripper_node, rate_ms, hold_s,
                                  send_calibrate=True, resume_full=True)
    if tripped is None:
        return
    if tripped:
        print("\n>> E0 tripped: the calibrate sequence does NOT break the watchdog.")
        print(">> Local emulation is exhausted. Confirm against the real stack:")
        print(">>   run 1: start stack, do NOT home, pkill RTI, wait 15 s,")
        print(">>          probe --read-only  -> gripper SHOULD be latched.")
        print(">>   run 2: start stack, HOME, pkill RTI, wait 15 s,")
        print(">>          probe --read-only  -> if unlatched, homing does it — then")
        print(">>          candump -ta can0 during homing and diff node-6 traffic.")
        return

    print("\n>> REPRODUCED AT E0: the calibrate sequence kills the armed watchdog.")
    print(">> Bisecting which component does it:")
    for label, cal, resume in (
            ("E1: NO calibrate (empty hold + resume only)", False, True),
            ("E2: calibrate, NO full-pack resume", True, False)):
        print(f"  {label}")
        tripped = _calibrate_scenario(
            comm, motors, gripper_node, rate_ms, hold_s,
            send_calibrate=cal, resume_full=resume)
        verdict = "?" if tripped is None else ("still trips" if tripped else "BREAKS ARMING")
        print(f"    -> {verdict}")
    print("\n  Reading: if E1 already breaks it, the EMPTY packs disarm the watchdog.")
    print("  If only E2 breaks it, calibrate disarms and the full-pack resume was")
    print("  masking nothing — calibrate itself is the bug. Report to firmware.")


def test_requests(comm, motors, gripper_node, rate_ms):
    """Phase F: do REMOTE-FRAME requests break an armed watchdog?

    After C0/D0 (and the user's calibrate A/B) exonerated every DATA frame the
    real stack sends node 6, the only remaining un-replayed traffic is the
    remote-request family the real boot uses and the replay didn't: the Kt
    fetch (cmd 33) and device-info (cmd 25, selfcheck + round-robin). A request
    disarming a watchdog would be a firmware bug nobody would think to test.
    """
    gripper = motors[gripper_node]
    requests = (
        ("Send_Respond_Kt (cmd 33 — boot Kt fetch)", gripper.Send_Respond_Kt),
        ("Send_Respond_Device_Info (cmd 25 — selfcheck/round-robin)",
         gripper.Send_Respond_Device_Info),
        ("Send_Respond_Encoder_data (cmd 28)", gripper.Send_Respond_Encoder_data),
        ("Send_Respond_Iq_data (cmd 27)", gripper.Send_Respond_Iq_data),
    )
    silence_s = rate_ms * 2 / 1000.0 + 2.0
    print(f"\n=== PHASE F: remote requests vs armed watchdog (rate {rate_ms} ms) ===")

    print("  F0: arm, fire ALL remote requests, silence, read")
    clear_errors(comm, motors, [gripper_node])
    feed_watchdogs(motors, [gripper_node], gripper_node)
    gripper.Send_Watchdog(_rate=int(rate_ms), _action="Idle")
    time.sleep(0.02)
    for _, send in requests:
        send()
        time.sleep(0.02)
    drain(comm)
    print(f"    bus SILENT {silence_s:.0f}s ...")
    time.sleep(silence_s)
    tripped = _latched(comm, motors, gripper_node)
    if tripped is None:
        return
    if tripped:
        print("\n>> F0 tripped: remote requests are innocent too. Every frame type the")
        print(">> stack can send node 6 now arms/keeps a working watchdog in isolation.")
        print(">> ENDGAME — capture the real thing and diff it:")
        print(">>   term1: candump -ta can0 > /tmp/gripper_boot_kill.log")
        print(">>   term2: start the stack, wait ~20 s (no homing, no GUI clicks),")
        print(">>          pkill -f RTI.py, wait 15 s, Ctrl+C the candump")
        print(">>   then:  python3 tools/probe_gripper_watchdog.py --read-only")
        print(">>          python3 tools/candump_node_analyze.py /tmp/gripper_boot_kill.log --node 6")
        return

    print("\n>> REPRODUCED AT F0: a remote request breaks the arming. Bisecting:")
    matrix = []
    for name, send in requests:
        print(f"  bisect: watchdog then {name}")
        clear_errors(comm, motors, [gripper_node])
        feed_watchdogs(motors, [gripper_node], gripper_node)
        gripper.Send_Watchdog(_rate=int(rate_ms), _action="Idle")
        time.sleep(0.02)
        send()
        drain(comm)
        time.sleep(silence_s)
        matrix.append((name, _latched(comm, motors, gripper_node)))
    print("\n  Bisect matrix (False = this request KILLED the arming):")
    for name, tripped in matrix:
        verdict = "?" if tripped is None else ("armed OK" if tripped else "KILLED ARMING")
        print(f"    {verdict:<14} <- {name}")


def _feed_once(motors, node_id, gripper_node):
    """One tick's worth of feeding: exactly what RTI sends this node each tick."""
    if node_id == gripper_node:
        motors[node_id].Send_gripper_data_pack(
            position_=0, speed_=0, current_=0,
            activate_=1, action_=0, ESTOP_=0, Release_dir_=0)
    else:
        motors[node_id].Send_data_pack_1(Position=None, Speed=None, Current=0)


def _agg_bit(comm, motor, node_id, timeout=0.3):
    """Live aggregate fault bit off the CAN id of a cmd-26 reply, or None."""
    state = poll_node(comm, motor, node_id, timeout=timeout)
    return None if state is None else state["agg"]


def _ensure_latched(comm, motors, node_id, gripper_node, rate_ms):
    """Make sure node has a latched watchdog fault; arm+starve if it doesn't."""
    agg = _agg_bit(comm, motors[node_id], node_id)
    if agg == 1:
        return True
    print(f"    node {node_id}: no fault latched — arming {rate_ms} ms and starving ...")
    clear_errors(comm, motors, [node_id])
    _feed_once(motors, node_id, gripper_node)
    time.sleep(0.05)
    motors[node_id].Send_Watchdog(_rate=int(rate_ms), _action="Idle")
    time.sleep(rate_ms / 1000.0 + 2.0)
    return _agg_bit(comm, motors[node_id], node_id) == 1


def test_clear(comm, motors, gripper_node, rate_ms):
    """Phase G: measure what Send_Clear_Error ACTUALLY does to a latched fault.

    Observed on the stack (2026-07-16): after one Clear Errors press, several
    drivers still asserted their fault 40-50 ms later (live CAN-id bit AND a
    fresh cmd-26 both faulted) despite 3 clear frames each and continuous
    feeding — while other, identically-treated drivers cleared. This phase
    measures, per node, with RTI out of the loop:
      G1  clear-while-fed: feed 1 s -> clear x3 -> keep feeding -> poll the
          live bit at ~10/50/100/300/700 ms. Reports clear latency, and how
          many volleys it takes if one is not enough.
      G2  watchdog state after clearing, measured under TRUE silence (no
          frames at all during the wait — a polled wait would feed the dog if
          firmware resets it on any received frame): G2a clear -> silence ->
          one read (trip = clear keeps the watchdog armed); G2b re-arm control
          (separates clear-disarms from node-never-trips).
    """
    nodes = sorted(motors)
    print(f"\n=== PHASE G: clear-error behaviour under feeding (rate {rate_ms} ms) ===")
    for node_id in nodes:
        print(f"\n  node {node_id}{' (gripper)' if node_id == gripper_node else ''}:")
        if not _ensure_latched(comm, motors, node_id, gripper_node, rate_ms):
            print("    could not obtain a latched fault — skipping")
            continue

        # --- G1: clear while feeding, measure how long the fault survives ---
        end = time.monotonic() + 1.0
        while time.monotonic() < end:                    # 1 s of normal feeding
            _feed_once(motors, node_id, gripper_node)
            drain(comm)
            time.sleep(0.004)
        volleys = 0
        cleared_after = None
        while volleys < 4 and cleared_after is None:
            volleys += 1
            for _ in range(3):                           # RTI's CLEAR_ERROR_SEND_REPEATS
                motors[node_id].Send_Clear_Error()
            t_clear = time.monotonic()
            for checkpoint_ms in (10, 50, 100, 300, 700):
                while (time.monotonic() - t_clear) * 1000 < checkpoint_ms:
                    _feed_once(motors, node_id, gripper_node)
                    drain(comm)
                    time.sleep(0.004)
                agg = _agg_bit(comm, motors[node_id], node_id)
                print(f"    volley {volleys} +{checkpoint_ms:>3} ms: live fault bit = {_c(agg)}")
                if agg == 0:
                    cleared_after = checkpoint_ms
                    break
        if cleared_after is None:
            print(f"    !! fault SURVIVED {volleys} volleys while fed — clear is not working here")
            continue
        print(f"    -> cleared within {cleared_after} ms on volley {volleys}"
              f"{' (ONE press would have failed on the stack!)' if volleys > 1 else ''}")

        # --- G2: does the watchdog survive the clear? TRUE silence, then ONE read.
        # The first version of this check polled every 250 ms during the wait — if
        # the firmware resets the watchdog on ANY received frame (not just data
        # packs), those polls FED it and faked a "disarmed after clear" verdict.
        # True silence removes the ambiguity; a single read afterwards suffices
        # because a tripped fault LATCHES.
        silence_s = rate_ms / 1000.0 + 2.0
        print(f"    G2a: TRUE silence {silence_s:.0f}s after the clear, then one read ...")
        time.sleep(silence_s)
        agg = _agg_bit(comm, motors[node_id], node_id)
        if agg == 1:
            print("    -> TRIPPED: the watchdog stayed armed through the clear (healthy),")
            print("       AND remote requests must feed the watchdog (the old polling G2")
            print("       kept it alive — probe artifact, firmware fine).")
            continue
        print(f"    -> no trip after clear (bit={_c(agg)}). G2b control: re-arm, same silence ...")
        motors[node_id].Send_Watchdog(_rate=int(rate_ms), _action="Idle")
        time.sleep(silence_s)
        agg = _agg_bit(comm, motors[node_id], node_id)
        if agg == 1:
            print("    -> re-armed node trips fine => the CLEAR genuinely left the watchdog")
            print("       DISARMED (one-shot behaviour) — a firmware-side finding.")
        else:
            print(f"    -> still no trip even after re-arm (bit={_c(agg)}) — this node will not")
            print("       trip in this state at all; needs a closer look.")

    print("\n  Reading: G1 latency > ~50 ms or volleys > 1 explains the stack symptom directly")
    print("  (RTI wipes the latch 40 ms after the press). G2a tripping = clear keeps the")
    print("  watchdog armed and requests feed it; G2a quiet + G2b tripping = clear disarms.")
    print("  Note: probed nodes are left with a latched fault — expect errors at next boot.")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--nodes", type=int, nargs="+", default=[0, 5, 6],
                        help="CAN node ids to probe (default: 0 5 6 — J1 SW103, J6 SW101, gripper)")
    parser.add_argument("--gripper-node", type=int, default=6,
                        help="which probed node is the gripper (default: 6)")
    parser.add_argument("--rate-ms", type=int, default=3000,
                        help="watchdog rate to arm in phase A (default: 3000 ms)")
    parser.add_argument("--wait-s", type=float, default=None,
                        help="silent window after arming (default: rate*2 + 2 s)")
    parser.add_argument("--read-only", action="store_true",
                        help="phase 0 only: report latched state, send nothing but requests")
    parser.add_argument("--skip-phase-b", action="store_true",
                        help="do not test which frames clear a latched gripper error")
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--bitrate", type=int, default=1000000)
    parser.add_argument("--setup-can", action="store_true",
                        help="bring the CAN interface up before probing")
    parser.add_argument("--verbose", action="store_true",
                        help="dump every received frame (diagnose aliasing / silence)")
    parser.add_argument("--replay-boot", action="store_true",
                        help="skip phases 0/A/B; replay RTI's boot config burst + tick "
                             "traffic on the gripper node and bisect what breaks the arming")
    parser.add_argument("--test-calibrate", action="store_true",
                        help="phase E: arm the watchdog, run the firmware CALIBRATE "
                             "sequence exactly as RTI does, then kill traffic and see if "
                             "it still trips. ⚠ THE GRIPPER PHYSICALLY SWEEPS ITS STROKE")
    parser.add_argument("--calibrate-hold-s", type=float, default=8.0,
                        help="empty-pack hold after calibrate, covers the sweep (default 8)")
    parser.add_argument("--test-requests", action="store_true",
                        help="phase F: fire the remote-frame requests the real boot sends "
                             "(Kt / device-info / encoder / iq) against an armed watchdog")
    parser.add_argument("--test-clear", action="store_true",
                        help="phase G: measure Send_Clear_Error against a latched fault "
                             "while feeding (clear latency, volleys needed, post-clear "
                             "watchdog counter state). Run after a pkill, or it arms+starves "
                             "each probed node itself")
    args = parser.parse_args()

    global VERBOSE
    VERBOSE = args.verbose

    wait_s = args.wait_s if args.wait_s is not None else args.rate_ms * 2 / 1000.0 + 2.0
    if wait_s * 1000 <= args.rate_ms:
        sys.exit(f"--wait-s {wait_s} does not exceed --rate-ms {args.rate_ms}: nothing can trip")

    ensure_stack_stopped()
    if args.setup_can:
        setup_can_interface(args.channel, args.bitrate)

    comm = Spectral.CanCommunication(bustype="socketcan", channel=args.channel,
                                     bitrate=args.bitrate)
    motors = {n: Spectral.SpectralCAN(node_id=n, communication=comm) for n in args.nodes}
    try:
        run_phases(args, comm, motors, wait_s)
    finally:
        bus = getattr(comm, "bus", None)
        if bus is not None:
            bus.shutdown()


def run_phases(args, comm, motors, wait_s):
    if args.replay_boot:
        replay_boot(comm, motors, args.gripper_node)
        return
    if args.test_calibrate:
        test_calibrate(comm, motors, args.gripper_node, args.rate_ms,
                       args.calibrate_hold_s)
        return
    if args.test_requests:
        test_requests(comm, motors, args.gripper_node, args.rate_ms)
        return
    if args.test_clear:
        test_clear(comm, motors, args.gripper_node, args.rate_ms)
        return

    # ---- Phase 0: read-only snapshot ------------------------------------
    print("=== PHASE 0: current latched state (read-only) ===")
    print("Meaningful if the stack ran and was killed >= 15 s ago: that run armed a")
    print("5000 ms watchdog everywhere, so an unfed node that CAN trip shows WDOG=1 here.\n")
    states = {n: poll_node(comm, motors[n], n) for n in args.nodes}
    print_state_table(states, args.gripper_node)

    g_state = states.get(args.gripper_node)
    if g_state is not None and g_state["watchdog_error"] == 1:
        print("\n>> Gripper HAS a latched watchdog error right now. The driver trips fine —")
        print(">> something in RTI's boot/tick traffic clears it before the first error poll.")
        print(">> Phase A/B will confirm and identify the clearing frame.")
    if args.read_only:
        return

    # ---- Phase A: controlled A/B -----------------------------------------
    print(f"\n=== PHASE A: controlled trip test (rate {args.rate_ms} ms, wait {wait_s:.0f} s) ===")
    print("  clearing errors on probed nodes ...")
    clear_errors(comm, motors, args.nodes)
    feed_watchdogs(motors, args.nodes, args.gripper_node)
    pre = {n: poll_node(comm, motors[n], n) for n in args.nodes}
    for node_id, s in sorted(pre.items()):
        if s is None:
            print(f"  !! node {node_id} not answering — its result will be meaningless")
        elif s["watchdog_error"] or s["error"]:
            print(f"  !! node {node_id} still latched after clear (err={s['error']} "
                  f"wdog={s['watchdog_error']}) — trip detection on it is compromised")
    print("  arming identical watchdogs ...")
    arm_watchdogs(motors, args.nodes, args.rate_ms)
    silent_wait(wait_s, args.rate_ms)
    post = {n: poll_node(comm, motors[n], n) for n in args.nodes}
    print()
    print_state_table(post, args.gripper_node)

    print("\n=== PHASE A verdicts ===")
    tripped = {}
    for node_id in sorted(post):
        s = post[node_id]
        if s is None:
            print(f"  node {node_id}: NO REPLY — cannot judge")
            continue
        tripped[node_id] = (s["watchdog_error"] == 1)
        role = "gripper" if node_id == args.gripper_node else "joint"
        print(f"  node {node_id} ({role}): watchdog {'TRIPPED' if tripped[node_id] else 'did NOT trip'}")

    joints = [n for n in tripped if n != args.gripper_node]
    g_tripped = tripped.get(args.gripper_node)
    if g_tripped is None:
        print("\n>> Gripper never answered — check power/bus and re-run.")
        return
    if g_tripped and joints and all(tripped[n] for n in joints):
        print("\n>> EVERY node trips under identical treatment: the gripper watchdog WORKS.")
        print(">> The boot symptom is an observation artifact — RTI traffic clears the latch")
        print(">> before the round-robin reads it. Phase B identifies the frame.")
        if not args.skip_phase_b:
            phase_b(comm, motors, args.gripper_node, args.rate_ms, wait_s)
    elif not g_tripped and joints and all(tripped[n] for n in joints):
        print("\n>> Joints trip, gripper does NOT — with RTI fully out of the loop and")
        print(">> byte-identical arming frames. This is DEVICE-SIDE: that driver either")
        print(">> ignores/does not apply SEND_WATCHDOG (cmd 15) or never runs the timer.")
        print(">> Next stop is firmware: check whether the watchdog check is gated on")
        print(">> Controller_mode (gripper sits at 11) or on an NVM/config flag.")
    else:
        print("\n>> Mixed/unexpected pattern — re-run with --wait-s raised; if it persists,")
        print(">> the probe assumptions need a second look (units of rate? requests feeding")
        print(">> the watchdog?).")


if __name__ == "__main__":
    main()
