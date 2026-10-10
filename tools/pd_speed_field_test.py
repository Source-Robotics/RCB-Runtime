#!/usr/bin/env python3
"""Does Send_data_pack_PD's Speed field reach the firmware's PD law? BENCH TEST.

Reading the firmware source says no. IN_DATA_PACK_PD unpacks bytes 3-5 into
PID.Feedforward_speed, but PD_mode() reads PID.Velocity_setpoint -- a different field that
nothing in mode 4 ever writes:

    communication_CAN.cpp, case IN_DATA_PACK_PD:
        PID.Position_setpoint   = three_bytes_to_int(temp_buffer);    // used
        PID.Feedforward_speed   = three_bytes_to_int(temp_buffer1);   // <- written here
        PID.Feedforward_current = two_bytes_to_int(temp_buffer2);     // used

    motor_control.cpp, PD_mode():
        Iq_setpoint = KP*(Position_setpoint - Position_Ticks)
                    + KD*(Velocity_setpoint - Velocity_Filter)        // <- read here
                    + Feedforward_current;

Identical in STEPFOC and Spectral BLDC, and FIRMWARE_VERSION is 106 in both -- the version
flashed on the arm. But that is source reading, and source reading has been wrong before in
this project, so this settles it on metal.

THE EXPERIMENT, in three parts, and part 0 is not optional:

  0. CONTROL -- offset Position and confirm the motor pushes back. This proves PD mode is
     actually engaged and the instrument can detect a change AT ALL. Without it, "A and B
     look the same" is unreadable: it could mean the Speed field is dead, or it could mean
     PD mode never started. Four null results in this project have already been mistaken
     for findings; this is the guard against a fifth.
  A. Speed = 0        while holding position.
  B. Speed = +LARGE   while holding the SAME position.

If Speed reaches the law, KD*(Speed - actual) is a large extra torque term and B's current
must differ from A's. If the field is discarded, A and B are identical.

REQUIREMENTS
  * RTI.py must be DOWN. This script owns can0 and would fight the runtime for it.
  * ONE motor powered on the bus, SECURELY MOUNTED, shaft free to turn.
  * The shaft MAY move during part 0 and, if the Speed field works, during part B.

    python3 tools/pd_speed_field_test.py --node 0
    python3 tools/pd_speed_field_test.py --node 0 --offset 400 --speed 40000
"""
import argparse
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import Spectral_BLDC as Spectral

RESPOND_1 = Spectral.SpectralCAN.RESPOND_DATA_PACK_1_ID


def rti_alive():
    """True if RTI.py is running -- its loop tick advances."""
    try:
        with open("/dev/shm/loop_tick", "rb") as f:
            a = f.read(8)
        time.sleep(0.15)
        with open("/dev/shm/loop_tick", "rb") as f:
            b = f.read(8)
        return a != b
    except FileNotFoundError:
        return False


def read_state(comm, motor, node, timeout=0.05):
    """Drain replies and return the newest (position, speed, current), or None.

    IN_DATA_PACK_PD ends with Data_pack_1_CAN(), so a PD command is itself the poll -- no
    Send_data_pack_1 is needed, which matters because data_pack_1 would set
    Controller_mode back to 1 or 2 and kick the driver straight out of PD mode.
    """
    got = None
    end = time.time() + timeout
    while time.time() < end:
        message, msg_id = comm.receive_can_messages(timeout=0.005)
        if message is None or msg_id is None or msg_id.node_id != node:
            continue
        if msg_id.command_id != RESPOND_1:
            continue
        motor.UnpackData(message, msg_id)
        if motor.position is not None:
            got = (float(motor.position), float(motor.speed), float(motor.current))
    return got


def run_phase(comm, motor, node, pos, speed, current, seconds, rate=200.0):
    """Stream one PD setpoint and collect the replies it triggers."""
    period = 1.0 / rate
    samples = []
    t0, nxt = time.time(), time.time()
    while time.time() - t0 < seconds:
        motor.Send_data_pack_PD(Position=int(pos), Speed=int(speed), Current=int(current))
        st = read_state(comm, motor, node, timeout=period * 0.6)
        if st:
            samples.append(st)
        nxt += period
        slp = nxt - time.time()
        if slp > 0:
            time.sleep(slp)
    return samples


def summarise(tag, samples):
    if len(samples) < 5:
        print(f"  {tag:<28} NO DATA ({len(samples)} samples)")
        return None
    pos = [s[0] for s in samples]
    spd = [s[1] for s in samples]
    cur = [s[2] for s in samples]
    n = len(samples) // 3
    out = (statistics.mean(cur[n:]), statistics.pstdev(cur[n:]),
           statistics.mean(pos[n:]), statistics.mean(spd[n:]))
    print(f"  {tag:<28} current {out[0]:9.1f} +/-{out[1]:6.1f} mA   "
          f"pos {out[2]:10.0f}   speed {out[3]:9.1f}")
    return out


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--node", type=int, default=0, help="CAN node id of the motor under test")
    p.add_argument("--seconds", type=float, default=2.5, help="per phase")
    p.add_argument("--offset", type=int, default=400,
                   help="encoder ticks for the CONTROL phase. Small: 400/16384 of a motor rev")
    p.add_argument("--speed", type=int, default=40000,
                   help="ticks/s for the Speed-field phases. Large on purpose -- if the "
                        "field is live, KD*(this - 0) must be unmissable")
    p.add_argument("--go", action="store_true", help="required: this ENERGISES a motor")
    a = p.parse_args()

    print("=" * 84)
    print("PD MODE: does the Speed field reach the control law?")
    print("=" * 84)
    if rti_alive():
        print("ABORT: RTI.py is RUNNING. It owns can0 and this script would fight it.")
        print("       Stop the runtime first (this clears homing).")
        return 1
    if not a.go:
        print("Refusing without --go. This ENERGISES a motor in PD mode; the shaft may")
        print("move during the control phase and, if the Speed field works, during phase B.")
        print("Motor must be SECURELY MOUNTED with the shaft free to turn.")
        return 1

    comm = Spectral.CanCommunication(bustype="socketcan", channel="can0", bitrate=1000000)
    motor = Spectral.SpectralCAN(node_id=a.node, communication=comm)
    try:
        # Wake the driver and find out where it is, using a PD packet with zero offset so
        # the very first command cannot make it jump.
        motor.Send_data_pack_PD(Position=0, Speed=0, Current=0)
        time.sleep(0.05)
        while comm.receive_can_messages(timeout=0)[0] is not None:
            pass
        st = None
        for _ in range(40):
            motor.Send_data_pack_PD(Position=0, Speed=0, Current=0)
            st = read_state(comm, motor, a.node, timeout=0.05)
            if st:
                break
        if st is None:
            print(f"ABORT: no reply from node {a.node}. Check power, wiring, termination,")
            print("       and that the node id is right.")
            return 1
        here = int(st[0])
        print(f"\n  node {a.node} is alive at position {here} ticks\n")

        # Settle AT the current position so nothing is moving before we measure.
        run_phase(comm, motor, a.node, here, 0, 0, 1.0)

        print("PHASE 0 -- CONTROL. Offset POSITION and confirm the motor pushes back.")
        print("  If this shows no change, PD mode is not engaged and nothing below is")
        print("  readable. Verify the instrument before trusting a null result.\n")
        base = summarise("hold, Speed=0 (baseline)",
                         run_phase(comm, motor, a.node, here, 0, 0, a.seconds))
        ctrl = summarise(f"POSITION +{a.offset} ticks",
                         run_phase(comm, motor, a.node, here + a.offset, 0, 0, a.seconds))
        run_phase(comm, motor, a.node, here, 0, 0, 1.0)          # back to the start

        if base is None or ctrl is None:
            print("\nABORT: no usable samples.")
            return 1
        dctrl = abs(ctrl[0] - base[0])
        print(f"\n  control delta: {dctrl:.1f} mA")
        if dctrl < 20.0:
            print("  *** PD MODE IS NOT RESPONDING TO POSITION EITHER. ***")
            print("      KP may be zero, the driver may be faulted, or mode 4 never engaged.")
            print("      Fix that before reading anything into the Speed result.")
            return 2
        print("  OK -- PD mode is live and the instrument can see a change.\n")

        print("PHASE A/B -- the actual question. SAME position, different Speed.\n")
        a0 = summarise("Speed = 0", run_phase(comm, motor, a.node, here, 0, 0, a.seconds))
        ap = summarise(f"Speed = +{a.speed}",
                       run_phase(comm, motor, a.node, here, a.speed, 0, a.seconds))
        an = summarise(f"Speed = -{a.speed}",
                       run_phase(comm, motor, a.node, here, -a.speed, 0, a.seconds))
        run_phase(comm, motor, a.node, here, 0, 0, 0.5)
        motor.Send_data_pack_1(Position=None, Speed=None, Current=0)   # leave it limp-safe

        if a0 is None or ap is None or an is None:
            print("\nABORT: no usable samples in phase A/B.")
            return 1

        dpos = abs(ap[0] - a0[0])
        dneg = abs(an[0] - a0[0])
        print("\n" + "=" * 84)
        print(f"  control (POSITION offset) changed current by  {dctrl:8.1f} mA")
        print(f"  Speed = +{a.speed:<8} changed current by       {dpos:8.1f} mA")
        print(f"  Speed = -{a.speed:<8} changed current by       {dneg:8.1f} mA")
        print("=" * 84)
        biggest = max(dpos, dneg)
        if biggest < max(5.0, dctrl * 0.05):
            print("\n  VERDICT: the Speed field is DISCARDED.")
            print("  A position offset moves the current; a huge speed command does not.")
            print("  IN_DATA_PACK_PD writes Feedforward_speed; PD_mode() reads Velocity_setpoint.")
            print("  Fix ONLY the PD case -- communication_CAN.cpp inside `case IN_DATA_PACK_PD`:")
            print("      - PID.Feedforward_speed = three_bytes_to_int(temp_buffer1);")
            print("      + PID.Velocity_setpoint = three_bytes_to_int(temp_buffer1);")
            print("  Do NOT apply the same edit to IN_DATA_PACK_1. That site looks identical")
            print("  (the PD case is a copy of it) but Feedforward_speed is CORRECT there:")
            print("  Position_mode() consumes it as the cascade feedforward,")
            print("      PID.Velocity_setpoint = PID.P_errSum + PID.Feedforward_speed;")
            print("  so changing it would break position mode to fix PD mode.")
            print("")
            print("  CONFIRM THE FIX WITHOUT FLASHING: the UART command PDV writes")
            print("  Velocity_setpoint directly and is not affected by this bug. Hold PD mode")
            print("  over CAN, send `PDV 40000` on the serial port, and watch the current. If")
            print("  it moves, the field assignment is the only thing wrong.")
            return 0
        print("\n  VERDICT: the Speed field DOES reach the law.")
        print("  The source reading was wrong -- Velocity_setpoint is being written")
        print("  somewhere this analysis missed. Step 3 is NOT gated on a firmware fix.")
        return 0
    finally:
        try:
            motor.Send_data_pack_1(Position=None, Speed=None, Current=0)
            comm.bus.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
