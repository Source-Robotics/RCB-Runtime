#!/usr/bin/env python3
"""Read motor Kt values over CAN using only command 33 remote frames.

Stop RTI.py before running this, otherwise RTI may consume the replies first.

Examples:
    python3 tools/debug_read_kt.py
    python3 tools/debug_read_kt.py --nodes 0 1 2 3 4 5 --timeout 0.5
    python3 tools/debug_read_kt.py --setup-can
"""

import argparse
import struct
import subprocess
import sys
import time

import can


KT_COMMAND_ID = 33


def combine_can_id(node_id, command_id, error_bit=0):
    """Match Spectral_BLDC CAN ID packing: node:4 bits, cmd:6 bits, err:1 bit."""
    return ((node_id & 0xF) << 7) | ((command_id & 0x3F) << 1) | (error_bit & 0x1)


def extract_can_id(arbitration_id):
    node_id = (arbitration_id >> 7) & 0xF
    command_id = (arbitration_id >> 1) & 0x3F
    error_bit = arbitration_id & 0x1
    return node_id, command_id, error_bit


def setup_can_interface(channel, bitrate):
    subprocess.run(["sudo", "ip", "link", "set", channel, "down"], check=False)
    subprocess.run(
        ["sudo", "ip", "link", "set", channel, "up", "type", "can", "bitrate", str(bitrate)],
        check=True,
    )
    subprocess.run(["sudo", "ifconfig", channel, "txqueuelen", "1000"], check=False)


def format_data(data):
    return " ".join(f"{byte:02X}" for byte in data)


def read_kt_for_node(bus, node_id, timeout, dlc, verbose):
    request_id = combine_can_id(node_id, KT_COMMAND_ID)
    request = can.Message(
        arbitration_id=request_id,
        is_remote_frame=True,
        is_extended_id=False,
        dlc=dlc,
    )

    print(f"J{node_id + 1} node={node_id}: sending RTR id=0x{request_id:03X} cmd={KT_COMMAND_ID} dlc={dlc}")
    bus.send(request)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        message = bus.recv(timeout=min(0.05, remaining))
        if message is None:
            continue

        rx_node, rx_cmd, rx_error = extract_can_id(message.arbitration_id)
        data_hex = format_data(message.data)

        if verbose:
            frame_type = "RTR" if message.is_remote_frame else "DATA"
            print(
                f"  rx {frame_type} id=0x{message.arbitration_id:03X} "
                f"node={rx_node} cmd={rx_cmd} err={rx_error} "
                f"dlc={message.dlc} data=[{data_hex}]"
            )

        if rx_node != node_id or rx_cmd != KT_COMMAND_ID:
            continue

        if message.is_remote_frame:
            print("  got matching remote frame, not a data response")
            continue

        if message.dlc != 4:
            print(f"  got matching response with wrong dlc={message.dlc}, data=[{data_hex}]")
            return None

        kt_value = struct.unpack(">f", bytes(message.data[:4]))[0]
        print(f"  Kt = {kt_value:.9g} Nm/A, raw=[{data_hex}], error_bit={rx_error}")
        return kt_value

    print("  no Kt response before timeout")
    return None


def main():
    parser = argparse.ArgumentParser(description="Read Kt from Spectral BLDC motors over CAN.")
    parser.add_argument("--channel", default="can0", help="SocketCAN channel, default: can0")
    parser.add_argument("--bitrate", type=int, default=1_000_000, help="CAN bitrate used with --setup-can")
    parser.add_argument("--setup-can", action="store_true", help="Bring CAN interface down/up before reading")
    parser.add_argument("--nodes", type=int, nargs="+", default=[0, 1, 2, 3, 4, 5], help="CAN node IDs to query")
    parser.add_argument("--timeout", type=float, default=1.0, help="Timeout per node in seconds")
    parser.add_argument("--dlc", type=int, default=4, help="RTR DLC/requested length, default: 4")
    parser.add_argument("--verbose", action="store_true", help="Print every received frame while waiting")
    args = parser.parse_args()

    if not 0 <= args.dlc <= 8:
        raise SystemExit("--dlc must be between 0 and 8")

    if args.setup_can:
        setup_can_interface(args.channel, args.bitrate)

    try:
        bus = can.interface.Bus(channel=args.channel, interface="socketcan", bitrate=args.bitrate)
    except Exception as exc:
        raise SystemExit(f"Failed to open {args.channel}: {exc}") from exc

    print(f"Reading Kt on {args.channel}. Stop RTI.py first so replies are not consumed by RTI.")
    results = {}
    try:
        for node_id in args.nodes:
            results[node_id] = read_kt_for_node(bus, node_id, args.timeout, args.dlc, args.verbose)
    finally:
        bus.shutdown()

    good = {node: kt for node, kt in results.items() if kt is not None}
    print("\nSummary:")
    for node_id in args.nodes:
        kt = results[node_id]
        value = "NO RESPONSE" if kt is None else f"{kt:.9g} Nm/A"
        print(f"  node {node_id}: {value}")

    return 0 if len(good) == len(args.nodes) else 1


if __name__ == "__main__":
    sys.exit(main())
