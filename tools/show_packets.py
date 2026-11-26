#!/usr/bin/env python3
"""Show the robot's telemetry packets live, as readable text (2026-11-26, TODO/p4_recording_plan.md 2B).

The robot sends one packet a tick, text or binary (`set_telemetry_format`); a binary packet cannot be read by eye. This
prints any of them as "tag: values" lines, from the robot's recorder link (TCP port 30004, every tick) or its UDP
telemetry (port 30003 on this PC: the robot's telemetry destination must be this PC or broadcast). Runs on the robot
or on a PC with the robotclient package.

    tools/show_packets.py 192.168.1.50                 # one packet a second, every tag
    tools/show_packets.py 192.168.1.50 --tags j_p,g_p,tick --every 0.1
    tools/show_packets.py 192.168.1.50 --udp --count 5
    tools/show_packets.py 127.0.0.1 --raw              # the packet's own bytes too (the first 256)

The robot serves one TCP recorder link at a time: while a recorder is connected, use --udp here. Exit code 0 after
--count packets or Ctrl-C, 1 when no packet came (the reason printed).
"""
from __future__ import annotations

import argparse
import math
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robotclient.telemetry import TelemetryClient, decode_packet, is_binary  # noqa: E402


def _fmt(v) -> str:
    if isinstance(v, list):
        return ", ".join(_fmt(x) for x in v)
    if isinstance(v, float):
        if math.isnan(v):
            return "nan"
        return f"{v:.0f}" if v.is_integer() and abs(v) < 1e15 else f"{v:.6g}"
    return str(v)


def show(raw: bytes, tags, show_raw: bool) -> str:
    """One packet as lines: its format and size, then "tag: values" (only `tags` when given)."""
    state = decode_packet(raw)
    lines = [f"--- {'binary' if is_binary(raw) else 'text'} packet, {len(raw)} bytes, {len(state)} tags"]
    for tag, value in state.items():
        if tags and tag not in tags:
            continue
        lines.append(f"{tag:>14}: {_fmt(value)}")
    if show_raw:
        lines.append(f"{'raw':>14}: {raw[:256]!r}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("robot", help="the robot's IP address")
    ap.add_argument("--udp", action="store_true", help="listen to the UDP telemetry (port 30003) instead of TCP 30004")
    ap.add_argument("--tags", default="", help="comma-separated tags to show (default: all)")
    ap.add_argument("--every", type=float, default=1.0, help="seconds between packets shown (default 1)")
    ap.add_argument("--count", type=int, default=0, help="stop after this many packets shown (default: Ctrl-C)")
    ap.add_argument("--raw", action="store_true", help="also print the packet's first 256 bytes")
    args = ap.parse_args(argv)
    tags = {t.strip() for t in args.tags.split(",") if t.strip()}
    tel = TelemetryClient(args.robot, transport="udp" if args.udp else "tcp")
    try:
        tel.connect()
    except OSError as e:
        print(f"could not {'listen on UDP' if args.udp else 'connect to TCP'} {tel.port}: {e}")
        return 1
    shown, next_t = 0, 0.0
    try:
        while not args.count or shown < args.count:
            try:
                raw = tel.read_raw(timeout=3.0)
            except socket.timeout:
                print("no packet for 3 s (is RTI running? over UDP: is this PC the telemetry destination?)")
                return 1
            except ConnectionError as e:
                print(e)
                return 1
            now = time.monotonic()
            if now < next_t:
                continue
            next_t = now + args.every
            print(show(raw, tags, args.raw), flush=True)
            shown += 1
    except KeyboardInterrupt:
        pass
    finally:
        tel.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
