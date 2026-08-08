"""Validate RTI's CAN frame-age reading with an independent socket.

Run WHILE RTI is running:   python3 tools/check_can_frame_age.py

socketcan is a broadcast interface: every bound socket gets its own copy of every frame,
with its own receive buffer. So this opens a SECOND socket on the same bus and measures the
same thing RTI measures. The two readings answer different halves of the question:

  fresh socket ~0.1-1 ms  AND  RTI ~50 ms   -> the delay is RTI's OWN receive backlog.
                                               The bus is fine; RTI is behind on draining it.
  fresh socket ALSO ~50 ms                  -> not a backlog. Either the frames genuinely
                                               arrive late, or the age maths is wrong.

It also prints the age DISTRIBUTION, which RTI's single max() figure hides: if the minimum is
small and only the maximum is large, most frames are current and just one class of frame is
old (a rare poll reply, say) — very different from every frame being uniformly stale, which is
what a real backlog looks like.

Read-only: opens its own socket, never transmits, and does not touch RTI.
"""

import argparse
import statistics
import sys
import time
from collections import defaultdict

try:
    import can
except ImportError:
    sys.exit("python-can not available — run inside the project venv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--seconds", type=float, default=5.0)
    args = ap.parse_args()

    print(f"Reading {args.channel} on an INDEPENDENT socket for {args.seconds:.0f}s "
          f"(RTI can keep running)...\n")

    try:
        bus = can.interface.Bus(channel=args.channel, bustype="socketcan")
    except Exception as e:
        sys.exit(f"could not open {args.channel}: {e}")

    ages = []
    per_node = defaultdict(list)
    t_end = time.time() + args.seconds
    try:
        while time.time() < t_end:
            msg = bus.recv(timeout=0.5)
            if msg is None:
                continue
            # Same computation RTI does: now - kernel RX timestamp.
            age = time.time() - msg.timestamp
            ages.append(age)
            node = (msg.arbitration_id >> 7) & 0x0F      # Spectral id packing
            per_node[node].append(age)
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()

    if not ages:
        sys.exit("no frames seen — is the bus up and RTI running?")

    ms = [a * 1e3 for a in ages]
    ms.sort()
    print(f"frames: {len(ms)}  ({len(ms)/args.seconds:.0f}/s)\n")
    print(f"  min    {ms[0]:8.3f} ms")
    print(f"  median {statistics.median(ms):8.3f} ms")
    print(f"  p95    {ms[int(len(ms)*0.95)]:8.3f} ms")
    print(f"  max    {ms[-1]:8.3f} ms")

    print("\n  per CAN node (median / max):")
    for node in sorted(per_node):
        v = sorted(a * 1e3 for a in per_node[node])
        print(f"    node {node:2d}: {statistics.median(v):8.3f} / {v[-1]:8.3f} ms   ({len(v)} frames)")

    med = statistics.median(ms)
    print()
    if med < 2.0:
        print("VERDICT: this socket sees the bus as CURRENT (median < 2 ms).")
        print("  If RTI reports a much larger age at the same moment, the delay is RTI's own")
        print("  receive backlog — its loop is not draining the socket fast enough. The bus,")
        print("  the wiring and the motors are not the problem.")
    else:
        print(f"VERDICT: even a fresh socket sees {med:.1f} ms median age.")
        print("  That is NOT an RTI backlog — the frames really are arriving that late, or the")
        print("  clock domains do not match. Investigate the bus/driver before touching RTI.")


if __name__ == "__main__":
    main()
