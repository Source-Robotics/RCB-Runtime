#!/usr/bin/env python3
"""Decode a candump capture through the Spectral CAN id packing, per node.

Companion to tools/probe_gripper_watchdog.py: when every isolated replay arms
a working watchdog but the real stack still leaves the gripper unlatched, the
remaining move is to capture the real thing and read what ACTUALLY went to the
node. This tool turns a raw `candump -ta can0` log into:

  * a per-command histogram for the chosen node (sent TO it and replies FROM
    it share ids; direction is inferred from frame kind: remote/config writes
    are host->node, data-frame replies to respond-commands are node->host),
  * a timeline of every CONFIG-CLASS frame (watchdog, limits, gains, estop,
    clear-error, idle, reset, save, set-can-id, heartbeat, cyclic, vlim) with
    raw data bytes — cmd 15 lines show the decoded watchdog rate/action,
  * bus error frames, flagged.

Capture procedure (the boot+kill scenario):
    terminal 1:  candump -ta can0 > /tmp/gripper_boot_kill.log
    terminal 2:  start the stack, wait ~20 s (no homing, no GUI clicks),
                 pkill -f RTI.py, wait 15 s
    terminal 1:  Ctrl+C
    then:        python3 tools/probe_gripper_watchdog.py --read-only
                 python3 tools/candump_node_analyze.py /tmp/gripper_boot_kill.log --node 6

Examples:
    python3 tools/candump_node_analyze.py capture.log --node 6
    python3 tools/candump_node_analyze.py capture.log --node 6 --all-frames
"""

import argparse
import re
import sys
from collections import Counter

COMMAND_NAMES = {
    0: "ESTOP", 1: "CLEAR_ERROR", 2: "DATA_PACK_1", 3: "RESPOND_DATA_PACK_1",
    4: "DATA_PACK_PD", 6: "DATA_PACK_2", 8: "DATA_PACK_3", 10: "PING",
    11: "SET_CAN_ID", 12: "IDLE", 13: "SAVE_CONFIG", 14: "RESET",
    15: "WATCHDOG", 16: "PD_GAINS", 17: "CURRENT_GAINS", 18: "VELOCITY_GAINS",
    19: "POSITION_GAINS", 20: "LIMITS", 22: "KT_WRITE", 23: "TEMPERATURE",
    24: "VOLTAGE", 25: "DEVICE_INFO", 26: "STATE_OF_ERRORS", 27: "IQ_DATA",
    28: "ENCODER_DATA", 29: "CYCLIC", 30: "HEARTBEAT_SETUP", 31: "HALL_INDEX",
    33: "KT_READ", 34: "VOLTAGE_LIMIT",
    59: "GRIPPER_CALIBRATE", 60: "GRIPPER_DATA_PACK",
}
# Frames that (re)configure or command the node — the ones that could plausibly
# touch watchdog state. Every one of these deserves a timeline row.
CONFIG_CMDS = {0, 1, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 29, 30, 34, 59}

# candump -ta:  (1752671234.567890)  can0  123   [2]  01 04
#               (1752671234.567890)  can0  123   [0]  remote request
LINE_RE = re.compile(
    r"\(\s*(?P<ts>\d+\.\d+)\)\s+\S+\s+(?P<id>[0-9A-Fa-f]+)\s+"
    r"\[(?P<dlc>\d+)\]\s*(?P<rest>.*)")


def decode_id(can_id):
    node = (can_id >> 7) & 0xF
    cmd = (can_id >> 1) & 0x3F
    err = can_id & 1
    return node, cmd, err


def parse_line(line):
    m = LINE_RE.search(line)
    if not m:
        return None
    can_id = int(m.group("id"), 16)
    rest = m.group("rest").strip()
    remote = "remote" in rest.lower()
    data = bytes.fromhex(rest.replace(" ", "")) if (rest and not remote) else b""
    return {
        "ts": float(m.group("ts")),
        "id": can_id,
        "dlc": int(m.group("dlc")),
        "remote": remote,
        "data": data,
        # candump renders error frames with 8-plus-hex-digit ids (CAN_ERR_FLAG set)
        "error_frame": len(m.group("id")) > 3,
    }


def watchdog_detail(data):
    if len(data) >= 5:
        rate = int.from_bytes(data[0:4], "big")
        action = {0: "Idle", 1: "Hold", 2: "Brake"}.get(data[4], f"?{data[4]}")
        return f"rate={rate} ms action={action}"
    return f"short payload ({len(data)} bytes!)"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logfile", help="output of: candump -ta can0")
    parser.add_argument("--node", type=int, default=6, help="node id to analyze (default 6)")
    parser.add_argument("--all-frames", action="store_true",
                        help="print every frame for the node, not just config-class")
    args = parser.parse_args()

    frames = []
    error_frames = 0
    with open(args.logfile) as fh:
        for line in fh:
            parsed = parse_line(line)
            if parsed is None:
                continue
            if parsed["error_frame"]:
                error_frames += 1
                continue
            frames.append(parsed)

    if not frames:
        sys.exit(f"no CAN frames parsed from {args.logfile} — is it candump -ta output?")

    t0 = frames[0]["ts"]
    node_frames = [f for f in frames if decode_id(f["id"])[0] == args.node]
    print(f"{len(frames)} frames total, {len(node_frames)} for node {args.node}, "
          f"{error_frames} bus error frames, span "
          f"{frames[-1]['ts'] - t0:.1f} s")

    hist = Counter()
    for f in node_frames:
        _, cmd, _ = decode_id(f["id"])
        kind = "req" if f["remote"] else "data"
        hist[(cmd, kind)] += 1
    print(f"\n--- node {args.node}: per-command histogram ---")
    for (cmd, kind), count in sorted(hist.items()):
        name = COMMAND_NAMES.get(cmd, f"cmd{cmd}")
        print(f"  cmd {cmd:>2} {name:<18} {kind:<4} x{count}")

    print(f"\n--- node {args.node}: config-class timeline ---")
    shown = 0
    for f in node_frames:
        _, cmd, err = decode_id(f["id"])
        if not args.all_frames and cmd not in CONFIG_CMDS:
            continue
        name = COMMAND_NAMES.get(cmd, f"cmd{cmd}")
        detail = watchdog_detail(f["data"]) if cmd == 15 else f["data"].hex(" ")
        kind = "REQ " if f["remote"] else "data"
        flag = " ERRBIT" if err else ""
        print(f"  +{f['ts'] - t0:9.3f}s  cmd {cmd:>2} {name:<18} {kind} "
              f"dlc={f['dlc']} [{detail}]{flag}")
        shown += 1
        if shown > 400:
            print("  ... (truncated at 400 rows — rerun with a shorter capture)")
            break
    if shown == 0:
        print("  (none)")

    last_data = [f for f in node_frames if not f["remote"]]
    if last_data:
        print(f"\nlast frame involving node {args.node}: "
              f"+{node_frames[-1]['ts'] - t0:.3f}s — everything after that is the "
              f"unfed window; the watchdog should trip <rate> ms later.")


if __name__ == "__main__":
    main()
