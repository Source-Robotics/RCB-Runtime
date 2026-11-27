"""python -m robotclient.recording: record episodes from one or more robots on this PC (Windows, Linux or macOS).

    python -m robotclient.recording 192.168.0.84
    python -m robotclient.recording arm1=192.168.0.84 arm2=192.168.0.85 --task "pick up the red cube"
    python -m robotclient.recording 192.168.0.84 --out D:/recordings --no-camera

Each robot: `name=address`, or just the address (named arm1, arm2, ... in order). Needs `pip install mcap` for the
default format. Each session is a folder in --out (its date and time); each episode a file in it (episode_001.mcap).
While it runs, each robot sends its `record` recipe (its old recipe is put back at the end).

Type a key, then Enter:
    (Enter)    start an episode; while recording: end it and keep it
    d          while recording: end the episode and delete it
    t TEXT     the task for the next episodes (stored in each episode)
    s          how each robot's links are doing
    q          quit (an episode being recorded is kept)
Ctrl+C quits too (an episode being recorded is kept).
"""
from __future__ import annotations

import argparse
import sys
import threading

from ..exceptions import RobotConnectionError, VersionMismatch
from .recorder import DEFAULT_FORMAT, FORMATS, RECIPE, Recorder

KEYS = ("keys: Enter = start / end+keep an episode, d = end+delete, t TEXT = the task, s = links, q = quit "
        "(each followed by Enter)")


def parse_robots(items) -> list:
    """["arm1=192.168.0.84", "192.168.0.85"] -> [("arm1", "192.168.0.84"), ("arm2", "192.168.0.85")]."""
    out = []
    for i, item in enumerate(items, start=1):
        name, sep, host = item.partition("=")
        out.append((name.strip(), host.strip()) if sep else (f"arm{i}", item.strip()))
    return out


def _describe(result: dict) -> str:
    parts = []
    for name, c in result.get("counts", {}).items():
        parts.append(f"{name}: {c['packets']} packets, {c['frames']} frames, {c['tick_gaps']} ticks missing, "
                     f"{c['frames_lost'] + c['frames_skipped']} frames missing")
    text = f"{result.get('seconds', 0.0):.2f} s; " + "; ".join(parts)
    if result.get("errors"):
        text += f"; ERRORS {result['errors']}"
    return text


def _links(rec: Recorder) -> str:
    lines = []
    for name, s in rec.stats()["robots"].items():
        off = s["clock_offset_ns"]
        lines.append(f"  {name}: packets {s['link']} ({s['packets']}); camera {s['camera']} ({s['frames']}); "
                     f"clock spread {s['clock_spread_ns'] / 1e6:.3f} ms" + ("" if off is not None else " (no clock)")
                     + (f"; last error: {s['last_error']}" if s["last_error"] else ""))
    return "\n".join(lines)


def _status_printer(rec: Recorder, stop: threading.Event, every_s: float = 2.0) -> None:
    last = {n: (s["packets"], s["frames"]) for n, s in rec.stats()["robots"].items()}
    while not stop.wait(every_s):
        st = rec.stats()
        now = {n: (s["packets"], s["frames"]) for n, s in st["robots"].items()}
        if st["recording"]:
            rates = ", ".join(f"{n}: {(now[n][0] - last[n][0]) / every_s:.0f} packets/s "
                              f"{(now[n][1] - last[n][1]) / every_s:.0f} frames/s" for n in now)
            print(f"REC episode {st['episode']}  {st['seconds']:.1f} s | {rates} | queued {st['queued']}", flush=True)
        last = now


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m robotclient.recording",
                                 description="Record episodes (each robot's packets every tick + its camera).")
    ap.add_argument("robots", nargs="+", help="each robot: name=address, or the address (arm1, arm2, ...)")
    ap.add_argument("--format", default=DEFAULT_FORMAT,
                    help=f"the format(s), comma-separated: {', '.join(FORMATS)} (default {DEFAULT_FORMAT})")
    ap.add_argument("--out", default="recordings", help="the folder for the sessions (default ./recordings)")
    ap.add_argument("--task", default="", help="the task of the episodes (change it with t TEXT)")
    ap.add_argument("--no-camera", action="store_true", help="record without the robots' cameras")
    ap.add_argument("--recipe", default=RECIPE, help=f"the recipe each robot sends while recording (default {RECIPE})")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for each robot's data at the start")
    args = ap.parse_args(argv)
    try:
        rec = Recorder(parse_robots(args.robots), args.format, args.out, camera=not args.no_camera,
                       recipe=args.recipe)
    except (ValueError, RuntimeError) as e:
        print(f"REFUSED: {e}")
        return 2
    task = args.task
    code = 0
    stop = threading.Event()
    try:
        print("Connecting to " + ", ".join(f"{s.name} ({s.host})" for s in rec.sources) + " ...", flush=True)
        rec.start(timeout=args.timeout)
        print(f"Recording into {rec.session_dir} as {', '.join(m.NAME for m in rec.formats)}.")
        print(_links(rec))
        print(f"Task: {task!r}" if task else "Task: (none; t TEXT sets it)")
        print(KEYS, flush=True)
        threading.Thread(target=_status_printer, args=(rec, stop), daemon=True).start()
        while True:
            try:
                line = input()
            except EOFError:
                break
            cmd = line.strip()
            if cmd == "":
                if rec.recording:
                    print(f"episode {rec.episodes} kept: " + _describe(rec.end(keep=True)), flush=True)
                else:
                    n = rec.begin(task)
                    print(f"episode {n} RECORDING (Enter = end and keep, d = end and delete)", flush=True)
            elif cmd == "d":
                if rec.recording:
                    r = rec.end(keep=False)
                    print(f"episode {r.get('episode')} deleted ({r.get('seconds', 0.0):.2f} s)", flush=True)
                else:
                    print("no episode is being recorded")
            elif cmd == "t" or cmd.startswith("t "):
                if cmd != "t":
                    task = cmd[2:].strip()
                print(f"Task: {task!r}", flush=True)
            elif cmd == "s":
                print(_links(rec), flush=True)
            elif cmd == "q":
                break
            else:
                print(KEYS, flush=True)
    except KeyboardInterrupt:
        print()
    except (RuntimeError, RobotConnectionError, VersionMismatch, OSError) as e:
        print(f"STOPPED: {e}")
        code = 1
    finally:
        stop.set()
        try:
            if rec.recording:
                print(f"episode {rec.episodes} kept: " + _describe(rec.end(keep=True)))
        finally:
            rec.close()
    print(f"{rec.kept} episode(s) kept" + (f" in {rec.session_dir}" if rec.session_dir else "") + ".")
    return code


if __name__ == "__main__":
    sys.exit(main())
