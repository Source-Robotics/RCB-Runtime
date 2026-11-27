"""python -m robotclient.recording: record episodes from one or more robots on this PC (Windows, Linux or macOS).

    python -m robotclient.recording 192.168.0.84
    python -m robotclient.recording arm1=192.168.0.84 arm2=192.168.0.85 --task "pick up the red cube"
    python -m robotclient.recording 192.168.0.84 --out D:/recordings --no-camera
    python -m robotclient.recording 192.168.0.84 --episode-s 20 --reset-s 10 --operator anna
    python -m robotclient.recording 192.168.0.84 --format mcap,lerobot
    python -m robotclient.recording export recordings/2026-11-27_13-49-02 --out datasets/freedrive

The `lerobot` format and `export` (episodes -> a LeRobot v3 dataset, robotclient/recording/export.py) need Python 3.12+
and `pip install "lerobot[dataset]" mcap`.

Each robot: `name=address`, or just the address (named arm1, arm2, ... in order). Needs `pip install mcap` for the
default format. Each session is a folder in --out (its date and time); each episode a file in it (episode_001.mcap).
While it runs, each robot sends its `record` recipe (its old recipe is put back at the end).

Keys act at once in a terminal, no Enter needed (when the input is a pipe or a script: one line per key):
    Enter   start an episode; while recording: end it (then its checks and two questions)
    d       while recording: end the episode and delete it
    g       the gripper: open <-> close, at any time (a foot pedal that types g works the same)
    r       between episodes: delete the last kept episode, to record it again (its number is reused)
    t       the task for the next episodes (type it, then Enter)
    s       how each robot's links are doing
    q       quit (an episode being recorded is kept)
Ctrl+C quits too (an episode being recorded is kept).

After each episode its checks run: the camera (gaps, lost frames, frames without their tick, clock or calibration),
the ticks (missing, slower than 8 ms), the drives' / gripper's / control board's error flags, a new error in the
robot's status, a mode change, each frame's tick among the episode's packets. If one fails it is shown: Enter deletes
the episode, k keeps it (marked in the file). Then: success? y / n / Enter = not said; then a note (Enter = none).
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

from ..exceptions import RobotConnectionError, VersionMismatch
from .export import CODECS, HEIGHT, WIDTH
from .export import export as export_episodes
from .recorder import DEFAULT_FORMAT, FORMATS, RECIPE, Recorder
from .terminal import ENTER, close_keyboard, next_key, open_keyboard, read_text

KEYS = ("keys: Enter = start / end an episode, d = end+delete, g = gripper open/close, r = delete the last episode "
        "(to record it again), t = the task, s = links, q = quit")


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
    failed = result.get("checks_failed")
    if failed is not None:
        text += f"; checks: {'all passed' if not failed else f'{len(failed)} FAILED'}"
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


def _gripper(rec: Recorder, args) -> None:
    r = rec.gripper(args.gripper, args.grip_speed, args.grip_current)
    if r["ok"]:
        print(f"gripper {r['robot']}: {'closing' if r['target'] else 'opening'}", flush=True)
    else:
        print(f"gripper {r['robot']}: REFUSED: {r['error']}", flush=True)


def _common(key: str, kb: dict, rec: Recorder, args, st: dict) -> bool:
    """The keys that work at any time: g, s, t. True when `key` was one of them."""
    if key == "g":
        _gripper(rec, args)
    elif key == "s":
        print(_links(rec), flush=True)
    elif key == "t":
        text = read_text(kb, f"task (Enter = keep {st['task']!r}): ")
        if text:
            st["task"] = text
        print(f"Task: {st['task']!r}", flush=True)
    else:
        return False
    return True


def _ask(kb: dict, rec: Recorder, args, st: dict, question: str, answers: tuple) -> str:
    """Ask until one of `answers` is pressed (g, s and t still work meanwhile)."""
    print(question, flush=True)
    while True:
        key = next_key(kb) or ""                        # no timeout: never None
        if key in answers:
            return key
        if not _common(key, kb, rec, args, st):
            print(question, flush=True)


def _after(kb: dict, rec: Recorder, args, st: dict, result: dict) -> None:
    """An ended episode: its checks shown (a failed one deleted unless k), success and a note asked, kept."""
    n = result["episode"]
    print(f"episode {n} ended: " + _describe(result), flush=True)
    if result["checks_failed"]:
        print("CHECKS FAILED:\n" + "\n".join("  " + line for line in result["checks_failed"]), flush=True)
        if _ask(kb, rec, args, st, "Enter = delete it, k = keep it (marked)", (ENTER, "k")) == ENTER:
            rec.decide(keep=False)
            print(f"episode {n} deleted", flush=True)
            return
    key = _ask(kb, rec, args, st, "success? y / n / Enter = not said", ("y", "n", ENTER))
    note = read_text(kb, "note (Enter = none): ")
    r = rec.decide(keep=True, notes={"success": {"y": True, "n": False}.get(key), "note": note})
    print(f"episode {n} kept" + (f"; ERRORS {r['errors']}" if r.get("errors") else ""), flush=True)


def _begin(rec: Recorder, args, st: dict) -> None:
    n = rec.begin(st["task"])
    limit = f"; it ends by itself after {args.episode_s:g} s" if args.episode_s else ""
    print(f"episode {n} RECORDING (Enter = end, d = end and delete, g = gripper{limit})", flush=True)


def _reset(kb: dict, rec: Recorder, args, st: dict) -> bool:
    """--reset-s: count down, then start the next episode (Enter: at once). False: q, quit."""
    print(f"reset: the next episode starts in {args.reset_s:g} s (Enter = now, q = quit)", flush=True)
    end = time.monotonic() + args.reset_s
    while True:
        key = next_key(kb, timeout=max(0.0, end - time.monotonic()))
        if key is None or key == ENTER:
            _begin(rec, args, st)
            return True
        if key == "q":
            return False
        if not _common(key, kb, rec, args, st):
            print("Enter = start now, q = quit", flush=True)


def _recording_key(key: str, kb: dict, rec: Recorder, args, st: dict) -> bool:
    """A key while an episode is recorded. False: quit."""
    if key == ENTER:
        _after(kb, rec, args, st, rec.end(keep=None))
        if args.reset_s:
            return _reset(kb, rec, args, st)
    elif key == "d":
        r = rec.end(keep=False)
        print(f"episode {r.get('episode')} deleted ({r.get('seconds', 0.0):.2f} s)", flush=True)
    elif key == "q":
        return False
    elif not _common(key, kb, rec, args, st):
        print(KEYS, flush=True)
    return True


def _idle_key(key: str, kb: dict, rec: Recorder, args, st: dict) -> bool:
    """A key between episodes. False: quit."""
    if key == ENTER:
        _begin(rec, args, st)
    elif key == "r":
        try:
            n = rec.delete_last()
            print(f"episode {n} deleted: Enter records it again", flush=True)
        except RuntimeError as e:
            print(f"r: {e}", flush=True)
    elif key == "d":
        print("no episode is being recorded", flush=True)
    elif key == "q":
        return False
    elif not _common(key, kb, rec, args, st):
        print(KEYS, flush=True)
    return True


def _run(kb: dict, rec: Recorder, args, st: dict) -> None:
    """The session's keys, until q (EOFError when the input ends)."""
    while True:
        key = next_key(kb, timeout=0.1 if rec.recording and args.episode_s else None)
        if key is None:                                 # --episode-s: is the episode over?
            if not (rec.recording and rec.stats()["seconds"] >= args.episode_s):
                continue
            print(f"episode {rec.episodes}: {args.episode_s:g} s", flush=True)
            key = ENTER
        handler = _recording_key if rec.recording else _idle_key
        if not handler(key, kb, rec, args, st):
            return


def _arguments(argv):
    ap = argparse.ArgumentParser(prog="python -m robotclient.recording",
                                 description="Record episodes (each robot's packets every tick + its camera).")
    ap.add_argument("robots", nargs="+", help="each robot: name=address, or the address (arm1, arm2, ...)")
    ap.add_argument("--format", default=DEFAULT_FORMAT,
                    help=f"the format(s), comma-separated: {', '.join(FORMATS)} (default {DEFAULT_FORMAT})")
    ap.add_argument("--out", default="recordings", help="the folder for the sessions (default ./recordings)")
    ap.add_argument("--task", default="", help="the task of the episodes (the t key changes it)")
    ap.add_argument("--operator", default="", help="who records (stored in each episode)")
    ap.add_argument("--no-camera", action="store_true", help="record without the robots' cameras")
    ap.add_argument("--recipe", default=RECIPE, help=f"the recipe each robot sends while recording (default {RECIPE})")
    ap.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for each robot's data at the start")
    ap.add_argument("--episode-s", type=float, default=0.0,
                    help="end each episode by itself after this many seconds (default 0: Enter ends it)")
    ap.add_argument("--reset-s", type=float, default=0.0,
                    help="after each episode's questions, start the next one after this many seconds "
                         "(default 0: Enter starts it)")
    ap.add_argument("--gripper", default=None, help="the robot whose gripper the g key works (default: the first)")
    ap.add_argument("--grip-speed", type=int, default=100, help="the g key's jaw speed, 0-255 (default 100)")
    ap.add_argument("--grip-current", type=int, default=200,
                    help="the g key's current limit in mA (default 200; at most the gripper XML's <Ilim>)")
    return ap.parse_args(argv)


def _export_main(argv) -> int:
    ap = argparse.ArgumentParser(prog="python -m robotclient.recording export",
                                 description="Export recorded episodes (.mcap) into a LeRobot dataset (v3).")
    ap.add_argument("inputs", nargs="+", help="episode files (.mcap) and / or session folders (their episodes)")
    ap.add_argument("--out", required=True, help="the dataset's folder: made, or appended to when it exists")
    ap.add_argument("--robot", default=None, help="whose data (default: each recording's first robot)")
    ap.add_argument("--width", type=int, default=WIDTH, help=f"the images' width (default {WIDTH})")
    ap.add_argument("--height", type=int, default=HEIGHT, help=f"the images' height (default {HEIGHT})")
    ap.add_argument("--codec", default="av1", choices=sorted(CODECS), help="the video codec (default av1)")
    ap.add_argument("--include-failed", action="store_true", help="also the episodes whose checks failed")
    ap.add_argument("--repo-id", default=None, help="the dataset's name for LeRobot (default local/<folder name>)")
    args = ap.parse_args(argv)
    try:
        r = export_episodes(args.inputs, args.out, robot=args.robot, width=args.width, height=args.height,
                            codec=args.codec, include_failed=args.include_failed, repo_id=args.repo_id,
                            progress=lambda text: print(text, flush=True))
    except (RuntimeError, ValueError) as e:
        print(f"REFUSED: {e}")
        return 2
    print(f"{r['episodes']} episode(s), {r['rows']} rows -> {r['dataset']}"
          + (f" ({'made' if r['created'] else 'appended to'})" if r["episodes"] else "")
          + (f"; {len(r['skipped'])} skipped" if r["skipped"] else ""))
    return 0


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else list(argv)
    if argv[:1] == ["export"]:
        return _export_main(argv[1:])
    args = _arguments(argv)
    try:
        rec = Recorder(parse_robots(args.robots), args.format, args.out, camera=not args.no_camera,
                       recipe=args.recipe, operator=args.operator)
        if args.gripper is not None and args.gripper not in [s.name for s in rec.sources]:
            raise ValueError(f"--gripper {args.gripper}: no robot of that name")
    except (ValueError, RuntimeError) as e:
        print(f"REFUSED: {e}")
        return 2
    st = {"task": args.task}
    code = 0
    stop = threading.Event()
    kb = None
    try:
        print("Connecting to " + ", ".join(f"{s.name} ({s.host})" for s in rec.sources) + " ...", flush=True)
        rec.start(timeout=args.timeout)
        print(f"Recording into {rec.session_dir} as {', '.join(m.NAME for m in rec.formats)}.")
        print(_links(rec))
        print(f"Task: {st['task']!r}" if st["task"] else "Task: (none; the t key sets it)")
        print(KEYS, flush=True)
        threading.Thread(target=_status_printer, args=(rec, stop), daemon=True).start()
        kb = open_keyboard()
        try:
            _run(kb, rec, args, st)
        except EOFError:
            pass
    except KeyboardInterrupt:
        print()
    except (RuntimeError, RobotConnectionError, VersionMismatch, OSError) as e:
        print(f"STOPPED: {e}")
        code = 1
    finally:
        stop.set()
        if kb is not None:
            close_keyboard(kb)
        try:
            if rec.recording:
                print(f"episode {rec.episodes} kept: " + _describe(rec.end(keep=True)))
            if rec.held:
                r = rec.decide(keep=True)
                print(f"episode {r.get('episode')} kept (its questions not answered)")
        finally:
            rec.close()
    print(f"{rec.kept} episode(s) kept" + (f" in {rec.session_dir}" if rec.session_dir else "") + ".")
    return code


if __name__ == "__main__":
    sys.exit(main())
