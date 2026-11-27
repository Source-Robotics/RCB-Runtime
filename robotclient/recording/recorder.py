"""The recorder's core: each robot's packets and camera, the episodes, the formats (TODO/p4_recording_plan.md 3.1).

`Recorder(robots, formats)`: `start()` connects to every robot (its commands, its recorder link and its camera) and
keeps them open; `begin(task)` starts an episode, `end(keep)` ends it (kept: its files written; not kept: deleted);
`close()` ends an open episode, gives each robot its old recipe back and disconnects. `stats()` says how each link
is doing. It prints nothing: `python -m robotclient.recording` is the program around it.

Threads: per robot one for its packets (TCP 30004: every tick) and one for its camera, and one writer. The readers
stamp each item on arrival, put the robot's own time on this PC's clock (recording.clock) and queue it; the writer
alone touches the files. An episode holds what HAPPENED between `begin` and `end` (by each item's own moment, not its
arrival): after `end` the writer waits up to CLOSE_GRACE_S for the items still on their way (a camera frame arrives
~35-60 ms after its image moment), then closes the files. At the other end, the items that arrive while no episode is
open are kept for PREROLL_S: an item can reach the writer before `begin` does (on Windows the first platform.platform()
in `begin` runs a command, tens of ms), and the new episode takes every kept item that happened at or after its start.

A format is a module (FORMATS) with: NAME, EXTENSION, missing(), open_episode(path, info), add_packet(...),
add_frame(...), add_clock(...), close_episode(handle, meta) -> path, discard_episode(handle). A new format = a new
module + one line in FORMATS.
"""
from __future__ import annotations

import importlib
import os
import platform
import queue
import shutil
import socket
import threading
import time
from collections import deque
from datetime import datetime, timezone

from ..camera_client import CameraClient
from ..client import RobotClient
from ..constants import DEFAULT_PORT
from ..exceptions import RobotConnectionError, RobotTimeout, VersionMismatch
from ..telemetry import TelemetryClient, decode_packet
from ..version import __version__
from .clock import RobotClock, unix_minus_perf_ns

FORMATS = {"mcap": "robotclient.recording.mcap_format"}
DEFAULT_FORMAT = "mcap"
RECIPE = "record"                 # every tag but the gains (data_recipe/record.xml)
CAMERA = "wrist"                  # the name of each robot's own camera in a recording
MIN_FREE_BYTES = 1 << 30          # an episode is not started with less free disk space than this
CLOSE_GRACE_S = 1.0               # after end(): the longest wait for items still on their way
PREROLL_S = 1.0                   # while no episode is open: how long the newest items are kept, for one that begins
RECONNECT_S = 1.0                 # a lost link is tried again after this long


def load_formats(names) -> list:
    """The format modules for `names` (a list, or text such as ``"mcap,lerobot"``).

    Raises:
        ValueError: no name, or a name that is no format (FORMATS).
        RuntimeError: a format whose package is not installed here (its `missing()` says what to install).
    """
    if isinstance(names, str):
        names = [n.strip() for n in names.split(",") if n.strip()]
    if not names:
        raise ValueError("no recording format named")
    mods: list = []
    for name in names:
        if name not in FORMATS:
            raise ValueError(f"no recording format {name!r}: the formats are {', '.join(FORMATS)}")
        mod = importlib.import_module(FORMATS[name])
        why = mod.missing()
        if why:
            raise RuntimeError(why)
        if mod not in mods:
            mods.append(mod)
    return mods


def _is_number(v) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v == v


def _iso(unix_ns: int) -> str:
    return datetime.fromtimestamp(unix_ns / 1e9, timezone.utc).isoformat(timespec="microseconds")


class RobotSource:
    """One robot of a recording: its name (``arm1``) and address, its clock mapping, how its links are doing."""

    def __init__(self, name: str, host: str, camera: bool = True) -> None:
        self.name = name
        self.host = host
        self.camera = camera
        self.clock = RobotClock()
        self.client: RobotClient | None = None
        self.old_recipe: str | None = None
        self.versions: dict = {}
        self.status: dict = {}
        self.link = "not started"
        self.camera_link = "not started" if camera else "off"
        self.packets = 0              # received since start()
        self.frames = 0
        self.bad_packets = 0
        self.reconnects = 0
        self.camera_reconnects = 0
        self.last_error = ""


class Recorder:
    """Records episodes from one or more robots (each: its packets every tick and its camera) into the formats named.

    Args:
        robots: ``[(name, address), ...]``; names become the recording's topics (``/arm1/...``).
        formats: Format names (``"mcap"``) or text ``"mcap,lerobot"``.
        out_dir: Where each session's folder is made.
        camera: Whether each robot's camera is recorded.
        recipe: The recipe set on each robot while recording (its old one is put back by `close()`).
        port: The robots' command port.

    Raises:
        ValueError / RuntimeError: as `load_formats`; ValueError for no robot or two with one name.
    """

    def __init__(self, robots, formats=DEFAULT_FORMAT, out_dir: str = "recordings", camera: bool = True,
                 recipe: str = RECIPE, port: int = DEFAULT_PORT) -> None:
        robots = list(robots)
        if not robots:
            raise ValueError("no robot to record")
        names = [n for n, _ in robots]
        if len(set(names)) != len(names):
            raise ValueError(f"two robots with one name: {names}")
        self.formats = load_formats(formats)
        self.sources = [RobotSource(n, h, camera) for n, h in robots]
        self.out_dir = out_dir
        self.recipe = recipe
        self.port = port
        self.session_dir = ""
        self.episodes = 0                    # episodes begun
        self.kept = 0
        self.recording = False
        self.episode_started = 0.0           # time.monotonic() of the open episode's begin
        self._unix = unix_minus_perf_ns()    # this PC's perf_counter_ns -> ns since 1970, fixed for the session
        self._q: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._threads: list = []
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ the session
    def start(self, timeout: float = 10.0) -> None:
        """Connect to every robot, set the recording recipe, start the readers and the writer, and wait until every
        robot's packets (and frames, with `camera`) arrive.

        Raises:
            RuntimeError: a robot's recorder link is taken by another program, or no packet / frame within `timeout`
                (the message names the robot and what its link says).
            robotclient errors from connecting (RobotConnectionError, VersionMismatch).
        """
        for src in self.sources:
            rc = RobotClient(name="recorder")
            rc.connect(src.host, self.port)
            src.client = rc
            end = time.monotonic() + 3.0
            st: dict = {}
            while time.monotonic() < end and "telemetry" not in st:
                st = rc.latest_status() or {}
                time.sleep(0.05)
            src.status = {k: st.get(k) for k in ("mode", "state", "homed", "recipe", "telemetry", "protocols",
                                                 "runtime_version")}
            peer = (st.get("telemetry") or {}).get("tcp_peer")
            if peer:
                raise RuntimeError(f"{src.name} ({src.host}): its recorder link is in use by {peer}; close that "
                                   "program first (the robot serves one recorder at a time)")
            try:
                src.versions = rc.get_versions()
            except Exception as e:  # noqa: BLE001  versions are information, not a reason to stop
                src.versions = {"error": repr(e)}
            old = st.get("recipe")
            if self.recipe and old != self.recipe:
                rc.set_recipe(self.recipe)
                src.old_recipe = old
        self.session_dir = os.path.join(self.out_dir, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
        os.makedirs(self.session_dir, exist_ok=True)
        for src in self.sources:
            self._spawn(self._telemetry_loop, src, f"packets {src.name}")
            if src.camera:
                self._spawn(self._camera_loop, src, f"camera {src.name}")
        self._spawn(self._writer_loop, None, "writer")
        end = time.monotonic() + timeout
        for src in self.sources:
            while src.clock.offset_ns is None or (src.camera and src.frames == 0):
                if time.monotonic() > end:
                    what = (f"no packets (link: {src.link})" if src.clock.offset_ns is None
                            else f"no camera frames (camera: {src.camera_link}; --no-camera records without it)")
                    raise RuntimeError(f"{src.name} ({src.host}): {what}")
                time.sleep(0.05)

    def close(self) -> None:
        """End an open episode (kept), stop the threads, give each robot its old recipe back, disconnect. Safe to call
        twice and after a failed `start()`."""
        if self.recording:
            self.end(keep=True)
        self._stop.set()
        self._q.put(("quit",))
        for th in self._threads:
            th.join(timeout=5.0)
        self._threads = []
        for src in self.sources:
            rc = src.client
            if rc is None:
                continue
            try:
                if src.old_recipe:
                    rc.set_recipe(src.old_recipe)
                    src.old_recipe = None
            finally:
                rc.disconnect()
                src.client = None

    # ------------------------------------------------------------------ episodes
    def begin(self, task: str = "", notes: dict | None = None) -> int:
        """Start episode n+1 now; returns its number. Its files are open when this returns.

        Raises:
            RuntimeError: an episode is open already, less than MIN_FREE_BYTES free on the disk, or a format could not
                open its file (the message says why).
        """
        if self.recording:
            raise RuntimeError("an episode is being recorded already")
        free = shutil.disk_usage(self.session_dir or self.out_dir).free
        if free < MIN_FREE_BYTES:
            raise RuntimeError(f"only {free / 1e9:.2f} GB free on the disk: no episode started")
        self.episodes += 1
        start_ns = time.perf_counter_ns() + self._unix
        info = {
            "recorder_version": __version__,
            "episode": self.episodes,
            "session": os.path.basename(self.session_dir),
            "task": task,
            "start_utc": _iso(start_ns),
            "sources": [{"name": s.name, "host": s.host, "camera": CAMERA if s.camera else None}
                        for s in self.sources],
            "robots": {s.name: {"versions": s.versions, "status": s.status} for s in self.sources},
            "pc": {"platform": platform.platform(), "python": platform.python_version(), "node": platform.node()},
            "time_base": "log time = when it happened, on the recording PC's clock (ns since 1970) through each "
                         "robot's clock mapping (/<robot>/clock); publish time = when it arrived at the PC",
            **(notes or {}),
        }
        done = threading.Event()
        result: dict = {}
        self._q.put(("begin", start_ns, task, info, done, result))
        done.wait()
        if result.get("error"):
            raise RuntimeError(result["error"])
        self.recording = True
        self.episode_started = time.monotonic()
        return self.episodes

    def end(self, keep: bool = True, notes: dict | None = None) -> dict:
        """End the open episode now: kept (its files finished and named) or not (deleted). Waits for the items still
        on their way (at most CLOSE_GRACE_S). Returns {"episode", "kept", "paths", "seconds", "counts", "errors"}.

        Raises:
            RuntimeError: no episode is open.
        """
        if not self.recording:
            raise RuntimeError("no episode is being recorded")
        stop_ns = time.perf_counter_ns() + self._unix
        done = threading.Event()
        result: dict = {}
        self._q.put(("end", stop_ns, keep, notes or {}, done, result))
        done.wait()
        self.recording = False
        if result.get("kept"):
            self.kept += 1
        return result

    def stats(self) -> dict:
        """How each robot's links are doing, and the episode: {"robots": {name: {...}}, "recording", "episode",
        "seconds", "queued"}."""
        return {
            "robots": {s.name: {"link": s.link, "camera": s.camera_link, "packets": s.packets, "frames": s.frames,
                                "bad_packets": s.bad_packets, "reconnects": s.reconnects,
                                "camera_reconnects": s.camera_reconnects, "clock_offset_ns": s.clock.offset_ns,
                                "clock_spread_ns": s.clock.spread_ns, "last_error": s.last_error}
                       for s in self.sources},
            "recording": self.recording, "episode": self.episodes,
            "seconds": (time.monotonic() - self.episode_started) if self.recording else 0.0,
            "queued": self._q.qsize(),
        }

    # ------------------------------------------------------------------ the readers
    def _spawn(self, fn, src, name: str) -> None:
        th = threading.Thread(target=fn, args=(src,) if src is not None else (), name=name, daemon=True)
        th.start()
        self._threads.append(th)

    def _telemetry_loop(self, src: RobotSource) -> None:
        unix = self._unix
        while not self._stop.is_set():
            tel = TelemetryClient(src.host, timeout=3.0)
            got = 0
            try:
                tel.connect()
                src.link = "connected, waiting for packets"
                while not self._stop.is_set():
                    try:
                        raw = tel.read_raw(timeout=1.0)
                    except socket.timeout:
                        src.link = "connected, no packets (is the robot's control loop running?)"
                        continue
                    arrival = time.perf_counter_ns()
                    try:
                        d = decode_packet(raw)
                    except ValueError as e:
                        src.bad_packets += 1
                        src.last_error = f"packet: {e}"
                        continue
                    got += 1
                    src.packets += 1
                    src.link = "receiving"
                    sent = d.get("timestamp")
                    moment = None
                    if isinstance(sent, (int, float)) and sent == sent:
                        closed = src.clock.add(sent, arrival)
                        moment = src.clock.to_pc_ns(sent)
                        if closed and moment is not None:
                            self._q.put(("clock", src, arrival + unix,
                                         {"robot_s": sent, "pc_ns": arrival + unix,
                                          "robot_to_pc_ns": moment - round(sent * 1e9) + unix,
                                          "spread_ns": src.clock.spread_ns, "samples": src.clock.samples}))
                    # a recipe without `timestamp`: its arrival
                    log = (moment if moment is not None else arrival) + unix
                    self._q.put(("packet", src, log, arrival + unix, raw, d))
            except (ConnectionError, OSError) as e:
                src.last_error = f"packets: {e}"
                src.link = ("closed by the robot before any packet (another program holds its recorder link?)"
                            if got == 0 else f"lost ({e}); reconnecting")
                src.reconnects += 1
            finally:
                tel.close()
            self._stop.wait(RECONNECT_S)

    def _camera_loop(self, src: RobotSource) -> None:
        unix = self._unix
        while not self._stop.is_set():
            cam = CameraClient()
            try:
                cam.connect(src.host, timeout=3.0)
                src.camera_link = "connected, waiting for frames"
                while not self._stop.is_set():
                    try:
                        frame = cam.read(timeout=1.0)
                    except RobotTimeout:
                        src.camera_link = "connected, no frames (camera unplugged or switched off?)"
                        continue
                    arrival = time.perf_counter_ns()
                    src.frames += 1
                    src.camera_link = "receiving"
                    moment = src.clock.to_pc_ns(frame.exposure_time)
                    if moment is None:
                        continue                                # no packet yet: the robot's clock is not known
                    self._q.put(("frame", src, moment + unix, arrival + unix, frame))
            except VersionMismatch as e:
                src.camera_link = f"refused: {e}"
                src.last_error = f"camera: {e}"
                return
            except (RobotConnectionError, OSError) as e:
                src.camera_link = f"lost ({e}); reconnecting"
                src.last_error = f"camera: {e}"
                src.camera_reconnects += 1
            finally:
                cam.close()
            self._stop.wait(RECONNECT_S)

    # ------------------------------------------------------------------ the writer
    def _writer_loop(self) -> None:
        ep: dict | None = None
        recent: deque = deque()                     # while no episode is open: the items of the last PREROLL_S
        preroll_ns = int(PREROLL_S * 1e9)
        while True:
            try:
                item = self._q.get(timeout=0.05 if ep is not None and ep["stop_ns"] is not None else 0.5)
            except queue.Empty:
                item = None
            if item is not None:
                kind = item[0]
                if kind == "quit":
                    if ep is not None:
                        self._finish(ep)
                    return
                if kind == "begin":
                    ep = self._open(*item[1:])
                    if ep is not None:                  # what happened after the start but reached the queue first
                        for old in recent:
                            if old[2] >= ep["start_ns"]:
                                self._write(ep, old)
                    recent.clear()
                elif kind == "end":
                    if ep is not None:
                        _, ep["stop_ns"], ep["keep"], ep["notes"], ep["done"], ep["result"] = item
                        ep["deadline"] = time.monotonic() + CLOSE_GRACE_S
                elif ep is not None:
                    self._write(ep, item)
                else:
                    recent.append(item)
                    while item[2] - recent[0][2] > preroll_ns:      # by log time (item[2] in every kind)
                        recent.popleft()
            if ep is not None and ep["stop_ns"] is not None and (not ep["pending"]
                                                                  or time.monotonic() >= ep["deadline"]):
                self._finish(ep)
                ep = None

    def _open(self, start_ns, task, info, done, result) -> dict | None:
        base = os.path.join(self.session_dir, f"episode_{info['episode']:03d}")
        handles = []
        try:
            for mod in self.formats:
                handles.append((mod, mod.open_episode(base + mod.EXTENSION, info)))
        except Exception as e:  # noqa: BLE001  told to begin()'s caller
            for mod, h in handles:
                mod.discard_episode(h)
            result["error"] = f"the {mod.NAME} file could not be opened: {e}"
            done.set()
            return None
        streams = {(s.name, "packet") for s in self.sources} | {(s.name, "frame") for s in self.sources if s.camera}
        counts = {s.name: {"packets": 0, "frames": 0, "frames_lost": 0, "frames_skipped": 0, "tick_gaps": 0,
                           "tick_restarts": 0, "first_tick": None, "last_tick": None,
                           "clock_offset_first_ns": None, "clock_offset_last_ns": None} for s in self.sources}
        done.set()
        return {"index": info["episode"], "task": task, "start_ns": start_ns, "stop_ns": None, "handles": handles,
                "failed": {}, "counts": counts, "pending": set(streams), "before": 0, "after": 0}

    def _write(self, ep: dict, item) -> None:
        kind, src, log_ns = item[0], item[1], item[2]
        if log_ns < ep["start_ns"]:
            ep["before"] += 1
            return
        if ep["stop_ns"] is not None and log_ns >= ep["stop_ns"]:
            ep["pending"].discard((src.name, kind))
            ep["after"] += 1
            return
        c = ep["counts"][src.name]
        if kind == "packet":
            tick = item[5].get("tick")
            if _is_number(tick):
                tick = int(tick)
                last = c["last_tick"]
                if last is not None:
                    if tick > last + 1:
                        c["tick_gaps"] += tick - last - 1
                    elif tick <= last:
                        c["tick_restarts"] += 1                 # the control loop restarted: its ticks from 0
                if c["first_tick"] is None:
                    c["first_tick"] = tick
                c["last_tick"] = tick
            c["packets"] += 1
            c["clock_offset_last_ns"] = src.clock.offset_ns           # this PC's perf_counter_ns - the robot's
            if c["clock_offset_first_ns"] is None:
                c["clock_offset_first_ns"] = src.clock.offset_ns
        elif kind == "frame":
            frame = item[4]
            c["frames"] += 1
            c["frames_lost"] += frame.lost
            c["frames_skipped"] += frame.skipped
        for mod, h in ep["handles"]:
            if mod.NAME in ep["failed"]:
                continue
            try:
                if kind == "packet":
                    mod.add_packet(h, src.name, log_ns, item[3], item[4], item[5])
                elif kind == "frame":
                    mod.add_frame(h, src.name, CAMERA, log_ns, item[3], item[4])
                elif kind == "clock":
                    mod.add_clock(h, src.name, log_ns, item[3])
            except Exception as e:  # noqa: BLE001  e.g. the disk full: this format stops, the others go on
                ep["failed"][mod.NAME] = repr(e)

    def _finish(self, ep: dict) -> None:
        result = ep.get("result")
        if result is None:                   # quit with no end(): kept
            result = {}
            ep.update(keep=True, notes={}, stop_ns=time.perf_counter_ns() + self._unix, done=None)
        seconds = (ep["stop_ns"] - ep["start_ns"]) / 1e9
        meta = {"episode": ep["index"], "task": ep["task"], "start_utc": _iso(ep["start_ns"]),
                "end_utc": _iso(ep["stop_ns"]), "start_ns": ep["start_ns"], "end_ns": ep["stop_ns"],
                "seconds": round(seconds, 6), "counts": ep["counts"], "items_before_start": ep["before"],
                "items_after_end": ep["after"], "unfinished_streams": sorted(f"{a}/{b}" for a, b in ep["pending"]),
                **ep["notes"]}
        paths, errors = [], dict(ep["failed"])
        for mod, h in ep["handles"]:
            try:
                if ep["keep"] and mod.NAME not in ep["failed"]:
                    if isinstance(h, dict) and h.get("state_errors"):
                        meta["viewer_errors"] = {"count": h["state_errors"], "last": h.get("last_state_error")}
                    paths.append(mod.close_episode(h, meta))
                else:
                    mod.discard_episode(h)
            except Exception as e:  # noqa: BLE001
                errors[mod.NAME] = repr(e)
        result.update(episode=ep["index"], kept=bool(ep["keep"]) and bool(paths), paths=paths,
                      seconds=seconds, counts=ep["counts"], errors=errors)
        if ep.get("done") is not None:
            ep["done"].set()
