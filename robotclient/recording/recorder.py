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

Each robot's status (the robot pushes it whenever it changes) and the recorder's own events (the gripper key) go into
the episode too. At its end the writer CHECKS the episode, per robot (`_checks`, 2026-11-27): the camera (gaps, lost
or skipped frames, frames without their tick, the camera's clock or the calibrated flag), the packets (missing ticks,
a tick slower than SLOW_TICK_MS, a drive / gripper / control-board error flag, an error key in the robot's status
that was not there at the start, a mode change) and the pairing (each frame's tick among the episode's packets). The
result is in `end()`'s answer and in the episode's metadata. `end(keep=None)` leaves the files open for the
operator's answers (keep or delete, success, a note); `decide()` then closes them. `gripper()` opens / closes a
robot's gripper (`set_gripper`: at once, in any mode); `delete_last()` deletes the last kept episode to record it
again.

A format is a module (FORMATS) with: NAME, EXTENSION, missing(), open_episode(path, info), add_packet(...),
add_frame(...), add_clock(...), add_status(...), add_event(...), close_episode(handle, meta) -> path,
discard_episode(handle). A new format = a new module + one line in FORMATS.
"""
from __future__ import annotations

import functools
import importlib
import math
import os
import platform
import queue
import shutil
import socket
import statistics
import threading
import time
from collections import deque
from datetime import datetime, timezone

from ..camera_client import CameraClient
from ..client import RobotClient
from ..constants import DEFAULT_PORT
from ..exceptions import RobotConnectionError, RobotError, RobotTimeout, VersionMismatch
from ..telemetry import TelemetryClient, decode_packet
from ..version import __version__
from .clock import RobotClock, unix_minus_perf_ns

FORMATS = {"mcap": "robotclient.recording.mcap_format", "lerobot": "robotclient.recording.lerobot_format"}
DEFAULT_FORMAT = "mcap"
RECIPE = "record"                 # every tag but the gains (data_recipe/record.xml)
CAMERA = "wrist"                  # the name of each robot's own camera in a recording
MIN_FREE_BYTES = 1 << 30          # an episode is not started with less free disk space than this
CLOSE_GRACE_S = 1.0               # after end(): the longest wait for items still on their way
PREROLL_S = 1.0                   # while no episode is open: how long the newest items are kept, for one that begins
SLOW_TICK_MS = 8.0                # a tick longer than this fails the episode's check (two ticks of 4 ms)
CAMERA_GAP_PERIODS = 2.5          # a gap between two exposures longer than this many frame periods fails the check
ERROR_FLAG_TAGS = ("j_err", "m_t_err", "m_enc_err", "m_vbus_err", "m_d_err", "m_vel_err", "m_c_err", "m_estop_err",
                   "m_w_err", "gm_t_err", "gm_enc_err", "gm_vbus_err", "gm_d_err", "gm_vel_err", "gm_c_err",
                   "gm_estop_err", "gm_w_err", "g_err", "g_temp_err", "g_tim_err", "g_estop_err", "cb_err")
GRIP_OPEN = 0                     # the gripper's scale (set_gripper, g_p): 0 fully open .. 255 fully closed
GRIP_CLOSED = 255
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


def _flag_set(value) -> bool:
    """An error flag (a number, or one per drive) is set: any value but 0 (NaN = not known = not set)."""
    values = value if isinstance(value, (list, tuple)) else (value,)
    return any(_is_number(v) and v != 0 for v in values)


def _check_packet(k: dict, d: dict) -> None:
    """One packet into its robot's running checks (`k`): the tick's length, the error flags, the mode."""
    period = d.get("period_ms")
    if isinstance(period, (int, float)) and period == period:
        k["longest_ms"] = max(k["longest_ms"], period)
        if period > SLOW_TICK_MS:
            k["slow"] += 1
    for tag in ERROR_FLAG_TAGS:
        value = d.get(tag)
        if value is not None and _flag_set(value):
            k["flags"][tag] = k["flags"].get(tag, 0) + 1
    mode = d.get("mode")
    if isinstance(mode, str) and (not k["modes"] or k["modes"][-1] != mode):
        k["modes"].append(mode)


def _check_frame(k: dict, frame) -> None:
    """One frame into its robot's running checks (`k`): its moment, its tick, its flags, the camera's exposure."""
    k["exposures"].append(frame.exposure_time)
    tick = frame.rti_tick
    if tick is None or tick != tick:
        k["no_tick"] += 1
    else:
        k["frame_ticks"].append(math.floor(tick))
    k["no_clock"] += not frame.camera_clock
    k["uncalibrated"] += not frame.calibrated
    k["auto_exposure"].add(bool(frame.auto_exposure))
    k["exposure_us"].add(frame.exposure_us)


def _camera_checks(c: dict, k: dict) -> dict:
    """The camera's checks of an episode, from its frames (`k`) and its counts (`c`)."""
    ex = k["exposures"]
    steps = [b - a for a, b in zip(ex, ex[1:])]
    period = statistics.median(steps) if steps else 0.0
    gaps = [s for s in steps if s > CAMERA_GAP_PERIODS * period] if period > 0 else []
    first, last = c["first_tick"], c["last_tick"]
    # A frame exposed in tick T has rti_tick T.x. One exposed just after the start can be in the tick before the
    # episode's first packet (that packet happened before the start), so first - 1 is still inside.
    outside = sum(1 for t in k["frame_ticks"] if first is None or t < first - 1 or t > last)
    return {
        "camera_gaps": {"ok": c["frames"] > 0 and not gaps and c["frames_lost"] == 0 and c["frames_skipped"] == 0,
                        "frames": c["frames"], "gaps": len(gaps), "longest_ms": round(max(gaps, default=0.0) * 1e3, 1),
                        "frame_period_ms": round(period * 1e3, 3), "lost": c["frames_lost"],
                        "skipped": c["frames_skipped"]},
        "frame_flags": {"ok": k["no_tick"] == 0 and k["no_clock"] == 0 and k["uncalibrated"] == 0,
                        "no_tick": k["no_tick"], "no_camera_clock": k["no_clock"],
                        "not_calibrated": k["uncalibrated"]},
        "pairing": {"ok": outside == 0, "frames_outside": outside, "first_tick": first, "last_tick": last},
    }


def _camera_settings(k: dict) -> dict:
    return {"auto_exposure": sorted(k["auto_exposure"]), "exposure_us": sorted(k["exposure_us"]),
            "calibrated": k["uncalibrated"] == 0}


def _numbers(result: dict) -> str:
    return ", ".join(f"{name} {value}" for name, value in result.items() if name != "ok")


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
        self.grip_pos: float | None = None    # the gripper's measured position (g_p, 0 open .. 255 closed), newest
        self.grip_target: int | None = None   # the last target the gripper key sent (None: none yet)
        self.status_errors: set = set()       # the error keys of the robot's newest status


class Recorder:
    """Records episodes from one or more robots (each: its packets every tick and its camera) into the formats named.

    Args:
        robots: ``[(name, address), ...]``; names become the recording's topics (``/arm1/...``).
        formats: Format names (``"mcap"``) or text ``"mcap,lerobot"``.
        out_dir: Where each session's folder is made.
        camera: Whether each robot's camera is recorded.
        recipe: The recipe set on each robot while recording (its old one is put back by `close()`).
        port: The robots' command port.
        operator: Who records (stored in each episode).

    Raises:
        ValueError / RuntimeError: as `load_formats`; ValueError for no robot or two with one name.
    """

    def __init__(self, robots, formats=DEFAULT_FORMAT, out_dir: str = "recordings", camera: bool = True,
                 recipe: str = RECIPE, port: int = DEFAULT_PORT, operator: str = "") -> None:
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
        self.operator = operator
        self.session_dir = ""
        self.episodes = 0                    # episodes begun
        self.kept = 0
        self.recording = False
        self.held = 0                        # the episode end(keep=None) left open for decide(); 0: none
        self.last_kept: dict | None = None   # end() / decide()'s answer for the last kept episode (delete_last)
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
            rc.on_status_change(functools.partial(self._on_status, src))
            rc.connect(src.host, self.port)
            src.client = rc
            end = time.monotonic() + 3.0
            st: dict = {}
            while time.monotonic() < end and "telemetry" not in st:
                st = rc.latest_status() or {}
                time.sleep(0.05)
            src.status = {k: st.get(k) for k in ("mode", "state", "homed", "recipe", "telemetry", "protocols",
                                                 "runtime_version")}
            src.status_errors = set(st.get("errors") or ())
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
        if self.held:
            self.decide(keep=True)               # not answered: kept, its failed checks marked
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
            RuntimeError: an episode is open already or waits for `decide()`, less than MIN_FREE_BYTES free on the
                disk, or a format could not open its file (the message says why).
        """
        if self.recording:
            raise RuntimeError("an episode is being recorded already")
        if self.held:
            raise RuntimeError(f"episode {self.held} waits to be kept or deleted (decide())")
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
            "operator": self.operator,
            "recipe": self.recipe or None,
            "start_utc": _iso(start_ns),
            "sources": [{"name": s.name, "host": s.host, "camera": CAMERA if s.camera else None}
                        for s in self.sources],
            "robots": {s.name: {"versions": s.versions, "status": s.status} for s in self.sources},
            "robot_errors_at_start": {s.name: sorted(s.status_errors) for s in self.sources},
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

    def end(self, keep: bool | None = True, notes: dict | None = None) -> dict:
        """End the open episode now. Waits for the items still on their way (at most CLOSE_GRACE_S), then checks it.

        Args:
            keep: True: kept (its files finished and named); False: deleted; None: its files stay open for the
                operator's answers, and `decide()` keeps or deletes it (the answer then says ``"pending": True``).
            notes: More metadata for the episode (each value text or JSON-able).

        Returns:
            {"episode", "kept", "paths", "seconds", "counts", "checks", "checks_failed", "errors"}: `checks` per robot,
            each check {"ok": bool, its numbers}; `checks_failed` one line per failed check ([] when all passed);
            `errors` per format that failed. With keep=None also "pending": True (kept False, paths []).

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
        if result.get("pending"):
            self.held = result["episode"]
        elif result.get("kept"):
            self.kept += 1
            self.last_kept = result
        return result

    def decide(self, keep: bool, notes: dict | None = None) -> dict:
        """Keep or delete the episode `end(keep=None)` left open; `notes` go into its metadata, e.g.
        ``{"success": True, "note": "..."}``. Returns as `end()`.

        Raises:
            RuntimeError: no episode waits for it.
        """
        if not self.held:
            raise RuntimeError("no episode waits to be kept or deleted")
        done = threading.Event()
        result: dict = {}
        self._q.put(("decide", bool(keep), notes or {}, done, result))
        done.wait()
        self.held = 0
        if result.get("kept"):
            self.kept += 1
            self.last_kept = result
        return result

    def delete_last(self) -> int:
        """Delete the last kept episode's files, to record it again: the next `begin()` takes its number. Once: the
        episode before it stays. Returns its number.

        Raises:
            RuntimeError: an episode is open or waits for `decide()`, no kept episode to delete (none yet, or
                deleted already), or the episode went into a LeRobot dataset (``--format lerobot``: it cannot be
                taken out of it here; nothing is deleted).
        """
        if self.recording or self.held:
            raise RuntimeError("end the episode first")
        last = self.last_kept
        if not last:
            raise RuntimeError("no kept episode to delete")
        if any(os.path.isdir(path) for path in last.get("paths", [])):
            raise RuntimeError("the last episode is in a LeRobot dataset (--format lerobot): it cannot be taken out "
                               "here; nothing deleted")
        for path in last.get("paths", []):
            if os.path.exists(path):
                os.remove(path)
        self.kept -= 1
        self.episodes = last["episode"] - 1
        self.last_kept = None
        return last["episode"]

    def gripper(self, robot: str | None = None, speed: int = 100, current: int = 200) -> dict:
        """Open a robot's gripper if it is closed, close it if it is open (the gripper key), with `set_gripper`: at
        once, in any mode, freedrive included.

        Which way: the opposite of the last target this sent; the first time, from the jaws' measured position in
        the newest packet (`g_p` in the open half: close); with neither, close. (The position alone would not do:
        jaws closed on a large part stand in the open half.) The press goes into an open episode
        (``/<robot>/events``).

        Args:
            robot: The robot's name; None: the first robot.
            speed: The jaws' speed, 0-255 on the gripper's scale.
            current: The gripper motor's current limit while the jaws move and hold, in mA (at most the gripper
                XML's `<Ilim>`: the robot refuses more).

        Returns:
            {"robot", "event", "target", "position", "speed", "current", "ok", "error"}: "ok" False and "error"
            the robot's message when it refused (another program holds control, no CAN gripper, a value out of
            range); the direction then stays as it was.

        Raises:
            ValueError: no robot of that name.
            RuntimeError: the recorder is not started.
        """
        src = self._source(robot)
        rc = src.client
        if rc is None:
            raise RuntimeError("the recorder is not started")
        pos = src.grip_pos
        if src.grip_target is not None:
            target = GRIP_OPEN if src.grip_target == GRIP_CLOSED else GRIP_CLOSED
        elif pos is not None:
            target = GRIP_CLOSED if pos < (GRIP_OPEN + GRIP_CLOSED) / 2 else GRIP_OPEN
        else:
            target = GRIP_CLOSED
        event = {"event": "gripper", "target": target, "position": pos, "speed": speed, "current": current}
        try:
            rc.set_gripper(target, speed, current, 1)
            src.grip_target = target
            event.update(ok=True, error=None)
        except (RobotError, ValueError, OSError) as e:
            event.update(ok=False, error=str(e) or type(e).__name__)
        now = time.perf_counter_ns() + self._unix
        self._q.put(("event", src, now, now, event))
        return {"robot": src.name, **event}

    def _source(self, name: str | None) -> RobotSource:
        if name is None:
            return self.sources[0]
        for src in self.sources:
            if src.name == name:
                return src
        raise ValueError(f"no robot {name!r}: the robots are {', '.join(s.name for s in self.sources)}")

    def _on_status(self, src: RobotSource, status: dict) -> None:
        """A status push (the client's callback thread): its error keys noted, the status into the episode."""
        now = time.perf_counter_ns() + self._unix
        src.status_errors = set(status.get("errors") or ())
        self._q.put(("status", src, now, now, dict(status)))

    def stats(self) -> dict:
        """How each robot's links are doing, and the episode: {"robots": {name: {...}}, "recording", "held",
        "episode", "seconds", "queued"}."""
        return {
            "robots": {s.name: {"link": s.link, "camera": s.camera_link, "packets": s.packets, "frames": s.frames,
                                "bad_packets": s.bad_packets, "reconnects": s.reconnects,
                                "camera_reconnects": s.camera_reconnects, "clock_offset_ns": s.clock.offset_ns,
                                "clock_spread_ns": s.clock.spread_ns, "last_error": s.last_error}
                       for s in self.sources},
            "recording": self.recording, "held": self.held, "episode": self.episodes,
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
                    gp = d.get("g_p")
                    if isinstance(gp, (int, float)) and gp == gp:
                        src.grip_pos = float(gp)
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
        ep: dict | None = None                      # the open episode
        held: dict | None = None                    # an ended episode waiting for decide() (its files open)
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
                    self._quit(ep, held)
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
                elif kind == "decide":
                    _, keep, notes, done, result = item
                    if held is not None:
                        result.update(self._close(held, keep, notes))
                        held = None
                    done.set()
                elif ep is not None:
                    self._write(ep, item)
                else:
                    recent.append(item)
                    while item[2] - recent[0][2] > preroll_ns:      # by log time (item[2] in every kind)
                        recent.popleft()
            if ep is not None and ep["stop_ns"] is not None and (not ep["pending"]
                                                                  or time.monotonic() >= ep["deadline"]):
                held = self._ended(ep)
                ep = None

    def _ended(self, ep: dict) -> dict | None:
        """An ended episode, every stream past its end (or the wait over): checked, then closed, or kept open for
        decide() (end(keep=None)); returns it when kept open."""
        self._settle(ep)
        held = None
        if ep["keep"] is None:
            ep["result"].update(ep["summary"], kept=False, paths=[], errors=dict(ep["failed"]), pending=True)
            held = ep
        else:
            ep["result"].update(self._close(ep, ep["keep"], {}))
        ep["done"].set()
        return held

    def _quit(self, ep: dict | None, held: dict | None) -> None:
        """The session's end: an open episode (ended or not) and one waiting for decide() are kept."""
        if ep is not None:
            self._settle(ep)
            ep["result"].update(self._close(ep, ep["keep"] is not False, {}))
            if ep["done"] is not None:
                ep["done"].set()
        if held is not None:
            self._close(held, True, {})

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
        chk = {s.name: {"slow": 0, "longest_ms": 0.0, "flags": {}, "modes": [], "new_errors": set(), "grip_presses": 0,
                        "exposures": [], "frame_ticks": [], "no_tick": 0, "no_clock": 0, "uncalibrated": 0,
                        "auto_exposure": set(), "exposure_us": set()} for s in self.sources}
        at_start = info.get("robot_errors_at_start") or {}
        done.set()
        return {"index": info["episode"], "task": task, "start_ns": start_ns, "stop_ns": None, "handles": handles,
                "failed": {}, "counts": counts, "pending": set(streams), "before": 0, "after": 0, "chk": chk,
                "errors_at_start": {s.name: set(at_start.get(s.name, ())) for s in self.sources}}

    def _write(self, ep: dict, item) -> None:
        kind, src, log_ns = item[0], item[1], item[2]
        if log_ns < ep["start_ns"]:
            ep["before"] += 1
            return
        if ep["stop_ns"] is not None and log_ns >= ep["stop_ns"]:
            ep["pending"].discard((src.name, kind))
            ep["after"] += 1
            return
        c, k = ep["counts"][src.name], ep["chk"][src.name]
        if kind == "packet":
            _check_packet(k, item[5])
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
            _check_frame(k, frame)
        elif kind == "status":
            k["new_errors"] |= set(item[4].get("errors") or ()) - ep["errors_at_start"][src.name]
        elif kind == "event" and item[4].get("event") == "gripper" and item[4].get("ok"):
            k["grip_presses"] += 1
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
                elif kind == "status":
                    mod.add_status(h, src.name, log_ns, item[4])
                elif kind == "event":
                    mod.add_event(h, src.name, log_ns, item[4])
            except Exception as e:  # noqa: BLE001  e.g. the disk full: this format stops, the others go on
                ep["failed"][mod.NAME] = repr(e)

    def _settle(self, ep: dict) -> None:
        """The episode's end: its length, its checks, its metadata (`ep["meta"]`) and the part of end()'s answer that
        does not depend on keeping it (`ep["summary"]`)."""
        if ep["stop_ns"] is None:                       # quit with no end(): kept
            ep.update(keep=True, notes={}, stop_ns=time.perf_counter_ns() + self._unix, done=None, result={})
        seconds = (ep["stop_ns"] - ep["start_ns"]) / 1e9
        checks, failed = self._checks(ep)
        chk = ep["chk"]
        ep["meta"] = {"episode": ep["index"], "task": ep["task"], "start_utc": _iso(ep["start_ns"]),
                      "end_utc": _iso(ep["stop_ns"]), "start_ns": ep["start_ns"], "end_ns": ep["stop_ns"],
                      "seconds": round(seconds, 6), "counts": ep["counts"], "items_before_start": ep["before"],
                      "items_after_end": ep["after"],
                      "unfinished_streams": sorted(f"{a}/{b}" for a, b in ep["pending"]),
                      "checks": checks, "checks_failed": failed,
                      "action_source": {n: {"modes": k["modes"], "gripper": "key" if k["grip_presses"] else None,
                                            "gripper_presses": k["grip_presses"], "teleop": None}
                                        for n, k in chk.items()},
                      "camera": {n: _camera_settings(k) for n, k in chk.items() if k["exposures"]},
                      **ep["notes"]}
        ep["summary"] = {"episode": ep["index"], "seconds": seconds, "counts": ep["counts"], "checks": checks,
                         "checks_failed": failed}

    def _checks(self, ep: dict) -> tuple:
        """Each robot's checks: ({robot: {check: {"ok": bool, its numbers}}}, [one line per failed check])."""
        checks: dict = {}
        failed: list = []
        for src in self.sources:
            c, k = ep["counts"][src.name], ep["chk"][src.name]
            res = {
                "ticks": {"ok": c["tick_gaps"] == 0 and c["tick_restarts"] == 0 and c["packets"] > 0,
                          "missing": c["tick_gaps"], "restarts": c["tick_restarts"], "packets": c["packets"]},
                "slow_ticks": {"ok": k["slow"] == 0, "count": k["slow"], "longest_ms": round(k["longest_ms"], 3),
                               "limit_ms": SLOW_TICK_MS},
                "error_flags": {"ok": not k["flags"], "packets": dict(k["flags"])},
                "robot_errors": {"ok": not k["new_errors"], "new": sorted(k["new_errors"]),
                                 "at_start": sorted(ep["errors_at_start"][src.name])},
                "mode": {"ok": len(k["modes"]) <= 1, "modes": k["modes"]},
            }
            if src.camera:
                res.update(_camera_checks(c, k))
            checks[src.name] = res
            failed += [f"{src.name} {name}: {_numbers(r)}" for name, r in res.items() if not r["ok"]]
        return checks, failed

    def _close(self, ep: dict, keep: bool, notes: dict) -> dict:
        """Keep (the metadata with `notes` written, the files finished and named) or delete the episode's files.
        Returns end()'s answer."""
        meta = {**ep["meta"], **notes}
        paths, errors = [], dict(ep["failed"])
        for mod, h in ep["handles"]:
            try:
                if keep and mod.NAME not in ep["failed"]:
                    if isinstance(h, dict) and h.get("state_errors"):
                        meta["viewer_errors"] = {"count": h["state_errors"], "last": h.get("last_state_error")}
                    paths.append(mod.close_episode(h, meta))
                else:
                    mod.discard_episode(h)
            except Exception as e:  # noqa: BLE001
                errors[mod.NAME] = repr(e)
        return {**ep["summary"], "kept": bool(keep) and bool(paths), "paths": paths, "errors": errors}
