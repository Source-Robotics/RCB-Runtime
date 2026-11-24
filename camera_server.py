"""The camera server: the gripper's USB camera, relayed over the network and stamped with RTI's tick (2026-11-23,
TODO/camera_plan.md P1).

Started by supervisor.py next to RTI, the dashboard, the executor, the self-collision worker and the recorder; normal
scheduling on the general cores (config/cpu_cores.py), its own address-space cap. It never imports RTI's code: it reads
RTI's shared-memory trace (utility/trace_ring.py) only, to turn a frame's time into an RTI tick.

  * Settings: config/system.xml <camera .../> (utility/camera_settings.py), read at start and within ~1 s of the file
    changing (the Camera tab writes them; a live gripper swap changes <active_gripper>). "auto" = the camera the
    active gripper's XML declares (<camera usb_id="..."/>); none declared = no camera (the user's D3).
  * The camera: MJPEG in the configured mode (hardware/uvc_camera.py), its controls applied at every open; reopened
    within ~1 s after an unplug and replug (proven 2026-11-23). Its USB controllers' interrupt THREADS go to the general
    cores (their interrupts cannot be given an affinity on the Pi 5: EIO), as RTI does for the CAN path.
  * Frames are never decoded or re-encoded. One HTTP port, config/networking.CAMERA_PORT (the user, 2026-11-24: one
    port, as the Pi webcam servers have it):
      /frames         every frame with its header (communication/camera_protocol.py: its time, RTI's tick), programs
      /stream         every frame as MJPEG (multipart), browsers and video tools
      /preview        MJPEG at preview_fps (the dashboard's Camera tab)
      /snapshot.jpg   the latest frame;  /  a page with the stream
    A client whose previous frame is still going out SKIPS the new one: no queue, so nobody ever sees old frames.
  * Frame times (P2, 2026-11-24): the camera's own clock. The camera's UVC metadata node is streamed with the video;
    utility/uvc_clock.py ties the camera's clock to CLOCK_MONOTONIC (+-9 us) and turns each frame's PTS into its time
    (FLAG_CAMERA_CLOCK; the raw PTS in the header). Until that clock is known (~0.2 s after an open), without a
    metadata node, or when the result disagrees with the driver's timestamp: the driver's timestamp (4 ms steps).
    RTI's tick is taken at the frame's time.
  * Status: JSON in shared memory (camera_protocol.STATUS_NAME) once a second, with a "beat" counter the supervisor's
    hang check watches (the user's D1).

Measured before it was built (the feasibility tools, ~/Desktop/camera_test/, 2026-11-21 / 23): 720p30 at ~11 Mbit/s,
under 1 % of one core, no measurable effect on the CAN bus, RTI's loop or the RTI link (api_tests/69 x5, 25 from the
PC). Logs to logs/camera.log (logger "cam").
"""
from __future__ import annotations

import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")   # numpy (the trace reader) under the address-space cap

import bisect                                                                     # noqa: E402
import errno                                                                      # noqa: E402
from collections import deque                                                     # noqa: E402
import logging                                                                    # noqa: E402
import math                                                                       # noqa: E402
import resource                                                                   # noqa: E402
import select                                                                     # noqa: E402
import signal                                                                     # noqa: E402
import socket                                                                     # noqa: E402
import subprocess                                                                 # noqa: E402
import sys                                                                        # noqa: E402
import time                                                                       # noqa: E402

_ROOT = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _ROOT)

from communication import camera_protocol as cp                                   # noqa: E402
from config.cpu_cores import GENERAL_CPU_CORES                                    # noqa: E402
from config.networking import CAMERA_PORT                                         # noqa: E402
from project_paths import LOGS_DIR, SYSTEM_XML                                    # noqa: E402
from utility import camera_settings                                               # noqa: E402
from utility.logger_setup import setup_logging                                    # noqa: E402
from utility.uvc_clock import CameraClock                                         # noqa: E402

logger = logging.getLogger("cam")  # docs/logging.md "Logger names"
LOG_FILE = os.path.join(str(LOGS_DIR), "camera.log")
GRIPPERS_DIR = os.path.join(_ROOT, "grippers")
MEMORY_CAP_MB = 512               # its own RLIMIT_AS (the supervisor sets none); measured 20 MB RSS while streaming
SETTINGS_CHECK_S = 1.0            # system.xml's modification time, looked at once a second
STATUS_EVERY_S = 1.0
REOPEN_S = 1.0                    # a missing or failed camera is tried again this often
TRACE_POLL_S = 0.1
TICK_WINDOW_S = 2.0               # trace rows kept for the time -> tick mapping
TICK_EDGE_S = 0.5                 # a time this far outside the rows kept has no tick (NaN)
MAX_CLIENTS = 8
SEND_BUFFER = 256 * 1024          # small: a slow client skips frames instead of seeing old ones
# The driver's timestamp (the transfer of the frame's first packet) comes this long after the camera's own time of
# the frame: measured 1.2-5.1 ms (2026-11-24). A camera time outside is not used for that frame (counted: disagreed).
CLOCK_LAG_S = (-0.0005, 0.025)
PTS_KEPT = 32                     # frames' PTS kept until their video frame is read
_stop = False


def _on_term(_s, _f):
    global _stop
    _stop = True


def _mtime(path):
    try:
        return os.stat(path).st_mtime_ns
    except OSError:
        return None


# ---------------------------------------------------------------- RTI's tick at a time

class TickMap:
    """CLOCK_MONOTONIC time -> RTI's loop tick, from the trace rows' (timestamp, tick) pairs (each row's timestamp is
    time.perf_counter() taken mid-tick, RTI.py's _hist_t: CLOCK_MONOTONIC on Linux, the clock of V4L2's timestamps).
    Interpolated between rows, extrapolated at most TICK_EDGE_S beyond them; NaN without rows."""

    def __init__(self, window_s: float = TICK_WINDOW_S):
        self.window_s = window_s
        self.times: list[float] = []
        self.ticks: list[float] = []

    def add(self, times, ticks) -> None:
        for t, k in zip(times, ticks):
            if self.times and t <= self.times[-1]:
                continue
            self.times.append(float(t))
            self.ticks.append(float(k))
        if self.times:
            cut = bisect.bisect_left(self.times, self.times[-1] - self.window_s)
            del self.times[:cut], self.ticks[:cut]

    def clear(self) -> None:
        self.times.clear()
        self.ticks.clear()

    def tick_at(self, t: float) -> float:
        n = len(self.times)
        if n < 2 or t < self.times[0] - TICK_EDGE_S or t > self.times[-1] + TICK_EDGE_S:
            return math.nan
        i = min(max(bisect.bisect_left(self.times, t), 1), n - 1)
        t0, t1, k0, k1 = self.times[i - 1], self.times[i], self.ticks[i - 1], self.ticks[i]
        return k0 + (t - t0) * (k1 - k0) / (t1 - t0) if t1 > t0 else k1


# ---------------------------------------------------------------- the camera's USB interrupt threads

def pin_usb_irq_threads(cores, proc_root: str = "/proc") -> dict:
    """Every USB controller interrupt thread (irq/<n>-xhci...) onto `cores` with `sudo -n taskset` (the Pi 5's USB
    interrupts take no affinity themselves). {thread name: "moved" / "already" / an error}. Never raises."""
    out = {}
    want = set(cores)
    try:
        with open(f"{proc_root}/interrupts") as f:
            numbers = {line.split(":", 1)[0].strip() for line in f if "xhci" in line}
    except OSError as e:
        return {"/proc/interrupts": str(e)}
    for comm in os.listdir(proc_root):
        if not comm.isdigit():
            continue
        try:
            with open(f"{proc_root}/{comm}/comm") as f:
                name = f.read().strip()
        except OSError:
            continue
        if not (name.startswith("irq/") and name.split("/", 1)[1].split("-", 1)[0] in numbers):
            continue
        pid = int(comm)
        try:
            if set(os.sched_getaffinity(pid)) == want:
                out[name] = "already"
                continue
            subprocess.run(["sudo", "-n", "taskset", "-pc", ",".join(map(str, sorted(want))), str(pid)],
                           check=True, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=5)
            out[name] = "moved"
        except (OSError, subprocess.SubprocessError) as e:
            out[name] = f"not moved: {e}"
    return out


# ---------------------------------------------------------------- clients

PAGE = (b"<!doctype html><meta charset=\"utf-8\"><title>Robot camera</title>"
        b"<body style=\"margin:0;background:#111\"><img src=\"/stream\" style=\"max-width:100%;display:block;"
        b"margin:auto\">")


class Client:
    """One connection. kind: "raw" (frames with a header), "stream" / "preview" (HTTP MJPEG), "http" (a request not
    read yet, or a one-off reply). A frame is queued only when the previous one has gone out (else it is skipped)."""

    def __init__(self, sock, addr, kind):
        self.sock, self.kind = sock, kind
        self.addr = f"{addr[0]}:{addr[1]}"
        self.pending = None               # memoryview still to send
        self.closing = False              # close once pending is out (a page / a snapshot)
        self.inbuf = b""
        self.frames = self.skipped = 0
        self.last_preview = 0.0

    def queue(self, payload: bytes, is_frame: bool = True) -> bool:
        if self.pending is not None:
            self.skipped += is_frame
            return False
        self.pending = memoryview(payload)
        self.frames += is_frame
        return True

    def flush(self) -> bool:
        """False when the client is gone, or done (a one-off reply sent)."""
        if self.pending is not None:
            try:
                n = self.sock.send(self.pending)
            except BlockingIOError:
                return True
            except OSError:
                return False
            self.pending = self.pending[n:] if n < len(self.pending) else None
        return not (self.closing and self.pending is None)

    def receive(self) -> bytes | None:
        """The bytes waiting; b"" when nothing is there; None when the client is gone."""
        try:
            data = self.sock.recv(4096)
        except BlockingIOError:
            return b""
        except OSError:
            return None
        return data or None


def _http_reply(status: str, ctype: str, body: bytes) -> bytes:
    return (f"HTTP/1.0 {status}\r\nContent-Type: {ctype}\r\nContent-Length: {len(body)}\r\nCache-Control: no-cache\r\n"
            f"Access-Control-Allow-Origin: *\r\n\r\n").encode() + body


STREAM_HEAD = (b"HTTP/1.0 200 OK\r\nCache-Control: no-cache\r\nAccess-Control-Allow-Origin: *\r\n"
               b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n")
FRAMES_HEAD = (b"HTTP/1.0 200 OK\r\nCache-Control: no-cache\r\nContent-Type: application/x-rcb-camera-frames; "
               b"protocol=%d\r\n\r\n" % cp.CAMERA_PROTOCOL_VERSION)


def mjpeg_part(jpeg: bytes) -> bytes:
    return b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg) + jpeg + b"\r\n"


def http_route(request: bytes, latest: bytes | None):
    """(new kind, bytes to send first, close after) for an HTTP request's first bytes."""
    parts = request.split(b" ", 2)
    path = parts[1].split(b"?", 1)[0] if len(parts) > 1 else b"/"
    if path == b"/frames":
        return "raw", FRAMES_HEAD, False
    if path == b"/stream":
        return "stream", STREAM_HEAD, False
    if path == b"/preview":
        return "preview", STREAM_HEAD, False
    if path == b"/snapshot.jpg":
        if latest is None:
            return "http", _http_reply("503 Service Unavailable", "text/plain", b"no frame yet"), True
        return "http", _http_reply("200 OK", "image/jpeg", latest), True
    if path == b"/":
        return "http", _http_reply("200 OK", "text/html", PAGE), True
    return "http", _http_reply("404 Not Found", "text/plain", b"not found"), True


def _listener(port: int):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", port))  # nosec B104 — the robot's own network (the isolated-network policy)
    s.listen(8)
    s.setblocking(False)
    return s


# ---------------------------------------------------------------- choosing the camera

def choose_device(settings: dict, gripper_cam: dict | None, cameras: list[dict]):
    """(path or None, why not). "auto" = the active gripper's camera by its USB id."""
    dev = settings["device"]
    if dev == "auto":
        usb = ((gripper_cam or {}).get("usb_id") or "").strip().lower()
        if not usb:
            return None, "the active gripper declares no camera"
    elif dev.startswith("usb:"):
        usb = dev[4:].lower()
    else:
        return (dev, "") if os.path.exists(dev) else (None, f"{dev} not found")
    for c in cameras:
        if c["usb_id"] == usb:
            return c["path"], ""
    others = ", ".join(f"{c['usb_id']} ({c['path']})" for c in cameras)
    return None, (f"no camera with USB id {usb} plugged in" +
                  (f"; found {others} — set <camera device=\"usb:...\"> in system.xml to use one" if others else ""))


def _without_timing(gcam: dict | None) -> dict | None:
    """The gripper's camera attributes without the timing calibration's (a change of those alone needs no reopen)."""
    return None if gcam is None else {k: v for k, v in gcam.items() if k not in camera_settings.TIMING_KEYS}


def _timing_why(own: bool, setting, timing, mode: str) -> str:
    """Why a frame's time is not calibrated ("" when it is)."""
    if not own:
        return "the camera's own clock is not in use"
    if timing is None:
        return f"no timing calibration for {mode} (api_tests/80_camera_timing.py)"
    if setting is None and timing["auto_us"] is None:
        return "calibrated for fixed exposures only; the camera is on automatic exposure"
    if setting is not None and timing["offset_us"] is None:
        return "calibrated for automatic exposure only; the camera is on a fixed exposure"
    return ""


# ---------------------------------------------------------------- the server

class CameraServer:
    def __init__(self, status_str):
        from hardware import uvc_camera
        self.uvc = uvc_camera
        self.status_str = status_str
        self.settings, problems = camera_settings.read(SYSTEM_XML)
        self.problems = list(problems)
        self.settings_mtime = _mtime(SYSTEM_XML)
        self.gripper_cam, self.gripper = camera_settings.gripper_camera(SYSTEM_XML, GRIPPERS_DIR)
        self.cam = None
        self.cameras_seen: list = []       # the cameras found at the last open attempt (the Camera tab offers them)
        self.device = None                 # {"path", "usb_id"} of the open camera
        self.describe: dict = {}
        self.controls: list = []
        self.control_problems: list = []
        self.control_notes: list = []
        self.state, self.message = "starting", ""
        self.next_open = 0.0
        self.listeners = {_listener(CAMERA_PORT): "http"}
        self.clients: list[Client] = []
        self.latest: bytes | None = None
        self.frame_no = self.frames = self.lost = self.reopens = 0
        self.prev_seq = None
        self.last_frame = 0.0
        self.window = []                   # (time, bytes) of the last 2 s, for fps / Mbit/s
        self.tickmap = TickMap()
        self.clock = CameraClock()             # the camera's own clock (P2)
        self.pts: dict = {}                    # driver sequence -> its frame's PTS, until the frame is read
        self.clock_frames = self.driver_frames = self.clock_disagreed = 0
        self.lags: deque = deque(maxlen=90)    # the driver's timestamp minus the camera's time, recent frames
        self.time_source = None                # "camera" / "driver": what the last frame's time came from
        self.calibrated = False                # the last frame's time includes the timing calibration (P3)
        self.timing_why = ""                   # ... and why not
        self.trace = None
        self.trace_cols = None
        self.next_trace = 0.0
        self.beat = 0
        self.irq = {}
        for p in self.problems:
            logger.warning(f"system.xml <camera>: {p} — its default is used")

    # ---- settings and the camera

    def check_settings(self) -> None:
        m = _mtime(SYSTEM_XML)
        gcam, gname = camera_settings.gripper_camera(SYSTEM_XML, GRIPPERS_DIR)
        if m == self.settings_mtime:
            # the gripper XML alone changed its timing calibration (api_tests/80 --apply): taken without a reopen
            if gname == self.gripper and gcam != self.gripper_cam and _without_timing(gcam) == _without_timing(
                    self.gripper_cam):
                logger.info(f"camera timing calibration loaded from grippers/{gname}.xml: "
                            f"{ {k: v for k, v in (gcam or {}).items() if k in camera_settings.TIMING_KEYS} }")
                self.gripper_cam = gcam
                return
            if gcam == self.gripper_cam:
                return
        self.settings_mtime = m
        new, problems = camera_settings.read(SYSTEM_XML)
        for p in problems:
            logger.warning(f"system.xml <camera>: {p} — its default is used")
        self.problems = list(problems)
        reopen_keys = ("enabled", "device", "width", "height", "fps")
        reopen = (any(new[k] != self.settings[k] for k in reopen_keys)
                  or _without_timing(gcam) != _without_timing(self.gripper_cam))
        controls_changed = new["controls"] != self.settings["controls"]
        if new != self.settings or gname != self.gripper:
            logger.info(f"settings changed: {self.settings} -> {new}; gripper {self.gripper} -> {gname}")
        self.settings, self.gripper_cam, self.gripper = new, gcam, gname
        if reopen:
            self.close_camera("the settings changed")
            self.next_open = 0.0
        elif controls_changed and self.cam is not None:
            self.apply_controls()

    def _camera_controls(self) -> dict:
        try:
            return {c["key"]: c for c in self.cam.controls()}
        except (OSError, AttributeError):
            return {}

    def apply_controls(self) -> None:
        """The camera's controls = system.xml's <camera controls>; every control NOT named there goes back to the
        camera's own default. A camera keeps a value until it loses power, so a control taken out of the setting
        stayed where it was — found 2026-11-24: a fixed exposure stayed after it was removed, and the Camera tab's
        "back to the defaults" reset nothing but the dynamic frame rate."""
        self.control_problems = []
        self.control_notes = []                    # set, but the camera ignores it now (its automatic mode is on)
        try:
            wanted = camera_settings.parse_controls(self.settings["controls"])
        except ValueError as e:
            self.control_problems.append(str(e))
            wanted = []
        named = dict(wanted)

        def is_mode(key):
            return key.startswith("auto_") or key.endswith(("_automatic", "_auto"))

        # The automatic modes first: a manual value (exposure_time_absolute, white_balance_temperature) is ignored
        # while its automatic mode is still on (the control reads "inactive"); then the camera is asked again.
        for modes in (True, False):
            current = self._camera_controls()
            steps = [(k, v, True) for k, v in wanted if is_mode(k) == modes]
            steps += [(k, c["default"], False) for k, c in current.items()
                      if k not in named and is_mode(k) == modes and not c["read_only"] and not c["inactive"]
                      and c["type"] in ("int", "bool", "menu", "intmenu") and c["value"] is not None
                      and c["value"] != c["default"]]
            for key, value, explicit in steps:
                self._set_one(key, value, explicit)
        self._report_controls()

    def _set_one(self, key: str, value: int, explicit: bool) -> None:
        try:
            self.cam.set_control(key, value)
            if not explicit:
                logger.info(f"camera control: {key} back to its default {value} (not in <camera controls>)")
        except KeyError:
            self.control_problems.append(f"{key}: this camera has no such control")
        except OSError as e:
            if e.errno == errno.EACCES:            # an inactive control: kept, applied once its automatic mode is off
                if explicit:
                    self.control_notes.append(f"{key}: not used while its automatic mode is on (kept for when it is "
                                              f"off)")
            else:
                self.control_problems.append(f"{key}: {e}")
        except ValueError as e:
            self.control_problems.append(f"{key}: {e}")

    def _report_controls(self) -> None:
        for n in self.control_notes:
            logger.info(f"camera control: {n}")
        for p in self.control_problems:
            logger.warning(f"camera control: {p}")
        self.refresh_controls()

    def refresh_controls(self) -> None:
        try:
            self.controls = self.cam.controls() if self.cam is not None else []
        except OSError:
            self.controls = []

    def open_camera(self, now: float) -> None:
        self.next_open = now + REOPEN_S
        if not self.settings["enabled"]:
            self._set_state("off", "switched off (the Camera tab / system.xml <camera enabled>)")
            return
        try:
            cameras = self.uvc.find_cameras()
        except OSError as e:
            cameras = []
            logger.debug(f"listing cameras failed: {e}")
        self.cameras_seen = cameras
        path, why = choose_device(self.settings, self.gripper_cam, cameras)
        if path is None:
            self._set_state("no camera", why)
            return
        try:
            self.describe = self.uvc.describe(path)
            self.cam = self.uvc.Camera(path, self.settings["width"], self.settings["height"], self.settings["fps"],
                                       meta=True)
        except OSError as e:
            self.cam = None
            hint = " — is the robot's user in the 'video' group? (scripts/setup_system.sh)" if e.errno == 13 else ""
            self._set_state("error", f"{path}: {e}{hint}")
            return
        self.device = {"path": path, "usb_id": next((c["usb_id"] for c in cameras if c["path"] == path), "")}
        self.apply_controls()
        self.irq = pin_usb_irq_threads(GENERAL_CPU_CORES)
        bad = {k: v for k, v in self.irq.items() if v not in ("moved", "already")}
        if bad:
            logger.warning(f"USB interrupt threads not on the general cores: {bad}")
        self.prev_seq = None
        self.last_frame = now
        self.clock.reset()
        self.pts.clear()
        self.lags.clear()
        self._set_state("streaming", "")
        meta = self.cam.meta_path
        logger.info(f"camera open: {path} ({self.device['usb_id']}), MJPEG {self.cam.width}x{self.cam.height} at "
                    f"{self.cam.fps:g} fps; USB interrupt threads {self.irq}; frame times: "
                    + (f"the camera's own clock (metadata {meta})" if meta else "the driver's timestamps"))
        if not meta:
            logger.warning(f"the camera's own clock is not available ({self.cam.meta_problem}): frames carry the "
                           f"driver's timestamps (4 ms steps)")

    def close_camera(self, why: str) -> None:
        if self.cam is None:
            return
        try:
            self.cam.close()
        except OSError:
            pass
        self.cam = None
        self.controls = []
        self.latest = None                         # no stale snapshot
        logger.info(f"camera closed: {why}")

    def _set_state(self, state: str, message: str) -> None:
        if (state, message) != (self.state, self.message):
            log = logger.warning if state == "error" else logger.info
            log(f"camera {state}{': ' + message if message else ''}")
        self.state, self.message = state, message

    # ---- frames

    def on_frame(self, frame, now: float) -> None:
        data, ts, seq, _flags = frame
        self.last_frame = now
        if self.message:                           # "no frame for N s" is over
            self._set_state("streaming", "")
        if self.prev_seq is not None:
            self.lost += max(0, seq - self.prev_seq - 1)
        self.prev_seq = seq
        self.frame_no += 1
        self.frames += 1
        self.latest = data
        self.window.append((now, len(data)))
        values = {c["key"]: c["value"] for c in self.controls}
        flags = 0
        exposure_us = 0
        setting = None                                            # the fixed exposure setting, None = automatic
        if values.get("auto_exposure") not in (None, 1):          # 1 = manual on UVC cameras
            flags |= cp.FLAG_AUTO_EXPOSURE
        elif values.get("exposure_time_absolute") is not None:
            setting = int(values["exposure_time_absolute"])
            exposure_us = setting * camera_settings.EXPOSURE_UNIT_US   # UVC: 100 us units (confirmed by the frame rate)
        timing = camera_settings.camera_timing(self.gripper_cam, camera_settings.mode_key(
            self.cam.width, self.cam.height, self.cam.fps))
        t_frame, pts, own = self.frame_time(seq, ts)
        if own:
            flags |= cp.FLAG_CAMERA_CLOCK
            offset = camera_settings.timing_offset_s(timing, setting)
            if offset is not None:
                t_frame += offset
                flags |= cp.FLAG_CALIBRATED
        self.calibrated = bool(flags & cp.FLAG_CALIBRATED)
        self.timing_why = _timing_why(own, setting, timing, camera_settings.mode_key(self.cam.width, self.cam.height,
                                                                                     self.cam.fps))
        tick = self.tickmap.tick_at(t_frame)
        if not math.isnan(tick):
            flags |= cp.FLAG_TICK
        raw = part = None
        for c in self.clients:
            if c.kind == "raw":
                if raw is None:
                    raw = cp.pack(self.frame_no, seq, self.cam.width, self.cam.height, flags, t_frame, tick,
                                  exposure_us, pts, now, len(data)) + data
                c.queue(raw)
            elif c.kind in ("stream", "preview"):
                if c.kind == "preview":
                    if now - c.last_preview < 1.0 / float(self.settings["preview_fps"]) - 0.005:
                        continue
                    if c.pending is None:
                        c.last_preview = now
                if part is None:
                    part = mjpeg_part(data)
                c.queue(part)

    def frame_time(self, seq: int, ts: float) -> tuple[float, int, bool]:
        """(the frame's time, its raw PTS or 0, from the camera's own clock?). The camera's metadata that arrived is
        taken first (it is finished just before its video frame). The driver's timestamp `ts` when the camera's clock
        is not known yet, the camera has no metadata, or its time disagrees with `ts` (CLOCK_LAG_S)."""
        read_meta = getattr(self.cam, "read_meta", None)
        if read_meta is not None:
            for mseq, err, data in read_meta():
                p = self.clock.add(data)
                if p is not None and not err:
                    self.pts[mseq] = p
            while len(self.pts) > PTS_KEPT:
                del self.pts[next(iter(self.pts))]
        pts = self.pts.pop(seq, None)
        t = self.clock.time_of(pts) if pts is not None else None
        if t is not None and CLOCK_LAG_S[0] <= ts - t <= CLOCK_LAG_S[1]:
            self.lags.append(ts - t)
            self.clock_frames += 1
            self.time_source = "camera"
            return t, pts, True
        if t is not None:
            self.clock_disagreed += 1
        self.driver_frames += 1
        self.time_source = "driver"
        return ts, pts or 0, False

    def clock_status(self) -> dict:
        """The frame-time part of the status: where the times come from and how well the camera's clock fits."""
        h = self.clock.health()
        meta = getattr(self.cam, "meta_path", None) if self.cam is not None else None
        if self.cam is not None and not meta:      # no metadata: that is why, whatever the clock held before
            why = getattr(self.cam, "meta_problem", "") or "no UVC metadata"
        else:
            why = "" if h["ok"] else h["why"]
        lags = sorted(self.lags)
        return {"source": self.time_source if self.cam is not None else None, "metadata": meta,
                "why": why, "pairs": h["pairs"], "fit_us": h["fit_us"],
                "device_mhz": h["device_mhz"], "frames_camera_clock": self.clock_frames,
                "frames_driver_time": self.driver_frames, "disagreed": self.clock_disagreed,
                "restarts": h["restarts"],
                "driver_after_ms": round(lags[len(lags) // 2] * 1e3, 2) if lags else None,
                "calibrated": self.calibrated if self.cam is not None else None,
                "calibration": (camera_settings.camera_timing(self.gripper_cam, camera_settings.mode_key(
                    self.cam.width, self.cam.height, self.cam.fps)) if self.cam is not None else None),
                "calibration_why": self.timing_why if self.cam is not None else ""}

    def poll_trace(self, now: float) -> None:
        if now < self.next_trace:
            return
        self.next_trace = now + TRACE_POLL_S
        try:
            if self.trace is None or self.trace.stale():
                from utility.trace_ring import TraceReader
                self.trace = TraceReader.attach()
                self.tickmap.clear()
                if self.trace is None:
                    return
                cols = {name: col for name, col in self.trace.layout["scalars"]}
                self.trace_cols = (cols["timestamp"], cols["tick"])
            rows, _lost = self.trace.poll()
            if len(rows):
                self.tickmap.add(rows[:, self.trace_cols[0]], rows[:, self.trace_cols[1]])
        except Exception as e:                     # the tick is a bonus: the stream goes on without it
            logger.debug(f"trace not readable: {e}")
            self.trace = None

    # ---- status

    def publish(self, now: float) -> None:
        self.window = [(t, n) for t, n in self.window if now - t <= 2.0]
        fps = mbit = 0.0
        if len(self.window) >= 2 and self.window[-1][0] > self.window[0][0]:
            span = self.window[-1][0] - self.window[0][0]
            fps = (len(self.window) - 1) / span
            mbit = sum(n for _, n in self.window[1:]) * 8 / span / 1e6
        status = {
            "beat": self.beat, "time": time.time(), "pid": os.getpid(),
            "state": self.state, "message": self.message,
            "gripper": self.gripper, "gripper_camera": self.gripper_cam, "cameras": self.cameras_seen,
            "device": self.device if self.cam is not None else None,
            "card": self.describe.get("card", "") if self.cam is not None else "",
            "mode": ({"width": self.cam.width, "height": self.cam.height, "fps": self.cam.fps}
                     if self.cam is not None else None),
            "modes": [m for m in self.describe.get("modes", []) if m["format"] == "MJPG"] if self.cam else [],
            "controls": [{k: c[k] for k in ("key", "name", "type", "min", "max", "step", "default", "value", "menu",
                                            "read_only", "inactive")} for c in self.controls],
            "fps": round(fps, 1), "mbit_s": round(mbit, 2),
            "frames": self.frames, "lost": self.lost, "reopens": self.reopens,
            "tick_mapping": bool(self.tickmap.times),
            "clock": self.clock_status(),
            "clients": [{"kind": c.kind, "addr": c.addr, "frames": c.frames, "skipped": c.skipped}
                        for c in self.clients if c.kind != "http"],
            "settings": self.settings, "problems": self.problems + self.control_problems,
            "notes": self.control_notes,
            "irq_threads": self.irq,
            "port": CAMERA_PORT,
            "protocol": cp.CAMERA_PROTOCOL_VERSION,
        }
        try:
            self.status_str.set_json(status)
        except Exception as e:                     # too big for the segment, or shared memory gone
            logger.warning(f"status not published: {e}")

    # ---- the loop

    def step(self, now: float) -> None:
        self.beat += 1
        if self.cam is None and now >= self.next_open:
            self.open_camera(now)
        elif self.cam is not None and now - self.last_frame > 3.0:
            self._set_state("streaming", f"no frame for {now - self.last_frame:.0f} s")
        self.poll_trace(now)
        rlist = list(self.listeners) + [c.sock for c in self.clients] + ([self.cam] if self.cam else [])
        wlist = [c.sock for c in self.clients if c.pending is not None]
        readable, writable, _ = select.select(rlist, wlist, [], 0.2)
        now = time.monotonic()
        if self.cam is not None and self.cam in readable:
            try:
                frame = self.cam.read()
            except OSError as e:
                self.close_camera(f"lost ({e})")
                self.reopens += 1
                self._set_state("no camera", f"lost: {e}")
                frame = None
            if frame is not None:
                self.on_frame(frame, now)
        for sock, kind in self.listeners.items():
            if sock in readable:
                try:
                    conn, addr = sock.accept()
                except OSError:
                    continue
                if len(self.clients) >= MAX_CLIENTS:
                    conn.close()
                    logger.warning(f"refused {addr[0]}: {MAX_CLIENTS} clients already")
                    continue
                conn.setblocking(False)
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                conn.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, SEND_BUFFER)
                self.clients.append(Client(conn, addr, kind))
        gone = []
        for c in self.clients:
            alive = True
            if c.sock in readable:
                data = c.receive()
                if data is None:
                    alive = False
                elif c.kind == "http" and not c.closing and data:
                    c.inbuf += data
                    if b"\r\n\r\n" in c.inbuf or len(c.inbuf) > 8192:
                        c.kind, first, c.closing = http_route(c.inbuf, self.latest)
                        c.queue(first, is_frame=False)
                        if c.kind == "raw":
                            logger.info(f"frames client connected: {c.addr}")
            if alive and (c.sock in writable or c.pending is not None):
                alive = c.flush()
            if not alive:
                gone.append(c)
        for c in gone:
            self.clients.remove(c)
            try:
                c.sock.close()
            except OSError:
                pass
            if c.kind == "raw":
                logger.info(f"frames client gone: {c.addr} ({c.frames} frames, {c.skipped} skipped for it)")

    def close(self) -> None:
        self.close_camera("stopping")
        for c in self.clients:
            c.sock.close()
        for s in self.listeners:
            s.close()


def main() -> int:
    setup_logging(LOG_FILE, log_to_console=False)
    from utility.process_utils import setup_process
    setup_process(GENERAL_CPU_CORES)                     # normal scheduling, never RTI's or the CAN core
    cap = MEMORY_CAP_MB << 20
    _soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (cap if hard == resource.RLIM_INFINITY else min(cap, hard), hard))
    signal.signal(signal.SIGTERM, _on_term)
    signal.signal(signal.SIGINT, _on_term)
    from data.shared_data import SharedString
    status_str = SharedString(cp.STATUS_NAME, cp.STATUS_SIZE)
    server = CameraServer(status_str)
    logger.info(f"camera server up: HTTP on {CAMERA_PORT} (/frames, /stream, /preview, /snapshot.jpg); "
                f"settings {server.settings}; gripper {server.gripper} camera {server.gripper_cam}")
    last_settings = last_status = 0.0
    try:
        while not _stop:
            now = time.monotonic()
            try:
                if now - last_settings > SETTINGS_CHECK_S:
                    last_settings = now
                    server.check_settings()
                server.step(now)
                if now - last_status > STATUS_EVERY_S:
                    last_status = now
                    server.refresh_controls()
                    server.publish(now)
            except Exception:                            # the server must keep running; the supervisor sees a crash
                logger.exception("camera server pass failed")
                time.sleep(0.5)
    finally:
        server.close()
        logger.info(f"camera server stopped: {server.frames} frames, {server.lost} lost by the driver")
    return 0


if __name__ == "__main__":
    sys.exit(main())
