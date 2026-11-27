"""The PAR6 arm's live link for LeRobot's programs: everything the plugin does, without LeRobot itself.

`par6.PAR6` (LeRobot's ``Robot``) only hands its calls to a `Link`; this module imports nothing from LeRobot, so the
robot's own tests check it on the robot's Python (TODO/p4_recording_plan.md step 4b, design and the user's decisions
of 2026-11-27).

What it does:

- **Connect.** A `RobotClient`, the arm's state checked: not homed or with errors -> refused (it never homes the
  arm and never clears an error). Then what can fail without touching the arm: the PC's UDP port for the state
  packets (the real-time client binds it; another program holding it -> refused, saying how to find it) and the wrist
  camera (newest frame only). Only then the arm: disabled -> enabled (it holds where it is), the robot's ``record``
  recipe for the session (each joint's reply time is in it; the old recipe back at the end), the real-time link in
  SOFT_RTI at `fps` with control claimed. The first target is the measured pose.
- **Observe.** The newest camera frame, and the state AT THAT FRAME'S MOMENT, computed by the export's own functions
  (`robotclient.recording.export.columns` / `state_at` / `image`): each joint interpolated on its own measurement
  times, the gripper 0 open .. 1 closed, the image resized the same way. A policy sees what it was trained on.
- **Act.** Joint targets (rad) capped: each target at most `max_speed_deg_s` from the one before (user 2026-11-27:
  30 deg/s), so a far target (LeRobot's 1 s return to the start pose, a replay's first action, a bad policy output)
  is walked to; the gripper by `set_gripper` when its target moved by `GRIP_DEADBAND` or more, at most every
  `GRIP_MIN_INTERVAL_S`.
- **Keep alive.** SOFT_RTI ends the stream after 3 command intervals without a command (RTI_LINK_LOST, the arm
  disabled). LeRobot's loops pause longer (saving an episode's video, a slow policy step), so a thread sends the
  target again (still capped, still walking towards the last action) whenever no action came for
  `KEEPALIVE_PERIODS` intervals.
- **Close.** The stream ended with `rti_disconnect` (the arm comes to rest and holds in IDLE, ENABLED: never
  disabled, J2 / J3 would sink), the recipe back, the connections closed. Never raises.
- **Clock.** Every time it measures is ``time.perf_counter()``, and its threads wait with ``time.sleep``: on Windows
  with Python <= 3.12 ``time.monotonic()`` advances only every 15.6 ms (and an ``Event.wait`` of 1 ms takes about as
  long). The cap's steps were sized with it on the first arm run (2026-11-27): 31.25 or 46.9 ms of travel for each
  33.3 ms, which the robot, reading the real-time client's own send times (perf_counter), played as a speed jumping
  by half four times a second (the arm jittered; COMMAND_JUMP warnings in pairs).

The library stays quiet (robotclient's rule): problems go to this module's logger, failures are raised.
"""
from __future__ import annotations

import collections
import logging
import math
import threading
import time

import numpy as np

from robotclient import CameraClient, RobotClient, RTIClient
from robotclient.constants import DEFAULT_PORT
from robotclient.recording import export as X
from robotclient.telemetry import decode_packet

log = logging.getLogger(__name__)

RECIPE = "record"              # the recorder's recipe: j_rx / g_rx, each joint's reply time
JOINTS = 6
NAMES = X.NAMES                # j1.pos .. j6.pos, gripper.pos: the export's names (one policy runs on either)
CAMERA = X.CAMERA              # "wrist": the dataset's observation.images.wrist
BUFFER_PACKETS = 250           # the state packets kept: 1 s at 250 Hz (an observation needs the last ~20 ms)
STATE_WAIT_S = 0.010           # how long an observation waits for the packet after its frame's moment
KEEPALIVE_PERIODS = 1.5        # no action for this many intervals: the keep-alive sends the target again
CAP_DT_PERIODS = 2.0           # a cap step never counts more time than this many intervals (no jump after a pause)
GRIP_DEADBAND = 2.0 / 255.0    # a gripper target that moved less than this is not sent
GRIP_MIN_INTERVAL_S = 0.1      # at most 10 gripper targets a second
STATUS_WAIT_S = 3.0            # how long connect() waits for the robot's first status
POLL_S = 0.001                 # the packet reader's and the waits' poll


def hard_errors(status: dict) -> list:
    """The status's errors that are not warnings (warnings share the list: `error_details` tells them apart; a status
    without details: every key counts)."""
    details = status.get("error_details")
    if isinstance(details, list):
        return [d.get("key") for d in details if isinstance(d, dict) and d.get("level", "error") != "warning"]
    return list(status.get("errors") or [])


class Link:
    """One PAR6 arm, live, for LeRobot: connect, observe, act, hold, close. See the module's docstring.

    Args:
        host: The robot's address.
        port: The robot's command port.
        fps: The loop's rate (LeRobot's fps, the dataset's): the SOFT_RTI rate.
        camera: Whether observations carry the wrist camera's image.
        width, height: The images' size (the dataset's).
        max_speed_deg_s: The cap: how far a joint target may move per second; 0 = no cap.
        gripper_speed, gripper_current: `set_gripper`'s speed and current (the recorder's key uses the same).
        transport: The real-time link: "udp" or "tcp".
        make_robot, make_rti, make_camera: The clients' classes (the tests give fakes).
    """

    def __init__(self, host: str, port: int = DEFAULT_PORT, fps: int = 30, camera: bool = True,
                 width: int = X.WIDTH, height: int = X.HEIGHT, max_speed_deg_s: float = 30.0,
                 gripper_speed: int = 100, gripper_current: int = 200, transport: str = "udp", *,
                 make_robot=RobotClient, make_rti=RTIClient, make_camera=CameraClient) -> None:
        if not fps or fps <= 0:
            raise ValueError(f"fps must be above 0, not {fps!r}")
        if max_speed_deg_s is None or not math.isfinite(max_speed_deg_s) or max_speed_deg_s < 0:
            raise ValueError(f"max_speed_deg_s must be 0 (no cap) or above, not {max_speed_deg_s!r}")
        self.host, self.port, self.fps, self.camera = host, port, int(fps), bool(camera)
        self.width, self.height = int(width), int(height)
        self.period = 1.0 / self.fps
        self.max_speed = math.radians(max_speed_deg_s)        # rad/s; 0 = no cap
        self.gripper_speed, self.gripper_current = int(gripper_speed), int(gripper_current)
        self.transport = transport
        self._make_robot, self._make_rti, self._make_camera = make_robot, make_rti, make_camera
        self.robot = None
        self.rti = None
        self.cam = None
        self.homed = False                 # the arm's homed flag, as last seen
        self.old_recipe: str | None = None
        self.counts = {"sent": 0, "keepalive": 0, "capped": 0, "gripper": 0, "late_state": 0, "repeated_frame": 0}
        self._lock = threading.Lock()      # the real-time client and the targets: LeRobot's thread and ours
        self._grip_lock = threading.Lock()  # the gripper's target: act() and the keep-alive
        self._stop = threading.Event()
        self._threads: list = []
        self._packets: collections.deque = collections.deque(maxlen=BUFFER_PACKETS)
        self._failed: BaseException | None = None
        self._sent = np.zeros(JOINTS)      # the last joint target sent (rad)
        self._desired = np.zeros(JOINTS)   # the last action's joints: the sent target walks towards it
        self._last_send = 0.0              # time.perf_counter() of the last target sent
        self._streaming = False            # the stream was asked for (rti.connect): close() ends it
        self._last_act = 0.0               # ... of the last act()
        self._grip_sent = 0.0              # the gripper target in force (0..1): the measured one until one is sent
        self._grip_desired: float | None = None
        self._grip_last = -math.inf
        self._last_frame = None

    # ------------------------------------------------------------------ connect / close
    @property
    def connected(self) -> bool:
        """Whether the link runs: connected, its threads alive, nothing failed."""
        return self.rti is not None and not self._stop.is_set() and self._failed is None

    def connect(self) -> None:
        """Connect to the arm and start streaming its measured pose (see the module's docstring).

        Raises:
            RuntimeError: the arm is not homed, has errors, or sends no state; its message says what to do.
            robotclient's errors: connecting, enabling, pairing (CommandRejected names why).
        """
        try:
            self._connect()
        except BaseException:
            self.close()
            raise

    def _connect(self) -> None:
        self._stop.clear()
        self._failed = None
        self._packets.clear()
        self._last_send = 0.0
        rc = self._make_robot(name="lerobot")
        self.robot = rc
        rc.connect(self.host, self.port)
        st: dict = {}
        end = time.perf_counter() + STATUS_WAIT_S
        while time.perf_counter() < end and "state" not in st:
            st = rc.latest_status() or {}
            if "state" not in st:
                time.sleep(0.05)
        if "state" not in st:
            raise RuntimeError(f"the arm at {self.host} sent no status within {STATUS_WAIT_S:.0f} s")
        self.homed = bool(st.get("homed"))
        if not self.homed:
            raise RuntimeError(f"the arm at {self.host} is not homed: home it from the dashboard, then connect again "
                               "(this plugin never homes the arm)")
        errors = hard_errors(st)
        if errors:
            raise RuntimeError(f"the arm at {self.host} has errors ({', '.join(map(str, errors))}): find out why and "
                               "clear them from the dashboard, then connect again")
        # What can fail without touching the arm comes first: the PC's UDP port (the real-time client binds it when
        # made and sends nothing) and the camera. The first arm run (2026-11-27) found a taken port only after the
        # arm had been enabled.
        try:
            rti = self._make_rti(rc, clock="pc", transport=self.transport)
        except OSError as e:
            raise OSError(f"{e}. On Windows, `netstat -ano -p udp | findstr :30002` gives the PID of the program "
                          "holding it (`tasklist /fi \"PID eq <pid>\"` its name); or use --robot.transport=tcp") from e
        self.rti = rti
        if self.camera:
            cam = self._make_camera()
            self.cam = cam
            cam.connect(self.host, latest_only=True)
            self._last_frame = cam.read(timeout=5.0)          # the camera sends: RobotTimeout otherwise
        if st.get("state") != "ENABLED":
            log.info("enabling the arm at %s (it holds where it is)", self.host)
            rc.enable(wait=True)
        old = st.get("recipe")
        if old != RECIPE:
            rc.set_recipe(RECIPE)
            self.old_recipe = old
        rti.capture(True)
        self._streaming = True                                # from here on, close() ends the stream
        rti.connect(rate_hz=self.fps)
        rc.wait_mode("SOFT_RTI")
        try:
            first = rti.read_once(timeout=2.0)
        except RuntimeError as e:
            raise RuntimeError(f"no state packet from the arm within 2 s ({e}). Over UDP the arm sends to this PC's "
                               "port 30002: on Windows allow Python through the firewall, on private AND public "
                               "networks (a direct cable is often a public one); or use --robot.transport=tcp") from e
        q = first.get("j_p")
        if not isinstance(q, (list, tuple)) or len(q) < JOINTS:
            raise RuntimeError("the arm's state packets carry no joint positions (j_p)")
        self._sent[:] = np.asarray(q[:JOINTS], dtype=float)
        self._desired[:] = self._sent
        gp = first.get("g_p")
        self._grip_sent = float(gp) / X.GRIP_SCALE if isinstance(gp, (int, float)) and gp == gp else 0.0
        self._grip_desired = None
        rti.claim()
        with self._lock:
            self._send_locked(time.perf_counter(), keepalive=False)
        self._last_act = time.perf_counter()
        for fn, name in ((self._reader, "par6 packets"), (self._keepalive, "par6 keep-alive")):
            t = threading.Thread(target=fn, name=name, daemon=True)
            t.start()
            self._threads.append(t)

    def close(self) -> None:
        """End the stream (the arm comes to rest and holds, ENABLED), put the recipe back, close everything.
        Safe to call twice, and after a failed connect; never raises (problems are logged)."""
        self._stop.set()
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads = []
        rc, rti, cam = self.robot, self.rti, self.cam
        self.robot = self.rti = self.cam = None
        streaming, self._streaming = self._streaming, False
        steps = []
        if rc is not None and rti is not None and streaming:
            steps.append(("end the stream", lambda: rc.rti_disconnect(wait=True)))
        if rti is not None:
            steps.append(("close the real-time link", rti.close))
        if cam is not None:
            steps.append(("close the camera", cam.close))
        if rc is not None and self.old_recipe is not None:
            old = self.old_recipe
            steps.append((f"put the recipe {old!r} back", lambda: rc.set_recipe(old)))
        if rc is not None:
            steps.append(("disconnect", rc.disconnect))
        for what, fn in steps:
            try:
                fn()
            except Exception as e:  # noqa: BLE001  close() must finish: each step on its own
                log.warning("could not %s: %s", what, e)
        self.old_recipe = None

    # ------------------------------------------------------------------ the threads
    def _reader(self) -> None:
        """Collect every state packet the real-time link reads into the buffer (newest last)."""
        try:
            while not self._stop.is_set():
                with self._lock:
                    self.rti.read_state()
                    raw, self.rti.captured = self.rti.captured, []
                for _arrival, data in raw:
                    try:
                        self._packets.append(decode_packet(data))
                    except ValueError as e:
                        log.warning("a state packet could not be read: %s", e)
                time.sleep(POLL_S)                  # not Event.wait: ~15.6 ms on Windows (see Clock above)
        except Exception as e:  # noqa: BLE001  ControlLost / OSError: the link is gone; observe / act raise it
            self._fail(e)

    def _keepalive(self) -> None:
        """Send the target again whenever LeRobot sent no action for KEEPALIVE_PERIODS intervals; flush a gripper
        target the rate limit held back."""
        try:
            while True:
                time.sleep(self.period / 2.0)       # not Event.wait (see Clock above)
                if self._stop.is_set():
                    return
                now = time.perf_counter()
                with self._lock:
                    if now - self._last_send >= KEEPALIVE_PERIODS * self.period:
                        self._send_locked(now, keepalive=True)
                self._gripper(now)
        except Exception as e:  # noqa: BLE001
            self._fail(e)

    def _fail(self, e: BaseException) -> None:
        if self._failed is None:
            self._failed = e
            log.error("the PAR6 link failed: %s", e)

    # ------------------------------------------------------------------ act
    def _send_locked(self, now: float, keepalive: bool) -> None:
        """Move the sent target towards the desired one (capped) and send it. Holds self._lock."""
        step = self._desired - self._sent
        if self.max_speed > 0.0:
            dt = min(max(now - self._last_send, 0.0), CAP_DT_PERIODS * self.period) if self._last_send else self.period
            limit = self.max_speed * dt
            if np.any(np.abs(step) > limit):
                self.counts["capped"] += 1
            step = np.clip(step, -limit, limit)
        self._sent = self._sent + step
        self.rti.send_joint_position(self._sent.tolist())
        self._last_send = now
        self.counts["keepalive" if keepalive else "sent"] += 1

    def _gripper(self, now: float) -> None:
        """Send the desired gripper target when it moved enough and the rate limit allows."""
        with self._grip_lock:
            g = self._grip_desired
            if g is None or abs(g - self._grip_sent) < GRIP_DEADBAND or now - self._grip_last < GRIP_MIN_INTERVAL_S:
                return
            self.robot.set_gripper(int(round(g * X.GRIP_SCALE)), speed=self.gripper_speed,
                                   current=self.gripper_current)
            self._grip_sent = g
            self._grip_last = now
            self.counts["gripper"] += 1

    def _check(self) -> None:
        if self._failed is not None:
            raise RuntimeError(f"the PAR6 link is lost: {self._failed}")
        if self.rti is None:
            raise RuntimeError("the PAR6 link is not connected")
        st = self.robot.latest_status() or {}
        mode, errors = st.get("mode"), hard_errors(st)
        if errors or (mode is not None and mode != "SOFT_RTI"):
            raise RuntimeError(f"the arm left the stream (mode {mode}, errors: {', '.join(map(str, errors)) or 'none'})")

    def act(self, action: dict) -> dict:
        """Send an action ({name: value} for NAMES; the gripper may be left out) and return what is in force: the
        joints as capped, the gripper's target.

        Raises:
            ValueError: a joint is missing, or a value is not a finite number (nothing is sent).
            RuntimeError: the link is lost or the arm left the stream.
        """
        self._check()
        try:
            q = np.array([float(action[n]) for n in NAMES[:JOINTS]])
        except KeyError as e:
            raise ValueError(f"the action has no {e.args[0]!r}: it needs {', '.join(NAMES[:JOINTS])}") from None
        g = action.get(NAMES[JOINTS])
        g = None if g is None else float(g)
        if not np.isfinite(q).all() or (g is not None and not math.isfinite(g)):
            raise ValueError(f"the action has a value that is not a finite number: {dict(action)}")
        now = time.perf_counter()
        with self._lock:
            self._desired[:] = q
            self._send_locked(now, keepalive=False)
            sent = self._sent.copy()
        self._last_act = now
        if g is not None:
            with self._grip_lock:
                self._grip_desired = min(max(g, 0.0), 1.0)
            self._gripper(now)
        return {**{n: float(v) for n, v in zip(NAMES, sent)}, NAMES[JOINTS]: float(self._grip_sent)}

    def hold(self) -> None:
        """Stop where the targets are now: the desired pose becomes the sent one, the gripper's its target in force."""
        with self._lock:
            self._desired[:] = self._sent
        with self._grip_lock:
            self._grip_desired = self._grip_sent

    # ------------------------------------------------------------------ observe
    def _frame(self):
        """The newest frame, waiting up to 1.5 intervals for one newer than the last returned."""
        last = self._last_frame
        end = time.perf_counter() + 1.5 * self.period
        while True:
            f = self.cam.latest()
            if f is not None and (last is None or f.frame != last.frame):
                return f
            if time.perf_counter() >= end:
                self.counts["repeated_frame"] += 1
                return f if f is not None else last
            time.sleep(POLL_S)

    def _state(self, t: float | None):
        """The 7 state values at the robot-clock moment t (None: the newest packet's)."""
        end = time.perf_counter() + STATE_WAIT_S
        while True:
            rows = list(self._packets)
            if rows and t is None:
                break
            if rows:
                state, _ = X.state_at([t], X.columns(rows))
                if np.isfinite(state[0]).all():
                    return state[0]
            if time.perf_counter() >= end:
                break
            time.sleep(POLL_S)
        if not rows:
            raise RuntimeError("no state packet from the arm")
        if t is not None:                                 # the packets do not reach the frame's moment: the newest
            self.counts["late_state"] += 1
        newest = X.columns(rows[-1:])
        return np.concatenate([newest["j_p"][0], newest["g_p"][0] / X.GRIP_SCALE])

    def observe(self) -> dict:
        """The observation: {j1.pos .. j6.pos (rad), gripper.pos (0..1), and with the camera "wrist": RGB uint8
        height x width x 3}, the state at the image's moment.

        Raises:
            RuntimeError: the link is lost, the arm left the stream, or no packet / frame arrived.
        """
        self._check()
        picture = None
        t = None
        if self.camera:
            f = self._frame()
            if f is None:
                raise RuntimeError("no camera frame from the arm")
            self._last_frame = f
            t = f.exposure_time
            picture = X.image(f.jpeg, self.width, self.height)
        state = self._state(t)
        obs: dict = {n: float(v) for n, v in zip(NAMES, state)}
        if picture is not None:
            obs[CAMERA] = picture
        return obs
