"""The wrist camera's frames over the network, each with its time and RTI tick.

Standard library only, like the rest of the client, so a PC needs nothing else. The robot's camera server
(camera_server.py) sends every frame after `GET /frames` on its HTTP port (30050): an HTTP response header, then per
frame a fixed little-endian header (camera protocol 1, communication/camera_protocol.py — this module keeps its own
copy, and dummy_test_scripts/test_camera_client.py keeps the two equal) and the camera's JPEG bytes. Decoding the JPEG
is left to the caller (OpenCV, Pillow), so the package imports no third-party module.
"""
from __future__ import annotations

import socket
import struct
import threading
import time
from typing import Iterator, NamedTuple

from .constants import CAMERA_PORT
from .exceptions import RobotConnectionError, RobotTimeout, VersionMismatch
from .version import CAMERA_PROTOCOL_VERSION

_HEADER = struct.Struct("<4sHHIIHHIddIIdI")
_MAGIC = b"RCBF"
_CONTENT_TYPE = "application/x-rcb-camera-frames"
_FLAG_CAMERA_CLOCK, _FLAG_CALIBRATED, _FLAG_AUTO_EXPOSURE, _FLAG_TICK = 0x01, 0x02, 0x04, 0x08
_MAX_JPEG = 16 << 20              # a length above this is a broken stream, not a frame
_POLL_S = 0.5                     # socket timeout inside a read: a stop or a deadline is seen this quickly


class CameraFrame(NamedTuple):
    """One image from the wrist camera, with the moment it was taken.

    `rti_tick` pairs the image with the robot's joint data: the real-time
    client's state packets carry the same tick (`state["tick"]`), so no clock
    needs to be shared between the robot and the PC.

    Attributes:
        jpeg: The image as the camera sent it, as JPEG bytes. Decode it with
            OpenCV or Pillow.
        exposure_time: When the image was taken, in seconds of the robot's
            monotonic clock.
        rti_tick: The robot's control-loop tick at `exposure_time`, with a
            fraction (one tick is 4 ms); `None` when the robot could not tell.
        camera_clock: Whether `exposure_time` comes from the camera's own clock,
            to a few microseconds, rather than the driver's timestamp, which
            comes in 4 ms steps and 1 to 5 ms late.
        calibrated: Whether `exposure_time` includes the calibrated offset to
            the middle of the exposure.
        auto_exposure: Whether the camera chose its exposure itself.
        exposure_us: The exposure length in microseconds; 0 when the camera
            chooses it (automatic exposure).
        frame: The robot's frame counter.
        driver_seq: The camera driver's frame number.
        width: The image width in pixels.
        height: The image height in pixels.
        pts: The camera's own timestamp of the frame, raw; 0 when not read.
        send_time: When the robot sent the frame, in seconds of its monotonic
            clock.
        received: When this PC had received the whole frame, in seconds of
            this PC's `time.monotonic()`.
        skipped: Frames the robot left out for this client just before this
            one, because the previous frame was still on its way.
        lost: Frames the camera driver lost just before this one.

    Examples:
        >>> from robotclient import CameraClient
        >>> cam = CameraClient()
        >>> cam.connect("192.168.0.84")
        >>> frame = cam.read()
        >>> frame.rti_tick, frame.width, frame.height
        >>> cam.close()
    """

    jpeg: bytes
    exposure_time: float
    rti_tick: float | None
    camera_clock: bool
    calibrated: bool
    auto_exposure: bool
    exposure_us: int
    frame: int
    driver_seq: int
    width: int
    height: int
    pts: int
    send_time: float
    received: float
    skipped: int
    lost: int


class CameraClient:
    """The wrist camera's frames from one robot, each with its time and tick.

    It receives the frames the robot's camera server sends, in one of two
    ways. By default every frame arrives in order, for recording: when this
    PC reads more slowly than the camera sends, the robot leaves frames out
    (`skipped`) instead of building a queue. With `latest_only=True` a
    background thread receives all the time and keeps only the newest frame,
    for a controller that always wants the current image.

    The camera runs whenever the robot service runs and a camera is plugged
    in; the dashboard's Camera tab shows its state and settings. Frames are
    JPEG images and are never decoded here.

    Attributes:
        host: The robot's address, set by `connect()`; `None` before.
        port: The camera server's port, set by `connect()`.
        latest_only: Whether only the newest frame is kept, as given to
            `connect()`.
        frames_received: Frames received since `connect()`.
        skipped: Frames the robot left out for this client since `connect()`.
        lost: Frames the camera driver lost since `connect()`.

    Examples:
        >>> from robotclient import CameraClient
        >>> cam = CameraClient()
        >>> cam.connect("192.168.0.84")
        >>> for _ in range(30):
        ...     frame = cam.read()
        ...     print(frame.rti_tick, len(frame.jpeg))
        ...
        >>> cam.close()
    """

    def __init__(self) -> None:
        """Create a client; nothing is sent until `connect()`.

        Examples:
            >>> from robotclient import CameraClient
            >>> cam = CameraClient()
        """
        self.host: str | None = None
        self.port = CAMERA_PORT
        self.latest_only = False
        self.frames_received = 0
        self.skipped = 0
        self.lost = 0
        self._sock: socket.socket | None = None
        self._buf = bytearray()
        self._prev: tuple[int, int] | None = None     # (frame, driver_seq) of the previous frame
        self._cond = threading.Condition()
        self._newest: CameraFrame | None = None
        self._newest_n = 0                             # frames_received when _newest arrived
        self._returned_n = 0                           # ... of the frame read() last returned
        self._error: Exception | None = None
        self._stop = False
        self._thread: threading.Thread | None = None

    @property
    def connected(self) -> bool:
        """Whether the stream from the robot is open.

        Examples:
            >>> from robotclient import CameraClient
            >>> cam = CameraClient().connect("192.168.0.84")
            >>> cam.connected
            True
            >>> cam.close()
        """
        return self._sock is not None and self._error is None

    def connect(self, host: str, port: int = CAMERA_PORT, *, latest_only: bool = False,
                timeout: float = 5.0, allow_mismatch: bool = False) -> CameraClient:
        """Connect to the robot's camera and start receiving frames.

        It asks the camera server for its timed frames and checks that the robot
        speaks this client's camera protocol. Frames arrive whether or not they
        are read: in the default mode the robot leaves frames out while this PC
        is behind, with `latest_only=True` a background thread keeps the newest.

        Args:
            host: The robot's address, as for `RobotClient.connect()`.
            port: The camera server's port. Keep the default unless the robot's
                configuration changed it.
            latest_only: Keep only the newest frame: `read()` then returns the
                newest frame not returned before, and `latest()` works.
            timeout: Seconds to wait for the connection and the server's answer.
            allow_mismatch: Connect even when the robot reports another camera
                protocol version. The frames are then read with this client's
                layout, which may be wrong.

        Returns:
            This `CameraClient`, so that calls can be chained.

        Raises:
            RobotConnectionError: The camera server cannot be reached, did not
                answer within `timeout`, or answered with something other than
                camera frames, for example because the camera is switched off.
            VersionMismatch: The robot speaks another camera protocol version,
                and `allow_mismatch` is false.
            RuntimeError: This client is connected already; call `close()`
                first.

        Examples:
            >>> from robotclient import CameraClient
            >>> cam = CameraClient().connect("192.168.0.84", latest_only=True)
            >>> cam.close()
        """
        if self._sock is not None:
            raise RuntimeError("the camera client is connected already — close() it first")
        try:
            sock = socket.create_connection((host, port), timeout=timeout)
        except OSError as e:
            raise RobotConnectionError(f"the robot's camera server at {host}:{port} cannot be reached: {e}") from e
        try:
            sock.sendall(b"GET /frames HTTP/1.0\r\n\r\n")
            head = bytearray()
            deadline = time.monotonic() + timeout
            while b"\r\n\r\n" not in head:
                if time.monotonic() > deadline or len(head) > 8192:
                    raise RobotConnectionError(f"the camera server at {host}:{port} did not answer")
                part = sock.recv(4096)
                if not part:
                    raise RobotConnectionError(f"the camera server at {host}:{port} closed the connection")
                head += part
        except socket.timeout as e:
            sock.close()
            raise RobotConnectionError(f"the camera server at {host}:{port} did not answer within {timeout} s") from e
        except (OSError, RobotConnectionError):
            sock.close()
            raise
        text, _, rest = bytes(head).partition(b"\r\n\r\n")
        lines = text.decode("latin-1").split("\r\n")
        fields = {k.strip().lower(): v.strip() for k, _, v in (ln.partition(":") for ln in lines[1:])}
        ctype = fields.get("content-type", "")
        protocol = None
        for param in ctype.split(";")[1:]:
            key, _, value = param.strip().partition("=")
            if key == "protocol" and value.isdigit():
                protocol = int(value)
        if not lines[0].split(" ")[1:2] == ["200"] or not ctype.startswith(_CONTENT_TYPE) or protocol is None:
            sock.close()
            raise RobotConnectionError(f"the camera server at {host}:{port} answered {lines[0]!r} "
                                       f"({ctype or 'no content type'}), not camera frames")
        if protocol != CAMERA_PROTOCOL_VERSION and not allow_mismatch:
            sock.close()
            raise VersionMismatch(f"the robot speaks camera protocol {protocol}, this client "
                                  f"{CAMERA_PROTOCOL_VERSION}: update the client (or the robot), or connect with "
                                  f"allow_mismatch=True")
        sock.settimeout(_POLL_S)
        self._sock, self.host, self.port, self.latest_only = sock, host, port, bool(latest_only)
        self._buf = bytearray(rest)
        self._prev, self._newest, self._error, self._stop = None, None, None, False
        self.frames_received = self.skipped = self.lost = 0
        self._newest_n = self._returned_n = 0
        if self.latest_only:
            self._thread = threading.Thread(target=self._pump, name="CameraClient", daemon=True)
            self._thread.start()
        return self

    def read(self, timeout: float = 5.0) -> CameraFrame:
        """Return the next frame, waiting for it if needed.

        In the default mode it is the next frame in the order the robot sent
        them. With `latest_only=True` it is the newest frame that no earlier
        call returned, so frames in between are passed over.

        Args:
            timeout: Seconds to wait for a frame.

        Returns:
            The frame.

        Raises:
            RobotTimeout: No frame arrived within `timeout`, for example while
                the camera is unplugged.
            RobotConnectionError: The client is not connected, or the robot
                closed the stream.

        Examples:
            >>> from robotclient import CameraClient
            >>> cam = CameraClient().connect("192.168.0.84")
            >>> frame = cam.read()
            >>> frame.exposure_time, frame.rti_tick
            >>> cam.close()
        """
        if self._sock is None:
            raise RobotConnectionError("the camera client is not connected — call connect() first")
        if not self.latest_only:
            return self._receive(time.monotonic() + timeout)
        with self._cond:
            if not self._cond.wait_for(lambda: self._newest_n > self._returned_n or self._error is not None
                                       or self._sock is None, timeout):
                raise RobotTimeout(f"no camera frame within {timeout} s")
            if self._newest_n > self._returned_n and self._newest is not None:
                self._returned_n = self._newest_n
                return self._newest
            raise self._error or RobotConnectionError("the camera client was closed")

    def latest(self) -> CameraFrame | None:
        """Return the newest frame received, without waiting.

        It can return the same frame twice; compare `frame` to tell. Only with
        `connect(..., latest_only=True)`.

        Returns:
            The newest frame, or `None` when none has arrived yet.

        Raises:
            RobotConnectionError: The client is not connected, or the robot
                closed the stream.
            RuntimeError: The client was connected without `latest_only=True`.

        Examples:
            >>> from robotclient import CameraClient
            >>> cam = CameraClient().connect("192.168.0.84", latest_only=True)
            >>> frame = cam.latest()
            >>> cam.close()
        """
        if self._sock is None:
            raise RobotConnectionError("the camera client is not connected — call connect() first")
        if not self.latest_only:
            raise RuntimeError("latest() needs connect(..., latest_only=True)")
        with self._cond:
            if self._error is not None:
                raise self._error
            if self._newest is not None:
                self._returned_n = self._newest_n
            return self._newest

    def frames(self, timeout: float = 5.0) -> Iterator[CameraFrame]:
        """Yield frames one after another until the stream ends.

        Each frame comes from `read()`, so the two modes behave as there. The
        loop ends when the client is closed.

        Args:
            timeout: Seconds to wait for each frame.

        Yields:
            The frames.

        Raises:
            RobotTimeout: No frame arrived within `timeout`.
            RobotConnectionError: The robot closed the stream.

        Examples:
            >>> from robotclient import CameraClient
            >>> cam = CameraClient().connect("192.168.0.84")
            >>> for frame in cam.frames():
            ...     if frame.frame >= 100:
            ...         break
            ...
            >>> cam.close()
        """
        while self._sock is not None:
            try:
                yield self.read(timeout)
            except RobotConnectionError:
                if self._sock is None:              # closed while waiting: the loop just ends
                    return
                raise

    def close(self) -> None:
        """Stop receiving and close the connection.

        Examples:
            >>> from robotclient import CameraClient
            >>> cam = CameraClient().connect("192.168.0.84")
            >>> cam.close()
        """
        sock, self._sock = self._sock, None
        self._stop = True
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        thread, self._thread = self._thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._cond:
            self._cond.notify_all()

    def __enter__(self) -> CameraClient:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- receiving

    def _pump(self) -> None:
        """latest_only: receive all the time, keep the newest frame."""
        while not self._stop:
            try:
                frame = self._receive(None)
            except Exception as e:  # noqa: BLE001 — handed to the reader
                with self._cond:
                    if not self._stop:
                        self._error = e if isinstance(e, RobotConnectionError) else RobotConnectionError(str(e))
                    self._cond.notify_all()
                return
            with self._cond:
                self._newest, self._newest_n = frame, self.frames_received
                self._cond.notify_all()

    def _fill(self, n: int, deadline: float | None) -> None:
        sock = self._sock
        while len(self._buf) < n:
            if self._stop or sock is None:
                raise RobotConnectionError("the camera client was closed")
            if deadline is not None and time.monotonic() > deadline:
                raise RobotTimeout(f"no camera frame within the timeout ({len(self._buf)} of {n} bytes)")
            try:
                part = sock.recv(max(65536, n - len(self._buf)))
            except socket.timeout:
                continue
            except OSError as e:
                raise RobotConnectionError(f"the camera stream broke: {e}") from e
            if not part:
                raise RobotConnectionError("the robot's camera server closed the stream")
            self._buf += part

    def _receive(self, deadline: float | None) -> CameraFrame:
        self._fill(_HEADER.size, deadline)
        (magic, _version, hlen, frame, seq, width, height, flags, t_exp, tick, exp_us, pts, t_send,
         length) = _HEADER.unpack_from(self._buf)
        if magic != _MAGIC or hlen < _HEADER.size or length > _MAX_JPEG:
            raise RobotConnectionError(f"the camera stream is broken (magic {magic!r}, header {hlen} bytes, "
                                       f"JPEG {length} bytes)")
        self._fill(hlen + length, deadline)
        jpeg = bytes(self._buf[hlen:hlen + length])
        del self._buf[:hlen + length]
        skipped = lost = 0
        if self._prev is not None:
            skipped = max(0, frame - self._prev[0] - 1)
            lost = max(0, seq - self._prev[1] - 1)
        self._prev = (frame, seq)
        self.frames_received += 1
        self.skipped += skipped
        self.lost += lost
        return CameraFrame(jpeg=jpeg, exposure_time=t_exp,
                           rti_tick=tick if flags & _FLAG_TICK and tick == tick else None,
                           camera_clock=bool(flags & _FLAG_CAMERA_CLOCK), calibrated=bool(flags & _FLAG_CALIBRATED),
                           auto_exposure=bool(flags & _FLAG_AUTO_EXPOSURE), exposure_us=exp_us, frame=frame,
                           driver_seq=seq, width=width, height=height, pts=pts, send_time=t_send,
                           received=time.monotonic(), skipped=skipped, lost=lost)
