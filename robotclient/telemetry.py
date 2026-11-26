"""Reading the robot's state and telemetry packets, in either of their two formats.

The robot sends one packet per control tick (250 a second on PAR6): the values its active recipe names (`set_recipe`).
The same packet goes to the PC paired in RTI / SOFT_RTI mode and to the telemetry destination. It comes in one of two
formats (`set_telemetry_format`):

* text (format 1): ``j_p:0.1235,1.4500,...,tick:12345,mode:IDLE`` — each value rounded to the recipe's decimals;
  only the first value of a tag carries its name.
* binary (format 2, 2026-11-26): a 16-byte header, a map naming each tag and its number of values, then the values as
  little-endian float64, exact. Every packet carries its own map, so one packet can be read alone::

      offset  size  field
      0       4     magic       b"RCBT"
      4       2     format      2
      6       2     values_at   the byte offset of the first value
      8       4     count       the number of values
      12      4     reserved    0
      16      ...   the map     ASCII "tag:n,tag:n,...", in the values' order; `mode` is "mode:1:IDLE|ENABLED|..."
                                (its value is the index into that list); padded with spaces up to values_at
      values_at     the values  count x float64

`decode_packet()` reads either and returns the same dictionary: each tag maps to a number, or to a list for a tag with
several values (the joints, a pose); `mode` maps to its name. A text packet's numbers are rounded to its recipe's
decimals, a binary packet's are exact; a value the robot does not have is NaN.

`TelemetryClient` receives the packets: over TCP from the robot's port 30004 (every tick while connected, none lost:
what a recorder uses) or over UDP on this PC's port 30003 (what the robot sends to its telemetry destination, which can
lose a packet).
"""
from __future__ import annotations

import math
import socket
import struct
import time

MAGIC = b"RCBT"
FORMAT_TEXT = 1
FORMAT_BINARY = 2
_HEADER = struct.Struct("<4sHHII")
_MAPS: dict = {}            # map bytes -> ([(tag, count, mode names or None)], total), so a map is parsed once
_MAPS_MAX = 64


def is_binary(packet) -> bool:
    """Whether `packet` (bytes) is a binary packet: it starts with the magic, which no text packet does."""
    return bytes(packet[:4]) == MAGIC


def parse_text(text: str) -> dict:
    """Parse a text packet into {tag: value or [values]}.

    Wire format (utility/serialization.py) is a flat comma-separated list where only the FIRST value of a vector
    carries its tag::

        j_p:0.12,1.45,-0.78,0.0,1.23,-0.56,j_s:0.01,-0.02,...,tick:12345

    So a chunk containing ':' opens a new tag and every following bare chunk belongs to it. Splitting on ',' alone
    (and requiring ':' per chunk) silently drops every joint array — that was the 2026-08-01 draft's bug.
    """
    out: dict = {}
    tag = None
    for chunk in text.split(','):
        if not chunk:
            continue
        if ':' in chunk:
            tag, _, first = chunk.partition(':')
            tag = tag.strip()
            out[tag] = [_num(first)]
        elif tag is not None:
            out[tag].append(_num(chunk))
    # Collapse single-element lists to scalars so state["tick"] is a number.
    return {k: (v[0] if len(v) == 1 else v) for k, v in out.items()}


def _num(s):
    s = s.strip()
    try:
        return float(s)
    except ValueError:
        return s


def _parse_map(raw: bytes) -> list:
    entries = []
    for entry in raw.decode("ascii").strip().split(","):
        if not entry:
            continue
        parts = entry.split(":")
        names = parts[2].split("|") if len(parts) > 2 else None
        entries.append((parts[0], int(parts[1]), names))
    return entries


def decode_binary(packet) -> dict:
    """Decode a binary packet (format 2) into {tag: value or [values]}. Raises ValueError for a packet that is not
    one, or of a newer format than this client reads."""
    data = bytes(packet)
    if len(data) < _HEADER.size:
        raise ValueError(f"a binary packet is at least {_HEADER.size} bytes, got {len(data)}")
    magic, fmt, values_at, count, _ = _HEADER.unpack_from(data, 0)
    if magic != MAGIC:
        raise ValueError("not a binary packet (no magic)")
    if fmt != FORMAT_BINARY:
        raise ValueError(f"binary packet format {fmt}: this client reads format {FORMAT_BINARY}")
    if len(data) < values_at + 8 * count:
        raise ValueError(f"a cut binary packet: {len(data)} bytes, its header says {values_at + 8 * count}")
    raw_map = data[_HEADER.size:values_at]
    cached = _MAPS.get(raw_map)
    if cached is None:
        entries = _parse_map(raw_map)
        cached = (entries, sum(n for _, n, _ in entries))
        if len(_MAPS) >= _MAPS_MAX:
            _MAPS.clear()
        _MAPS[raw_map] = cached
    entries, total = cached
    if total != count:                  # checked on every packet, a cached map too
        raise ValueError(f"a binary packet's map names {total} values, its header {count}")
    values = struct.unpack_from(f"<{count}d", data, values_at)
    out: dict = {}
    i = 0
    for tag, n, names in entries:
        part = values[i:i + n]
        i += n
        if names is not None:
            k = part[0]
            out[tag] = names[int(k)] if not math.isnan(k) and 0 <= int(k) < len(names) else ""
        else:
            out[tag] = part[0] if n == 1 else list(part)
    return out


TELEMETRY_TCP_PORT = 30004      # the robot's recorder link
TELEMETRY_UDP_PORT = 30003      # where the robot's UDP telemetry arrives
_MAX_FRAME = 1 << 20            # a packet longer than 1 MB is no packet: the stream is out of step


class TelemetryClient:
    """Receives the robot's telemetry packets, one per control tick.

    `transport="tcp"` connects to the robot's TCP port 30004 and gets every tick's packet, none lost while connected;
    the robot serves one such connection at a time and drops it when this program stops reading for several seconds.
    `transport="udp"` listens on this PC's UDP port 30003 for the packets the robot sends to its telemetry destination
    (`RobotClient.set_telemetry_destination()`), keeping the robot's own; UDP can lose a packet.

    Args:
        robot_ip: The robot's address.
        transport: `"tcp"` (default) or `"udp"`.
        port: The port; default 30004 for TCP, 30003 for UDP.
        timeout: Seconds `connect()` waits for the robot (TCP).

    Raises:
        ValueError: `transport` is not `"tcp"` or `"udp"`.

    Examples:
        >>> with TelemetryClient("192.168.1.50") as tel:
        ...     state = tel.read()
        ...     print(state["tick"], state["j_p"])
    """

    def __init__(self, robot_ip: str, transport: str = "tcp", port: int | None = None, timeout: float = 5.0):
        if transport not in ("tcp", "udp"):
            raise ValueError(f"transport must be 'tcp' or 'udp', got {transport!r}")
        self.robot_ip = robot_ip
        self.transport = transport
        self.port = int(port if port is not None else (TELEMETRY_TCP_PORT if transport == "tcp" else TELEMETRY_UDP_PORT))
        self.timeout = float(timeout)
        self.sock: socket.socket | None = None
        self.packets = 0            # packets received
        self.bytes = 0              # bytes received, the TCP lengths included
        self._buf = bytearray()

    def connect(self) -> "TelemetryClient":
        """Connect (TCP) or start listening (UDP). Returns self.

        Raises:
            OSError: The robot refused or did not answer (TCP), or the UDP port is in use on this PC.
        """
        if self.transport == "tcp":
            sock = socket.create_connection((self.robot_ip, self.port), timeout=self.timeout)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", self.port))
        self.sock = sock
        return self

    def read_raw(self, timeout: float | None = 1.0) -> bytes:
        """The next packet as the robot sent it (what a recorder stores). `timeout`: seconds to wait, None = for ever.

        Raises:
            socket.timeout: No packet within `timeout`.
            ConnectionError: The robot closed the TCP connection, or the stream is out of step.
            RuntimeError: Not connected.
        """
        if self.sock is None:
            raise RuntimeError("not connected: call connect() first")
        end = None if timeout is None else time.monotonic() + timeout
        if self.transport == "udp":
            while True:
                self.sock.settimeout(None if end is None else max(0.0, end - time.monotonic()) or 1e-6)
                data, addr = self.sock.recvfrom(65535)
                if addr[0] == self.robot_ip:
                    self.packets += 1
                    self.bytes += len(data)
                    return data
        while True:
            if len(self._buf) >= 4:
                size = int.from_bytes(self._buf[:4], "big")
                if size > _MAX_FRAME:
                    raise ConnectionError(f"a {size}-byte packet: the stream is out of step")
                if len(self._buf) >= 4 + size:
                    data = bytes(self._buf[4:4 + size])
                    del self._buf[:4 + size]
                    self.packets += 1
                    self.bytes += 4 + size
                    return data
            self.sock.settimeout(None if end is None else max(0.0, end - time.monotonic()) or 1e-6)
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("the robot closed the telemetry connection")
            self._buf.extend(chunk)

    def read(self, timeout: float | None = 1.0) -> dict:
        """The next packet, decoded (`decode_packet()`): {tag: value or [values]}. Raises as `read_raw()`."""
        return decode_packet(self.read_raw(timeout))

    def close(self) -> None:
        """Close the connection or the listening socket. Safe to call twice."""
        if self.sock is not None:
            try:
                self.sock.close()
            finally:
                self.sock = None

    def __enter__(self) -> "TelemetryClient":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.close()


def decode_packet(packet) -> dict:
    """Decode one state / telemetry packet, text (str or bytes) or binary (bytes), into {tag: value or [values]}."""
    if isinstance(packet, str):
        return parse_text(packet)
    if is_binary(packet):
        return decode_binary(packet)
    return parse_text(bytes(packet).decode("utf-8", errors="replace"))
