"""The two message types the recorder writes for viewers, as ROS 2 messages (CDR), encoded by hand.

MCAP viewers (Foxglove, Lichtblick, PlotJuggler) show a channel whose schema is a ROS 2 message definition (schema
encoding ``ros2msg``) and whose messages are CDR (message encoding ``cdr``), with no ROS installed anywhere. The
recorder writes two such types:

* ``sensor_msgs/msg/CompressedImage``: a camera frame, its JPEG carried untouched, so a recording stores each image
  once and a viewer shows it;
* ``rcb_msgs/msg/State``: one robot packet, one field per tag (a number, a fixed array of numbers or a text), so a
  viewer plots any value. Its definition is made from the packet's own tags (`state_layout`, `state_definition`).

CDR here is the plain little-endian encoding ROS 2 uses (XCDR1): a 4-byte header ``00 01 00 00``, then each field at
an offset that is a multiple of its own size, counted from the end of that header (zero bytes fill the gaps). A
``string`` is a uint32 length that counts a closing NUL, the UTF-8 bytes and the NUL; a ``uint8[]`` is a uint32 count
and the bytes; a fixed ``float64[n]`` is the n values with no count.

Standard library only, like the rest of robotclient.
"""
from __future__ import annotations

import re
import struct

ENCAPSULATION = b"\x00\x01\x00\x00"        # CDR, little-endian

IMAGE_SCHEMA_NAME = "sensor_msgs/msg/CompressedImage"
IMAGE_DEFINITION = """std_msgs/Header header
string format
uint8[] data
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

STATE_SCHEMA_NAME = "rcb_msgs/msg/State"

_U32 = struct.Struct("<I")
_STAMP = struct.Struct("<iI")


def _string(text: str, offset: int) -> tuple:
    """A CDR string written at `offset` (counted from the end of the 4-byte header) -> (its bytes with the padding
    before it, the offset after it)."""
    raw = text.encode("utf-8") + b"\0"
    pad = -offset % 4
    return b"\0" * pad + _U32.pack(len(raw)) + raw, offset + pad + 4 + len(raw)


def encode_image(stamp_ns: int, frame_id: str, image_format: str, data: bytes) -> bytes:
    """One ``sensor_msgs/msg/CompressedImage`` as CDR bytes.

    Args:
        stamp_ns: The image's moment, nanoseconds since 1970 (the header's stamp).
        frame_id: The camera's name, e.g. ``"arm1/wrist"``.
        image_format: ``"jpeg"`` for the camera's frames.
        data: The image bytes, carried unchanged.

    Raises:
        struct.error: `stamp_ns` is negative or past the year 2038 (the stamp's seconds are an int32).
    """
    sec, nanosec = divmod(int(stamp_ns), 1_000_000_000)
    parts = [ENCAPSULATION, _STAMP.pack(sec, nanosec)]
    offset = _STAMP.size
    for text in (frame_id, image_format):
        raw, offset = _string(text, offset)
        parts.append(raw)
    pad = -offset % 4
    parts.append(b"\0" * pad + _U32.pack(len(data)))
    parts.append(bytes(data))
    return b"".join(parts)


def decode_image(data: bytes) -> tuple:
    """A ``sensor_msgs/msg/CompressedImage`` from its CDR bytes -> (stamp_ns, frame_id, image_format, data): the
    reverse of `encode_image` (the exporter takes the JPEG back out of a recording).

    Raises:
        ValueError: not a little-endian CDR message, or cut short / malformed.
    """
    if bytes(data[:4]) != ENCAPSULATION:
        raise ValueError("not a little-endian CDR message")
    body = bytes(data[4:])

    def take(offset: int, align: int = 4) -> tuple:
        offset += -offset % align
        if offset + 4 > len(body):
            raise ValueError("a CompressedImage cut short")
        n = _U32.unpack_from(body, offset)[0]
        if offset + 4 + n > len(body):
            raise ValueError("a CompressedImage cut short")
        return body[offset + 4:offset + 4 + n], offset + 4 + n

    if len(body) < _STAMP.size:
        raise ValueError("a CompressedImage cut short")
    sec, nanosec = _STAMP.unpack_from(body, 0)
    texts = []
    offset = _STAMP.size
    for _ in range(2):
        raw, offset = take(offset)
        if not raw.endswith(b"\0"):
            raise ValueError("a CompressedImage string without its closing NUL")
        try:
            texts.append(raw[:-1].decode("utf-8"))
        except UnicodeDecodeError as e:
            raise ValueError(f"a CompressedImage string not UTF-8: {e}") from None
    image, _ = take(offset)
    return sec * 1_000_000_000 + nanosec, texts[0], texts[1], image


_NOT_NAME = re.compile(r"[^a-z0-9]+")


def field_names(tags) -> list:
    """Each tag as a ROS 2 field name: lower case, a letter first, single underscores, none at the end; unique (a
    clash gets ``_2``, ``_3``...). Most tags already are such names and stay as they are; ``TCP_p`` becomes
    ``tcp_p`` (the definition notes the tag beside each renamed field)."""
    out: list = []
    seen: set = set()
    for tag in tags:
        name = _NOT_NAME.sub("_", str(tag).lower()).strip("_") or "field"
        if not name[0].isalpha():
            name = "f_" + name
        base, k = name, 2
        while name in seen:
            name = f"{base}_{k}"
            k += 1
        seen.add(name)
        out.append(name)
    return out


def state_layout(decoded: dict) -> tuple:
    """A decoded packet's fields as ((tag, kind, n), ...): kind ``"f"`` one number, ``"a"`` a list of n numbers,
    ``"s"`` a text (``mode``). Two packets with the same layout share one message definition."""
    out = []
    for tag, value in decoded.items():
        if isinstance(value, str):
            out.append((tag, "s", 1))
        elif isinstance(value, (list, tuple)):
            out.append((tag, "a", len(value)))
        else:
            out.append((tag, "f", 1))
    return tuple(out)


def state_definition(layout: tuple) -> str:
    """The ``rcb_msgs/msg/State`` definition (ROS 2 .msg text) for a layout: one field per tag, in the packet's order:
    ``float64``, ``float64[n]`` or ``string``; a field renamed by `field_names` notes its tag."""
    lines = ["# One robot packet, one field per tag (robotclient.telemetry.decode_packet)."]
    for name, (tag, kind, n) in zip(field_names(t for t, _, _ in layout), layout):
        typ = "string" if kind == "s" else "float64" if kind == "f" else f"float64[{n}]"
        lines.append(f"{typ} {name}" + (f"  # tag {tag}" if name != tag else ""))
    return "\n".join(lines) + "\n"


def encode_state(layout: tuple, decoded: dict) -> bytes:
    """One packet (decoded) as ``rcb_msgs/msg/State`` CDR bytes, for the definition of `layout`.

    Raises:
        KeyError: the packet lacks a tag of `layout`.
        struct.error: a value does not fit its field (a text where a number belongs, a list of another length).
        TypeError: as struct.error, for a value of the wrong kind.
    """
    parts = [ENCAPSULATION]
    offset = 0
    for tag, kind, n in layout:
        value = decoded[tag]
        if kind == "s":
            raw, offset = _string(str(value), offset)
            parts.append(raw)
            continue
        pad = -offset % 8
        if kind == "f":
            parts.append(b"\0" * pad + struct.pack("<d", value))
        else:
            if len(value) != n:
                raise struct.error(f"tag {tag}: {len(value)} values, the layout has {n}")
            parts.append(b"\0" * pad + struct.pack(f"<{n}d", *value))
        offset += pad + 8 * n
    return b"".join(parts)
