"""RTI's binary telemetry packet (TELEMETRY_FORMAT 2, 2026-11-26, TODO/p4_recording_plan.md section 2B).

The same recipe as the text packet (format 1), its values copied out of the tick's row of RTI's trace
(utility/trace_ring.py) as float64: exact, and one numpy copy whatever the recipe (~7 us a tick for every value the
robot has; the text of them all costs ~250 us). Every packet carries its own MAP, so a single packet can be read alone,
as the text can (the user's choice, 2026-11-26).

Wire layout, little-endian:

    offset  size  field
    0       4     magic       b"RCBT"
    4       2     format      2
    6       2     values_at   the byte offset of the first value (a multiple of 8)
    8       4     count       the number of float64 values
    12      4     reserved    0
    16      ...   the map     ASCII "tag:n,tag:n,..." — each tag and its number of values, in the values' order;
                              `mode` is "mode:1:IDLE|ENABLED|..." (its value is the index into that list, -1 =
                              unknown); padded with spaces up to values_at
    values_at  count x 8      the values (float64; an integer is exact, a missing value NaN)

A text packet starts with a tag ("j_p:..."), never with the magic, so a reader tells the two apart by the first four
bytes (robotclient/telemetry.py decodes both). `BinaryPacker` builds the header and the map once per recipe; a tick
only fills the values (nothing allocated).
"""
from __future__ import annotations

import struct

import numpy as np

MAGIC = b"RCBT"
FORMAT_BINARY = 2
HEADER = struct.Struct("<4sHHII")


class BinaryPacker:
    """Packs a recipe's tags out of trace rows. `columns`: {tag: (start, size)} (TraceWriter.columns); `mode_names`:
    the trace's mode order (trace_ring.mode_names())."""

    def __init__(self, columns: dict, mode_names):
        self._columns = dict(columns)
        self._modes = tuple(str(m) for m in mode_names)
        self.recipe: object = None  # the recipe object the packet was built for (RTI compares identity)
        self.packet: bytearray | None = None    # sent: header + map once, the values refilled per tick
        self.tags: tuple = ()       # the tags in the packet, in order
        self.skipped: tuple = ()    # recipe tags the trace lacks (none since every tag is a trace channel)
        self._idx: np.ndarray | None = None
        self._vals: np.ndarray | None = None

    def set_fields(self, fields, recipe=None) -> tuple:
        """Build the packet for these tags (a recipe change: allocates). Returns the tags left out (not in the
        trace). An empty recipe gives a packet with no values."""
        tags: list[str] = []
        skipped: list[str] = []
        idx: list[int] = []
        entries: list[str] = []
        for tag in fields:
            col = self._columns.get(tag)
            if col is None:
                skipped.append(tag)
                continue
            start, size = col
            tags.append(tag)
            idx.extend(range(start, start + size))
            entries.append(f"{tag}:{size}:{'|'.join(self._modes)}" if tag == "mode" else f"{tag}:{size}")
        text = ",".join(entries).encode("ascii")
        values_at = HEADER.size + len(text)
        values_at += (-values_at) % 8
        count = len(idx)
        packet = bytearray(values_at + 8 * count)
        HEADER.pack_into(packet, 0, MAGIC, FORMAT_BINARY, values_at, count, 0)
        packet[HEADER.size:HEADER.size + len(text)] = text
        packet[HEADER.size + len(text):values_at] = b" " * (values_at - HEADER.size - len(text))
        self._vals = np.frombuffer(packet, dtype="<f8", count=count, offset=values_at)
        self._idx = np.asarray(idx, dtype=np.intp)
        self.packet = packet
        self.tags = tuple(tags)
        self.skipped = tuple(skipped)
        self.recipe = recipe
        return self.skipped

    def pack(self, row):
        """This tick's packet: the values copied out of `row` (a trace row). Allocates nothing; the same bytearray is
        returned every tick (a sender copies it: UDP into the kernel, TCP into its queue)."""
        np.take(row, self._idx, out=self._vals)
        return self.packet
