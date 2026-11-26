"""The recorder's TCP link to RTI's packets (2026-11-26, TODO/p4_recording_plan.md section 2B D).

RTI's telemetry goes out over UDP every tick, whether anyone listens or not. A program that must not miss a tick — a
recorder of training data — can instead connect over TCP to TELEMETRY_TCP_PORT and get the same packets on that
connection, each prefixed with its length (4 bytes, big-endian), as the SOFT_RTI TCP stream frames them.

Inside RTI's tick, so it never waits and copies little:

  * one connection at a time; a second one is accepted and closed at once (the listener never wedges);
  * the listener is looked at every `accept_every` ticks (a refused connect costs nothing on the other ticks);
  * a packet goes straight to the kernel (`sendmsg` of the length and the packet: no copy) while nothing is queued;
    what the kernel will not take yet is queued as one bytes copy, and the queue is sent from its head as memoryview
    slices (never compacted). The kernel's own buffer grows to 4 MB (TCP autotuning, net.ipv4.tcp_wmem), ~3 s of
    every tag in binary;
  * past `max_backlog` bytes queued (the PC stopped reading for seconds) the connection is DROPPED, counted in `drops`:
    a recorder reconnects and counts the ticks it missed. The arm is never affected: this link controls nothing.
"""
from __future__ import annotations

import socket
from collections import deque

from config.networking import TELEMETRY_TCP_PORT, accept_rti_tcp, drain_rti_tcp_backlog, init_rti_tcp_listener

MAX_BACKLOG = 2 * 1024 * 1024       # bytes queued on top of the kernel's buffer before the link is dropped
ACCEPT_EVERY = 25                   # ticks between looks at the listener (0.1 s at 250 Hz)


class TelemetryLink:
    """`tick(n, payload)` once per tick: accepts a recorder now and then, sends it the tick's packet. `peer` = the
    connected (ip, port) or None; `drops` = connections dropped for falling behind (a recorder that closes or resets
    its connection is not counted); `sent` = packets sent whole."""

    def __init__(self, port: int | None = None, max_backlog: int = MAX_BACKLOG, accept_every: int = ACCEPT_EVERY):
        self.lsock = init_rti_tcp_listener(TELEMETRY_TCP_PORT if port is None else port)
        self.port = self.lsock.getsockname()[1]
        self.max_backlog = int(max_backlog)
        self.accept_every = max(1, int(accept_every))
        self.conn: socket.socket | None = None
        self.peer: tuple | None = None
        self.queue: deque[memoryview] = deque()
        self.queued = 0
        self.drops = 0
        self.sent = 0
        self.changed = False          # set when peer / drops change: RTI publishes its status then (never per tick)

    def tick(self, n: int, payload) -> None:
        """This tick: `n` the tick counter, `payload` the packet (bytes / bytearray) or None. Never raises, never
        blocks."""
        if n % self.accept_every == 0:
            if self.conn is None:
                conn, addr = accept_rti_tcp(self.lsock, None)
                if conn is not None:
                    self.conn, self.peer = conn, addr
                    self.changed = True
            else:
                drain_rti_tcp_backlog(self.lsock)              # one recorder at a time: the others closed
        if self.conn is None or payload is None:
            return
        size = len(payload)
        try:
            if not self.queue:                              # straight to the kernel: the length and the packet
                head = size.to_bytes(4, "big")
                try:
                    k = self.conn.sendmsg((head, payload))
                except (BlockingIOError, InterruptedError):
                    k = 0
                if k == size + 4:
                    self.sent += 1
                    return
                rest = (head + bytes(payload))[k:]          # the part the kernel did not take: one copy
                self.queue.append(memoryview(rest))
                self.queued += len(rest)
            else:                                           # behind: this packet joins the queue, the queue is sent
                frame = size.to_bytes(4, "big") + bytes(payload)
                self.queue.append(memoryview(frame))
                self.queued += len(frame)
                self._flush()
        except OSError:                                     # the recorder went away (reset, broken pipe)
            self._drop(stalled=False)
            return
        if self.queued > self.max_backlog:
            self._drop(stalled=True)

    def _flush(self) -> None:
        """Send from the queue's head until the kernel takes no more. OSError (a dead link) goes to the caller."""
        conn = self.conn
        while self.queue and conn is not None:
            head = self.queue[0]
            try:
                k = conn.send(head)
            except (BlockingIOError, InterruptedError):
                return
            self.queued -= k
            if k < head.nbytes:
                self.queue[0] = head[k:]
                return
            self.queue.popleft()
            self.sent += 1

    def _drop(self, stalled: bool) -> None:
        try:
            if self.conn is not None:
                self.conn.close()
        except OSError:
            pass
        self.conn = self.peer = None
        self.queue.clear()
        self.queued = 0
        self.drops += bool(stalled)
        self.changed = True

    def close(self) -> None:
        """RTI's stop: the connection and the listener closed."""
        for s in (self.conn, self.lsock):
            if s is not None:
                try:
                    s.close()
                except OSError:
                    pass
        self.conn = self.peer = None
