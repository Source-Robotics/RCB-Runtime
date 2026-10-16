"""
Networking setup for inter-process communication.

ZMQ command channels use IPC (Unix domain sockets) instead of TCP loopback.
IPC skips the TCP/IP stack entirely — direct kernel buffer copy between
processes on the same machine. ~2–10 µs latency vs ~20–60 µs for TCP loopback.

Socket topology:
  ZMQ_CMD_SOCKET             — RTI binds (PULL),  all senders connect (PUSH)
  ZMQ_STATUS_SOCKET          — command_executor binds (PULL), RTI connects (PUSH)
  ZMQ_EXEC_TRAJECTORY_SOCKET — RTI binds (PULL), command_executor planner connects (PUSH)
  ZMQ_EXEC_STATUS_SOCKET     — command_executor binds (PULL), RTI connects (PUSH)

UDP ports are unchanged — they serve external network communication.
"""

# comms_handler.py
import logging
import os
import zmq
import time
from multiprocessing import Queue
import socket
import json
import select

logger = logging.getLogger("rti.net")  # docs/logging.md "Logger names"


# =============================================================================
# Port map — two UDP flows, one ZMQ command channel
#
#  PC (src: 30011*) ──command──▶ Robot (dst: 30001)   ← robot listens here
#  Robot (src: 30022*) ─state──▶ PC   (dst: 30002)    ← PC listens here
#
#  * optional fixed ports — ephemeral ports work fine if not needed
# =============================================================================

GUI_PORT             = 8081   # NiceGUI web dashboard (local)

# --- Telemetry broadcast (robot → local network, always running) ---
TELEMETRY_PORT        = 30003  # Destination port for UDP broadcast telemetry
SOURCE_PORT_BROADCAST = 30030  # Robot's fixed source port for telemetry sends

# Destination address for telemetry packets.
#   "255.255.255.255" — limited broadcast: every device on the subnet receives
#       every packet. Simple, zero config.  Bad on WiFi: AP sends every packet
#       at the base rate (1–6 Mbps), wakes all sleeping clients 333×/s, starves
#       unicast traffic → WiFi clients lose connectivity at high rates.
#   Specific IP      — unicast: only the dashboard PC receives the packets.
#       Switch handles it as a normal unicast frame; no airtime penalty; WiFi
#       clients are not disturbed.  Change this to your dashboard PC's IP to
#       fix WiFi degradation.  You can still run multiple dashboards by setting
#       this to the subnet's directed broadcast (e.g. "192.168.0.255"), which
#       does NOT hit the WiFi base-rate penalty on most managed APs (unlike
#       255.255.255.255 which is delivered at the mandatory lowest rate).
TELEMETRY_DESTINATION = "255.255.255.255"   # ← change to dashboard PC IP to fix WiFi

# --- RTI mode UDP (robot ↔ external PC, only active in RTI mode) ---
RTI_RECV_BUF         = 2048   # >> the 101-byte command packet; see drain_rti_udp
RTI_PORT_RECEIVE     = 30001  # Robot binds here; PC sends commands to this port
RTI_PORT_SENDER      = 30002  # PC binds here; robot sends state packets to this port

SOURCE_PORT_RTI      = 30022  # Optional: robot's fixed source port for RTI state sends
                               #   (bind commented out in udp_socket_send_setup — use if
                               #    the PC needs a predictable source port for filtering)
EXPECTED_REMOTE_PORT = 30011  # Optional: only accept commands from this port on the PC
                               #   pass allowed_port=None to drain_rti_udp to disable

# --- SOFT_RTI TCP transport (optional; UDP stays the default) ---
# One connection carries BOTH directions: the PC's commands up, the robot's state packets
# down. The robot LISTENS and the PC dials in, mirroring the UDP convention where the robot
# binds and access control lives robot-side.
#
# Why offer TCP at all, when UDP is the right transport for real-time control: industrial
# streaming interfaces ship both (real-time over UDP, soft real-time over TCP), and a PC that already speaks TCP through a
# firewall should not have to open a second hole. What it costs is HEAD-OF-LINE BLOCKING —
# a lost packet stalls everything behind it until retransmission, so UDP's "one dropped
# setpoint" becomes TCP's "50 frozen ticks". That trade is the point of measuring both.
RTI_PORT_STREAM_TCP  = 30003  # Robot binds and listens here for the SOFT_RTI TCP stream
RTI_TCP_BACKLOG_MAX  = 262144 # bytes of unsent state before the link is declared failed
RTI_TCP_MAX_READS    = 64     # recv() calls per tick; bounds the drain so a flooding PC
                               #   cannot hold the RT loop inside it (see drain_rti_tcp)

# --- Future / other ---
SIMULATOR_PORT       = 27890  # Reserved for simulator connection (not yet implemented)

# ZeroMQ IPC socket paths — Unix domain sockets, same-machine only.
# IPC avoids the TCP/IP stack, giving ~2–10 µs latency vs ~20–60 µs for tcp://localhost.
ZMQ_CMD_SOCKET    = "ipc:///tmp/robot_cmd"     # RTI receives commands on this (PULL/bind)
ZMQ_STATUS_SOCKET = "ipc:///tmp/robot_status"  # RTI sends status on this (PUSH/connect)

# --- EXEC mode trajectory transport (Phase 3) ---
# Planner thread (in command_executor) PUSHes trajectory batches; RTI PULLs them
# non-blocking in the RT loop. RTI PUSHes checkpoint completions; command_executor PULLs.
ZMQ_EXEC_TRAJECTORY_SOCKET = "ipc:///tmp/robot_exec_trajectory"  # planner→RTI batches (RTI PULL/bind)
ZMQ_EXEC_STATUS_SOCKET     = "ipc:///tmp/robot_exec_status"      # RTI→executor checkpoints (executor PULL/bind)

# You can also add other constants here later

# The core plan (RTI 3, the CAN path 2, everything else 0-1) lives in config/cpu_cores.py; RT_CPU_CORE is
# re-exported here for the modules that have always imported it from this one.
from config.cpu_cores import RT_CPU_CORE, GENERAL_CPU_CORES  # noqa: E402,F401


def background_zmq_context(ctx=None, cores=GENERAL_CPU_CORES):
    """A ZMQ context whose C++ I/O and reaper threads run at NORMAL priority, OFF the RT core (2026-10-01).

    RTI's main thread is SCHED_FIFO 99 on core 3 and every thread it starts inherits that, so each
    zmq.Context() put two more FIFO-99 threads on the RT core — where a busy one delays the loop (equal FIFO
    priority never preempts). They only move bytes between sockets and never hold the Python GIL, so normal
    priority on the other cores is safe for them. (The Python listener thread is deliberately LEFT on core 3 at
    FIFO 99: it does hold the GIL, and preempted on a busy core it would make the loop wait for the GIL.)

    Apply before the context's FIRST socket: libzmq starts its threads then. Priority 0 is set explicitly
    because SCHED_OTHER requires it (libzmq 4.3.5 copes with the inherited 99 too — tested 2026-10-01 — but
    this does not rely on it). Verified: without this the threads are FIFO on the parent's core; with it,
    SCHED_OTHER on every other core (test_rt_health.py)."""
    ctx = zmq.Context() if ctx is None else ctx
    ctx.set(zmq.THREAD_SCHED_POLICY, os.SCHED_OTHER)
    ctx.set(zmq.THREAD_PRIORITY, 0)
    for cpu in cores:                       # the general cores: off RTI's core AND the CAN core (2026-10-12)
        ctx.set(zmq.THREAD_AFFINITY_CPU_ADD, cpu)
    return ctx


# A burst of commands within one tick is normal: the RT loop takes ONE command per tick (RTI.py's
# command_queue.get_nowait()), so two commands 0.1 ms apart — clear_errors then set_mode:EXEC, every
# api_tests/45 run — wait in the queue by design. The old "Queue already has a command! This should NOT happen!"
# fired on exactly that (1 487 times in three days) and meant nothing. What would mean something is the queue
# growing past what a burst explains — the loop not draining it. So: warn once per excursion past this depth,
# with the depth in the text (2026-10-03).
COMMAND_QUEUE_WARN_DEPTH = 16        # 64 ms of backlog at 250 Hz


def enqueue_command(command_queue, msg, depth_state):
    """Put one received command on the RT loop's queue; `depth_state` is the listener's {"warned": bool}."""
    depth = command_queue.qsize()
    if depth >= COMMAND_QUEUE_WARN_DEPTH:
        if not depth_state["warned"]:
            logger.warning(f"{depth} commands waiting for the RT loop (it takes one per tick) — is it draining?")
            depth_state["warned"] = True
    elif depth_state["warned"]:
        depth_state["warned"] = False
    command_queue.put(msg)


def zmq_listener(command_queue: Queue):

    ctx = background_zmq_context()
    depth_state = {"warned": False}
    
    # Command Receiver (PULL) — bind to IPC socket; senders connect to the same path
    cmd_socket = ctx.socket(zmq.PULL)
    cmd_socket.bind(ZMQ_CMD_SOCKET)

    logger.info(f"ZMQ command listener bound to {ZMQ_CMD_SOCKET}")

    while True:
        try:
            # Non-blocking check or block with timeout to allow for clean exits
            msg = cmd_socket.recv_string()
            logger.info(f"Received: {msg}")

            enqueue_command(command_queue, msg, depth_state)
            
        except Exception as e:
            logger.error(f"Error in listener: {e}")
            time.sleep(0.1)


def zmq_sender_setup():

    ctx = background_zmq_context()
    status_socket = ctx.socket(zmq.PUSH)
    # LINGER 0: nothing binds ZMQ_STATUS_SOCKET since the executor rewrite, so every "done" RTI pushes stays queued,
    # and with the default LINGER (-1, forever) the context's term() waited for them when SystemExit left main():
    # 15 of 20 service stops hung until the 10 s shutdown watchdog (the dumps in RTI.out.log, 2026-10-06..10).
    # Pending status messages are worthless once RTI stops; the other PUSH sockets set 0 too (2026-10-10).
    status_socket.setsockopt(zmq.LINGER, 0)
    #status_socket.setsockopt(zmq.SNDHWM, 10) #Default is 1000! The sender can queue 10 messages for that connection, the 11th blocks or fails
    #tatus_socket.setsockopt(zmq.IMMEDIATE, 1) # detect disconnected receiver
    #status_socket.setsockopt(zmq.SNDTIMEO, 0) # add timeout instead of pure non-block
    # Connect to the IPC status socket; command_executor binds (PULL) on the other end
    status_socket.connect(ZMQ_STATUS_SOCKET)

    return status_socket

def udp_socket_receive_setup():
    """Initialize and return the UDP receive socket (RTI commands from PC)."""
    RTI_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    RTI_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    RTI_socket.bind(('0.0.0.0', RTI_PORT_RECEIVE))
    # Non-blocking so drain_rti_udp() can use one recvfrom() per packet instead of a
    # select() + recvfrom() pair. Halves the syscalls on the RT hot path, and makes the
    # drain loop's correctness obvious from the code rather than dependent on select().
    RTI_socket.setblocking(False)
    return RTI_socket


# Alias used by RTI.py when imported from config.network_config
init_rti_receive_socket = udp_socket_receive_setup


# ---------------------------------------------------------------------------------------
# SOFT_RTI TCP transport
#
# THREE RULES, all of which exist because this runs inside a 4 ms RT tick:
#   1. Nothing blocks. Every socket is non-blocking and every call here returns immediately.
#   2. TCP_NODELAY is mandatory. Nagle coalesces small writes, and our command packet is 101
#      bytes — without it the stream picks up ~40 ms latency spikes.
#   3. TCP is a BYTE STREAM, not messages. A read can return half a packet or three and a
#      half, so the partial tail must survive to the next tick. That is what `buf` is for.
# ---------------------------------------------------------------------------------------

def init_rti_tcp_listener(port=None):
    """Robot-side listening socket for the SOFT_RTI TCP transport. Cheap; safe at startup.

    `port` is injectable so the offline tests can bind an ephemeral port (0) instead of the
    real one. Without that they could only pass while the runtime was DOWN -- RTI holds
    RTI_PORT_STREAM_TCP from startup -- and a test that only runs when the robot is off is a
    test nobody runs.
    """
    lsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lsock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    lsock.bind(('0.0.0.0', RTI_PORT_STREAM_TCP if port is None else port))
    # Backlog > 1 because this port gets probed: a reachability check, a port scan, a PC
    # that dials before pairing. Each such connection waits in this queue, and with a
    # backlog of 1 two of them wedged the port until RTI restarted — `ss` showed
    # `LISTEN Recv-Q 2 Send-Q 1` and every further connect timed out. The backlog alone is
    # not the fix (see drain_rti_tcp_backlog); it is the margin.
    lsock.listen(8)
    lsock.setblocking(False)
    return lsock


def drain_rti_tcp_backlog(lsock, limit=8):
    """Accept and immediately CLOSE anything waiting, when no stream is wanted.

    The listener is bound from startup, but RTI only accepts while a PC is paired AND the
    transport is tcp. Everything arriving outside that window — a reachability probe, a port
    scan, a PC that dials early — otherwise sits in the accept queue forever, and once the
    queue fills the port stops answering entirely. That is a wedged stream port from a
    handful of harmless TCP connects, which is both a real failure mode for the off-robot
    test suite and a trivial denial of service.

    Returns how many were reaped. Non-blocking; costs one failed syscall on an idle tick.
    """
    n = 0
    while n < limit:
        try:
            conn, _addr = lsock.accept()
        except (BlockingIOError, InterruptedError):
            break
        except OSError:
            break
        try:
            conn.close()
        except OSError:
            pass
        n += 1
    return n


def accept_rti_tcp(lsock, allowed_ip):
    """Non-blocking accept. Returns (conn, addr), or (None, None) if nobody is waiting.

    `allowed_ip` is the PC already paired over the executor's TCP channel — a connection
    from anywhere else is closed immediately rather than served, so the stream port cannot
    become a second, unauthenticated way to drive the arm.
    """
    try:
        conn, addr = lsock.accept()
    except (BlockingIOError, InterruptedError):
        return None, None
    except OSError:
        return None, None
    if allowed_ip is not None and addr[0] != allowed_ip:
        try:
            conn.close()
        except OSError:
            pass
        return None, None
    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    # Reap half-open sockets. A PC that closes politely sends a FIN and we see it at once;
    # one whose cable is pulled or whose machine dies sends nothing, and without keepalive
    # the socket would sit open forever. Linux defaults to 2 hours idle, which is useless
    # here, so probe aggressively: ~3 s to notice. This is hygiene, NOT the safety path —
    # the handler's command timeout is what stops the arm, and it fires long before this.
    conn.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    for opt, val in (("TCP_KEEPIDLE", 1), ("TCP_KEEPINTVL", 1), ("TCP_KEEPCNT", 3)):
        if hasattr(socket, opt):
            conn.setsockopt(socket.IPPROTO_TCP, getattr(socket, opt), val)
    conn.setblocking(False)
    return conn, addr


def drain_rti_tcp(conn, buf, frame_size):
    """Drain the stream into `buf` and return (newest_complete_frame, frames_this_tick).

    Mirrors drain_rti_udp's contract, including the count: only the newest setpoint can be
    applied, so a PC sending faster than the loop ticks has the rest discarded, and the count
    is what makes that visible instead of silent.

    `buf` is a bytearray MUTATED IN PLACE so the partial tail survives to the next tick
    without allocating. Returns (None, -1) if the peer closed or the socket died — TCP gives
    us a real disconnect signal, which UDP never can, and the caller should treat it as an
    immediate link loss rather than waiting for the command timeout.
    """
    # BOUNDED, not drain-until-empty. CLAUDE.md records `flush_rx()` live-locking because it
    # drained until the source went quiet and a busy bus never does — 0 flashes in 56 minutes,
    # no error, no timeout. A PC streaming flat out is the same shape of hazard: the socket
    # stays readable and the loop keeps finding data. The cap costs nothing in normal use
    # (one or two reads per tick) and turns a stall into a tick of latency.
    reads = 0
    while reads < RTI_TCP_MAX_READS:
        try:
            chunk = conn.recv(RTI_RECV_BUF)
        except (BlockingIOError, InterruptedError):
            break                    # socket drained — the normal exit
        except OSError:
            return None, -1
        if not chunk:
            return None, -1          # orderly close by the peer
        buf.extend(chunk)
        reads += 1
    whole = len(buf) // frame_size
    if whole == 0:
        return None, 0
    end = whole * frame_size
    latest = bytes(buf[end - frame_size:end])
    del buf[:end]                    # keep only the partial tail
    return latest, whole


def send_rti_tcp(conn, out, payload):
    """Queue one length-prefixed state packet and flush whatever the socket will take.

    Returns True if the link is healthy, False if it has failed and should be dropped.

    NEVER blocks and never partially frames. State packets are variable length (the recipe
    decides), so each is prefixed with a 4-byte big-endian length — unlike the command
    packet, which is a fixed 101 bytes and needs no prefix.

    BACKPRESSURE. If the PC stops reading, TCP stops accepting writes and `out` grows. Rather
    than dropping bytes — which would desynchronise the framing and corrupt every packet
    after it — the backlog is capped and the link is declared FAILED past RTI_TCP_BACKLOG_MAX
    (~256 KB, several seconds of state). A PC that far behind has already lost real-time
    control; saying so beats pretending the link is fine while its view of the robot silently
    ages.
    """
    out.extend(len(payload).to_bytes(4, "big"))
    out.extend(payload)
    while out:
        try:
            sent = conn.send(out)
        except (BlockingIOError, InterruptedError):
            break                    # socket full; the rest waits for the next tick
        except OSError:
            return False
        if sent <= 0:
            break
        del out[:sent]
    return len(out) <= RTI_TCP_BACKLOG_MAX


def udp_socket_send_setup():

    # --- RTI socket send---
    RTI_socket_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) 
    # RTI_socket_sender.bind((sender_ip, source_port))  # Bind to the fixed (LOCAL) send port and IP, otherwise it uses ephemeral port (ie it is autoassigned)


    # --- Telemetry socket (for broadcasting robot data) ---
    telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # SO_BROADCAST is required when TELEMETRY_DESTINATION is 255.255.255.255.
    # It is harmless when sending unicast — the socket still works normally.
    telemetry_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    telemetry_socket.bind(('0.0.0.0', SOURCE_PORT_BROADCAST))

    return RTI_socket_sender, telemetry_socket


def drain_rti_udp(sock, allowed_ip, allowed_port):
    """
    Drain all pending UDP packets from the RTI receive socket.
    Returns the latest valid data payload (bytes) from the allowed sender,
    or None if no valid packet was received this cycle.

    Args:
        sock:          Bound UDP receive socket (non-blocking via select).
        allowed_ip:    IP address string of the allowed remote sender.
                       None disables the IP check (accept any sender).
        allowed_port:  Port number of the allowed remote sender.
                       None disables the port check (accept any source port).

    A None filter means "don't check this field". This matters because UDP source
    ports are ephemeral unless the sender explicitly binds one, so the port check
    has to be opt-in — before 2026-08-01 the comparison was `addr[1] == allowed_port`,
    which silently rejected EVERY packet when None was passed (the documented way to
    disable it). See TODO/RTI_mode_plan.md §2.1 / §9.1.

    Returns:
        (bytes | None, int): the most recent VALID packet, and how many packets were pulled
        off the socket this tick. The count is what makes over-rate visible: only the newest
        setpoint can be applied, so a PC sending faster than the loop ticks has the rest
        silently discarded — success rate still reads 100%, misses still read 0, and half the
        operator's control effort vanishes with nothing to show it. Callers that only want
        the data can ignore the second element.
    """
    latest_data = None
    received = 0          # every packet pulled off the socket, accepted or not
    while True:
        try:
            # RECV_BUF is far larger than the 101-byte command packet on purpose. UDP
            # truncates SILENTLY: a datagram bigger than the buffer loses its tail with no
            # error, and the symptom would be "commands mysteriously rejected" rather than
            # anything pointing at the buffer. Oversize is flagged below instead.
            data, addr = sock.recvfrom(RTI_RECV_BUF)
        except BlockingIOError:
            break                     # socket drained — the normal exit
        except (InterruptedError, KeyboardInterrupt):
            break
        except OSError:
            break                     # never let a transient socket error kill the RT loop

        received += 1
        if len(data) >= RTI_RECV_BUF:
            # Cannot distinguish "exactly buffer-sized" from "truncated", so treat both as
            # suspect rather than feeding a possibly-headless packet to the unpacker.
            logger.warning("dropped a %d-byte datagram — at or over the %d-byte "
                            "receive buffer, so it may be truncated", len(data), RTI_RECV_BUF)
            continue
        if ((allowed_ip is None or addr[0] == allowed_ip) and
                (allowed_port is None or addr[1] == allowed_port)):
            latest_data = data
        # Packets from other senders are dropped but still counted: they consumed link
        # bandwidth and their existence is worth seeing.
    return latest_data, received


def broadcast_udp_telemetry(sock, data):
    """
    Sends telemetry data to TELEMETRY_DESTINATION:TELEMETRY_PORT.

    'data' can be a string, number, or dictionary.

    Change TELEMETRY_DESTINATION in this file to switch between broadcast
    (255.255.255.255) and unicast (specific dashboard PC IP).  Unicast
    eliminates WiFi degradation caused by high-rate broadcast traffic.
    """
    try:
        # Convert dict to JSON string if necessary
        if isinstance(data, (dict, list)):
            msg = json.dumps(data)
        else:
            msg = str(data)

        sock.sendto(msg.encode(), (TELEMETRY_DESTINATION, TELEMETRY_PORT))
    except Exception:
        # Avoid heavy printing in the real-time loop; consider logging instead
        pass


# PEP8 renames 2026-10-03: the old spellings stay importable for one release (external scripts).
UDP_socket_receive_setup = udp_socket_receive_setup
UDP_socket_send_setup = udp_socket_send_setup
broadcast_UDP_telemetry = broadcast_udp_telemetry
