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
import zmq
import time
from multiprocessing import Queue
import socket
import json
import select

logger = logging.getLogger(__name__)


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
                               #   (bind commented out in UDP_socket_send_setup — use if
                               #    the PC needs a predictable source port for filtering)
EXPECTED_REMOTE_PORT = 30011  # Optional: only accept commands from this port on the PC
                               #   pass allowed_port=None to drain_rti_udp to disable

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

def zmq_listener(command_queue: Queue):

    ctx = zmq.Context()
    
    # Command Receiver (PULL) — bind to IPC socket; senders connect to the same path
    cmd_socket = ctx.socket(zmq.PULL)
    cmd_socket.bind(ZMQ_CMD_SOCKET)

    logger.info(f"ZMQ command listener bound to {ZMQ_CMD_SOCKET}")

    while True:
        try:
            # Non-blocking check or block with timeout to allow for clean exits
            msg = cmd_socket.recv_string()
            logger.info(f"Received: {msg}")

            if command_queue.qsize() > 0: # Checks if your internal command queue already contains something.
                logger.warning("Queue already has a command! This should NOT happen!")
                # You can log this to a file or raise an alert

            command_queue.put(msg) # Put command into queue
            #socket.send_string("ack")  # Immediate ACK
            
        except Exception as e:
            logger.error(f"Error in listener: {e}")
            time.sleep(0.1)


def zmq_sender_setup():

    ctx = zmq.Context()
    status_socket = ctx.socket(zmq.PUSH)
    #status_socket.setsockopt(zmq.SNDHWM, 10) #Default is 1000! The sender can queue 10 messages for that connection, the 11th blocks or fails
    #tatus_socket.setsockopt(zmq.IMMEDIATE, 1) # detect disconnected receiver
    #status_socket.setsockopt(zmq.SNDTIMEO, 0) # add timeout instead of pure non-block
    # Connect to the IPC status socket; command_executor binds (PULL) on the other end
    status_socket.connect(ZMQ_STATUS_SOCKET)

    return status_socket

def UDP_socket_receive_setup():
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
init_rti_receive_socket = UDP_socket_receive_setup


def UDP_socket_send_setup():

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
            logging.warning("[RTI] dropped a %d-byte datagram — at or over the %d-byte "
                            "receive buffer, so it may be truncated", len(data), RTI_RECV_BUF)
            continue
        if ((allowed_ip is None or addr[0] == allowed_ip) and
                (allowed_port is None or addr[1] == allowed_port)):
            latest_data = data
        # Packets from other senders are dropped but still counted: they consumed link
        # bandwidth and their existence is worth seeing.
    return latest_data, received


def broadcast_UDP_telemetry(sock, data):
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


# --- Command socket (for receiving commands) ---
#command_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#command_sock.bind((robot_IP, command_port)) #Liste to specific IP for example assigned to ethernet connection?
#command_sock.bind(('0.0.0.0', command_port))  # Listen on all interfaces (ethernet & wifi)
# If i want to allow only data from specific IP address i can filter 
# Also for safety i could use a safeword / secret at the beggining of package to accept only data from there
# Example: in GUI or by sending command: allowed_ips = ["192.168.1.100", "192.168.1.101"] 
# other options: password or Cryptographic Authentication HMAC signatures, Unicast + IP Whitelisting + HMAC Authentication
# Receive a command
#print("Waiting for command...")
# This is blocking
#command_data, addr = command_sock.recvfrom(1024)
#print(f"Received command from {addr}: {command_data.decode()}")