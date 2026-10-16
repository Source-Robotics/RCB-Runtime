#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: can_hardware.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description: 
    CAN bus hardware initialization and management.
    Handles CAN communication setup and bus operations.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import logging
import os
import socket
import struct
import Spectral_BLDC as Spectral
import can
import time
import subprocess

logger = logging.getLogger("rti.can")  # docs/logging.md "Logger names"

# Requested CAN TX socket buffer. The kernel caps the request at
# net.core.wmem_max and then doubles it, so on a stock system (wmem_max
# 212992) this yields 425984 bytes ≈ ~550 frames — double the default
# headroom against the silent-ENOBUFS frame-drop failure (see CLAUDE.md §6).
# If wmem_max is ever raised via sysctl, the same request takes the full
# amount automatically. SO_SNDBUF is a LIMIT, not an allocation: memory is
# only consumed by frames actually queued, so the at-rest cost is zero.
CAN_TX_SNDBUF_REQUEST_BYTES = 4 * 1024 * 1024

# Automatic bus-off recovery delay, milliseconds. 0 = DISABLED, which was the
# effective setting before 2026-08-04 because `ip link set ... up` was called
# without this parameter.
#
# Bus-off is not hypothetical: it was observed on the bench rig the first time a
# driver board lost power mid-transmission. With recovery disabled the
# controller stays off the bus until someone runs `ip link set can0 down/up` by
# hand — so a single motor failing mid-frame would take CAN down and RTI would
# never get it back on its own. 100 ms is long enough not to thrash a genuinely
# broken bus, short enough to be invisible across a 4 ms control tick.
CAN_BUS_OFF_RESTART_MS = 100


# ==========================================================================
# CAN Interface Management (OS level)
# ==========================================================================

def is_can_interface_up(interface='can0'):
    """Return True if the CAN network interface is currently in UP state."""
    try:
        result = subprocess.run(
            ['ip', 'link', 'show', interface],
            capture_output=True, text=True, check=True
        )
        return 'state UP' in result.stdout
    except subprocess.CalledProcessError:
        return False


def start_can_interface(interface='can0', bitrate='1000000', txqueuelen='1000',
                        restart_ms=CAN_BUS_OFF_RESTART_MS):
    """
    Bring up a CAN network interface.

    Always brings the interface down first to ensure a clean state,
    which is required to change bitrate or type parameters.

    Args:
        interface (str): CAN interface name (default: 'can0')
        bitrate (str|int): CAN bus bitrate (default: '1000000')
        txqueuelen (str|int): TX queue length (default: '1000')
        restart_ms (int): automatic bus-off recovery delay in ms; 0 disables it.
                          See CAN_BUS_OFF_RESTART_MS — do not set this to 0.
    """
    logger.info(f"Starting CAN interface '{interface}'...")

    # Always bring the interface down first — required to change bitrate/type,
    # and handles the case where the UP flag is set but state is DOWN (NO-CARRIER).
    try:
        subprocess.run(['sudo', 'ip', 'link', 'set', interface, 'down'], check=True)
        logger.info(f"'{interface}' brought down cleanly.")
    except subprocess.CalledProcessError as e:
        logger.warning(f"Could not bring '{interface}' down (may already be down): {e}")

    try:
        subprocess.run(
            ['sudo', 'ip', 'link', 'set', interface, 'up', 'type', 'can',
             'bitrate', str(bitrate), 'restart-ms', str(int(restart_ms))],
            check=True
        )
        subprocess.run(
            ['sudo', 'ifconfig', interface, 'txqueuelen', str(txqueuelen)],
            check=True
        )
        logger.info(f"Interface '{interface}' started at {bitrate} bps "
                    f"(txqueuelen={txqueuelen}, restart-ms={int(restart_ms)})")
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to start CAN interface: {e}")
        raise


def stop_can_interface(interface='can0'):
    """
    Bring down a CAN network interface.

    Args:
        interface (str): CAN interface name (default: 'can0')
    """
    try:
        subprocess.run(['sudo', 'ip', 'link', 'set', interface, 'down'], check=True)
        logger.info(f"Interface '{interface}' stopped")
    except subprocess.CalledProcessError as e:
        logger.warning(f"Failed to stop CAN interface: {e}")


# ==========================================================================
# The CAN path gets a core of its own (2026-10-12; config/cpu_cores.py)
# ==========================================================================
# Measured on the arm (api_tests/69_can_gaps.py, TODO/can_tx_timing_plan.md §8): the CAN chip (on SPI) has an
# interrupt THREAD (irq/<n>-spi0.0, SCHED_FIFO 50) that the kernel may run on any core, and the send work it
# starts is queued on THAT core's per-CPU kernel worker, at normal priority.
#   - On RTI's core (where it landed whenever RTI slept, ~21 % of the time) that worker cannot run while RTI's
#     tick does: all CAN traffic stopped until RTI slept again — 2-3 ms freezes, ~10 a second in hard RTI.
#   - On a core shared with other programs it competes with them: under load (a Python interpreter starting
#     every 2 s) 134-172 stalls per 10 s, commands up to 9 ms late, J6's replies late on ~7.5 % of reads.
# So the CAN chip's and its SPI controller's interrupt threads go to CAN_CPU_CORE ALONE, and every interactive
# program (systemd's user.slice: SSH, VS Code, scripts, tests) is confined to the general cores — the runtime's
# own processes pin themselves there (config/cpu_cores.py). Measured that way under the same load: 3-7 stalls,
# as fast as with no load. The interrupt thread is created when the interface comes UP, so this runs after every
# start_can_interface(). The SPI controller's runtime power saving goes off too: it was NOT the cause, but it
# woke kernel workers ~19 000 times a second, and the verified setup had it off (user, 2026-10-12).

def can_spi_parts(device_path):
    """The CAN interface's resolved sysfs device path -> (the CAN chip's device name, its SPI controller's
    sysfs directory), e.g. '/sys/devices/.../1f00050000.spi/spi_master/spi0/spi0.0' ->
    ('spi0.0', '/sys/devices/.../1f00050000.spi'). None when the chip does not sit on SPI (a USB adapter)."""
    head, sep, _ = device_path.partition("/spi_master/")
    if not sep:
        return None
    return os.path.basename(device_path), head


def irq_numbers(interrupts_text, names):
    """/proc/interrupts -> the IRQ numbers whose device name (the line's last column) is one of `names`."""
    out = []
    for line in interrupts_text.splitlines():
        head, sep, rest = line.partition(":")
        fields = rest.split()
        if sep and head.strip().isdigit() and fields and fields[-1] in names:
            out.append(int(head))
    return out


def irq_thread_pids(numbers, proc_root="/proc"):
    """{pid: name} of the kernel's interrupt threads of those IRQs, 'irq/<n>-<device>' (matched on the
    'irq/<n>-' prefix: a thread name may be cut short)."""
    prefixes = tuple(f"irq/{n}-" for n in numbers)
    out = {}
    if not prefixes:
        return out
    for entry in os.listdir(proc_root):
        if not entry.isdigit():
            continue
        try:
            with open(os.path.join(proc_root, entry, "comm")) as fh:
                name = fh.read().strip()
        except OSError:
            continue
        if name.startswith(prefixes):
            out[int(entry)] = name
    return out


def parse_cpu_list(text):
    """'0-1,3' -> {0, 1, 3} (the kernel's CPU-list format)."""
    out = set()
    for part in text.strip().split(","):
        if not part:
            continue
        a, _, b = part.partition("-")
        out.update(range(int(a), int(b or a) + 1))
    return out


def can_path_tasks(interface="can0", sys_root="/sys", proc_root="/proc"):
    """READER for the dashboard (2026-10-13): what place_can_path places — the CAN chip's and its SPI controller's
    interrupt threads, found the same way ({pid: (role, thread name)}, role "CAN chip IRQ" / "CAN SPI IRQ"), and the SPI
    controller's runtime-power setting ("on" as RTI sets it; None if unknown). Reads only."""
    out = {"threads": {}, "power": None}
    parts = can_spi_parts(os.path.realpath(os.path.join(sys_root, "class", "net", interface, "device")))
    if parts is None:
        return out
    chip, controller_dir = parts
    try:
        with open(os.path.join(proc_root, "interrupts")) as fh:
            interrupts = fh.read()
    except OSError:
        interrupts = ""
    for role, name in (("CAN chip IRQ", chip), ("CAN SPI IRQ", os.path.basename(controller_dir))):
        for pid, tname in irq_thread_pids(irq_numbers(interrupts, (name,)), proc_root).items():
            out["threads"][pid] = (role, tname)
    try:
        with open(os.path.join(controller_dir, "power", "control")) as fh:
            out["power"] = fh.read().strip()
    except OSError:
        pass
    return out


def place_can_path(can_core, general_cores, interface="can0", sys_root="/sys", proc_root="/proc",
                   run=subprocess.run, get_affinity=os.sched_getaffinity):
    """Give the CAN path a core of its own: the CAN chip's and its SPI controller's interrupt threads to
    `can_core` only, the SPI controller kept powered, interactive programs (systemd's user.slice) confined to
    `general_cores` (runtime only: re-applied at every RTI start). Logs what it did; logs an ERROR naming what it
    could not do (RTI still runs, but CAN traffic can then stall). Returns
    {"threads": {name: [cores]}, "power": str | None, "user_cpus": [cores] | None, "problems": [str]}."""
    result = {"threads": {}, "power": None, "user_cpus": None, "problems": []}
    general = sorted(general_cores)
    dev = os.path.realpath(os.path.join(sys_root, "class", "net", interface, "device"))
    parts = can_spi_parts(dev)
    if parts is None:
        logger.info(f"'{interface}' is not an SPI CAN chip ({dev}) — no interrupt threads to place")
    else:
        chip, controller_dir = parts
        controller = os.path.basename(controller_dir)
        try:
            with open(os.path.join(proc_root, "interrupts")) as fh:
                interrupts = fh.read()
        except OSError as e:
            interrupts = ""
            result["problems"].append(f"cannot read {proc_root}/interrupts: {e}")
        threads = {}
        for name in (chip, controller):          # the CAN chip's interrupt and its SPI controller's: both measured
            found = irq_thread_pids(irq_numbers(interrupts, (name,)), proc_root)
            if not found:
                result["problems"].append(f"no interrupt thread found for {name}")
            threads.update(found)
        for pid, name in sorted(threads.items()):
            try:
                run(["sudo", "taskset", "-pc", str(can_core), str(pid)], check=True, stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE)
                now = sorted(get_affinity(pid))
            except (subprocess.CalledProcessError, OSError) as e:
                result["problems"].append(f"{name} (pid {pid}): {e}")
                continue
            result["threads"][name] = now
            if now != [can_core]:
                result["problems"].append(f"{name} (pid {pid}) is on cores {now}, not [{can_core}]")
        power = os.path.join(controller_dir, "power", "control")
        try:
            with open(power) as fh:
                state = fh.read().strip()
            if state != "on":
                run(["sudo", "tee", power], input=b"on\n", check=True, stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE)
                with open(power) as fh:
                    state = fh.read().strip()
            result["power"] = state
            if state != "on":
                result["problems"].append(f"{power} reads '{state}', not 'on'")
        except (subprocess.CalledProcessError, OSError) as e:
            result["problems"].append(f"{power}: {e}")
    # Interactive programs off the CAN core (and RTI's): systemd's user.slice, runtime only.
    cpus = ",".join(str(c) for c in general)
    effective = os.path.join(sys_root, "fs", "cgroup", "user.slice", "cpuset.cpus.effective")
    try:
        run(["sudo", "systemctl", "set-property", "--runtime", "user.slice", f"AllowedCPUs={cpus}"], check=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        with open(effective) as fh:
            got = sorted(parse_cpu_list(fh.read()))
        result["user_cpus"] = got
        if got != general:
            result["problems"].append(f"user.slice runs on cores {got}, not {general}")
    except (subprocess.CalledProcessError, OSError, ValueError) as e:
        result["problems"].append(f"user.slice AllowedCPUs={cpus}: {e}")
    if result["problems"]:
        logger.error(f"could not give the CAN path core {can_core} alone: {'; '.join(result['problems'])} — "
                     f"CAN traffic can stall (api_tests/69_can_gaps.py measures it)")
    else:
        logger.info(f"interrupt threads {', '.join(result['threads']) or '(none)'} on core {can_core} alone; "
                    f"user programs on cores {result['user_cpus']}; SPI power: {result['power']}")
    return result


# ==========================================================================
# Wait for the tick's first frame to leave (TODO/can_tx_timing_plan.md option A, 2026-10-12; since then part of
# option B, read first / send early, §9) — its state is option B's switch (system.xml <can_read_first>, ON by default)
# ==========================================================================
# RTI's first command frame of a tick is loaded into the CAN chip by a kernel worker on RTI's own core,
# which runs only when RTI sleeps: the tick's commands leave ~1.5 ms after they were computed, and the
# joints' exchange must fit in RTI's sleep (the last replies miss the next read when a tick runs long).
# With this on, RTI sleeps right after queueing its commands until its own first frame (J1's data
# command) has actually gone out — seen on a second raw socket, which the kernel hands our own frames
# when they finish sending (candump sees them the same way) — and the rest of the exchange runs during
# RTI's tick on the other cores. NEVER a fixed short sleep instead: waking RTI while that worker is
# halfway through loading a frame holds the CAN driver until RTI sleeps again — the freeze of 2026-10-12.

_CAN_EFF_FLAG, _CAN_RTR_FLAG, _CAN_SFF_MASK = 0x80000000, 0x40000000, 0x7FF
FIRST_FRAME_IDS = (0x004, 0x008)        # node 0 (J1): data pack 1 (command 2), data pack PD (command 4)


def open_first_frame_socket(interface="can0", can_ids=FIRST_FRAME_IDS):
    """A raw CAN socket that receives only those standard data frames — our own included (the sending socket's
    loopback, on by default, hands every other socket of the interface each frame once it has been sent)."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    mask = _CAN_SFF_MASK | _CAN_EFF_FLAG | _CAN_RTR_FLAG
    sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b"".join(struct.pack("=II", i, mask) for i in can_ids))
    sock.bind((interface,))
    return sock


class FirstFrameWait:
    """Off until set_enabled(True) (RTI's `can_read_first:1`, option B — whose switch this object's state is). Per
    tick: prepare() before the commands are queued, then wait() right after — or skip() on a tick that queued none
    (HOMING / FLASHING)."""

    DRAIN_MAX = 512

    def __init__(self, interface="can0", timeout_s=0.001, open_socket=open_first_frame_socket):
        self.interface = interface
        self.timeout_s = timeout_s
        self._open = open_socket
        self._sock = None
        self._buf = bytearray(16)
        self.enabled = False
        self.waits = 0
        self.timeouts = 0
        self.wait_sum = 0.0
        self.wait_max = 0.0

    def set_enabled(self, on):
        """Cold path (a command): open / close the socket, log the change and, when switching off, the waits."""
        on = bool(on)
        if on == self.enabled:
            return True
        if on:
            try:
                self._sock = self._open(self.interface)
                self._sock.settimeout(self.timeout_s)
            except OSError as e:
                self._sock = None
                logger.error(f"read-first: cannot open a socket on {self.interface}: {e} — stays OFF (the old tick order)")
                return False
            self.waits = self.timeouts = 0
            self.wait_sum = self.wait_max = 0.0
            self.enabled = True
            logger.info(f"read-first ON: RTI reads CAN at the start of each tick and waits (up to "
                        f"{self.timeout_s * 1e3:.1f} ms) for its first frame to leave (TODO/can_tx_timing_plan.md option B)")
            return True
        self.enabled = False
        mean = self.wait_sum / self.waits * 1e3 if self.waits else 0.0
        logger.info(f"read-first OFF after {self.waits} waits: mean {mean:.3f} ms, max {self.wait_max * 1e3:.3f} ms, "
                    f"{self.timeouts} timed out")
        try:
            self._sock.close()
        except (OSError, AttributeError):
            pass
        self._sock = None
        return True

    def prepare(self):
        """Before the tick's commands are queued: throw away every echo already there, so the next frame the socket
        gives is this tick's. EVERY tick, not only after a timeout (fixed 2026-10-12, found on the arm): an echo that
        came later than the next tick's drain was taken as that tick's own, and from then on every wait returned at
        once on the previous tick's echo — 'one behind' for good, the wait silently doing nothing. Now a late echo
        costs at most one tick."""
        self._drain()

    def skip(self):
        """A tick that queued no data commands (the next prepare() drains what it left)."""

    def wait(self):
        """Right after the tick's commands were queued: sleep until J1's command has left (or timeout_s). Returns the
        seconds waited, -1.0 on a timeout."""
        t0 = time.perf_counter()
        try:
            self._sock.recv_into(self._buf)
        except OSError:                      # socket.timeout is one
            self.timeouts += 1
            return -1.0
        dt = time.perf_counter() - t0
        self.waits += 1
        self.wait_sum += dt
        if dt > self.wait_max:
            self.wait_max = dt
        return dt

    def _drain(self):
        """Read everything waiting, without blocking — every tick while the switch is on (a few syscalls)."""
        self._sock.setblocking(False)
        try:
            for _ in range(self.DRAIN_MAX):
                self._sock.recv_into(self._buf)
        except OSError:                      # BlockingIOError: empty
            pass
        finally:
            self._sock.settimeout(self.timeout_s)


# ==========================================================================
# The fast CAN read (2026-10-12, option B "send early" — TODO/can_tx_timing_plan.md §9.10)
# ==========================================================================
# python-can's receive: a select() on the socket, recvmsg(), then a full Message object (a dozen attributes and
# checks) and a log call — 15.7 us a frame measured on this Pi, this reader 6.1 (2026-10-12, live bus, the system
# interpreter, cores 0-1; the empty read that ends each tick's drain ~3 us either way). It reads the SAME socket (the
# bus's own: nothing is duplicated, sends are untouched) with one non-blocking recvmsg and computes exactly the fields
# python-can computes — same structs, same arithmetic, error frames included — into a light frame object, and the
# same (node, command, error bit) the Spectral library derives. RTI's CAN read uses it while option B is on.

class CanFrame:
    """The fields of a received frame RTI's decode uses (timestamp, arbitration_id, dlc, data) — the values python-can's
    Message would hold."""
    __slots__ = ("timestamp", "arbitration_id", "dlc", "data", "is_extended_id", "is_remote_frame", "is_error_frame")


class FastCanReader:
    def __init__(self, communication):
        import can.interfaces.socketcan.constants as cc
        import can.interfaces.socketcan.socketcan as sc
        from Spectral_BLDC.Spectral_BLDC import CanMessage
        self._sock = communication.bus.socket
        self._mtu = cc.CANFD_MTU
        self._anc = sc.RECEIVED_ANCILLARY_BUFFER_SIZE
        self._hdr = sc.CAN_FRAME_HEADER_STRUCT
        self._ts = sc.RECEIVED_TIMESTAMP_STRUCT
        self._eff, self._rtr, self._err = cc.CAN_EFF_FLAG, cc.CAN_RTR_FLAG, cc.CAN_ERR_FLAG
        self._op_error = can.CanOperationError       # the module's own `import can`
        self._can_message = CanMessage

    def receive_can_messages(self, timeout=0):
        """(frame, CanMessage(node_id, command_id, error_bit)), or (None, None) when the socket is empty. Never waits
        (RTI's read always asked for timeout=0)."""
        try:
            cf, ancillary, _, _ = self._sock.recvmsg(self._mtu, self._anc, socket.MSG_DONTWAIT)
        except (BlockingIOError, InterruptedError):
            return None, None
        except OSError as error:                     # as python-can: the interface went down, ...
            raise self._op_error(f"Error receiving: {error.strerror}", error.errno) from error
        can_id, dlc, _ = self._hdr.unpack_from(cf)
        seconds, nanoseconds = self._ts.unpack_from(ancillary[0][2])
        frame = CanFrame()
        frame.timestamp = seconds + nanoseconds * 1e-9
        frame.is_extended_id = bool(can_id & self._eff)
        frame.is_remote_frame = bool(can_id & self._rtr)
        frame.is_error_frame = bool(can_id & self._err)
        frame.arbitration_id = can_id & 0x1FFFFFFF if frame.is_extended_id else can_id & 0x000007FF
        frame.dlc = dlc
        frame.data = bytearray(cf[8:8 + dlc])
        aid = frame.arbitration_id
        return frame, self._can_message((aid >> 7) & 0xF, (aid >> 1) & 0x3F, aid & 0x1)


# ==========================================================================
# CAN Communication Setup
# ==========================================================================

def initialize_can_communication(bustype='socketcan', channel='can0', bitrate=1000000):
    """
    Initialize CAN communication interface.
    
    Args:
        bustype (str): Type of CAN interface (default: 'socketcan')
        channel (str): CAN channel name (default: 'can0')
        bitrate (int): CAN bus bitrate in bps (default: 1000000)
    
    Returns:
        CanCommunication: Initialized CAN communication object
    """
    try:
        communication = Spectral.CanCommunication(
            bustype=bustype,
            channel=channel,
            bitrate=bitrate
        )
        logger.info(f"Communication initialized: {channel} @ {bitrate} bps")
        raise_can_tx_buffer(communication)
        count_can_tx_drops(communication)
        return communication

    except Exception as e:
        logger.error(f"Failed to initialize CAN: {e}")
        raise


def raise_can_tx_buffer(communication, request_bytes=CAN_TX_SNDBUF_REQUEST_BYTES):
    """Grow the CAN socket's TX buffer (SO_SNDBUF) — best effort, never fatal.

    A full TX buffer makes send() fail with ENOBUFS, which Spectral_BLDC
    currently swallows — the frame silently never leaves the Pi (root cause of
    the 2026-07 gripper-watchdog bug). Pacing and small bursts keep the queue
    near-empty; this raise is the backstop that moves the cliff itself.

    Only applies to socketcan buses (needs communication.bus.socket); anything
    else is skipped with a log line.
    """
    sock = getattr(getattr(communication, "bus", None), "socket", None)
    if sock is None:
        logger.info("CAN TX buffer: bus exposes no raw socket — skipping SO_SNDBUF raise")
        return
    try:
        before = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(request_bytes))
        after = sock.getsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF)
        logger.info(f"CAN TX buffer: {before} -> {after} bytes "
                    f"(~{after // 768} frames; requested {int(request_bytes)}, "
                    f"kernel caps at net.core.wmem_max then doubles)")
    except OSError as e:
        logger.warning(f"CAN TX buffer raise failed (keeping default): {e}")


CAN_TX_DROP_LOG_WINDOW_S = 10.0      # at most one "frames dropped" line per this many seconds


def count_can_tx_drops(communication, window_s=CAN_TX_DROP_LOG_WINDOW_S):
    """Make the library's send layer COUNT what it swallows (2026-10-03).

    Spectral_BLDC.CanCommunication.send_can_message catches python-can's CanOperationError and does nothing, so
    a frame the socketcan TX queue refused is gone without a trace — CLAUDE.md §6 records how that cost the
    gripper its watchdog for weeks. (python-can's socketcan send() with no timeout is a zero-timeout poll: it
    raises "Transmit buffer full" at once and never blocks the RT loop — read in python-can 4.5.0.)

    This replaces the INSTANCE's send with one that sends exactly the same frame and counts the refusals:
    the first drop of a quiet period is logged at WARNING at once, then at most one line per `window_s` with
    the count. Never raises into the caller (the library's contract). The counter stays on the object as
    `communication.can_tx_drops` for diagnostics; returned too.
    """
    bus = communication.bus
    counter = {"dropped": 0, "pending": 0, "last_log": float("-inf"), "last_error": ""}

    def send_can_message(message_id, data=b"", remote_frame=False):
        message = can.Message(arbitration_id=message_id, data=data, is_remote_frame=remote_frame,
                              is_extended_id=False)
        try:
            bus.send(message)
        except Exception as e:                       # CanOperationError ("Transmit buffer full"), or anything else
            counter["dropped"] += 1
            counter["pending"] += 1
            counter["last_error"] = str(e)
            now = time.monotonic()
            if now - counter["last_log"] >= window_s:
                logger.warning(f"{counter['pending']} frame(s) dropped — the TX queue refused them ({e}); "
                               f"{counter['dropped']} since start")
                counter["pending"] = 0
                counter["last_log"] = now

    communication.send_can_message = send_can_message
    communication.can_tx_drops = counter
    return counter


# ==========================================================================
# CAN Bus Scanning
# ==========================================================================
# ==========================================================================
# Scan can bus for all available CAN devices! NOTE that no 2 devices can share
# same CAN id!
# ==========================================================================    
def scan_can_bus(communication, max_id=16, timeout=0.0025, num_pings=2, window_s=0.02):
    """
    Scan CAN bus for all available devices.

    Pings every ID (num_pings sweeps), then reads every frame that arrives for `window_s` seconds and counts a node
    only when the frame is ITS PING REPLY. 2026-10-09: the scan runs inside the live loop (RTI tick 8), where the
    receive queue already holds the drivers' normal replies; it used to read ONE frame per ping and count the node if
    that frame merely came from it — so it reported every node missing on 31 of 46 starts (while the same selfcheck
    read their firmware versions), and could count a node from a frame that was not a ping reply.

    Args:
        communication: CAN communication object
        max_id (int): Maximum CAN ID to scan (default: 16)
        timeout (float): Timeout of each receive poll (default: 0.0025s)
        num_pings (int): Number of ping attempts per device (default: 2)
        window_s (float): How long replies are collected after the pings (default: 0.02s)

    Returns:
        np.array: Array indicating which CAN IDs responded (1=present, 0=absent)
    """
    import numpy as np

    scan_received_ids = np.zeros(16, dtype=int)

    # Create test nodes for scanning
    CAN_test_node = [
        Spectral.SpectralCAN(node_id=i, communication=communication)
        for i in range(max_id)
    ]

    logger.info(f"Scanning CAN bus (IDs 0-{max_id-1})...")

    # Perform multiple ping sweeps
    for ping_round in range(num_pings):
        for i in range(max_id):
            CAN_test_node[i].Send_Respond_Ping()

    # Collect the ping replies — every other frame (the loop's own replies) is skipped
    ping_id = Spectral.SpectralCAN.SEND_RESPOND_PING_ID
    deadline = time.monotonic() + window_s
    while time.monotonic() < deadline:
        message, msg_ID = communication.receive_can_messages(timeout=timeout)
        if msg_ID is not None and msg_ID[1] == ping_id and 0 <= msg_ID[0] < max_id:
            scan_received_ids[msg_ID[0]] = 1
    
    # Print results
    detected_ids = [i for i in range(max_id) if scan_received_ids[i] == 1]
    logger.info(f"Detected devices at IDs: {detected_ids}")
    logger.info(f"Scan result: {scan_received_ids}")
    
    return scan_received_ids


# ==========================================================================
# Motor Configuration Pre-loading
# ==========================================================================

# ==========================================================================
# CAN Bus Diagnostics
# ==========================================================================

# ==========================================================================
# Complete CAN System Initialization
# ==========================================================================

def initialize_can_system(channel='can0', bitrate=1000000, 
                         perform_scan=True, scan_on_startup=True):
    """
    Complete CAN system initialization.
    
    Args:
        channel (str): CAN channel name (default: 'can0')
        bitrate (int): CAN bus bitrate (default: 1000000)
        perform_scan (bool): Perform bus scan (default: True)
        scan_on_startup (bool): Scan immediately after init (default: True)
    
    Returns:
        dict: Dictionary containing:
            - communication: CAN communication object
            - scan_results: Scan results if performed, else None
    """
    logger.info("CAN system initializing...")
    
    # Initialize CAN communication
    communication = initialize_can_communication(
        bustype='socketcan',
        channel=channel,
        bitrate=bitrate
    )
    
    scan_results = None
    if perform_scan and scan_on_startup:
        time.sleep(0.5)  # Give devices time to boot
        scan_results = scan_can_bus(communication)
    
    logger.info("CAN system initialization complete")
    
    return {
        'communication': communication,
        'scan_results': scan_results
    }


if __name__ == "__main__":
    """Test CAN hardware initialization (requires actual hardware)"""
    print("CAN Hardware Test")
    print("-" * 50)
    print("\nNote: This test requires actual CAN hardware")
    print("Please ensure CAN interface is up: sudo ip link set can0 up type can bitrate 1000000")
    
    try:
        # Initialize CAN system
        can_system = initialize_can_system(
            channel='can0',
            bitrate=1000000,
            perform_scan=True
        )
        
        print("\nCAN system initialized successfully!")
        print("You can now communicate with CAN devices.")
        
    except Exception as e:
        print(f"\nFailed to initialize CAN system: {e}")
        print("Make sure:")
        print("  1. CAN hardware is connected")
        print("  2. CAN interface is up")
        print("  3. You have proper permissions")
