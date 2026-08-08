"""Flash motor-driver firmware over CAN via the STM32F103 bootloader.

Standalone runtime-DOWN tool, same contract as tune_motor_loops.py: it takes the
CAN bus for itself, so it refuses to start while RTI.py is alive. Stop the
runtime first:

    sudo systemctl stop autorun.service      # or rcb-runtime.service, see below

Typical session:

    # who is out there (REBOOTS every motor on the bus into its bootloader)
    python3 tools/flash_motor_firmware.py scan

    # find boards already sitting in the bootloader (failed flash), reboots nothing
    python3 tools/flash_motor_firmware.py scan --no-reset

    # sanity-check an image without touching the bus
    python3 tools/flash_motor_firmware.py validate --file fw.bin

    # flash joint 3 from a local image
    python3 tools/flash_motor_firmware.py flash --node 2 --file fw.bin

    # flash the gripper from the latest GitHub release
    python3 tools/flash_motor_firmware.py flash --node 6 --product spectral-bldc --latest

    # all six joints, sequentially, same image
    python3 tools/flash_motor_firmware.py flash-all-joints --file fw.bin

Protocol: commands on CAN ID 0x700, replies on 0x701+board_id, page data
streamed on ID (board_id<<7)|seq. After
the final WRITE_CRC the host goes silent; the bootloader validates the stored
CRC and reboots into the application on its own ~3 s inactivity timeout —
there is no jump command.

Design notes, deliberate:
  - python-can is used directly (not Spectral_BLDC's send layer) because
    Spectral's ``send_can_message`` swallows TX failures — the documented
    silent-ENOBUFS hazard. ``bus.send()`` raising is a feature here.
  - The WBUF word-by-word fallback from the reference host tools is NOT
    ported. It exists for lossy USB serial adapters; on socketcan the
    STREAM_STATUS chunk-verify + retry ladder is the recovery mechanism, and
    dead code in a flashing tool is a liability.
  - A board whose flash was interrupted keeps an invalid app CRC and waits in
    the bootloader forever: recover with ``scan --no-reset`` + ``flash``. The
    one near-unrecoverable mistake — an image linked at 0x08000000 instead of
    0x08003000 — is caught by ``validate_firmware_bytes`` before any erase.
"""

import argparse
import fcntl          # single-flasher lock; see acquire_bus_lock()
import json
import os
import struct
import subprocess
import sys
from multiprocessing import resource_tracker, shared_memory
import threading
import time
from collections import namedtuple
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace

import can

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import Spectral_BLDC as Spectral
from Spectral_BLDC import CAN_utils as spectral_can_utils
from hardware.can_hardware import (
    is_can_interface_up,
    start_can_interface,
    raise_can_tx_buffer,
)
import tune_motor_loops as T          # _rti_is_running — the bus-ownership guard

# ---------------------------------------------------------------- protocol constants
# CAN command codes (UART codes differ — this tool is CAN-only).
BL_WBUF = 0x01            # legacy word write, unused here (see module docstring)
BL_WPAGE = 0x02
BL_WCRC = 0x03
BL_PING = 0x04
BL_SET_ID = 0x05
BL_ERASE_APP = 0x06
BL_STREAM_BEGIN = 0x07
BL_STREAM_STATUS = 0x08

BL_ERROR_NAMES = {
    0: "OK",
    1: "INVALID_OFFSET",
    2: "INVALID_CRC",
    3: "FLASH_WRITE",
    4: "INVALID_PAGE_NUM",
}

CANID_BL_CMD = 0x700
CANID_BL_RPL_BASE = 0x701
APP_IN_RESET_CMD = 14     # Spectral app command — identical to SpectralCAN.Send_Reset()
BL_BOARD_ID_MAX = 13      # 14/15 reserved for the host (motor_setup.py, CLAUDE.md)
APP_NODE_ID_MAX = 15

PAGE_SIZE = 1024
CAN_STREAM_FRAME_COUNT = 128          # 8-byte frames per 1 KB page
# Frames between STREAM_STATUS verifies. Inherited from the reference
# implementation, and now MEASURED (2026-08-04 overnight sweep, 7-board rig,
# 100 flashes across 32/16/8/4/1 frames — every size 20/20 clean, zero retries):
#
#     32 -> 23.4 s    16 -> 24.1 s    8 -> 28.4 s    4 -> 37.0 s    1 -> 88.2 s
#
# Kept at 16 deliberately. 32 is ~3% faster and was equally clean, but on 20
# runs against 16's 100+ — not enough evidence to move a default for 0.7 s on a
# 24 s operation. Raise it only after a 100-run soak at 32 comes back clean.
# The wider result is that there is enormous margin: even chunk 1, at 12,513
# STREAM_STATUS round trips per flash, never needed a single retry.
DEFAULT_STREAM_CHUNK_FRAMES = 16
MAX_APP_PAGES = 116
APP_BASE_ADDRESS = 0x08003000
FLASH_END_ADDRESS = 0x08020000
RAM_START_ADDRESS = 0x20000000
RAM_END_ADDRESS = 0x20005000

# Retry ladder and timeouts. Conservative on purpose: a retry costs milliseconds,
# a half-written page costs a trip to the bench with an SWD probe.
PAGE_RETRIES = 10
STREAM_CHUNK_RETRIES = 5
# Scan listen windows. An ACTIVE scan has to outlast the app-reset -> bootloader
# delay, MEASURED at ~8.3 s on the bench rig (2026-08-04) — the same delay that
# made the original flash time out. A 1.2 s window (the reference default, and
# this tool's until now) can NEVER see a board that is still rebooting, so an
# active scan silently found only boards that were already in a bootloader.
# A PASSIVE scan needs no such margin: a board already sitting in its bootloader
# answers the first ping, so keep it short and snappy.
SCAN_LISTEN_ACTIVE_S = 20.0
SCAN_LISTEN_PASSIVE_S = 2.0

PING_RETRIES_NO_RESET = 4
# MEASURED ON HARDWARE (2026-08-04, 7-board bench rig, StepFOC v103): the
# bootloader does not answer until ~8.2 s after IN_RESET — the board stays
# silent for that whole time, then the bootloader appears. The reference host
# tools wait 40 x 0.2 s = 8 s and this tool originally waited 80 x 0.1 s = 8 s,
# so BOTH were timing out a fraction of a second before the bootloader came up,
# which reads as "could not reach bootloader" on a perfectly healthy board.
# 250 x 0.1 s = 25 s gives ~3x the measured delay. Cost of being generous is
# only paid when a board genuinely never appears.
PING_RETRIES_AFTER_RESET = 250
COMMAND_GAP_S = 0.005
APP_RESET_GAP_S = 0.25
CMD_TIMEOUT_S = 0.25
PING_TIMEOUT_S = 0.2
STREAM_BEGIN_TIMEOUT_S = 0.5
STREAM_STATUS_TIMEOUT_S = 0.25
WPAGE_TIMEOUT_S = 1.0
WCRC_TIMEOUT_S = 2.0
ERASE_TIMEOUT_S = 15.0    # erase itself is ~3.5 s

# Node roles on this robot (PAR6 + optional CAN gripper). Labels only — the
# protocol does not care; scan reports whatever answers.
NODE_ROLES = {0: "J1", 1: "J2", 2: "J3", 3: "J4", 4: "J5", 5: "J6",
              6: "gripper", 13: "timing-dummy"}
JOINT_NODE_IDS = tuple(range(6))
GRIPPER_NODE_ID = 6

# Two service names exist in the wild: scripts/install_service.sh installs
# "rcb-runtime.service", while older boxes (this one included) run the legacy
# "autorun.service" wrapper around autorun.sh. Detect rather than guess — a
# stop command naming the wrong unit is worse than useless in a hurry.
RUNTIME_SERVICE_CANDIDATES = ("rcb-runtime.service", "autorun.service")


def runtime_service_name():
    """Best guess at the systemd unit that owns supervisor.py on this box.

    Both unit files can be installed at once (this Pi has both, with only
    autorun.service enabled), so file existence alone picks the wrong one.
    Rank by what systemd actually thinks: running beats enabled beats merely
    installed.
    """
    best_rank, best_unit = -1, None
    for unit in RUNTIME_SERVICE_CANDIDATES:
        rank = -1
        for query, score in (("is-active", 2), ("is-enabled", 1)):
            try:
                result = subprocess.run(["systemctl", query, unit],
                                        capture_output=True, text=True, timeout=5)
            except (OSError, subprocess.SubprocessError):
                continue
            state = result.stdout.strip()
            if state in ("active", "activating", "enabled", "enabled-runtime"):
                rank = max(rank, score)
        if rank < 0 and any(Path(d, unit).exists()
                            for d in ("/etc/systemd/system", "/lib/systemd/system")):
            rank = 0
        if rank > best_rank:
            best_rank, best_unit = rank, unit
    return best_unit or RUNTIME_SERVICE_CANDIDATES[0]


def stop_runtime_hint():
    return (
        "REFUSING TO TOUCH THE BUS: RTI.py is running (loop_tick is advancing).\n"
        "It owns the CAN bus. Stop the runtime first:  "
        f"sudo systemctl stop {runtime_service_name()}"
    )


class BootloaderError(Exception):
    """A bootloader command failed (error reply or no reply after retries)."""


class FlashCancelled(Exception):
    """Cooperative cancel: the caller's cancel event was set mid-session."""


# ---------------------------------------------------------------- CRC32 (STM32 style)
# STM32F1 hardware CRC: poly 0x04C11DB7, init 0xFFFFFFFF, no reflection, no
# final XOR, fed as little-endian 32-bit words MSB-first (CRC-32/MPEG-2 on a
# little-endian byte stream). Must match what the bootloader computes on-chip, or
# every page verify fails.
_CRC_POLY = 0x04C11DB7
_CRC_INIT = 0xFFFFFFFF


def _build_crc_table():
    table = []
    for byte in range(256):
        crc = byte << 24
        for _ in range(8):
            crc = ((crc << 1) ^ _CRC_POLY if crc & 0x80000000 else crc << 1) & 0xFFFFFFFF
        table.append(crc)
    return tuple(table)


_CRC_TABLE = _build_crc_table()


def stm32_crc32(data):
    """Table-driven fast path (~20x the bitwise loop on the Pi)."""
    if len(data) % 4:
        raise ValueError(f"stm32_crc32: length {len(data)} must be a multiple of 4")
    crc = _CRC_INIT
    for (word,) in struct.iter_unpack("<I", data):
        crc ^= word
        for _ in range(4):
            crc = ((crc << 8) & 0xFFFFFFFF) ^ _CRC_TABLE[crc >> 24]
    return crc


def stm32_crc32_bitwise(data):
    """Bit-by-bit reference implementation. Kept as the cross-check oracle for the
    table-driven version (see test_firmware_flasher.py)."""
    if len(data) % 4:
        raise ValueError(f"stm32_crc32: length {len(data)} must be a multiple of 4")
    crc = _CRC_INIT
    for (word,) in struct.iter_unpack("<I", data):
        crc = (crc ^ word) & 0xFFFFFFFF
        for _ in range(32):
            if crc & 0x80000000:
                crc = ((crc << 1) ^ _CRC_POLY) & 0xFFFFFFFF
            else:
                crc = (crc << 1) & 0xFFFFFFFF
    return crc


def pad_to(data, boundary, fill=0xFF):
    """Pad to a boundary with 0xFF — the erased-flash value, so padding never
    changes the CRC the bootloader computes over the same flash region."""
    remainder = len(data) % boundary
    if remainder == 0:
        return bytes(data)
    return bytes(data) + bytes([fill]) * (boundary - remainder)


# ---------------------------------------------------------------- frame primitives
BlReply = namedtuple("BlReply", "board_id cmd error par1")


def bl_cmd_frame(board_id, cmd, par1=0, par2=0):
    """Command frame: ID 0x700, [board_id][cmd][par1 LE16][par2 LE32].
    NOTE: par2 is LITTLE-endian on the wire. Some protocol notes claim otherwise;
    the device firmware is little-endian and that is what matters."""
    data = struct.pack("<BBHI", board_id & 0xFF, cmd & 0xFF,
                       par1 & 0xFFFF, par2 & 0xFFFFFFFF)
    return can.Message(arbitration_id=CANID_BL_CMD, is_extended_id=False, data=data)


def stream_data_frame(board_id, seq, payload8):
    """Page-data frame: ID (board_id<<7)|seq, 8 raw page bytes, no reply."""
    if len(payload8) != 8:
        raise ValueError(f"stream frame payload must be 8 bytes, got {len(payload8)}")
    can_id = ((board_id & 0x0F) << 7) | (seq & 0x7F)
    return can.Message(arbitration_id=can_id, is_extended_id=False, data=payload8)


def app_can_id(node_id, command_id, error_bit=0):
    """Spectral application-protocol 11-bit ID: (node<<7)|(cmd<<1)|err."""
    return ((node_id & 0xF) << 7) | ((command_id & 0x3F) << 1) | (error_bit & 0x1)


def parse_bl_reply(msg):
    """Parse a can.Message as a bootloader reply, or None if it isn't one.
    Replies: ID 0x701..0x70E, DLC 5, [board_id][cmd][error][par1_lo][par1_hi]."""
    if msg is None or msg.is_remote_frame or msg.is_error_frame:
        return None
    if not (0 <= msg.arbitration_id - CANID_BL_RPL_BASE <= BL_BOARD_ID_MAX):
        return None
    if len(msg.data) < 5:
        return None
    return BlReply(board_id=msg.data[0], cmd=msg.data[1], error=msg.data[2],
                   par1=msg.data[3] | (msg.data[4] << 8))


# ---------------------------------------------------------------- bus lifecycle
def rti_is_running():
    """RTI owns the CAN bus in normal operation; two senders corrupt both."""
    return T._rti_is_running()


def read_robot_mode():
    """Read RTI's current mode straight from shared memory, or None if absent.

    Deliberately does NOT import data.shared_data — that module creates/attaches
    ~200 segments and prints a wall of text. SharedString's format is a 4-byte
    little-endian length prefix followed by UTF-8 (data/shared_data.py:55-67),
    so reading it directly is three lines and touches nothing.
    """
    try:
        shm = shared_memory.SharedMemory(name="robot_mode")
    except (FileNotFoundError, OSError):
        return None
    try:
        length = int.from_bytes(bytes(shm.buf[:4]), "little")
        if not 0 < length <= len(shm.buf) - 4:
            return None
        return bytes(shm.buf[4:4 + length]).decode("utf-8", errors="replace")
    except Exception:
        return None
    finally:
        # Detach only. Unlinking would destroy the segment for every process.
        try:
            resource_tracker.unregister(shm._name, "shared_memory")
        except Exception:
            pass
        shm.close()


def bus_available():
    """Is it safe for THIS process to transmit on can0? -> (ok, reason).

    Two supported workflows, checked in order:

    1. RUNTIME UP, RTI in FLASHING mode — the normal path. RTI has deliberately
       gone silent and granted us the bus while it keeps ticking (E-stop, IO and
       telemetry stay live). See utility/state_machine.py MAINTENANCE_MODES.
    2. RUNTIME DOWN — the recovery path. No shared memory at all, or loop_tick
       frozen. Used when RTI cannot run: a driver stuck in its bootloader after a
       failed flash may stop the runtime from coming up cleanly at all.

    Anything else means RTI is live and driving motors, so transmitting would
    interleave with its traffic and corrupt both sides.
    """
    # Liveness is checked BEFORE the mode, because shared memory OUTLIVES the
    # RTI process: after a stop, /dev/shm/robot_mode still holds whatever mode
    # was last set. Reading it first made a runtime-down run report itself as
    # grant-based, which then armed the per-page grant re-check against an RTI
    # that no longer exists. Same access decision either way, but the wrong
    # reason — and this project has been bitten before by stale shm reading as
    # live (see the park verdict in nicegui_test.py).
    if not rti_is_running():
        return True, "RTI is not running — bus is free"
    mode = read_robot_mode()
    if mode == "FLASHING":
        return True, "RTI is in FLASHING mode — bus granted"
    if mode:
        return False, (
            f"REFUSING TO TOUCH THE BUS: RTI is running in {mode} mode and owns can0.\n"
            "Put it in FLASHING mode first (GUI: Firmware tab, or "
            "RobotClient.enter_flashing_mode()),\n"
            f"or stop the runtime entirely:  sudo systemctl stop {runtime_service_name()}"
        )
    return False, stop_runtime_hint()


# --- single-flasher lock ------------------------------------------------------
# The CAN bus is a system-wide exclusive resource, but until now nothing enforced
# that. RTI's FLASHING grant stops RTI from transmitting; it does NOT stop a
# SECOND flasher from starting. Every entry point is a way in — a second browser
# tab, another machine's dashboard, an SSH session, a script, the soak harness —
# and two flashers interleaving page writes is the exact two-transmitter failure
# the whole design exists to prevent.
#
# flock, not a PID file: the kernel drops the lock when the holder dies, however
# it dies (SIGKILL, crash, power loss). A PID file survives its owner and wedges
# the next run; a flock cannot.
_bus_lock_handle = None      # module-level so the fd outlives the call and the
                             # lock is held for the process lifetime


def _bus_lock_path(channel):
    from project_paths import TEMP_DATA_DIR
    return TEMP_DATA_DIR / f".{channel}.flashlock"


def acquire_bus_lock(channel="can0"):
    """Take the exclusive flasher lock for `channel`, or raise saying who has it."""
    global _bus_lock_handle
    if _bus_lock_handle is not None:
        return
    try:
        path = _bus_lock_path(channel)
        path.parent.mkdir(parents=True, exist_ok=True)
        handle = open(path, "a+")
    except (OSError, ImportError):
        # Cannot create the lock file (read-only fs, missing config). Refusing
        # here would block the recovery path for a purely advisory guard, so
        # carry on unlocked — the operator is no worse off than before.
        return
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        handle.seek(0)
        holder = handle.read().strip() or "another process"
        handle.close()
        raise RuntimeError(
            f"REFUSING: {channel} is already being used by a flasher.\n"
            f"  {holder}\n"
            "Only one flasher may touch the bus at a time — two would interleave "
            "page writes and corrupt both.\nWait for it to finish, or stop it."
        )
    handle.seek(0)
    handle.truncate()
    handle.write(f"pid={os.getpid()} since={time.strftime('%H:%M:%S')} "
                 f"cmd={' '.join(sys.argv[1:])[:120]}\n")
    handle.flush()
    _bus_lock_handle = handle


def release_bus_lock():
    """Drop the lock. Best-effort — process exit releases it regardless."""
    global _bus_lock_handle
    if _bus_lock_handle is None:
        return
    try:
        fcntl.flock(_bus_lock_handle.fileno(), fcntl.LOCK_UN)
        _bus_lock_handle.close()
    except OSError:
        pass
    _bus_lock_handle = None


def open_bus(channel="can0", bitrate=1_000_000):
    """Open an exclusive raw python-can bus for flashing.

    Refuses while RTI is alive, refuses if another flasher already holds the
    bus, brings the interface up if it is down, and raises SO_SNDBUF as a
    backstop against ENOBUFS during stream bursts.
    """
    ok, reason = bus_available()
    if not ok:
        raise RuntimeError(reason)
    acquire_bus_lock(channel)
    if not is_can_interface_up(channel):
        start_can_interface(interface=channel, bitrate=str(bitrate))
    bus = can.interface.Bus(interface="socketcan", channel=channel)
    # raise_can_tx_buffer expects a Spectral-shaped object with .bus.socket.
    raise_can_tx_buffer(SimpleNamespace(bus=bus))
    return bus


def close_bus(bus):
    if bus is not None:
        try:
            bus.shutdown()
        except Exception:
            pass
    release_bus_lock()


def flush_rx(bus, timeout_s=0.02, max_frames=2000, max_seconds=0.5):
    """Drain stale frames so an old reply is never matched to a new command.

    BOUNDED, and that is the whole point. This was "drain until the queue is
    empty", which live-locks on a bus that is never empty: any other traffic
    arriving faster than the loop drains it means `recv()` never returns None
    and the flasher spins forever, consuming frames and making no progress.

    Found 2026-08-05 by the contention harness — with only ~480 frames/s of
    unrelated background traffic (about 6% of a 1 Mbit bus) a flash hung
    indefinitely rather than erroring out. It never reached the retry ladder,
    because it never got far enough to retry anything.

    Draining stale frames is housekeeping, so giving up is safe: a leftover
    frame cannot be mistaken for a real reply anyway — `_wait_reply` matches on
    (board_id, cmd, par1-echo). The cap only has to stop the spin.
    """
    deadline = time.monotonic() + max_seconds
    for _ in range(max_frames):
        if bus.recv(timeout_s) is None:
            return
        if time.monotonic() >= deadline:
            return


def send_frame(bus, msg, timeout_s=1.0):
    """bus.send with one retry on a full TX queue (ENOBUFS backstop)."""
    try:
        bus.send(msg, timeout_s)
    except can.CanOperationError:
        time.sleep(0.02)
        bus.send(msg, timeout_s)


def send_frame_best_effort(bus, msg):
    """Send a DISCOVERY ping, tolerating a bus with nobody left on it.

    Returns True if the frame went out, False if the controller could not place
    it. CAN needs at least one other node to acknowledge a frame; an active scan
    has just rebooted every board, so for the ~8 s they take to reach their
    bootloaders the Pi is the ONLY node present. The controller then retries the
    unacknowledged frame forever, the socketcan queue fills, and `send()` raises
    ENOBUFS ("Transmit buffer full").

    That is the expected physics of an empty bus, not a fault, and it must not
    abort the scan: RX keeps working throughout, and the boards start
    acknowledging again the moment their bootloaders come up. Observed on the
    bench rig 2026-08-04 — the controller went ERROR-PASSIVE twice and recovered
    on its own both times, never reaching bus-off.

    NEVER use this on the flash path. There a dropped frame corrupts a page and
    `send()` must raise.
    """
    try:
        bus.send(msg, 0.05)
        return True
    except can.CanOperationError:
        return False


def _check_cancel(cancel):
    if cancel is not None and cancel.is_set():
        raise FlashCancelled("Flash cancelled")


def _check_grant_still_held(was_granted_by_mode):
    """Abort if RTI took the bus back mid-flash.

    Only meaningful when we started under a FLASHING-mode grant. RTI can leave
    that mode at any time — E-stop, a safety stop, an operator clicking Exit —
    and the instant it does it resumes transmitting. Two transmitters is the one
    failure this whole design exists to avoid, so we stop rather than fight for
    the bus. Costs one shared-memory string read per page.

    Stopping here is safe: the board keeps an invalid app CRC and waits in its
    bootloader, which `scan --no-reset` finds and a re-flash fixes.
    """
    if not was_granted_by_mode:
        return
    mode = read_robot_mode()
    if mode != "FLASHING":
        raise FlashCancelled(
            f"RTI left FLASHING mode (now {mode or 'unknown'}) and has taken the CAN "
            "bus back. Stopped mid-flash — the board is waiting in its bootloader; "
            "re-enter FLASHING mode and flash again.")


# ---------------------------------------------------------------- command / reply
def _wait_reply(bus, board_id, cmd, par1, timeout_s):
    """Return the error code of a matching reply, or None on timeout.
    Matches on (board_id, cmd, par1-echo) so a late reply to a previous
    command is never mistaken for this one's."""
    deadline = time.monotonic() + timeout_s
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        reply = parse_bl_reply(bus.recv(min(remaining, 0.05)))
        if (reply is not None and reply.board_id == board_id
                and reply.cmd == cmd and reply.par1 == (par1 & 0xFFFF)):
            return reply.error


def bl_cmd_response(bus, board_id, cmd, par1=0, par2=0, *,
                    timeout_s=CMD_TIMEOUT_S, retries=10, cancel=None):
    """Send a command and require an OK reply. Resends on timeout; raises
    immediately on an error reply (the board answered — retrying the same
    thing would just repeat the error)."""
    for _ in range(retries):
        _check_cancel(cancel)
        time.sleep(COMMAND_GAP_S)
        send_frame(bus, bl_cmd_frame(board_id, cmd, par1, par2))
        error = _wait_reply(bus, board_id, cmd, par1, timeout_s)
        if error is None:
            continue
        if error != 0:
            name = BL_ERROR_NAMES.get(error, f"unknown({error})")
            raise BootloaderError(
                f"Bootloader cmd 0x{cmd:02X} returned {name} (par1={par1})")
        return
    raise BootloaderError(
        f"No reply from board {board_id} for cmd 0x{cmd:02X} after {retries} tries")


def ping_bootloader(bus, board_id, timeout_s=PING_TIMEOUT_S):
    send_frame(bus, bl_cmd_frame(board_id, BL_PING))
    return _wait_reply(bus, board_id, BL_PING, 0, timeout_s) == 0


def wait_for_bootloader(bus, board_id, retries, timeout_s=PING_TIMEOUT_S, cancel=None):
    for _ in range(retries):
        _check_cancel(cancel)
        if ping_bootloader(bus, board_id, timeout_s):
            return True
    return False


def send_app_reset(bus, node_id):
    """Kick a running application into the bootloader.

    Sends the Spectral IN_RESET(14) command — byte-identical to
    SpectralCAN.Send_Reset() — three rounds in four variants (data/RTR x
    error-bit) so it lands regardless of the app's parser state.
    """
    reset_id = app_can_id(node_id, APP_IN_RESET_CMD, 0)
    reset_id_err = app_can_id(node_id, APP_IN_RESET_CMD, 1)
    for _ in range(3):
        for arb_id in (reset_id, reset_id_err):
            send_frame(bus, can.Message(arbitration_id=arb_id,
                                        is_extended_id=False, data=b""))
            send_frame(bus, can.Message(arbitration_id=arb_id,
                                        is_extended_id=False,
                                        is_remote_frame=True, dlc=0))
        time.sleep(0.02)


def scan_bootloader_boards(bus, *, reset_apps=True, listen_s=None,
                           on_log=None, cancel=None):
    """Find every board currently in (or coaxable into) bootloader mode.

    reset_apps=True: broadcast app-reset to nodes 0..15 first — REBOOTS every
    motor on the bus (each comes back to its app ~3 s after the bus goes
    quiet). reset_apps=False is the passive scan: only finds boards already
    sitting in the bootloader, e.g. after an interrupted flash.

    listen_s defaults to whichever window the mode actually needs — see
    SCAN_LISTEN_ACTIVE_S / SCAN_LISTEN_PASSIVE_S. Do not shorten the active one
    below the measured app-reset -> bootloader delay or the scan reports an
    empty bus on perfectly healthy boards.
    """
    if listen_s is None:
        listen_s = SCAN_LISTEN_ACTIVE_S if reset_apps else SCAN_LISTEN_PASSIVE_S
    log = on_log or (lambda line: None)
    if reset_apps:
        log("Broadcasting app reset to nodes 0..15 (all motors reboot)...")
        for node in range(APP_NODE_ID_MAX + 1):
            _check_cancel(cancel)
            try:
                send_app_reset(bus, node)
            except can.CanOperationError:
                # Boards drop off progressively as they take the reset, so the
                # later frames in this sweep can already be unacknowledged.
                pass
            time.sleep(0.01)
        time.sleep(0.15)
    flush_rx(bus)
    found = set()
    deadline = time.monotonic() + listen_s
    next_ping = 0.0
    announced_empty = False
    while time.monotonic() < deadline:
        _check_cancel(cancel)
        if time.monotonic() >= next_ping:
            delivered = 0
            for board_id in range(BL_BOARD_ID_MAX + 1):
                if send_frame_best_effort(bus, bl_cmd_frame(board_id, BL_PING)):
                    delivered += 1
            if delivered:
                next_ping = time.monotonic() + 0.2
            else:
                # Nobody is acknowledging yet. Ping slower so the TX queue can
                # drain the moment a bootloader appears, and keep listening —
                # this is the normal state for most of an active scan.
                if not announced_empty:
                    log("  (no node acknowledging yet — boards still rebooting; "
                        "still listening)")
                    announced_empty = True
                next_ping = time.monotonic() + 0.5
        reply = parse_bl_reply(bus.recv(0.02))
        if reply is not None and reply.cmd == BL_PING and reply.error == 0:
            if reply.board_id not in found:
                log(f"  bootloader board {reply.board_id} "
                    f"({NODE_ROLES.get(reply.board_id, 'unexpected')})")
            found.add(reply.board_id)
    return sorted(found)


def catch_boot_window(bus, listen_s=60.0, on_log=None):
    """Ping continuously and report the instant a bootloader answers.

    For boards whose APPLICATION does not implement the bootloader handshake.
    The reference integration requires the app to zero the SRAM magic at
    0x20001000 before resetting (docs/INTEGRATION.md: "ensure bootloader doesn't
    fast-path past itself"). An app that just calls NVIC_SystemReset() reboots
    into a bootloader that sees a VALID magic and jumps straight back to the
    application in ~10 ms — so a software reset can never reach it.

    A cold power-cycle leaves the magic uninitialised, so the bootloader opens
    its full startup window (~2 s). This holds the bus and pings hard so that
    window cannot be missed. Power-cycle the board while it runs.
    """
    log = on_log or (lambda line: None)
    found = {}
    flush_rx(bus)
    deadline = time.monotonic() + listen_s
    started = time.monotonic()
    while time.monotonic() < deadline:
        for board_id in range(BL_BOARD_ID_MAX + 1):
            # Best-effort by necessity: this command exists to be run WHILE the
            # operator power-cycles boards, so an empty, unacknowledging bus is
            # the normal condition here, not the exception.
            send_frame_best_effort(bus, bl_cmd_frame(board_id, BL_PING))
        window = time.monotonic() + 0.05
        while time.monotonic() < window:
            reply = parse_bl_reply(bus.recv(0.01))
            if reply is not None and reply.cmd == BL_PING and reply.error == 0:
                if reply.board_id not in found:
                    found[reply.board_id] = round(time.monotonic() - started, 2)
                    log(f"  BOOTLOADER FOUND: board id {reply.board_id} "
                        f"at t+{found[reply.board_id]}s")
    return found


def set_board_id(bus, current_id, new_id):
    """Change a bootloader's board ID (persisted to its config page).
    Port of can_flash.py change_id — the ack arrives from the NEW id."""
    for board_id, label in ((current_id, "current"), (new_id, "new")):
        if not 0 <= board_id <= BL_BOARD_ID_MAX:
            raise ValueError(f"Invalid {label} board ID {board_id} — must be "
                             f"0..{BL_BOARD_ID_MAX} (14/15 are reserved)")
    if not wait_for_bootloader(bus, current_id, PING_RETRIES_NO_RESET):
        raise BootloaderError(f"Could not connect to bootloader board {current_id}")
    flush_rx(bus)
    send_frame(bus, bl_cmd_frame(current_id, BL_SET_ID, new_id))
    error = _wait_reply(bus, new_id, BL_SET_ID, new_id, 1.0)
    if error is None:
        raise BootloaderError("No SET_ID acknowledgement from the new board ID")
    if error != 0:
        name = BL_ERROR_NAMES.get(error, f"unknown({error})")
        raise BootloaderError(f"SET_ID returned {name}")


# ---------------------------------------------------------------- image validation
@dataclass
class ValidationResult:
    ok: bool
    errors: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    size: int = 0
    padded_size: int = 0
    pages: int = 0
    stack_pointer: int = None
    reset_vector: int = None


def validate_firmware_bytes(data, app_start=APP_BASE_ADDRESS,
                            flash_end=FLASH_END_ADDRESS,
                            ram_start=RAM_START_ADDRESS,
                            ram_end=RAM_END_ADDRESS):
    """Pre-flight image checks, on bytes so uploads can be validated in memory.
    Catches the one mistake that needs an SWD probe to undo: an image linked at
    0x08000000 instead of app_start."""
    errors, warnings = [], []
    size = len(data)
    max_size = flash_end - app_start
    padded_size = size + (PAGE_SIZE - size % PAGE_SIZE if size % PAGE_SIZE else 0)
    pages = padded_size // PAGE_SIZE

    if size == 0:
        return ValidationResult(False, ["File is empty."], [], 0, 0, 0, None, None)
    if size < 8:
        return ValidationResult(False, ["File is too short to contain a vector table."],
                                [], size, padded_size, pages, None, None)
    if size > max_size:
        errors.append(f"File is {size} bytes, but app flash space is only {max_size} bytes.")
    if padded_size > max_size:
        errors.append(f"File pads to {padded_size} bytes ({pages} pages), "
                      "exceeding app flash space.")

    stack_pointer = int.from_bytes(data[0:4], "little")
    reset_vector = int.from_bytes(data[4:8], "little")
    reset_address = reset_vector & ~1

    if stack_pointer in (0x00000000, 0xFFFFFFFF):
        errors.append("Initial stack pointer is blank/invalid.")
    elif not ram_start <= stack_pointer <= ram_end:
        errors.append(f"Initial stack pointer 0x{stack_pointer:08X} is outside RAM "
                      f"0x{ram_start:08X}-0x{ram_end:08X}.")
    elif stack_pointer % 4 != 0:
        warnings.append(f"Initial stack pointer 0x{stack_pointer:08X} is not 4-byte aligned.")

    if reset_vector in (0x00000000, 0xFFFFFFFF):
        errors.append("Reset vector is blank/invalid.")
    else:
        if (reset_vector & 1) == 0:
            errors.append(f"Reset vector 0x{reset_vector:08X} does not have the Thumb bit set.")
        if not app_start <= reset_address < flash_end:
            errors.append(f"Reset address 0x{reset_address:08X} is outside app flash "
                          f"0x{app_start:08X}-0x{flash_end - 1:08X}.")
        elif reset_address < app_start + 0x100:
            warnings.append(f"Reset address 0x{reset_address:08X} is very close to app "
                            "start; this can be valid, but double-check the linker script.")

    if 0x08000000 <= reset_address < app_start:
        errors.append("This looks like firmware linked for 0x08000000, not the "
                      f"bootloader app start 0x{app_start:08X}.")

    return ValidationResult(ok=not errors, errors=errors, warnings=warnings,
                            size=size, padded_size=padded_size, pages=pages,
                            stack_pointer=stack_pointer, reset_vector=reset_vector)


def format_validation(result):
    lines = [f"Size:    {result.size} bytes",
             f"Padded:  {result.padded_size} bytes ({result.pages} page(s))"]
    if result.stack_pointer is not None:
        lines.append(f"SP:      0x{result.stack_pointer:08X}")
    if result.reset_vector is not None:
        lines.append(f"Reset:   0x{result.reset_vector:08X}")
    lines.extend(f"WARNING: {w}" for w in result.warnings)
    lines.extend(f"ERROR:   {e}" for e in result.errors)
    lines.append("Result:  PASS" if result.ok else "Result:  FAIL")
    return "\n".join(lines)


# ---------------------------------------------------------------- flash session
def _progress(cb, stage, fraction, page, total_pages, message):
    if cb is not None:
        cb({"stage": stage, "fraction": fraction, "page": page,
            "total_pages": total_pages, "message": message})


def new_stats():
    """Counters for one flash session. Retries are the MARGIN — a run that
    succeeds with zero retries and one that barely scrapes through after 40 both
    report success, and only the retry count tells them apart. The soak test
    lives on this (tools/soak_flash_test.py)."""
    return {"pages": 0, "page_retries": 0, "chunk_retries": 0}


def _bump(stats, key):
    if stats is not None:
        stats[key] = stats.get(key, 0) + 1


def _send_stream_chunk(bus, board_id, page_bytes, start_seq, end_seq, frame_gap_s,
                       stats=None):
    """Blast one chunk of stream frames, then verify via STREAM_STATUS.
    A failed verify (timeout or INVALID_CRC = incomplete bitmask) resends the
    whole chunk — the frames carry no ack, so this is the loss recovery."""
    for attempt in range(STREAM_CHUNK_RETRIES):
        if attempt:
            _bump(stats, "chunk_retries")
        for seq in range(start_seq, end_seq + 1):
            offset = seq * 8
            send_frame(bus, stream_data_frame(board_id, seq,
                                              page_bytes[offset:offset + 8]))
            if frame_gap_s:
                time.sleep(frame_gap_s)
        try:
            bl_cmd_response(bus, board_id, BL_STREAM_STATUS, end_seq,
                            timeout_s=STREAM_STATUS_TIMEOUT_S, retries=1)
            return
        except BootloaderError:
            time.sleep(0.01)
    raise BootloaderError(f"Stream chunk ending at seq {end_seq} did not verify "
                          f"after {STREAM_CHUNK_RETRIES} attempts")


def _flash_page(bus, board_id, page_num, page_bytes, page_crc,
                chunk_frames, frame_gap_s, cancel, stats=None):
    last_error = None
    for attempt in range(PAGE_RETRIES):
        _check_cancel(cancel)
        if attempt:
            _bump(stats, "page_retries")
        try:
            flush_rx(bus)
            bl_cmd_response(bus, board_id, BL_STREAM_BEGIN, page_num,
                            timeout_s=STREAM_BEGIN_TIMEOUT_S, retries=1, cancel=cancel)
            for chunk_start in range(0, CAN_STREAM_FRAME_COUNT, chunk_frames):
                _check_cancel(cancel)
                end_seq = min(chunk_start + chunk_frames - 1,
                              CAN_STREAM_FRAME_COUNT - 1)
                _send_stream_chunk(bus, board_id, page_bytes,
                                   chunk_start, end_seq, frame_gap_s, stats)
            bl_cmd_response(bus, board_id, BL_WPAGE, page_num, page_crc,
                            timeout_s=WPAGE_TIMEOUT_S, retries=1, cancel=cancel)
            _bump(stats, "pages")
            return
        except BootloaderError as error:
            last_error = error
            time.sleep(0.05)
    raise BootloaderError(f"Page {page_num}: {last_error}")


def flash_firmware(bus, board_id, firmware, *, erase_first=True,
                   app_reset_node_id=None, skip_app_reset=False,
                   stream_chunk_frames=DEFAULT_STREAM_CHUNK_FRAMES,
                   stream_frame_gap_s=0.0, on_progress=None, on_log=None,
                   cancel=None, stats=None):
    """Full flash session against one board. Port of flashFirmwareOverCan.

    on_progress gets dicts {stage, fraction, page, total_pages, message} with
    stages pinging | app-resetting | erasing | flashing | finalising | done.
    on_log gets human-readable lines. cancel is a threading.Event checked
    between pings/chunks/pages; raises FlashCancelled when set.
    Raises BootloaderError on protocol failure, ValueError on bad inputs.
    """
    log = on_log or (lambda line: None)
    if not 0 <= board_id <= BL_BOARD_ID_MAX:
        raise ValueError(f"Invalid board ID {board_id} — must be 0..{BL_BOARD_ID_MAX}")

    # If we hold the bus by RTI's grant rather than because the runtime is down,
    # re-check that grant as we go — see _check_grant_still_held.
    _granted_by_mode = (read_robot_mode() == "FLASHING")
    if _granted_by_mode:
        log("Bus granted by RTI FLASHING mode (runtime stays live).")

    padded = pad_to(firmware, PAGE_SIZE)
    total_pages = len(padded) // PAGE_SIZE
    if total_pages > MAX_APP_PAGES:
        raise ValueError(f"Firmware is {total_pages} pages — the bootloader can "
                         f"only host {MAX_APP_PAGES} ({MAX_APP_PAGES} KB)")
    app_crc = stm32_crc32(padded)
    log(f"Firmware: {len(firmware)} bytes -> padded to {len(padded)} bytes "
        f"({total_pages} page(s)). App CRC = 0x{app_crc:08X}")

    # ---- find or wake the bootloader
    _progress(on_progress, "pinging", 0.0, None, total_pages, "Pinging bootloader...")
    log(f"Pinging bootloader at board ID {board_id}...")
    pinged = wait_for_bootloader(bus, board_id, PING_RETRIES_NO_RESET,
                                 PING_TIMEOUT_S, cancel)
    if not pinged and not skip_app_reset:
        reset_target = board_id if app_reset_node_id is None else app_reset_node_id
        _progress(on_progress, "app-resetting", 0.02, None, total_pages,
                  f"Sending app reset to node {reset_target}...")
        log(f"No bootloader reply. Sending app reset to node {reset_target}.")
        send_app_reset(bus, reset_target)
        time.sleep(APP_RESET_GAP_S)
        pinged = wait_for_bootloader(bus, board_id, PING_RETRIES_AFTER_RESET,
                                     0.1, cancel)
    if not pinged:
        raise BootloaderError(
            f"Could not reach bootloader at board {board_id}. Try power-cycling "
            "the board and catching its 2 s startup window (scan --no-reset).")
    log(f"Connected to bootloader board {board_id}.")

    # ---- optional erase (idempotent, so a lost ack is safely retried once)
    if erase_first:
        _check_cancel(cancel)
        _progress(on_progress, "erasing", 0.04, None, total_pages,
                  "Erasing application flash (~3.5 s)...")
        log("Erasing application flash area...")
        bl_cmd_response(bus, board_id, BL_ERASE_APP,
                        timeout_s=ERASE_TIMEOUT_S, retries=2, cancel=cancel)
        log("Erase complete.")

    # ---- stream every page
    stage_start = 0.08 if erase_first else 0.04
    stage_end = 0.96
    for page_num in range(total_pages):
        _check_cancel(cancel)
        _check_grant_still_held(_granted_by_mode)
        page_bytes = padded[page_num * PAGE_SIZE:(page_num + 1) * PAGE_SIZE]
        page_crc = stm32_crc32(page_bytes)
        _progress(on_progress, "flashing",
                  stage_start + (page_num / total_pages) * (stage_end - stage_start),
                  page_num, total_pages,
                  f"Flashing page {page_num + 1}/{total_pages}...")
        _flash_page(bus, board_id, page_num, page_bytes, page_crc,
                    stream_chunk_frames, stream_frame_gap_s, cancel, stats)
        if page_num % 8 == 0 or page_num == total_pages - 1:
            log(f"Page {page_num + 1}/{total_pages} OK (CRC 0x{page_crc:08X})")

    # ---- commit the whole-app CRC; the board reboots on its own after silence
    _check_cancel(cancel)
    _progress(on_progress, "finalising", 0.98, None, total_pages,
              "Committing app CRC...")
    log("Verifying whole-app CRC and committing config page...")
    bl_cmd_response(bus, board_id, BL_WCRC, total_pages, app_crc,
                    timeout_s=WCRC_TIMEOUT_S, retries=3, cancel=cancel)
    log("Done. The board validates the CRC and boots the new application "
        "within ~3 s of bus silence.")
    _progress(on_progress, "done", 1.0, None, total_pages, "Flash complete")


# ---------------------------------------------------------------- app device info
class SpectralBusAdapter:
    """Duck-types Spectral's communication object over the raw flashing bus,
    so device info can be read on the same socket (port of can_fleet_update)."""

    def __init__(self, bus):
        self.bus = bus

    def send_can_message(self, message_id, data=b"", remote_frame=False):
        payload = bytes(data or b"")
        send_frame(self.bus, can.Message(arbitration_id=message_id, data=payload,
                                         is_remote_frame=remote_frame,
                                         is_extended_id=False, dlc=len(payload)))

    def receive_can_messages(self, timeout=None):
        message = self.bus.recv(timeout=timeout)
        if message is None:
            return None, None
        node_id, command_id, error_bit = spectral_can_utils.extract_from_can_id(
            message.arbitration_id)
        return message, SimpleNamespace(node_id=node_id, command_id=command_id,
                                        error_bit=error_bit)


def read_device_info(bus, node_id, timeout_s=0.6):
    """Ask a running APPLICATION (not bootloader) for its identity. Returns
    {"software_version", "hardware_version", "serial_number", "batch_date"}
    or None if the node did not answer — e.g. it is still in the bootloader."""
    adapter = SpectralBusAdapter(bus)
    motor = Spectral.SpectralCAN(node_id=node_id, communication=adapter)
    flush_rx(bus)
    motor.Send_Respond_Device_Info()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        message, unpacked = adapter.receive_can_messages(timeout=min(remaining, 0.05))
        if message is None or message.is_remote_frame or unpacked.node_id != node_id:
            continue
        try:
            motor.UnpackData(message, unpacked)
        except Exception:
            continue
        if motor.SOFTWARE_VERSION is not None:
            return {"software_version": motor.SOFTWARE_VERSION,
                    "hardware_version": motor.HARDWARE_VERSION,
                    "serial_number": motor.SERIAL_NUMBER,
                    "batch_date": motor.BATCH_DATE}
    return None


# ================================================================ CLI
EXIT_OK = 0
EXIT_FLASH_ERROR = 1
EXIT_PREFLIGHT = 2
EXIT_VALIDATION = 3
EXIT_UNREACHABLE = 4
EXIT_SOURCE = 5

SAFETY_WARNING = (
    "SAFETY: flashing REBOOTS motor drivers — they lose torque while doing it.\n"
    "The arm must be resting in a safe pose (it will go limp), and after the\n"
    "runtime restarts the robot MUST be re-homed."
)


def _preflight_or_exit():
    ok, reason = bus_available()
    if not ok:
        print(reason, file=sys.stderr)
        raise SystemExit(EXIT_PREFLIGHT)
    print(f"  {reason}")


def _confirm_or_exit(args, action):
    if getattr(args, "yes", False):
        return
    print(SAFETY_WARNING)
    try:
        answer = input(f"Type 'yes' to {action}: ").strip().lower()
    except EOFError:
        # No stdin at all. This is a legitimate way to run everything up to the
        # confirmation and stop — the dashboard's whole-robot "Check" does
        # exactly that, so the preflight it shows is the real one rather than a
        # second implementation that could drift from it. Treat it as a decline,
        # not a crash: an unanswered safety prompt is a "no".
        print("\nNo input available — treating as 'no'. Aborted.")
        raise SystemExit(EXIT_PREFLIGHT)
    if answer != "yes":
        print("Aborted.")
        raise SystemExit(EXIT_PREFLIGHT)


def _node_label(node_id):
    role = NODE_ROLES.get(node_id)
    return f"{node_id} ({role})" if role else str(node_id)


# --- machine-readable output -------------------------------------------------
# With --progress-json every event is one JSON object per line on stdout, so a
# GUI can spawn this as a subprocess and just render the stream. That keeps ONE
# flashing implementation: the thing the dashboard runs is the thing you test
# from a terminal, and the dashboard never has to open a CAN socket itself.
#
#   {"type": "log",      "message": str}
#   {"type": "progress", "stage": str, "fraction": float,
#                        "page": int|null, "total_pages": int|null, "message": str}
#   {"type": "result",   "ok": bool, "exit_code": int, "message": str}
#   {"type": "boards",   "boards": [int]}          — scan only
_json_mode = False
_json_stream = None   # the REAL stdout, captured before it is redirected


def _emit(obj):
    print(json.dumps(obj), file=_json_stream or sys.stdout, flush=True)


def _out_log(line):
    """Log sink — JSON line or plain text, depending on --progress-json."""
    if _json_mode:
        _emit({"type": "log", "message": str(line)})
    else:
        print(line)


def _out_progress(update):
    if _json_mode:
        _emit({"type": "progress", **update})


def _out_boards(boards):
    """Structured scan result, so a GUI renders chips instead of parsing prose.

    Additive to the contract above: a consumer that only knows log/progress/
    result ignores this line, which is why it is safe to emit unconditionally.
    """
    if _json_mode:
        _emit({"type": "boards", "boards": [int(b) for b in boards]})


_EXIT_MEANINGS = {
    EXIT_OK:          "completed successfully",
    EXIT_FLASH_ERROR: "flash failed",
    EXIT_PREFLIGHT:   "refused before touching the bus",
    EXIT_VALIDATION:  "firmware image failed validation",
    EXIT_UNREACHABLE: "bootloader did not answer",
    EXIT_SOURCE:      "could not obtain the firmware image",
}

_result_emitted = False


def _out_result(ok, exit_code, message):
    """Terminal outcome. Exactly one of these is emitted per run in JSON mode —
    main() backstops any exit path that forgot, so a caller can always rely on
    a result line rather than having to interpret a bare exit code."""
    global _result_emitted
    if _json_mode and not _result_emitted:
        _result_emitted = True
        _emit({"type": "result", "ok": bool(ok), "exit_code": int(exit_code),
               "message": str(message)})


# --- product matching --------------------------------------------------------
# A board in bootloader mode cannot say which product it is — every driver
# reports the same hardware_version, so flashing StepFOC firmware onto a Spectral
# BLDC board is invisible to the protocol. The ROBOT knows, though: each joint
# declares <driver_type> in robots/<robot>.xml and each CAN gripper in
# grippers/<gripper>.xml, using the same ids as FIRMWARE_PRODUCTS. That turns an
# unverifiable confirmation into an actual check.
#
# It is a strong guard, not ground truth: the XML records what the builder
# BELIEVES is installed, so swapping a board without updating it makes the check
# confidently wrong. Hence --force-product.
def expected_driver_types():
    """node id -> declared firmware product. Empty dict if config is unreadable.

    Must never raise: this tool has to keep working on a box where the runtime
    config is missing or broken (that is often exactly why you are flashing).
    """
    try:
        from config.xml_parser import load_full_robot_config, load_system_config
        from project_paths import SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR
        sysconf = load_system_config(SYSTEM_XML, ROBOTS_DIR)
        cfg = load_full_robot_config(sysconf["robot_xml_path"], GRIPPERS_DIR,
                                     sysconf["active_gripper"])
    except Exception:
        return {}
    mapping = {}
    for index, declared in enumerate(cfg.get("motor_driver_type") or []):
        if declared:
            mapping[index] = declared
    gripper = cfg.get("gripper_driver_type")
    if gripper:
        mapping[int(cfg.get("joint_num") or len(mapping))] = gripper
    return mapping


def infer_product_from_filename(path):
    """Best-effort product id from a .bin's name, or None if not certain.

    A raw .bin carries no product marker, so the filename is the only hint when
    --product was not given. Ambiguous or absent -> None, which degrades to
    "cannot verify" rather than to a guess.
    """
    try:
        import firmware_github
        known = list(firmware_github.FIRMWARE_PRODUCTS)
    except Exception:
        known = ["stepfoc", "spectral-bldc"]
    name = Path(path).name.lower()
    hits = [product for product in known if product in name]
    return hits[0] if len(hits) == 1 else None


def check_product_match(node_id, product, force=False):
    """-> (allowed, message). Refuses only when BOTH sides are known and differ."""
    expected = expected_driver_types().get(node_id)
    label = _node_label(node_id)
    if expected is None:
        return True, (f"Node {label}: robot XML declares no driver_type — cannot "
                      "verify this image matches the board.")
    if product is None:
        return True, (f"Node {label}: expects '{expected}', but the image's product "
                      "is unknown (pass --product, or name the file after it) — "
                      "cannot verify.")
    if product == expected:
        return True, f"Node {label}: expects '{expected}', image is '{product}' - match."
    if force:
        return True, (f"OVERRIDE: node {label} is declared '{expected}' but this "
                      f"image is '{product}'. Flashing anyway (--force-product).")
    return False, (
        f"REFUSING: node {label} is declared '{expected}' in the robot XML, but this "
        f"image is '{product}'.\n"
        "Flashing the wrong product leaves a board running firmware for different "
        "hardware.\n"
        "If the XML is out of date for this board, re-run with --force-product.")


def _require_board_id(node_id):
    """Physical devices only ever sit on 0..13 — 14/15 are the host's."""
    if not 0 <= node_id <= BL_BOARD_ID_MAX:
        print(f"ERROR: node {node_id} is not a flashable device — board IDs are "
              f"0..{BL_BOARD_ID_MAX} (14/15 reserved for the host)", file=sys.stderr)
        raise SystemExit(EXIT_PREFLIGHT)


def _resolve_firmware_or_exit(args):
    """Return (firmware_bytes, description, app_start, product) from --file/--product.

    `product` is what the IMAGE is believed to be — declared by --product, else
    inferred from the filename, else None ("unknown"). It is compared against
    what the robot XML says each node should have.
    """
    app_start = APP_BASE_ADDRESS
    if args.file is not None:
        path = Path(args.file)
        if not path.is_file():
            print(f"ERROR: no such file: {path}", file=sys.stderr)
            raise SystemExit(EXIT_SOURCE)
        product = args.product or infer_product_from_filename(path)
        return path.read_bytes(), str(path), app_start, product
    if args.product is not None:
        import firmware_github
        try:
            downloaded = firmware_github.download_firmware(
                args.product, tag=args.version, on_log=print)
        except firmware_github.FirmwareFetchError as error:
            print(f"ERROR: {error}", file=sys.stderr)
            raise SystemExit(EXIT_SOURCE)
        manifest = downloaded["manifest"]
        base = firmware_github.manifest_app_base_address(manifest)
        if base is not None:
            app_start = base
        target = str(manifest.get("target", ""))
        if target and "stm32f103" not in target.lower():
            print(f"WARNING: manifest target is '{target}', not an stm32f103 — "
                  "make sure this image matches the driver.")
        description = f"{args.product} {downloaded['tag']} ({downloaded['path'].name})"
        return downloaded["path"].read_bytes(), description, app_start, args.product
    print("ERROR: provide a firmware source: --file FW.bin or "
          "--product NAME [--latest | --version TAG]", file=sys.stderr)
    raise SystemExit(EXIT_SOURCE)


def _validate_or_exit(args, firmware, app_start):
    result = validate_firmware_bytes(firmware, app_start=app_start)
    print(format_validation(result))
    if result.ok:
        return
    if not getattr(args, "skip_validate", False):
        print("Refusing to flash an image that failed validation "
              "(override with --skip-validate --yes).", file=sys.stderr)
        raise SystemExit(EXIT_VALIDATION)
    if not getattr(args, "yes", False):
        print("--skip-validate additionally requires --yes.", file=sys.stderr)
        raise SystemExit(EXIT_VALIDATION)
    print("WARNING: flashing DESPITE failed validation (--skip-validate).")


def mark_flash_performed(node_id):
    """Record that a driver was reflashed, so RTI can invalidate homing.

    Flashing REBOOTS the driver, and the encoders live on the motors — which is
    the whole reason this arm needs homing at all. The joint's position
    reference is therefore gone, but `joint_homed` stays 1 because RTI never
    restarted: FLASHING mode's headline advantage (the loop keeps running) is
    exactly what stops the usual invalidation from happening. Without this the
    robot claims a reference it does not have, and the next absolute move is
    computed against a bogus zero.

    Best-effort by design: if the marker cannot be written the flash itself
    still succeeded, and the operator is told to re-home in the CLI output.
    """
    try:
        # Deferred like the rest of this file's repo imports: the flasher has to
        # keep working on a box where the runtime config is missing or broken.
        from project_paths import FLASH_PERFORMED_MARKER
        FLASH_PERFORMED_MARKER.parent.mkdir(parents=True, exist_ok=True)
        FLASH_PERFORMED_MARKER.write_text(
            f"{time.time():.0f} node={node_id}\n")
        return True
    except (OSError, ImportError) as error:
        _out_log(f"WARNING: could not record the flash for RTI ({error}). "
                 f"RE-HOME THE ROBOT MANUALLY before moving it.")
        return False


def _flash_one_node(bus, args, node_id, firmware):
    start = time.perf_counter()
    try:
        flash_firmware(
            bus, node_id, firmware,
            erase_first=not args.no_erase,
            app_reset_node_id=args.app_node,
            skip_app_reset=args.skip_app_reset,
            stream_chunk_frames=args.chunk_frames,
            stream_frame_gap_s=args.frame_gap,
            on_log=_out_log,
            on_progress=_out_progress,
        )
    except FlashCancelled as error:
        _out_log(f"CANCELLED: {error}")
        _out_result(False, EXIT_FLASH_ERROR, str(error))
        if not _json_mode:
            print(str(error), file=sys.stderr)
        raise SystemExit(EXIT_FLASH_ERROR)
    except BootloaderError as error:
        message = f"ERROR flashing node {_node_label(node_id)}: {error}"
        code = (EXIT_UNREACHABLE if "Could not reach bootloader" in str(error)
                else EXIT_FLASH_ERROR)
        _out_result(False, code, message)
        # In JSON mode the result line above carries it; in text mode print once,
        # to stderr. (_out_log would have printed a second copy to stdout.)
        if not _json_mode:
            print(message, file=sys.stderr)
        raise SystemExit(code)
    mark_flash_performed(node_id)
    _out_log(f"Node {_node_label(node_id)} flashed in "
             f"{time.perf_counter() - start:.1f} s. "
             f"HOMING IS NOW INVALID — the driver rebooted and lost its "
             f"position reference; re-home before moving the arm.")


def _post_flash_reminder():
    print(f"\nNext steps:  sudo systemctl start {runtime_service_name()}"
          "   then RE-HOME the robot")
    print("(a board that failed mid-flash stays in the bootloader — "
          "`scan --no-reset` finds it; just flash again)")


def cmd_scan(args):
    _preflight_or_exit()
    if args.catch_boot:
        listen_s = 60.0 if args.listen is None else args.listen
        bus = open_bus(args.channel)
        try:
            _out_log(f"Listening for {listen_s:.0f}s — POWER-CYCLE THE BOARD(S) NOW.")
            _out_log("(a software reset cannot reach a bootloader whose app does not "
                     "clear the SRAM magic; a cold boot can)")
            boards = sorted(catch_boot_window(bus, listen_s=listen_s, on_log=_out_log))
        finally:
            close_bus(bus)
        _out_boards(boards)
        if boards:
            _out_log("Bootloaders caught: " + ", ".join(_node_label(b) for b in boards))
            return EXIT_OK
        if _json_mode:
            _out_log("No bootloader answered during the window.")
        else:
            print("No bootloader answered during the window.", file=sys.stderr)
        return EXIT_UNREACHABLE

    if not args.no_reset:
        _confirm_or_exit(args, "scan (REBOOTS every motor on the bus)")
    bus = open_bus(args.channel)
    try:
        boards = scan_bootloader_boards(bus, reset_apps=not args.no_reset,
                                        listen_s=args.listen, on_log=_out_log)
    finally:
        close_bus(bus)
    _out_boards(boards)
    if boards:
        _out_log("Bootloaders found: " + ", ".join(_node_label(b) for b in boards))
        if not args.no_reset:
            _out_log("(each board reboots back into its application ~3 s after "
                     "the bus goes quiet)")
    else:
        _out_log("No bootloaders answered." +
                 ("" if args.no_reset else " Are any boards powered?"))
    return EXIT_OK


def cmd_ping(args):
    _preflight_or_exit()
    _require_board_id(args.node)
    bus = open_bus(args.channel)
    try:
        alive = ping_bootloader(bus, args.node)
    finally:
        close_bus(bus)
    print(f"Bootloader board {_node_label(args.node)}: "
          f"{'ALIVE' if alive else 'no reply'}")
    return EXIT_OK if alive else EXIT_UNREACHABLE


def cmd_app_reset(args):
    _preflight_or_exit()
    _require_board_id(args.node)
    _confirm_or_exit(args, f"reset node {_node_label(args.node)} into its bootloader")
    bus = open_bus(args.channel)
    try:
        send_app_reset(bus, args.node)
        time.sleep(APP_RESET_GAP_S)
        alive = wait_for_bootloader(bus, args.node, 20, 0.1)
    finally:
        close_bus(bus)
    print(f"Node {_node_label(args.node)} "
          f"{'is now in bootloader mode' if alive else 'did not answer a bootloader ping'}")
    return EXIT_OK if alive else EXIT_UNREACHABLE


def cmd_info(args):
    _preflight_or_exit()
    _require_board_id(args.node)
    bus = open_bus(args.channel)
    try:
        info = read_device_info(bus, args.node)
    finally:
        close_bus(bus)
    if info is None:
        print(f"Node {_node_label(args.node)} did not answer a device-info request "
              "(is it in bootloader mode, or powered off?)")
        return EXIT_UNREACHABLE
    for key, value in info.items():
        print(f"{key}: {value}")
    return EXIT_OK


def cmd_validate(args):
    firmware = Path(args.file).read_bytes()
    result = validate_firmware_bytes(firmware)
    print(format_validation(result))
    return EXIT_OK if result.ok else EXIT_VALIDATION


def cmd_flash(args):
    _preflight_or_exit()
    _require_board_id(args.node)
    firmware, description, app_start, product = _resolve_firmware_or_exit(args)
    print(f"Firmware source: {description}")
    _validate_or_exit(args, firmware, app_start)
    allowed, message = check_product_match(args.node, product, args.force_product)
    print(message)
    if not allowed:
        raise SystemExit(EXIT_VALIDATION)
    _confirm_or_exit(args, f"flash node {_node_label(args.node)}")
    bus = open_bus(args.channel)
    try:
        _flash_one_node(bus, args, args.node, firmware)
    finally:
        close_bus(bus)
    _out_result(True, EXIT_OK, f"Node {_node_label(args.node)} flashed successfully")
    _post_flash_reminder()
    return EXIT_OK


def cmd_flash_all_joints(args):
    _preflight_or_exit()
    firmware, description, app_start, product = _resolve_firmware_or_exit(args)
    print(f"Firmware source: {description}")
    _validate_or_exit(args, firmware, app_start)
    # One image across many nodes — every one of them has to match, so a robot
    # with mixed driver types is refused here rather than half-flashed.
    refused = []
    for node_id in JOINT_NODE_IDS:
        allowed, message = check_product_match(node_id, product, args.force_product)
        print(message)
        if not allowed:
            refused.append(node_id)
    if refused:
        print(f"Refusing: nodes {refused} do not match this image. Flash them "
              "individually with their own image.", file=sys.stderr)
        raise SystemExit(EXIT_VALIDATION)
    _confirm_or_exit(args, f"flash ALL joints {list(JOINT_NODE_IDS)} sequentially "
                           "(never the gripper)")
    bus = open_bus(args.channel)
    completed = []
    try:
        for node_id in JOINT_NODE_IDS:
            print(f"\n=== Joint node {_node_label(node_id)} ===")
            _flash_one_node(bus, args, node_id, firmware)
            completed.append(node_id)
    except SystemExit:
        print(f"\nCompleted before failure: "
              f"{[_node_label(n) for n in completed] or 'none'}", file=sys.stderr)
        raise
    finally:
        close_bus(bus)
    print(f"\nAll {len(completed)} joints flashed.")
    _post_flash_reminder()
    return EXIT_OK


# --- whole-robot update -------------------------------------------------------
# `flash-all-joints` sends ONE image to nodes 0-5 and refuses a robot whose nodes
# disagree. A real robot disagrees by construction: PAR6 is six stepfoc joints
# plus a spectral-bldc gripper, so "update the robot" can never be one file.
#
# The unit that makes sense is a VERSION, not a file. Pick V107; each node's
# product comes from its <driver_type> in the robot/gripper XML, and every node
# gets the image that belongs to it.


def resolve_robot_images(version, local_dir, declared, on_log):
    """{product_id: (bytes, description, app_start)} for every declared product.

    Resolves and validates EVERYTHING before the caller touches the bus. A
    missing spectral-bldc build has to fail here, not four joints into an
    update that can no longer be completed.
    """
    products = sorted(set(declared.values()))
    images = {}
    for product in products:
        if local_dir:
            candidates = [p for p in sorted(Path(local_dir).glob("*.bin"))
                          if infer_product_from_filename(p) == product]
            if not candidates:
                raise FirmwareSourceError(
                    f"No .bin for '{product}' in {local_dir}. Files must be "
                    f"named so the product is unambiguous (e.g. "
                    f"{product}-firmware-V107.bin).")
            if len(candidates) > 1:
                raise FirmwareSourceError(
                    f"{len(candidates)} candidate images for '{product}' in "
                    f"{local_dir}: {[p.name for p in candidates]}. Leave exactly "
                    "one per product — guessing here flashes the wrong firmware.")
            path = candidates[0]
            images[product] = (path.read_bytes(), str(path), APP_BASE_ADDRESS)
            on_log(f"  {product}: {path.name}")
            continue

        import firmware_github
        try:
            got = firmware_github.download_firmware(product, tag=version,
                                                    on_log=lambda _l: None)
        except firmware_github.FirmwareFetchError as error:
            raise FirmwareSourceError(f"{product}: {error}")
        app_start = (firmware_github.manifest_app_base_address(got["manifest"])
                     or APP_BASE_ADDRESS)
        images[product] = (got["path"].read_bytes(),
                           f"{product} {got['tag']}", app_start)
        on_log(f"  {product}: {got['tag']}"
               + ("  (cached)" if got.get("cached") else "  (downloaded)"))
    return images


class FirmwareSourceError(Exception):
    """A required image could not be resolved. Raised before the bus is opened."""


def cmd_update_robot(args):
    _preflight_or_exit()
    declared = expected_driver_types()
    if not declared:
        print("ERROR: the robot config declares no <driver_type> for any node, "
              "so there is nothing to resolve. Flash nodes individually.",
              file=sys.stderr)
        return EXIT_VALIDATION

    nodes = [n for n in sorted(declared) if n >= args.from_node]
    if not nodes:
        print(f"ERROR: --from-node {args.from_node} skips every declared node.",
              file=sys.stderr)
        return EXIT_VALIDATION

    # 1. Resolve + validate every image FIRST.
    _out_log("Resolving firmware for every declared driver type...")
    try:
        images = resolve_robot_images(args.version, args.dir, declared, _out_log)
    except FirmwareSourceError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        _out_result(False, EXIT_SOURCE, str(error))
        return EXIT_SOURCE
    for product, (data, description, app_start) in images.items():
        result = validate_firmware_bytes(data, app_start=app_start)
        if not result.ok and not args.skip_validate:
            print(f"ERROR: {description} failed validation: "
                  f"{'; '.join(result.errors)}", file=sys.stderr)
            return EXIT_VALIDATION
        _out_log(f"  {product}: {result.pages} page(s), validation "
                 + ("OK" if result.ok
                    else f"OVERRIDDEN ({'; '.join(result.errors)})"))

    # 2. Inventory: what is each node running now? Best-effort — a node in a
    #    bootloader from an earlier failure answers nothing, and that is exactly
    #    a node this command should still update.
    bus = open_bus(args.channel)
    inventory = {}
    try:
        for node in nodes:
            info = read_device_info(bus, node, timeout_s=0.3)
            inventory[node] = info["software_version"] if info else None
        _out_log("")
        _out_log(f"{'node':<6}{'role':<10}{'driver':<16}{'current':<10}target")
        for node in nodes:
            current = inventory[node]
            target = images[declared[node]][1]
            _out_log(f"{node:<6}{NODE_ROLES.get(node, '?'):<10}"
                     f"{declared[node]:<16}"
                     f"{(str(current) if current is not None else 'no reply'):<10}"
                     f"{Path(target).name if '/' in target else target}")
        _out_log("")

        _confirm_or_exit(args, f"update {len(nodes)} node(s) "
                               f"{[_node_label(n) for n in nodes]} — the arm is "
                               f"LIMP for roughly {len(nodes) * 28 // 60 + 1} min")

        # 3. Flash each node with ITS product's image.
        completed, failed = [], None
        for node in nodes:
            product = declared[node]
            data, description, _ = images[product]
            _out_log(f"\n=== {_node_label(node)} <- {description} ===")
            try:
                _flash_one_node(bus, args, node, data)
            except SystemExit as exiting:
                failed = (node, exiting.code)
                break
            completed.append(node)
    finally:
        close_bus(bus)

    if failed is not None:
        node, code = failed
        print(f"\nUPDATE INCOMPLETE — {len(completed)}/{len(nodes)} node(s) done: "
              f"{[_node_label(n) for n in completed] or 'none'}", file=sys.stderr)
        print(f"Node {_node_label(node)} failed. The robot now has MIXED firmware.",
              file=sys.stderr)
        print(f"Resume without redoing the ones that worked:\n"
              f"  python3 tools/flash_motor_firmware.py update-robot "
              f"{_resume_args(args)} --from-node {node}", file=sys.stderr)
        _out_result(False, code, f"update stopped at node {node}")
        return code

    _out_log(f"\nAll {len(completed)} node(s) updated.")
    _post_flash_reminder()
    return EXIT_OK


def _resume_args(args):
    """Reconstruct the source flags so the resume hint is copy-pasteable."""
    if args.dir:
        return f"--dir {args.dir}"
    if args.version:
        return f"--version {args.version}"
    return "--latest"


def cmd_set_id(args):
    _preflight_or_exit()
    _confirm_or_exit(args, f"change bootloader board ID {args.node} -> {args.new_id}")
    bus = open_bus(args.channel)
    try:
        set_board_id(bus, args.node, args.new_id)
    except BootloaderError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return EXIT_FLASH_ERROR
    finally:
        close_bus(bus)
    print(f"Board ID changed {args.node} -> {args.new_id} "
          "(the application syncs its node ID from the config page)")
    return EXIT_OK


def _add_common(parser):
    parser.add_argument("--channel", default="can0", help="CAN interface (default can0)")
    parser.add_argument("--yes", action="store_true",
                        help="skip the interactive safety confirmation")
    parser.add_argument("--progress-json", action="store_true",
                        help="emit one JSON object per line on stdout instead of "
                             "human text (for the GUI, which runs this as a "
                             "subprocess). Implies --yes: there is no terminal "
                             "to confirm at.")


def _add_flash_options(parser):
    source = parser.add_argument_group("firmware source (one required)")
    source.add_argument("--file", help="local .bin image")
    source.add_argument("--product", choices=("stepfoc", "spectral-bldc"),
                        help="download from this product's GitHub releases")
    source.add_argument("--version", default=None, metavar="TAG",
                        help="release tag (with --product)")
    source.add_argument("--latest", action="store_true",
                        help="latest release (with --product; same as omitting --version)")
    parser.add_argument("--no-erase", action="store_true",
                        help="skip ERASE_APP (not recommended)")
    parser.add_argument("--skip-validate", action="store_true",
                        help="flash even if the image fails validation (needs --yes)")
    parser.add_argument("--force-product", action="store_true",
                        help="flash even if the image's product differs from the "
                             "<driver_type> declared for that node in the robot XML "
                             "(use when the XML is out of date for this board)")
    parser.add_argument("--skip-app-reset", action="store_true",
                        help="board is already in bootloader mode")
    parser.add_argument("--chunk-frames", type=int, default=DEFAULT_STREAM_CHUNK_FRAMES,
                        help="stream frames per STREAM_STATUS verify (default 16; "
                             "lower if chunks keep failing)")
    parser.add_argument("--frame-gap", type=float, default=0.0, metavar="S",
                        help="pause between stream frames in seconds (default 0)")


def build_parser():
    parser = argparse.ArgumentParser(
        description="Flash Spectral/STEPFOC motor-driver firmware over CAN "
                    "(STM32F103 bootloader host). Runtime must be stopped.")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("scan", help="find boards in bootloader mode "
                                    "(default variant REBOOTS every motor)")
    p.add_argument("--no-reset", action="store_true",
                   help="passive: only find boards already in the bootloader")
    p.add_argument("--listen", type=float, default=None,
                   help=f"listen window in seconds. Default depends on mode: "
                        f"{SCAN_LISTEN_ACTIVE_S:.0f}s for an active scan (must outlast the "
                        f"~8.3s app-reset -> bootloader delay), {SCAN_LISTEN_PASSIVE_S:.0f}s "
                        f"passive, 60s with --catch-boot")
    p.add_argument("--catch-boot", action="store_true",
                   help="ping continuously so a COLD BOOT window cannot be missed. "
                        "Use when a software reset never reaches the bootloader "
                        "(app does not clear the SRAM magic) — start this, then "
                        "power-cycle the board.")
    _add_common(p)
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser("ping", help="ping one bootloader board")
    p.add_argument("--node", type=int, required=True)
    _add_common(p)
    p.set_defaults(func=cmd_ping)

    p = sub.add_parser("app-reset", help="reboot one running motor into its bootloader")
    p.add_argument("--node", type=int, required=True)
    _add_common(p)
    p.set_defaults(func=cmd_app_reset)

    p = sub.add_parser("info", help="read device info from a RUNNING application")
    p.add_argument("--node", type=int, required=True)
    _add_common(p)
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("validate", help="check a .bin without touching the bus")
    p.add_argument("--file", required=True)
    # Only --progress-json, not the whole _add_common set: validate never opens
    # the bus, so --channel and --yes would be meaningless noise in its help.
    p.add_argument("--progress-json", action="store_true",
                   help="emit JSON lines instead of human text")
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("flash", help="flash one node")
    p.add_argument("--node", type=int, required=True,
                   help="bootloader board ID (= CAN node: 0-5 joints, 6 gripper)")
    p.add_argument("--app-node", type=int, default=None,
                   help="app node ID for the reset command, if it differs "
                        "from --node (rare)")
    _add_flash_options(p)
    _add_common(p)
    p.set_defaults(func=cmd_flash)

    p = sub.add_parser("flash-all-joints",
                       help="flash nodes 0-5 sequentially with the same image "
                            "(never the gripper)")
    p.set_defaults(app_node=None)
    _add_flash_options(p)
    _add_common(p)
    p.set_defaults(func=cmd_flash_all_joints)

    p = sub.add_parser(
        "update-robot",
        help="update EVERY declared node to one firmware version, each with the "
             "image its <driver_type> calls for")
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--version", metavar="TAG",
                     help="release tag to put on every node (e.g. V107)")
    src.add_argument("--latest", action="store_true",
                     help="newest release for each product")
    src.add_argument("--dir", metavar="PATH",
                     help="local folder holding one .bin per product, named so "
                          "the product is unambiguous")
    p.add_argument("--from-node", type=int, default=0, metavar="N",
                   help="resume an interrupted update: skip nodes below N")
    p.add_argument("--skip-validate", action="store_true",
                   help="flash even if an image fails validation (dangerous)")
    p.add_argument("--force-product", action="store_true",
                   help="ignore a driver_type mismatch on a node")
    p.add_argument("--no-erase", action="store_true")
    p.add_argument("--skip-app-reset", action="store_true")
    p.add_argument("--app-node", type=int, default=None)
    p.add_argument("--chunk-frames", type=int, default=DEFAULT_STREAM_CHUNK_FRAMES)
    p.add_argument("--frame-gap", type=float, default=0.0)
    _add_common(p)
    p.set_defaults(func=cmd_update_robot)

    p = sub.add_parser("set-id", help="change a bootloader's board ID")
    p.add_argument("--node", type=int, required=True, help="current board ID")
    p.add_argument("--new-id", type=int, required=True)
    _add_common(p)
    p.set_defaults(func=cmd_set_id)

    return parser


def main():
    global _json_mode, _json_stream
    args = build_parser().parse_args()
    if getattr(args, "progress_json", False):
        # Machine-readable mode. Nothing is attached to stdin, so an interactive
        # confirmation would deadlock — the caller already confirmed in its UI.
        _json_mode = True
        args.yes = True
        # Claim the real stdout for JSON, then point plain `print` at stderr.
        # Structural rather than tidy: this file (and firmware_github, which
        # takes an on_log callable) prints human text from ~30 places, and a
        # single stray line would derail a parser mid-flash. Redirecting once
        # means no future print can break the contract, and the text stays
        # visible on stderr for debugging.
        _json_stream = sys.stdout
        sys.stdout = sys.stderr
    try:
        try:
            _code = args.func(args)
        except SystemExit as exiting:
            # Backstop: several refusal paths (validation, bad node, aborted
            # confirmation) exit directly. Make sure JSON callers still get a
            # result line rather than only an exit status.
            _code = exiting.code if isinstance(exiting.code, int) else EXIT_FLASH_ERROR
            _out_result(_code == EXIT_OK, _code,
                        _EXIT_MEANINGS.get(_code, f"exited with code {_code}"))
            raise
        _out_result(_code == EXIT_OK, _code,
                    _EXIT_MEANINGS.get(_code, f"exited with code {_code}"))
        raise SystemExit(_code)
    except can.CanOperationError as error:
        # Almost always "Transmit buffer full": CAN needs at least one other
        # node to ACK a frame. With every board unpowered, the cable off, or
        # termination missing, the controller retries forever and the queue
        # backs up. Saying that beats a raw socketcan traceback.
        message = (f"CAN transmit failed ({error}).\n"
                   "Nothing on the bus is acknowledging. Check that the boards "
                   "are powered, the CAN cable is connected,\nand the bus is "
                   "terminated (~60 ohm across CANH/CANL with power off).")
        _out_result(False, EXIT_UNREACHABLE, message)
        if not _json_mode:
            print(message, file=sys.stderr)
        raise SystemExit(EXIT_UNREACHABLE)
    except RuntimeError as error:
        # open_bus refusal or interface bring-up failure
        _out_result(False, EXIT_PREFLIGHT, str(error))
        if not _json_mode:
            print(str(error), file=sys.stderr)
        raise SystemExit(EXIT_PREFLIGHT)
    except KeyboardInterrupt:
        message = ("Interrupted. A board mid-flash stays in its bootloader — "
                   "`scan --no-reset` finds it; flash again to recover.")
        _out_result(False, EXIT_FLASH_ERROR, message)
        if not _json_mode:
            print(f"\n{message}", file=sys.stderr)
        raise SystemExit(EXIT_FLASH_ERROR)


if __name__ == "__main__":
    main()
