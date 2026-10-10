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
import socket
import Spectral_BLDC as Spectral
import can
import time
import subprocess

logger = logging.getLogger(__name__)

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
                logger.warning(f"[CAN] {counter['pending']} frame(s) dropped — the TX queue refused them ({e}); "
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
