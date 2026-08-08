#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: command_executor.py
Author: Petar Crnjak
Date: 2024-06-11
Version: 1.0
Description: 
    This script demonstrates a detailed 
    header format with additional metadata.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
Dependencies: os, sys
"""

"""
logging.info("Init GPIO")
logging.debug("This goes to the file only")
logging.info("System initialized")
logging.warning("This shows up in both file and terminal")
logging.error("This is bad 😬")
logging.critical("This is bad 😬")

"""


import zmq
import threading
import time
from queue import Queue, Empty
import socket
import os
import psutil
from datetime import datetime, timezone
import random
import string
import json
import logging
import numpy as np
from utility.logger_setup import setup_logging
import joblib
from utility.RTI_utils import *
from config.networking import ZMQ_CMD_SOCKET, ZMQ_STATUS_SOCKET
try:
    from rule_table import *  # Optional: rule_table.py not required to run
except ImportError:
    pass  # rule_table.py not present - rule-based dispatch disabled

# ========================================================================== #
# Setup logging
# ========================================================================== #
DEBUG = True
from project_paths import HIGH_LEVEL_LOG, JOBLIB_DATA
setup_logging(str(HIGH_LEVEL_LOG), log_to_console=DEBUG)

# NOTE: os.system('pkill -SIGINT -f supervisor.py') was here - REMOVED (was killing supervisor on startup)



# ========================================================================== #
# PORT Settings
# ========================================================================== #
GUI_port = 27777           # We receive commands from GUI
Supervisor_port = 28888    # We receive commands from Supervisor

TCP_port = 29000           # TCP server port (used for localhost and all external IPs)
command_queue = Queue()    # Queue for receiving commands from all sources

from utility.process_utils import setup_process
setup_process(core=2, rt=False)  # HIGH_LEVEL: own core, normal scheduling

allowed_IPs = {"192.168.0.179", "192.168.1.101", '127.0.0.1'}

# Global state to track current command waiting for ACK
robot_mode = "idle" # 
robot_state = "enabled" # Enabled or disabled
error_state = None
# TODO In every thread i need this data i need to:
    #robot_mode2 = read_shared_string(robot_mode_shm)
    #robot_state2 = read_shared_string(robot_state_shm)
    #error_list2 = read_shared_json(error_state_shm)
current_command = None # command that is currently being executed by high_level
ack_event = threading.Event()

# Global dictionary to store client connections
active_tcp_clients = {}


def print_current_queue(queue):
    temp_list = list(queue.queue)  # Safe: .queue is a deque, list() is a shallow copy
    print("\n========== CURRENT QUEUE ==========")
    if not temp_list:
        print("[Queue is empty]")
    else:
        for i, cmd in enumerate(temp_list):
            print(f"{i+1}. id={cmd['id']} | source={cmd['source']} | cmd={cmd['msg']}")
    print("===================================\n")

queue_bypass_commands= {"stop", "pause", "resume", "clear", "enable", "disable"}

# ========================================================================== #
# Command validation
# ========================================================================== #
def is_command_allowed(mode, state, command):
    """
    Check if a command is allowed given the current robot mode and state.
    TODO: Implement full validation logic against mode/state machine.
    """
    return True


# ========================================================================== #
# Place received commands in queue or skip it if allowed
# ========================================================================== #
def enqueue_command(queue, source, command, ip=None):
    """
    Add a command to the shared command queue.
    Includes source label, timestamp, IP (if known), and actual command.
    """
    # Create a short id for this specific msg
    cmd_id = generate_short_id()
    # we received some msg from a source with ip, pack it in dict.
    # timestamp is wall-clock time.time() (NOT perf_counter) so it stays
    # meaningful across processes — full audit trail of who sent what, when.
    record = {
        "id": cmd_id,
        "source": source,
        "ip": ip or "unknown",
        "timestamp": time.time(),
        "msg": command
    }

    # Check if this command bypasses the queue
    try:
        #cmd_type = json.loads(command)["cmd"]
        cmd_type = cmd_name(command) # Extracts just a command from a msg, ex ctraj(1,2,3) is ctraj

    except Exception as e:
        logging.warning(f"Failed to extract command from {command}: {e}")
        cmd_type = None

    # If command is located in bypass_commands send it instantly
    if cmd_type in queue_bypass_commands:
        send_directly_to_rti(record)
    # 
    else:
        queue.put(record)
        logging.info(f"RECEIVED - id={cmd_id} - from={source} - IP={ip or 'unknown'} - msg={command}")
        print_current_queue(queue)

    return cmd_id


# ========================================================================== #
# Ignore Queue and send directly to RTI via ZMQ
# ========================================================================== #
def send_directly_to_rti(command_record):
    """
    Immediately send a command to RTI without using the queue.
    This bypasses normal command queuing behavior.
    """
    ctx = zmq.Context()
    socket = ctx.socket(zmq.PUSH)
    socket.connect(ZMQ_CMD_SOCKET)
    global current_command

    try:
        current_command = command_record
        # Convert the command record dictionary to a JSON string.
        json_msg = json.dumps(command_record)
        execute_robot_command(command_record) #TODO also do joblib stuff here
        # Send the JSON string to the RTI over the PUSH socket.
        socket.send_string(json_msg)
        logging.info(f"[BYPASS] Directly sent to RTI - id={command_record['id']} - cmd={command_record['msg']}")
    except Exception as e:
        logging.error(f"[BYPASS] Failed to send directly: {e}")
    finally:
        # Always close the socket and terminate the context to release resources.
        socket.close()
        ctx.term()

forward_to_TCP_commands = queue_bypass_commands

# ========================================================================== #
# Listen to responses from RTI (ZMQ)
# ========================================================================== #
def zmq_listener(queue: Queue):
    """
    Listens for status replies from RTI via ZMQ IPC (Unix domain socket).
    Binds to ZMQ_STATUS_SOCKET; RTI connects and pushes "done" after each command.
    """
    ctx = zmq.Context()
    socket = ctx.socket(zmq.PULL)
    socket.bind(ZMQ_STATUS_SOCKET)
    global current_command

    while True:
        # Block here until a message is received on this socket from RTI
        msg = socket.recv_string()
        # Convert received string to dict, string is in format:
            #"id": cmd_id,
            #"source": source,
            #"ip": ip or "unknown",
            #"msg": command,
            #"output" : data
        parsed_msg = json.loads(msg) 
        # Compare the id of the command we received and the one we sent
        if (parsed_msg['id'] == current_command['id']) and (parsed_msg['msg'] == current_command['msg']):
            # Log that our RTI has finished executing the command with 'id'
            logging.info(f"FINISHED - id={parsed_msg['id']} - source={parsed_msg['source']} - ip={parsed_msg['ip'] or 'unknown'} - msg={parsed_msg['msg']} - output={parsed_msg['output']}")
            #logging.info(f"FINISHED - {parsed_msg}")
            current_command = None # We finished executing the command so it is None
            ack_event.set()

            # If the command we received was sent via TCP, forward the command we received from RTI
            if parsed_msg.get("source") == "TCP":
                ip = parsed_msg.get("ip")
                conn = active_tcp_clients.get(ip)
                if conn:
                    try:
                        conn.sendall((json.dumps(parsed_msg) + "\n").encode())
                        logging.info(f"TCP Sent RTI result back to {ip}")
                    except Exception as e:
                        logging.error(f"TCP Failed to send to {ip}: {e}")

        # Handle special RTI-originated commands not tied to current_command (Like errors, pause...)
        # Forward them to a TCP client if it is acive 
        # TODO also if its error, clear the queue here and all the other stuff
        else:
            try:
                # Try to parse the command type from the 'msg' field
                # msg_payload = json.loads(parsed_msg["msg"])
                # cmd_type = msg_payload.get("cmd", None)
                cmd_type = cmd_name(parsed_msg["msg"])

            except Exception as e:
                logging.warning(f"RTI response couldn't be parsed for cmd type: {e}")
                cmd_type = None

            # TODO change this logic
            if cmd_type in forward_to_TCP_commands:
                # ✅ Use the first available TCP connection (if any)
                if active_tcp_clients:
                    for ip, conn in active_tcp_clients.items():
                        try:
                            conn.sendall((json.dumps(parsed_msg) + "\n").encode())
                            logging.info(f"[FORWARD] Sent '{cmd_type}' from RTI to TCP client {ip}")
                            break  # Only send to one client
                        except Exception as e:
                            logging.error(f"[FORWARD] Failed to send '{cmd_type}' to {ip}: {e}")
                else:
                    logging.info(f"[FORWARD] Received '{cmd_type}' from RTI but no TCP client connected.")
            else:
                logging.warning(f"Unexpected RTI response: {msg} != {current_command}")


# ========================================================================== #
# Execute received commands
# ========================================================================== #
def execute_robot_command(msg):
    """
    Interpret and execute the robot command using the provided robot client.

    Args:
        msg (dict): A command dictionary with 'msg', 'args', etc.
        robot (RobotClient): An instance of the robot communication class.

    Returns:
        bool: True if handled or executed successfully, False otherwise.
    """
    try:

        #raise Exception("Intentional failure for testing")

        command = msg.get("msg", "")
        args = msg.get("args", [])

        if command.startswith("ctraj"):
            # Optional: Log or process some data before running
            data_joblib = {
                "flag": "empty",
                "array": np.random.randn(10, 10),
                "description": f"Executing ctraj: {command}",
                "data_type": "joint angles",
                "timestamp": "2025-07-20 14:30",
                "array_len": 200
            }
            
            #  joblib.dump(data_joblib, "data.pkl")
            joblib.dump(data_joblib, str(JOBLIB_DATA))

            return True

        elif command.startswith("jtraj"):

            return True

        else:
            logging.info(f"ℹ️ Command received but not executed: {command}")
            return True  # Command not meant for execution, but handled
    except Exception as e:
        logging.error(f"❌ Failed to execute robot command: {e}")
        return False


# ========================================================================== #
# Get a command from queue and forward it to RTI
# ========================================================================== #
def forward_to_rti(queue: Queue):
    """
    Takes commands from the command queue and pushes them to RTI via ZMQ IPC.
    Connects to ZMQ_CMD_SOCKET; RTI binds and receives on the other end.
    """
    ctx = zmq.Context()
    socket = ctx.socket(zmq.PUSH)
    socket.connect(ZMQ_CMD_SOCKET)

    global current_command

    while True:
        try:
            msg = queue.get(timeout=1)
            # TODO command list execution here!
            execute_robot_command(msg) #TODO also do joblib stuff here
            ack_event.clear()
            current_command = msg #msg["cmd"]
            json_msg = json.dumps(msg)  # Convert the whole dictionary to a JSON string
            
            logging.info(f"FORWARDED - id={msg['id']} - source={msg['source']} - ip={msg['ip'] or 'unknown'} - msg={msg['msg']}")
            socket.send_string(json_msg)

            ack_event.wait()  # This blocks until .set() is called by zmq_listener
        except Empty:
            continue  # No messages in queue, loop again


# ========================================================================== #
# Get msgs from GUI and supervisor here. 
# GUI and supervisor dont receive responses
# ========================================================================== #
def handle_push(label: str, port: int, queue: Queue):
    """
    Generic function for receiving PULL-style messages over ZMQ.
    Used for GUI and Supervisor.
    """
    ctx = zmq.Context() # Create a new ZeroMQ context for this listener.
    socket = ctx.socket(zmq.PULL) # Create a PULL socket to receive messages pushed by other sockets.
                                   # PULL sockets can receive from multiple PUSH senders.
    socket.bind(f"tcp://*:{port}")  # Bind the PULL socket to all network interfaces on the given port.
    logging.info(f"{label} Listening on port {port}")
    global robot_mode
    global robot_state
    global error_state
    global current_command
    
    while True:
        # Block here until a message is received
        msg = socket.recv_string()
        cmd_type = cmd_name(msg) # Extracts just a command from a msg, ex ctraj(1,2,3) is ctraj
        print(f"cmd_type is {cmd_type}")

        # TODO check if the msg is valid here?
        valid = is_command_allowed(mode = robot_mode, state = robot_state, command = cmd_type)

        # If msg is valid and priority is right
        if valid:
            enqueue_command(queue, label, msg, ip="local")
        else:
            logging.warning(f"REJECTED - 'msg':{msg}")


# ========================================================================== #
# Handle TCP connection
# ========================================================================== #
def handle_client(conn, addr, command_queue):
    """
    Handles an individual TCP client connection.
    Reads and processes incoming data, sends back a response, and cleans up on disconnect.
    """
    ip, port = addr
    active_tcp_clients[ip] = conn  # Store connection

    logging.info(f"TCP Handling client {ip}:{port}")
    global robot_mode
    global robot_state
    global error_state
    global current_command

    try:
        while True:
            data = conn.recv(1024)  # Blocking call
            if not data:
                logging.warning(f"TCP {ip}:{port} disconnected.")
                break
            msg = data.decode().strip()
            cmd_type = cmd_name(msg)
            print(f"cmd_type is {cmd_type}")
            #print(f"[TCP] Received from {ip}: {msg}")
            # TODO check if msg is valid, if yes return OK/ack if
            #   If not return that its invalid msg
            valid = is_command_allowed(mode = robot_mode, state = robot_state, command = cmd_type)
            if valid:
                cmd_id = enqueue_command(command_queue, "TCP", msg, ip=ip)  #  Get command ID
            else: 
                logging.warning(f"REJECTED - 'msg':{msg}")
                error_msg = f"[REJECTED] Invalid command: {msg}\n"
                conn.sendall(error_msg.encode())
            #conn.sendall(f"{cmd_id}\n".encode())  #  Send back ID as response
            #conn.sendall()
    except ConnectionResetError:
        logging.info(f"TCP {ip}:{port} connection reset.")
    finally:
        del active_tcp_clients[ip]  #  Remove connection on disconnect
        conn.close()

# ========================================================================== #
# TCP server
# ========================================================================== #
def tcp_server(command_queue, host='0.0.0.0', port=TCP_port):
    """
    TCP server that accepts connections and spawns handler threads for each.
    Rejects connections from IPs not in ALLOWED_IPS.
    """
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen(5)
        logging.info(f"[TCP] Server listening on {host}:{port}")

        while True:
            conn, addr = s.accept()
            ip, _ = addr
            if ip not in allowed_IPs:
                logging.warning(f"TCP Rejected connection from {ip}")
                conn.sendall(b"Connection not allowed.\n")
                conn.close()
                continue

            logging.info(f"TCP Accepted connection from {ip}")
            thread = threading.Thread(
                target=handle_client,
                args=(conn, addr, command_queue),
                daemon=True
            )
            thread.start()




# ========================================================================== #
# Main
# ========================================================================== #

if __name__ == "__main__":
    # Thread that listens for replies from RTI
    threading.Thread(target=zmq_listener, args=(command_queue,), daemon=True).start()

    # Threads that listen to GUI and supervisor
    threading.Thread(target=handle_push, args=("GUI", GUI_port, command_queue), daemon=True).start()
    threading.Thread(target=handle_push, args=("Supervisor", Supervisor_port, command_queue), daemon=True).start()

    # Thread to forward all commands to RTI
    threading.Thread(target=forward_to_rti, args=(command_queue,), daemon=True).start()

    # Thread that listens to TCP from outside and localhost for onboard python and blockly code running
    threading.Thread(target=tcp_server, args=(command_queue,), daemon=True).start()
    
    time.sleep(0.01)
    logging.info("All threads started. Listening for commands...")

    # Keep main thread alive
    while True:
        time.sleep(1)

