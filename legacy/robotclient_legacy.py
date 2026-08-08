#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: robotclient.py
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

import socket
import json
from utility.RTI_utils import cmd_name as _cmd_name

class Robot:
    
    # TODO ??
    LOCAL_PORT = 29000      # Port used for local connections
    REMOTE_PORT = 29000     # Port used for remote connections

    def __init__(self, ip, port=None, timeout=None):
        """
        If no port is specified, choose it automatically based on whether
        the connection is local or remote.
        If you pass a port manually → uses it directly.
            If you don’t pass port →
            If ip is "127.0.0.1" or "localhost", uses LOCAL_PORT.
            Otherwise uses REMOTE_PORT.
        """
        self.ip = ip
        self.timeout = timeout

        # Decide which port to use if none provided
        if port is None:
            if self._is_local(ip):
                self.port = self.LOCAL_PORT
            else:
                self.port = self.REMOTE_PORT
        else:
            self.port = port

        self.conn = socket.create_connection((ip, self.port), timeout=timeout)
        self.conn.settimeout(timeout)
        self.last_command = None  # Store the last command sent (as string)


    @staticmethod
    def _is_local(ip: str) -> bool:
        """Return True if IP is localhost or matches local network interface."""
        return ip in ("127.0.0.1", "localhost", "::1")


    def set_timeout(self, timeout: float):
        """Dynamically adjust the socket timeout."""
        self.timeout = timeout
        if self.conn:
            self.conn.settimeout(timeout)

    @staticmethod
    def cmd_name(full_command: str) -> str:
        """Extracts the base command name before the first parenthesis."""
        return _cmd_name(full_command)

    def _send_raw(self, command_str: str):
        """Send a raw command like 'ctraj(1,2,3)' and wait for a matching response."""
        if not command_str.endswith("\n"):
            command_str += "\n"

        self.last_command = command_str.strip()  # Save clean version for comparison
        self.conn.sendall(command_str.encode())  # Send the command

        buffer = ""

        # Get ctraj from ctraj(1,2,3,4,t=1)
        expected_cmd = self.cmd_name(self.last_command)

        while True:
            try:
                part = self.conn.recv(4096).decode()
                buffer += part

                lines = buffer.splitlines(keepends=False)
                buffer = ""  # Can be improved for partial-line handling

                for line in lines:
                    try:
                        parsed = json.loads(line)
                        # we receive packet like "id": cmd_id,"source": source,"ip": ip or "unknown","msg": command
                        received_cmd = self.cmd_name(parsed.get("msg", ""))  # get only msg 

                        # If it's a direct match, return output
                        # so if "ctraj" = "ctraj"
                        if received_cmd == expected_cmd:
                            return parsed.get("output", None)

                        # If it's an error message, raise it immediately
                        elif received_cmd == "error":
                            error_detail = parsed.get("error", "Unknown robot error.")
                            raise RuntimeError(f"[{self.ip}] ❌ Robot error: {error_detail}")

                        # Otherwise, ignore unrelated messages
                        else:
                            print(f"[{self.ip}] ⚠️  Ignoring unrelated message: {parsed}")

                    except json.JSONDecodeError:
                        print(f"[{self.ip}] ⚠️  Could not parse response: {line}")

            except socket.timeout:
                raise RuntimeError(f"❌ Timeout waiting for response to: {self.last_command}")

    # === Robot command wrappers ===
    def init(self): return self._send_raw("init()")

    # hold pos, impedance, admitance, idle, grav hold, motors off, 
    def ctraj(self, *args): return self._send_raw(f"ctraj({', '.join(str(a) for a in args)})")
    def jtraj(self, *args): return self._send_raw(f"jtraj({', '.join(str(a) for a in args)})")
    def jog(self, axis, amount): return self._send_raw(f"jog('{axis}', {amount})")

    def read_input(self, *args): return self._send_raw(f"input({', '.join(repr(a) for a in args)})")
    def set_output(self, *args): return self._send_raw(f"output({', '.join(repr(a) for a in args)})")


    def calibrate(self, *args): return self._send_raw(f"calibrate({', '.join(repr(a) for a in args)})")
    def home(self, *args): return self._send_raw(f"home({', '.join(repr(a) for a in args)})")
    def master(self, *args): return self._send_raw(f"master({', '.join(repr(a) for a in args)})")
    def stop(self, *args): return self._send_raw(f"stop({', '.join(repr(a) for a in args)})")
    def pause(self, *args): return self._send_raw(f"pause({', '.join(repr(a) for a in args)})")
    def resume(self, *args): return self._send_raw(f"resume({', '.join(repr(a) for a in args)})")
    def gripper(self, *args): return self._send_raw(f"gripper({', '.join(repr(a) for a in args)})")
    def idle(self, *args): return self._send_raw(f"idle({', '.join(repr(a) for a in args)})")
    def rti_begin(self, *args): return self._send_raw(f"rti_begin({', '.join(repr(a) for a in args)})")
    def rti_end(self, *args): return self._send_raw(f"rti_end({', '.join(repr(a) for a in args)})")

    def send(self, command_string): return self._send_raw(command_string)

    # mx:export_to=robot_sidecar_globals.py
    def example_command(self, *velocities: float):
        """Move the robot by specifying joint velocities.

        Parameters
        ----------
        velocities : float
            A list of joint velocities in degrees per second. Must match the number of robot joints.

        Raises
        ------
        ValueError
            If the number of arguments doesn't match the expected joint count.
        TypeError
            If any value is not a float or convertible to float.
        """
        if len(velocities) != self.num_joints:
            raise ValueError(
                f"example_command: expected {self.num_joints} joint velocities, got {len(velocities)}"
            )
        try:
            velocities = [float(v) for v in velocities]
        except (ValueError, TypeError) as e:
            raise TypeError("All joint velocities must be numeric.") from e

        return self._send_raw(f"MoveJointsVel({', '.join(map(str, velocities))})")


    def close(self):
        if self.conn:
            self.conn.close()



# ========================================================================== #
# Module test
# ========================================================================== #
import threading
import time

def mock_robot_server(host="127.0.0.1", port=29000):
    import socket
    import json

    def respond_to(command):
        base_cmd = command.strip().split("(")[0]
        if base_cmd == "init":
            return {"msg": command.strip(), "output": "initialized"}
        elif base_cmd == "home":
            return {"msg": command.strip(), "output": 1}
        elif base_cmd == "ctraj":
            return {"msg": "error", "output": "Trajectory started"}
        elif base_cmd == "error":
            return {"msg": "error", "error": "Simulated error"}
        else:
            return {"msg": command.strip(), "output": f"Unknown command: {command.strip()}"}

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen(1)
        print(f"[MOCK ROBOT] Listening on {host}:{port}")
        conn, addr = s.accept()
        with conn:
            print(f"[MOCK ROBOT] Connected by {addr}")
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                cmd = data.decode().strip()
                response = respond_to(cmd)
                conn.sendall((json.dumps(response) + "\n").encode())


# ==========================================================================
# Mock robot server test
# ==========================================================================
if __name__ == "__main__":
    import sys

    # Start mock server in background thread
    server_thread = threading.Thread(target=mock_robot_server, daemon=True)
    server_thread.start()

    # Give server time to start
    time.sleep(1)

    ROBOT_IP = "127.0.0.1"
    try:
        print("[INFO] Connecting to mock robot...")
        robot = Robot(ROBOT_IP)

        print("[INFO] Sending init() command...")
        print(f"[RESULT] {robot.init()}")

        print("[INFO] Sending home() command...")
        print(f"[RESULT] {robot.home()}")

        print("[INFO] Sending ctraj() command...")
        print(f"[RESULT] {robot.ctraj(0.1, 0.2, 0.3)}")

        robot.close()
        print("[INFO] Done.")

    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)