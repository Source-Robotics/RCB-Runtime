"""The program run the editor started — one at a time — and its event log: the server's half of the runner
protocol (blockly/runner.py). TODO/blockly_plan.md §4, §7.

    start(name)            spawn `python -m blockly.runner programs/<name>.json --stop-on-eof`
    move_here(name, point) a one-block program the server builds: a joint move to the point at 10 %
    check(name, workspace, points)   a Check: the real planner, offline, from where the arm is (blockly/check.py)
    pause() / resume() / step()      a word on the runner's stdin
    stop()                 robot.stop() on the SERVER's own connection FIRST — it works even if the runner is stuck
                           or dead — then "stop" on stdin, SIGTERM after 1 s, SIGKILL after 2 s
    events_since(seq)      what happened since `seq`, plus a snapshot (every open browser sees the same run)
    position()             the arm's joints + TCP pose, for "use current position"

The server stops the robot again when the runner DIED (any exit but 0-3: it may not have done it, and the executor
does not yet stop motion when a client disconnects) or failed with motion possibly running (its error says
"arm": "stopped"). Not after a failure the robot reported ("arm": "holding"): the arm then holds in EXEC, as an
EXEC program leaves it (user, 2026-10-05). Not after "refused before it began" either: it could stop someone else's
jog.

The server's robot connection only reads and stops (both open to every client); it never takes control, so it
cannot get in the way of the dashboard or of the program itself.
"""
import collections
import json
import os
import signal
import subprocess
import sys
import threading
import time

from blockly import programs, run_records
from project_paths import LOGS_DIR
from utility.housekeeping import rotate_by_size
from robotclient import RobotClient
from robotclient.constants import DEFAULT_PORT
from robotclient.exceptions import RobotError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAX_EVENTS = 2000
MOVE_HERE_SPEED = 10                  # percent, "Move here" in the Points list
MOVE_HERE_FILE = ".move_here.json"    # in programs/, hidden from the list (a name cannot start with ".")
CHECK_FILE = ".check.json"
CHECK_TIMEOUT_S = 120.0
CHECK_BUSY_MODES = ("HOMING", "JOG", "RTI", "SOFT_RTI", "FLASHING")   # the arm is moving under someone's hand
STOP_TERM_S = 1.0
STOP_KILL_S = 2.0

_settings = {"host": "127.0.0.1", "port": DEFAULT_PORT, "client_factory": lambda: RobotClient(sync=False, name="blocks"),
             "runner_args": [], "log_path": os.path.join(str(LOGS_DIR), "block_programs.log")}   # LOGS_DIR: the simulator's / a net's own
_lock = threading.RLock()
_run = {"proc": None, "program": None, "state": "idle", "block": None, "error": None, "exit": None,
        "started": None, "seq": 0, "record": None}
_events = collections.deque(maxlen=MAX_EVENTS)
_robot = {"client": None}


class RunError(Exception):
    """A run request that cannot be done now; the message is for the user."""


def configure(host=None, port=None, client_factory=None, runner_args=None, log_path=None, record_dir=None):
    """Where the robot is, the client for the server's own connection, extra runner arguments (the nets pass a
    fake client), where the runners' stderr goes, where the run records go (blockly/run_records.py)."""
    for key, value in (("host", host), ("port", port), ("client_factory", client_factory),
                       ("runner_args", runner_args), ("log_path", log_path)):
        if value is not None:
            _settings[key] = value
    run_records.configure(record_dir)


def start(name):
    path = programs.program_path(name)
    if not os.path.exists(path):
        raise RunError(f"there is no program {name!r} — save it first")
    return _start(name, path)


def move_here(point_name, point):
    """Move the arm to one taught point ("Move here" in the Points list): a one-block program — a joint move at
    MOVE_HERE_SPEED % — that the SERVER builds from the point's numbers (the browser sends data, never code: D1),
    run like any program, so Stop, the log and every check before a start apply. One click, as several
    small arms do (some industrial cobots move only while a button is held — 2026-10-05, user's choice)."""
    programs.check_point_name(point_name)
    points = programs.check_points({point_name: point})
    workspace = {"blocks": {"languageVersion": 0, "blocks": [
        {"type": "rcb_start", "id": "start", "x": 0, "y": 0, "next": {"block": {
            "type": "rcb_move_joints", "id": "move_here", "fields": {"SPEED": MOVE_HERE_SPEED},
            "inputs": {"TARGET": {"block": {"type": "rcb_point", "id": "move_here_point",
                                            "fields": {"NAME": point_name}}}}}}}]}}
    path = os.path.join(programs.programs_dir(), MOVE_HERE_FILE)
    with _lock:
        if running():
            raise RunError(f"the program {_run['program']!r} is running — stop it first")
        programs.write_program_file(path, {"name": "move here", "workspace": workspace, "points": points})
        return _start(f"move here: {point_name}", path)


def check(name, workspace, points):
    """A Check (blockly/check.py): the program run against the real planner, offline, from where the arm is now.
    Blocks until it ends; returns {"result": "passed" | "limited" | "failed", "error", "summary", "seconds"}.

    Refused while a program runs, or while the arm is homing / jogging / streamed: the planner loads in a new process
    and competes with the RT loop for memory bandwidth (api_tests/58 measures how much). One at a time (the server)."""
    if running():
        raise RunError(f"the program {_run['program']!r} is running — Check after it ends")
    try:
        mode = _client().get_status().get("mode")
    except (RobotError, OSError) as e:
        _drop_client()
        raise RunError(f"the robot did not answer ({e}) — Check starts from where the arm is") from None
    if mode in CHECK_BUSY_MODES:
        raise RunError(f"the arm is busy ({mode}) — Check when it is idle")
    start = position()["joints_deg"]
    path = os.path.join(programs.programs_dir(), CHECK_FILE)
    programs.write_program_file(path, {"name": name, "workspace": workspace, "points": points or {}})
    cmd = [sys.executable, "-m", "blockly.runner", path, "--dry-run",
           "--start-joints", ",".join(f"{v:.6f}" for v in start)]
    t0 = time.monotonic()
    try:
        done = subprocess.run(cmd, cwd=ROOT, stdin=subprocess.DEVNULL, capture_output=True, text=True,
                              timeout=CHECK_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        raise RunError(f"the Check took longer than {CHECK_TIMEOUT_S:.0f} s and was stopped") from None
    seconds = round(time.monotonic() - t0, 1)
    if done.stderr.strip():
        with open(_settings["log_path"], "a") as log:
            log.write(f"--- {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} check {name!r}\n{done.stderr}")
    events = []
    for line in done.stdout.splitlines():
        try:
            events.append(json.loads(line))
        except ValueError:
            pass
    error = next((e for e in events if isinstance(e, dict) and e.get("type") == "error"), None)
    summary = next((e for e in events if isinstance(e, dict) and e.get("type") == "check"), None)
    if done.returncode == 0 and summary is not None:
        return {"result": summary["result"], "error": None, "summary": summary, "seconds": seconds}
    if error is None:
        error = {"block": None, "code": "CHECK", "message": f"the Check ended unexpectedly (exit {done.returncode})"}
    return {"result": "failed", "error": {k: error.get(k) for k in ("block", "code", "message")},
            "summary": None, "seconds": seconds}


def _start(name, path):
    with _lock:
        if running():
            raise RunError(f"the program {_run['program']!r} is running — stop it first")
        cmd = [sys.executable, "-m", "blockly.runner", path, "--stop-on-eof",
               "--host", str(_settings["host"]), "--port", str(_settings["port"]), *_settings["runner_args"]]
        os.makedirs(os.path.dirname(_settings["log_path"]), exist_ok=True)
        # No run holds the file now: rotate it when it is over 2 MB, keeping 3 (it grew without a limit until
        # 2026-10-15; utility/housekeeping.py). Stamps are UTC since then, as every other log.
        rotate_by_size(_settings["log_path"], 2_000_000, keep=3)
        with open(_settings["log_path"], "a") as log:
            log.write(f"--- {time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())} run {name!r}\n")
            log.flush()
            proc = subprocess.Popen(cmd, cwd=ROOT, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                                    text=True, bufsize=1)
        _events.clear()
        _run.update(proc=proc, program=name, state="starting", block=None, error=None, exit=None,
                    started=time.time(), record=run_records.begin(name, path))   # the durable record (2026-10-15)
        _record({"type": "run", "program": name})
        threading.Thread(target=_read, args=(proc,), name="block-program-reader", daemon=True).start()
    return snapshot()


def pause():
    _send("pause")


def resume():
    _send("resume")


def step():
    _send("step")


def stop():
    """Stop the arm and the program. Always does the first part, even with no program running: a Stop button
    must never be refused."""
    stopped = _stop_robot()
    with _lock:
        proc = _run["proc"] if running() else None
    if proc is not None:
        _write(proc, "stop")
        threading.Thread(target=_escalate, args=(proc,), name="block-program-stop", daemon=True).start()
    return {"robot_stopped": stopped, "program_running": proc is not None}


def running():
    with _lock:
        return _run["proc"] is not None and _run["proc"].poll() is None and _run["exit"] is None


def snapshot():
    with _lock:
        return {"running": running(), "program": _run["program"], "state": _run["state"], "block": _run["block"],
                "error": _run["error"], "exit": _run["exit"], "seq": _run["seq"]}


def events_since(seq):
    """Events after `seq` (and `gap` if some were dropped from the buffer), with the snapshot."""
    with _lock:
        events = [e for e in _events if e["seq"] > seq]
        gap = bool(_events) and _events[0]["seq"] > seq + 1
        return {"events": events, "gap": gap, **snapshot()}


def position():
    """{"joints_deg": [...], "pose": [x, y, z (m), rx, ry, rz (rad)]} of the arm now. Raises RunError."""
    try:
        client = _client()
        return {"joints_deg": [float(v) for v in client.get_joints()],
                "pose": [float(v) for v in client.get_pose()]}
    except (RobotError, OSError) as e:
        _drop_client()
        raise RunError(f"the robot did not answer ({e})") from None


# ── inside ──────────────────────────────────────────────────────────────────────────────────────────────────────

def _record(event):
    with _lock:
        _run["seq"] += 1
        event = {"seq": _run["seq"], "t": round(time.time(), 3), **event}
        _events.append(event)
        run_records.append(_run.get("record"), event)
        if event.get("type") == "state":
            _run["state"] = event.get("state")
        elif event.get("type") == "block":
            _run["block"] = event.get("id")
        elif event.get("type") == "error":
            _run["error"] = {k: event.get(k) for k in ("block", "code", "message", "arm")}


def _read(proc):
    for line in proc.stdout:
        try:
            event = json.loads(line)
        except ValueError:
            event = {"type": "log", "level": "warning", "text": "runner: " + line.strip()[:500]}
        if isinstance(event, dict):
            _record(event)
    code = proc.wait()
    if code not in (0, 1, 2, 3) or (code == 1 and (_run["error"] or {}).get("arm") == "stopped"):
        _stop_robot()
    if code not in (0, 1, 2, 3):
        _record({"type": "error", "block": _run["block"], "code": "RUNNER_DIED",
                 "message": f"the program process ended unexpectedly (exit {code}); the robot was told to stop"})
        _record({"type": "state", "state": "error"})
    with _lock:
        _run["exit"] = code
    _record({"type": "run_end", "exit": code})
    run_records.finish(_run.get("record"), code, _run["state"], _run["error"])


def _send(word):
    with _lock:
        if not running():
            raise RunError("no program is running")
        proc = _run["proc"]
    _write(proc, word)


def _write(proc, word):
    try:
        proc.stdin.write(word + "\n")
        proc.stdin.flush()
    except (BrokenPipeError, OSError, ValueError):
        pass


def _escalate(proc):
    """A runner that has not ended after "stop": SIGTERM after 1 s, SIGKILL after 2 s."""
    for after, sig in ((STOP_TERM_S, signal.SIGTERM), (STOP_KILL_S, signal.SIGKILL)):
        deadline = time.monotonic() + after
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return
            time.sleep(0.05)
        if proc.poll() is None:
            try:
                proc.send_signal(sig)
            except OSError:
                return


def _stop_robot():
    try:
        _client().stop()
        return True
    except (RobotError, OSError):
        _drop_client()
        return False


def _client():
    with _lock:
        client = _robot["client"]
        if client is None:
            client = _settings["client_factory"]()
            client.connect(_settings["host"], _settings["port"])
            _robot["client"] = client
        return client


def _drop_client():
    with _lock:
        client, _robot["client"] = _robot["client"], None
    if client is not None:
        try:
            client.disconnect()
        except (RobotError, OSError):
            pass
