"""What a generated block program calls: `from blockly import runtime as rt` — TODO/blockly_plan.md §4, §7.

A program runs ONE BLOCK AT A TIME: every block finishes physically before the next one is announced, so the block
the editor highlights is the block the arm is doing. The client is async (a move returns at the executor's
"queued" ack) and this module does the waiting, because a sync client counts paused time against its 120 s move
timeout (robotclient DEFAULT_MOVE_TIMEOUT) — a long pause would fail the program.

The hooks, in the order generated code calls them:
    robot = rt.start()   connect; refuse unless enabled, homed, error-free and not motion-aborted; take control
                         (not forced; waits up to 10 s for another client to give it back); enter EXEC.
                         Never enables or homes by itself (decision D5).
    rt.block(id)         wait for the previous block to finish, announce `id`, hold here while paused
    rt.loop()            top of every loop body: the same wait, plus a short sleep when the last iteration waited for
                         nothing, so `repeat forever {}` cannot spin a core
    rt.wait(s)           `s` seconds of RUNNING time (a pause freezes the countdown)
    rt.finish()          wait for the last block; the robot stays in EXEC (stiff, where it ended); control released

Control — pause() / resume() / step() / stop() — may come from any thread or a signal handler (the runner's stdin
reader, its SIGTERM handler). They only set flags here; the robot commands they need (pause, resume, stop) are sent
by one worker thread, never by the caller: a signal handler that sent a command itself could deadlock on the
connection's send lock if it interrupted the main thread inside a send. `stop` and `pause` are never gated by
control ownership; `resume` is (it is OWNED), so it goes over the program's own connection — this one.

Events go to `emit` — by default one JSON object per line on stdout (the runner's protocol, plan §4):
    {"type": "state", "state": "starting|running|paused|done"}   ("stopped" / "error" are the runner's)
    {"type": "block", "id": "<block id>"}
    {"type": "log", "level": "info", "text": "..."}
"""
import json
import math
import queue
import sys
import threading
import time

from robotclient import RobotClient
from robotclient.constants import DEFAULT_MOVE_TIMEOUT, DEFAULT_PORT
from robotclient.exceptions import CommandRejected, RobotError, RobotTimeout

SLICE_S = 0.1           # how often a waiting program looks at stop / pause
LOOP_GUARD_S = 0.002    # sleep per loop iteration that waited for nothing
INPUT_POLL_S = 0.05     # how often "wait until input" reads the input
CONTROL_WAIT_S = 10.0   # how long start() waits for another client to give control back (the dashboard: 5 s)
CONTROL_POLL_S = 0.5
CHECK_MAX_BLOCKS = 2000     # a Check (dry run) stops after this many blocks …
CHECK_MAX_LOOPS = 100000    # … or this many loop passes: an endless program is checked for its start


class ProgramStopped(Exception):
    """Raised at the program's next hook after stop()."""


class ProgramRefused(Exception):
    """The robot is not in a state to run a program; the message says what to do."""


class ProgramHalted(Exception):
    """The program's "end the program here" block: a normal end."""


class CheckLimit(Exception):
    """A Check (dry run) reached CHECK_MAX_BLOCKS / CHECK_MAX_LOOPS: the program may not end; what came before is checked."""


class ProgramFailed(Exception):
    """Something a block needed did not happen (an input that never came, a pose out of reach); `code` like the
    executor's error codes."""

    def __init__(self, code, message):
        super().__init__(f"[{code}] {message}")
        self.code = code
        self.message = message


_settings = {"host": "127.0.0.1", "port": DEFAULT_PORT, "move_timeout": DEFAULT_MOVE_TIMEOUT,
             "client_factory": lambda: RobotClient(sync=False, name="blocks"), "emit": None, "control_wait": CONTROL_WAIT_S,
             "dry_run": False}
_cv = threading.Condition()     # an RLock inside: stop() from a signal handler cannot deadlock on it
_flags = {"stop": False, "paused": False, "step": False, "robot_paused": False}
_run = {"robot": None, "block": None, "waited": True, "blocks": 0, "loops": 0, "dry_wait_s": 0.0}
_robot_cmds = queue.Queue()
_emit_lock = threading.Lock()


def configure(host=None, port=None, move_timeout=None, client_factory=None, emit=None, control_wait=None,
              dry_run=None):
    """Settings for the next start(): where the robot is, how long one block may run (paused time not counted),
    the client to use (a dry-run client for "Check", a fake in tests), where events go, how long to wait for
    another client to give control back, and DRY RUN (a Check, blockly/check.py): waits take no time, inputs
    arrive at once, the number of blocks / loop passes is capped."""
    for key, value in (("host", host), ("port", port), ("move_timeout", move_timeout),
                       ("client_factory", client_factory), ("emit", emit), ("control_wait", control_wait),
                       ("dry_run", dry_run)):
        if value is not None:
            _settings[key] = value


def current_block():
    """The id of the block running now (None before the first)."""
    return _run["block"]


def robot():
    """The program's client (None before start)."""
    return _run["robot"]


def emit(event):
    if _settings["emit"] is not None:
        _settings["emit"](event)
        return
    with _emit_lock:
        sys.stdout.write(json.dumps(event) + "\n")
        sys.stdout.flush()


def log(text, level="info"):
    emit({"type": "log", "level": level, "text": str(text)})


# The pose-angle convention the points and pose blocks are in (= utility.kinematics_backend.POSE_CONVENTION, which
# a net pins), checked against the executor's get_status before a program runs.
POSE_ANGLES = "xyz-mobile"


# ── the hooks generated code calls ──────────────────────────────────────────────────────────────────────────────

def start():
    emit({"type": "state", "state": "starting"})
    robot = _settings["client_factory"]()
    robot.connect(_settings["host"], _settings["port"])
    _run["robot"] = robot
    threading.Thread(target=_robot_worker, name="program-control", daemon=True).start()
    s = robot.get_status()
    if s.get("software_estop"):
        raise ProgramRefused("the E-stop is pressed")
    if s.get("error_active") or s.get("mode") == "ACTIVE_ERROR":
        raise ProgramRefused("the robot has errors: clear them first")
    if s.get("state") != "ENABLED":
        raise ProgramRefused("the robot is not enabled: enable it first")
    if not s.get("homed"):
        raise ProgramRefused("the robot is not homed: home it first")
    if s.get("pose_angles") != POSE_ANGLES:
        # 2026-10-21: the pose angles became R = Rx(rx) Ry(ry) Rz(rz). This runner converts old programs' points to
        # them; an executor from before would read those numbers the old way and turn the tool somewhere else.
        raise ProgramRefused("the robot service runs code from before the pose-angle change of 2026-10-21: restart it "
                             "(sudo systemctl restart autorun.service), then home the robot")
    if (robot.status() or {}).get("motion_aborted"):
        raise ProgramRefused("motion is aborted after a failed command: clear errors first")
    _take_control(robot)
    if s.get("mode") != "EXEC":
        robot.enter_exec_mode(wait=True)
    emit({"type": "state", "state": "running"})
    return robot


def _take_control(robot):
    """Take control, not forced. Held by someone else: wait up to control_wait seconds — the dashboard gives control
    back GUI_IDLE_RELEASE_S (5 s) after its last command, so a Run pressed right after a dashboard click would
    otherwise be refused (seen on the arm 2026-10-05) — then refuse with who holds it. A stop ends the wait."""
    deadline = time.monotonic() + _settings["control_wait"]
    said = False
    while True:
        _check_stop()
        try:
            robot.take_control()
            return
        except CommandRejected as e:
            if e.code != "CONTROL_HELD":
                raise
            if time.monotonic() >= deadline:
                raise ProgramRefused(f"another client controls the robot ({e.message})") from None
            if not said:
                log("another client has control (the dashboard gives it back 5 s after its last click) — waiting "
                    f"up to {_settings['control_wait']:g} s", "warning")
                said = True
        with _cv:
            _cv.wait(CONTROL_POLL_S)


def checked():
    """What a dry run went through: {"blocks", "wait_s"} — the runner reports it with the planner's summary."""
    return {"blocks": _run["blocks"], "wait_s": round(_run["dry_wait_s"], 2)}


def block(block_id):
    _settle()
    _run["blocks"] += 1
    if _settings["dry_run"] and _run["blocks"] > CHECK_MAX_BLOCKS:
        _run["blocks"] -= 1
        raise CheckLimit()
    _run["block"] = block_id
    emit({"type": "block", "id": block_id})
    with _cv:
        if _flags["step"]:          # the step ran the block before this one: hold before this one runs
            _flags["step"] = False
            _flags["paused"] = True
            emit({"type": "state", "state": "paused"})
    _hold()


def loop():
    _settle()
    if _settings["dry_run"]:
        _run["loops"] += 1
        if _run["loops"] > CHECK_MAX_LOOPS:
            raise CheckLimit()
        return
    if not _run["waited"]:
        time.sleep(LOOP_GUARD_S)
    _run["waited"] = False
    _hold()


def wait(seconds):
    _settle()
    if _settings["dry_run"]:            # a Check: counted, not waited
        _run["dry_wait_s"] += float(seconds)
        return
    remaining = float(seconds)
    last = time.monotonic()
    while True:
        _check_stop()
        now = time.monotonic()
        if not _flags["paused"]:
            remaining -= now - last
        last = now
        if remaining <= 0:
            break
        with _cv:
            _cv.wait(min(SLICE_S, remaining))
    _run["waited"] = True


def finish():
    _settle()
    emit({"type": "state", "state": "done"})


def halt():
    """The "end the program here" block: wait for the last block, then end normally (the runner reports done)."""
    _settle()
    raise ProgramHalted()


# ── helpers the blocks call ─────────────────────────────────────────────────────────────────────────────────────

def pose(x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg):
    """A pose block's mm / degrees -> the API's metres / radians."""
    return [x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0,
            math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)]


def shifted(base_pose, dx_mm, dy_mm, dz_mm):
    """The same pose moved along the WORLD axes."""
    p = [float(v) for v in base_pose]
    p[0] += dx_mm / 1000.0
    p[1] += dy_mm / 1000.0
    p[2] += dz_mm / 1000.0
    return p


def here(robot):
    """Where the arm is now, as a point: {joints_deg, pose}."""
    return {"joints_deg": [float(v) for v in robot.get_joints()], "pose": [float(v) for v in robot.get_pose()]}


def reach(robot, target_pose):
    """Joint angles that put the TCP on `target_pose` (the solution nearest the arm now), for a joint move to a pose.
    The planner's own IK only searches near its start; is_reachable searches the whole range."""
    r = robot.is_reachable(*target_pose)
    if not r.get("reachable"):
        raise ProgramFailed("NOT_REACHABLE", "the arm cannot reach this pose inside its joint limits")
    if r.get("self_collision"):
        raise ProgramFailed("SELF_COLLISION", f"every way the arm can reach this pose brings the {r['self_collision']} "
                                              f"too close together")
    if r.get("allowed") is False:
        raise ProgramFailed("ZONE_LIMIT", f"this pose is inside the safety zone {r.get('zone')!r}")
    return r["joints"]


def turn_joint(robot, joint, deg, speed):
    """Turn joint `joint` (1-based) by `deg` from where it is now."""
    angles = [float(v) for v in robot.get_joints()]
    angles[joint - 1] += deg
    return robot.move_joints(*angles, speed=speed)


def holding(robot):
    """True if the last gripper move stopped on something (its result, not a guess from the position)."""
    _settle()
    result = robot.gripper_result()
    return bool(result is not None and result.object_detected)


def wait_input(robot, pin, on, timeout_s):
    """Wait until input `pin` is `on`, at most `timeout_s` seconds of RUNNING time; ProgramFailed after that.
    A Check assumes it arrives at once."""
    if _settings["dry_run"]:
        return
    remaining = float(timeout_s)
    last = time.monotonic()
    while True:
        _check_stop()
        if bool(robot.get_digital_input(pin)) == on:
            break
        now = time.monotonic()
        if not _flags["paused"]:
            remaining -= now - last
        last = now
        if remaining <= 0:
            raise ProgramFailed("INPUT_TIMEOUT",
                                f"input {pin} did not turn {'on' if on else 'off'} within {timeout_s:g} s")
        with _cv:
            _cv.wait(INPUT_POLL_S)
    _run["waited"] = True


def count(start, end, step):
    """Blockly's "count with": from `start` to `end` INCLUDED, by |step|, up or down as the ends say."""
    if step == 0:
        raise ProgramFailed("PROGRAM_ERROR", "a 'count' loop's step is 0")
    step = abs(step)
    i = start
    if start <= end:
        while i <= end:
            yield i
            i += step
    else:
        while i >= end:
            yield i
            i -= step


def power(a, b):
    """a to the power b as a float: never a huge integer (which would hang the program), never a complex number."""
    return math.pow(a, b)


# ── control: any thread, or a signal handler ────────────────────────────────────────────────────────────────────

def pause():
    with _cv:
        if _flags["stop"] or _flags["paused"]:
            return
        _flags["paused"] = True
        _flags["step"] = False
        _cv.notify_all()
    _robot_cmds.put("pause")
    emit({"type": "state", "state": "paused"})


def resume():
    with _cv:
        if _flags["stop"] or not _flags["paused"]:
            return
        _flags["paused"] = False
        _cv.notify_all()
    _robot_cmds.put("resume")
    emit({"type": "state", "state": "running"})


def step():
    """While paused: run the highlighted block, hold again before the next."""
    with _cv:
        if _flags["stop"] or not _flags["paused"]:
            return
        _flags["step"] = True
        _flags["paused"] = False
        _cv.notify_all()
    _robot_cmds.put("resume")
    emit({"type": "state", "state": "running"})


def stop():
    with _cv:
        if _flags["stop"]:
            return
        _flags["stop"] = True
        _cv.notify_all()
    _robot_cmds.put("stop")


def stop_robot_now():
    """robot.stop() from the CALLER's thread — only for the runner's main thread once the program has ended (never
    from a signal handler). Best effort: the program is over either way."""
    robot = _run["robot"]
    if robot is not None:
        try:
            robot.stop()
        except (RobotError, OSError):
            pass


def close():
    """Give control back and disconnect (best effort) — the end of every run, however it ended."""
    robot = _run["robot"]
    _robot_cmds.put(None)
    if robot is None:
        return
    for call in (robot.release_control, robot.disconnect):
        try:
            call()
        except (RobotError, OSError):
            pass


# ── inside ──────────────────────────────────────────────────────────────────────────────────────────────────────

def _robot_worker():
    """Sends pause / resume / stop for the control functions, in order, from one thread."""
    while True:
        cmd = _robot_cmds.get()
        robot = _run["robot"]
        if cmd is None or robot is None:
            return
        try:
            if cmd == "pause":
                robot.pause()
                _flags["robot_paused"] = True
            elif cmd == "resume":
                if _flags["robot_paused"]:
                    robot.resume()
                    _flags["robot_paused"] = False
            elif cmd == "stop":
                robot.stop()
        except (RobotError, OSError) as e:
            log(f"{cmd} was not accepted by the robot: {e}", "warning")


def _check_stop():
    if _flags["stop"]:
        raise ProgramStopped()


def _hold():
    """Wait here while paused; stop still works."""
    with _cv:
        while _flags["paused"] and not _flags["stop"]:
            _cv.wait(SLICE_S)
    _check_stop()


def _settle():
    """Wait for the last queued command to finish. Raises what the robot reports for it (CommandRejected with the
    executor's code), ProgramStopped, or RobotTimeout after move_timeout seconds of RUNNING time.

    A command that ends STOPPED means somebody stopped the robot — the editor's Stop (whose robot.stop() reaches the
    executor BEFORE the runner reads its own "stop": found on the arm 2026-10-05), the dashboard's, any client's: the
    executor cancels pending motion that way. That ends the program as stopped, not failed."""
    robot = _run["robot"]
    running_s = 0.0
    while True:
        _check_stop()
        try:
            robot.wait_motion_complete(timeout=SLICE_S)
            return
        except CommandRejected as e:
            if e.code != "STOPPED":
                raise
            if not _flags["stop"]:
                log("the robot was stopped from outside the program (a Stop button or another client)", "warning")
            raise ProgramStopped() from None
        except RobotTimeout:
            _run["waited"] = True
            if not _flags["paused"]:
                running_s += SLICE_S
            if running_s > _settings["move_timeout"]:
                raise RobotTimeout(f"the block did not finish within {_settings['move_timeout']:g} s") from None
