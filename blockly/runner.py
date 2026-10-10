"""Run ONE block program in its own process — TODO/blockly_plan.md §4, §7.

    python3 -m blockly.runner PROGRAM.json [--host 127.0.0.1] [--port 29000] [--stop-on-eof]

PROGRAM.json is a program file (blockly/programs.py: name, points, workspace). The runner writes the Python itself
(blockly/codegen.py — the file never carries code) and runs it.

    python3 -m blockly.runner PROGRAM.json --dry-run --start-joints 90,-105,165,0,-30,180

is a Check (blockly/check.py): the real planner, offline, from those joints; nothing is sent to the robot. It ends
with {"type": "check", "result": "passed" | "limited", "blocks", "wait_s", "moves", "motion_s"} — or an error on the
first block the robot would refuse.

stdout: one JSON object per line — blockly/runtime.py's events, plus at the end
    {"type": "error", "block": "<id>" | null, "code": "JOINT_LIMIT", "message": "...", "arm": "holding" | "stopped"}
    {"type": "state", "state": "done" | "stopped" | "error"}
stdin:  one word per line — pause | resume | step | stop. With --stop-on-eof (the editor's server passes it) the
        end of stdin means stop: the process that started this one has gone. Without it EOF is ignored, so a
        program started with no terminal keeps running.
SIGTERM / SIGINT: stop.
Exit code: 0 done, 1 failed, 2 stopped, 3 refused before it began (another program running, a program that
cannot run, a robot not ready).

Why a process of its own: user programs can loop forever or crash; a process can be stopped and takes nothing else
with it (the firmware flasher's pattern). It must not disturb the real-time loop either: it imports only the
standard library and robotclient, runs at nice 10, and stays off RTI's core.
"""
import argparse
import fcntl
import importlib
import os
import signal
import sys
import threading
import time
import traceback

from blockly import runtime as rt
from blockly.codegen import CodegenError, generate
from blockly.programs import MAX_FILE_BYTES, ProgramError, parse_program
from robotclient.exceptions import CommandRejected, MotionFailed, RobotError

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# RCB_PROGRAM_LOCK moves it (the nets use their own, so a test never blocks a real program or the reverse)
LOCK_PATH = os.environ.get("RCB_PROGRAM_LOCK") or os.path.join(ROOT, "temp_data", ".program.runlock")
RT_CORE = 3                 # RTI.py's core (SCHED_FIFO 99); a program never runs there
NICE = 10
NICE_CHECK = 19             # a Check loads and runs the planner: the lowest priority there is

EXIT_DONE, EXIT_FAILED, EXIT_STOPPED, EXIT_REFUSED = 0, 1, 2, 3


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run one block program (see the module docstring).")
    ap.add_argument("program", help="program file (JSON)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--stop-on-eof", action="store_true", help="stop when stdin closes (set by the editor's server)")
    ap.add_argument("--move-timeout", type=float, default=None,
                    help="seconds one block may RUN (paused time not counted); default robotclient's 120 s")
    ap.add_argument("--control-wait", type=float, default=None,
                    help="seconds to wait for another client to give control back (default 10)")
    ap.add_argument("--client-factory", default=None, metavar="MODULE:CALLABLE",
                    help="a callable returning the client to use instead of RobotClient (tests, dry runs)")
    ap.add_argument("--dry-run", action="store_true",
                    help="a Check: the real planner, offline (blockly/check.py) — nothing is sent to the robot")
    ap.add_argument("--start-joints", default=None, metavar="J1,...,J6",
                    help="with --dry-run: where the arm is (degrees) — the planner starts there")
    args = ap.parse_args(argv)

    _be_polite(NICE_CHECK if args.dry_run else NICE)
    if args.dry_run:            # never touches the robot: no run lock (a Check may run beside nothing else, the server
        lock = open(os.devnull)  # sees to that) and the planner's client
        factory = _load_factory(args.client_factory) or _check_factory(args.start_joints)
    else:
        lock, holder = _take_lock(os.path.basename(args.program))
        if lock is None:
            return _fail(None, "PROGRAM_RUNNING", f"another program is running ({holder})", EXIT_REFUSED)
        factory = _load_factory(args.client_factory)
    try:
        name, workspace, points = _read_program(args.program)
        code = generate(workspace, name, points)
    except CodegenError as e:
        return _fail(e.block_id, "PROGRAM", e.message, EXIT_REFUSED)
    except (OSError, ProgramError) as e:
        return _fail(None, "PROGRAM", f"the program file cannot be read: {e}", EXIT_REFUSED)

    rt.configure(host=args.host, port=args.port, move_timeout=args.move_timeout, control_wait=args.control_wait,
                 client_factory=factory, dry_run=args.dry_run)
    signal.signal(signal.SIGTERM, lambda *_: rt.stop())
    signal.signal(signal.SIGINT, lambda *_: rt.stop())
    threading.Thread(target=_control, args=(sys.stdin, args.stop_on_eof), name="stdin-control",
                     daemon=True).start()
    try:
        exec(compile(code, f"<block program {name!r}>", "exec"), {"__name__": "__block_program__"})
        _report_check(args.dry_run, "passed")
        return EXIT_DONE
    except rt.ProgramHalted:
        rt.emit({"type": "state", "state": "done"})
        _report_check(args.dry_run, "passed")
        return EXIT_DONE
    except rt.CheckLimit:
        _report_check(True, "limited")
        return EXIT_DONE
    except rt.ProgramStopped:
        rt.stop_robot_now()
        rt.emit({"type": "state", "state": "stopped"})
        return EXIT_STOPPED
    except rt.ProgramRefused as e:
        return _fail(None, "NOT_READY", str(e), EXIT_REFUSED)
    # After a failure the arm is left as an EXEC program leaves it (user, 2026-10-05: "act the same way it does for
    # exec"): a command the robot refused or failed has already stopped the program in the executor — the commands
    # behind it are ABORTED and new motion is refused until Clear Errors — and the arm HOLDS, stiff, in EXEC. So is a
    # failure of the program itself (a pose out of reach, an input that never came, 1 / 0): it happens between blocks,
    # with the arm at rest. Only where motion may still be running — a block that ran too long, a lost connection, a
    # bug of ours — is the robot stopped (which drops it to IDLE).
    except (CommandRejected, rt.ProgramFailed) as e:
        return _fail(rt.current_block(), e.code, e.message, EXIT_FAILED, arm="holding")
    except MotionFailed as e:      # the planner's own message: the block already says which command it was
        first = e.failures[0] if e.failures else {}
        return _fail(rt.current_block(), first.get("code", "MOTION_FAILED"), first.get("message") or str(e),
                     EXIT_FAILED, arm="holding")
    except (ArithmeticError, ValueError, TypeError) as e:     # the program's own arithmetic: 1 / 0, ...
        return _fail(rt.current_block(), "PROGRAM_ERROR", f"{type(e).__name__}: {e}", EXIT_FAILED, arm="holding")
    except RobotError as e:     # RobotTimeout (still moving?), RobotConnectionError: stop it
        rt.stop_robot_now()
        return _fail(rt.current_block(), type(e).__name__, str(e), EXIT_FAILED, arm="stopped")
    except Exception as e:      # a bug in the runtime or the generator: say where, stop the arm
        rt.stop_robot_now()
        where = traceback.format_exc(limit=-1).strip().splitlines()[-1]
        return _fail(rt.current_block(), "INTERNAL", f"{type(e).__name__}: {e} ({where})", EXIT_FAILED, arm="stopped")
    finally:
        rt.close()
        lock.close()        # releases the flock


def _fail(block_id, code, message, exit_code, arm=None):
    """The error event. `arm` says what the failure left: "holding" (stiff in EXEC, as an EXEC program leaves it),
    "stopped" (robot.stop() sent: IDLE), or absent (refused before anything moved)."""
    event = {"type": "error", "block": block_id, "code": str(code), "message": str(message)}
    if arm:
        event["arm"] = arm
    rt.emit(event)
    rt.emit({"type": "state", "state": "error"})
    return exit_code


def _read_program(path):
    """(name, workspace, points) — checked by blockly/programs.py, as the editor's saves are."""
    with open(path) as f:
        program = parse_program(f.read(MAX_FILE_BYTES + 1))
    name = program.get("name") or os.path.splitext(os.path.basename(path))[0]
    return name, program["workspace"], program["points"]


def _control(stream, stop_on_eof):
    commands = {"pause": rt.pause, "resume": rt.resume, "step": rt.step, "stop": rt.stop}
    for line in stream:
        fn = commands.get(line.strip())
        if fn is not None:
            fn()
    if stop_on_eof:
        rt.stop()


def _take_lock(program_name):
    """(open lock file, None), or (None, who holds it). A flock, not a PID file: the kernel drops it when the holder
    dies, however it dies. If the file cannot be created the run goes ahead unlocked — control ownership in the
    executor still keeps a second program off the arm."""
    try:
        os.makedirs(os.path.dirname(LOCK_PATH), exist_ok=True)
        f = open(LOCK_PATH, "a+")
    except OSError:
        return open(os.devnull), None
    try:
        fcntl.flock(f.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        f.seek(0)
        holder = f.read().strip() or "unknown"
        f.close()
        return None, holder
    f.seek(0)
    f.truncate()
    f.write(f"pid={os.getpid()} since={time.strftime('%Y-%m-%dT%H:%M:%S')} program={program_name}")
    f.flush()
    return f, None


def _load_factory(spec):
    if not spec:
        return None
    module, _, attr = spec.partition(":")
    return getattr(importlib.import_module(module), attr)


def _report_check(dry_run, result):
    """The end of a Check: what it went through and what the planner planned."""
    if dry_run:
        from blockly import check
        rt.emit({"type": "check", "result": result, **rt.checked(), **check.summary(rt.robot())})


def _check_factory(start_joints):
    from blockly import check
    try:
        start = [float(v) for v in (start_joints or "").split(",")]
    except ValueError:
        start = []
    if len(start) != 6:
        raise SystemExit("--dry-run needs --start-joints with six angles in degrees")
    return lambda: check.make_client(start)


def _be_polite(nice):
    """Lower priority and keep off RTI's core: memory traffic from other cores is what slows the RT loop
    (TODO/future_todo.md, 2026-10-01), so this keeps a program cheap, not free."""
    try:
        os.nice(nice)
    except OSError:
        pass
    try:
        cores = os.sched_getaffinity(0) - {RT_CORE}
        if cores:
            os.sched_setaffinity(0, cores)
    except (AttributeError, OSError):
        pass


if __name__ == "__main__":
    sys.exit(main())
