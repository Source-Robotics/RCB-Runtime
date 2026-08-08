#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
command_executor.py — EXEC-mode command hub.

Rewritten 2026-07-01 for EXEC mode (see TODO/exec_mode_implementation_checklist.md,
executive_mode_plan.md). The previous version (a blocking, one-command-at-a-time string
forwarder) is preserved at legacy/command_executor_pre_exec_rewrite.py.

Responsibilities:
  - TCP server (port 29000) speaking newline-delimited JSON: {"id","cmd","args"}.
  - Immediate commands served locally: data queries from shared memory; robot-control /
    mode commands forwarded to RTI on ZMQ_CMD_SOCKET in RTI's existing wire format.
  - Queued motion commands (`move_joints`/`move_linear`/`delay`) → in-process motion queue →
    TrajectoryPlanner thread → ZMQ trajectory batches to RTI's ExecHandler.
  - Checkpoint listener: RTI (and the planner, on errors) push completion/error status on
    ZMQ_EXEC_STATUS_SOCKET; we route each back to the TCP client that owns the command id.
  - Async by default: queued commands get {"status":"queued"} immediately; {"status":
    "complete"} follows when RTI finishes the motion.

Not on this path: the live GUI and Supervisor talk directly to RTI over ZMQ_CMD_SOCKET;
they do not go through this process. The executor↔RTI wire format is unchanged.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import json
import logging
import socket
import threading
import time
from queue import Queue, Full

import numpy as np
import zmq

from project_paths import HIGH_LEVEL_LOG, SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR
from data.shared_memory_index import (
    ISOLATED_INPUT_1, ISOLATED_INPUT_2, ISOLATED_INPUT_3,
    ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3,
)
from utility.logger_setup import setup_logging
from utility.process_utils import setup_process
from utility.RTI_utils import generate_short_id
from config.networking import (
    ZMQ_CMD_SOCKET,
    ZMQ_EXEC_STATUS_SOCKET,
)
import config.xml_parser as read_XML2
from communication.protocol import IMMEDIATE, QUEUED, kind_of, validate
from motion.trajectory_planner import TrajectoryPlanner
from motion.exec_commands import (
    MotionOptions, QueuedCommand, normalize_profile_name, normalize_ik_solver_name,
)
from motion.ik_config import DEFAULT_FK_JACOBIAN_BACKEND
from utility.activity_log import get_exec_activity_logger, log_activity

# ========================================================================== #
# Setup
# ========================================================================== #
DEBUG = True
setup_logging(str(HIGH_LEVEL_LOG), log_to_console=DEBUG)
setup_process(core=2, rt=False)  # HIGH_LEVEL: own core, normal scheduling

# Activity log — separate mechanism from the above (see TODO/activity_log_design.md).
_exec_activity_logger = get_exec_activity_logger()


def _fmt_command(name, args):
    """Raw, untouched command call text for the activity log's `command` field."""
    return f"{name}({', '.join(map(str, args))})" if args else str(name)


def _log_exec_reject(cid, code, message=None, command=None):
    log_activity(_exec_activity_logger, "EXEC", "warning", code, id=cid, command=command, message=message)

TCP_PORT = 29000
# Which hosts may open a TCP command session on port 29000. This is the ONLY thing this set
# gates — RTI's UDP state stream is unicast to whatever address the PC's TCP socket reports
# (RTIClient.connect -> robot.local_ip()), so the return path follows the real connection and
# needs no entry here. A rejected connection logs its source IP, so a wrong entry names its
# own fix rather than failing silently.
#
# DO NOT COMMIT YOUR OWN MACHINE'S ADDRESS HERE. This is an access-control list in a public
# repository: every entry tells anyone already on the network which source IPs are trusted,
# which is exactly what they would need to spoof. Add your control PC locally and keep it out
# of git — e.g. `git update-index --skip-worktree command_executor.py` while you work, or set
# it from the environment:
#
#     RCB_ALLOWED_IPS=192.168.0.42 python3 command_executor.py
import os as _os

ALLOWED_IPS = {
    "192.168.0.179",
    "192.168.1.101",
    "127.0.0.1",
}
# Extra hosts for this machine only, comma-separated. Additive on purpose: it can widen the
# allowlist for a test session without a source edit that might get committed by accident.
ALLOWED_IPS |= {ip.strip() for ip in _os.environ.get("RCB_ALLOWED_IPS", "").split(",")
                if ip.strip()}
STATUS_POLL_HZ = 50.0   # how often the watcher checks shared memory (pushes only on change)
EXEC_HEARTBEAT_HZ = 50.0  # liveness counter rate for RTI's EXEC-link watchdog (Phase 5)

MOTION_QUEUE_MAXSIZE = 256       # plan §12 decision 8 — reject rather than grow unbounded

motion_queue = Queue(maxsize=MOTION_QUEUE_MAXSIZE)   # motion commands → planner

# Digital IO — pin 1-3 (client-facing, 1-indexed) -> shared-memory gpio_states index.
# RTI reads/writes these every tick (write_all_outputs / update_gpio) regardless of
# mode; the executor only touches shared memory, never GPIO/lgpio directly.
_DIGITAL_OUTPUT_PINS = {1: ISOLATED_OUTPUT_1, 2: ISOLATED_OUTPUT_2, 3: ISOLATED_OUTPUT_3}
_DIGITAL_INPUT_PINS  = {1: ISOLATED_INPUT_1,  2: ISOLATED_INPUT_2,  3: ISOLATED_INPUT_3}

_COMPLETION_POLICY_NAMES = ("commanded", "settled", "strict")
_COMPLETION_POLICY_SET = set(_COMPLETION_POLICY_NAMES)

_motion_defaults = MotionOptions(
    profile="trapezoid",
    speed=100.0,
    blend=0.0,
    blend_mode="exact_velocity",
    ik_solver="pinocchio",
    completion_policy="settled",
)
_defaults_lock = threading.Lock()
_seq_lock = threading.Lock()
_next_seq = 1

# ========================================================================== #
# Robot config + shared memory (planner limits, immediate-command reads)
# ========================================================================== #
# Load the robot config first (carries the joint count), then attach shared memory.
_sys = read_XML2.load_system_config(SYSTEM_XML, ROBOTS_DIR)
cfg = read_XML2.load_full_robot_config(
    str(_sys['robot_xml_path']), grippers_dir=GRIPPERS_DIR,
    active_gripper=_sys['active_gripper'])
JOINT_NUM = int(cfg['joint_num'])
# Velocity and acceleration come from the ROBOT XML — <exec_limits> per joint, which is EXEC's
# own share of the hardware ceiling in <joint_*_limit>.
#
# Two corrections live in these two lines, in order:
#
# 1. Acceleration used to be recomputed here as `MAX_VEL / 0.5`, from before the XML carried an
#    acceleration limit at all. Once that field existed there were two independent sources for
#    one number: RTI read the XML, EXEC recomputed, and they matched only because the XML values
#    had been DERIVED with this same formula. Lowering the XML limit to soften the arm would
#    have softened RTI and silently done nothing to EXEC — the mode actually in use.
#
# 2. Reading the shared <joint_*_limit> then made the opposite mistake: on 2026-08-06 the
#    ceiling was opened up to tune the RTI limiter, and every queued EXEC move silently became
#    ~3.3x punchier as a side effect. <exec_limits> holds EXEC's pre-liberal values, so the mode
#    being TUNED and the mode being USED no longer move together.
#
# Still correct for a robot XML with no <exec_limits>: xml_parser falls back to the ceiling,
# which is exactly what this read before. See test_rti_mode.py sections 44 and 48.
MAX_VEL   = np.asarray(cfg['exec_velocity_limit'], dtype=np.float64)[:JOINT_NUM]
MAX_ACCEL = np.asarray(cfg['exec_acceleration_limit'], dtype=np.float64)[:JOINT_NUM]
# Jerk closes the last instance of the same pattern: motion/planner_pipeline.py computed it as
# `acceleration * RUCKIG_JERK_FACTOR` in consumer code, so EXEC ran at 28.8 rad/s^3 on joint 0
# while the robot XML declared 16000 and nothing pointed at the disagreement. Now stated in
# <exec_limits><jerk>. Read only by profile='ruckig' and ruckig_corner blending — no other
# profile bounds jerk (trapezoid/toppra STEP acceleration; quintic/s_curve have a fixed shape).
MAX_JERK  = np.asarray(cfg['exec_jerk_limit'], dtype=np.float64)[:JOINT_NUM]


def _build_robot_model():
    try:
        from utility.robot_factory import create_robot
        from utility.pinocchio_attach import attach_pinocchio
        robot = create_robot(_sys["active_robot"], gripper_name=cfg.get("gripper_name", ""))
    except Exception:
        logging.exception(
            "[EXEC] could not construct planner robot model; move_pose will be unavailable")
        return None

    try:
        attach_pinocchio(robot)
        logging.info("[EXEC] Pinocchio helpers attached to planner robot model")
    except Exception:
        logging.exception(
            "[EXEC] Pinocchio attach failed for planner robot model; "
            "move_pose with ik_solver='pinocchio' will be unavailable")
    return robot


ROBOT_MODEL = None

from data import shared_data
shared_data.setup(joints=JOINT_NUM)

# ========================================================================== #
# Shared state (thread-safe helpers)
# ========================================================================== #
_planner = None                 # TrajectoryPlanner, set in main()

active_clients = {}             # ip -> conn
_clients_lock = threading.Lock()
pending_client = {}             # command id -> conn (queued commands awaiting completion)
_pending_lock = threading.Lock()

_client_send_lock = threading.Lock()   # serialize sendall across threads
_rti_send_lock = threading.Lock()      # ZMQ PUSH socket is not thread-safe

_rti_ctx = zmq.Context.instance()
_rti_sock = _rti_ctx.socket(zmq.PUSH)
_rti_sock.setsockopt(zmq.LINGER, 200)
_rti_sock.connect(ZMQ_CMD_SOCKET)


def reply(conn, obj):
    """Send one newline-delimited JSON object to a client (thread-safe)."""
    if conn is None:
        return
    try:
        with _client_send_lock:
            conn.sendall((json.dumps(obj) + "\n").encode())
    except Exception as e:
        logging.error(f"[TCP] send to client failed: {e}")


def send_to_rti(msg_str, cid, source):
    """Forward a command to RTI in its existing wire format: {"msg","id","source"}."""
    payload = json.dumps({"msg": msg_str, "id": cid, "source": source})
    with _rti_send_lock:
        _rti_sock.send_string(payload)
    logging.info(f"[RTI] forwarded '{msg_str}' (id={cid})")


def send_payload_to_rti(payload, cid, source):
    """Forward a full command dict (not just a msg string) to RTI, merging id/source in.

    Needed by set_pid_gains, whose RTI handler reads its joint + gain fields off the command
    dict rather than parsing a colon string. `payload` must carry the "msg" key itself.
    """
    out = json.dumps({**payload, "id": cid, "source": source})
    with _rti_send_lock:
        _rti_sock.send_string(out)
    logging.info(f"[RTI] forwarded '{payload.get('msg')}' (id={cid})")


def _next_command_seq():
    global _next_seq
    with _seq_lock:
        seq = _next_seq
        _next_seq += 1
    return seq


def _get_motion_defaults():
    with _defaults_lock:
        return _motion_defaults


def _set_motion_defaults(**updates):
    global _motion_defaults
    with _defaults_lock:
        _motion_defaults = MotionOptions.from_dict(updates, _motion_defaults)
        return _motion_defaults


# Options a client may NOT set per command. Keep this in sync with what the planner actually
# consumes — a stale entry here is just as bad as the silent-drop bug it mirrors.
#
# `accel` used to sit here as "not wired yet". It IS wired (trajectory_planner scales the
# acceleration limit by it), but the blacklist was never updated, so every client call carrying
# accel= was REJECTED at this boundary and the feature was unreachable. The offline tests missed
# it because they call _plan_move_joints() directly and never cross this line. Removed 2026-07-13.
#
# `completion_policy` stays: it is a session setting, not a per-move one, and set_completion_policy()
# is the deliberate way to change it.
_UNSUPPORTED_PER_COMMAND_OPTIONS = {
    "completion_policy": "use set_completion_policy() instead of a per-command option",
}


def _reject_unsupported_per_command_options(option_overrides):
    bad = sorted(set(option_overrides or {}) & set(_UNSUPPORTED_PER_COMMAND_OPTIONS))
    if not bad:
        return
    details = "; ".join(f"{key}: {_UNSUPPORTED_PER_COMMAND_OPTIONS[key]}" for key in bad)
    raise ValueError(details)

def _build_queued_command(cid, cmd, args, option_overrides):
    _reject_unsupported_per_command_options(option_overrides)
    options = MotionOptions.from_dict(option_overrides, _get_motion_defaults())
    return QueuedCommand.build(cid, _next_command_seq(), cmd, args, options)


def _ik_solver_status_fields():
    """(engine, fk/jacobian backend) for status, honestly labeled and INDEPENDENT.
    `default_ik_solver` is the IK ENGINE (pinocchio DLS vs roboticstoolbox ik_LM);
    `default_fk_jacobian_backend` is which library computes the per-sample FK/Jacobian
    (fast pinocchio by default, regardless of engine — see motion/ik_config.py)."""
    raw = _get_motion_defaults().ik_solver
    try:
        engine = normalize_ik_solver_name(raw)
    except ValueError:
        engine = raw
    return engine, DEFAULT_FK_JACOBIAN_BACKEND


def _recipe_names():
    """Telemetry recipes present on disk, for status(). [] if the directory is unreadable.

    Globbed rather than cached: recipes are hot-reloadable (drop an XML in and hit reload),
    so a list captured at startup would go stale and advertise names `set_recipe` would then
    refuse. Status is push-on-change, not per-tick, so a 5-entry glob costs nothing here —
    it would not be acceptable on RTI's hot path.

    RTI validates `set_recipe` against ITS OWN live RecipeManager, so this list is a
    convenience for discovery, never the authority.
    """
    try:
        from project_paths import DATA_RECIPE_DIR
        return sorted(p.stem for p in DATA_RECIPE_DIR.glob("*.xml"))
    except Exception:
        return []


def _error_keys():
    """Active error keys from shared memory, e.g. ["RTI_LINK_LOST"]. [] when clean.

    error_state is a SharedString holding the JSON the GUI's Errors tab renders; we forward
    only the keys, which are self-describing and compact. Never raises — a status push must
    not fail because an error report was malformed.
    """
    try:
        data = shared_data.error_state.get_json()
        if data in (None, "NONE", []):
            return []
        return [e["key"] for e in data if isinstance(e, dict) and "key" in e]
    except Exception:
        return []


def _planner_status():
    engine, backend = _ik_solver_status_fields()
    if _planner is None:
        return {
            "executing_seq": 0,
            "completed_seq": 0,
            "queue_depth": motion_queue.qsize(),
            "active_command": None,
            "current_profile": _get_motion_defaults().profile,
            "default_ik_solver": engine,
            "default_fk_jacobian_backend": backend,
            "last_error": None,
            "last_diagnostic": None,
        }
    planner_status = _planner.status_snapshot()
    planner_status["default_ik_solver"] = engine
    planner_status["default_fk_jacobian_backend"] = backend
    return planner_status


def read_live_state():
    mode = shared_data.robot_mode.get()
    state = shared_data.robot_state.get()
    homed = shared_data.homed_state.get() == "HOMED"
    return mode, state, homed


# ========================================================================== #
# Immediate command handling
#
# One dispatch table, one handler signature (args, cid, conn). Adding a new
# immediate command (digital IO, gripper, ...) means adding one function here and
# one line in IMMEDIATE_HANDLERS — mirrors the MODE_HANDLERS pattern in
# utility/mode_dispatch.py. _forward_to_rti() covers the common "just forward and
# ack ok" shape shared by enable/disable/home/clear_errors/pause/resume/
# enter_exec_mode, so those don't need one-off functions.
# ========================================================================== #
def _handle_get_joints(args, cid, conn):
    data = np.rad2deg(shared_data.joint['joint_position'][:JOINT_NUM]).tolist()
    reply(conn, {"id": cid, "status": "ok", "data": data})


def _handle_get_pose(args, cid, conn):
    data = list(map(float, shared_data.tcp['robot_pose'][:6]))
    reply(conn, {"id": cid, "status": "ok", "data": data})


def _handle_get_status(args, cid, conn):
    mode, state, homed = read_live_state()
    planner = _planner_status()
    reply(conn, {"id": cid, "status": "ok", "data": {
        "mode": mode, "state": state, "homed": homed,
        "error_active": int(shared_data.rti['error_active'][0]),
        "safety_violation": int(shared_data.rti['safety_violation'][0]),
        "software_estop": int(shared_data.rti['software_estop'][0]),
        "exec_paused": int(shared_data.rti['exec_paused'][0]),
        "exec_settling": int(shared_data.rti['exec_settling'][0]),
        "exec_completion_policy": _COMPLETION_POLICY_NAMES[
            min(max(int(shared_data.rti['exec_completion_policy_idx'][0]), 0),
                len(_COMPLETION_POLICY_NAMES) - 1)],
        "planner": planner,
        "joints": np.rad2deg(shared_data.joint['joint_position'][:JOINT_NUM]).tolist(),
    }})


def _handle_set_joint_speed(args, cid, conn):
    pct = float(args[0]) if args else 100.0
    _set_motion_defaults(speed=pct)
    _planner.set_speed(pct)
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_joint_speed", args))


def _handle_set_blending(args, cid, conn):
    pct = float(args[0]) if args else 0.0
    _set_motion_defaults(blend=pct)
    _planner.set_blend(pct)
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_blending", args))


def _handle_set_profile(args, cid, conn):
    if not args:
        _log_exec_reject(cid, "BAD_ARGS", "set_profile needs [profile]", command="set_profile")
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS",
                            "message": "set_profile needs [profile]"})
    try:
        profile = normalize_profile_name(args[0])
    except ValueError as e:
        _log_exec_reject(cid, "BAD_PROFILE", str(e), command=_fmt_command("set_profile", args))
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_PROFILE",
                            "message": str(e)})
    _set_motion_defaults(profile=profile)
    reply(conn, {"id": cid, "status": "ok", "data": {"profile": profile}})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_profile", args))


def _handle_set_default_ik_solver(args, cid, conn):
    if not args:
        _log_exec_reject(cid, "BAD_ARGS", "set_default_ik_solver needs [solver]", command="set_default_ik_solver")
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS",
                            "message": "set_default_ik_solver needs [solver]"})
    try:
        solver = normalize_ik_solver_name(args[0])
    except ValueError as e:
        _log_exec_reject(cid, "BAD_IK_SOLVER", str(e), command=_fmt_command("set_default_ik_solver", args))
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_IK_SOLVER",
                            "message": str(e)})
    _set_motion_defaults(ik_solver=solver)
    reply(conn, {"id": cid, "status": "ok", "data": {"ik_solver": solver}})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_default_ik_solver", args))


def _handle_set_completion_policy(args, cid, conn):
    if not args:
        _log_exec_reject(cid, "BAD_ARGS", "set_completion_policy needs [policy]", command="set_completion_policy")
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS",
                            "message": "set_completion_policy needs [policy]"})
    policy = str(args[0]).strip().lower()
    if policy not in _COMPLETION_POLICY_SET:
        _log_exec_reject(cid, "BAD_POLICY", "policy must be commanded, settled, or strict",
                          command=_fmt_command("set_completion_policy", args))
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_POLICY",
                            "message": "policy must be commanded, settled, or strict"})
    _set_motion_defaults(completion_policy=policy)
    send_to_rti(f"exec_completion_policy:{policy}", cid, "TCP")
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_completion_policy", args))


def _handle_stop(args, cid, conn):
    _planner.flush()                       # drop queued motion + reset endpoint
    _fail_pending("STOPPED", "motion stopped")
    send_to_rti("set_mode:IDLE", cid, "TCP")
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command="stop")


def _handle_set_digital_output(args, cid, conn):
    if len(args) < 2:
        _log_exec_reject(cid, "BAD_ARGS", "set_digital_output needs [pin, value]", command="set_digital_output")
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS",
                            "message": "set_digital_output needs [pin, value]"})
    idx = _DIGITAL_OUTPUT_PINS.get(int(args[0]))
    if idx is None:
        _log_exec_reject(cid, "BAD_PIN", f"output pin must be 1-3, got {args[0]}",
                          command=_fmt_command("set_digital_output", args))
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_PIN",
                            "message": f"output pin must be 1-3, got {args[0]}"})
    # No RTI command needed — write_all_outputs() reads this shared-memory array and
    # writes the physical pin every tick, same as any other RTI-owned IO.
    shared_data.gpio_states[idx] = 1 if args[1] else 0
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_digital_output", args))


def _handle_get_digital_input(args, cid, conn):
    if not args:
        _log_exec_reject(cid, "BAD_ARGS", "get_digital_input needs [pin]", command="get_digital_input")
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS",
                            "message": "get_digital_input needs [pin]"})
    idx = _DIGITAL_INPUT_PINS.get(int(args[0]))
    if idx is None:
        _log_exec_reject(cid, "BAD_PIN", f"input pin must be 1-3, got {args[0]}",
                          command=_fmt_command("get_digital_input", args))
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_PIN",
                            "message": f"input pin must be 1-3, got {args[0]}"})
    reply(conn, {"id": cid, "status": "ok", "data": bool(shared_data.gpio_states[idx])})


def _forward_to_rti(rti_msg, cmd_name=None):
    """Build a handler that forwards a FIXED rti_msg to RTI as-is and acks ok."""
    display_name = cmd_name or rti_msg
    def handler(args, cid, conn):
        send_to_rti(rti_msg, cid, "TCP")
        reply(conn, {"id": cid, "status": "ok"})
        log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command=display_name)
    return handler


def _handle_enter_flashing_mode(args, cid, conn):
    """Enter FLASHING so an external flasher can own the CAN bus.

    Motors go limp here, and nothing on the robot can tell whether the pose
    survives that — so entry needs an explicit human claim:

      args: []              -> no assertion. RTI REFUSES. Deliberate: the
                               do-nothing call must not be the one that gets in.
      args: [False]         -> ":PARKED"  operator parked it and confirmed
      args: [True]          -> ":FORCE"   operator asserts it is supported
                               WITHOUT parking (a driver that lost its firmware
                               cannot be moved to park at all)

    Both claims open the gate; they are distinguished so the log records which
    was made. A malformed argument must never be the reason a safety check is
    skipped, so anything unrecognised falls back to the safer PARKED wording
    rather than FORCE.
    """
    if not args:
        send_to_rti("set_mode:FLASHING", cid, "TCP")   # RTI will refuse it
        reply(conn, {"id": cid, "status": "ok"})
        log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                     command="enter_flashing_mode (NO ASSERTION — will be refused)")
        return
    # Strict identity, not truthiness: bool("nonsense") is True, so a plain
    # bool() here would let a malformed argument select the LEAST safe path.
    # Only a literal True/1 means FORCE; everything else degrades to PARKED.
    force = args[0] is True or args[0] == 1
    send_to_rti("set_mode:FLASHING:FORCE" if force else "set_mode:FLASHING:PARKED",
                cid, "TCP")
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command="enter_flashing_mode " + ("(FORCED, NOT PARKED)" if force
                                                   else "(operator confirmed PARKED)"))


def _forward_fmt(prefix, cmd_name, argc):
    """Build a handler that forwards `prefix:arg0:arg1:...` to RTI (its colon-delimited wire
    format, e.g. jog_set:2:1:0.5). `argc` is the exact arg count; a mismatch is rejected, not
    forwarded, so a malformed call can never reach RTI as a truncated command."""
    def handler(args, cid, conn):
        if len(args) != argc:
            msg = f"{cmd_name} needs {argc} arg(s), got {len(args)}"
            _log_exec_reject(cid, "BAD_ARGS", msg, command=cmd_name)
            return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS", "message": msg})
        rti_msg = ":".join([prefix] + [str(a) for a in args])
        send_to_rti(rti_msg, cid, "TCP")
        reply(conn, {"id": cid, "status": "ok"})
        log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                     command=_fmt_command(cmd_name, args))
    return handler


# Gains RTI's _handle_set_pid_gains reads off the command dict, per joint (or the gripper).
_PID_GAIN_KEYS = ("KPP", "KPV", "KIV", "KPIQ", "KIIQ", "KP", "KD", "Ilim", "vel_lim", "vlim")


def _handle_set_pid_gains(args, cid, conn):
    """set_pid_gains carries {joint, <gain>: value, ...} as a dict (RTI reads the fields off the
    command dict, not a colon string), so forward the dict whole via send_payload_to_rti."""
    if not isinstance(args, dict) or "joint" not in args:
        msg = "set_pid_gains needs a dict with a 'joint' field and one or more gains"
        _log_exec_reject(cid, "BAD_ARGS", msg, command="set_pid_gains")
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS", "message": msg})
    payload = {"msg": "set_pid_gains", "joint": int(args["joint"])}
    for key in _PID_GAIN_KEYS:
        if key in args:
            payload[key] = float(args[key])
    send_payload_to_rti(payload, cid, "TCP")
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_pid_gains", [payload["joint"]]))


IMMEDIATE_HANDLERS = {
    "get_joints":       _handle_get_joints,
    "get_pose":         _handle_get_pose,
    "get_status":       _handle_get_status,
    "set_joint_speed":  _handle_set_joint_speed,
    "set_blending":     _handle_set_blending,
    "set_profile":      _handle_set_profile,
    "set_default_profile": _handle_set_profile,
    "set_ik_solver":   _handle_set_default_ik_solver,
    "set_default_ik_solver": _handle_set_default_ik_solver,
    "set_completion_policy": _handle_set_completion_policy,
    "stop":             _handle_stop,
    "enable":           _forward_to_rti("enable", "enable"),
    "disable":          _forward_to_rti("disable", "disable"),
    "home":             _forward_to_rti("home", "home"),
    "clear_errors":     _forward_to_rti("clear_errors", "clear_errors"),
    "enter_exec_mode":  _forward_to_rti("set_mode:EXEC", "enter_exec_mode"),
    # pause/resume route to exec_pause/exec_resume: straight to the ExecHandler
    # instance (RTI.py MODE_HANDLERS lookup), not a mode transition or a generic
    # "pause" RTI never actually handled.
    "pause":            _forward_to_rti("exec_pause", "pause"),
    "resume":           _forward_to_rti("exec_resume", "resume"),
    "set_digital_output": _handle_set_digital_output,
    "get_digital_input":  _handle_get_digital_input,
    # --- jog + live config (GUI migration Phase 1; see TODO/gui_executor_migration_plan.md) ---
    "enter_jog_mode":   _forward_to_rti("set_mode:JOG", "enter_jog_mode"),
    "enter_idle_mode":  _forward_to_rti("set_mode:IDLE", "enter_idle_mode"),
    # Firmware flashing. enter takes one optional arg: force=True appends :FORCE,
    # which bypasses RTI's parked-pose check for an operator who has physically
    # supported the arm. Exit is a plain return to IDLE.
    "safety_stop":         _forward_to_rti("set_mode:SAFETY_STOP", "safety_stop"),
    "release_safety_stop": _forward_to_rti("set_mode:IDLE", "release_safety_stop"),
    "enter_flashing_mode": _handle_enter_flashing_mode,
    "exit_flashing_mode":  _forward_to_rti("set_mode:IDLE", "exit_flashing_mode"),
    "jog_set":          _forward_fmt("jog_set", "jog_set", 3),
    "jog_stop":         _forward_to_rti("jog_stop", "jog_stop"),
    "jog_accel":        _forward_fmt("jog_accel", "jog_accel", 1),
    "jog_profile":      _forward_fmt("jog_profile", "jog_profile", 1),
    "jog_jerk_factor":  _forward_fmt("jog_jerk_factor", "jog_jerk_factor", 1),
    "jog_control_mode": _forward_fmt("jog_control_mode", "jog_control_mode", 1),
    "gravity_comp":     _forward_fmt("gravity_comp", "gravity_comp", 1),
    "set_pid_gains":    _handle_set_pid_gains,
    # --- RTI mode handshake (TODO/RTI_mode_plan.md §4d). The UDP setpoint stream
    # bypasses this process entirely and goes straight to RTI's socket; only the
    # session lifecycle is arbitrated here. rti_connect carries the PC's IP because
    # the robot cannot discover it before UDP starts.
    "rti_connect":      _forward_fmt("rti_connect", "rti_connect", 1),
    "rti_claim":        _forward_to_rti("rti_claim", "rti_claim"),
    "rti_release":      _forward_to_rti("rti_release", "rti_release"),
    "rti_disconnect":   _forward_to_rti("rti_disconnect", "rti_disconnect"),
    "rti_set_rate_limiter":     _forward_fmt("rti_set_rate_limiter", "rti_set_rate_limiter", 1),
    "rti_set_gravity_comp":     _forward_fmt("rti_set_gravity_comp", "rti_set_gravity_comp", 1),
    "set_recipe":               _forward_fmt("set_recipe", "set_recipe", 1),
    "rti_set_start_pose_check": _forward_fmt("rti_set_start_pose_check",
                                             "rti_set_start_pose_check", 1),
    "rti_set_lowpass":  _forward_fmt("rti_set_lowpass", "rti_set_lowpass", 1),
    "gripper_calibrate": _forward_to_rti("gripper_calibrate", "gripper_calibrate"),
    "gripper_ctrl_mode": _forward_fmt("gripper_ctrl_mode", "gripper_ctrl_mode", 1),
    "gripper_set":       _forward_fmt("gripper_set", "gripper_set", 4),
}


def handle_immediate(cmd, msg, cid, conn):
    handler = IMMEDIATE_HANDLERS.get(cmd)
    if handler is None:
        _log_exec_reject(cid, "UNHANDLED", f"immediate command '{cmd}' not handled", command=cmd)
        return reply(conn, {"id": cid, "status": "error", "code": "UNHANDLED",
                            "message": f"immediate command '{cmd}' not handled"})
    handler(msg.get("args", []) or [], cid, conn)


def _fail_pending(code, message):
    """Fail every queued command still awaiting completion (used on stop/error)."""
    with _pending_lock:
        for cid, conn in list(pending_client.items()):
            reply(conn, {"id": cid, "status": "error", "code": code, "message": message})
        pending_client.clear()


# ========================================================================== #
# Checkpoint listener — routes RTI/planner status back to the owning client
# ========================================================================== #
def checkpoint_listener():
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PULL)
    sock.bind(ZMQ_EXEC_STATUS_SOCKET)
    logging.info(f"[EXEC] checkpoint listener bound to {ZMQ_EXEC_STATUS_SOCKET}")

    while True:
        try:
            status = sock.recv_json()
        except Exception as e:
            logging.error(f"[EXEC] checkpoint recv error: {e}")
            continue

        cid = status.get("checkpoint_id")
        with _pending_lock:
            conn = pending_client.pop(cid, None)
        if conn is None:
            logging.info(f"[EXEC] status for unknown/closed id={cid}: {status.get('status')}")
            continue

        out = {"id": cid, "status": status.get("status", "complete")}
        if status.get("status") == "error":
            out["code"] = status.get("code", "EXEC_ERROR")
            out["message"] = status.get("message", "")
        # Gripper completions carry the settle reason (Robotiq-style: 1/2 = an
        # object stopped the jaws, 3 = reached the target with nothing there).
        # Forwarded verbatim so the client can tell a successful pick from a
        # gripper that closed on air. Absent for every non-gripper command.
        if "gripper_object" in status:
            out["gripper_object"] = status["gripper_object"]
        reply(conn, out)
        logging.info(f"[EXEC] routed {out['status']} for id={cid}")


# ========================================================================== #
# Status watcher — pushes {"type":"status",...} to all clients on state change
# (Option B: discrete status events on the one TCP socket — NOT a firehose).
# ========================================================================== #
def current_status():
    """Read the robot's discrete control state from shared memory."""
    mode, state, homed = read_live_state()
    policy_idx = int(shared_data.rti["exec_completion_policy_idx"][0])
    if policy_idx < 0 or policy_idx >= len(_COMPLETION_POLICY_NAMES):
        policy_idx = 1
    evt = {
        "type": "status",
        "mode": mode,
        "state": state,
        "homed": homed,
        "err": int(shared_data.rti["error_active"][0]),
        # The error KEYS, not just the 0/1 flag. Without these a remote PC can see THAT
        # something went wrong but never WHAT: every hard error forces ACTIVE_ERROR, which
        # looks identical from outside — an E-stop, a CAN dropout, a motor overtemp and an
        # RTI command timeout all present as "the robot stopped talking". A named error
        # field in the published state is what makes them distinguishable.
        # Push-on-change, so this costs nothing while nothing is wrong.
        "errors": _error_keys(),
        "safety": int(shared_data.rti["safety_violation"][0]),
        "estop": int(shared_data.rti["software_estop"][0]),
        "exec_paused": int(shared_data.rti["exec_paused"][0]),
        "exec_settling": int(shared_data.rti["exec_settling"][0]),
        "exec_completion_policy": _COMPLETION_POLICY_NAMES[policy_idx],
        # Which telemetry recipe the state stream is carrying, and what else is available.
        # `set_recipe` refuses an unknown name, so a PC needs a way to discover the valid
        # ones rather than guessing — and knowing the ACTIVE one is what tells a remote
        # operator why a tag they expected is missing from their packets.
        "recipe": shared_data.active_recipe.get(),
        "recipes": _recipe_names(),
    }
    evt.update(_planner_status())
    return evt


def _broadcast_status(evt):
    with _clients_lock:
        conns = list(active_clients.values())
    for conn in conns:
        reply(conn, evt)


def status_watcher():
    """Edge-triggered: compare status to the last sent; push to all clients on change."""
    period = 1.0 / STATUS_POLL_HZ
    last = None
    logging.info(f"[STATUS] watcher running ({STATUS_POLL_HZ:.0f} Hz check, push-on-change)")
    while True:
        evt = current_status()
        if evt != last:
            last = evt
            _broadcast_status(evt)
        time.sleep(period)


def executor_heartbeat():
    """Monotonic liveness counter for RTI's EXEC-link watchdog (Phase 5).

    Runs on its OWN thread — deliberately not piggybacked on status_watcher (whose
    _broadcast_status can block on a slow client) or the planner (which is busy for the whole
    duration of a long TOPPRA solve). This loop touches nothing but the counter and sleep, so it
    keeps ticking through any legitimate planner work and stalls ONLY if the whole process
    dies/wedges — which is exactly the condition RTI must detect. See
    TODO/gui_executor_migration_plan.md Phase 5.
    """
    period = 1.0 / EXEC_HEARTBEAT_HZ
    while True:
        shared_data.rti["exec_heartbeat"][0] += 1.0
        time.sleep(period)


# ========================================================================== #
# Per-client TCP handling
# ========================================================================== #
def _read_lines(conn):
    """Yield decoded newline-delimited messages from a blocking socket."""
    buf = b""
    while True:
        data = conn.recv(4096)
        if not data:
            return
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            line = line.strip()
            if line:
                yield line.decode()


def dispatch(msg, conn):
    cid = msg.get("id") or generate_short_id()
    cmd = msg.get("cmd")
    args = msg.get("args", []) or []

    if kind_of(cmd) is None:
        _log_exec_reject(cid, "UNKNOWN_CMD", f"unknown command '{cmd}'", command=_fmt_command(cmd, args))
        return reply(conn, {"id": cid, "status": "error", "code": "UNKNOWN_CMD",
                            "message": f"unknown command '{cmd}'"})

    mode, state, homed = read_live_state()
    ok, reason = validate(cmd, mode, state, homed)
    if not ok:
        _log_exec_reject(cid, "REJECTED", reason, command=_fmt_command(cmd, args))
        return reply(conn, {"id": cid, "status": "error", "code": "REJECTED",
                            "message": reason})

    if kind_of(cmd) == IMMEDIATE:
        handle_immediate(cmd, msg, cid, conn)
    else:  # QUEUED
        try:
            queued = _build_queued_command(cid, cmd, args, msg.get("options", {}) or {})
        except (TypeError, ValueError) as e:
            _log_exec_reject(cid, "BAD_OPTIONS", str(e), command=_fmt_command(cmd, args))
            return reply(conn, {"id": cid, "status": "error", "code": "BAD_OPTIONS",
                                "message": str(e)})
        try:
            motion_queue.put_nowait(queued)
        except Full:
            _log_exec_reject(cid, "QUEUE_FULL", f"motion queue full (max {MOTION_QUEUE_MAXSIZE})",
                              command=_fmt_command(cmd, args))
            return reply(conn, {"id": cid, "status": "error", "code": "QUEUE_FULL",
                                "message": f"motion queue full (max {MOTION_QUEUE_MAXSIZE})"})
        # Only register as pending once actually queued — a rejected (full-queue)
        # command must never end up waiting for a checkpoint that will never arrive.
        with _pending_lock:
            pending_client[cid] = conn
        reply(conn, {"id": cid, "status": "queued", "seq": queued.seq})
        log_activity(_exec_activity_logger, "EXEC", "info", "CMD_QUEUED", id=cid,
                     command=_fmt_command(cmd, args))


def handle_client(conn, addr):
    ip, port = addr
    with _clients_lock:
        active_clients[ip] = conn
    logging.info(f"[TCP] client connected {ip}:{port}")
    reply(conn, current_status())   # snapshot so the client isn't blind until first change
    try:
        for line in _read_lines(conn):
            try:
                msg = json.loads(line)
            except json.JSONDecodeError as e:
                reply(conn, {"status": "error", "code": "BAD_JSON", "message": str(e)})
                continue
            dispatch(msg, conn)
    except ConnectionResetError:
        logging.info(f"[TCP] {ip}:{port} reset")
    except Exception as e:
        logging.error(f"[TCP] {ip}:{port} handler error: {e}")
    finally:
        with _clients_lock:
            active_clients.pop(ip, None)
        # Drop any of this client's pending completions.
        with _pending_lock:
            for cid in [c for c, cn in pending_client.items() if cn is conn]:
                pending_client.pop(cid, None)
        conn.close()
        logging.info(f"[TCP] {ip}:{port} disconnected")


def tcp_server(host="0.0.0.0", port=TCP_PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((host, port))
        s.listen(5)
        logging.info(f"[TCP] server listening on {host}:{port}")
        while True:
            conn, addr = s.accept()
            ip, _ = addr
            if ip not in ALLOWED_IPS:
                logging.warning(f"[TCP] rejected connection from {ip}")
                try:
                    conn.sendall(b'{"status":"error","code":"FORBIDDEN"}\n')
                finally:
                    conn.close()
                continue
            threading.Thread(target=handle_client, args=(conn, addr), daemon=True).start()


# ========================================================================== #
# Main
# ========================================================================== #
def main():
    global _planner, ROBOT_MODEL
    if ROBOT_MODEL is None:
        ROBOT_MODEL = _build_robot_model()
    _planner = TrajectoryPlanner(
        motion_queue, JOINT_NUM, MAX_VEL, MAX_ACCEL,
        # Only profile='ruckig' and ruckig_corner blending bound jerk; the rest ignore it.
        # It used to be derived inside the planner as `acceleration * 3.0`, which meant the
        # <joint_jerk_limit> the XML gained on 2026-08-02 was never read by EXEC at all.
        max_jerk=MAX_JERK,
        dt=float(cfg.get('delta_t', 0.01)),
        read_current_joints=lambda: shared_data.joint['joint_position'][:JOINT_NUM],
        read_gripper_status=lambda: int(shared_data.gripper['gripper_object_detection'][0]),
        read_gripper_position=lambda: int(shared_data.gripper['gripper_position'][0]),
        # Trajectory samples RTI still has buffered. The rolling buffer holds a blended
        # move's braking tail back so a later command can be spliced in, and reads this
        # to know when RTI is about to run dry and the tail MUST go out
        # (TODO/blending_design.md §6). Written every RTI tick by ExecHandler.
        read_exec_runway=lambda: float(shared_data.rti['exec_samples_remaining'][0]),
        robot=ROBOT_MODEL,
    )
    _planner.start()

    threading.Thread(target=checkpoint_listener, daemon=True).start()
    threading.Thread(target=status_watcher, daemon=True).start()
    threading.Thread(target=executor_heartbeat, daemon=True).start()
    logging.info(f"[EXEC] command hub ready — {JOINT_NUM} joints, "
                 f"vmax={np.round(MAX_VEL, 2).tolist()} rad/s, "
                 f"amax={np.round(MAX_ACCEL, 2).tolist()} rad/s^2 (from robot XML)")
    tcp_server()


if __name__ == "__main__":
    main()
