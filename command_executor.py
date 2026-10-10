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
from pathlib import Path
import threading
import time
from queue import Queue, Full

import numpy as np
import zmq

from project_paths import HIGH_LEVEL_LOG, SYSTEM_XML, ROBOTS_DIR, GRIPPERS_DIR, CELLS_DIR, DATA_RECIPE_DIR, FILTER_CONFIG_PATH
from data.shared_memory_index import (
    ISOLATED_INPUT_1, ISOLATED_INPUT_2, ISOLATED_INPUT_3,
    ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3,
    BUTTON1, BUTTON2,
)
from utility.logger_setup import setup_logging
from utility.process_utils import setup_process
from utility.RTI_utils import generate_short_id
from config.networking import (
    ZMQ_CMD_SOCKET,
    ZMQ_EXEC_STATUS_SOCKET,
)
import config.xml_parser as read_XML2
from communication.protocol import IMMEDIATE, QUEUED, kind_of, validate, is_owned
from motion.trajectory_planner import TrajectoryPlanner
from motion.cartesian_paths import pose_from_xyz_rpy
from motion.reachability import find_reachable_clear, make_starts
from utility.zones import reach_reply, zones_summary
from utility.filter_config import build_filter_banks, parse_filter_config
from utility.payload import check_payload
from utility.haptic_walls import (damping_ratio_window, effective_inertia_range, joint_drive_torque,
                                  make_effective_inertia, resolve_joint_walls, wall_dynamics_limits, walls_public)
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

# RCB_ALLOWED_IPS="*" turns the IP check OFF: any machine that can reach port 29000 may
# connect. This is what industrial arms commonly do — they do not authenticate a control
# client, and their protocols typically have no rejection status for an unauthorized
# caller. They rely on network isolation plus EXCLUSIVE CONTROL OWNERSHIP, and
# the second half is the one that actually prevents two parties fighting over the arm.
#
# Reachable only through the environment, deliberately never as a value in this file: an
# allowlist that ships wide open is one bad merge from being everyone else's default, and
# this file is tracked in a public repo.
ALLOW_ANY_IP = "*" in ALLOWED_IPS
ALLOWED_IPS.discard("*")
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
# Safety zones of the active cell (config/cells/<active_cell>.xml; utility/zones.py). A bad cell file
# RAISES here: running without the zones the operator configured would be silently disarming them.
_GRIPPER_XML = GRIPPERS_DIR / f"{_sys['active_gripper']}.xml"
try:
    _zone_setup = read_XML2.load_zone_setup(SYSTEM_XML, CELLS_DIR, _GRIPPER_XML)
except (ValueError, FileNotFoundError) as _e:
    logging.critical(f"[EXEC] safety zones: {_e} — fix the cell file (or empty <active_cell>), then restart")
    raise
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


_self_collision_model = None      # set by _build_robot_model() when <self_collision> is enabled


def _build_self_collision(robot):
    """A SelfCollisionModel of the arm + the active gripper on `robot`'s DH chain (its OWN Pinocchio model and data:
    one per thread that uses it). Raises when the geometry files cannot be loaded."""
    from utility.self_collision import SelfCollisionModel, load_geometry
    rg, tg = load_geometry(ROBOTS_DIR, GRIPPERS_DIR, cfg)
    return SelfCollisionModel(robot, rg, tg, _sys["self_collision"],
                              cfg["self_collision"]["pairs"] + cfg["gripper_collision"]["pairs"])


def _build_robot_model():
    global _self_collision_model
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
        # Self-collision (TODO/self_collision_plan.md): the convex pieces of the arm + the active gripper, checked on every
        # planned path. ENABLED and unloadable = an error at start (the cell-file rule), never a silent "no check".
        _self_collision_model = None
        if _sys["self_collision"]["enabled"]:
            try:
                _self_collision_model = _build_self_collision(robot)
            except Exception as _e:
                logging.critical(f"[EXEC] self-collision: {_e} — fix the geometry files named in the robot / gripper XML, "
                                 f"or set <self_collision><enabled>0</enabled>, then restart")
                raise SystemExit(2)
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

active_clients = {}             # conn -> conn (a set; keyed by CONNECTION, not IP)
_clients_lock = threading.Lock()

# ====================================================================================== #
# Control ownership — TODO/control_arbitration_plan.md §4
#
# Industrial arms commonly refuse a second control client or allow one control connection, and
# authenticate nobody. What they enforce is EXCLUSIVITY, and that is the half that
# actually stops two parties fighting over an arm — which is why this matters more once the
# IP allowlist is off.
#
# Identity is the TCP CONNECTION, not the IP: two operators behind one NAT, or two scripts
# on one workstation, are different clients and must be told apart. (`active_clients` was
# keyed by IP and is now keyed by connection for the same reason.)
#
# THE RULE, non-negotiable: lock the queue, never lock the stop. See protocol.ALWAYS_OPEN.
# ====================================================================================== #
CONTROL_IDLE_TIMEOUT_S = 30.0   # released after this long with no owned command AND an
                                 # empty queue. Without it the dashboard — which holds a
                                 # persistent connection — would own the robot forever after
                                 # one button press, and every PC would have to force-take.
_owner_lock = threading.Lock()
_owner_conn = None              # the owning connection object, or None
_owner_label = None             # "ip:port", for humans and for the refusal message
_owner_since = 0.0
_owner_last = 0.0               # last owned command from the holder


def _client_label(conn):
    try:
        ip, port = conn.getpeername()[:2]
        return f"{ip}:{port}"
    except OSError:
        return "unknown"


# rti['rti_substate'] (motion/rti_handler.py SUBSTATE_CODE): 0 unpaired, 1 connected, 2 control_active,
# 3 stopping_clean, 4 stopping_error. 2 and up means a PC is DRIVING the arm right now.
_RTI_DRIVING_MIN = 2.0
_RTI_MODES = ("RTI", "SOFT_RTI")


def _rti_driving():
    """True while a PC holds an RTI/SOFT_RTI control session (or it is winding down).

    THE MODE CHECK IS ESSENTIAL. `rti_substate` is published by the RTI handler ONLY while it is
    stepping, so after a link-loss stop — the robot leaves RTI for ACTIVE_ERROR — shared memory
    keeps the last value it wrote: 4 (stopping_error), for minutes, until the next RTI session.
    The first version of this rule read that stale 4 as "still driving": the executor never
    expired the dashboard's ownership, and every script after a link loss was refused
    CONTROL_HELD (found 2026-09-30 by the loss test that caused it). Sub-state only means
    something while the robot is IN an RTI mode.

    An RTI session streams setpoints over UDP/TCP, OUTSIDE this executor: for its whole length the
    executor sees no owned command at all, so the idle rule below would expire the holder in the
    middle of a stream. Then anyone could claim, and `set_mode` / `enter_exec_mode` would pull the
    mode out from under the live stream; the streamer's own cleanup (`enter_idle_mode`) would be
    refused CONTROL_HELD. Never raises: if shared memory cannot be read, fall back to the timer.
    """
    try:
        if shared_data.robot_mode.get() not in _RTI_MODES:
            return False
        return float(shared_data.rti["rti_substate"][0]) >= _RTI_DRIVING_MIN
    except Exception:
        return False


def _owner_stale_locked():
    """Has the holder gone idle? Call with _owner_lock held."""
    if _owner_conn is None:
        return False
    # A non-empty queue means the holder is still mid-programme even if it has not spoken
    # for a while — never time out underneath a running sequence. Same for a live RTI session.
    if not motion_queue.empty() or _rti_driving():
        return False
    return (time.time() - _owner_last) > CONTROL_IDLE_TIMEOUT_S


def control_holder():
    """Label of the current holder, or None. Resolves an expired claim as it reads."""
    with _owner_lock:
        if _owner_conn is None or _owner_stale_locked():
            return None
        return _owner_label


def _claim_control(conn, force=False):
    """Take or refresh ownership. Returns (ok, message_if_refused)."""
    global _owner_conn, _owner_label, _owner_since, _owner_last
    with _owner_lock:
        if _owner_conn is not None and _owner_conn is not conn:
            if not force and not _owner_stale_locked():
                held = time.time() - _owner_since
                return False, (f"control is held by {_owner_label} (for {held:.0f}s). "
                               f"It frees automatically after "
                               f"{CONTROL_IDLE_TIMEOUT_S:.0f}s idle, or call "
                               f"take_control(force=True) to take it now.")
            logging.warning("[CTRL] %s TOOK control from %s (%s)", _client_label(conn),
                            _owner_label, "forced" if force else "previous holder idle")
        if _owner_conn is not conn:
            _owner_conn = conn
            _owner_label = _client_label(conn)
            _owner_since = time.time()
            logging.info("[CTRL] control acquired by %s", _owner_label)
        _owner_last = time.time()
        return True, None


# --- Physical confirmation for a FORCED takeover (arbitration plan 4.4) -----------------
FORCE_CONFIRM_TIMEOUT_S = 20.0   # a typical physical-access window
_FORCE_POLL_S = 0.05


# Which box button does what. Matches render.confirm_view's existing convention — B2
# confirms, B1 cancels — so the same button never means "proceed" on one safety prompt and
# "cancel" on another. If the physical layout makes it awkward, swap these two names AND the
# footer text in supervisor/ui/app.py render() ("hold B2=ALLOW  B1=DENY") — the supervisor
# does not import this module, so the screen will not follow on its own.
# test_rti_command_wiring.py fails if the two disagree.
_BTN_ALLOW = BUTTON2
_BTN_DENY = BUTTON1
FORCE_ALLOW_HOLD_S = 1.0   # ALLOW is a deliberate HOLD; DENY is an instant press.


def _button_pressed(idx=None):
    """True while that control-box button is held.

    ⚠️ INVERTED IN SHARED MEMORY. supervisor.py writes `0 if is_pressed else 1`, so 0 means
    PRESSED. Reading this the obvious way round grants on release, or grants continuously
    while nobody is touching it.
    """
    return int(shared_data.gpio_states[_BTN_ALLOW if idx is None else idx]) == 0


def _force_confirm_required():
    return float(shared_data.rti["force_confirm_enabled"][0]) >= 0.5


# ONE takeover prompt at a time. The OLED, the shared-memory flags and the buttons are a single
# physical resource: with two requests waiting at once, one press would satisfy BOTH loops, the
# screen could show a different requester from the one the operator meant to allow, and the
# first request to finish would clear the other's prompt. A second request is refused at once
# (CONFIRM_BUSY) rather than queued: an operator cannot meaningfully approve a request that is
# not the one on the screen.
_confirm_lock = threading.Lock()


def _await_physical_confirm(who, timeout=FORCE_CONFIRM_TIMEOUT_S, alive=None):
    """Block until someone at the control box ALLOWS or DENIES a forced takeover.

    Returns (ok, why). Industrial arms commonly need a physical action at the robot before one
    session can take control from another (a button on the arm, a pendant's enabling device).
    This is that gate, on
    the hardware we have.

        HOLD  B2 for FORCE_ALLOW_HOLD_S   -> allowed
        PRESS B1                          -> denied, immediately
        neither within `timeout`          -> refused

    DESIGN POINTS THAT ARE NOT OBVIOUS:

    1. **ALLOW is a hold, DENY is a tap.** Handing the arm to another machine should take a
       deliberate act; refusing should take the least effort there is. A brush against the
       box must never be able to grant control, and must always be able to refuse it.

    2. **Each button needs a release→press EDGE, not a level.** Someone leaning on a button,
       or one wedged by tape, would otherwise pre-authorise (or pre-refuse) every request the
       instant it arrived. So a button only counts once it has been seen UP during this wait.

    3. **That edge requirement is also what makes this FAIL CLOSED.** supervisor.py writes
       the buttons into shared memory at ~50 Hz. If the supervisor is dead the values never
       change, no edge can occur, and the request times out and is refused — with no
       heartbeat or liveness check needed. Failing closed is safe here precisely because it
       cannot strand anyone: `stop`/`pause`/`disable`/`clear_errors` are never gated, and an
       idle holder frees control after CONTROL_IDLE_TIMEOUT_S regardless.

    4. **Deny is checked BEFORE allow on every poll.** If both are somehow down on the same
       tick, the answer is no.

    `alive()`, if given, is polled each tick: when the requester has gone away the prompt is
    withdrawn at once instead of asking an operator to approve a request nobody is waiting on.

    This blocks for up to `timeout`, so it must NOT run on the requester's connection thread —
    see _handle_take_control, which runs it on a helper thread. On the connection thread it
    stalls every later command from that same client, including its own stop.
    """
    rti = shared_data.rti
    # monotonic, not time.time(): the Pi's wall clock can STEP when NTP syncs, and a backwards
    # step would stretch this window past the client's timeout — granting control to a caller
    # that has already given up. CLOCK_MONOTONIC is system-wide on Linux, so the supervisor
    # (data_sources.force_confirm) reads the same deadline off the same clock.
    deadline = time.monotonic() + timeout
    rti["force_confirm_pending"][0] = 1.0
    rti["force_confirm_deadline"][0] = deadline
    shared_data.force_confirm_who.set(str(who))
    logging.warning("[CTRL] %s is requesting a FORCED takeover — waiting at the control box "
                    "(hold B2 = allow, B1 = deny, %.0f s)", who, timeout)
    try:
        allow_armed = not _button_pressed(_BTN_ALLOW)   # seen UP at least once this wait
        deny_armed = not _button_pressed(_BTN_DENY)
        held_since = None
        while time.monotonic() < deadline:
            if alive is not None and not alive():
                return False, "the requesting client disconnected before anyone answered."
            deny = _button_pressed(_BTN_DENY)
            if not deny:
                deny_armed = True
            elif deny_armed:
                logging.warning("[CTRL] takeover DENIED at the control box for %s", who)
                return False, ("DENIED at the control box — someone there refused this "
                               "takeover.")

            allow = _button_pressed(_BTN_ALLOW)
            if not allow:
                allow_armed = True
                held_since = None
            elif allow_armed:
                now = time.monotonic()
                if held_since is None:
                    held_since = now
                elif now - held_since >= FORCE_ALLOW_HOLD_S:
                    logging.warning("[CTRL] takeover ALLOWED at the control box for %s", who)
                    return True, None
            time.sleep(_FORCE_POLL_S)
        if not allow_armed:
            return False, ("the ALLOW button was already held down when the request arrived, "
                           "so no deliberate hold could be registered. Release it and retry. "
                           "(If the supervisor process is not running the buttons cannot be "
                           "read at all, and a forced takeover is refused by design.)")
        return False, (f"no response at the control box within {timeout:.0f}s — forced "
                       f"takeover refused. Someone there must hold B2 to allow it.")
    finally:
        rti["force_confirm_pending"][0] = 0.0
        rti["force_confirm_deadline"][0] = 0.0
        shared_data.force_confirm_who.set("")


def _release_control(conn, why):
    """Give up ownership if `conn` holds it. Safe to call for a non-holder."""
    global _owner_conn, _owner_label
    with _owner_lock:
        if _owner_conn is not conn:
            return False
        logging.info("[CTRL] control released by %s (%s)", _owner_label, why)
        _owner_conn = None
        _owner_label = None
        return True
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


# is_reachable runs in the CLIENT's thread, so it gets its OWN robot model: pinocchio keeps one
# scratch buffer per model, and two threads sharing one returned wrong poses on 3880 of 80000 calls.
# Built on first use (0.15 s); the lock covers several clients asking at once. Its self-collision model
# (2026-10-07: a solution that self-collides is skipped for a clear branch) is its own for the same reason.
_reach = {"model": None, "starts": None, "lo": None, "hi": None, "sc": None, "sc_error": None}
_reach_lock = threading.Lock()


def _reach_model():
    """Build the is_reachable model on first use. The caller holds _reach_lock."""
    if _reach["model"] is None:
        global _self_collision_model
        planner_sc = _self_collision_model          # _build_robot_model() rebinds it; the planner's stays the planner's
        _reach["model"] = _build_robot_model()
        _self_collision_model = planner_sc
        _reach["lo"] = np.asarray(cfg['software_joint_limit_negative'], dtype=np.float64)[:JOINT_NUM]
        _reach["hi"] = np.asarray(cfg['software_joint_limit_positive'], dtype=np.float64)[:JOINT_NUM]
        _reach["starts"] = make_starts(_reach["lo"], _reach["hi"])
        if _reach["model"] is not None and _sys["self_collision"]["enabled"]:
            try:
                _reach["sc"] = _build_self_collision(_reach["model"])
            except Exception as e:                  # loadable at start, so a file changed since: refuse, never "no check"
                _reach["sc_error"] = str(e)
                logging.error(f"[EXEC] is_reachable: no self-collision model ({e})")
    return _reach["model"]


def _handle_is_reachable(args, cid, conn):
    try:
        vals = [float(v) for v in (args or [])]
    except (TypeError, ValueError):
        vals = []
    if len(vals) not in (6, 6 + JOINT_NUM) or not np.all(np.isfinite(vals)):
        msg = (f"is_reachable needs [x, y, z, rx, ry, rz] (m, rad), optionally followed by "
               f"{JOINT_NUM} 'near' joint angles (deg)")
        _log_exec_reject(cid, "BAD_ARGS", msg, command=_fmt_command("is_reachable", args))
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS", "message": msg})
    near = (np.deg2rad(vals[6:]) if len(vals) > 6
            else np.asarray(shared_data.joint['joint_position'][:JOINT_NUM], dtype=np.float64).copy())
    with _reach_lock:
        _reach_model()
        if _reach["model"] is None:
            msg = "no robot model (see the executor log)"
            _log_exec_reject(cid, "NO_ROBOT_MODEL", msg, command="is_reachable")
            return reply(conn, {"id": cid, "status": "error", "code": "NO_ROBOT_MODEL", "message": msg})
        if _reach["sc_error"] is not None:
            msg = f"self-collision checking is on but its model could not be built: {_reach['sc_error']}"
            _log_exec_reject(cid, "SELF_COLLISION_MODEL", msg, command="is_reachable")
            return reply(conn, {"id": cid, "status": "error", "code": "SELF_COLLISION_MODEL", "message": msg})
        target = pose_from_xyz_rpy(vals[:6]).A
        q, pair = find_reachable_clear(_reach["model"], target, near, _reach["lo"], _reach["hi"], _reach["starts"],
                                       _reach["sc"])
        collides = None if pair is None else _reach["sc"].describe(*pair)
    reply(conn, {"id": cid, "status": "ok", "data": reach_reply(
        None if q is None else np.rad2deg(q).tolist(), target.ravel().tolist(), _zone_setup, collides)})


def _handle_get_zones(args, cid, conn):
    """The zones in force (for scripts and the GUI). A read: open to every client."""
    reply(conn, {"id": cid, "status": "ok", "data": zones_summary(_zone_setup)})


def _handle_reload_zones(args, cid, conn):
    """Re-read <active_cell> and the cell file, and apply them here (planner, is_reachable) and in
    RTI (safety monitor) — no restart, so no re-home. A bad file is REFUSED and the zones in force stay
    as they were: a reload must never leave the robot with fewer zones than the operator believes."""
    global _zone_setup
    try:
        setup = read_XML2.load_zone_setup(SYSTEM_XML, CELLS_DIR, _GRIPPER_XML)
    except (ValueError, FileNotFoundError) as e:
        _log_exec_reject(cid, "ZONE_CONFIG", str(e), command="reload_zones")
        return reply(conn, {"id": cid, "status": "error", "code": "ZONE_CONFIG",
                            "message": f"{e} — the zones in force are unchanged"})
    _zone_setup = setup
    if _planner is not None:
        _planner.set_zones(setup["zones"], setup["tool_spheres"])
    # The parsed, validated setup travels WITH the command (2026-10-04): RTI rebuilds its zones from it
    # (config.xml_parser.zone_setup_from_wire) and reads no file in its loop.
    send_payload_to_rti({"msg": "reload_zones", "setup": zones_summary(setup)}, cid, "TCP")
    logging.info(f"[EXEC] zones reloaded: cell '{setup['cell']}', "
                 f"{sum(z['enabled'] for z in setup['zones'])} of {len(setup['zones'])} zones enabled")
    reply(conn, {"id": cid, "status": "ok", "data": zones_summary(setup)})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command="reload_zones")


def _handle_reload_filters(args, cid, conn):
    """reload_filters — parse config/filters.xml HERE and hand RTI the spec table (2026-10-04): RTI builds its
    banks from the table and reads no file in its loop (review S4). The table is built once here too, so an
    invalid combination (a median with an even window, an ema without tau, ...) is REFUSED (FILTER_CONFIG) and
    the filters in force stay — RTI validates again before swapping."""
    try:
        config = parse_filter_config(FILTER_CONFIG_PATH, JOINT_NUM)
        build_filter_banks(config, JOINT_NUM, float(cfg["delta_t"]))
    except Exception as e:                      # ET.ParseError, ValueError, KeyError, OSError: all "fix the file"
        _log_exec_reject(cid, "FILTER_CONFIG", str(e), command="reload_filters")
        return reply(conn, {"id": cid, "status": "error", "code": "FILTER_CONFIG",
                            "message": f"{FILTER_CONFIG_PATH.name}: {e} — the filters in force are unchanged"})
    send_payload_to_rti({"msg": "reload_filters", "config": config}, cid, "TCP")
    kinds = {sig: sorted({sp.get("kind", "none") for sp in specs}) for sig, specs in config.items()}
    logging.info(f"[EXEC] filters parsed and sent to RTI: {kinds}")
    reply(conn, {"id": cid, "status": "ok", "data": {"signals": kinds}})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command="reload_filters")


def _handle_reload_recipes(args, cid, conn):
    """reload_recipes — read data_recipe/*.xml HERE and hand RTI the texts (2026-10-04): RTI parses them from
    memory (RecipeManager.reload_from_texts) and never scans the directory in its loop. Unreadable directory ->
    refused (RECIPE_DIR); a bad file inside is skipped by RTI with a log line, as reload() always did."""
    try:
        files = sorted(DATA_RECIPE_DIR.glob("*.xml"))
        texts = {f.name: f.read_text(encoding="utf-8", errors="replace") for f in files}
    except OSError as e:
        _log_exec_reject(cid, "RECIPE_DIR", str(e), command="reload_recipes")
        return reply(conn, {"id": cid, "status": "error", "code": "RECIPE_DIR", "message": str(e)})
    send_payload_to_rti({"msg": "reload_recipes", "recipes": texts}, cid, "TCP")
    logging.info(f"[EXEC] {len(texts)} recipe file(s) read and sent to RTI: {sorted(texts)}")
    reply(conn, {"id": cid, "status": "ok", "data": {"files": sorted(texts)}})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command="reload_recipes")


def _handle_set_payload(args, cid, conn):
    """set_payload [mass_kg, [x, y, z]] — the part the gripper holds (utility/payload.py). Bounds checked here
    (and again in RTI): 0 <= mass <= the robot XML's <max_payload_kg>, |COM| <= 0.5 m. OWNED: it changes the
    gravity model and the walls' dynamics. The reply carries what was asked; status()["payload"] shows what RTI
    applied (shared memory), which lags this reply by a tick or two."""
    mass = args[0] if isinstance(args, (list, tuple)) and len(args) >= 1 else None
    com = args[1] if isinstance(args, (list, tuple)) and len(args) >= 2 else (0.0, 0.0, 0.0)
    if isinstance(args, (list, tuple)) and len(args) == 4:          # set_payload [m, x, y, z] also accepted
        com = args[1:4]
    ok, why = check_payload(mass, com, float(cfg['safety']['max_payload_kg']))
    if not ok:
        _log_exec_reject(cid, "PAYLOAD_LIMIT", why, command=_fmt_command("set_payload", list(args) if isinstance(args, (list, tuple)) else []))
        return reply(conn, {"id": cid, "status": "error", "code": "PAYLOAD_LIMIT", "message": why})
    payload = {"msg": "set_payload", "mass": float(mass), "com": [float(v) for v in com]}
    send_payload_to_rti(payload, cid, "TCP")
    reply(conn, {"id": cid, "status": "ok", "data": {"mass": payload["mass"], "com": payload["com"],
                                                     "max_payload_kg": float(cfg['safety']['max_payload_kg'])}})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid,
                 command=_fmt_command("set_payload", [payload["mass"], payload["com"]]))


# Freedrive joint walls (utility/haptic_walls.py JOINT WALLS, docs/safety_zones.md §2b). The executor keeps the
# LIVE specs (the robot XML at start-up, then set_joint_wall) and resolves them exactly as RTI does — the same
# function and inputs, the inertia from its own model (is_reachable's, same samples and seed as RTI). set_joint_wall
# forwards ALL specs, so one Apply also re-syncs an RTI that restarted (RTI starts from the robot XML).
_walls = {"specs": [read_XML2.wall_spec(s) for s in cfg['freedrive_walls'][:JOINT_NUM]], "inertia": None,
          "inertia_max": None, "fn": None}
_walls["saved"] = [dict(s) for s in _walls["specs"]]
_walls_lock = threading.Lock()
_HAS_ENDSTOP = np.nan_to_num(np.asarray(cfg['has_hall_sensor'], dtype=float), nan=0.0)[:JOINT_NUM] < 0.5


def _resolve_walls(specs):
    if _walls["inertia"] is None:
        with _reach_lock:
            model = _reach_model()
            _walls["fn"] = make_effective_inertia(model) if model is not None else None
            rng = effective_inertia_range(_walls["fn"], cfg['software_joint_limit_negative'],
                                          cfg['software_joint_limit_positive'])
            if rng is not None:
                _walls["inertia"], _walls["inertia_max"] = rng
    return resolve_joint_walls(specs, _zone_setup["haptic"], cfg['software_joint_limit_negative'],
                               cfg['software_joint_limit_positive'], cfg['joint_limit_negative'],
                               cfg['joint_limit_positive'], _HAS_ENDSTOP, _walls["inertia"], _walls["inertia_max"])


def _walls_reply(specs):
    walls = walls_public(_resolve_walls(specs))
    drive = joint_drive_torque(cfg)
    # The pose-dependent scale at the arm's MEASURED pose — max(1, I(q)/I_lightest), what RTI multiplies stiffness
    # and damping by right now for an ADAPTIVE joint (a fixed one stays at 1; utility/haptic_walls.py JOINT WALLS).
    # Callers hold _walls_lock, which also serialises _walls["fn"] (one Pinocchio Data).
    here = np.ones(JOINT_NUM)
    if _walls["fn"] is not None and _walls["inertia"] is not None:
        ieff = np.zeros(JOINT_NUM)
        _walls["fn"](np.asarray(shared_data.joint['joint_position'][:JOINT_NUM], dtype=np.float64).copy(), ieff)
        here = np.maximum(1.0, ieff / _walls["inertia"])
    for w, spec, saved in zip(walls, specs, _walls["saved"]):
        w["unsaved"] = spec != saved
        w["drive_torque_nm"] = float(drive[w["joint"] - 1])      # what the drive can give (display only)
        w["scale_here"] = float(here[w["joint"] - 1])
    hp = _zone_setup["haptic"]
    return {"walls": walls, "robot_xml": Path(str(_sys['robot_xml_path'])).name,
            "policy": {k: hp[k] for k in ("enabled", "joint_walls", "stiffness", "damping", "velocity_tau",
                                          "joint_max_torque", "joint_breakthrough")}}


def _check_wall_change(raw, current):
    """A client's change to one joint's wall spec, merged onto `current`. Returns (spec, None) or (None, why).
    Stricter than wall_spec (which falls back with a warning for the XML): a client gets an error instead."""
    if not isinstance(raw, dict):
        return None, "the wall spec must be an object, e.g. {\"frequency_hz\": 8.0}"
    unknown = sorted(set(raw) - set(read_XML2.WALL_SPEC_AUTO))
    if unknown:
        return None, f"unknown wall field(s) {unknown}; known: {sorted(read_XML2.WALL_SPEC_AUTO)}"
    spec = dict(current)
    auto = lambda v: v is None or str(v).strip().lower() == "auto"
    if "at" in raw:
        if not (auto(raw["at"]) or raw["at"] in ("soft", "hard")):
            return None, f"at must be soft, hard or auto, got {raw['at']!r}"
        spec["at"] = None if auto(raw["at"]) else raw["at"]
    if "freedrive" in raw:
        if raw["freedrive"] not in ("warn", "stop"):
            return None, f"freedrive must be warn or stop, got {raw['freedrive']!r}"
        spec["freedrive"] = raw["freedrive"]
    if "scaling" in raw:
        if not (auto(raw["scaling"]) or raw["scaling"] in read_XML2.WALL_SCALINGS):
            return None, f"scaling must be adaptive, fixed or auto, got {raw['scaling']!r}"
        spec["scaling"] = None if auto(raw["scaling"]) else raw["scaling"]
    ceiling = float(_zone_setup["haptic"]["joint_max_torque"])
    lo_bt, hi_bt = read_XML2._HAPTIC_BOUNDS["joint_breakthrough"]
    rules = {"frequency_hz": (lambda v: v > 0.0, "> 0"),
             "damping_ratio": (lambda v: v > 0.0, "> 0"),
             "filter_ms": (lambda v: 0.0 <= v <= read_XML2.WALL_FILTER_MAX_MS,
                           f"in [0, {read_XML2.WALL_FILTER_MAX_MS:g}] ms (the stability table's filters)"),
             "max_torque_nm": (lambda v: 0.0 < v <= ceiling, f"in (0, {ceiling:g}] (the <haptic_walls> ceiling)"),
             "breakthrough_deg": (lambda v: lo_bt <= v <= hi_bt, f"in [{lo_bt:g}, {hi_bt:g}]")}
    for key, (ok, rng) in rules.items():
        if key not in raw:
            continue
        if auto(raw[key]):
            spec[key] = None
            continue
        try:
            v = float(raw[key])
        except (TypeError, ValueError):
            return None, f"{key} must be a number or auto, got {raw[key]!r}"
        if np.isfinite(v):
            v = float(f"{v:.4g}")          # 4 significant digits: what the robot XML will hold (sliders send noise)
        if not (np.isfinite(v) and ok(v)):
            return None, f"{key} must be {rng}, got {raw[key]!r}"
        spec[key] = v
    return spec, None


def _handle_get_joint_walls(args, cid, conn):
    """The freedrive joint walls in force (live specs, resolved values, stable ranges). A read."""
    with _walls_lock:
        data = _walls_reply(_walls["specs"])
    reply(conn, {"id": cid, "status": "ok", "data": data})


def _handle_set_joint_wall(args, cid, conn):
    """set_joint_wall [joint (1-based), {field: value | "auto"}] — change one joint's freedrive wall LIVE (not saved;
    save_joint_walls writes the robot XML). A stiffness/damping outside the stable table for that joint is REFUSED
    (WALL_UNSTABLE, with the stable range) — an unstable wall buzzes and moves the arm on its own."""
    joint = args[0] if len(args) >= 1 else None
    if not (len(args) == 2 and isinstance(joint, int) and 1 <= joint <= JOINT_NUM):
        msg = f"set_joint_wall needs [joint 1..{JOINT_NUM}, {{field: value}}]"
        _log_exec_reject(cid, "BAD_ARGS", msg, command=_fmt_command("set_joint_wall", args))
        return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS", "message": msg})
    j = joint - 1
    with _walls_lock:
        spec, why = _check_wall_change(args[1], _walls["specs"][j])
        if spec is None:
            _log_exec_reject(cid, "BAD_ARGS", why, command=_fmt_command("set_joint_wall", args))
            return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS", "message": why})
        specs = [dict(s) for s in _walls["specs"]]
        specs[j] = spec
        w = _resolve_walls(specs)[j]
        why = None
        if spec["at"] == "hard" and _HAS_ENDSTOP[j]:
            why = f"J{joint} has an endstop: its hard limit IS the endstop, a wall there never engages — use soft"
        elif spec["breakthrough_deg"] is not None and spec["breakthrough_deg"] * np.pi / 180.0 > w["bt_max"] + 1e-9:
            why = (f"J{joint}: breakthrough {spec['breakthrough_deg']:g} deg is past its endstop — at most "
                   f"{np.degrees(w['bt_max']):.1f} deg past the wall")
        if why:
            why += " — nothing changed"
            _log_exec_reject(cid, "BAD_ARGS", why, command=_fmt_command("set_joint_wall", args))
            return reply(conn, {"id": cid, "status": "error", "code": "BAD_ARGS", "message": why})
        if w["note"]:
            hp = _zone_setup["haptic"]
            f_auto, _, _ = wall_dynamics_limits(hp)
            f_req = f_auto if spec["frequency_hz"] is None else spec["frequency_hz"]
            tau_req = float(hp["velocity_tau"]) if spec["filter_ms"] is None else spec["filter_ms"] / 1000.0
            zwin = damping_ratio_window(f_req, tau_req)
            at = f" with a {1000 * tau_req:g} ms filter"
            msg = (f"J{joint}: at {f_req:.3g} Hz{at} the damping ratio must be {zwin[0]:.3g}..{zwin[1]:.3g}" if zwin
                   else f"J{joint}: {f_req:.3g} Hz is above the stable maximum "
                        f"{wall_dynamics_limits(hp, tau_req)[2]:.3g} Hz{at}") + " — nothing changed"
            _log_exec_reject(cid, "WALL_UNSTABLE", msg, command=_fmt_command("set_joint_wall", args))
            return reply(conn, {"id": cid, "status": "error", "code": "WALL_UNSTABLE", "message": msg})
        if not w["enabled"]:
            msg = (f"J{joint} has no freedrive wall (joint walls off in <haptic_walls>, or no inertia from the "
                   f"model) — nothing changed")
            _log_exec_reject(cid, "WALL_OFF", msg, command=_fmt_command("set_joint_wall", args))
            return reply(conn, {"id": cid, "status": "error", "code": "WALL_OFF", "message": msg})
        _walls["specs"] = specs
        send_payload_to_rti({"msg": "set_joint_walls", "specs": specs}, cid, "TCP")
        data = _walls_reply(specs)
    logging.info(f"[EXEC] J{joint} freedrive wall set live: {spec}")
    reply(conn, {"id": cid, "status": "ok", "data": data})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command="set_joint_wall")


def _handle_save_joint_walls(args, cid, conn):
    """Write the live freedrive wall specs into the robot XML (<freedrive_wall> per joint)."""
    path = Path(str(_sys['robot_xml_path']))
    with _walls_lock:
        try:
            n = read_XML2.write_freedrive_walls(path, _walls["specs"])
        except (OSError, ValueError, read_XML2.ET.ParseError) as e:
            msg = f"could not write {path.name}: {e}"
            _log_exec_reject(cid, "SAVE_FAILED", msg, command="save_joint_walls")
            return reply(conn, {"id": cid, "status": "error", "code": "SAVE_FAILED", "message": msg})
        _walls["saved"] = [dict(s) for s in _walls["specs"]]
        data = _walls_reply(_walls["specs"])
    logging.info(f"[EXEC] freedrive walls saved to {path.name} ({n} joints)")
    reply(conn, {"id": cid, "status": "ok", "data": data})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command="save_joint_walls")


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


def _handle_clear_errors(args, cid, conn):
    """Clear RTI's error latch AND the motion abort: one recovery path for both."""
    if _planner is not None:
        _planner.clear_abort()
    send_to_rti("clear_errors", cid, "TCP")
    reply(conn, {"id": cid, "status": "ok"})
    log_activity(_exec_activity_logger, "EXEC", "info", "CMD_EXECUTED", id=cid, command="clear_errors")


# Commands that take RTI out of EXEC (RTI.py: disable -> IDLE, home -> HOMING, the rest by name).
# `stop` is deliberately absent: it cancels the program itself (flush + STOPPED), no abort.
_LEAVES_EXEC = frozenset({"disable", "home", "enter_idle_mode", "enter_jog_mode", "safety_stop",
                          "enter_flashing_mode", "rti_connect", "rti_claim"})


def _on_exec_exit(reason):
    """RTI has left (or is about to leave) EXEC.

    1. Whatever moves the arm next is not EXEC, so the next EXEC move must start from the
       MEASURED position, not the planner's last endpoint (note_exec_exit).
    2. A program still in flight cannot continue: ExecHandler.exit() discards RTI's buffered
       trajectory without reporting it, so without this its clients waited out their full
       timeout for moves that were never going to run. Abort it — the same latch as a failed
       move, cleared by clear_errors.
    """
    if _planner is None:
        return
    _planner.note_exec_exit()
    with _pending_lock:
        busy = bool(pending_client)
    if busy:
        msg = f"the program was interrupted: {reason}"
        _planner.interrupt("ABORTED", msg)
        _fail_pending("ABORTED", msg + " — clear errors to run motion again")
        log_activity(_exec_activity_logger, "EXEC", "error", "MOTION_ABORTED", message=msg)


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


def _handle_set_force_confirm(args, cid, conn):
    """set_force_confirm(enabled) — require a control-box button press for a FORCED takeover.

    Operator policy, so it lives in shared memory and the GUI owns the switch. Ships OFF: the
    feature changes who can take the arm, and that should be a deliberate choice rather than
    something that appears under an existing installation after an update.
    """
    on = bool(args and args[0])
    # Raising protection: anyone, always. Lowering it: only the control holder (or nobody
    # holding). Without that asymmetry the gate is one command deep — a remote client would
    # simply disable it and then force-take, and the button would never be asked for.
    if not on:
        holder = control_holder()
        if holder is not None and holder != _client_label(conn):
            why = (f"cannot disable the takeover confirmation while {holder} holds control. "
                   f"Raising protection is always allowed; lowering it is not.")
            _log_exec_reject(cid, "CONTROL_HELD", why, command="set_force_confirm(False)")
            return reply(conn, {"id": cid, "status": "error", "code": "CONTROL_HELD",
                                "message": why, "holder": holder})
    shared_data.rti["force_confirm_enabled"][0] = 1.0 if on else 0.0
    logging.warning("[CTRL] forced-takeover button confirmation %s", "ENABLED" if on else "DISABLED")
    _broadcast_status(current_status())
    reply(conn, {"id": cid, "status": "ok", "data": {"force_confirm": on}})


def _complete_takeover(cid, conn, force):
    """Claim control for `conn` and reply. Shared by the immediate and the confirmed path."""
    got, why = _claim_control(conn, force=force)
    if not got:
        return reply(conn, {"id": cid, "status": "error", "code": "CONTROL_HELD",
                            "message": why, "holder": control_holder()})
    if conn not in active_clients:
        # The requester hung up between the button press and this claim. handle_client has
        # already run its release, so without this the arm would sit owned by a dead socket
        # until the idle timeout.
        _release_control(conn, "requester disconnected during takeover")
        return
    _broadcast_status(current_status())     # everyone sees the new holder at once
    reply(conn, {"id": cid, "status": "ok", "data": {"holder": control_holder()}})


def _confirmed_takeover_worker(cid, conn):
    """Wait at the control box, then complete or refuse. Runs on its own thread.

    Every path must end in exactly one reply — the client is blocked on this id — so the whole
    body is guarded: an unexpected exception becomes an error reply, not a 30 s silence.
    """
    try:
        ok, why = _await_physical_confirm(_client_label(conn),
                                          alive=lambda: conn in active_clients)
        if not ok:
            _log_exec_reject(cid, "CONFIRM_REQUIRED", why, command="take_control(force)")
            return reply(conn, {"id": cid, "status": "error", "code": "CONFIRM_REQUIRED",
                                "message": why, "holder": control_holder()})
        _complete_takeover(cid, conn, force=True)
    except Exception as exc:
        logging.exception("[CTRL] confirmed takeover crashed")
        reply(conn, {"id": cid, "status": "error", "code": "INTERNAL",
                     "message": f"takeover failed: {exc}"})
    finally:
        _confirm_lock.release()


def _handle_take_control(args, cid, conn):
    """take_control([force]) — claim control explicitly, optionally taking it from a holder.

    Normally unnecessary: ownership attaches on the first owned command. This exists for the
    deliberate takeover — a wedged session, or an operator who needs the arm now — and for a
    client that wants to reserve control before it starts issuing commands.
    """
    force = bool(args and args[0])
    # Only a takeover FROM A LIVE HOLDER needs physical confirmation. Forcing an unheld robot
    # is just a claim, and gating that would make the feature an obstacle rather than a guard.
    if force and _force_confirm_required() and control_holder() is not None:
        if not _confirm_lock.acquire(blocking=False):
            msg = ("another takeover request is already waiting at the control box — "
                   "answer or wait it out, then retry.")
            _log_exec_reject(cid, "CONFIRM_BUSY", msg, command="take_control(force)")
            return reply(conn, {"id": cid, "status": "error", "code": "CONFIRM_BUSY",
                                "message": msg, "holder": control_holder()})
        # The wait is up to 20 s. handle_client reads and dispatches ONE command at a time, so
        # waiting HERE would leave every later command from this client unread — including its
        # own disable/stop. That is the GUI's exact situation: one connection carries both the
        # takeover and the Disable button. So the wait goes on its own thread and this handler
        # returns; the worker sends the reply.
        try:
            threading.Thread(target=_confirmed_takeover_worker, args=(cid, conn),
                             name="takeover-confirm", daemon=True).start()
        except Exception:
            _confirm_lock.release()    # the worker owns the release; it never started
            raise
        return
    _complete_takeover(cid, conn, force)


def _handle_release_control(args, cid, conn):
    """release_control() — give up control. Always allowed, never refused."""
    released = _release_control(conn, "explicit release")
    if released:
        _broadcast_status(current_status())
    reply(conn, {"id": cid, "status": "ok", "data": {"released": released,
                                                     "holder": control_holder()}})


IMMEDIATE_HANDLERS = {
    "get_joints":       _handle_get_joints,
    "get_pose":         _handle_get_pose,
    "get_status":       _handle_get_status,
    "is_reachable":     _handle_is_reachable,
    "get_zones":        _handle_get_zones,
    "reload_zones":     _handle_reload_zones,
    "reload_filters":   _handle_reload_filters,
    "set_payload":      _handle_set_payload,
    "get_joint_walls":  _handle_get_joint_walls,
    "set_joint_wall":   _handle_set_joint_wall,
    "save_joint_walls": _handle_save_joint_walls,
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
    "clear_errors":     _handle_clear_errors,
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
    "friction_comp":    _forward_fmt("friction_comp", "friction_comp", 1),
    "inertia_comp":     _forward_fmt("inertia_comp", "inertia_comp", 1),
    "coriolis_comp":    _forward_fmt("coriolis_comp", "coriolis_comp", 1),
    "set_pid_gains":    _handle_set_pid_gains,
    # --- RTI mode handshake (TODO/RTI_mode_plan.md §4d). The UDP setpoint stream
    # bypasses this process entirely and goes straight to RTI's socket; only the
    # session lifecycle is arbitrated here. rti_connect carries the PC's IP because
    # the robot cannot discover it before UDP starts.
    # Arity 3: pc_ip, the optional command rate, and the transport. An empty rate means
    # hard RTI; a number means SOFT_RTI and sizes the timeout. Transport is "udp" (default,
    # every existing caller) or "tcp". _forward_fmt validates EXACT arity, so the client
    # always sends all three fields rather than a variable-length call — a variable-length
    # rti_connect is how SOFT_RTI was made unreachable on its first hardware run.
    "rti_connect":      _forward_fmt("rti_connect", "rti_connect", 3),
    "rti_claim":        _forward_to_rti("rti_claim", "rti_claim"),
    "rti_release":      _forward_to_rti("rti_release", "rti_release"),
    "rti_disconnect":   _forward_to_rti("rti_disconnect", "rti_disconnect"),
    "rti_set_rate_limiter":     _forward_fmt("rti_set_rate_limiter", "rti_set_rate_limiter", 1),
    "rti_set_gravity_comp":     _forward_fmt("rti_set_gravity_comp", "rti_set_gravity_comp", 1),
    "rti_set_tcp_vel_limit":    _forward_fmt("rti_set_tcp_vel_limit", "rti_set_tcp_vel_limit", 1),
    "rti_set_command_timeout":  _forward_fmt("rti_set_command_timeout", "rti_set_command_timeout", 1),
    "rti_set_vel_scale":        _forward_fmt("rti_set_vel_scale", "rti_set_vel_scale", 1),
    "rti_set_servo_generator":  _forward_fmt("rti_set_servo_generator", "rti_set_servo_generator", 4),
    "rti_set_pace":             _forward_fmt("rti_set_pace", "rti_set_pace", 1),
    "rti_set_state_rate":       _forward_fmt("rti_set_state_rate", "rti_set_state_rate", 1),
    "reload_recipes":           _handle_reload_recipes,     # reads the files here, RTI parses from memory
    "take_control":             _handle_take_control,
    "release_control":          _handle_release_control,
    "set_force_confirm":        _handle_set_force_confirm,
    "rti_set_min_duration":     _forward_fmt("rti_set_min_duration", "rti_set_min_duration", 1),
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
        # Who holds control, or None. Published on the status stream rather than in shared
        # memory: shm needs a coordinated restart of every process plus a re-home, and this
        # event already reaches the GUI and every connected client.
        "control_holder": control_holder(),
        "force_confirm": bool(float(shared_data.rti["force_confirm_enabled"][0]) >= 0.5),
        "force_confirm_pending": bool(float(shared_data.rti["force_confirm_pending"][0]) >= 0.5),
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
        # Why queued motion is being refused, if it is: {cid, code, message} of the failure that
        # aborted the program, or None. Cleared by clear_errors.
        "motion_aborted": _planner.abort_info() if _planner is not None else None,
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
        # The two freedrive switches (IDLE): gravity comp (the arm floats) and friction compensation on top of it.
        "gravity_comp": bool(shared_data.rti["gravity_comp_enabled"][0] > 0.5),
        "friction_comp": bool(shared_data.rti["friction_comp_enabled"][0] > 0.5),
        # Inertia / Coriolis feedforward on the commanded motion (OFF after every RTI start).
        "inertia_comp": bool(shared_data.rti["inertia_comp_enabled"][0] > 0.5),
        "coriolis_comp": bool(shared_data.rti["coriolis_comp_enabled"][0] > 0.5),
        # The declared payload RTI's gravity model carries (set_payload; 0 after every RTI start).
        "payload": {"mass": float(shared_data.rti["payload_mass"][0]),
                    "com": [float(shared_data.rti["payload_com_x"][0]), float(shared_data.rti["payload_com_y"][0]),
                            float(shared_data.rti["payload_com_z"][0])]},
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
    last_mode = None
    logging.info(f"[STATUS] watcher running ({STATUS_POLL_HZ:.0f} Hz check, push-on-change)")
    while True:
        evt = current_status()
        mode = evt.get("mode")
        if last_mode == "EXEC" and mode != "EXEC":
            # RTI-initiated exits (an error, E-stop, link loss) land in ACTIVE_ERROR, which
            # holds until clear_errors — far longer than one watcher period, so none is missed.
            errs = evt.get("errors") or []
            _on_exec_exit(f"robot left EXEC (now {mode}" + (f", errors: {errs}" if errs else "") + ")")
            evt = current_status()                  # publish the abort in this same push
        last_mode = mode
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

    # CONTROL OWNERSHIP. One gate, immediately after validation, because dispatch() is the
    # only path every command takes — putting it anywhere else means a command can be added
    # later that skips it. Ownership attaches on the first OWNED command rather than needing
    # an explicit claim, so a single-client setup behaves exactly as it always did.
    if is_owned(cmd):
        got, why = _claim_control(conn)
        if not got:
            _log_exec_reject(cid, "CONTROL_HELD", why, command=_fmt_command(cmd, args))
            return reply(conn, {"id": cid, "status": "error", "code": "CONTROL_HELD",
                                "message": why, "holder": control_holder()})

    if kind_of(cmd) == IMMEDIATE:
        if cmd in _LEAVES_EXEC and mode == "EXEC":
            # Synchronous, not left to status_watcher: a script can leave EXEC and come back
            # within one watcher period, and the watcher would never see it.
            _on_exec_exit(f"'{cmd}' was sent before the program finished")
        handle_immediate(cmd, msg, cid, conn)
    else:  # QUEUED
        aborted = _planner.abort_info() if _planner is not None else None
        if aborted is not None:
            why = (f"motion is aborted: command {aborted['cid']} failed ({aborted['code']})"
                   if aborted.get("cid") else f"motion is aborted: {aborted['message']}")
            why += " — clear errors (clear_errors / the Clear Errors button) to run motion again"
            _log_exec_reject(cid, "MOTION_ABORTED", why, command=_fmt_command(cmd, args))
            return reply(conn, {"id": cid, "status": "error", "code": "MOTION_ABORTED",
                                "message": why})
        try:
            queued = _build_queued_command(cid, cmd, args, msg.get("options", {}) or {})
        except (TypeError, ValueError) as e:
            _log_exec_reject(cid, "BAD_OPTIONS", str(e), command=_fmt_command(cmd, args))
            return reply(conn, {"id": cid, "status": "error", "code": "BAD_OPTIONS",
                                "message": str(e)})
        # Register BEFORE queueing. The planner can fail a command within a millisecond of
        # picking it up (a JOINT_LIMIT target is refused before any planning), and an error for
        # an id not yet registered is dropped as "unknown" — the client then hangs until its
        # timeout. A rejected (full-queue) command is unregistered again below.
        with _pending_lock:
            pending_client[cid] = conn
        try:
            motion_queue.put_nowait(queued)
        except Full:
            with _pending_lock:
                pending_client.pop(cid, None)
            _log_exec_reject(cid, "QUEUE_FULL", f"motion queue full (max {MOTION_QUEUE_MAXSIZE})",
                              command=_fmt_command(cmd, args))
            return reply(conn, {"id": cid, "status": "error", "code": "QUEUE_FULL",
                                "message": f"motion queue full (max {MOTION_QUEUE_MAXSIZE})"})
        reply(conn, {"id": cid, "status": "queued", "seq": queued.seq})
        log_activity(_exec_activity_logger, "EXEC", "info", "CMD_QUEUED", id=cid,
                     command=_fmt_command(cmd, args))


def handle_client(conn, addr):
    ip, port = addr
    with _clients_lock:
        # Keyed by CONNECTION, not IP: two operators behind one NAT, or two scripts on one
        # workstation, are different clients. Keying by IP also silently dropped the earlier
        # connection's entry when the same host reconnected.
        active_clients[conn] = conn
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
            active_clients.pop(conn, None)
        # A dropped TCP connection must never hold the robot hostage.
        _release_control(conn, "disconnected")
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
        if ALLOW_ANY_IP:
            logging.warning("[TCP] IP ACCESS LIST DISABLED (RCB_ALLOWED_IPS=*) — any machine "
                            "that can reach %s:%d may connect. Exclusive control ownership, "
                            "not the IP list, is what stops two parties commanding at once.",
                            host, port)
        while True:
            conn, addr = s.accept()
            ip, _ = addr
            if not ALLOW_ANY_IP and ip not in ALLOWED_IPS:
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
        # Software joint limits from the robot XML — the same numbers RTI and JOG hold the arm
        # to. Passed explicitly so the check survives a robot model that failed to build.
        joint_limits=(cfg['software_joint_limit_negative'], cfg['software_joint_limit_positive']),
        zones=_zone_setup["zones"], tool_spheres=_zone_setup["tool_spheres"],
    )
    _planner.set_self_collision(_self_collision_model)
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
