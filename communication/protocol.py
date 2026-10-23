"""Command protocol for the EXEC command hub (Phase D).

Client ↔ command_executor speak newline-delimited JSON over TCP:

    Client → executor:   {"id": "aB3xYz", "cmd": "move_joints", "args": [0,-60,60,0,0,0]}
    executor → client:   {"id": "aB3xYz", "status": "queued"}
                         {"id": "aB3xYz", "status": "complete"}
                         {"id": "hJ5bDs", "status": "ok", "data": [...]}
                         {"id": "aB3xYz", "status": "error", "code": "...", "message": "..."}

This module owns the command classification and the validation rules — it is pure data +
functions, importable by both the executor and (later) tests without side effects.
"""

# The command protocol's version (2026-10-21, versioning V3). Raise it ONLY for an incompatible change of this protocol:
# a command renamed or removed, an argument or a reply field that changes its meaning or unit, a changed default. A new
# command, a new optional argument, a new reply field is NOT a new version (an older client never sends or reads them).
# robotclient/version.py keeps the client's copy (pinned equal by dummy_test_scripts/test_protocol_versions.py); the
# executor publishes it in every status message, and a client refuses an executor whose number differs.
PROTOCOL_VERSION = 1

# The commands whose arguments carry pose ANGLES (2026-10-21, with versioning V5). Pose angles changed their meaning
# on 2026-10-21 (R = Rx(rx) Ry(ry) Rz(rz)), and a client from before then sends no "protocol" key: the executor refuses
# these commands from such a connection (CLIENT_TOO_OLD) rather than read its angles the new way and turn the tool
# elsewhere. is_reachable is here because its joint answer is fed straight into a joint move. Reads (get_pose) are not:
# they move nothing. A circle takes a centre and a normal vector, no angles.
POSE_COMMANDS = frozenset({
    "move_pose", "move_linear", "move_linear_rel_world", "move_linear_rel_tool",
    "move_arc", "move_arc_rel_world", "move_arc_rel_tool",
    "move_spline", "move_spline_rel_world", "move_spline_rel_tool",
    "is_reachable",
})

# Command kinds
IMMEDIATE = "immediate"   # executed now: served from shared memory or forwarded to RTI
QUEUED = "queued"         # placed on the motion queue → planner → RTI

# --- Control ownership -----------------------------------------------------------------
#
# ONE rule, from TODO/control_arbitration_plan.md §4.1: **lock the queue, never lock the
# stop.** If arbitration can lock someone out of stopping the robot, the safety feature has
# become the hazard.
#
# This is an allowlist of commands that are NEVER gated, and the polarity is deliberate:
# `is_owned()` DEFAULTS TO TRUE, so a command added later is protected by default rather
# than silently unprotected because nobody remembered to classify it. Getting that backwards
# is how an arbitration layer quietly stops arbitrating.
#
# What earns a place here: reads, and actions that make the robot SAFER. Nothing that moves
# it, arms it, or changes how it will behave when it does move.
ALWAYS_OPEN = frozenset({
    # reads — monitoring is never exclusive, on any robot
    "get_joints", "get_pose", "get_status", "get_digital_input", "is_reachable", "get_zones",
    # Diagnostics (2026-10-15): move nothing, change no limit — a health query, a logger's level, a telemetry
    # capture into a file on the robot.
    "get_health", "set_log_level", "capture_telemetry", "stop_capture",
    "get_joint_walls",
    # The first-run check's data (2026-10-18): what the drives, the CAN bus and the box's inputs report.
    "get_hardware",
    # Every part's version (2026-10-21): the runtime, the OS, the description package, the box, each drive.
    "get_versions",
    # make-it-safer — anyone, always, no exceptions
    "stop", "pause", "disable", "safety_stop", "clear_errors",
    # A jog has NO timeout: the GUI sends one jog on mouse-down and one jog_stop on
    # mouse-up, and the arm moves for as long as that takes. If control changes hands
    # mid-jog (forced takeover, or the holder's 30 s idle expiry during a long button hold),
    # the jogger's release-stop would be refused CONTROL_HELD and the arm would keep going
    # until somebody thought to send `stop`. Found 2026-09-30 while building the dashboard
    # surface: the rule below was already written in the jog_stop protocol comment
    # ("a released jog button always halts motion") and the gate contradicted it.
    "jog_stop",
    # Giving up control is always allowed. `take_control` is here too, but NOT because it is
    # harmless — because routing it through the gate would make a forced takeover impossible:
    # you would need control in order to take control. It does its own claim, with the force
    # flag, and is the ONLY way past a live holder.
    "release_control", "take_control",
    # ASYMMETRIC, and the asymmetry is the whole point. RAISING protection must always be
    # allowed: the operator standing at the robot has to be able to turn on the button gate
    # while a remote machine holds the arm — that is exactly when they need it. LOWERING it
    # is refused to a non-holder inside the handler, because "disable the gate, then force"
    # would otherwise bypass physical confirmation in one command by anyone on the network.
    "set_force_confirm",
})


# Other IDs a client may send for a command: the names before the naming pass of 2026-10-23, still accepted so that
# every existing program and client keeps working. The executor maps an alias to its command the moment a message
# arrives, before any check, so both spellings get the same rules, the same handler and the same log line. The
# robotclient keeps the old Python names as aliases of the same functions. A new command needs no entry here.
COMMAND_ALIASES = {
    "move_linear_rel_wrf": "move_linear_rel_world",
    "move_arc_rel_wrf": "move_arc_rel_world",
    "move_circle_rel_wrf": "move_circle_rel_world",
    "move_spline_rel_wrf": "move_spline_rel_world",
    "move_linear_rel_trf": "move_linear_rel_tool",
    "move_arc_rel_trf": "move_arc_rel_tool",
    "move_circle_rel_trf": "move_circle_rel_tool",
    "move_spline_rel_trf": "move_spline_rel_tool",
    "gripper_open": "open_gripper",
    "gripper_close": "close_gripper",
    "gripper_calibrate": "calibrate_gripper",
    "jog_set": "jog",
    "jog_accel": "set_jog_accel",
    "jog_profile": "set_jog_profile",
    "jog_jerk_factor": "set_jog_jerk_factor",
    "jog_control_mode": "set_jog_control_mode",
    "gravity_comp": "set_gravity_comp",
    "friction_comp": "set_friction_comp",
    "inertia_comp": "set_inertia_comp",
    "coriolis_comp": "set_coriolis_comp",
    "friction_ff": "set_friction_ff",
    "can_read_first": "set_can_read_first",
    "collision_detection": "set_collision_detection",
    "gripper_ctrl_mode": "set_gripper_ctrl_mode",
    "gripper_set": "set_gripper",
    "set_default_profile": "set_profile",
    "set_default_ik_solver": "set_ik_solver",
}


def canonical_command(cmd):
    """The command an ID names: an alias's command, else the ID itself."""
    return COMMAND_ALIASES.get(cmd, cmd)


def is_owned(cmd):
    """True if `cmd` requires control ownership. Unknown/new commands are OWNED."""
    return canonical_command(cmd) not in ALWAYS_OPEN

# cmd → (kind, required_mode | None, required_state | None, requires_homed)
#   required_mode: list of allowed RobotMode names, or None = any mode
#   required_state: exact robot_state string ("ENABLED"), or None = any
COMMAND_RULES = {
    # --- data queries (always allowed, read-only) ---
    "get_joints":     (IMMEDIATE, None, None, False),
    "get_pose":       (IMMEDIATE, None, None, False),
    "get_status":     (IMMEDIATE, None, None, False),
    # --- diagnostics (2026-10-15; docs/logging.md) ---
    "get_health":        (IMMEDIATE, None, None, False),
    "set_log_level":     (IMMEDIATE, None, None, False),
    "capture_telemetry": (IMMEDIATE, None, None, False),
    "stop_capture":      (IMMEDIATE, None, None, False),
    "get_hardware":      (IMMEDIATE, None, None, False),   # utility/hardware_report.py (2026-10-18)
    "get_versions":      (IMMEDIATE, None, None, False),   # utility/versions.py (2026-10-21)
    # IK only, moves nothing — any mode, any state, homed or not (`near` defaults to the measured
    # joints, which only picks WHICH solution is returned, never whether one exists).
    "is_reachable":   (IMMEDIATE, None, None, False),
    # Safety zones of the active cell (docs/safety_zones.md). get_zones is a read; reload_zones changes
    # how the robot will behave, so it is OWNED (not in ALWAYS_OPEN). Any mode: zones can be edited and
    # applied while the robot is disabled or in an error.
    "get_zones":      (IMMEDIATE, None, None, False),
    "reload_zones":   (IMMEDIATE, None, None, False),
    # Freedrive joint walls (docs/safety_zones.md §2b, the dashboard's Walls tab). get_joint_walls is a read;
    # set_joint_wall (live) and save_joint_walls (robot XML) change how freedrive feels, so they are OWNED.
    "get_joint_walls":  (IMMEDIATE, None, None, False),
    "set_joint_wall":   (IMMEDIATE, None, None, False),
    "save_joint_walls": (IMMEDIATE, None, None, False),
    # --- robot control (always allowed) ---
    "enable":        (IMMEDIATE, None, None, False),
    "disable":       (IMMEDIATE, None, None, False),
    "home":          (IMMEDIATE, None, None, False),
    "clear_errors":   (IMMEDIATE, None, None, False),
    "stop":          (IMMEDIATE, None, None, False),
    # pause/resume only mean something mid-EXEC-motion — reject cleanly elsewhere
    # instead of silently forwarding a command RTI has nothing to apply it to.
    "pause":         (IMMEDIATE, ["EXEC"], None, False),
    "resume":        (IMMEDIATE, ["EXEC"], None, False),
    # --- mode + planner config ---
    # EXEC and JOG are reached ONLY from IDLE (utility/state_machine.py VALID_TRANSITIONS). Asked for
    # from any other mode, RTI refuses the switch and the client just waits out its timeout, so refuse
    # it here, with the way through. "Already there" stays allowed: a script calling enter_exec_mode()
    # defensively while in EXEC must not start raising.
    "enter_exec_mode": (IMMEDIATE, ["IDLE", "EXEC"], "ENABLED", True),   # needs enabled + homed
    "set_joint_speed": (IMMEDIATE, None, None, False),        # planner speed scale
    "set_blending":   (IMMEDIATE, None, None, False),        # planner blend (Phase F)
    "set_profile":    (IMMEDIATE, None, None, False),        # default planner profile
    "set_ik_solver": (IMMEDIATE, None, None, False),         # default Cartesian IK solver
    "set_completion_policy": (IMMEDIATE, None, None, False), # EXEC checkpoint completion
    # --- digital IO (isolated pins 1-3; RTI reads/writes these every tick regardless
    # of mode, same as the physical E-stop) ---
    "set_digital_output": (IMMEDIATE, None, None, False),
    "get_digital_input":  (IMMEDIATE, None, None, False),
    # --- jog + live config (migrated from the GUI's old direct-to-RTI push;
    #     see TODO/gui_executor_migration_plan.md) ---
    # Mode entry: JOG mirrors enter_exec_mode's rule — the state machine's REQUIRES_HOMED
    # set contains JOG, and every non-IDLE transition needs ENABLED. IDLE is always allowed.
    "enter_jog_mode":  (IMMEDIATE, ["IDLE", "JOG"], "ENABLED", True),
    "enter_idle_mode": (IMMEDIATE, None, None, False),
    # --- firmware flashing (docs/firmware_flashing.md) ---
    # FLASHING hands the CAN bus to an external flasher while RTI keeps ticking.
    # Deliberately NOT gated on ENABLED or homed here: the state machine treats it
    # as a MAINTENANCE mode precisely so a disabled, unhomed or faulted robot can
    # still be serviced — a driver stuck in its bootloader shows up as a
    # disconnected node, i.e. an active error. The gate that DOES apply is the
    # parked-pose check, which only RTI can evaluate (it needs live gravity
    # torque), so it is enforced there rather than duplicated here.
    # Motors driven by NOTHING: Position=None, Speed=None, Current=0. The arm
    # goes limp and WILL fall under gravity. No mode/state gate — the one
    # command guaranteed to stop the machine acting must never be refusable.
    "safety_stop":       (IMMEDIATE, None, None, False),
    "release_safety_stop": (IMMEDIATE, None, None, False),
    "enter_flashing_mode": (IMMEDIATE, None, None, False),
    "exit_flashing_mode":  (IMMEDIATE, None, None, False),
    # Jog actuation: START only in JOG mode (RTI ignores its jog_set outside JOG anyway). STOP is
    # allowed in any mode so a released jog button always halts motion, even if a mode change raced.
    "jog":         (IMMEDIATE, ["JOG"], None, False),
    "jog_stop":        (IMMEDIATE, None, None, False),
    # Jog configuration — persistent on RTI's JOG handler, applied regardless of current mode.
    "set_jog_accel":        (IMMEDIATE, None, None, False),
    "set_jog_profile":      (IMMEDIATE, None, None, False),
    "set_jog_jerk_factor":  (IMMEDIATE, None, None, False),
    "set_jog_control_mode": (IMMEDIATE, None, None, False),
    # Runtime config relays — RTI-owned state, any mode.
    "set_gravity_comp":     (IMMEDIATE, None, None, False),
    "set_friction_comp":    (IMMEDIATE, None, None, False),   # freedrive friction compensation switch (OWNED)
    "set_inertia_comp":     (IMMEDIATE, None, None, False),   # inertia feedforward switch (OWNED, 2026-10-10)
    "set_coriolis_comp":    (IMMEDIATE, None, None, False),   # Coriolis feedforward switch (OWNED, 2026-10-10)
    "set_friction_ff":      (IMMEDIATE, None, None, False),   # friction feedforward switch (OWNED, 2026-10-17)
    "set_can_read_first":   (IMMEDIATE, None, None, False),   # RTI reads CAN first, sends early (option B; OWNED, 2026-10-12)
    "set_collision_detection": (IMMEDIATE, None, None, False),  # off | warn | stop until RTI restarts (OWNED, 2026-10-16)
    # The Hall sensor check (robotics/hall_check.py, 2026-10-13): the Hall-sensor joint limp, its pin shown live. IDLE
    # only, enabled or not, homed or not (OWNED); RTI stops it itself before any mode change.
    "hall_check":       (IMMEDIATE, ["IDLE"], None, False),
    # One joint limp for hand turning (robotics/free_joint.py, 2026-10-18; the first-run check): IDLE only, enabled or
    # not, homed or not (OWNED); RTI holds it again itself before any mode change and after 2 minutes.
    "free_joint":       (IMMEDIATE, ["IDLE"], None, False),
    # A small powered test move of one joint, homed or not (robotics/joint_test.py, 2026-10-18; the first-run check's
    # stage 3): IDLE and ENABLED (OWNED); RTI stops it itself on a mode change, a disable, or a joint falling behind.
    "joint_test":       (IMMEDIATE, ["IDLE"], "ENABLED", False),
    "set_pid_gains":    (IMMEDIATE, None, None, False),
    # --- RTI mode session control (TODO/RTI_mode_plan.md §3). Only the handshake goes
    # through here; the UDP setpoint stream never touches command_executor.
    # rti_connect mirrors enter_exec_mode/enter_jog_mode — it requests a mode the state
    # machine gates on ENABLED + homed, so reject here rather than let RTI refuse it later.
    # The other three act on an already-paired session and RTI validates the sub-state,
    # so they stay mode-agnostic: a release/disconnect must work even mid-error, which is
    # exactly when you most need to hand control back.
    "rti_connect":    (IMMEDIATE, None, "ENABLED", True),
    # BOTH RTI contracts. Gating on "RTI" alone made SOFT_RTI enterable but unusable --
    # the same "a legal transition is not a reachable one" failure as FLASHING (2026-08-04),
    # and the SECOND copy of it: the identical gate in RTI.py's _handle_rti_claim was fixed
    # during the mode split and this one was missed, because nothing tested the two layers
    # together. Found on hardware, first bring-up run.
    "rti_claim":      (IMMEDIATE, ["RTI", "SOFT_RTI"], "ENABLED", True),
    "rti_release":    (IMMEDIATE, None, None, False),
    "rti_disconnect": (IMMEDIATE, None, None, False),
    # Safety-layer toggles (plan §11c). Settable any time; they only take effect on the
    # NEXT claim for the start-pose check, immediately for the rate limiter.
    "rti_set_rate_limiter":     (IMMEDIATE, None, None, False),
    "rti_set_gravity_comp":     (IMMEDIATE, None, None, False),
    "rti_set_dynamics_ff":      (IMMEDIATE, None, None, False),
    "rti_set_tcp_vel_limit":    (IMMEDIATE, None, None, False),
    "rti_set_command_timeout":  (IMMEDIATE, None, None, False),
    "rti_set_vel_scale":        (IMMEDIATE, None, None, False),
    "rti_set_servo_generator":  (IMMEDIATE, None, None, False),
    "rti_set_pace":             (IMMEDIATE, None, None, False),
    "rti_set_state_rate":       (IMMEDIATE, None, None, False),
    "reload_recipes":           (IMMEDIATE, None, None, False),
    # Filters: command_executor parses config/filters.xml and hands RTI the spec table (2026-10-04); RTI reads no
    # file in its loop. Open like reload_recipes: an operator edits the file and asks for it to be applied.
    "reload_filters":           (IMMEDIATE, None, None, False),
    # The part the gripper holds (utility/payload.py): changes the gravity model and the walls' dynamics -> OWNED.
    "set_payload":              (IMMEDIATE, None, None, True),
    # --- control ownership ---
    "take_control":             (IMMEDIATE, None, None, False),
    "set_force_confirm":        (IMMEDIATE, None, None, False),
    "release_control":          (IMMEDIATE, None, None, False),
    "rti_set_min_duration":     (IMMEDIATE, None, None, False),
    "set_recipe":               (IMMEDIATE, None, None, False),
    "rti_set_start_pose_check": (IMMEDIATE, None, None, False),
    "rti_set_lowpass":          (IMMEDIATE, None, None, False),
    # Gripper immediate controls (distinct from the queued open_gripper / close_gripper / move_gripper
    # below); RTI guards all three on active_CAN_gripper. gripper_set is the firmware/motor-mode
    # direct target (position:speed:current:action) the GUI's gripper card uses.
    "calibrate_gripper": (IMMEDIATE, None, None, False),
    "set_gripper_ctrl_mode": (IMMEDIATE, None, None, False),
    "set_gripper":       (IMMEDIATE, None, None, False),
    # --- motion (queued, require EXEC + enabled) ---
    "move_joints":    (QUEUED, ["EXEC"], "ENABLED", True),
    "move_pose":      (QUEUED, ["EXEC"], "ENABLED", True),
    "move_linear":    (QUEUED, ["EXEC"], "ENABLED", True),
    "move_linear_rel_world": (QUEUED, ["EXEC"], "ENABLED", True),
    "move_linear_rel_tool": (QUEUED, ["EXEC"], "ENABLED", True),
    "move_arc":       (QUEUED, ["EXEC"], "ENABLED", True),
    "move_arc_rel_world":   (QUEUED, ["EXEC"], "ENABLED", True),
    "move_arc_rel_tool":   (QUEUED, ["EXEC"], "ENABLED", True),
    "move_circle":    (QUEUED, ["EXEC"], "ENABLED", True),
    "move_circle_rel_world": (QUEUED, ["EXEC"], "ENABLED", True),
    "move_circle_rel_tool": (QUEUED, ["EXEC"], "ENABLED", True),
    "move_spline":    (QUEUED, ["EXEC"], "ENABLED", True),
    "move_spline_rel_world": (QUEUED, ["EXEC"], "ENABLED", True),
    "move_spline_rel_tool": (QUEUED, ["EXEC"], "ENABLED", True),
    "delay":         (QUEUED, ["EXEC"], None, False),
    # --- gripper (queued, same queue as joint moves so ordering is preserved;
    # fire-and-forget — checkpoint fires on send, not on physical completion) ---
    "open_gripper":  (QUEUED, ["EXEC"], "ENABLED", True),
    "close_gripper": (QUEUED, ["EXEC"], "ENABLED", True),
    "move_gripper":  (QUEUED, ["EXEC"], "ENABLED", True),
}


def kind_of(cmd):
    """Return IMMEDIATE / QUEUED, or None if the command is unknown."""
    rule = COMMAND_RULES.get(canonical_command(cmd))
    return rule[0] if rule else None


def validate(cmd, mode, state, homed):
    """Check a command against the live robot mode/state/homed.

    Returns (ok: bool, reason: str | None).
    """
    cmd = canonical_command(cmd)
    rule = COMMAND_RULES.get(cmd)
    if rule is None:
        return False, f"unknown command '{cmd}'"
    _, req_mode, req_state, req_homed = rule
    if req_mode is not None and mode not in req_mode:
        hint = " — enter_idle_mode() first" if "IDLE" in req_mode and mode != "IDLE" else ""
        return False, f"{cmd} requires mode in {req_mode}, robot is in {mode}{hint}"
    if req_state is not None and state != req_state:
        return False, f"{cmd} requires state {req_state}, robot is {state}"
    if req_homed and not homed:
        return False, f"{cmd} requires the robot to be homed"
    return True, None
