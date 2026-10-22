"""THE catalog of every error, warning and event code the robot reports (2026-10-15, the message standard).

A LEAF module (imports nothing of ours). docs/error_codes.md is generated from it (tools/error_codes_doc.py); the
rules every entry follows are docs/logging.md "Error messages" — the structure industrial robot controllers commonly
use (a number, a title, the cause, the consequence, the remedy) and the common writing guidelines for error messages
(what happened and why, what it did, what to do; plain words, no blame).

Every entry:
    number  a stable 4-digit number, grouped by subsystem (GROUPS); never reused, never renumbered
    title   what happened, in a few words: sentence case, no period (<= 50 characters)
    cause   why: one or two sentences, ending with a period             (warnings and errors)
    effect  what the robot did about it, one or two sentences           (warnings and errors)
    fix     what to do, as instructions                                  (warnings and errors)
An info event has a number and a title only. The code (the key) is the identifier clients and the logs use; the
per-occurrence detail (which joint, how far, the value) is the entry's message, never the catalog text.
dummy_test_scripts/test_error_catalog.py holds every entry to these rules and every emitted code to an entry.

ERROR_DESCRIPTIONS is the old name (utility/error_checks.py and nets use it): the same dict.
"""

GROUPS = {
    1: "Safety: E-stop, limits, walls, speed, zones, self-collision, collisions",
    2: "Drives, motors and the gripper",
    3: "CAN and communication",
    4: "Homing",
    5: "Motion and the planner",
    6: "Commands and control",
    7: "Configuration files",
    8: "System",
    9: "Events (information)",
}

# What the robot does — the same words for the same consequence.
_HARD = "The robot stops and is disabled until errors are cleared."
_HARD_BACK = _HARD + " Afterwards only motion back toward the allowed range is possible."
_WARN = "Warning only: the robot keeps running. The warning clears itself when the condition ends."
_REFUSED = "The command is refused. Nothing changes."
_MOVE = ("The move does not start. The rest of the program is cancelled, and motion commands are refused until "
         "errors are cleared.")
_STREAM = ("The robot braked to rest within its limits and refused the stream. It is disabled until errors are "
           "cleared.")
_AGAIN_RTI = "Clear errors, then claim control again."


KINDS = ("stop", "fails", "warning", "event")   # what it means for the robot (docs/error_codes.md's "Kind")


def _kind(effect):
    if effect.startswith("Warning only"):
        return "warning"
    if effect.startswith(("The robot stops", "The robot braked")):
        return "stop"
    return "fails"


def _e(number, title, cause, effect, fix, kind=None):
    """kind: stop (the robot stops, disabled), fails (the command / move / program fails or is refused; the robot
    is not affected), warning (the robot keeps running); read from the effect unless given."""
    return {"number": number, "title": title, "kind": kind or _kind(effect), "cause": cause, "effect": effect,
            "fix": fix}


def _info(number, title):
    return {"number": number, "title": title, "kind": "event", "cause": None, "effect": None, "fix": None}


def _drive(number, part, title, cause, fix):
    """A drive fault, joint or gripper motor: the same text with the part's name."""
    return _e(number, title, cause.format(part=part), _HARD, fix.format(part=part))


CATALOG = {
    # ---------------------------------------------------------------- 1xxx safety
    "ESTOP": _e(1001, "Emergency stop pressed",
                "An emergency stop button is pressed, or its circuit is open.", _HARD,
                "Make the area safe and release the E-stop. Clear errors, then enable the robot."),
    "SW_ESTOP": _e(1002, "Software emergency stop active",
                   "A software emergency stop was set by a command or the dashboard.", _HARD,
                   "Release the software E-stop. Clear errors, then enable the robot."),
    "OUT_OF_RANGE": _e(1101, "Joint outside its software limit",
                       "The joint's measured position went past its software limit while the robot moved it. A "
                       "collision, a push or a move the limits should have stopped can cause this.",
                       _HARD_BACK + " In IDLE freedrive this is only a warning.",
                       "Clear errors, then jog the joint back inside its range. Find what moved it out."),
    "SOFT_LIMIT_WARNING": _e(1102, "Joint past its software limit",
                             "The joint is past its software limit: it was moved there by hand in freedrive, or it "
                             "rests there.",
                             "Warning only. Jog, EXEC and RTI motion may only move the joint back inside.",
                             "Move the joint back inside its limit. The warning then clears itself."),
    "HARD_LIMIT": _e(1103, "Joint past its hard limit",
                     "A joint without an endstop or freedrive wall passed its hard limit, where its cables start to "
                     "wind up. Nothing physical stops it there.", _HARD_BACK,
                     "Check the joint's cables. Clear errors, then move the joint back inside."),
    "BEYOND_ENDSTOP": _e(1104, "Joint position beyond its endstop",
                         "The measured position is further past the joint's physical endstop than the robot XML "
                         "allows (<endstop_tolerance_deg>). A real arm cannot be there, so the position reference is "
                         "wrong: a slipped belt or gear, or an encoder fault.",
                         _HARD + " Homing is invalidated.",
                         "Check the joint's belt, gears and encoder. Clear errors, then home the robot again."),
    "JOINT_WALL": _e(1105, "Joint pushed through its freedrive wall",
                     "The joint was pushed through its freedrive wall by more than the wall's breakthrough "
                     "distance, and this wall is set to stop (robot XML <freedrive_wall freedrive=\"stop\">).",
                     _HARD_BACK, "Clear errors and enable the robot, then move the joint back inside the wall."),
    "JOINT_WALL_WARNING": _e(1106, "Joint pushed through its freedrive wall",
                             "The joint was pushed through its freedrive wall in freedrive by more than the wall's "
                             "breakthrough distance.", _WARN,
                             "Turn the joint back inside the wall. On J6 check the cables too: its wall marks the "
                             "cable wind-up limit."),
    "OVERSPEED": _e(1201, "Joint too fast",
                    "The joint's filtered speed stayed above its velocity limit times the safety margin.", _HARD,
                    "Find what commanded the joint so fast. Clear errors, then enable the robot."),
    "OVERSPEED_WARNING": _e(1202, "Joint too fast",
                            "The joint's filtered speed went above its velocity limit times the safety margin. The "
                            "check runs in warn-only mode.", _WARN,
                            "If this was normal motion, raise the joint's limit or margin before the check is set "
                            "to stop. RTI.log has the peak."),
    "TCP_OVERSPEED": _e(1203, "Tool too fast",
                        "The tool moved faster than <max_tcp_velocity> times the safety margin.", _HARD,
                        "Find what commanded the arm so fast. Clear errors, then enable the robot."),
    "TCP_OVERSPEED_WARNING": _e(1204, "Tool too fast",
                                "The tool moved faster than <max_tcp_velocity> times the safety margin. The check "
                                "runs in warn-only mode.", _WARN,
                                "If this was normal motion, raise <max_tcp_velocity> or its margin. RTI.log has the "
                                "peak."),
    "TRACKING": _e(1205, "Joint lagging its command",
                   "During EXEC motion the joint fell too far behind its commanded position: a bump, a collision or "
                   "a stall.", _HARD,
                   "Check for an obstruction or an overload. Clear errors, then continue."),
    "TRACKING_WARNING": _e(1206, "Joint lagging its command",
                           "During EXEC motion the joint fell further behind its commanded position than its limit. "
                           "The check runs in warn-only mode.", _WARN,
                           "If this was normal motion, raise <tracking_limit_deg> for the joint before the check is "
                           "set to stop."),
    "ZONE_VIOLATION": _e(1301, "Safety zone violated",
                         "The tool or a tool sphere left an allowed zone or entered a forbidden zone of the active "
                         "cell. The message names the zone.",
                         _HARD + " Afterwards only motion that does not go deeper into the zone is possible.",
                         "Clear errors, then move the robot out of the zone. If the zone is wrong, correct it in "
                         "config/cells."),
    "ZONE_WARNING": _e(1302, "Robot in a warning zone",
                       "The robot is inside a warning zone of the active cell.",
                       "Warning only: the robot keeps running. The warning clears when the robot leaves the zone.",
                       "None needed."),
    "SELF_COLLISION": _e(1401, "Self-collision",
                         "Two parts of the arm came closer than the self-collision margin on the measured pose. The "
                         "message names the pair.",
                         _HARD + " Afterwards only directions that open the gap are possible.",
                         "Clear errors, then jog the two parts apart. If a planned move caused it, report the pair "
                         "and the move."),
    "SELF_COLLISION_WARNING": _e(1402, "Self-collision margin reached",
                                 "Two parts of the arm are closer than the self-collision margin. The check runs in "
                                 "warn-only mode.", _WARN,
                                 "None needed. Once the check is trusted, set <self_collision><reaction> to stop."),
    "SELF_COLLISION_STALE": _e(1403, "Self-collision check not running",
                               "The self-collision worker has not reported for half a second, so self-collisions "
                               "are not being watched.",
                               "Warning only: the robot keeps running without its self-collision check.",
                               "Check the self-collision log in the Logs tab. The worker restarts with the service."),
    "COLLISION": _e(1501, "Collision detected",
                    "While the robot moved the joint, the external torque on it rose above its collision threshold "
                    "(robot XML <collision_torque_nm>) compared with the torque it had at rest before the move: the "
                    "arm hit something, or carries a load it was not told about.",
                    _HARD,
                    "Check what the arm hit and clear the way. Declare a held part with set_payload. Clear errors, "
                    "then continue. A program that pushes on purpose switches the check off with "
                    "set_collision_detection."),
    "COLLISION_WARNING": _e(1502, "Collision detected",
                            "While the robot moved the joint, the external torque on it rose above its collision "
                            "threshold compared with the torque it had at rest before the move. The check runs in "
                            "warn-only mode.", _WARN,
                            "If nothing was hit, raise <collision_torque_nm> for the joint or re-measure it with "
                            "api_tests/76 before the check is set to stop. RTI.log has the peak."),
    "CONTACT_WARNING": _e(1503, "Contact detected",
                          "While the robot moved the joint, the external torque on it rose above its contact "
                          "threshold (robot XML <contact_torque_nm>) compared with the torque it had at rest before "
                          "the move.", _WARN,
                          "None needed if the contact was expected. If nothing touched the arm, raise "
                          "<contact_torque_nm> for the joint or re-measure it with api_tests/76."),

    # ---------------------------------------------------------------- 2xxx drives, motors, gripper
    "TEMPERATURE": _drive(2101, "the joint's motor", "Motor overheating",
                          "The drive reports that {part} is too hot.",
                          "Let the motor cool down and check the joint for a high load. Clear errors, then enable the "
                          "robot."),
    "VBUS": _drive(2102, "the joint", "Drive supply voltage out of range",
                   "The drive of {part} reports its supply voltage outside its range.",
                   "Check the power supply and the joint's power wiring. Clear errors, then enable the robot."),
    "WATCHDOG": _drive(2103, "the joint", "Drive watchdog tripped",
                       "The drive of {part} received no command within its watchdog time: the CAN link to it, or "
                       "RTI's loop, stalled.",
                       "Check the joint's CAN cable and power. Clear errors, then enable the robot."),
    "ENCODER": _drive(2104, "the joint", "Encoder fault",
                      "The drive of {part} reports a lost or corrupted encoder signal.",
                      "Check the joint's encoder cable and connector. Clear errors, then home the robot again."),
    "DRIVER": _drive(2105, "the joint", "Drive fault",
                     "The drive of {part} reports an internal hardware fault.",
                     "Switch the robot off and on. If the fault returns, replace the drive."),
    "VELOCITY": _drive(2106, "the joint's motor", "Motor over its velocity limit",
                       "The drive reports that {part} exceeded its velocity limit.",
                       "Check for a collision or an obstruction. Clear errors, then enable the robot."),
    "CURRENT": _drive(2107, "the joint", "Motor overcurrent",
                      "The drive of {part} detected an overcurrent.",
                      "Check the joint for a mechanical jam. Clear errors, then enable the robot."),
    "ESTOP_MOTOR": _drive(2108, "the joint", "Drive emergency stop input",
                          "The drive of {part} reports an emergency stop on its own input.",
                          "Check the drive's E-stop input wiring. Clear errors, then enable the robot."),
    "HOLD_OSCILLATION_WARNING": _e(2109, "Joint buzzing while holding",
                                   "The motor current keeps swinging fast while the joint is commanded to stand "
                                   "still: the drive's control loop is fighting itself, which you hear as a buzz.",
                                   _WARN,
                                   "Lower the joint's speed gains (KPV, KPP). If that does not help, check the "
                                   "encoder magnet and the mechanics."),
    "GRIPPER_MOTOR_TEMPERATURE": _drive(2201, "the gripper motor", "Gripper motor overheating",
                                        "The drive reports that {part} is too hot.",
                                        "Let the gripper motor cool down. Clear errors, then enable the robot."),
    "GRIPPER_MOTOR_VBUS": _drive(2202, "the gripper", "Gripper supply voltage out of range",
                                 "The drive of {part} reports its supply voltage outside its range.",
                                 "Check the power supply and the gripper's power wiring. Clear errors, then enable "
                                 "the robot."),
    "GRIPPER_MOTOR_WATCHDOG": _drive(2203, "the gripper", "Gripper watchdog tripped",
                                     "The drive of {part} received no command within its watchdog time.",
                                     "Check the gripper's CAN cable and power. Clear errors, then enable the robot."),
    "GRIPPER_MOTOR_ENCODER": _drive(2204, "the gripper", "Gripper encoder fault",
                                    "The drive of {part} reports a lost or corrupted encoder signal.",
                                    "Check the gripper's encoder cable and connector. Clear errors, then enable the "
                                    "robot."),
    "GRIPPER_MOTOR_DRIVER": _drive(2205, "the gripper", "Gripper drive fault",
                                   "The drive of {part} reports an internal hardware fault.",
                                   "Switch the robot off and on. If the fault returns, replace the gripper drive."),
    "GRIPPER_MOTOR_VELOCITY": _drive(2206, "the gripper motor", "Gripper motor over its velocity limit",
                                     "The drive reports that {part} exceeded its velocity limit.",
                                     "Check the gripper for an obstruction. Clear errors, then enable the robot."),
    "GRIPPER_MOTOR_CURRENT": _drive(2207, "the gripper", "Gripper motor overcurrent",
                                    "The drive of {part} detected an overcurrent.",
                                    "Check the gripper for a mechanical jam. Clear errors, then enable the robot."),
    "GRIPPER_MOTOR_ESTOP": _drive(2208, "the gripper", "Gripper drive emergency stop input",
                                  "The drive of {part} reports an emergency stop on its own input.",
                                  "Check the gripper drive's E-stop input wiring. Clear errors, then enable the "
                                  "robot."),
    "GRIPPER_ERROR": _e(2301, "Gripper fault", "The gripper reports a general fault.", _HARD,
                        "Clear errors and enable the robot. If the fault returns, switch the gripper off and on."),
    "GRIPPER_ESTOP": _e(2302, "Gripper emergency stop", "The gripper reports an emergency stop signal.", _HARD,
                        "Clear the gripper's E-stop condition. Clear errors, then enable the robot."),
    "GRIPPER_TIMEOUT": _e(2303, "Gripper timeout", "The gripper reports a timeout: it got no command in time.", _HARD,
                          "Check the gripper's CAN cable and power. Clear errors, then enable the robot."),
    "GRIPPER_SETTLE_TIMEOUT": _e(2304, "Gripper did not settle",
                                 "A gripper command did not report a settled position within 5 s.",
                                 "The gripper command fails. The rest of the program continues.",
                                 "Check that the jaws move freely and the gripper is powered."),

    # ---------------------------------------------------------------- 3xxx CAN and communication
    "CAN_LOST": _e(3001, "Drive connection lost",
                   "No CAN data arrived from the joint's drive for too long.", _HARD + " Homing is invalidated.",
                   "Check the CAN cable, its termination and the joint's power. Clear errors, then home the robot "
                   "again."),
    "CAN_STALE": _e(3002, "Drive data late", "The joint's CAN data is getting old: its drive answers late.", _WARN,
                    "None needed if it clears. If it repeats, check the CAN cable and the bus load."),
    "CAN_REPLY_MISSED_WARNING": _e(3003, "Drive replies keep arriving late",
                                   "The joint's drive replies missed RTI's CAN read 5 or more times a second for 3 s, "
                                   "so its measured state was one tick old on those ticks.", _WARN,
                                   "Check that RTI's start-up line about the CAN interrupt threads reports no "
                                   "problem. RTI.log has the peak."),
    "GRIPPER_DISCONNECTED": _e(3004, "Gripper connection lost", "The gripper sends no CAN replies.",
                               _HARD + " Homing is invalidated.",
                               "Check the gripper's CAN cable and power. Clear errors, then home the robot again."),
    "EXEC_LINK_LOST": _e(3101, "Executor stopped responding",
                         "The command executor went silent while the robot ran EXEC motion.", _HARD,
                         "Restart the service. Clear errors, then enable the robot."),
    "RTI_LINK_LOST": _e(3102, "External PC stopped sending",
                        "The PC that held RTI control stopped sending setpoints.", _STREAM,
                        "Check the network and the PC's program. " + _AGAIN_RTI),
    "SEND_TIMEOUT": _e(3201, "Trajectory not taken by RTI",
                       "The executor could not hand a trajectory to RTI in time: RTI is not taking trajectory "
                       "batches.", _MOVE,
                       "Check in the Logs tab that RTI is running. Restart the service if it is not."),

    # ---------------------------------------------------------------- 4xxx homing
    "NOT_HOMED": _e(4001, "Robot not homed",
                    "The robot has not been homed since RTI started, so the joint positions have no reference.",
                    "Warning only. Motion that needs a reference (EXEC, RTI) is refused until the robot is homed.",
                    "Home the robot."),
    "HOMING_FAILED": _e(4002, "Joint homing failed",
                        "Homing of this joint failed. The message says why: an endstop or Hall sensor not reached in "
                        "time, two passes too far apart, no position at the endstop, or the joint homed away from "
                        "where this arm always homes.", _HARD,
                        "Move the arm to its park pose by hand and check that nothing is in the way. Clear errors, "
                        "enable and home again. After a deliberate mechanical change, accept the new reference in "
                        "System Info, Arms (homing)."),
    "GRIPPER_HOMING_FAILED": _e(4003, "Gripper homing failed",
                                "The gripper's calibration did not finish in time, or its motor homing found no "
                                "endstop. The message says which.", _HARD,
                                "Check that the jaws move freely and the gripper is powered. Clear errors, enable and "
                                "home again."),
    "HOMING_ABORTED": _e(4004, "Homing aborted", "An error latched while the robot was homing.",
                         "Homing stopped and the robot is not homed.",
                         "Clear the error the error list names, then home the robot again.", kind="stop"),
    "HOMING_INVALIDATED": _e(4005, "Homing invalidated",
                             "The position reference can no longer be trusted: a drive lost its connection, a joint "
                             "measured past its endstop, or firmware was flashed. The message says which.",
                             "The robot is not homed. Motion that needs a reference is refused.",
                             "Home the robot again.", kind="warning"),
    "HALL_CHECK_REFUSED": _e(4006, "Hall sensor check refused",
                             "The Hall sensor check cannot start now. The message says why.", _REFUSED,
                             "Do what the message asks, usually going to IDLE, then start the check again."),
    "FREE_JOINT_REFUSED": _e(4007, "Freeing a joint refused",
                             "A joint cannot be freed for hand turning now. The message says why.", _REFUSED,
                             "Do what the message asks, usually going to IDLE, then try again."),
    "JOINT_TEST_REFUSED": _e(4008, "Joint test move refused",
                             "A test move cannot start now. The message says why.", _REFUSED,
                             "Do what the message asks, usually enabling the arm in IDLE, then try again."),

    # ---------------------------------------------------------------- 5xxx motion and the planner
    "PLAN_FAIL": _e(5001, "Trajectory planning failed",
                    "The planner could not plan the move. The message gives the reason.", _MOVE,
                    "Check the move's target and options. The executor's log in the Logs tab has the detail."),
    "MOTION_ABORTED": _e(5002, "Program aborted",
                         "A queued command failed or motion was interrupted, so the rest of the program was "
                         "cancelled. The message names the command and the reason.",
                         "New motion commands are refused until errors are cleared.",
                         "Fix what the failing command's error says. Clear errors, then send the program again."),
    "ABORTED": _e(5003, "Command cancelled",
                  "The command was cancelled before it ran, because an earlier command failed or motion was "
                  "interrupted.", "The command did not run.", "Clear errors, then send the program again."),
    "STOPPED": _e(5004, "Command stopped", "A stop ended the program before this command ran.",
                  "The command did not run.", "Send the command again if it is still needed."),
    "SETTLE_TIMEOUT": _e(5005, "Move did not settle",
                         "The final position did not settle within tolerance in time. The strict completion policy "
                         "reports this as a failure.", "The command fails. The rest of the program continues.",
                         "Check for a load or an obstruction near the target, or use the settled completion "
                         "policy."),
    "JOINT_LIMIT": _e(5006, "Move outside the joint limits",
                      "The planned move would take a joint outside its software limits. The message names the joint "
                      "and its limits.", _MOVE,
                      "Change the target, or move the arm back inside its limits first."),
    "ZONE_LIMIT": _e(5007, "Move into a safety zone",
                     "The planned move would take the tool or a robot sphere into a stop zone of the active cell.",
                     _MOVE,
                     "Change the target or the path so that it stays out of the zone. If the zone is wrong, correct "
                     "it in config/cells."),
    "ZONE_CHECK_UNAVAILABLE": _e(5008, "Zone check unavailable",
                                 "Safety zones are active, but the planner has no robot model to check them.", _MOVE,
                                 "Find in the executor's log (Logs tab) why the robot model did not load, then "
                                 "restart the service."),
    "ACCEL_LIMIT_VIOLATION": _e(5009, "Planned acceleration over its limit",
                                "The planned trajectory's acceleration exceeds the limit it was planned against. "
                                "This is an internal planner check.", _MOVE,
                                "Lower the move's speed or acceleration, or use another profile. Report it if it "
                                "repeats."),
    "DURATION_INFEASIBLE": _e(5010, "Requested duration too short",
                              "The move cannot be done in the requested duration within the limits.", _MOVE,
                              "Ask for a longer duration (the message gives the shortest possible), or leave the "
                              "duration out."),
    "INVALID_DURATION": _e(5011, "Invalid duration", "The requested duration is not a positive number.", _MOVE,
                           "Give a duration above 0 s, or leave it out."),
    "STOP_FALLBACK": _e(5012, "Stop not confirmed",
                        "RTI did not confirm a controlled stop within 1.5 s. RTI may run older code than the "
                        "executor.", "The robot was switched to IDLE instead.",
                        "Restart the service so that RTI and the executor run the same code. If it repeats, check "
                        "RTI.log in the Logs tab."),
    "IK_TARGET_UNREACHABLE": _e(5101, "Target out of reach",
                                "No joint solution reaches the requested pose: it is outside the arm's workspace or "
                                "joint limits.", _MOVE,
                                "Choose a pose within reach, or approach it from another configuration."),
    "IK_PARTIAL_PATH": _e(5102, "Path only partly reachable",
                          "The inverse kinematics could solve only part of the Cartesian path.", _MOVE,
                          "Shorten the path, or move it closer to the arm's reachable region."),
    "IK_MARGIN_VIOLATION": _e(5103, "Path too close to a limit",
                              "The solved path passes too close to a joint limit or a singularity.", _MOVE,
                              "Adjust the target pose, or plan the path through another region."),
    "IK_POSE_ERROR": _e(5104, "Solution misses the pose",
                        "The solved joint configuration does not reach the requested pose within tolerance, often "
                        "near a singularity.", _MOVE,
                        "Move the target away from singular poses, or use a joint move."),
    "IK_BRANCH_JUMP": _e(5105, "Configuration flip on the path",
                         "Along the Cartesian path the solution jumped to another arm configuration, so a joint "
                         "would flip.", _MOVE,
                         "Shorten the path, start from another pose, or use a joint move."),
    "IK_SINGULARITY_WARNING": _e(5106, "Path near a singularity",
                                 "The planned path passes near a kinematic singularity.", "Warning only: the move runs.",
                                 "If the motion is rough, plan the path away from the singularity."),
    "CARTESIAN_PATH_DEVIATION": _e(5107, "Tool strays from the path",
                                   "The planned tool path strays from the programmed line or curve by more than the "
                                   "tolerance.", _MOVE,
                                   "Move the path away from singular poses, shorten it, or use a joint move."),
    "CURVE_BLEND_UNSUPPORTED": _e(5108, "Blending not supported on curves", "Curve commands cannot be blended yet.",
                                  _MOVE, "Send the curve with blend set to 0."),
    "CURVE_GEOMETRY_INVALID": _e(5109, "Invalid curve",
                                 "The curve's points do not define a valid curve, for example collinear points for an "
                                 "arc.", _MOVE, "Check the curve's points against the command reference."),
    "BAD_ARGUMENTS": _e(5110, "Invalid curve arguments",
                        "A curve command has the wrong number of values or an unknown frame.", _MOVE,
                        "Check the command's arguments against the command reference."),
    "TOPPRA_UNAVAILABLE": _e(5201, "TOPPRA profile unavailable",
                             "The TOPPRA profile was requested, but its package cannot be imported.", _MOVE,
                             "Install the toppra package, or choose another profile."),
    "TOPPRA_INFEASIBLE": _e(5202, "TOPPRA found no solution",
                            "TOPPRA could not time the path within the limits.", _MOVE,
                            "Check the speed and acceleration limits, or choose another profile."),
    "LIMIT_REACHED": _e(5301, "Jog stopped at a limit", "The jog reached a joint limit or a workspace limit.",
                        "The jog stops there. Jogging away from the limit is possible.", "Jog the other way.",
                        kind="warning"),
    "SELF_COLLISION_BLOCK": _e(5302, "Jog stopped at the collision margin",
                               "Jogging further would bring two parts of the arm too close.", "The jog stops there.",
                               "Jog the other way, or move another joint.", kind="warning"),

    # ---------------------------------------------------------------- 6xxx commands and control
    "UNKNOWN_CMD": _e(6001, "Unknown command", "The command name is not known.", _REFUSED,
                      "Check the spelling against the command reference."),
    "BAD_JSON": _e(6002, "Invalid message", "The message is not valid JSON.", _REFUSED,
                   "Check how the client encodes its messages."),
    "BAD_ARGS": _e(6003, "Invalid arguments", "The command's arguments failed validation. The message says which.",
                   _REFUSED, "Check the number and types of the arguments against the command reference."),
    "BAD_OPTIONS": _e(6004, "Invalid motion options", "The command's motion options failed validation.", _REFUSED,
                      "Check the profile, speed, blend and completion policy values."),
    "BAD_PROFILE": _e(6005, "Unknown motion profile", "The requested trajectory profile is not known.", _REFUSED,
                      "Use a profile from the command reference."),
    "BAD_IK_SOLVER": _e(6006, "Unknown IK solver", "The requested IK solver is not known.", _REFUSED,
                        "Use a solver from the command reference."),
    "BAD_POLICY": _e(6007, "Unknown completion policy", "The requested completion policy is not known.", _REFUSED,
                     "Use commanded, settled or strict."),
    "BAD_PIN": _e(6008, "Invalid I/O pin", "The command names an I/O pin the robot does not have.", _REFUSED,
                  "Use a pin number in the range the message gives."),
    "FORBIDDEN": _e(6009, "Connection not allowed", "The client's IP address is not on the executor's allowed list.",
                    "The connection is closed.",
                    "Connect from an allowed machine, or add its address to RCB_ALLOWED_IPS."),
    "REJECTED": _e(6010, "Command not allowed now",
                   "The command is not allowed in the robot's current mode or state. The message says what it needs.",
                   _REFUSED, "Bring the robot into the mode or state the message names, then send the command again."),
    "QUEUE_FULL": _e(6011, "Motion queue full", "The motion queue holds its maximum number of commands.", _REFUSED,
                     "Wait until queued moves finish, then send the command again."),
    "UNHANDLED": _e(6012, "Command not handled",
                    "The executor accepted the command but has no handler for it. This is an internal error.",
                    _REFUSED, "Report it with the command and the executor's log."),
    "INTERNAL": _e(6013, "Internal error", "The command failed with an unexpected error.",
                   "The command did not complete.", "Report it with the executor's log from the Logs tab."),
    "MODE_REJECTED": _e(6014, "Mode change refused",
                        "RTI refused the mode change. The message says why: not homed, not enabled, an active error, "
                        "or a change that is not allowed from this mode.", "The robot stays in its mode.",
                        "Do what the message asks first, then request the mode again."),
    "ENABLE_REJECTED": _e(6015, "Enable refused",
                          "RTI refused to enable the robot. The message says why, for example an active error or a "
                          "safety stop.", "The robot stays disabled.",
                          "Clear the errors or the safety condition first, then enable the robot again."),
    "PAYLOAD_LIMIT": _e(6016, "Payload not accepted",
                        "The payload is above the robot's maximum, or its values are not valid.", _REFUSED,
                        "Check the mass and the centre of mass against <max_payload_kg> in the robot XML."),
    "WALL_OFF": _e(6017, "No freedrive wall on this joint",
                   "This joint has no freedrive wall to change: joint walls are off, or there is no inertia for it.",
                   _REFUSED, "Turn joint walls on in <haptic_walls>, or choose another joint."),
    "WALL_UNSTABLE": _e(6018, "Wall setting unstable",
                        "The requested wall frequency and damping would be unstable at this filter setting.", _REFUSED,
                        "Use a frequency and damping ratio inside the range the message gives."),
    "NO_ROBOT_MODEL": _e(6019, "No robot model", "The executor has no robot model, so it cannot check reachability.",
                         _REFUSED,
                         "Find in the executor's log (Logs tab) why the model did not load, then restart the service."),
    "SUPERSEDED": _e(6020, "Gripper command replaced", "A newer gripper command replaced this one before it settled.",
                     "This command ends and the newer one runs.", "None needed."),
    "CAPTURE_REFUSED": _e(6021, "Capture not started",
                          "A telemetry capture cannot start or stop: one is already running, the length is outside 1 "
                          "to 60 s, the recorder process is not running, nothing was captured, or a stop was asked "
                          "while none runs. The message says which.",
                          _REFUSED,
                          "Wait for the running capture to finish, ask for a length of 1 to 60 s, or check that the "
                          "recorder runs."),
    "CLIENT_TOO_OLD": _e(6022, "Client too old for pose commands",
                         "The client declares no command protocol version, so it may be from before 0.9.0, when pose "
                         "angles had another meaning. A pose it sends could turn the tool to another orientation.",
                         _REFUSED,
                         "Use the robotclient that came with the robot's runtime. A client that sends raw JSON adds "
                         "\"protocol\": 1 to each message."),
    "CONTROL_HELD": _e(6101, "Control held by another client", "Another client holds control of the robot.",
                       _REFUSED,
                       "Release control on that client, or take control. A forced takeover asks for confirmation "
                       "at the control box."),
    "CONFIRM_REQUIRED": _e(6102, "Takeover not confirmed",
                           "A forced takeover needs a confirmation at the control box, and it was denied or timed out.",
                           "Control stays with the other client.",
                           "Try again and confirm at the control box, or ask the other client to release control."),
    "CONFIRM_BUSY": _e(6103, "Takeover already waiting",
                       "Another forced takeover is already waiting for confirmation at the control box.", _REFUSED,
                       "Answer or wait out that request, then try again."),
    "RTI_POSITION_LIMIT": _e(6201, "PC target outside the joint limits",
                             "The PC's RTI target would move the joint past its software limit, or toward it faster "
                             "than the joint can still stop.", _STREAM,
                             "Keep the PC's targets inside the joint limits and slow down before them. " + _AGAIN_RTI),
    "RTI_VELOCITY_LIMIT": _e(6202, "PC stream too fast",
                             "The PC's hard-RTI stream moved the joint faster than its RTI velocity limit plus 10 %.",
                             _STREAM, "Slow the PC's trajectory down to the joint's velocity limit. " + _AGAIN_RTI),
    "RTI_ACCELERATION_LIMIT": _e(6203, "PC stream accelerates too hard",
                                 "The PC's hard-RTI stream changed the joint's speed faster than its acceleration "
                                 "limit plus 10 %: a jump, a sudden stop or a wild packet.", _STREAM,
                                 "Send a smooth trajectory, or filter noisy targets (rti_set_lowpass). " + _AGAIN_RTI),
    "RTI_JERK_LIMIT": _e(6204, "PC stream too jerky",
                         "The PC's hard-RTI stream changed the joint's acceleration faster than its jerk limit plus "
                         "10 %.", _STREAM,
                         "Send a jerk-limited trajectory, or filter noisy targets (rti_set_lowpass). " + _AGAIN_RTI),
    "RTI_NO_SEND_TIME": _e(6205, "PC packet without its send time",
                           "A hard-RTI position command arrived without the PC's send time. Hard RTI follows the PC's "
                           "own timing and needs it.", _STREAM,
                           "Keep send times on in the client (the default), or use SOFT_RTI for a sender without "
                           "them. " + _AGAIN_RTI),

    # ---------------------------------------------------------------- 7xxx configuration files
    "ZONE_CONFIG": _e(7001, "Cell file unreadable", "The safety-zone cell file could not be read.",
                      "The zones in force stay unchanged.",
                      "Fix the cell file the message names, or empty <active_cell>, then reload the zones."),
    "FILTER_CONFIG": _e(7002, "Filter file invalid", "The filter file config/filters.xml could not be parsed.",
                        "The filters in force stay unchanged.",
                        "Fix config/filters.xml where the message says, then reload the filters."),
    "RECIPE_DIR": _e(7003, "Recipe folder unreadable", "The telemetry recipe folder could not be read.",
                     "The recipes in force stay unchanged.",
                     "Check that data_recipe exists and is readable, then reload the recipes."),
    "SELF_COLLISION_MODEL": _e(7004, "Self-collision model not built",
                               "Self-collision checking is on, but its model could not be built from the geometry "
                               "files.", "Reachability checks are refused.",
                               "Fix the geometry files the message names (robot or gripper XML), then restart the "
                               "service."),
    "SAVE_FAILED": _e(7005, "Walls not saved", "The freedrive walls could not be written to the robot XML.",
                      "The file is unchanged. The walls in force stay as they are.",
                      "Check that the robot XML is writable and valid, then save again."),

    # ---------------------------------------------------------------- 8xxx system
    "LOOP_DEGRADED": _e(8001, "Control loop running slow", "RTI's control loop runs slower than its configured rate.",
                        _WARN,
                        "Check the CPU load and temperature, and whether a drive went silent. Heavy jobs on the Pi "
                        "slow the loop."),
    "LOOP_CRITICAL": _e(8002, "Control loop critically slow",
                        "RTI's control loop was critically slow: its period stayed over 1.5 times nominal for a "
                        "second, or one tick took over 3 times nominal.", _HARD,
                        "Check the CPU load and temperature, and the timing in the RT Diagnostics tab. Clear errors, "
                        "then enable the robot."),

    # ---------------------------------------------------------------- 9xxx events (information)
    "MODE_CHANGE": _info(9001, "Mode changed"),
    "STATE_CHANGE": _info(9002, "Robot enabled or disabled"),
    "CMD_QUEUED": _info(9003, "Command queued"),
    "CMD_STARTED": _info(9004, "Command started"),
    "CMD_COMPLETED": _info(9005, "Command completed"),
    "CMD_EXECUTED": _info(9006, "Command executed"),
    "JOG_STARTED": _info(9007, "Jog started"),
    "JOG_STOPPED": _info(9008, "Jog stopped"),
    "LIMIT_CLEARED": _info(9009, "Jog limit cleared"),
    "COMPENSATION_CHANGED": _info(9010, "Compensation changed"),
    "PAYLOAD_CHANGED": _info(9011, "Payload changed"),
    "HOMING_STARTED": _info(9012, "Homing started"),
    "HOMING_JOINT_HOMED": _info(9013, "Joint homed"),
    "HOMING_COMPLETED": _info(9014, "Homing completed"),
    "HALL_CHECK": _info(9015, "Hall sensor check"),
    "BLEND_SPLICED": _info(9016, "Move blended into the running move"),
    "BLEND_SPLICE_REJECTED": _info(9017, "Blend into the running move skipped"),
    "CAPTURE_STARTED": _info(9018, "Telemetry capture started"),
    "CAPTURE_SAVED": _info(9019, "Telemetry capture saved"),
    "COLLISION_DETECTION_CHANGED": _info(9020, "Collision detection changed"),
    "FREE_JOINT": _info(9021, "Joint freed for hand turning"),
    "JOINT_TEST": _info(9022, "Joint test move"),
}

ERROR_DESCRIPTIONS = CATALOG          # the old name (utility/error_checks.py, nets)
_BY_NUMBER = {v["number"]: k for k, v in CATALOG.items()}


def entry(code):
    """The catalog entry of a code ("J3:WATCHDOG" and "Z_cell:ZONE_VIOLATION" work too), or None."""
    return CATALOG.get(code.split(":", 1)[-1]) if code else None


def short_key_of(code):
    """'J3:WATCHDOG' -> 'WATCHDOG', 'Z_cell:ZONE_WARNING' -> 'ZONE_WARNING'; a bare code unchanged."""
    return code.split(":", 1)[-1] if code else code


def code_of(number):
    """The code of a number, or None."""
    return _BY_NUMBER.get(number)


def label(code):
    """'2103 WATCHDOG' — how a code is shown next to its title; the bare code when it is not catalogued."""
    e = entry(code)
    short = code.split(":", 1)[-1] if code else code
    return f"{e['number']} {short}" if e else str(short)
