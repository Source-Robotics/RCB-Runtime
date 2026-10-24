# Changelog

What changed in each version of the RCB runtime. The newest version is at the top.

- The runtime's version is in the `VERSION` file at the repository root, written as `MAJOR.MINOR.PATCH`.
  - **PATCH** goes up for a fix that no program and no configuration file can see.
  - **MINOR** goes up for new commands, fields or configuration keys that keep older programs working.
  - **MAJOR** goes up when a protocol or a configuration file format changes incompatibly.
- Each release is a git tag `vMAJOR.MINOR.PATCH`. Its GitHub release notes are this file's section for that version,
  printed by `tools/release_notes.py`.
- Which client, protocol, firmware and hardware versions work with each release is in `COMPATIBILITY.md`.
- Changes not released yet collect under **Unreleased**.

## [Unreleased]

### Added

- Full docstrings for the robot control commands (`enable`, `disable`, `home`, `clear_errors`, the mode changes,
  `stop`, `pause`, `resume`, `safety_stop` and its release, the flashing commands, and the control commands): what
  each does in every mode, every code it can be refused with, what it raises, and an example.
- The same for every other public `RobotClient` method (waiting, connection and events, motion settings, jogging,
  I/O, the gripper, status and data, safety and dynamics, the real-time interface, diagnostics and service), and for
  `RTIClient` with its classes and `GripperResult`.

### Changed

- A client that disconnects while its jog runs stops that jog: the robot brakes the joint to rest. The joint used to
  run on until its joint limit.
- `stop()`, `disable()` and an error stop the gripper's jaws where they are. They used to go on to their last target
  and press there.
- `joint_test()` on a homed arm refuses a move past the joint's soft limits, and the other joints hold still during a
  test. They used to float in freedrive.
- `wait_error_reset()` ignores the `NOT_HOMED` warning, which describes a state, not a fault. It never returned on an
  arm that was not homed.
- `set_pid_gains()` refuses an `Ilim` or `vel_lim` above the joint's limit in the robot XML, a negative gain and the
  gripper's drive (`BAD_ARGS`), and refuses to run while the robot homes or the Hall sensor check, a freed joint or a
  joint test runs (`REJECTED`): each relies on the joint's current limit, which the command resets.
- `set_digital_output()` takes only true or false, 1 or 0. The Python client raises `ValueError` for anything else and
  a raw client gets `BAD_ARGS`; the string "false" or "0" used to switch the output on.
- `rti_set_rate_limiter()`, `rti_set_gravity_comp()` and `rti_set_start_pose_check()` refuse anything but true or
  false with `BAD_ARGS`. A raw client's string "true" used to switch these protections off.
- `move_gripper()` refuses a position or speed outside 0 to 255 and a current outside 1 mA to the gripper XML's current
  limit (`BAD_ARGUMENTS`). The values used to wrap at 256, and a current of -1 reached the drive as 65535 mA.
- The control box's isolated outputs start off each time the robot software starts. After a restart, an output that
  had been on used to switch back on by itself.
- `home()`, `enter_flashing_mode()`, `release_safety_stop()` and `exit_flashing_mode()` are refused with `REJECTED`
  when the robot is in a mode they cannot act in: `home()` needs IDLE mode and an enabled robot,
  `enter_flashing_mode()` IDLE mode or the error state, `release_safety_stop()` the motors off, and
  `exit_flashing_mode()` flashing mode. Before, the robot accepted the call and then refused the mode change in its
  activity log only, so a wait timed out. In EXEC mode, `home()` and `enter_flashing_mode()` also cancelled every
  client's program while the arm went on playing the moves the robot already held, and the two releases acted as
  `enter_idle_mode()` in every other mode.
- `take_control()` and `set_force_confirm()` accept only true or false, 1 or 0, or "1" or "0" from a raw client, and
  refuse anything else with `BAD_ARGS`. The JSON string "false" used to read as true, which for `take_control()`
  meant a forced takeover. The Python client always sent a real boolean.
- With the takeover confirmation on, the client that holds control no longer needs the button press for its own
  `take_control(force=True)`.
- A command refused for the robot's mode names the way out of that mode: `clear_errors()` in the error state,
  `release_safety_stop()` with the motors off, `exit_flashing_mode()` while flashing, and waiting while homing.

### Fixed

- A jog speed that was not a number ran the joint at full speed, and a jog of a joint that does not exist was
  answered "ok" and raised an exception inside the control loop. Both are refused with `BAD_ARGS`.
- `move_gripper(position, current=...)` without a speed sent the current as the speed.
- A dry run of a gripper command moved the jaws. It now checks the command and sends nothing, as for moves.
- In soft real-time mode the robot refused the client's stop, so `RTIClient.stop()` and `MotionFinished` ended the
  session with `RTI_LINK_LOST`. They now end it with a clean stop.
- A callback (`on_status_change()`, `on_motion_complete()`, `on_motion_error()`, `on_error()`) that raised stopped
  the client's receiver: the client counted as disconnected and refused every command, `stop()` included. The
  exception is now printed and the client goes on.
- `reload_zones()` with a malformed `config/system.xml` or gripper XML, and `is_reachable()` with geometry files that
  no longer load, dropped the client's connection. They answer `ZONE_CONFIG` and `SELF_COLLISION_MODEL`.
- `save_joint_walls()` writes the robot XML through a temporary file, so a write that fails, for example on a full SD
  card, leaves the file as it was instead of cut short.
- `safety_stop()` with an error latched switched the motors off for one control tick (4 ms) only: the robot went
  straight back to holding the arm. The motors now stay off until `release_safety_stop()`, and an error that latches
  while they are off no longer switches them on again.
- After almost every restart, one control tick of about 70 ms, a few seconds after homing, latched `LOOP_CRITICAL`
  and disabled the robot. It was Python's garbage collector walking every object the real-time process made while
  starting. Those objects are now frozen before the control loop starts: on the arm, a full collection took 67 ms
  before and 0 ms after. Any collection that still pauses the loop for more than 5 ms is logged in `RTI.log`.
- The dashboard's firmware-flashing override dialog said the motors lose torque within about 5 s. It now shows the
  drive watchdog time from `config/system.xml`, 0.1 s, and says that the joints brake and the gripper coasts.

### Removed

- `RobotClient.rti_set_profile()`. It sent a command the robot no longer has, so every call failed with
  `UNKNOWN_CMD`. Choose the streaming contract with `rti_connect(pc_ip, rate_hz=...)`: with a rate the robot uses the
  soft real-time mode, without one the hard real-time mode.

## [0.10.0] - 2026-10-23

### Added

- `VERSIONING.md`: which version to change for which kind of change, and how a release is made, including the
  documentation.
- `docstrings_guide.md`: how every docstring is written. The documentation's Python API reference is built from the
  `robotclient` docstrings.
- Full docstrings for the motion commands (`move_joints`, `move_pose`, `move_linear`, the arc, circle and spline
  moves and their relative forms, `delay`): every argument with its unit, every per-move setting, what the call
  returns and raises, every error code it can fail with once the robot accepted it, and an example.

### Changed

- Every command has one documented name. Its old names keep working, in Python (the same function) and on the wire
  (the robot maps an old command ID to its command):
  - The relative moves are `move_linear_rel_world` / `move_linear_rel_tool`, and the same for arcs, circles and
    splines. `_rel_wrf` / `_rel_trf` are their other names.
  - The gripper commands start with the verb, as every other command does: `open_gripper`, `close_gripper`,
    `calibrate_gripper`. `gripper_open`, `gripper_close` and `gripper_calibrate` still work.
  - `set_profile` and `set_ik_solver` are the names of their pairs; `set_default_profile` and
    `set_default_ik_solver` still work. `latest_status()` is the new name of `status()`, so that it is not mistaken
    for `get_status()`, which asks the robot.
  - A command's wire ID is its method's name. Fourteen settings were sent under another ID, for example
    `gravity_comp` for `set_gravity_comp` and `jog_set` for `jog`. The robot accepts both IDs. A client from this
    release reads the robot's version when it connects and sends an older robot the IDs it knows, so a program runs
    on either; only a command that did not exist on the older robot is refused there, as before.
- `delay()` refuses a time of 0 or less, or one that is not a finite number, with `INVALID_DURATION`, the moves'
  rule for `duration`. It used to finish after about 8 ms without an error, so `delay(-5)` written for `delay(5)` went
  unnoticed. A G-code `G4 P0` dwell still pauses for nothing (the G-code runner skips it), and a negative `P` is
  refused when the program is read.

### Fixed

- `move_joints(..., dry_run=True)` and `delay(..., dry_run=True)` planned the move and sent it to the robot, so a
  "dry" joint move moved the arm. They now plan and check it and send nothing, as every other move already did.
  Two more ways a dry run reached the robot are closed: a dry-run move with a `zone`, `tolerance` or
  `ruckig_corner` blend was planned and sent as part of the blended chain around it (any move type), and a move
  blended toward a dry-run successor was sent carrying a corner speed into a move that never came. A dry run now
  never joins a chain, and the move before it ends at rest.
- A dry run of `move_pose`, `move_linear`, the relative moves and the arc, circle and spline moves stopped after
  the inverse kinematics, before the timing and the joint-limit, safety-zone and self-collision checks, so it could
  pass a move the robot then refused. Every move's dry run now plans in full and runs every check, then sends
  nothing.
- With `profile="toppra"`, a timing failure on a straight-line or curved move was reported as `PLAN_FAIL`. It is now
  `TOPPRA_INFEASIBLE` or `TOPPRA_UNAVAILABLE`, as on joint moves.
- `move_pose` now has the acceleration check every other move has: a planned motion over the acceleration limit is
  refused with `ACCEL_LIMIT_VIOLATION` instead of being sent.
- The motion docstrings listed `completion_policy` as a per-move setting, which the robot refuses: it is a session
  setting, changed with `set_completion_policy()`. They now name all 13 per-move settings the robot accepts.

## [0.9.0] - 2026-10-22

The first numbered release. The runtime had no version before this one, so this section describes what 0.9.0
contains, then the changes that affect programs written for the earlier, unnumbered code.

### Contents

- **Real-time control loop** at 250 Hz on a core of its own, with real-time scheduling and locked memory. The CAN
  interrupt work has a core of its own too. The loop reads the drives first and sends their commands early in each
  tick.
- **Queued motion (EXEC):** joint, linear, relative and pose moves, with five motion profiles. The default profile is
  jerk-limited. Moves can be blended, and a G-code subset is supported. Pause, resume and stop all brake along the
  planned path. Every move is checked against the joint limits, the safety zones and self-collision before it
  starts. A failed move aborts the program.
- **Real-time streaming:** a hard real-time mode, where a PC sends one target every tick, and a soft mode, where the
  robot interpolates targets sent at a lower rate. Both stop under control and refuse motion the arm cannot do.
- **Safety:**
  - a safety monitor for joint limits, endstops, overspeed, tracking, a buzzing hold, and contact or collision
  - safety zones, with walls you can feel in freedrive
  - freedrive walls at every joint limit
  - self-collision checks in the planner, the monitor and jogging
  - a 100 ms drive watchdog
  - an E-stop channel policy per control box PCB version
- **Dynamics:** gravity compensation, inertia and Coriolis feedforward, a declared payload, and an external torque
  estimate that stays valid while the arm moves.
- **Homing** that recognises the arm and reports each failure with its reason.
- **Tools around the robot:**
  - the web dashboard on port 8081
  - block programming
  - drive firmware updates over CAN
  - a guided first-run check of a newly built robot
  - an activity log, incident recordings and a support bundle
  - a simulated robot for testing without hardware
- **Versioning:**
  - `robot.get_versions()` reports every part's version
  - configuration files carry a format number
  - the three protocols carry a version number each, and the Python client checks them when it connects
  - the robot and gripper XMLs name their hardware revision
  - the runtime declares the drive firmware it supports

### Changed: affects programs written for the earlier code

- **Pose angles.** A pose `[x, y, z, rx, ry, rz]` now means R = Rx(rx) Ry(ry) Rz(rz): about the base X axis by rx,
  then the tool's new Y axis by ry, then its own Z axis by rz. Stored poses, recordings and logs from before keep the
  old angles. Saved block programs in the old format convert when they are read. The Python client takes pose angles
  in radians by default, or in degrees with `angle_unit="deg"`.
- **Pose commands need a declared protocol.** The executor refuses a command that carries pose angles with
  `CLIENT_TOO_OLD` when the client declares no command protocol version, because a client from before 0.9.0 means
  the old angles. The Python client declares it. A client that sends raw JSON adds `"protocol": 1` to each message.
- **Per-joint error keys count from 1.** `J1:...` is the first joint.
- **Hard real-time streaming needs the send time** on every position target, and runs over UDP only. The Python
  client sends it by default.
- **The default EXEC motion profile is jerk-limited.** A joint move with no profile takes about 1.5 times as long as
  with the old trapezoid, while a straight TCP line was measured slightly faster. Pass `profile="trapezoid"` for the
  old timing.
- **A stop in EXEC holds the arm in EXEC** instead of switching to freedrive.
- **The drive watchdog is 100 ms.** It was 5 s. The arm is no longer held about 0.1 s after the runtime stops.
- **Log lines have one format:** `LEVEL | UTC time | tick | logger | message`. Scripts that parse the logs should use
  `utility/logger_setup.parse_line`.

### Known limitations

- This runtime is not a safety-rated system. The E-stop is the safety function.
- The control box board does not report its own version yet. The PCB version is declared in `config/system.xml`.
