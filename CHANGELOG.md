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

Nothing yet.

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
