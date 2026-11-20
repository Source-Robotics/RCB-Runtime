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

- The dashboard's status bar shows the active gripper or tool (`TOOL: <name>`, `none` without one).
- A tool without a drive (the Flange, a vacuum tool, MHZ2-10D) is changed live too, both ways: from a gripper with a
  drive to the Flange and back, without restarting the robot service. The Change-gripper mode works with any tool;
  the card lists every tool file ("(no drive)" marked), and Done does not wait for a drive that is not there. Coming
  from a tool without a drive, the new gripper's drive is checked (product, tool ID) once it answers, and its loss is
  reported after one second if it never does.

- The dashboard's Change gripper is one step: with the arm enabled and homed it moves the arm to the pose a homing
  ends in (a checked move at 25 %), then disables it and begins the change; every joint holds at zero speed while the gripper is
  swapped. The card tells you when to unplug the gripper. After Done it offers "Home the arm".

- A held jog has a dead-man. The dashboard's browser tells the robot every 0.2 s while a jog button or joystick is
  pressed (`jog_alive()`, new); a jog whose keep-alives stop for 0.6 s — a lost button release, a context menu, a tab
  switched or a lost connection — is stopped. RTI brakes a jog when the executor stops answering for 0.5 s. The
  jog buttons react to the left mouse button only and work on a touch screen. A program's `jog()` is unchanged.

- The command check: in EXEC, JOG, RTI and SOFT_RTI the robot checks what it commands each tick — the commanded
  speed, the speed of each position step, the acceleration and the position — against the robot's speed and
  acceleration limits times a margin (1.2) and the soft limits. Every mode plans within those limits, so a command
  beyond them is a fault in the robot's software. It ships warning-only (`COMMAND_JUMP_WARNING`, 1208); set
  `<safety_monitor><command_check>` in `config/system.xml` to `stop` (`COMMAND_JUMP`, 1207) once normal use shows no
  warnings. Homing is not checked.
- `tools/run_nets.py --changed [REF]` runs only the tests a change touches.
- The first-run check (`tools/first_run.py`) has stages 4 and 5, and runs stages 0-5 by default. Stage 4 homes the
  arm and explains a failed homing for a builder: the robot's own reason, with what to check for that kind of failure.
  Stage 5 checks the homed arm: the gripper closes and opens, and a slow J1 move is stopped part-way.
- The dashboard's **First run** tab runs the first-run check: choose the stages, Start; its questions are buttons,
  a wait has a Skip button, Stop the run ends it early; the transcript shows every result with what to check; the
  report and earlier reports can be downloaded. The check runs as its own process (`tools/first_run.py --ui json`),
  so a reloaded page finds the run again, and the dashboard stopping ends the run cleanly (the move in progress
  stopped, the report written).
- The first-run report can carry the support bundle (`tools/first_run.py --bundle ask | yes | no`, default ask):
  one zip with the report, the logs, the incident recordings and the configuration — what to send when asking for
  help. Like the Logs tab's bundle it needs the arm disabled.
- The dashboard shows what each drive does when idle (disabled, Idle, a watchdog trip): "brake" (it resists being
  turned and slows to a stop, but does not hold a position) or "coast" (it turns freely), as the drive reports it —
  the Gripper tab's "When idle" line and the Motor tab's "When idle" column. RTI asks every drive for its identity
  every ~4 s; drive firmware before 109 does not answer and shows "---".
- Homing asks instead of failing when a joint homes more than 1 deg from where this arm's joint always homes (for
  example after the gripper was re-fitted): the joints homing in the same step finish, then the arm holds and the
  dashboard asks "Accept as new reference" (the robot keeps the new spot and homing goes on, no second homing) or
  "Stop homing" (the homing ends, not homed: park the arm and home it again). From a program:
  `status()["homing_question"]`, `accept_homing_reference()` or `stop()`, and `home(wait=True)` / `wait_homed()`
  raise the new `HomingQuestion`. It asks once the arm is recognised from the joints homed before; a first joint
  that matches no known arm still fails the homing.
- `<homing_max_travel_deg>` per joint in the robot XML: a joint whose homing search turns further than that from
  where its homing began stops and fails its homing with that reason (a start pose away from the park pose, a dead
  Hall sensor, a missing endstop). PAR6: J1 150, J2 90, J3 100, J4 220, J5 240, J6 150 deg — 1.3-2x the largest
  travel of its last 98 homings. A gripper XML can replace a joint's limit (`<arm_joint_homing_max_travel_deg>`):
  the bare Flange's J6 gets 360 deg (nothing turns with it, and it can start at any orientation); the grippers,
  Vacuum and MHZ2-10D keep 150 (they start from their usual pose). Missing = no limit, as before. Until now J6, which
  homes to a Hall sensor, was limited only by its 15 s timeout (about one full turn).
- The arm is recognised by its drives' board ids before homing (drive firmware 109+): RTI reads each drive's board id
  and calibration id at start (and a drive's again after it reconnects), and the start-up log names the arm. Homing then
  checks every joint against that arm, the first one too. Another arm's boards = a new arm, learned on its first homings
  without "Learn a new arm"; a replaced or recalibrated drive (5 of the 6 boards still match) = the warning
  `J<n>:DRIVE_CHANGED_WARNING` (2113) until a homing stores it. An arm learned before gets its boards at its next homing.
  The dashboard shows the board ids (Motor tab, the Gripper tab) and which arm they name (System Info, Arms).
- A Drive parameters tab and `read_drive_parameters()` / `get_drive_parameters()`: every parameter each drive holds
  (about 90: gains and limits, the motor model, the control loops, the watchdog, the build, the board ids), read on
  demand — "Read all" or one drive, and by default when the tab opens (a switch, remembered per browser) — and once
  by RTI itself 4 s after it starts, so they are at hand from the start. RTI asks one
  parameter every other status poll, so the frames per tick and the control loop stay as they are (about 5 s for every
  drive). The gains and limits RTI sends from the robot XML show the XML's value too when the drive holds another.
- Each drive records its own next fault (drive firmware 109+): armed at start, after Clear Errors and after each read,
  it keeps the motor current, speed and position error at 1562 Hz from 0.5 s before the fault; the robot reads it
  back after the fault (2 samples per request in the status-poll slot: the per-tick data is never delayed) and saves it
  in `temp_data/drive_captures/`; the Recordings tab plots it with the fault at 0 ms.
- Two checks of the drives 12 s after the robot starts: each drive holds the gains and limits the robot sent it (after the
  start-up parameter read; otherwise `J<n>:GAINS_MISMATCH_WARNING` (2114), both values in the log), and each is the
  product its `<driver_type>` names (otherwise `J<n>:DRIVE_PRODUCT_MISMATCH` (2115): the arm cannot be enabled until the
  drive or the XML is fixed).
- Changing drive parameters, one value at a time and read back after: the gains and limits RTI sends are applied live
  (a joint) and saved with `save_drive_parameter_to_xml()` into the robot or gripper XML — only that number changes in
  the file, every comment stays, a copy goes to `temp_data/xml_backups/` first, and the file is read again and put back
  unless that one value changed; the next start uses it. A tuning set of the drive's own values (its motor model, loop
  bandwidths and speed estimate, commutation delay, filters, the watchdog fallback, the anti-cogging and encoder-table
  switches, ...) is written into the drive with `write_drive_parameter()` while the arm is disabled, and kept after a
  power-off once `save_drive_config()` stores the drive's whole working config in its flash (arm disabled, after a
  confirmation in the dashboard; the drive does not answer for up to a second while it saves, and the robot does not
  report it lost for 3 s). Everything else stays read-only.
- Sound through the motors (drive firmware 109+): a chime when the robot is ready after it starts (~13 s, skipped
  when an error is latched then), a chime when homing ends, a tune when an error latches, `identify_arm()` ("which arm
  is this?", open to every client) and programs' tones (`beep()`, `play_tune()`, and a "beep" block in the Blocks
  editor's I/O). Each of the five is switched on or off in `config/system.xml`'s new `<sound>` block, on the System
  Info tab's Sound card or with `set_sound()`, which also chooses the playing drive (J2 as shipped: the loudest on
  PAR6), the axis of the robot's own sounds (`q` as shipped: louder, the tone is a small torque at its pitch, played
  at rest where nothing judges it; `d`: no torque), the loudness and the sound's voltage cap, and saves them in
  `system.xml` (only those values change, every comment stays, a copy goes to `temp_data/xml_backups/` first).
  Programs' tones always play on `d`: they can sound during a move, where the contact and collision checks read the
  motor current. Nothing moves; a drive plays while it holds, so the arm may be enabled or disabled, not with the
  motors off. A tune's few frames go out in the status-poll slot: the per-tick data is never delayed.
- A library of 15 short tunes for the robot's sounds (`power_up`, `rise`, `low_double`, `fanfare`, `sweep_up`,
  `sweep_down`, `ding_dong`, `double_beep`, `chirp`, `fall`, `minor`, `alarm`, `triple_low`, `heartbeat`,
  `arpeggio`): each of the robot's own sounds (ready after start, homing done, error, identify) plays the one picked
  for it on the Sound card (a ▶ plays each) or with `set_sound(boot_tune=..., homing_done_tune=..., error_tune=...,
  identify_tune=...)`; `play_sound(name)` plays one from a program. The robot's own sounds and library tunes play on
  the robot's sound axis at rest and on `d` in EXEC, JOG, RTI and SOFT_RTI, so a tone never disturbs a move.
- The gripper's tool ID (drive firmware 109+): each sized gripper XML declares `<tool_id>` (1 the SSG-48; an MSG 11-33,
  the tens digit its rail 100 / 150 / 200 mm, the units its motor small / medium / large), and the robot compares it
  with the tool ID the gripper's drive reports, about once a second. Another ID stops the robot
  (`GRIPPER_TOOL_MISMATCH`, 2116: it cannot be enabled, since the gripper XML's tool length, mass and collision model
  would be wrong); a drive with none stored gets a note. The Gripper tab shows both and stores the XML's ID in the
  drive with a confirmation (`store_gripper_tool_id()`: the ID and a flash save, the arm disabled).
- Change the gripper without losing the homing: `begin_gripper_change()` (the Gripper tab's "Change gripper", the arm
  disabled; its joints keep holding) idles the jaws and stops counting the gripper's silence and faults, so unplugging
  it no longer un-homes the arm; enabling, the gripper's commands and every mode but IDLE and motors off are refused
  meanwhile, and so are parameter writes, drive saves and storing a tool ID on the gripper's drive; a gripper plugged
  in again gets its settings by itself. `end_gripper_change()` (Done) once it answers; then calibrate its jaws. `latest_status()["gripper"]` reports `changing`, `answering`, the fitted drive's `tool_id` and
  the gripper files it `proposed`.
- Another gripper, live: `end_gripper_change("SSG48", removed=True)` (the Gripper tab's card: the file its tool ID
  proposes, confirmed) applies that gripper's file without a restart: RTI takes its settings, its tool length, mass
  and collision model and its homing set-up within three loop ticks; the gripper's drive must be the product the file
  names and carry its tool ID (or none). `<active_gripper>` is saved in `config/system.xml` first (put back when RTI
  refuses the file), then RTI applies it, then the executor, the self-collision checker and the dashboard restart with
  it (about 10 s); the reply comes after all three, and from the apply until that restart enabling and moves are
  refused. A step that fails replies `GRIPPER_SWAP_FAILED` (6027) naming it; when RTI fails while applying, the arm
  cannot be enabled until the robot service restarts. The arm keeps holding and its homing. `removed` (the card's box,
  unticked by default and confirmed when ticked): the gripper left the flange, so J5 and J6 are no longer homed (J6's
  Hall magnet is on the gripper side).
  For now grippers with a drive only. `tools/restart_gui_and_executor.sh --with-self-collision` restarts the checker
  too.
- `scripts/setup_system.sh` gives the robot service its own real-time limit, `LimitRTPRIO=99`
  (`/etc/systemd/system/autorun.service.d/rtprio.conf`). RTI gets its priority under the service even after an
  `apt upgrade` of Python, which removes the capability on the Python binary; before, RTI then refused to start at the
  next restart. Run `sudo bash scripts/setup_system.sh` once, then restart the service. README *Real-time priority*
  has a troubleshooting entry for an RTI that keeps restarting.

### Changed

- A `move_joints()` or `move_linear()` (or its relative forms) whose blend cannot be done with its profile is refused
  with `BAD_OPTIONS` before anything is planned: `blend_mode="zone"` or `"tolerance"` needs `profile="toppra"`,
  `"ruckig_corner"` needs `profile="ruckig"` and a `move_joints()`. Such moves used to run one by one with only a log
  line, and their blend in millimetres was read as an exact-velocity percent, which put velocity steps between them.
- J2's speed limit is 1.2 rad/s (the joint, EXEC and RTI limits; it was 1.5). Its drive has always capped it at
  1.23 rad/s, so a move above about 82 % fell behind its command; the arm moves as fast as before. The configuration
  check now warns when any joint's drive cap is more than 2 % below its speed limit.
- The first-run check's overall result is INCOMPLETE (exit code 3) when no step passed, warned or failed: every step
  skipped was a PASS. Its configuration step is a WARN when the configuration loads with warnings (they were hidden
  under a PASS), and the check gives control back after each step that took it, so the Enable it asks for in the
  dashboard is not refused. Its stop test moves J1 30 deg instead of 15, so the stop comes mid-move.
- A configured gripper whose file does not exist stops the start with the list of the gripper files there (it was
  silently "no gripper": J5 homed with the wrong offset). `none` still means no gripper.
- RTI.log writes one line for a short burst of late CAN replies, naming every joint in it, instead of one line per
  joint.
- The generic gripper file `grippers/MSG.xml` is removed: it could not be used as shipped.

- Drive firmware 110 or newer is what this runtime supports (it said 103): an older drive is a warning at run time, on
  the Versions card, and a FAIL in the first-run check. Every drive of the arm runs 110 since 2026-11-16; the runtime
  relies on it (the watchdog's hold, the board ids, the drive parameters, sound).

- `end_gripper_change(removed=True)` (the gripper taken off the flange) leaves the whole arm not homed, instead of
  J5 and J6 only: the arm homes again. That homing takes J5's and J6's new spots for this gripper
  without asking; J1 to J4 are checked as always. J6's spot is stored per gripper, as J5's already was.

- The robotics toolbox is the official `roboticstoolbox-python` 1.5.0 (with `spatialmath-python` 1.1.19) instead of
  our fork `roboticstoolbox-python-source-robotics` 1.3.0.post2. Both install the same `roboticstoolbox` package:
  `scripts/install.sh` removes the fork first. The robot's live numbers (gravity, feedforward, kinematics) come from
  Pinocchio and do not change. The reachability search (`is_reachable`, block programs' pose moves, dry runs) reads
  either toolbox's IK result and keeps the robot's IK chain instead of rebuilding it at every solve.
- **The gripper's object codes mean what the gripper firmware sends: 1 = object detected while CLOSING, 2 = while
  OPENING** (they were the other way round). The driver library `Spectral_BLDC` 1.20 read the firmware's two bits
  swapped, and the runtime, the client's `GripperResult` and the dashboard were built on that; with 1.22 (now pinned)
  every part uses the firmware's meaning. A program that tests `code == 2` after a close must test `1` (or use
  `GripperResult.object_detected`, unchanged). Codes 0 and 3 are unchanged.
- **Command protocol 3** (was 2), for the gripper codes above: a client and a robot of different command protocols
  refuse each other at connect, so a 1.1.x client cannot read the codes the old way on a new robot. Update the
  robotclient on every PC together with the robot. A client that sends raw JSON sends `"protocol": 3`.
- `open_gripper()` / `close_gripper()`, the dashboard's Open / Close and the blocks' gripper scale use the gripper's
  full stroke, positions 0 and 255 (they were 1 and 254). Gripper firmware 110 sets both calibrated ends clear of the
  mechanical stops, so the jaws arrive there with a few mA instead of pressing into a stop; on older gripper firmware
  0 and 255 lay inside the stops (flash the gripper to 110 first).
- The first-run check waits up to 15 s for the dashboard to let go of control (it holds it for 5 s after the last
  click, for example on Enable) instead of skipping the step at once.

### Fixed

- The first-run check's small test moves (stage 3) no longer move a joint the way that has no room. On an arm not
  homed yet, the check asks whether the arm rests folded and moves each joint only its free way from that rest pose
  (the robot XML's new `<first_run_move from_rest>`, `+` when missing), never the other way automatically; on a homed
  arm it plans each test move both ways first (joint limits, safety zones, self-collision; nothing moves) and moves
  only a way that passes. A test move refused with "no position read yet — try again" is retried the same way (it was
  taken for blocked and the joint was moved the other way), and the robot's joint test takes its start position from
  the drive's last data reply, so that refusal is rare. The direction question is answered by the + and - signs on the
  joint (+, -, I saw no movement, or show me again: the same move once more, up to 3 times), and the by-hand variant
  asks to turn each joint toward its + sign; the robot XML's `<positive_direction>` words now say where the + sign
  belongs and appear in a backwards FAIL's hint.
- The first-run check's stop test (stage 5) clears a refused plan before it tries the other way (every later move was
  refused `MOTION_ABORTED`); it stops J1 once it has turned 10 deg of its 30 (it stopped at a fixed 0.6 s, when the
  slow, gently accelerating move had barely started, so the stop was never tested) and judges the commanded J1 too (a
  move that did not play is told apart from a joint that did not follow). The dashboard's First run tab showed the
  result line twice.
- The first-run check's Hall-sensor step guides in short steps by the + and - signs on the joint, with a beep through
  the arm at every switch (ON high, OFF low): find the magnet (either way until the sensor switches ON), then three
  times back until OFF and slowly the way the joint homes until ON — the point homing stops at. The three ON points
  must lie within 2 deg. An ON reached turning the other way is said and not counted. The repeat question is gone: it
  asked to turn the joint "the SAME way" and compared the robot's latest edge angles, which read about 30 deg apart on
  a good sensor when the joint was turned the other way or overshot.
- The Gripper tab's Mode toggle sent a command whenever the display updated it to the robot's mode: right after a
  gripper swap a `NO_GRIPPER` error (the executor still had the old tool), control taken for 5 s on opening the tab,
  and a click undone before the robot confirmed it. Only a click sends now.
- After a gripper swap the supervisor logged the planned restart of the executor, the dashboard and the collision
  checker as three errors "stopped unexpectedly"; `tools/restart_gui_and_executor.sh` now leaves a note, and they are
  logged as planned. The swap dialog says the robot answers again within about a minute (it said about 10 s).
- A batch of EXEC trajectory that does not start where the arm is commanded to be (farther than its joints can move
  in one tick) is refused before it moves: the program is dropped and `EXEC_DISCONTINUITY` (3103) is raised. An EXEC
  underrun (the path goes on, but no batch arrived) is logged and counted; it was a silent dead stop. A Stop never
  starts the next command of the stopped program.
- An exact-velocity blend checks the next move's target (joint limits, safety zones, self-collision) before it plans a
  corner speed into it: a next move that fails them made the arm stop dead at the corner from that speed.
- The command check no longer trips on every JOG / EXEC entry when a joint was pushed past its soft limit by hand: a
  joint that starts past it trips only when it is commanded a tolerance further out, as OUT_OF_RANGE does.
- The collision check takes a new baseline after the momentum observer restarts (as `set_payload` makes it do): the
  newly declared payload read as contact on the next move.
- `<jog_defaults>` in system.xml: a speed of 20 (as every API speed since protocol 2) or `nan` was clamped to full
  jog speed, and an acceleration time of `nan` gave the hardest ramp, without a word. A value that is not a finite
  number in its range now gets the default, with a warning naming the field.
- The configuration check no longer warns "kt not found" for a tool without a drive (the Flange, a vacuum or
  pneumatic gripper).
- The MSG gripper files other than the measured `MSG_small_motor_150mm_rail` get its calibration wait (without it
  every 12th homing failed), the two other small-motor files its measured Kt and gains, the large-motor 150 mm file
  its inertia in the right order, and each a `<not_measured>` list that makes the configuration check warn while it
  is active (the medium / large motors' Kt and gains and every variant's jaw travel and mass are placeholders).

- Every 16.4 s the CAN commands fell up to 20 ms behind for about 80 ms (each joint missed 4-5 replies in a row).
  The cause was the SD card: Linux re-tunes it every 16.4 s, about 41 ms of kernel work on whichever core uses
  the card next, the CAN core too, where it held up the CAN send work. `scripts/setup_system.sh` now switches
  the periodic re-tuning off in the Pi 5 device trees (the card still tunes at boot and after a checksum error)
  and runs the card at DDR50 (top speed about 50 MB/s; its wide timing margin needs no periodic re-tuning).
  `--check` reports a device tree a firmware update has replaced. It takes effect after a reboot.
- A CAN dropout of 40-200 ms during homing re-sent the drive's full current limit, so the rest of the homing pushed
  into the endstop at full current (and a freed or Hall-checked joint was energised under the user's hand). RTI now
  records the limits it last sent each drive, and a reconnect re-sends those.
- A homing move that timed out with a joint far from its target carried on, and J4 / J6 then homed in a pose nobody
  checked. A joint more than 2 deg from its target now fails the homing, naming it and how far it was.
- The homing's endstop detection counted high current over the whole approach: a burst early on armed it, and any
  short pause later read as the endstop (a false stall). It now looks at the last 80 ms only.
- The arm check recognised a known arm from one homed joint alone, even when the drive boards said it was a new arm;
  it now needs two joints, and never when the boards say "new".
- An error while the homing question waited left the question open (the dialog kept reopening, Accept was refused),
  and the Joints table kept "homed" after a failed homing.
- The PID tuner sent J1's values to whichever joint was selected: its fields were loaded once. They now reload when
  the joint changes, and Apply refuses fields loaded for another joint or a current limit of 0.
- The dashboard's SW E-STOP button showed each page's own idea of the E-stop: on another page, or after a reload,
  pressing "SW E-STOP" released it. It and Grav Comp now show the robot's state on every page.
- The control box's "Stop all", "Reboot Pi" and "Shutdown" restarted every script in the same pass. "Restart RTI" /
  GUI / HL ask for confirmation, and "Restart RTI" is refused while the arm is enabled.
- The supervisor stopped RTI on any error in its main loop (a UART fault dropped the arm), and started nothing when
  the OLED did not open: both are logged now, and the robot runs on.
- A sound node the robot does not have (`<sound><node>` 7-15) ended RTI at the boot chime after every start; the tune
  is dropped and logged.
- A reset from a refused IP ended the executor; a CAN read error ended RTI (read-first mode, the default) — now it
  reads nothing, the drives go stale and CAN_LOST latches with RTI running.
- A J6 jog stopped after exactly 360 deg with a false self-collision block (the scan stopped at one turn; J6's soft
  range is 458 deg).

- `scripts/check_system.py` passed a Pi on which RTI cannot start. It tried real-time priority 1 (RTI needs 99) and
  reported a refusal as a warning under "no required failures". It now tries 99 (on a core away from RTI's, back to
  normal priority at once), fails on the Pi with the fix, and counts the warnings in its last line. Its CAN lines show
  the bus state and the error counters (the state was cut to "can state ERR", the same for a healthy and a failing
  bus). Run as `python3 scripts/check_system.py`, it now switches to the repo's `.venv` as it always meant to (it
  checked the system Python's packages).
- RTI's refusal to start without real-time priority gave the wrong fix: "run under the service (root)" (the service
  runs as the robot's user) or turning `<realtime_required>` off (the loop then runs late). It now names the Python
  binary, the setup script and the `setcap` command, the `apt upgrade` that removes the capability, or, when RTI's core
  is not available, starting RTI through the service.
- The arm no longer drops a little when it is disabled out of gravity compensation or stops on an error. Drive
  firmware 110 starts its speed loop from zero when it takes over from another mode, so the bare zero-speed hold
  (no current) let the joints fall for about 10 ms before it caught them (0.2-0.4 deg on the elbow and the wrist).
  Once the arm is homed and every joint answers, the hold now carries the gravity model's torque as the drive's
  feedforward current.
- A Stop pressed while the planner was still planning a move was undone by it: the move's plan, finishing after the
  Stop, wrote its end point back, so the next move started from the cancelled move's end (a step at its start).
- A move whose samples hold a NaN or an infinite value, or none at all, is refused (`PLAN_FAIL`, nothing of it sent);
  RTI refuses such a batch too, fails its command and brakes to rest. It used to raise inside RTI's loop and end RTI.
- One client that stopped reading (a suspended PC, a paused script) could block every reply and, through them, a
  Stop: the stop path now tells RTI first and replies after, each client has its own send lock, and a client that
  does not read for 2 s is disconnected.
- A command sent while RTI was not running (during a restart) was kept and run when RTI came back, after the client
  had been told RTI did not answer: such a command now fails at once with `RTI_NOT_RUNNING`.
- A stop, a disable, an error or motors off now also stop a running gripper calibration (it swept on for up to 30 s).
- The error tune after an error in a move plays on the d axis (no torque) instead of q, and a q tune still playing is
  silenced when a move mode begins; a new tune replaces what still waits (a program beeping in a loop queued a minute
  of sound and slowed RTI's other polls).
- A stop of a running program after the executor alone had restarted (`tools/restart_gui_and_executor.sh`, or a
  gripper change) did not brake: the planner numbered its stops from 0 again while RTI kept the highest number it had
  seen and took the new stop for an older one — yet confirmed it at once, so `stop()` returned "ok" while the arm
  played out what it had. The planner now numbers its stops from the clock, so a restarted executor always continues
  above.
- RTI could not start with any gripper file but `MSG_small_motor_150mm_rail`: a gripper without
  `<homing_calibration_wait_s>` gave a NaN calibration wait, and the start-up crashed rounding it (since 2026-10-24).
  An undeclared wait is 2 s again.

- The first-run check's Hall-sensor check (stage 2), its test moves (stage 3) and its by-hand check failed with "the
  check itself failed" when the robot refused to start them; they now report the robot's reason. A test move refused
  one way, for example past a soft limit on a homed arm, is tried the other way.
- The first-run check, found while writing its guide:
  - skipping a button's release or an input's switch-back skips that button or input alone, as it already did at the
    press; stopping the run at a question keeps the results already measured in that step;
  - Ctrl-C while a step works on its own (the setup check, the CAN-bus watch) skips that step instead of ending the
    tool without a report;
  - skipping the homing, or a homing that runs longer than 180 s, stops it (the arm is then not homed); a homing
    failure names its joint;
  - a gripper move that does not finish within 30 s is a FAIL that names it, not "the check itself failed";
  - the stop test warns instead of passing when J1 had not started moving before the stop;
  - by hand: skipping while a joint is being freed skips that joint alone; a joint turned but never held still is not
    reported as an encoder fault; the question says what the other joints do meanwhile (hold, float in freedrive, or
    limp when the arm is disabled).
- An RTI command packet with a value that is not a number (NaN or infinite) ended the robot's control loop, and J2
  and J3 sagged until the drives braked. It is now refused like the other stream checks: the arm brakes to rest along
  its path and `RTI_BAD_VALUE` (6207) latches.
- A joint current beyond what the drive frame carries (±32767 mA, for example a torque given in the wrong unit) was
  sent wrapped around: full current in the opposite direction. Every joint's current is now kept within its `<Ilim>`
  in the robot XML (the drive's own cap, so normal motion is unchanged), with a warning; a current that is not a
  number is sent as 0 mA instead of ending the control loop.
- A CAN error frame (for example a bus-off) was counted as a reply from J1. Error, remote and extended frames are now
  ignored by the reply handling.
- A drive's torque constant (Kt, read at start with `kt_source` auto) was used whatever it was, NaN included. A value
  further than `<kt_tolerance>` (new in `config/system.xml`, 0.3 = ±30 %) from the robot XML's Kt now keeps the XML
  value, the start-up log names both values, and the warning `KT_MISMATCH_WARNING` (2110) shows for that joint (or
  the gripper) until the robot service restarts.
- The command check's warning `COMMAND_JUMP_WARNING` (1208) would have disabled the robot like an error: the error
  system kept its own list of which codes are warnings, and it lacked this one. The list now comes from the error
  catalog, so every code documented as a warning is one.
- A voltage limit above 60000 mV (the most a drive accepts) is clamped to it instead of reaching the drive.
- Every joint brakes when idle, whatever its drive has saved: the robot sends brake at its start and after a joint
  reconnects (in the drive's RAM, never saved). Drive firmware 110 saves brake-or-coast, and a coast saved by anyone
  would have let J2 and J3 drop at every stop, fault, homing idle and flash.
- A drive fault with none of the known causes latched nothing: the joint stopped without a named error. It now latches
  `NOT_CALIBRATED` (2111) when the drive reports its motor not calibrated, else `DRIVE_FAULT` (2112).
- `tools/flash_motor_firmware.py app-reset` waited 2 s for the bootloader, which appears ~8 s after the reset on the
  arm's drives: it always reported "did not answer". It now waits as long as `flash` does.
- `tools/arm_smoke.py` refuses to start, naming the joint, while a joint is past its soft limit: its RTI link test
  holds the current pose, and from a pose left by hand slightly past a soft limit the robot (correctly) refused to
  hold it, half-way through the run.
- The dashboard's Kt values and the gripper's force estimate showed and used each drive's polled Kt, also when the
  robot was using the robot XML's instead. They now show and use the Kt the robot chose at start.
- `grippers/MSG_small_motor_150mm_rail.xml`: `<kt>` 0.174 Nm/A, what its drive reports (it was 0.111, an unverified
  placeholder, 57 % off).
- With the IK engine set to `roboticstoolbox` (not the default), a failed solve came back as an answer — up to 65 cm
  off the target — because the fork reports success as 0 / 1, which the check missed. It is now a failure.

## [1.1.1] - 2026-10-25

### Fixed

- Switching the read-first tick order at runtime (`set_can_read_first()`) no longer opens or closes a socket inside
  the control loop, where it could stall the loop like the loop-delay measurement did: the socket is opened once when
  the robot software starts. If it could not open then, the switch is refused (`RTI_REFUSED`).
- `rti_claim()` and `rti_release()` refused with no PC paired said "sub_state is 'None'"; they now say the session
  is not paired (`rti_connect()` first).

## [1.1.0] - 2026-10-25

### Added

- `RTI_REFUSED` (6026): the real-time controller refused a command; the message gives its reason.

### Changed

- Every command the robot passes to its real-time controller (58 of them: `enable`, `home`, the mode changes,
  `stop`, the jog, gripper and real-time settings, `hall_check`, `free_joint`, `joint_test`, `set_payload`,
  `set_recipe`, the reloads, ...) now replies with the controller's own answer. A refusal raises `CommandRejected`
  with its code and reason (`MODE_REJECTED`, `ENABLE_REJECTED`, `HALL_CHECK_REFUSED`, `FREE_JOINT_REFUSED`,
  `JOINT_TEST_REFUSED`, `RTI_REFUSED`); until now the reply was "ok" and the refusal reached only the activity log.
  No answer within 1 s raises `RTI_NOT_RUNNING`. These replies take about one control tick longer (4 to 8 ms).
  Asking for the mode the robot is already in is still answered "ok".

- The test recordings in `temp_data` (what the measurement scripts in `api_tests` save) are kept to 500 MB together,
  down from 1 GB; the oldest runs are deleted first. On a Pi with a larger SD card, set
  `utility/housekeeping.RECORDINGS_BUDGET_BYTES` back to 1 GB.
- The control loop's delay per joint is measured on request instead of live: the dashboard's RT Diagnostics card has
  a Measure button (3 s), and `tools/loop_delay.py` does the same in a terminal. A separate process records the CAN
  bus and reads the control loop's own timing; the control loop does nothing for it.

### Fixed

- Single control-loop ticks of 8 to 44 ms that latched LOOP_CRITICAL and disabled the arm, now and then and at
  random: the live loop-delay measurement switched a CAN socket filter inside the control loop, and that call waits
  for the operating system's network lock whenever anything else holds it. A Wi-Fi reconnect was the usual trigger.
  The control loop makes no such call any more.

## [1.0.0] - 2026-10-25

### Added

- Full docstrings for the robot control commands (`enable`, `disable`, `home`, `clear_errors`, the mode changes,
  `stop`, `pause`, `resume`, `safety_stop` and its release, the flashing commands, and the control commands): what
  each does in every mode, every code it can be refused with, what it raises, and an example.
- The same for every other public `RobotClient` method (waiting, connection and events, motion settings, jogging,
  I/O, the gripper, status and data, safety and dynamics, the real-time interface, diagnostics and service), and for
  `RTIClient` with its classes and `GripperResult`.

### Changed

- **Every speed share is a percent, and the command protocol is 2.** `set_joint_speed()`, a move's `speed`, `accel`
  and `jerk`, and `jog()`'s speed take a percent: above 0 and at most 100 (0 to 100 for `jog()`), so `1` means 1 %
  and `0.5` half a percent. A value of 1 or less used to be a fraction, so a program asking for "1 %" ran at 100 %;
  `jog()`'s speed was always a fraction. A program that passes fractions must pass percents now. A client and a robot
  of different command protocols refuse each other at connect instead of misreading a speed.
- When the client holding control disconnects, or loses its connection, while moves it queued still run, the robot
  stops the arm as `stop()` does. They used to play to the end with nobody connected. Call `wait_idle()` before
  disconnecting to let them finish.
- A homing that Stop, Disable or another mode interrupts leaves the arm not homed: home it again. The arm used to stay
  marked homed while the joints that had started homing still read as homing.
- Every `rti_connect()` switches the rate limiter, the start-pose check and real-time gravity compensation on, and sets
  the tool speed cap back to its start value. A session used to inherit them switched off from an earlier one, from any
  PC.
- A collision-detection change returns to its start value when the client that made it releases control, loses it or
  disconnects. It used to last until the robot software restarted. `"warn"` and `"stop"` are refused when nothing can
  be checked, and the status reports `"off"` while the safety monitor is off.
- `set_gravity_comp()` is refused while a joint moves or a PC drives the arm in a real-time session: the switch is a
  step in the joints' torque.
- `rti_connect()` is refused outside IDLE mode and the real-time mode it asks for, and while a PC drives the arm; it
  used to end a running claim without braking. `rti_disconnect()` is refused outside IDLE mode, the real-time modes and
  the error state; it used to end a safety stop or a firmware flash and abandon a homing.
- The real-time interface's settings, `set_gripper()`, `set_joint_speed()` and `set_blending()` refuse a value outside
  their range with `BAD_ARGS`. A negative or NaN tool speed cap used to switch the cap off, and a `set_gripper()`
  current of -1 reached the drive as 65535 mA.
- Gripper commands are refused on a robot without a CAN gripper (`NO_GRIPPER`), and the queued ones while the gripper is
  in motor mode (`GRIPPER_MOTOR_MODE`). They used to be accepted and then time out or settle without moving.
- `get_joints()`, `get_pose()` and `get_digital_input()` are refused with `RTI_NOT_RUNNING` while the control loop is
  not running or the robot is flashing, instead of answering old values.
- `set_log_level()` refuses the loggers of the robot's other processes, which it never reached.
- The jog's ramp settings (`set_jog_accel()`, `set_jog_jerk_factor()`, `set_jog_profile()`) take effect once the joint
  is at rest: a longer ramp applied mid-jog could carry the joint past its soft limit. `set_jog_profile()` and
  `set_jog_control_mode()` take any letter case, and the profile also its usual other spellings.
- Arcs, circles and splines ignore the session's default blend (`set_blending()`); one that passes its own blend above 0
  still fails. Every curve used to fail while a default blend was set.
- The Hall sensor check stops by itself 2 minutes after the last `hall_check(True)`, as a freed joint does.
- `calibrate_gripper()` ends when the gripper reports itself calibrated, and drops the gripper's target: the jaws no
  longer move back to an old target 30 s later.
- A gripper command's settle time grows at low speeds (about 30 s at speed 15); a slow full stroke used to fail after
  5 s.
- A first real-time command too far from the arm latches `RTI_START_POSE` instead of `RTI_LINK_LOST`.
- The client's callbacks run on a thread of their own, so a command or a wait inside one works. `on_error()` receives
  the robot's id-less errors, `on_motion_error()` the failures of a lost connection, and a failure a call already
  raised is not raised again by `wait_idle()`. Waits raise `RobotConnectionError` for a lost connection. `connect()`
  refuses a second connect and a connection that never answered as a robot, and starts afresh. A joint number must be a
  whole number.
- The telemetry recipe returns to `"standard"` only when the control loop starts, no longer when the dashboard or
  another process restarts.
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

- `safety_stop()` now stops the gripper's jaws where they are, as `stop()`, `disable()` and an error do.
- A raw client's argument the robot could not read dropped that client's connection, and with it control. It is
  answered `BAD_ARGS` now, a fault on the robot's side `INTERNAL`.
- A gripper command that settled on its position could report code 0, "still moving", so `wait_released_part()` failed
  an open that arrived. It reports 3.
- `rti_set_servo_generator()` stored a gain before it refused a bad look-ahead; the gain then took effect later.
- `set_pid_gains()` is recorded with its values in the activity log, and `joint_test()` checks the current limit in
  force after it.
- `reload_recipes()` without a recipe folder answered "ok"; it answers `RECIPE_DIR`.
- A network that fails silently is noticed in about 11 s on both sides (TCP keepalive); it took minutes, or never with
  nothing to send.
- `set_log_level("rti.handler", "DEBUG")` no longer writes a line for every real-time packet from inside the control
  loop.
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
