#!/usr/bin/env python3
"""06: digital IO and the gripper suite.

The gripper half exists because EXEC was validated without one fitted. It fills
the four unticked rows in TODO/exec_command_verification_checklist.md, plus two
paths that checklist does not cover.

--gripper-signal answers the question everything else rests on: does
`gripper_object_detection` reset to 0 when a NEW move starts, or hold a stale
value from the previous one? If it held stale, _poll_gripper_completions() would
fire on its first poll — every command reporting complete before the jaws move,
wait_idle() returning early, and every other test here passing while meaning
nothing. That failure is silent, which is why it still runs first.

ANSWERED 2026-08-06 on the MSG gripper: it RESETS. Measured 3 -> 0 (t+20 ms) ->
3 (t+722 ms). Completion no longer depends on it either way — the planner
cross-checks commanded position and travel direction — but the trace is still
worth re-running on new gripper hardware or new firmware.

    python3 api_tests/06_io.py --gripper-signal --go     # DO THIS ONE FIRST
    python3 api_tests/06_io.py --gripper-all --go        # unattended set
    python3 api_tests/06_io.py --gripper-object --go     # needs an object in the jaws
    python3 api_tests/06_io.py --gripper-timeout --go    # needs the jaws blocked

Position is a RAW 0-255 scale (1 = fully open, 254 = fully closed), NOT
millimeters — firmware-mode grippers expose no verified mm conversion.
"""
import time

from _common import (parser, connect, require_go, prepare_exec, goto_start,
                     finish, show, main_guard)
from robotclient import GripFailed


def _pins(args):
    if args.all:
        return [1, 2, 3]
    if args.pins:
        return [int(pin) for pin in args.pins]
    return [int(args.pin)]


def _read_inputs(robot, pins):
    return {pin: bool(robot.get_digital_input(pin)) for pin in pins}


def _print_inputs(states, prefix="inputs"):
    parts = [f"{pin}={'ON' if value else 'off'}" for pin, value in states.items()]
    print(f"{prefix}: " + ", ".join(parts), flush=True)


def _watch_inputs(robot, pins, interval, count, mirror_outputs=False, clear_on_exit=False):
    print("Watching digital inputs. Press Ctrl-C to stop.", flush=True)
    last = None
    ticks = 0
    try:
        while count <= 0 or ticks < count:
            states = _read_inputs(robot, pins)
            if states != last:
                _print_inputs(states, prefix=time.strftime("%H:%M:%S"))
                last = states
            if mirror_outputs:
                for pin, value in states.items():
                    robot.set_digital_output(pin, value)
            ticks += 1
            time.sleep(max(0.02, float(interval)))
    except KeyboardInterrupt:
        print("\nStopped live IO watch.")
    finally:
        if clear_on_exit:
            for pin in pins:
                robot.set_digital_output(pin, False)


def _blink_outputs(robot, pins, count, period):
    half = max(0.02, float(period) / 2.0)
    for i in range(int(count)):
        print(f"Blink {i + 1}/{int(count)}: outputs ON {pins}", flush=True)
        for pin in pins:
            robot.set_digital_output(pin, True)
        time.sleep(half)
        print(f"Blink {i + 1}/{int(count)}: outputs off {pins}", flush=True)
        for pin in pins:
            robot.set_digital_output(pin, False)
        time.sleep(half)


# --------------------------------------------------------------------------- #
# Gripper helpers
# --------------------------------------------------------------------------- #
# gripper_object_detection is read only by the planner, server-side, straight
# from shared memory — there is no client accessor and it is not in the status
# stream. Sampling it at ~100 Hz is the whole point of the signal test, so this
# reads shared memory directly. That ties --gripper-signal to running ON the
# robot; every other gripper test here works over TCP from anywhere.
_GRIPPER_SHM = None


def _gripper_shm():
    global _GRIPPER_SHM
    if _GRIPPER_SHM is None:
        from data import shared_data
        _GRIPPER_SHM = shared_data.gripper
    return _GRIPPER_SHM


def _detect():
    return int(_gripper_shm()['gripper_object_detection'][0])


def _pos():
    return int(_gripper_shm()['gripper_position'][0])


_DETECT_MEANING = {0: "moving", 1: "object while opening", 2: "object while closing",
                   3: "reached target, no object"}


def _sample_until_settled(limit_s, period=0.01):
    """Sample (t, detect, position) until detect goes nonzero or limit_s elapses."""
    trace, start = [], time.perf_counter()
    while True:
        now = time.perf_counter() - start
        det = _detect()
        trace.append((now, det, _pos()))
        if det != 0 and now > 0.05:      # ignore a stale value in the first frame
            break
        if now >= limit_s:
            break
        time.sleep(period)
    return trace


def _summarise(trace, label):
    """Collapse a sample trace to its transitions — that is what the test is about."""
    print(f"\n-- {label} --")
    transitions, last = [], None
    for t, det, pos in trace:
        if det != last:
            transitions.append((t, det, pos))
            last = det
    for t, det, pos in transitions:
        print(f"   t={t:6.3f}s  detect={det} ({_DETECT_MEANING.get(det, '?')})  pos={pos}")
    print(f"   {len(trace)} samples over {trace[-1][0]:.3f}s, "
          f"{len(transitions)} transition(s)")
    return transitions


def _errors_collector(robot):
    """Capture motion errors so SUPERSEDED / GRIPPER_SETTLE_TIMEOUT can be asserted."""
    seen = []
    robot.on_motion_error(lambda info: seen.append(info))
    return seen


def test_gripper_signal(robot, args):
    """THE test: does gripper_object_detection reset to 0 when a new move starts?

    Drives the gripper to a known end, then commands the opposite end while
    sampling. A correct signal reads nonzero (settled from the previous move),
    drops to 0 as the new move starts, and returns nonzero on arrival.
    """
    print("\n=== GRIPPER SIGNAL: does the completion status reset on a new move? ===")
    print("Pre-positioning (open) so the status is settled before we start...")
    robot.gripper_open(timeout=args.timeout)
    robot.wait_motion_complete(timeout=args.timeout)
    time.sleep(0.5)

    before = _detect()
    print(f"status BEFORE the new command: detect={before} "
          f"({_DETECT_MEANING.get(before, '?')}), pos={_pos()}")
    if before == 0:
        print("   NOTE: already 0 while idle — this test cannot distinguish 'reset'\n"
              "   from 'never set'. Check the open above actually completed.")

    print("Commanding close, sampling at 100 Hz ...")
    robot.gripper_close(timeout=args.timeout)
    trace = _sample_until_settled(args.timeout)
    transitions = _summarise(trace, "close after open")

    dropped_to_zero = any(det == 0 for _t, det, _p in trace[1:])
    settled = trace[-1][1] != 0
    print("\nVERDICT")
    if before != 0 and dropped_to_zero and settled:
        print("  PASS — status reset to 0 on the new move and went nonzero on arrival.")
        print("  Completion tracking is REAL. The rest of the suite is meaningful.")
    elif before != 0 and not dropped_to_zero:
        print("  *** FAIL — the status held its stale nonzero value. ***")
        print("  _poll_gripper_completions() will fire on its first poll, so every")
        print("  gripper command reports complete before the jaws move, and")
        print("  wait_idle() returns early. Do not trust the other gripper tests.")
        print("  Fix: gate completion on a 0-observation first, or on position.")
    else:
        print("  INCONCLUSIVE — see the trace above.")
    return transitions


def test_gripper_openclose(robot, args):
    """Checklist rows 1-2: open and close in free space, with settle timing."""
    print("\n=== GRIPPER OPEN / CLOSE (free space) ===")
    for name, call in (("open", robot.gripper_open), ("close", robot.gripper_close)):
        t0 = time.perf_counter()
        call(timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        dt = time.perf_counter() - t0
        print(f"  {name:5} completed in {dt:5.2f}s  ->  pos={_pos()} detect={_detect()}")


def test_gripper_positions(robot, args):
    """Checklist row 3: raw 0-255 targets reach and settle."""
    print("\n=== GRIPPER POSITIONS (raw 0-255 scale) ===")
    print(f"  {'target':>7}  {'reached':>7}  {'err':>5}  {'time':>6}  detect")
    for target in args.gripper_positions:
        t0 = time.perf_counter()
        robot.move_gripper(int(target), timeout=args.timeout)
        robot.wait_motion_complete(timeout=args.timeout)
        dt = time.perf_counter() - t0
        reached = _pos()
        print(f"  {target:>7}  {reached:>7}  {reached - int(target):>5}  {dt:5.2f}s  "
              f"{_detect()}")


def test_gripper_supersede(robot, args):
    """Checklist row 4: a second command must SUPERSEDE the first, not falsely complete it.

    ASYNC ONLY. In sync mode _queued() blocks until the checkpoint fires, so a
    single client physically cannot issue the second command while the first is
    in flight — the condition under test cannot arise.
    """
    print("\n=== GRIPPER SUPERSEDE (two commands, no wait between) ===")
    if args.sync:
        print("  SKIPPED: --sync blocks on each command, so nothing can be superseded.")
        return
    seen = _errors_collector(robot)
    robot.gripper_open(timeout=args.timeout)
    time.sleep(0.05)                       # long enough to be in flight, short enough to catch
    robot.gripper_close(timeout=args.timeout)
    try:
        robot.wait_idle(timeout=args.timeout, raise_on_error=False)
    except Exception as exc:
        print(f"  wait_idle raised: {exc}")
    codes = [e.get("code") for e in seen]
    print(f"  motion errors seen: {codes or 'none'}")
    if "SUPERSEDED" in codes:
        print("  PASS — the overwritten command errored instead of falsely completing.")
    else:
        print("  *** FAIL — expected SUPERSEDED on the first command. ***")
        print("  Without it, an overwritten target reports success it never reached.")


def test_gripper_timeout(robot, args):
    """Probe the GRIPPER_SETTLE_TIMEOUT path. Not in the checklist.

    HARDWARE FINDING 2026-08-06: blocking the jaws does NOT reach this path. The
    gripper reports "object detected while closing" (code 2) for an obstruction
    exactly as it does for a part — it cannot tell them apart, and should not
    have to. So the command completes normally in ~0.4 s, correctly.

    GRIPPER_SETTLE_TIMEOUT fires only when the gripper reports NOTHING for
    GRIPPER_SETTLE_TIMEOUT_S — a CAN dropout or a firmware hang mid-move, not a
    mechanical obstruction. Staging that by hand means pulling the gripper's CAN
    connection while a move is in flight, which also disturbs every joint on the
    bus. Left deliberately unstaged; this test documents the boundary rather
    than contriving a dangerous reproduction.

    What it DOES verify: a blocked gripper resolves promptly and cleanly instead
    of wedging the motion queue, and reports the reason.
    """
    print("\n=== GRIPPER TIMEOUT (jaws must be BLOCKED for this to mean anything) ===")
    print("  Expecting GRIPPER_SETTLE_TIMEOUT after GRIPPER_SETTLE_TIMEOUT_S (5s default).")

    # OPEN FIRST. Without this the test is a no-op: if the gripper is already
    # closed (which it is, after every other test in this suite), the close
    # target is inside GRIPPER_POSITION_TOLERANCE and completes on the position
    # criterion in ~40 ms without the jaws ever moving. Blocking them then
    # proves nothing. Found 2026-08-06 on the first hardware run of this test.
    print("  Opening first so the close has somewhere to travel ...")
    robot.gripper_open(timeout=args.timeout)
    robot.wait_gripper_move_complete(timeout=args.timeout)
    print(f"  opened to pos={_pos()}")
    input("  BLOCK THE JAWS so they cannot close, then press Enter... ")

    seen = _errors_collector(robot)
    t0 = time.perf_counter()
    robot.gripper_close(timeout=args.timeout)
    try:
        robot.wait_idle(timeout=args.timeout, raise_on_error=False)
    except Exception as exc:
        print(f"  wait_idle raised: {exc}")
    dt = time.perf_counter() - t0
    codes = [e.get("code") for e in seen]
    outcome = robot.gripper_result()
    print(f"  resolved after {dt:.2f}s; motion errors: {codes or 'none'}")
    print(f"  outcome: {outcome!r}  (shared memory detect={_detect()}, pos={_pos()})")
    if any("TIMEOUT" in str(c) for c in codes):
        print("  PASS — failed cleanly rather than hanging the queue.")
    elif dt < 0.2:
        print(f"  INVALID — resolved in {dt:.2f}s, far too fast to have moved.")
        print("  The gripper was already at the target, so nothing was tested.")
    elif outcome is not None and outcome.object_detected:
        print("  EXPECTED — the obstruction was reported as an object (code "
              f"{outcome.code}), not a timeout.")
        print("  A blocked jaw and a gripped part are the same signal; the gripper")
        print("  cannot distinguish them. The queue resolved cleanly, which is the")
        print("  property that matters here. GRIPPER_SETTLE_TIMEOUT needs a gripper that")
        print("  reports NOTHING (CAN dropout / firmware hang) — see the docstring.")
    elif not codes:
        print("  Completed with no error and no object reported — unexpected.")
        print("  Check the outcome line above.")


def test_gripper_object(robot, args):
    """Does gripping an OBJECT report 1/2, versus 3 for free travel? Not in the checklist."""
    print("\n=== GRIPPER OBJECT DETECTION (put something in the jaws) ===")
    robot.gripper_open(timeout=args.timeout)
    robot.wait_motion_complete(timeout=args.timeout)
    input("  Place an object between the jaws, then press Enter... ")
    robot.gripper_close(timeout=args.timeout)
    trace = _sample_until_settled(args.timeout)
    _summarise(trace, "close onto an object")
    final = trace[-1][1]
    print(f"\n  shared memory: detect={final} ({_DETECT_MEANING.get(final, '?')}), pos={_pos()}")

    # The same answer through the public API — this is what a real pick would
    # use, and it must agree with the raw signal above. If it does not, the
    # outcome is being lost somewhere between the planner and the client.
    robot.wait_motion_complete(timeout=args.timeout)
    api = robot.gripper_result()
    if api is None:
        print("  *** API returned no gripper result — the outcome is not reaching")
        print("      the client. Pick verification is impossible over TCP. ***")
    else:
        print(f"  via API      : {api!r}")
        if api.code != final:
            print(f"  *** API code {api.code} disagrees with shared memory {final}. ***")

    # The safe idiom, end to end — this is what a real pick should look like.
    from robotclient import GripFailed
    try:
        held = robot.wait_holding_part(timeout=args.timeout)
        print(f"  wait_holding_part() -> {held!r}")
    except GripFailed as exc:
        print(f"  wait_holding_part() raised GripFailed: {exc}")
        print("  (correct IF the jaws are empty; a real failure if they are not)")

    if final in (1, 2):
        print("  PASS — reported an object rather than 'reached target'.")
        print("  This is what makes a grip distinguishable from a miss.")
    elif final == 3:
        print("  *** Reported 'reached target, no object' while holding one. ***")
        print("  Grip success cannot be detected from this signal on this gripper.")

    # --- the release half -------------------------------------------------
    # wait_released_part() is the mirror of wait_holding_part() and had no
    # coverage at all. Opening with the part still between the jaws is the
    # case worth catching: the arm would otherwise travel away carrying it.
    print("\n  --- release ---")
    input("  Leave the object in place and press Enter to OPEN... ")
    robot.gripper_open(timeout=args.timeout)
    try:
        released = robot.wait_released_part(timeout=args.timeout)
        print(f"  wait_released_part() -> {released!r}")
        print("  PASS — the jaws opened clear of the object.")
    except GripFailed as exc:
        print(f"  wait_released_part() raised GripFailed: {exc}")
        print("  Correct IF something still blocks the jaws; a real failure if")
        print("  they are visibly clear.")
    print(f"  final: detect={_detect()} pos={_pos()}")


def test_gripper_sequencing(robot, args):
    """Does a queued joint move WAIT for the gripper? Documented as: no.

    executive_mode_plan.md marks gripper-parallel-execution NOT built, and the
    planner docstring says dispatch stays FIFO but does not block on physical
    completion. So the arm is expected to start moving while the jaws travel.
    Confirming that is the point — a pick-and-place written assuming otherwise
    releases its part mid-move.
    """
    print("\n=== GRIPPER / ARM SEQUENCING ===")
    if args.sync:
        print("  SKIPPED: --sync blocks on gripper_close, so the overlap this test")
        print("  measures cannot occur. That is exactly why sync mode is the safe")
        print("  default for pick-and-place — run this without --sync to see the")
        print("  overlap that async code has to handle itself.")
        return
    from _common import START_POSE_DEG
    robot.gripper_open(timeout=args.timeout)
    robot.wait_motion_complete(timeout=args.timeout)

    print("  Queuing gripper_close then move_joints with NO wait between them.")
    t0 = time.perf_counter()
    robot.gripper_close(timeout=args.timeout)
    target = list(START_POSE_DEG)
    target[0] += 10.0
    robot.move_joints(target, speed=args.speed, timeout=args.timeout)

    gripper_settled_at = None
    arm_started_at = None
    start_j0 = robot.get_joints()[0]
    while time.perf_counter() - t0 < args.timeout:
        if gripper_settled_at is None and _detect() != 0:
            gripper_settled_at = time.perf_counter() - t0
        if arm_started_at is None and abs(robot.get_joints()[0] - start_j0) > 0.5:
            arm_started_at = time.perf_counter() - t0
        if gripper_settled_at is not None and arm_started_at is not None:
            break
        time.sleep(0.01)
    robot.wait_idle(timeout=args.timeout, raise_on_error=False)

    print(f"  gripper settled at t={gripper_settled_at}")
    print(f"  arm started at    t={arm_started_at}")
    if arm_started_at is not None and gripper_settled_at is not None \
            and arm_started_at < gripper_settled_at:
        print("  CONFIRMED (as documented): the arm does NOT wait for the gripper.")
        print("  Use wait_motion_complete() between them for strict sequencing.")
    else:
        print("  The arm did not start before the gripper settled — either the")
        print("  gripper was very fast, or sequencing differs from the docs.")


def run():
    p = parser("06: digital IO and gripper checks")
    p.add_argument("--pin", type=int, default=1)
    p.add_argument("--pins", type=int, nargs="+", help="input/output pins to test; default is --pin")
    p.add_argument("--all", action="store_true", help="use pins 1, 2, and 3")
    p.add_argument("--watch", action="store_true", help="live-read inputs until Ctrl-C or --count")
    p.add_argument("--interval", type=float, default=0.1, help="live watch polling interval in seconds")
    p.add_argument("--count", type=int, default=0, help="watch samples; 0 means until Ctrl-C")
    p.add_argument("--blink", action="store_true", help="blink selected output LEDs; requires --go")
    p.add_argument("--blink-count", type=int, default=5)
    p.add_argument("--blink-period", type=float, default=0.5)
    p.add_argument("--mirror", action="store_true",
                   help="mirror selected input states to same-numbered outputs; requires --go")
    p.add_argument("--clear-outputs-on-exit", action="store_true",
                   help="turn selected outputs off when live output mode exits")
    p.add_argument("--gripper", action="store_true", help="gripper open/close with settle timing")
    p.add_argument("--gripper-signal", action="store_true",
                   help="RUN THIS FIRST: does gripper_object_detection reset on a new "
                        "move? Everything else assumes it does. Must run ON the robot.")
    p.add_argument("--gripper-positions", type=int, nargs="+", metavar="POS",
                   help="raw 0-255 targets to sweep, e.g. --gripper-positions 1 64 128 200 254")
    p.add_argument("--gripper-supersede", action="store_true",
                   help="queue two gripper commands; the first must report SUPERSEDED")
    p.add_argument("--gripper-timeout", action="store_true",
                   help="BLOCK THE JAWS first: expect GRIPPER_SETTLE_TIMEOUT, not a hang")
    p.add_argument("--gripper-object", action="store_true",
                   help="prompts you to place an object: expect detect 1/2, not 3")
    p.add_argument("--gripper-sequencing", action="store_true",
                   help="confirm a queued joint move does NOT wait for the gripper")
    p.add_argument("--gripper-all", action="store_true",
                   help="the unattended gripper set (signal, open/close, positions, "
                        "supersede) — excludes the ones needing an object or blocked jaws")
    args = p.parse_args()

    if args.gripper_all:
        args.gripper_signal = args.gripper = args.gripper_supersede = True
        if not args.gripper_positions:
            args.gripper_positions = [1, 64, 128, 200, 254]

    robot = connect(args)
    pins = _pins(args)
    show("digital inputs", _read_inputs(robot, pins))

    if args.watch and not args.mirror:
        _watch_inputs(robot, pins, args.interval, args.count)
    elif args.mirror:
        if require_go(args, "mirror digital inputs to same-numbered outputs"):
            _watch_inputs(robot, pins, args.interval, args.count,
                          mirror_outputs=True,
                          clear_on_exit=args.clear_outputs_on_exit)

    if args.blink:
        if require_go(args, "blink selected digital outputs"):
            _blink_outputs(robot, pins, args.blink_count, args.blink_period)

    if not args.watch and not args.blink and not args.mirror:
        if require_go(args, "toggle digital output and optional gripper"):
            for pin in pins:
                robot.set_digital_output(pin, True)
            for pin in pins:
                robot.set_digital_output(pin, False)

    # --- gripper suite -----------------------------------------------------
    # Ordered deliberately: the signal test first, because every test after it
    # assumes completion tracking is real.
    _gripper_asked = any((args.gripper_signal, args.gripper, args.gripper_positions,
                          args.gripper_supersede, args.gripper_timeout,
                          args.gripper_object, args.gripper_sequencing))
    if _gripper_asked and require_go(args, "drive the gripper"):
        # Gripper commands are QUEUED, so they need EXEC + ENABLED + homed just
        # like a joint move — the digital-IO half above is immediate and does
        # not. Without this the whole suite fails at the first gripper_open.
        prepare_exec(robot, args)
        if args.gripper_sequencing:
            # The only gripper test that moves the ARM, so it is the only one
            # that needs a known starting pose.
            goto_start(robot, args)
        if args.gripper_signal:
            test_gripper_signal(robot, args)
        if args.gripper:
            test_gripper_openclose(robot, args)
        if args.gripper_positions:
            test_gripper_positions(robot, args)
        if args.gripper_supersede:
            test_gripper_supersede(robot, args)
        if args.gripper_object:
            test_gripper_object(robot, args)
        if args.gripper_timeout:
            test_gripper_timeout(robot, args)
        if args.gripper_sequencing:
            test_gripper_sequencing(robot, args)

    if not args.go and not args.watch:
        print("Read-only IO check complete. Use --watch for live input monitoring.")
    finish(robot)


if __name__ == "__main__":
    main_guard(run)
