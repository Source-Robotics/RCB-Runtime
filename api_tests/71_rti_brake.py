#!/usr/bin/env python3
"""71: hard RTI's CONTROLLED STOPS at speed (2026-10-13). Until now they were tested only with the arm at rest
(25 --test-timeout); this is what happens when the streaming PC dies, stops or sends garbage in the middle of a motion.

Each case streams J1 (only J1 moves) in hard RTI, robot-clocked, stamped: accelerate to --speed (60 deg/s), cruise,
then the EVENT:
  silent  the PC stops sending (a crashed / wedged controller): the robot replays on to its 40 ms watchdog, brakes,
          and latches RTI_LINK_LOST once at rest
  stop    the PC sends a stop packet (MotionFinished -> CMD_STOP): the robot brakes to rest, no error
  jump    the PC sends a target 10 deg ahead in one tick: refused (RTI_VELOCITY_LIMIT / ..._ACCELERATION_LIMIT), the
          robot brakes, the error latched once at rest
Measured, per case, from the robot's own telemetry (the `tracking` recipe — commanded and measured position / speed /
acceleration every tick): how far J1 travels after the event (vs the expected: the event's latency + the watchdog for
`silent` + v^2 / 2a at J1's RTI limits), the commanded deceleration's peak (vs the RTI acceleration limit), and — for the
two error cases — that the error arrived with the arm at rest.

Between cases J1 goes back to where it started (EXEC, 20 %). Before: the arm ENABLED, homed, error-free, IDLE; J1 free
to turn +-35 deg from where it is (the test refuses otherwise). Run it in the simulator first:
    .venv/bin/python tools/sim_run.py --tests "01_mode --go,71_rti_brake --go"
On the arm, someone at the E-stop:
    cd ~/Desktop/RCB-Runtime && .venv/bin/python api_tests/71_rti_brake.py --go
Options: --speed deg/s (60), --cases silent,stop,jump, --selftest (the analysis on synthetic traces; no robot).
Exit 0 = all passed, 1 = a check failed, 2 = refused / not run.
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
J = 0                                   # J1
DT = 0.004
ACCEL = 3.0                             # rad/s^2: the stream's own ramp (well inside every check)
CRUISE_S = 0.25
JUMP_DEG = 10.0
REST_DPS = 1.0                          # deg/s: "at rest"
TICK_LATENCY = 1                        # ticks: the event reaches the robot on its next tick
WATCHDOG_S = 0.040                      # hard RTI's command timeout

fails = []


def check(name, ok, detail=""):
    print(("  PASS  " if ok else "  FAIL  ") + name + ("" if ok or not detail else f"   [{detail}]"), flush=True)
    if not ok:
        fails.append(name)


# ------------------------------------------------------------------------------------------------ the analysis
def expected_travel_deg(case, v, a_lim, j_lim):
    """How far J1 may go after the event: the latency (+ the watchdog for `silent`) at v, then braking within a / j."""
    coast = v * DT * TICK_LATENCY + (v * WATCHDOG_S if case == "silent" else 0.0)
    brake = v * v / (2.0 * a_lim) + v * a_lim / (2.0 * j_lim)
    return math.degrees(coast + brake)


def analyse(case, trace, t_event, v, a_lim, j_lim, t_error=None):
    """trace: [(t, q_meas, q_cmd, v_meas, v_cmd, a_cmd)] in rad / rad/s / rad/s^2 from the robot's packets; t_event the
    event's time on the same clock; t_error when the error reached the PC (None: no error). Returns a dict."""
    before = [r for r in trace if r[0] <= t_event]
    after = [r for r in trace if r[0] > t_event]
    if not before or not after:
        return {"ok": False, "why": "no packets around the event"}
    q0, sgn = before[-1][1], (1.0 if before[-1][4] >= 0 else -1.0)
    # At rest: from here on the COMMANDED speed is zero — the robot's own decision (it raises an error case's error
    # there and ends the stream, so "from here on" may be one packet; the measured speed is filtered and trails it by
    # a few ms — reported, not judged).
    rest = None
    for k, r in enumerate(after):
        if all(abs(math.degrees(x[4])) < REST_DPS for x in after[k:]):
            rest = r
            break
    q_end = (rest or after[-1])[1]
    travel = math.degrees(sgn * (q_end - q0))
    peak_dec = max((-sgn * r[5] for r in after), default=0.0)
    exp = expected_travel_deg(case, v, a_lim, j_lim)
    at_rest_when_error = None
    if t_error is not None:
        last = [r for r in trace if r[0] <= t_error]
        at_rest_when_error = bool(last) and abs(math.degrees(last[-1][4])) < REST_DPS
    return {"ok": True, "travel_deg": travel, "expected_deg": exp, "peak_decel": peak_dec, "rest": rest is not None,
            "rest_s": (rest[0] - t_event) if rest else None, "at_rest_when_error": at_rest_when_error,
            "overshoot_deg": math.degrees(max(0.0, max(sgn * (r[1] - q_end) for r in after)))}


def judge(case, res, a_lim):
    if not res.get("ok"):
        check(f"{case}: analysed", False, res.get("why"))
        return
    check(f"{case}: J1 at rest after the event ({res['rest_s'] and round(res['rest_s'], 3)} s)", res["rest"])
    check(f"{case}: travel after the event {res['travel_deg']:.2f} deg <= 1.5 x expected {res['expected_deg']:.2f} + 0.3",
          res["travel_deg"] <= 1.5 * res["expected_deg"] + 0.3, f"{res['travel_deg']:.2f}")
    check(f"{case}: the commanded deceleration stays within the RTI limit (+15 %): peak {res['peak_decel']:.1f} of "
          f"{a_lim:.1f} rad/s^2", res["peak_decel"] <= 1.15 * a_lim, f"{res['peak_decel']:.1f}")
    check(f"{case}: no turning back after the stop (overshoot {res['overshoot_deg']:.3f} deg <= 0.2)",
          res["overshoot_deg"] <= 0.2, f"{res['overshoot_deg']:.3f}")
    if case in ("silent", "jump"):
        check(f"{case}: the error arrived with the arm at rest", res["at_rest_when_error"] is True,
              str(res["at_rest_when_error"]))


def selftest():
    """The analysis on synthetic traces whose answers are known: a clean brake, a brake that runs on too far, one that
    decelerates too hard, an error raised while still moving."""
    print("71 selftest: the analysis on synthetic traces")
    v, a, jl = math.radians(60.0), 32.0, 16000.0

    def brake_trace(coast_s, a_used, err_at_rest=True):
        tr, t, q, vv = [], 0.0, 0.0, v
        for _ in range(100):                         # cruise up to the event at t = 0.4 s
            tr.append((t, q, q, vv, vv, 0.0))
            t, q = t + DT, q + vv * DT
        t_ev = t - DT
        tc = 0.0
        while vv > 0:
            acc = 0.0 if tc < coast_s else -a_used
            vv = max(0.0, vv + acc * DT)
            q += vv * DT
            tr.append((t, q, q, vv, vv, acc))
            t, tc = t + DT, tc + DT
        for _ in range(60):
            tr.append((t, q, q, 0.0, 0.0, 0.0))
            t += DT
        t_err = (t - 0.1) if err_at_rest else (t_ev + coast_s + 0.01)
        return tr, t_ev, t_err

    global fails
    saved, fails = fails, []
    tr, te, terr = brake_trace(WATCHDOG_S, a)
    r = analyse("silent", tr, te, v, a, jl, terr)
    ok_clean = r["ok"] and r["rest"] and r["travel_deg"] <= 1.5 * r["expected_deg"] + 0.3 and r["at_rest_when_error"]
    tr, te, terr = brake_trace(0.3, a)
    r_far = analyse("stop", tr, te, v, a, jl)
    tr, te, terr = brake_trace(0.0, 60.0)
    r_hard = analyse("stop", tr, te, v, a, jl)
    tr, te, terr = brake_trace(WATCHDOG_S, a, err_at_rest=False)
    r_early = analyse("silent", tr, te, v, a, jl, terr)
    fails = saved
    check("selftest: a clean brake passes (travel within bounds, at rest, the error at rest)", ok_clean, r)
    check("selftest: a brake that coasts 0.3 s is caught as too far", r_far["travel_deg"] > 1.5 * r_far["expected_deg"] + 0.3,
          r_far["travel_deg"])
    check("selftest: a 60 rad/s^2 brake is caught as too hard for a 32 limit", r_hard["peak_decel"] > 1.15 * a,
          r_hard["peak_decel"])
    check("selftest: an error raised while still moving is caught", r_early["at_rest_when_error"] is False,
          r_early["at_rest_when_error"])
    print(f"\n71 selftest: {'ALL PASS' if not fails else f'{len(fails)} FAILED'}")
    return 1 if fails else 0


# ------------------------------------------------------------------------------------------------ on the robot
def run_case(robot, case, q_start, speed_dps, a_lim, j_lim, direction):
    from robotclient import RTIClient, JointPositionCommand, MotionFinished, ControlLost
    v = math.radians(speed_dps)
    t_ramp = v / ACCEL
    trace, ev = [], {}
    rti = RTIClient(robot, clock="robot")
    rti.connect()                                            # hard RTI, stamped (the client's default)
    time.sleep(0.3)
    rti.claim()
    # Drain the packets queued since pairing: control() times its callback by the robot's tick from the FIRST packet it
    # reads, so a backlog makes the stream's clock run ahead of real time (found in the simulator: the 'silent' event
    # fired 0.3 s early).
    while rti.read_state() is not None:
        pass
    q_cmd = list(q_start)

    def target_at(t):
        if t <= t_ramp:
            return 0.5 * ACCEL * t * t, ACCEL * t
        return 0.5 * ACCEL * t_ramp ** 2 + v * (t - t_ramp), v

    def cb(state):
        t = state.elapsed
        trace.append((t, state["j_p"][J], state["j_pc"][J], state["j_s"][J], state["j_sc"][J], state["j_ac"][J]))
        if "t" not in ev and t >= t_ramp + CRUISE_S:
            ev["t"] = t
        if "t" in ev:
            if case == "silent":
                return None
            if case == "stop":
                return MotionFinished()
            if case == "jump" and not ev.get("sent"):
                ev["sent"] = True
                q = list(q_cmd)
                q[J] += direction * math.radians(JUMP_DEG)
                return JointPositionCommand(q)
            return None
        d, sp = target_at(t)
        q = list(q_start)
        q[J] += direction * d
        q_cmd[:] = q
        ff = [0.0] * len(q)
        ff[J] = direction * sp
        return JointPositionCommand(q, ff)

    err = t_err = None
    t0 = time.perf_counter()
    try:
        rti.control(cb)
    except ControlLost as e:
        err, t_err = e, (trace[-1][0] if trace else None)
    if case == "stop":                                       # the stream goes on while the robot holds: watch it stop
        t_end = time.perf_counter() + 1.5
        last_t = trace[-1][0] if trace else 0.0
        while time.perf_counter() < t_end:
            try:
                s = rti.read_once(timeout=0.2)
            except Exception:                                # noqa: BLE001 — the stream ended
                break
            last_t += DT
            trace.append((last_t, s["j_p"][J], s["j_pc"][J], s["j_s"][J], s["j_sc"][J], s["j_ac"][J]))
    # The teardown of api_tests/_rti_session.py: release, close, then leave RTI mode (after a CLEAN stop the robot
    # stays in RTI, holding) — each best-effort, a link that is already gone must not mask the result. After an error
    # the robot has already left RTI (ACTIVE_ERROR): waiting for it to leave again only ran out a 10 s timeout.
    for fn in (rti.release, rti.close, lambda: robot.rti_disconnect(wait=err is None)):
        try:
            fn()
        except Exception:                                    # noqa: BLE001 — best effort
            pass
    st = robot.status()
    errors = [str(e) for e in (st.get("errors") or [])]
    if not st.get("err"):                    # no hard error: the list holds only warnings (LOOP_DEGRADED in simulation)
        errors = []
    print(f"    {len(trace)} packets in {time.perf_counter() - t0:.1f} s; error: {err!s:.120}; robot errors: {errors}")
    return trace, ev.get("t"), t_err, err, errors, v


def main():
    ap = argparse.ArgumentParser(description="71: hard RTI's controlled stops at speed")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=29000)
    ap.add_argument("--go", action="store_true", help="actually move J1")
    ap.add_argument("--speed", type=float, default=60.0, help="J1's speed at the event, deg/s")
    ap.add_argument("--cases", default="silent,stop,jump")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    if args.selftest:
        return selftest()
    if not args.go:
        print("Nothing moved: add --go (J1 moves up to ~35 deg at --speed; someone at the E-stop).")
        return 2
    from robotclient import RobotClient
    import config.xml_parser as xp
    from project_paths import GRIPPERS_DIR, ROBOTS_DIR, SYSTEM_XML
    sysc = xp.load_system_config(str(SYSTEM_XML), str(ROBOTS_DIR))
    cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                    active_gripper=sysc["active_gripper"])
    a_lim = float(cfg["rti_acceleration_limit"][J])
    j_lim = float(cfg.get("rti_jerk_limit", cfg["joint_jerk_limit"])[J])
    lo, hi = math.degrees(float(cfg["software_joint_limit_negative"][J])), math.degrees(float(cfg["software_joint_limit_positive"][J]))
    robot = RobotClient()
    robot.connect(args.host, args.port)
    st = robot.get_status()
    if st.get("state") != "ENABLED" or st.get("homed") is not True or st.get("error_active"):
        print(f"REFUSED: the arm must be ENABLED, homed and error-free (state={st.get('state')} homed={st.get('homed')} "
              f"err={st.get('error_active')})")
        return 2
    if st.get("mode") != "IDLE":
        robot.enter_idle_mode(wait=True)
    q_start = [math.radians(float(x)) for x in robot.get_joints()]
    j1 = math.degrees(q_start[J])
    span = 0.5 * ACCEL * (math.radians(args.speed) / ACCEL) ** 2 + math.radians(args.speed) * CRUISE_S
    reach = math.degrees(span) + 2 * expected_travel_deg("silent", math.radians(args.speed), a_lim, j_lim) + JUMP_DEG
    room_up, room_down = hi - j1, j1 - lo
    if max(room_up, room_down) < reach:
        print(f"REFUSED: J1 at {j1:.1f} deg needs {reach:.0f} deg of room on one side (soft limits {lo:.0f} / {hi:.0f})")
        return 2
    direction = 1.0 if room_up >= room_down else -1.0
    prev_recipe = (robot.status().get("recipe") or "standard")
    robot.set_recipe("tracking")
    print(f"71: J1 from {j1:.1f} deg, {'+' if direction > 0 else '-'} direction, {args.speed:.0f} deg/s at the event; "
          f"RTI limits a {a_lim:.1f} rad/s^2, jerk {j_lim:.0f} rad/s^3")
    try:
        for case in [c.strip() for c in args.cases.split(",") if c.strip()]:
            print(f"\n--- {case}")
            trace, t_ev, t_err, err, errors, v = run_case(robot, case, q_start, args.speed, a_lim, j_lim, direction)
            res = analyse(case, trace, t_ev if t_ev is not None else 0.0, v, a_lim, j_lim, t_err)
            if res.get("ok"):
                print(f"    travel after the event {res['travel_deg']:.2f} deg (expected <= {res['expected_deg']:.2f}), "
                      f"at rest after {res['rest_s']} s, peak deceleration {res['peak_decel']:.1f} rad/s^2")
            judge(case, res, a_lim)
            want = {"silent": "RTI_LINK_LOST", "jump": "_LIMIT", "stop": None}[case]
            if want is None:
                check("stop: no error latched", not errors, errors)
            else:
                check(f"{case}: {want} latched", any(want in e for e in errors), errors)
            robot.clear_errors(wait=True)
            robot.enable(wait=True)
            robot.enter_exec_mode(wait=True)
            robot.move_joints(*[math.degrees(x) for x in q_start], speed=20)
            robot.wait_idle(timeout=30)
            robot.enter_idle_mode(wait=True)
    finally:
        try:
            robot.set_recipe(prev_recipe)
        except Exception:                                    # noqa: BLE001
            pass
        robot.disconnect()
    print(f"\n71: {'ALL PASS' if not fails else f'{len(fails)} FAILED: ' + '; '.join(fails)}")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
