"""S0 — the Pi and the configuration: nothing needs power yet (2026-10-18, TODO/first_run_plan.md).

    setup     scripts/setup_system.sh --check: the kernel, boot settings, Python packages, groups, the service
    config    config/system.xml's robot + gripper, both XMLs parsed and validated (the runtime's own validator)
    pcb       the control box version system.xml declares, confirmed by the user against the board
    service   a connection to the robot service and RTI's loop running
"""
from __future__ import annotations

import subprocess

from first_run.core import FAIL, INFO, PASS, WARN, result, step
from project_paths import GRIPPERS_DIR, PROJECT_ROOT, ROBOTS_DIR, SYSTEM_XML

SETUP_SCRIPT = PROJECT_ROOT / "scripts" / "setup_system.sh"
SETUP_TIMEOUT_S = 180


def _is_one(v) -> bool:
    """An XML flag read as a number (1 / 1.0) — NaN, None or text are not set."""
    try:
        f = float(v)
    except (TypeError, ValueError):
        return False
    return f == f and int(f) == 1


def parse_setup_check(text) -> dict:
    """setup_system.sh's report -> {"missing": [(what, [fixes])], "notes": [..], "ok": n}. Its lines: '  OK  ...',
    '  MISSING  ...' followed by '           -> fix' lines, '  NOTE  ...'."""
    missing, notes, ok = [], [], 0
    for line in str(text).splitlines():
        body = line.strip()
        if body.startswith("MISSING"):
            missing.append((body[len("MISSING"):].strip(), []))
        elif body.startswith("->") and missing:
            missing[-1][1].append(body[2:].strip())
        elif body.startswith("NOTE"):
            notes.append(body[len("NOTE"):].strip())
        elif body.startswith("OK"):
            ok += 1
    return {"missing": missing, "notes": notes, "ok": ok}


def check_setup(ctx, ui):
    st = SETUP
    ui.say("Checking the Pi's setup (scripts/setup_system.sh --check, a few seconds) ...")
    run = ctx.get("run", subprocess.run)
    try:
        p = run(["bash", str(SETUP_SCRIPT), "--check"], capture_output=True, text=True, timeout=SETUP_TIMEOUT_S)
    except (OSError, subprocess.SubprocessError) as e:
        return result(st, WARN, f"could not run the setup check: {e}",
                      hints=[f"run it by hand: bash {SETUP_SCRIPT} --check"])
    rep = parse_setup_check(p.stdout)
    details = {"output": p.stdout[-6000:], "exit_code": p.returncode, "notes": rep["notes"]}
    if rep["missing"]:
        hints = [f"{what}" + (f" — fix: {'; '.join(fixes)}" if fixes else "") for what, fixes in rep["missing"]]
        hints.append(f"then: sudo bash {SETUP_SCRIPT} (applies what it can) and run this check again")
        return result(st, FAIL, f"{len(rep['missing'])} item(s) missing in the Pi's setup", value=len(rep["missing"]),
                      expected=0, hints=hints, details=details)
    if p.returncode != 0:
        return result(st, WARN, f"the setup check exited with {p.returncode} but named nothing missing",
                      hints=[f"read its output: bash {SETUP_SCRIPT} --check"], details=details)
    return result(st, PASS, f"the Pi's setup is complete ({rep['ok']} checks OK)", value=0, expected=0,
                  details=details)


def check_config(ctx, ui):
    import config.xml_parser as xp   # here, not at import: a test of the other stages needs no XML parser

    st = CONFIG
    try:
        sysc = xp.load_system_config(SYSTEM_XML, ROBOTS_DIR)
        cfg = xp.load_full_robot_config(str(sysc["robot_xml_path"]), grippers_dir=GRIPPERS_DIR,
                                        active_gripper=sysc["active_gripper"])
    except Exception as e:   # any parse failure is the finding
        return result(st, FAIL, f"the configuration does not load: {type(e).__name__}: {e}",
                      hints=[f"{SYSTEM_XML}: <robot> must name a file in robots/, <gripper> one in grippers/",
                             "an XML syntax error: the message names the file and the line"])
    ok, issues = xp.validate_robot_config(cfg)
    can_gripper = _is_one(cfg.get("gripper_CAN_gripper", 0))        # NaN = no gripper
    hall = [i for i, v in enumerate(list(cfg.get("has_hall_sensor", []))[:int(cfg["joint_num"])]) if _is_one(v)]
    ctx["config"] = {"robot": sysc["active_robot"], "gripper": sysc["active_gripper"],
                     "joint_num": int(cfg["joint_num"]), "can_gripper": can_gripper,
                     "hall_joint": hall[0] if hall else None, "pcb_version": int(sysc["pcb_version"]),
                     "positive_direction": list(cfg.get("positive_direction") or []),
                     "first_run_move": list(cfg.get("first_run_move") or [])}
    value = {k: ctx["config"][k] for k in ("robot", "gripper", "joint_num", "can_gripper")}
    summary = (f"robot {sysc['active_robot']} ({cfg['joint_num']} joints), gripper {sysc['active_gripper']}"
               f"{' (on CAN)' if can_gripper else ''}")
    if not ok:
        return result(st, FAIL, summary + f" — {len(issues)} problem(s) in the robot / gripper XML", value=value,
                      hints=list(issues), details={"issues": list(issues)})
    return result(st, PASS, summary, value=value, details={"issues": list(issues)})


def check_pcb(ctx, ui):
    from hardware.control_box import capabilities

    st = PCB
    cfg = ctx.get("config")
    if not cfg:
        return result(st, INFO, "skipped: the configuration did not load")
    v = cfg["pcb_version"]
    ch = capabilities(v)["estop_channels"]
    answer = ui.choose(f"config/system.xml says the control box has PCB v{v} (its E-stop is read on {ch} channel"
                       f"{'s' if ch != 1 else ''}). Is that the board in your box? The version is printed on the board.",
                       ["yes", "no", "not sure"])
    if answer == "yes":
        return result(st, PASS, f"PCB v{v}, confirmed by the user", value=v)
    if answer == "no":
        return result(st, FAIL, f"system.xml declares PCB v{v}, the user says the board differs", value=v,
                      hints=["set <control_box><pcb_version> in config/system.xml to the board's version, then "
                             "restart the robot service (sudo systemctl restart autorun.service): the E-stop is read "
                             "according to it"])
    return result(st, WARN, f"system.xml declares PCB v{v}, not confirmed", value=v,
                  hints=["look for the version on the control box's PCB (silkscreen) before relying on the E-stop "
                         "check; v1 boards read the E-stop on one channel only"])


def check_service(ctx, ui):
    st = SERVICE
    try:
        robot = ctx["connect"]()
    except Exception as e:   # the connection failing is the finding
        return result(st, FAIL, f"no connection to the robot service: {type(e).__name__}: {e}",
                      hints=["start it: sudo systemctl start autorun.service",
                             "its state: systemctl status autorun.service; the logs: logs/supervisor.log, "
                             "logs/high_level.log (the executor), logs/RTI.log",
                             "the next stages need it: they read the drives through it"])
    ctx["robot"] = robot
    health = robot.get_health() or {}
    loop = next((c for c in health.get("components", []) if c.get("name") == "control loop"), None)
    if loop is None or loop.get("level") == "STALE":
        return result(st, FAIL, "the robot service answers but RTI's control loop is not running",
                      hints=["logs/RTI.log and logs/RTI.out.log say why it stopped (a configuration error is named "
                             "there)", "restart: sudo systemctl restart autorun.service"],
                      details={"health": health})
    hw_n = ctx.get("config", {}).get("joint_num")
    status = robot.get_status() or {}
    details = {"health": health, "status": status}
    return result(st, PASS, f"connected; RTI running ({loop.get('summary', '')}); mode {status.get('mode')}, "
                  f"{status.get('state')}", value=hw_n, details=details)


SETUP = step("S0.setup", 0, "The Pi's setup", check_setup,
             about="the kernel, boot settings, Python packages, groups and the robot service (setup_system.sh)")
CONFIG = step("S0.config", 0, "The configuration loads", check_config,
              about="config/system.xml's robot and gripper, parsed and checked by the runtime's own validator")
PCB = step("S0.pcb", 0, "The control box version", check_pcb,
           about="the PCB version system.xml declares (it decides how the E-stop is read), confirmed against the board")
SERVICE = step("S0.service", 0, "The robot service runs", check_service,
               about="a connection to the robot's executor, and RTI's control loop ticking")
STEPS = [SETUP, CONFIG, PCB, SERVICE]
