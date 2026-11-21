#!/usr/bin/env python3
"""Preflight checks for RCB Control Runtime installs.

The checker is intentionally diagnostic: it does not bring hardware up, start
services, or modify system state. Missing robot hardware is reported clearly so a
fresh clone can fail usefully instead of crashing later in RTI startup. The one
thing it tries for real is RTI's real-time priority, in a short-lived child
process kept off RTI's core (check_rt_permissions).

On a robot Pi the complete check is `bash scripts/setup_system.sh --check`; this
one is the quick one (README "OS image").
"""

from __future__ import annotations

import os
import platform
import re
import resource
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config.cpu_cores import CAN_CPU_CORE, RT_CPU_CORE                        # noqa: E402  (a leaf: no imports)
from hardware.can_health import format_can_link_stats, read_can_link_stats  # noqa: E402  (stdlib only)

VENV = ROOT / ".venv"
VENV_PYTHON = VENV / "bin" / "python"
RTI_PRIORITY = 99   # RTI.py: setup_realtime_process(cpu_core=RT_CPU_CORE, priority=99, ...)
RTPRIO_DROPIN = Path("/etc/systemd/system/autorun.service.d/rtprio.conf")   # scripts/setup_system.sh


def reexec_in_venv() -> None:
    """Run the check with the robot's interpreter, the repo's .venv: its packages, its real-time permission.

    2026-11-18: this compared the two interpreters' resolved paths, but .venv/bin/python is a link to the system's
    python3.11 — the same file — so `python3 scripts/check_system.py` never switched and checked the system Python's
    packages. A venv is told by its prefix.
    """
    if VENV_PYTHON.exists() and Path(sys.prefix).resolve() != VENV.resolve():
        os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), __file__, *sys.argv[1:]])


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    required: bool = True


checks: list[Check] = []


def add(name: str, ok: bool, detail: str, required: bool = True) -> None:
    checks.append(Check(name, ok, detail, required))


def run(cmd: list[str]) -> subprocess.CompletedProcess[str] | None:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, check=False)
    except FileNotFoundError:
        return None


def file_text(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip("\x00\n ")
    except OSError:
        return None


def on_robot_pi() -> bool:
    """A Raspberry Pi: the robot's target, where RTI must get its real-time priority (a PC is for development)."""
    return "Raspberry Pi" in (file_text(Path("/proc/device-tree/model")) or "")


def check_python() -> None:
    version = ".".join(map(str, sys.version_info[:3]))
    add("Python >= 3.11", sys.version_info >= (3, 11), version)


def check_repo_layout() -> None:
    required = ["RTI.py", "supervisor.py", "command_executor.py", "config/system.xml", "requirements.txt"]
    missing = [item for item in required if not (ROOT / item).exists()]
    detail = f"root={ROOT}" if not missing else f"missing: {', '.join(missing)}"
    add("Repo layout", not missing, detail)
    for dirname in ["logs", "temp_data"]:
        path = ROOT / dirname
        try:
            path.mkdir(exist_ok=True)
            add(f"Writable {dirname}/", os.access(path, os.W_OK), str(path))
        except OSError as exc:
            add(f"Writable {dirname}/", False, str(exc))


def check_disk() -> None:
    usage = shutil.disk_usage(ROOT)
    free_gb = usage.free / (1024 ** 3)
    add("Free disk >= 1GB", free_gb >= 1.0, f"{free_gb:.2f} GB free", required=False)


def check_platform() -> None:
    model = file_text(Path("/proc/device-tree/model"))
    if model:
        add("Raspberry Pi 5 target", "Raspberry Pi 5" in model, model, required=False)
    else:
        add("Raspberry Pi 5 target", False, "not running on a Pi or model unavailable", required=False)

    kernel = platform.release()
    realtime_flag = file_text(Path("/sys/kernel/realtime"))
    rt_ok = realtime_flag == "1" or "rt" in kernel.lower() or "preempt_rt" in kernel.lower()
    add("RT kernel hint", rt_ok, f"kernel={kernel}, /sys/kernel/realtime={realtime_flag!r}", required=False)

    cpu_count = os.cpu_count() or 0
    add("CPU cores >= 4", cpu_count >= 4, f"{cpu_count} core(s)")


def check_can() -> None:
    result = run(["ip", "-details", "link", "show", "can0"])
    if result is None:
        add("ip command", False, "iproute2 not installed")
        return
    if result.returncode != 0:
        add("CAN can0 exists", False, result.stderr.strip() or "can0 not found")
        return
    output = result.stdout.replace("\n", " ")
    add("CAN can0 exists", True, "can0 found")
    up = "state UP" in output or "UP" in output.split()
    add("CAN can0 up", up, "state UP" if up else output[:180])
    rate = re.search(r"\bbitrate (\d+)", output)
    add("CAN bitrate 1000000", rate is not None and rate.group(1) == "1000000",
        f"bitrate {rate.group(1)}" if rate else "no bitrate in 'ip -details link show can0' (not configured yet)",
        required=False)
    check_can_bus_state(read_can_link_stats("can0"))


# What a CAN controller state other than ERROR-ACTIVE means. The state counts the frames that failed recently (no
# acknowledgement, a bit error): a few failures make it ERROR-WARNING, more ERROR-PASSIVE, too many BUS-OFF.
CAN_STATE_MEANING = {
    "ERROR-WARNING": "frames are failing on the bus",
    "ERROR-PASSIVE": "frames keep failing on the bus",
    "BUS-OFF": "the controller left the bus after too many failed frames",
    "STOPPED": "the controller is stopped (can0 down?)",
}
CAN_STATE_CAUSES = ("usual causes: no drive powered (nothing acknowledges a frame), the termination (about 60 ohm "
                    "between CAN-H and CAN-L with the power off; about 120 ohm = one terminator missing) or the wiring")


def check_can_bus_state(stats: dict | None) -> None:
    """The CAN controller's state and its error counters since can0 came up.

    2026-11-18 (a builder's report): the CAN lines cut the interface text at 180 characters, so the state read
    "can state ERR" — the same for ERROR-ACTIVE (healthy) and ERROR-WARNING / ERROR-PASSIVE (frames failing). A few
    error-passive transitions from start-ups with the drives switched off are normal; totals that grow while the arm
    runs are not (supervisor.py logs every change, hardware/can_health.py).
    """
    if not stats or not stats.get("state"):
        add("CAN bus state", False, "cannot read it (ip -details -statistics link show can0)", required=False)
        return
    state = stats["state"]
    if state == "ERROR-ACTIVE":
        add("CAN bus state", True, f"ERROR-ACTIVE (healthy); totals: {format_can_link_stats(stats)}", required=False)
        return
    meaning = CAN_STATE_MEANING.get(state, "not the healthy ERROR-ACTIVE")
    add("CAN bus state", False, f"{state}: {meaning}; {CAN_STATE_CAUSES}; totals: {format_can_link_stats(stats)}",
        required=False)


def check_devices() -> None:
    devices = [
        ("I2C device", Path("/dev/i2c-1"), False),
        ("UART device", Path("/dev/ttyAMA0"), False),
    ]
    for label, path, required in devices:
        exists = path.exists()
        access = os.access(path, os.R_OK | os.W_OK) if exists else False
        detail = f"{path} exists={exists} read/write={access}"
        add(label, exists and access, detail, required=required)

    gpiochips = sorted(Path("/dev").glob("gpiochip*"))
    usable = [path for path in gpiochips if os.access(path, os.R_OK | os.W_OK)]
    if usable:
        detail = "usable: " + ", ".join(str(path) for path in usable)
        add("GPIO chip access", True, detail, required=False)
    else:
        detail = "found: " + (", ".join(str(path) for path in gpiochips) if gpiochips else "none")
        add("GPIO chip access", False, detail, required=False)


# The real-time test, run by a child of this Python (the robot's interpreter). argv: the priority, the cores to keep
# off. It pins itself to a core away from RTI's and the CAN path's FIRST — at FIFO 99 on RTI's core it would compete
# with a running robot's tick — takes SCHED_FIFO at the priority, reads it back and returns to normal scheduling at
# once (always allowed). Exit 0 = granted and back to normal.
RT_PROBE = """
import os, sys
prio, avoid = int(sys.argv[1]), {int(c) for c in sys.argv[2].split(",")}
cores = sorted(os.sched_getaffinity(0) - avoid)
if not cores:
    print(f"no core outside {sorted(avoid)} to test on (allowed: {sorted(os.sched_getaffinity(0))})")
    sys.exit(3)
os.sched_setaffinity(0, {cores[0]})
ran = sorted(os.sched_getaffinity(0))
try:
    os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(prio))
    got = (os.sched_getscheduler(0), os.sched_getparam(0).sched_priority)
except OSError as e:
    print(f"{type(e).__name__}: {e}")
    sys.exit(1)
finally:
    os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
if got != (os.SCHED_FIFO, prio):
    print(f"asked for SCHED_FIFO {prio}, got policy {got[0]} priority {got[1]}")
    sys.exit(1)
if os.sched_getscheduler(0) != os.SCHED_OTHER:
    print("did not return to normal scheduling")
    sys.exit(1)
print(f"SCHED_FIFO {prio} granted (tried on core(s) {ran}, then back to normal)")
"""


def check_rt_permissions(robot_pi: bool) -> None:
    """RTI's real-time priority (SCHED_FIFO 99), tried for real with this Python — the robot's interpreter.

    2026-11-18 (a builder's report): this tried priority 1 and reported a refusal as a warning, so a Pi on which RTI
    refuses to start passed the check ("no required failures"). Now it tries RTI's own priority, and on the robot's
    Pi a refusal is a failure with the fix. Under autorun.service, rtprio.conf (LimitRTPRIO=99) grants it without the
    capability; a shell — where this check runs — has only the capability, so with rtprio.conf in place a refusal
    here is a warning: the service's RTI still starts.
    """
    soft, hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
    binary = os.path.realpath(sys.executable)
    getcap = run(["getcap", binary])
    if getcap is None:
        cap = "getcap not installed"
    else:
        cap = "has cap_sys_nice" if "cap_sys_nice" in getcap.stdout else "no cap_sys_nice"
    facts = f"{binary}: {cap}; RLIMIT_RTPRIO soft={soft} hard={hard}"
    probe = subprocess.run([sys.executable, "-c", RT_PROBE, str(RTI_PRIORITY), f"{RT_CPU_CORE},{CAN_CPU_CORE}"],
                           capture_output=True, text=True, check=False)
    said = " ".join((probe.stdout + probe.stderr).split()) or f"exit code {probe.returncode}"
    if probe.returncode == 0:
        add("RT priority permission", True, f"{said}; {facts}")
        return
    fix = ("Fix: sudo bash scripts/setup_system.sh — it installs autorun.service with rtprio.conf (LimitRTPRIO=99), "
           "where RTI gets its priority (review I8, 2026-11-21: no capability on the interpreter any more)")
    service_ok = "LimitRTPRIO=99" in (file_text(RTPRIO_DROPIN) or "")
    if service_ok:
        add("RT priority permission", False,
            f"this shell cannot get SCHED_FIFO {RTI_PRIORITY} ({said}; {facts}). RTI under autorun.service still gets "
            f"it ({RTPRIO_DROPIN}), an RTI started any other way does not. {fix}", required=False)
        return
    add("RT priority permission", False,
        f"RTI needs SCHED_FIFO {RTI_PRIORITY} and cannot get it ({said}; {facts}): RTI refuses to start. {fix}",
        required=robot_pi)


MODULES_FILE = ROOT / "scripts" / "required_modules.txt"


def required_modules() -> dict:
    """{module: required} from scripts/required_modules.txt — the one list setup_system.sh checks too (review I8,
    2026-11-21: this file had its own, with pinokin, which nothing imports any more, and without coal)."""
    modules = {}
    for line in MODULES_FILE.read_text(encoding="utf-8").splitlines():
        words = line.split("#", 1)[0].split()
        if words:
            modules[words[0]] = "optional" not in words[1:]
    return modules


def check_imports() -> None:
    modules = required_modules()
    for module, required in modules.items():
        code = f"import importlib; importlib.import_module({module!r})"
        result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=False)
        if result.returncode == 0:
            stderr = " ".join(result.stderr.split())
            detail = "ok" if not stderr else f"ok, emitted stderr: {stderr[:180]}"
            add(f"Import {module}", True, detail, required=required)
        else:
            stderr = " ".join(result.stderr.split())
            stdout = " ".join(result.stdout.split())
            detail = stderr or stdout or f"exit code {result.returncode}"
            add(f"Import {module}", False, detail[:220], required=required)


def summary_line(required_failures: int, warnings: int) -> str:
    """The last line. 2026-11-18: it said "no required failures" alone, and a warning that RTI cannot start went
    unread; the warnings are counted now."""
    if required_failures:
        return (f"System check finished with {required_failures} required failure(s)"
                + (f" and {warnings} warning(s)." if warnings else "."))
    if warnings:
        return f"System check finished with no required failures and {warnings} warning(s): read the WARN lines."
    return "System check finished with no required failures."


def arm_enabled() -> bool:
    """The running robot's state reads ENABLED (shared memory read as a file: no import, no attach). Review I8,
    2026-11-21: the import checks start ~25 heavy interpreters, and heavy jobs on this Pi slowed RTI's ticks while the
    arm held itself."""
    path = Path("/dev/shm") / (os.environ.get("RCB_SHM_PREFIX", "") + "robot_state")
    try:
        raw = path.read_bytes()
        n = int.from_bytes(raw[:4], "little")
        return raw[4:4 + n].decode("utf-8", "replace").strip() == "ENABLED"
    except (OSError, ValueError):
        return False


def main() -> int:
    if arm_enabled() and "--force" not in sys.argv[1:]:
        print("The arm is ENABLED: this check starts ~25 heavy imports, which can slow the real-time loop while the arm "
              "holds itself. Disable the arm first (or run with --force).")
        return 2
    print(f"RCB Control Runtime system check\nrepo: {ROOT}\n")
    check_python()
    check_repo_layout()
    check_disk()
    check_platform()
    check_can()
    check_devices()
    check_rt_permissions(on_robot_pi())
    check_imports()

    required_failures = 0
    warnings = 0
    for check in checks:
        status = "OK" if check.ok else ("FAIL" if check.required else "WARN")
        print(f"[{status:4}] {check.name}: {check.detail}")
        if check.required and not check.ok:
            required_failures += 1
        elif not check.ok:
            warnings += 1

    print()
    print(summary_line(required_failures, warnings))
    return 1 if required_failures else 0


if __name__ == "__main__":
    reexec_in_venv()
    raise SystemExit(main())
