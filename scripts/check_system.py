#!/usr/bin/env python3
"""Preflight checks for RCB Control Runtime installs.

The checker is intentionally diagnostic: it does not bring hardware up, start
services, or modify system state. Missing robot hardware is reported clearly so a
fresh clone can fail usefully instead of crashing later in RTI startup.
"""

from __future__ import annotations

import os
import platform
import resource
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# When users run ./scripts/check_system.py after install, prefer the project
# virtualenv so the check reflects the runtime environment, not user-site Python.
VENV_PYTHON = ROOT / ".venv" / "bin" / "python"
if VENV_PYTHON.exists() and Path(sys.executable).resolve() != VENV_PYTHON.resolve():
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
    add("CAN can0 up", "state UP" in output or "UP" in output.split(), output[:180])
    add("CAN bitrate 1000000", "bitrate 1000000" in output, output[:180], required=False)


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


def check_rt_permissions() -> None:
    soft, hard = resource.getrlimit(resource.RLIMIT_RTPRIO)
    probe = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "import os; "
                "os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(1)); "
                "policy=os.sched_getscheduler(0); "
                "prio=os.sched_getparam(0).sched_priority; "
                "print(f'policy={policy} prio={prio}')"
            ),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if probe.returncode == 0:
        detail = (
            f"probe ok ({probe.stdout.strip()}); "
            f"euid={os.geteuid()} RLIMIT_RTPRIO soft={soft} hard={hard}"
        )
        add("RT priority permission", True, detail, required=False)
    else:
        stderr = " ".join(probe.stderr.split())
        detail = (
            f"probe failed: {stderr or f'exit code {probe.returncode}'}; "
            f"euid={os.geteuid()} RLIMIT_RTPRIO soft={soft} hard={hard}"
        )
        add("RT priority permission", False, detail, required=False)


def check_imports() -> None:
    modules = {
        "can": True,
        "zmq": True,
        "psutil": True,
        "numpy": True,
        "scipy": True,
        "nicegui": True,
        "serial": True,
        "PIL": True,
        "Spectral_BLDC": True,
        "roboticstoolbox": True,
        "spatialmath": True,
        "pinocchio": False,
        # Added 2026-08-08. All three were missing here, so this check passed installs that
        # could not actually run the robot — including the one that shipped without `pinokin`
        # pinned at all.
        #   ruckig  — RTI mode's POSITION RATE LIMITER since 2026-08-02. Without it RTI
        #             silently falls back to a legacy cascade that does not converge, so this
        #             is a safety dependency, not an optional profile.
        #   pinokin — SE(3) helpers used by motion/cartesian_paths.py; absent means no
        #             Cartesian motion planning.
        #   toppra  — the only blend-capable time parameterizer (zone / tolerance blending).
        "ruckig": True,
        "pinokin": True,
        "toppra": True,
        "lgpio": True,
        "gpiozero": True,
        "adafruit_ssd1306": False,
        "board": False,
    }
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


def main() -> int:
    print(f"RCB Control Runtime system check\nrepo: {ROOT}\n")
    check_python()
    check_repo_layout()
    check_disk()
    check_platform()
    check_can()
    check_devices()
    check_rt_permissions()
    check_imports()

    required_failures = 0
    for check in checks:
        status = "OK" if check.ok else ("FAIL" if check.required else "WARN")
        print(f"[{status:4}] {check.name}: {check.detail}")
        if check.required and not check.ok:
            required_failures += 1

    print()
    if required_failures:
        print(f"System check finished with {required_failures} required failure(s).")
        return 1
    print("System check finished with no required failures.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
