#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor.py
Author: Petar Crnjak
Date: 2024-06-11
Version: 3.0
Description:
    Control-box supervisor. Owns the two front-panel buttons, the 2 LEDs and the
    128x64 OLED, talks to the mainboard PCB over UART, and starts/monitors the
    high-level scripts (RTI, GUI, command executor).

    This file is the THIN entry point: hardware init, the three threads, the
    heartbeat, and process-action handling. All OLED UI lives in the
    `supervisor/ui/` package and shared runtime in `supervisor/state.py`:

        supervisor/state.py        - ui{}, settings{}, processes{}, action_queue, snapshot
        supervisor/data_sources.py - read-only getters for the screens
        supervisor/ui/theme.py     - font (DejaVu Sans 11) + geometry
        supervisor/ui/render.py    - Screen drawing helpers
        supervisor/ui/pages.py     - Home dashboard + Info-page registry
        supervisor/ui/settings.py  - Settings registry (numbers/toggles/actions)
        supervisor/ui/app.py       - UI state machine (HOME/INFO/SETTINGS/EDIT/CONFIRM)

    UI model — one rule: TAP = move, HOLD = select, HOLD B1 = back.
    To add an Info page: decorate a function in ui/pages.py.
    To add a setting:    one constructor call in ui/settings.py.

    Threads:
        - main loop (this file)    : buttons + UART + LED heartbeat, ~50 Hz
        - update_display           : renders the current OLED screen
        - update_script_supervisor : starts/watches scripts, runs menu actions

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import os
# BEFORE anything imports data.shared_data (review I3, 2026-11-20): the supervisor is the service's FIRST process — when
# it starts, nothing of the service runs (systemd stopped the whole unit), so it may recreate a segment an update
# resized and publish the layout. Without this it attached the old run's segments as a non-owner, refused the new
# layout, and never started RTI (the service would restart for ever). Read and removed at the import: the scripts it
# starts do not inherit it (RTI marks itself again).
os.environ["RCB_SHM_OWNER"] = "1"
import sys
import time
import queue
import logging
logger = logging.getLogger("sup")  # docs/logging.md "Logger names"
log_can_link = logging.getLogger("sup.can_link")  # docs/logging.md "Logger names"
log_gpio = logging.getLogger("sup.gpio")  # docs/logging.md "Logger names"
log_pcb = logging.getLogger("sup.pcb")  # docs/logging.md "Logger names"
log_process = logging.getLogger("sup.process")  # docs/logging.md "Logger names"
import threading
import subprocess

import serial
from gpiozero import LED as GpioLED

from data import shared_data
shared_data.setup(joints=6)  # Must run before any shared_data array is touched

from data.shared_memory_index import LED1, LED2, BUTTON1, BUTTON2
from project_paths import SUPERVISOR_LOG, CRITICAL_LOG, PROJECT_ROOT
from utility.logger_setup import setup_logging
from hardware.display import init_display
from hardware.button_handler import ButtonHandler
from hardware.gpio_handler import LED1_PIN, LED2_PIN, BUTTON1_PIN, BUTTON2_PIN
from hardware.can_health import (
    read_can_link_stats, describe_can_link_change, format_can_link_stats,
)
from supervisor.process_manager import (
    start_scripts, stop_scripts, restart_script, monitor_scripts, kill_stale,
)
from utility.supervisor_utils import read_pcb_data, send_data

from supervisor import state
from supervisor import settings_store
from supervisor.ui import app
from supervisor.ui import splash
from utility.process_utils import setup_process

setup_process(core=0, rt=False)   # supervisor: own core (0), normal scheduling

# Restore user settings (contrast/blink) saved from the OLED menu.
settings_store.load()

# ==========================================================================
# Logging
# ==========================================================================
# DEBUG=True  -> also echo this file's logs to the terminal
# DEBUG=False -> log to file only. Under systemd the terminal IS the journal, which rsyslog copies into
#                /var/log/syslog: with True every line was written three times (2026-10-01: ~150 MB/day).
DEBUG = False
setup_logging(str(SUPERVISOR_LOG), log_to_console=DEBUG, critical_log_file=str(CRITICAL_LOG),
              critical_rotates=False)   # RTI rotates critical_errors.log; the supervisor only appends (review I6)


def handle_exception(exc_type, exc_value, exc_traceback):
    """Write any uncaught exception to the log before the process dies."""
    logger.critical("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))


sys.excepthook = handle_exception

# ==========================================================================
# Configuration
# ==========================================================================
LOOP_TIME = 0.02             # main loop period — 20 ms (~50 Hz) works great
DISPLAY_REFRESH_TIME = 0.08  # OLED redraw period — 80 ms
SCRIPT_MONITOR_TIME = 2.0    # how often managed scripts are health-checked
CAN_LINK_MONITOR_TIME = 1.0  # how often the kernel's CAN counters are sampled
CAN_LINK_INTERFACE = 'can0'

# UART to the mainboard PCB
UART_PORT = '/dev/ttyAMA0'   # Pi5: primary UART on GPIO14 (TX) / GPIO15 (RX)
BAUD_RATE = 256000

# Managed scripts launched and watched by this supervisor.
RUN_ALL_SCRIPTS = True
_ROOT = str(PROJECT_ROOT)
# Every script runs with THIS interpreter (2026-10-16, user: "one Python for the robot and the tests"): autorun.sh
# starts the supervisor with the repo's .venv/bin/python3, so the robot runs exactly the packages the tests run
# (requirements.txt). Until then the scripts were started as "python3 ...": the system Python with ~/.local, whose
# versions had drifted (numpy 2.4.6 vs the pinned 2.3.5). .venv's python3 is the same binary, real-time capability
# included.
_PY = sys.executable or "python3"
SCRIPTS = {
    "RTI":        f"{_PY} {_ROOT}/RTI.py",
    "GUI":        f"{_PY} {_ROOT}/nicegui_test.py",
    "HIGH_LEVEL": f"{_PY} {_ROOT}/command_executor.py",
    # the self-collision worker (utility/self_collision.py): the measured pose against the convex pieces on another
    # core, ~100 Hz, into shared memory for RTI's monitor and jog (TODO/self_collision_plan.md, 2026-10-06)
    "SELF_COLLISION": f"{_PY} {_ROOT}/self_collision_worker.py",
    # the recorder: incident dumps and captures from RTI's shared-memory trace, in its own process so they never
    # cost RTI's loop anything (utility/trace_ring.py, utility/trace_recorder.py; 2026-10-16)
    "RECORDER": f"{_PY} {_ROOT}/recorder.py",
} if RUN_ALL_SCRIPTS else {}

# ==========================================================================
# Hardware init
# ==========================================================================
class _NoDisplay:
    """The OLED's stand-in when it does not open (2026-11-18, review D2): every call does nothing. The supervisor
    opened the display at import with no guard, so a dead display or cable started nothing of the robot."""

    def __getattr__(self, name):
        return lambda *args, **kwargs: None


try:
    disp = init_display()[0]     # UI builds its own frames; we only need the device
except Exception:
    logger.exception("the OLED display did not open — the robot starts without it (the control box menu is dark)")
    disp = _NoDisplay()
led1 = GpioLED(LED1_PIN)
led2 = GpioLED(LED2_PIN)

try:
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=0.5)
except (serial.SerialException, OSError) as exc:
    # Degrade gracefully: the OLED + buttons still work, only PCB comms are lost.
    log_pcb.error(f"Could not open UART {UART_PORT}: {exc}. PCB comms disabled.")
    ser = None

led_state = False
last_led_toggle = time.perf_counter()

# Buttons route straight into the UI state machine (TAP=move, HOLD=select).
button1 = ButtonHandler(BUTTON1_PIN, app.on_b1_short, app.on_b1_long)
button2 = ButtonHandler(BUTTON2_PIN, app.on_b2_short, app.on_b2_long)


# ==========================================================================
# Display thread
# ==========================================================================
def _apply_contrast():
    """Push contrast to the OLED only when it changed (keeps all disp.* on one thread)."""
    if state.settings["contrast"] != state.ui["_applied_contrast"]:
        try:
            disp.contrast(int(state.settings["contrast"]))
            state.ui["_applied_contrast"] = state.settings["contrast"]
        except Exception:
            logger.exception("Could not set OLED contrast")


def update_display():
    """Render the current UI screen in a loop (the UI decides what to draw)."""
    while True:
        try:
            _apply_contrast()
            screen = app.render()
            disp.image(screen.img)
            disp.show()
        except Exception:
            logger.exception("Display update failed")
        time.sleep(DISPLAY_REFRESH_TIME)


# ==========================================================================
# Script supervisor thread (sole owner of `state.processes`)
# ==========================================================================
def _power_off(command):
    """Cleanly stop managed scripts, then reboot / shut down the Pi."""
    logger.warning(f"{command} requested from OLED menu — stopping scripts first")
    try:
        stop_scripts(state.processes)
    except Exception:
        log_process.exception("Error stopping scripts before power action")
    subprocess.run(["sudo", "-n", command], check=False)


def _robot_enabled() -> bool:
    """The arm is ENABLED (shared memory); False when it cannot be read."""
    try:
        rs = shared_data.robot_state
        return rs is not None and rs.get() == "ENABLED"
    except Exception:
        return False


def _drain_actions():
    """Run queued menu actions here (single thread that mutates `state.processes`)."""
    while True:
        try:
            cmd = state.action_queue.get_nowait()
        except queue.Empty:
            return
        try:
            if cmd[0] == "restart":
                if cmd[1] == "RTI" and _robot_enabled():
                    # 2026-11-18 (review G2): restarting RTI drops the drives' hold (0.1 s after it stops) — never with
                    # the arm enabled; disable it (or park / support it) first
                    logger.warning("Restart RTI refused from the menu: the arm is ENABLED — disable it first")
                    continue
                state.scripts_stopped = False          # a restart after "Stop all" brings the scripts back
                restart_script(cmd[1], state.processes, SCRIPTS)
            elif cmd[0] == "stop_all":
                state.scripts_stopped = True           # the monitor leaves them stopped (review I1)
                stop_scripts(state.processes)
            elif cmd[0] == "reboot":
                state.scripts_stopped = True
                _power_off("reboot")
            elif cmd[0] == "shutdown":
                state.scripts_stopped = True
                _power_off("poweroff")
        except Exception:
            logger.exception(f"Menu action failed: {cmd}")


def monitor_can_link():
    """Log the kernel's CAN counters whenever they move.

    Automatic bus-off recovery (CAN_BUS_OFF_RESTART_MS) is what makes this
    necessary: a bus-off now clears in ~100 ms, which is short of the 200 ms
    disconnect threshold, so RTI sees a transient stale-data warning and nothing
    else. Without this, a bus quietly flapping on a bad connector is
    indistinguishable from ordinary frame loss.

    Change-triggered rather than periodic — a healthy bus stays silent in the
    log, so anything that DOES appear here is a real event. Lives in the
    supervisor because it shells out to `ip`, which has no business anywhere
    near RTI's 4 ms tick.
    """
    previous = None
    while True:
        try:
            current = read_can_link_stats(CAN_LINK_INTERFACE)
            if current is None:
                previous = None          # interface gone; re-baseline on return
            else:
                if previous is None:
                    log_can_link.info("baseline — %s",
                                 format_can_link_stats(current))
                else:
                    message = describe_can_link_change(previous, current)
                    if message:
                        log_can_link.warning("%s", message)
                previous = current
        except Exception:
            log_can_link.exception("CAN link monitor pass failed")
        time.sleep(CAN_LINK_MONITOR_TIME)


def update_script_supervisor():
    """Start the managed scripts, then run menu actions + health-check in a loop."""
    kill_stale(SCRIPTS)  # clear orphans from a previous supervisor run before starting
    start_scripts(SCRIPTS=SCRIPTS, processes=state.processes)
    while True:
        try:
            _drain_actions()
            if state.scripts_stopped:
                # 2026-11-18 (review I1): "Stop all" / Reboot / Shutdown empty the process table, and the monitor read
                # an empty table as "emergency stop lifted" and started everything again in the same pass — Stop all
                # was "restart all, arm un-homed", and a reboot started a fresh RTI while the Pi shut down
                state.global_monitor_data = {name: {"status": "stopped (menu)"} for name in SCRIPTS}
                time.sleep(SCRIPT_MONITOR_TIME)
                continue
            state.global_monitor_data = monitor_scripts(
                emergency_stop=False, processes=state.processes, SCRIPTS=SCRIPTS
            )
            # Every SCRIPT_MONITOR_TIME: DEBUG only. Crashes and restarts are logged by the process
            # manager when they happen (2026-10-01: this line + the blink-tick lines were ~100 MB/day).
            log_process.debug(state.global_monitor_data)
        except Exception:
            log_process.exception("Script supervisor pass failed")
        time.sleep(SCRIPT_MONITOR_TIME)


# ==========================================================================
# Main-loop helpers
# ==========================================================================
def read_uart():
    """Pull one PCB frame (if any) into shared memory."""
    if ser is None:
        return
    pcb_data = read_pcb_data(ser, expected_length=4)
    if pcb_data is not None:
        shared_data.control_board_data[:] = pcb_data


_last_gpio_logged = None      # GPIO state last written to the log, LEDs excluded (on_blink_tick)


def on_blink_tick():
    """Once per blink period: toggle LEDs, send PCB heartbeat, publish GPIO state."""
    global led_state

    led_state = not led_state

    if ser is not None:
        send_data(ser, "heartbeat\n\r")

    shared_data.gpio_states[LED1] = led_state
    shared_data.gpio_states[LED2] = not led_state
    # Buttons are published from the 50 Hz main loop now, not here — see the comment there.

    led1.value = int(shared_data.gpio_states[LED1])
    led2.value = int(shared_data.gpio_states[LED2])

    # Only when something other than the two blinking LEDs changed (inputs, E-stop, buttons, outputs).
    global _last_gpio_logged
    now = tuple(v for i, v in enumerate(shared_data.gpio_states) if i not in (LED1, LED2))
    if now != _last_gpio_logged:
        _last_gpio_logged = now
        log_gpio.info(f"GPIO state: {list(shared_data.gpio_states)}")


def shutdown():
    """Best-effort cleanup on exit."""
    try:
        stop_scripts(state.processes)  # don't leave RTI/GUI/HL orphaned holding their ports
    except Exception:
        log_process.exception("Error stopping managed scripts on shutdown")
    try:
        led1.off()
        led2.off()
    except Exception:
        pass
    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass
    logger.info("Supervisor shutdown complete")


# ==========================================================================
# Entry point
# ==========================================================================
def main():
    global last_led_toggle

    os.chdir(str(PROJECT_ROOT))

    splash.show(disp)   # boot logo, then the display thread takes over with Home

    threading.Thread(target=update_display, daemon=True).start()
    threading.Thread(target=update_script_supervisor, daemon=True).start()
    threading.Thread(target=monitor_can_link, daemon=True).start()

    last_error = {"text": None, "at": 0.0}
    try:
        while True:
            start_time = time.perf_counter()
            try:
                _main_pass()
            except Exception as exc:
                # 2026-11-18 (review I2): any error here (a serial error on the PCB's UART, a button read) ran
                # shutdown(), which stops RTI — the drives let go, homing lost, because of a UART fault. Logged (the
                # same error at most once a minute) and the loop goes on.
                text = f"{type(exc).__name__}: {exc}"
                if text != last_error["text"] or time.monotonic() - last_error["at"] > 60.0:
                    logger.exception("supervisor main loop pass failed — going on")
                    last_error.update(text=text, at=time.monotonic())
            # Pace the loop to LOOP_TIME (never sleep negative).
            elapsed = time.perf_counter() - start_time
            time.sleep(max(LOOP_TIME - elapsed, 0))
    except KeyboardInterrupt:
        logger.info("Supervisor interrupted by user")
    finally:
        shutdown()


def _main_pass():
    """One pass of the main loop: the buttons, the UART, the blink tick, the menu's idle check."""
    global last_led_toggle

    # Takeover-prompt latch brackets the button checks — arm BEFORE (so a press
    # that starts during the prompt is caught from its first edge), release AFTER
    # (so a short press, which fires on release INSIDE check_button, still sees it
    # set). See app._modal_latch for why checking "is the prompt up" is not enough.
    app.modal_latch_arm()
    button1.check_button()
    button2.check_button()
    app.modal_latch_release(button1._stable, button2._stable)

    # Publish the buttons EVERY loop (50 Hz), not in the blink tick. They used to
    # be written only in on_blink_tick(), i.e. once per `blink_time` — 1.0 s by
    # default and settable up to 5 s — so the executor, which reads them from shared
    # memory to confirm or refuse a forced control takeover, could miss a quick DENY
    # tap entirely and needed an ALLOW hold of up to two blink periods to see it at
    # all. Active-low on purpose (0 = pressed); every reader already assumes that.
    shared_data.gpio_states[BUTTON1] = 0 if button1.gz_button.is_pressed else 1
    shared_data.gpio_states[BUTTON2] = 0 if button2.gz_button.is_pressed else 1
    read_uart()

    if time.perf_counter() - last_led_toggle >= state.settings["blink_time"]:
        on_blink_tick()
        last_led_toggle = time.perf_counter()

    app.idle_check()   # auto-return to Home after inactivity


if __name__ == "__main__":
    main()
