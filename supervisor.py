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
import sys
import time
import queue
import logging
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
setup_logging(str(SUPERVISOR_LOG), log_to_console=DEBUG, critical_log_file=str(CRITICAL_LOG))


def handle_exception(exc_type, exc_value, exc_traceback):
    """Write any uncaught exception to the log before the process dies."""
    logging.critical("Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback))


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
SCRIPTS = {
    "RTI":        f"python3 {_ROOT}/RTI.py",
    "GUI":        f"python3 {_ROOT}/nicegui_test.py",
    "HIGH_LEVEL": f"python3 {_ROOT}/command_executor.py",
    # the self-collision worker (utility/self_collision.py): the measured pose against the convex pieces on another
    # core, ~100 Hz, into shared memory for RTI's monitor and jog (TODO/self_collision_plan.md, 2026-10-06)
    "SELF_COLLISION": f"python3 {_ROOT}/self_collision_worker.py",
} if RUN_ALL_SCRIPTS else {}

# ==========================================================================
# Hardware init
# ==========================================================================
disp = init_display()[0]     # UI builds its own frames; we only need the device
led1 = GpioLED(LED1_PIN)
led2 = GpioLED(LED2_PIN)

try:
    ser = serial.Serial(UART_PORT, BAUD_RATE, timeout=0.5)
except (serial.SerialException, OSError) as exc:
    # Degrade gracefully: the OLED + buttons still work, only PCB comms are lost.
    logging.error(f"Could not open UART {UART_PORT}: {exc}. PCB comms disabled.")
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
            logging.exception("Could not set OLED contrast")


def update_display():
    """Render the current UI screen in a loop (the UI decides what to draw)."""
    while True:
        try:
            _apply_contrast()
            screen = app.render()
            disp.image(screen.img)
            disp.show()
        except Exception:
            logging.exception("Display update failed")
        time.sleep(DISPLAY_REFRESH_TIME)


# ==========================================================================
# Script supervisor thread (sole owner of `state.processes`)
# ==========================================================================
def _power_off(command):
    """Cleanly stop managed scripts, then reboot / shut down the Pi."""
    logging.warning(f"{command} requested from OLED menu — stopping scripts first")
    try:
        stop_scripts(state.processes)
    except Exception:
        logging.exception("Error stopping scripts before power action")
    subprocess.run(["sudo", "-n", command], check=False)


def _drain_actions():
    """Run queued menu actions here (single thread that mutates `state.processes`)."""
    while True:
        try:
            cmd = state.action_queue.get_nowait()
        except queue.Empty:
            return
        try:
            if cmd[0] == "restart":
                restart_script(cmd[1], state.processes, SCRIPTS)
            elif cmd[0] == "stop_all":
                stop_scripts(state.processes)
            elif cmd[0] == "reboot":
                _power_off("reboot")
            elif cmd[0] == "shutdown":
                _power_off("poweroff")
        except Exception:
            logging.exception(f"Menu action failed: {cmd}")


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
                    logging.info("[CAN link] baseline — %s",
                                 format_can_link_stats(current))
                else:
                    message = describe_can_link_change(previous, current)
                    if message:
                        logging.warning("[CAN link] %s", message)
                previous = current
        except Exception:
            logging.exception("CAN link monitor pass failed")
        time.sleep(CAN_LINK_MONITOR_TIME)


def update_script_supervisor():
    """Start the managed scripts, then run menu actions + health-check in a loop."""
    kill_stale(SCRIPTS)  # clear orphans from a previous supervisor run before starting
    start_scripts(SCRIPTS=SCRIPTS, processes=state.processes)
    while True:
        try:
            _drain_actions()
            state.global_monitor_data = monitor_scripts(
                emergency_stop=False, processes=state.processes, SCRIPTS=SCRIPTS
            )
            # Every SCRIPT_MONITOR_TIME: DEBUG only. Crashes and restarts are logged by the process
            # manager when they happen (2026-10-01: this line + the blink-tick lines were ~100 MB/day).
            logging.debug(state.global_monitor_data)
        except Exception:
            logging.exception("Script supervisor pass failed")
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
        logging.info(f"GPIO state: {list(shared_data.gpio_states)}")


def shutdown():
    """Best-effort cleanup on exit."""
    try:
        stop_scripts(state.processes)  # don't leave RTI/GUI/HL orphaned holding their ports
    except Exception:
        logging.exception("Error stopping managed scripts on shutdown")
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
    logging.info("Supervisor shutdown complete")


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

    try:
        while True:
            start_time = time.perf_counter()

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

            # Pace the loop to LOOP_TIME (never sleep negative).
            elapsed = time.perf_counter() - start_time
            time.sleep(max(LOOP_TIME - elapsed, 0))
    except KeyboardInterrupt:
        logging.info("Supervisor interrupted by user")
    finally:
        shutdown()


if __name__ == "__main__":
    main()
