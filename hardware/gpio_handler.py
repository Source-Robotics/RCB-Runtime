#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: gpio_handler.py
Author: Petar Crnjak
Company: Source Robotics
Date: 19.1.2026
Version: 1.0
Description:
    GPIO handling for robot inputs and outputs.
    Manages isolated inputs/outputs, E-stops, and general GPIO pins.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import logging
import os

import lgpio
from data.shared_memory_index import (
    ISOLATED_INPUT_1, ISOLATED_INPUT_2, ISOLATED_INPUT_3,
    ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3,
    ESTOP_2, ESTOP_1, GPIO_1, GPIO_2, GPIO_3, GPIO_4
)
from data import shared_data
from utility import sim_mode

logger = logging.getLogger(__name__)

# GPIO state constants for clarity
HIGH = 1
LOW = 0

_gpio_handle = None
_gpio_chip = None
_claimed_pins = set()

# The simulated robot (RCB_SIM=1, utility/sim_mode.py; 2026-10-13): NO real pin is read or written. A simulated run
# must not switch the control box's isolated outputs (real equipment may hang on them) nor depend on its real E-stop:
# until then RTI in simulation read the Pi's real inputs and drove its real outputs. The inputs live in shared memory
# only (gpio_states — set at start: E-stop released, inputs off); a tool changes them by writing the GPIO segment
# (sim/faults.py presses the E-stop that way); the outputs stay in shared memory.
_simulated = False

# ==========================================================================
# BCM pin numbers for Raspberry Pi-compatible 40-pin header
# ==========================================================================
ISOLATED_INPUT_1_PIN  = 19   # BOARD 35 -> BCM 19
ISOLATED_INPUT_2_PIN  = 13   # BOARD 33 -> BCM 13
ISOLATED_INPUT_3_PIN  = 7    # BOARD 26 -> BCM 7

ISOLATED_OUTPUT_1_PIN = 25   # BOARD 22 -> BCM 25
ISOLATED_OUTPUT_2_PIN = 24   # BOARD 18 -> BCM 24
ISOLATED_OUTPUT_3_PIN = 22   # BOARD 15 -> BCM 22

ESTOP_2_PIN = 6              # BOARD 31 -> BCM 6
ESTOP_1_PIN = 5              # BOARD 29 -> BCM 5

GPIO_1_PIN = 4               # BOARD 7  -> BCM 4
GPIO_2_PIN = 27              # BOARD 13 -> BCM 27
GPIO_3_PIN = 18              # BOARD 12 -> BCM 18
GPIO_4_PIN = 17              # BOARD 11 -> BCM 17

LED1_PIN    = 16             # BOARD 36 -> BCM 16
LED2_PIN    = 20             # BOARD 38 -> BCM 20
BUTTON2_PIN = 21             # BOARD 40 -> BCM 21
BUTTON1_PIN = 26             # BOARD 37 -> BCM 26


# ==========================================================================
# GPIO Initialization
# ==========================================================================

def _input_pins():
    return (
        ISOLATED_INPUT_1_PIN,
        ISOLATED_INPUT_2_PIN,
        ISOLATED_INPUT_3_PIN,
        ESTOP_2_PIN,
        ESTOP_1_PIN,
        GPIO_1_PIN,
        GPIO_2_PIN,
        GPIO_3_PIN,
        GPIO_4_PIN,
    )


def _output_pins():
    return (
        ISOLATED_OUTPUT_1_PIN,
        ISOLATED_OUTPUT_2_PIN,
        ISOLATED_OUTPUT_3_PIN,
    )


def _candidate_gpio_chips():
    configured_chip = os.getenv("RCB_GPIO_CHIP")
    if configured_chip is not None:
        try:
            yield int(configured_chip)
        except ValueError:
            logger.warning(f"Ignoring invalid RCB_GPIO_CHIP={configured_chip!r}")

    # Pi 5 usually exposes the 40-pin header on gpiochip4. Older boards commonly use gpiochip0.
    for chip in (4, 0, 1, 2, 3):
        yield chip


def _open_gpio_chip():
    global _gpio_handle, _gpio_chip

    if _gpio_handle is not None:
        return

    seen = set()
    errors = []
    for chip in _candidate_gpio_chips():
        if chip in seen:
            continue
        seen.add(chip)
        try:
            _gpio_handle = lgpio.gpiochip_open(chip)
            _gpio_chip = chip
            return
        except Exception as exc:
            errors.append(f"gpiochip{chip}: {exc}")

    raise RuntimeError("Unable to open a GPIO chip for RCB I/O: " + "; ".join(errors))


def init_gpio():
    """
    Init GPIOs (BCM pin numbering).
    """
    _open_gpio_chip()

    for pin in _input_pins():
        lgpio.gpio_claim_input(_gpio_handle, pin, lgpio.SET_PULL_DOWN)
        _claimed_pins.add(pin)

    # LED and button pins are owned by supervisor.py (via gpiozero) - not claimed here.

    for pin in _output_pins():
        lgpio.gpio_claim_output(_gpio_handle, pin, LOW)
        _claimed_pins.add(pin)


def initialize_gpio():
    """
    Initialize GPIO pins for robot I/O — or, in simulation, the simulated I/O (no real pin touched).
    """
    global _simulated
    _simulated = sim_mode.enabled()
    if _simulated:
        for idx in (ISOLATED_INPUT_1, ISOLATED_INPUT_2, ISOLATED_INPUT_3, GPIO_1, GPIO_2, GPIO_3, GPIO_4):
            shared_data.gpio_states[idx] = LOW
        shared_data.gpio_states[ESTOP_1] = HIGH          # active LOW: HIGH = released
        shared_data.gpio_states[ESTOP_2] = HIGH
        logger.info("[SIM] GPIO simulated: no real pin is read or written — inputs and outputs live in shared memory "
                    "(E-stop released, inputs off)")
        return
    init_gpio()
    logger.info(f"GPIO pins initialized on gpiochip{_gpio_chip}")


# ==========================================================================
# GPIO Input Reading
# ==========================================================================

# Debounce state tracking for inputs (smoothing filter)
_input_history = {
    ISOLATED_INPUT_1: [],
    ISOLATED_INPUT_2: [],
    ISOLATED_INPUT_3: [],
    ESTOP_1: [],
    ESTOP_2: [],
    GPIO_1: [],
    GPIO_2: [],
    GPIO_3: [],
    GPIO_4: []
}
_DEBOUNCE_SAMPLES = 5  # Require 5 consecutive stable reads to change state


def _apply_debounce(input_index, raw_value):
    """
    Apply debounce filter to raw GPIO reading.
    Requires multiple stable reads before accepting state change.

    Args:
        input_index (int): Input index
        raw_value (int): Raw GPIO reading (0 or 1)

    Returns:
        int: Debounced value (0 or 1)
    """
    # Keep sliding window of last N readings
    history = _input_history[input_index]

    # First ever reading for this pin: seed the whole window with it, so the very first tick
    # reports the REAL pin state.
    #
    # Without this, the fallback at the bottom returns shared_data.gpio_states[input_index] —
    # zero-initialised shared memory that was never a measurement. The E-stops are ACTIVE-LOW, so
    # that zero reads as PRESSED: every boot raised a false ESTOP for the ~4 ticks before the
    # window filled, and because ESTOP is a hard LATCHING error it then stuck around until the
    # user pressed Clear Errors. A ~16 ms unknown became a permanent error.
    #
    # You cannot debounce against history you do not have — the first real read IS the initial
    # state. This is not a safety loosening: if the E-stop genuinely is pressed at boot the first
    # read is 0 and it latches exactly as before.
    if not history:
        history.extend([raw_value] * (_DEBOUNCE_SAMPLES - 1))

    history.append(raw_value)
    if len(history) > _DEBOUNCE_SAMPLES:
        history.pop(0)

    # If all recent readings are the same, accept that value
    if len(history) == _DEBOUNCE_SAMPLES and all(h == history[0] for h in history):
        return history[0]

    # Otherwise, keep current state
    return shared_data.gpio_states[input_index]


def read_inputs_into_shared():
    """Read every input pin with debouncing into shared memory — the RT loop's per-tick call (2026-10-03).
    read_all_inputs() below adds the dict the GUI and scripts want; the loop never used it."""
    if _simulated:                       # simulation: the inputs are whatever shared memory holds
        return
    # Read isolated inputs with debounce filter
    raw_input_1 = lgpio.gpio_read(_gpio_handle, ISOLATED_INPUT_1_PIN)
    shared_data.gpio_states[ISOLATED_INPUT_1] = _apply_debounce(ISOLATED_INPUT_1, raw_input_1)

    raw_input_2 = lgpio.gpio_read(_gpio_handle, ISOLATED_INPUT_2_PIN)
    shared_data.gpio_states[ISOLATED_INPUT_2] = _apply_debounce(ISOLATED_INPUT_2, raw_input_2)

    raw_input_3 = lgpio.gpio_read(_gpio_handle, ISOLATED_INPUT_3_PIN)
    shared_data.gpio_states[ISOLATED_INPUT_3] = _apply_debounce(ISOLATED_INPUT_3, raw_input_3)

    # Read E-stop buttons with debounce filter
    raw_estop_1 = lgpio.gpio_read(_gpio_handle, ESTOP_1_PIN)
    shared_data.gpio_states[ESTOP_1] = _apply_debounce(ESTOP_1, raw_estop_1)

    raw_estop_2 = lgpio.gpio_read(_gpio_handle, ESTOP_2_PIN)
    shared_data.gpio_states[ESTOP_2] = _apply_debounce(ESTOP_2, raw_estop_2)

    # Read general GPIO pins with debounce filter (user-configurable inputs)
    raw_gpio_1 = lgpio.gpio_read(_gpio_handle, GPIO_1_PIN)
    shared_data.gpio_states[GPIO_1] = _apply_debounce(GPIO_1, raw_gpio_1)

    raw_gpio_2 = lgpio.gpio_read(_gpio_handle, GPIO_2_PIN)
    shared_data.gpio_states[GPIO_2] = _apply_debounce(GPIO_2, raw_gpio_2)

    raw_gpio_3 = lgpio.gpio_read(_gpio_handle, GPIO_3_PIN)
    shared_data.gpio_states[GPIO_3] = _apply_debounce(GPIO_3, raw_gpio_3)

    raw_gpio_4 = lgpio.gpio_read(_gpio_handle, GPIO_4_PIN)
    shared_data.gpio_states[GPIO_4] = _apply_debounce(GPIO_4, raw_gpio_4)


def read_all_inputs():
    """
    Read all GPIO inputs with debouncing and update shared memory.

    Returns:
        dict: Dictionary containing all input states
    """
    read_inputs_into_shared()
    return {
        'isolated_input_1': shared_data.gpio_states[ISOLATED_INPUT_1],
        'isolated_input_2': shared_data.gpio_states[ISOLATED_INPUT_2],
        'isolated_input_3': shared_data.gpio_states[ISOLATED_INPUT_3],
        'estop_1': shared_data.gpio_states[ESTOP_1],
        'estop_2': shared_data.gpio_states[ESTOP_2],
        'gpio_1': shared_data.gpio_states[GPIO_1],
        'gpio_2': shared_data.gpio_states[GPIO_2],
        'gpio_3': shared_data.gpio_states[GPIO_3],
        'gpio_4': shared_data.gpio_states[GPIO_4]
    }


# E-stop channel policy (2026-10-04, hardware/control_box.py): how many hardware channels the fitted PCB can be
# trusted on. PCB v1: channel 2 (GPIO6) always reads pressed (layout fault) -> channel 1 only, as it has always
# been. v2+: both channels, cross-checked — any channel pressed = pressed; the two disagreeing for longer than
# _estop_disagree_limit ticks is a CHANNEL FAULT (a broken wire or contact on one channel): treated as pressed and
# logged once per episode, so a single-channel failure can neither hide a press nor pass unnoticed.
_estop_channels = 1
_estop_disagree_limit = 50           # ticks (200 ms at 250 Hz): longer than contact bounce, shorter than a hand
_estop_disagree_count = 0
_estop_fault = False


def configure_estop(channels, disagree_ticks=50):
    """Called once at start-up with the PCB's capability (control_box.capabilities()['estop_channels'])."""
    global _estop_channels, _estop_disagree_limit, _estop_disagree_count, _estop_fault
    _estop_channels = 2 if int(channels) >= 2 else 1
    _estop_disagree_limit = max(1, int(disagree_ticks))
    _estop_disagree_count = 0
    _estop_fault = False
    logger.info(f"[E-stop] hardware channels in use: {_estop_channels}"
                + (f" (cross-checked, fault after {_estop_disagree_limit} ticks of disagreement)" if _estop_channels == 2
                   else " (channel 2 not trusted on this PCB)"))


def estop_channel_fault():
    """True while the two channels disagree past the limit (dual-channel PCBs only)."""
    return _estop_fault


def check_estop_active():
    """
    Check if any E-stop is active — hardware GPIO or software flag.

    Returns:
        bool: True if any E-stop is active, False otherwise
    """
    global _estop_disagree_count, _estop_fault
    # Hardware e-stop: active LOW (pressed = 0). Debounced values from shared_data.
    # Channel 2 only on a PCB that can be trusted on it (configure_estop).
    hw_estop = shared_data.gpio_states[ESTOP_1] == 0
    if _estop_channels == 2:
        ch2 = shared_data.gpio_states[ESTOP_2] == 0
        if ch2 != hw_estop:
            _estop_disagree_count += 1
            if _estop_disagree_count > _estop_disagree_limit and not _estop_fault:
                _estop_fault = True
                logger.error(f"[E-stop] CHANNEL FAULT: channel {1 if hw_estop else 2} reads pressed, the other released, "
                             f"for {_estop_disagree_count} ticks — a broken wire or contact; treated as PRESSED until both agree")
            hw_estop = True
        else:
            if _estop_fault:
                logger.warning("[E-stop] channel fault cleared — both channels agree again")
            _estop_disagree_count = 0
            _estop_fault = False

    # Software e-stop: GUI (or any client) sets shared_data.rti['software_estop'][0] = 1.
    sw_estop = bool(shared_data.rti["software_estop"][0])

    return hw_estop or sw_estop


# ==========================================================================
# GPIO Output Control
# ==========================================================================

# The level last written to each output pin (None = not yet written). An output holds its level, so the
# per-tick write only has to happen when shared memory asks for a DIFFERENT one (2026-10-03: three ioctls per
# tick bought nothing). Every other writer of these pins keeps this in step.
_OUTPUTS = ((ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_1_PIN), (ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_2_PIN),
            (ISOLATED_OUTPUT_3, ISOLATED_OUTPUT_3_PIN))
_last_written = {ISOLATED_OUTPUT_1: None, ISOLATED_OUTPUT_2: None, ISOLATED_OUTPUT_3: None}


def write_all_outputs():
    """
    Write the GPIO outputs from shared memory to the physical pins — each one only when its level changed.
    """
    if _simulated:                       # simulation: the level stays in shared memory, no pin is driven
        return
    for idx, pin in _OUTPUTS:
        value = int(shared_data.gpio_states[idx])
        if value != _last_written[idx]:
            lgpio.gpio_write(_gpio_handle, pin, value)
            _last_written[idx] = value


def set_output(output_index, value):
    """
    Set a specific output pin value.

    Args:
        output_index (int): Output index (ISOLATED_OUTPUT_1, etc.)
        value (int): 0 or 1
    """
    shared_data.gpio_states[output_index] = value

    # Map index to pin
    pin_map = {
        ISOLATED_OUTPUT_1: ISOLATED_OUTPUT_1_PIN,
        ISOLATED_OUTPUT_2: ISOLATED_OUTPUT_2_PIN,
        ISOLATED_OUTPUT_3: ISOLATED_OUTPUT_3_PIN
    }

    if output_index in pin_map and not _simulated:
        lgpio.gpio_write(_gpio_handle, pin_map[output_index], int(value))
        _last_written[output_index] = int(value)


def set_all_outputs_low():
    """
    Set all outputs to LOW (0).
    """
    for output_idx in [ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3]:
        set_output(output_idx, 0)
    logger.info("All outputs set to LOW")


def set_all_outputs_high():
    """
    Set all outputs to HIGH (1).
    """
    for output_idx in [ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, ISOLATED_OUTPUT_3]:
        set_output(output_idx, 1)
    logger.info("All outputs set to HIGH")


# ==========================================================================
# Convenience Output Control Functions
# ==========================================================================

def set_output_high(output_index):
    """
    Set a specific output HIGH (1).

    Args:
        output_index (int): Output index (ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, or ISOLATED_OUTPUT_3)
    """
    set_output(output_index, 1)


def set_output_low(output_index):
    """
    Set a specific output LOW (0).

    Args:
        output_index (int): Output index (ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, or ISOLATED_OUTPUT_3)
    """
    set_output(output_index, 0)


def toggle_output(output_index):
    """
    Toggle a specific output between HIGH and LOW.

    Args:
        output_index (int): Output index (ISOLATED_OUTPUT_1, ISOLATED_OUTPUT_2, or ISOLATED_OUTPUT_3)
    """
    current_state = shared_data.gpio_states[output_index]
    set_output(output_index, 1 - current_state)


# Named convenience functions for each isolated output
def set_output_1_high():
    """Set ISOLATED_OUTPUT_1 to HIGH."""
    set_output_high(ISOLATED_OUTPUT_1)


def set_output_1_low():
    """Set ISOLATED_OUTPUT_1 to LOW."""
    set_output_low(ISOLATED_OUTPUT_1)


def set_output_2_high():
    """Set ISOLATED_OUTPUT_2 to HIGH."""
    set_output_high(ISOLATED_OUTPUT_2)


def set_output_2_low():
    """Set ISOLATED_OUTPUT_2 to LOW."""
    set_output_low(ISOLATED_OUTPUT_2)


def set_output_3_high():
    """Set ISOLATED_OUTPUT_3 to HIGH."""
    set_output_high(ISOLATED_OUTPUT_3)


def set_output_3_low():
    """Set ISOLATED_OUTPUT_3 to LOW."""
    set_output_low(ISOLATED_OUTPUT_3)


# ==========================================================================
# Combined GPIO Update (for main loop)
# ==========================================================================

def update_gpio():
    """
    Update all GPIO - read inputs and write outputs.
    Call this function in the main control loop.
    """
    read_all_inputs()
    write_all_outputs()


# ==========================================================================
# GPIO State Inspection
# ==========================================================================

def get_gpio_state_summary():
    """
    Get a summary of all GPIO states.

    Returns:
        dict: Dictionary containing all GPIO states
    """
    return {
        'inputs': {
            'isolated_input_1': shared_data.gpio_states[ISOLATED_INPUT_1],
            'isolated_input_2': shared_data.gpio_states[ISOLATED_INPUT_2],
            'isolated_input_3': shared_data.gpio_states[ISOLATED_INPUT_3],
            'estop_1': shared_data.gpio_states[ESTOP_1],
            'estop_2': shared_data.gpio_states[ESTOP_2]
        },
        'outputs': {
            'isolated_output_1': shared_data.gpio_states[ISOLATED_OUTPUT_1],
            'isolated_output_2': shared_data.gpio_states[ISOLATED_OUTPUT_2],
            'isolated_output_3': shared_data.gpio_states[ISOLATED_OUTPUT_3]
        }
    }


def print_gpio_states():
    """
    Print formatted GPIO state summary.
    """
    states = get_gpio_state_summary()

    print("\n" + "="*50)
    print("GPIO STATE SUMMARY")
    print("="*50)

    print("\nInputs:")
    for name, value in states['inputs'].items():
        status = "HIGH" if value else "LOW"
        print(f"  {name:20s}: {status}")

    print("\nOutputs:")
    for name, value in states['outputs'].items():
        status = "HIGH" if value else "LOW"
        print(f"  {name:20s}: {status}")

    print("="*50 + "\n")


# ==========================================================================
# Safety Functions
# ==========================================================================

def emergency_stop_all_outputs():
    """
    Emergency function to set all outputs LOW immediately.
    Call this when E-stop is triggered or error occurs.
    """
    if not _simulated:
        lgpio.gpio_write(_gpio_handle, ISOLATED_OUTPUT_1_PIN, 0)
        lgpio.gpio_write(_gpio_handle, ISOLATED_OUTPUT_2_PIN, 0)
        lgpio.gpio_write(_gpio_handle, ISOLATED_OUTPUT_3_PIN, 0)
    for idx in _last_written:
        _last_written[idx] = 0

    shared_data.gpio_states[ISOLATED_OUTPUT_1] = 0
    shared_data.gpio_states[ISOLATED_OUTPUT_2] = 0
    shared_data.gpio_states[ISOLATED_OUTPUT_3] = 0

    logger.warning("Emergency stop - all outputs set LOW")


def gpio_cleanup():
    """
    Clean up GPIO resources on shutdown.
    """
    global _gpio_handle, _gpio_chip

    try:
        if _gpio_handle is None:
            return

        set_all_outputs_low()
        for pin in list(_claimed_pins):
            try:
                lgpio.gpio_free(_gpio_handle, pin)
            except Exception as exc:
                logger.debug(f"GPIO free skipped for pin {pin}: {exc}")
        _claimed_pins.clear()
        lgpio.gpiochip_close(_gpio_handle)
        _gpio_handle = None
        _gpio_chip = None
        logger.info("GPIO cleanup complete")
    except Exception as e:
        logger.warning(f"GPIO cleanup error: {e}")


if __name__ == "__main__":
    """Test GPIO handler"""
    print("GPIO Handler Test")
    print("-" * 50)

    try:
        # Initialize GPIO
        initialize_gpio()

        # Read inputs
        print("\nReading inputs...")
        inputs = read_all_inputs()
        print("Input states:")
        for name, value in inputs.items():
            print(f"  {name}: {value}")

        # Check E-stop
        estop_active = check_estop_active()
        print(f"\nE-stop active: {estop_active}")

        # Print full summary
        print_gpio_states()

        # Test outputs (be careful with real hardware!)
        print("\nTest output control (press Ctrl+C to skip)...")
        import time
        try:
            print("Setting outputs HIGH for 1 second...")
            set_all_outputs_high()
            time.sleep(1)

            print("Setting outputs LOW...")
            set_all_outputs_low()
            time.sleep(1)

        except KeyboardInterrupt:
            print("\nOutput test skipped")

        # Cleanup
        gpio_cleanup()

    except Exception as e:
        print(f"\nError during GPIO test: {e}")
        print("Make sure:")
        print("  1. You're running on a Raspberry Pi")
        print("  2. You have GPIO permissions")
        print("  3. shared_data module is properly initialized")
