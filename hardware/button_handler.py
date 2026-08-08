#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: button_handler.py
Author: Petar Crnjak
Date: 2024-06-11
Version: 2.0
Description:
    Polled push-button handler with debounce and short/long press detection.

    Call check_button() every main-loop iteration. The handler samples the pin
    every call (fast), debounces state changes, and reports:
        - short press : fired on RELEASE, only if it was NOT a long press
        - long  press : fired ONCE the moment the hold passes LONG_PRESS_TIME
                        (while still held — snappy, no need to release first)

    Polarity note (Raspberry Pi 5):
        On Pi5 the gpiozero/lgpio backend can read inverted vs Pi4. The button
        MUST read False at rest and True when pressed. If it reads True at rest,
        flip `pull_up` (or pass pull_up=None + active_state) when constructing
        the handler. See the diagnostic in the repo notes.

License: GNU General Public License v3.0
Contact: petar@source-robotics.com
"""

import time
import logging
from gpiozero import Button as GpioButton

logger = logging.getLogger(__name__)
logger.info("button_handler.py")

# Press timings (seconds)
LONG_PRESS_TIME = 0.6   # hold >= this => long press
DEBOUNCE_TIME = 0.03    # a state must be steady this long before it counts


class ButtonHandler:
    """Per-button debounced short/long press state machine (active-low by default)."""

    def __init__(self, button_pin, callback_short=None, callback_long=None, pull_up=True):
        """
        Args:
            button_pin (int): BCM pin number of the button.
            callback_short (callable): called on a short press (on release).
            callback_long (callable): called once when a long press is recognised.
            pull_up (bool|None): gpiozero pull config. True = internal pull-up,
                pressed pulls LOW (active-low). Flip to False, or use None with an
                external resistor, if the pin reads inverted on Pi5.
        """
        self.callback_short = callback_short
        self.callback_long = callback_long
        self.gz_button = GpioButton(button_pin, pull_up=pull_up)

        self._raw = False           # last raw reading
        self._stable = False        # debounced "pressed" state
        self._last_change = time.perf_counter()
        self._press_start = 0.0
        self._long_fired = False    # guards long press to one shot per hold

    def check_button(self):
        """Sample the button once. Call this every main-loop iteration."""
        now = time.perf_counter()
        raw = self.gz_button.is_pressed

        # Debounce: a change must hold steady for DEBOUNCE_TIME before it counts.
        if raw != self._raw:
            self._raw = raw
            self._last_change = now
            return
        if (now - self._last_change) < DEBOUNCE_TIME:
            return

        if raw and not self._stable:                 # rising edge — pressed
            self._stable = True
            self._press_start = now
            self._long_fired = False

        elif raw and self._stable:                   # held — fire long once
            if not self._long_fired and (now - self._press_start) >= LONG_PRESS_TIME:
                self._long_fired = True
                if self.callback_long:
                    self.callback_long()

        elif not raw and self._stable:               # falling edge — released
            self._stable = False
            if not self._long_fired and self.callback_short:
                self.callback_short()
