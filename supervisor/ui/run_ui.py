#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/ui/run_ui.py
Description:
    Run ONLY the OLED UI for incremental testing — no script monitor, no UART.
    Stop supervisor first (it owns the I2C display).

        python3 -m supervisor.ui.run_ui          # drive the real OLED, buttons live
        python3 -m supervisor.ui.run_ui --png    # render current screen to a PNG
"""

import sys
import time

from data import shared_data
shared_data.setup(joints=6)

from supervisor import settings_store
settings_store.load()

from supervisor.ui import app


def _png():
    app.render().save_png("supervisor/ui/preview.png")
    print("saved supervisor/ui/preview.png")


def _oled():
    import threading
    from hardware.display import init_display
    from hardware.button_handler import ButtonHandler
    from hardware.gpio_handler import BUTTON1_PIN, BUTTON2_PIN

    disp = init_display()[0]

    # Display on its own thread so the slow I2C framebuffer push (~25-100 ms)
    # never throttles button polling — mirrors the real supervisor architecture.
    def display_loop():
        while True:
            screen = app.render()
            disp.image(screen.img)
            disp.show()
            time.sleep(0.05)

    threading.Thread(target=display_loop, daemon=True).start()

    b1 = ButtonHandler(BUTTON1_PIN, app.on_b1_short, app.on_b1_long)
    b2 = ButtonHandler(BUTTON2_PIN, app.on_b2_short, app.on_b2_long)
    print("UI running on OLED. Ctrl+C to exit.")
    while True:                       # tight ~10 ms button loop
        b1.check_button()
        b2.check_button()
        app.idle_check()
        time.sleep(0.01)


if __name__ == "__main__":
    if "--png" in sys.argv:
        _png()
    else:
        _oled()
