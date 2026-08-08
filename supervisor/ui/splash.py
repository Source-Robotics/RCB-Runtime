#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/ui/splash.py
Description:
    Boot splash for the OLED — shows the company logo for a couple of seconds at
    startup before the Home screen takes over. Logo assets are 128x64 (already
    sized for this panel) and converted to 1-bit on the fly.

    Falls back to a text splash if the image can't be loaded, and never raises —
    a missing logo must not stop the supervisor from booting.
"""

import time
import logging

from PIL import Image, ImageDraw
from project_paths import IMAGES_DIR, OLED_UI_FONT
from supervisor.ui import theme

logger = logging.getLogger(__name__)

LOGO_FILE = IMAGES_DIR / "LOGO2.ppm"   # 128x64; swap to LOGO1.ppm for the other mark
SPLASH_SECONDS = 2.0


def _logo_image():
    """Load the logo as a 1-bit 128x64 frame, or None if unavailable."""
    try:
        img = Image.open(LOGO_FILE).convert("1")
        if img.size != (theme.WIDTH, theme.HEIGHT):
            img = img.resize((theme.WIDTH, theme.HEIGHT))
        return img
    except Exception:
        logger.warning(f"Could not load splash logo {LOGO_FILE}; using text splash")
        return None


def _text_image():
    """Fallback splash when the logo asset is missing."""
    from PIL import ImageFont
    img = Image.new("1", (theme.WIDTH, theme.HEIGHT))
    d = ImageDraw.Draw(img)
    try:
        big = ImageFont.truetype(str(OLED_UI_FONT), 16)
    except Exception:
        big = theme.FONT
    d.text((6, 18), "Source", font=big, fill=255)
    d.text((6, 36), "Robotics", font=big, fill=255)
    return img


def show(disp, seconds=SPLASH_SECONDS):
    """Push the splash to the OLED and hold it for `seconds`. Never raises."""
    try:
        img = _logo_image() or _text_image()
        disp.image(img)
        disp.show()
        time.sleep(seconds)
    except Exception:
        logging.exception("Splash failed (continuing boot)")
