#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: display.py
Description: OLED display initialization and management for Raspberry Pi.
    Handles 128x64 I2C OLED display initialization and setup.
"""

import board
import busio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import logging
from project_paths import OLED_FONT

logger = logging.getLogger(__name__)


def init_display():
    """
    Initialize OLED display connected to the 128x64 I2C of Raspberry Pi.

    Returns:
        tuple: (disp, draw, image, font, x, top, bottom)
            - disp: adafruit_ssd1306 display object
            - draw: PIL ImageDraw object for drawing
            - image: PIL Image object
            - font: PIL ImageFont object (loaded from OLED_FONT path)
            - x: Starting x position for text
            - top: Top padding position
            - bottom: Bottom padding position
    """
    logger.info("INIT OLED")
    i2c = busio.I2C(board.SCL, board.SDA)
    disp = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c)
    disp.fill(0)
    disp.show()

    width = disp.width
    height = disp.height
    image = Image.new('1', (width, height))

    draw = ImageDraw.Draw(image)
    draw.rectangle((0, 0, width, height), outline=0, fill=0)

    padding = -2
    top = padding
    bottom = height - padding
    x = 0
    font = ImageFont.truetype(str(OLED_FONT), 15)

    return disp, draw, image, font, x, top, bottom
