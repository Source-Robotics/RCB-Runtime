#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/ui/theme.py
Description:
    OLED UI look: font + geometry. Change sizes here, nowhere else.
    DejaVu Sans @ 11 px → 5 rows full screen, 4 rows + tab bar on Home.
"""

from PIL import ImageFont
from project_paths import OLED_UI_FONT

WIDTH, HEIGHT = 128, 64
SIZE = 11
LINE_H = 12        # row pitch for SIZE-px text
BAR_H = 13         # bottom tab-bar height
ROWS_FULL = 5      # rows that fit full screen (Info/Settings)
ROWS_HOME = 4      # rows above the tab bar (Home)

FONT = ImageFont.truetype(str(OLED_UI_FONT), SIZE)
