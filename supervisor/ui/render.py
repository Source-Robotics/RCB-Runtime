#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/ui/render.py
Description:
    Screen — a 128x64 1-bit frame with high-level helpers so pages never deal
    with pixels. row(i, left, right) lays out one line (right field right-aligned);
    tabbar() draws the bottom tabs with a square inverted selection + horizontal
    scroll markers; title()/back_hint() are the Info/Settings chrome.
    A Screen renders to the OLED (`.img`) or to a scaled PNG (`.save_png`).
"""

from PIL import Image, ImageDraw
from supervisor.ui import theme


class Screen:
    def __init__(self):
        self.img = Image.new("1", (theme.WIDTH, theme.HEIGHT))
        self.d = ImageDraw.Draw(self.img)

    # --- primitives ---
    def clear(self):
        self.d.rectangle((0, 0, theme.WIDTH, theme.HEIGHT), fill=0)
        return self

    def _text(self, x, y, s, fill=255):
        self.d.text((x, y), s, font=theme.FONT, fill=fill)

    def _w(self, s):
        return self.d.textlength(s, font=theme.FONT)

    # --- high-level rows ---
    def row(self, i, left, right=None, selected=False):
        """Row i (0-based). `right` is right-aligned. `selected` = inverted bar."""
        y = i * theme.LINE_H
        fg = 0 if selected else 255
        if selected:
            self.d.rectangle((0, y, theme.WIDTH - 1, y + theme.LINE_H - 1), fill=255)
        self._text(2 if selected else 0, y, str(left), fill=fg)
        if right is not None:
            r = str(right)
            self._text(int(theme.WIDTH - self._w(r)), y, r, fill=fg)

    def title(self, text, indicator=None):
        """Top title row + optional right-aligned 'n/m' indicator."""
        self._text(0, 0, str(text))
        if indicator:
            self._text(int(theme.WIDTH - self._w(str(indicator))), 0, str(indicator))

    def back_hint(self, text="hold B1 = back"):
        self._text(0, (theme.ROWS_FULL - 1) * theme.LINE_H, text)

    def title_bar(self, left, right=None):
        """Inverted full-width header on row 0 (white bar, dark text) — marks the
        screen as a section header, visually consistent with the selected tab."""
        self.d.rectangle((0, 0, theme.WIDTH - 1, theme.LINE_H - 1), fill=255)
        self._text(2, 0, str(left), fill=0)
        if right is not None:
            r = str(right)
            self._text(int(theme.WIDTH - self._w(r) - 1), 0, r, fill=0)

    def body(self, i, left, right=None):
        """Body row i (0-based) — drawn just below the title bar (actual row i+1).
        Pages use this so they never need to know about the header offset."""
        self.row(i + 1, left, right)

    def list_view(self, items, selected, value_of):
        """Scrolling list below the title bar. `items` -> labels via .['label'];
        `value_of(item)` -> right-hand text; the selected row is inverted.
        Keeps the selected row in view; draws v markers when scrolled."""
        rows = theme.ROWS_FULL - 1          # rows available under the title bar
        n = len(items)
        # window start: keep selected centered-ish, clamped to [0, n-rows]
        start = min(max(0, selected - rows // 2), max(0, n - rows))
        for r in range(rows):
            i = start + r
            if i >= n:
                break
            self.row(r + 1, items[i]["label"], value_of(items[i]),
                     selected=(i == selected))
        # scroll affordances at the right edge of the title bar area
        if start > 0:
            self._text(theme.WIDTH - 7, 0, "^", fill=0)
        if start + rows < n:
            self._text(theme.WIDTH - 7, (theme.ROWS_FULL - 1) * theme.LINE_H, "v")

    def confirm_view(self, label):
        """Confirmation prompt for a destructive action (reboot / shutdown / stop)."""
        self.title_bar(f"{label}?")
        self._text(0, 2 * theme.LINE_H, "hold B2 = YES")
        self._text(0, 3 * theme.LINE_H, "B1 = cancel")

    def edit_view(self, label, value_text):
        """Centered '-  value  +' editor screen."""
        self.title_bar(label)
        mid = f"-   {value_text}   +"
        x = int((theme.WIDTH - self._w(mid)) / 2)
        self._text(x, 2 * theme.LINE_H, mid)
        self._text(0, (theme.ROWS_FULL - 1) * theme.LINE_H, "B2=save  B1=back")

    def tabbar(self, labels, selected):
        """Bottom tab bar. Selected = filled square box + dark text. Scrolls with
        '<' / '>' markers when the tabs don't all fit."""
        pad, gap = 3, 2
        y = theme.HEIGHT - theme.BAR_H
        widths = [self._w(l) + 2 * pad for l in labels]

        # Grow a visible window outward from the selected tab until it won't fit.
        start = end = selected
        used = widths[selected]
        while True:
            grew = False
            if start - 1 >= 0 and used + gap + widths[start - 1] <= theme.WIDTH:
                start -= 1
                used += gap + widths[start]
                grew = True
            if end + 1 < len(labels) and used + gap + widths[end + 1] <= theme.WIDTH:
                end += 1
                used += gap + widths[end]
                grew = True
            if not grew:
                break

        x = 0
        if start > 0:
            self._text(x, y + 1, "<")
            x += int(self._w("<")) + gap
        for i in range(start, end + 1):
            w = widths[i]
            if i == selected:
                self.d.rectangle((x, y, x + w, y + theme.BAR_H - 1), fill=255)
                self._text(x + pad, y + 1, labels[i], fill=0)
            else:
                self._text(x + pad, y + 1, labels[i])
            x += w + gap
        if end < len(labels) - 1:
            self._text(int(theme.WIDTH - self._w(">")), y + 1, ">")

    # --- output ---
    def save_png(self, path, scale=6):
        (self.img.convert("L")
             .resize((theme.WIDTH * scale, theme.HEIGHT * scale), Image.NEAREST)
             .save(path))
