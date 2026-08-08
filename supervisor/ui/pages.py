#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: supervisor/ui/pages.py
Description:
    Home dashboard + the Info-page carousel.

    To add an Info page: write a function taking a Screen and decorate it.
    It shows up in the Info carousel automatically — no other edits, and the
    order here is the carousel order, so reordering/removing is trivial:

        @info_page("Network")
        def _net(s):
            s.body(0, "IP", data.ip())     # body rows are 0-based, below the header

    Pages draw with s.body(i, left, right); the INFO header bar + page indicator
    are drawn by app.py, so pages only own their content.
"""

from supervisor import data_sources as data
from config.networking import GUI_PORT

SERVER_PORT = GUI_PORT

INFO_PAGES = []   # list of (title, render_fn) — carousel order


def info_page(title):
    def wrap(fn):
        INFO_PAGES.append((title, fn))
        return fn
    return wrap


# ==========================================================================
# Home dashboard (4 rows, read-only)
# ==========================================================================
def render_home(s):
    s.row(0, f"Mode {data.mode()}", data.state_())
    s.row(1, f"CAN {data.can()}", f"ES {data.estop()}")
    s.row(2, f"IP {data.ip()}")
    s.row(3, f"T {data.temp()}", f"RTI {data.rti_status()}")


# ==========================================================================
# Info carousel
# ==========================================================================
@info_page("Network")
def _page_network(s):
    s.body(0, "IP", data.ip())
    s.body(1, "Host", data.hostname())
    s.body(2, "Port", str(SERVER_PORT))


@info_page("System")
def _page_system(s):
    s.body(0, "Temp", data.temp())
    s.body(1, "Mem", data.mem_pct())
    s.body(2, "Load", data.load1())
    s.body(3, "Up", data.uptime())


@info_page("Scripts")
def _page_scripts(s):
    st = data.scripts()
    s.body(0, "RTI", _stat(st, "RTI"))
    s.body(1, "GUI", _stat(st, "GUI"))
    s.body(2, "HL", _stat(st, "HIGH_LEVEL"))


@info_page("Motors")
def _page_motors(s):
    p = data.motor_positions()
    p += [0] * (6 - len(p))          # pad if fewer joints
    s.body(0, f"J1 {p[0]}", f"J2 {p[1]}")
    s.body(1, f"J3 {p[2]}", f"J4 {p[3]}")
    s.body(2, f"J5 {p[4]}", f"J6 {p[5]}")


def _stat(snapshot, name):
    return (snapshot.get(name, {}) or {}).get("status", "—")
