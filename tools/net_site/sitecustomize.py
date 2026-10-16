"""Run first in every net tools/run_nets.py starts (2026-10-15): the net's temporary files go to the runner's folder for
it (temp_data/nets/tmp/<net>), not /tmp on the SD card.

Why not TMPDIR: this Pi's python3 carries a file capability (cap_sys_nice, for RTI's real-time priority), so the C
library starts every Python in secure mode and drops TMPDIR (with LD_* and a few others) from its environment — a
TMPDIR set by the runner never reaches the net. RCB_TMPDIR does; this sets tempfile.tempdir from it. Then it runs the
sitecustomize it shadows (the system's, an empty file on this Pi), so nothing else changes.
"""
import os
import runpy
import sys

_tmp = os.environ.get("RCB_TMPDIR")
if _tmp and os.path.isdir(_tmp):
    import tempfile
    tempfile.tempdir = _tmp

_here = os.path.dirname(os.path.abspath(__file__))
for _entry in sys.path:
    _dir = os.path.abspath(_entry or os.curdir)
    _file = os.path.join(_dir, "sitecustomize.py")
    if _dir != _here and os.path.isfile(_file):
        runpy.run_path(_file, run_name="sitecustomize")
        break
