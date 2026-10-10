"""Block programming for the arm (Blockly) — TODO/blockly_plan.md.

The editor page (web/) runs in a browser; the Pi turns its block program into a `robotclient` script (codegen.py)
and serves both (server.py). Nothing here touches shared memory or the RT loop: the only door into the robot is
`robotclient`, which is what lets a different GUI mount this folder unchanged.

Kept free of imports on purpose: a generated program imports `blockly.runtime`, and must not pull in the web
server with it.
"""
