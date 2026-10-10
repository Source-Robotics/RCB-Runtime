"""Client-side constants."""

DEFAULT_PORT = 29000          # command_executor TCP server
CONNECT_TIMEOUT = 5.0         # seconds
IMMEDIATE_TIMEOUT = 5.0       # seconds to wait for an immediate command reply
# A FORCED control takeover can block on the robot for its whole physical-confirmation
# window while someone walks over and presses the control-box button. Must stay comfortably
# ABOVE command_executor.FORCE_CONFIRM_TIMEOUT_S (20 s) — if the client gives up first, the
# robot may still grant control to a caller that is no longer listening.
FORCE_TAKE_TIMEOUT = 30.0
ACK_TIMEOUT = 5.0             # seconds to wait for a queued-command "queued" ack
DEFAULT_MOVE_TIMEOUT = 120.0  # seconds to wait for a single motion to complete (sync mode)

# Status-event waits (single TCP socket, Option B)
WAIT_STATE_TIMEOUT = 10.0           # wait_enabled / wait_mode default
WAIT_HOMED_TIMEOUT = 180.0          # homing can take a while
