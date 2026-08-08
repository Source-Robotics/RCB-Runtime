"""Client-side constants."""

DEFAULT_PORT = 29000          # command_executor TCP server
CONNECT_TIMEOUT = 5.0         # seconds
IMMEDIATE_TIMEOUT = 5.0       # seconds to wait for an immediate command reply
ACK_TIMEOUT = 5.0             # seconds to wait for a queued-command "queued" ack
DEFAULT_MOVE_TIMEOUT = 120.0  # seconds to wait for a single motion to complete (sync mode)

# Status-event waits (single TCP socket, Option B)
WAIT_STATE_TIMEOUT = 10.0           # wait_enabled / wait_mode default
WAIT_HOMED_TIMEOUT = 180.0          # homing can take a while
