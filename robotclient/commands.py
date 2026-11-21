"""Command id generation. Command names match communication/protocol.py::COMMAND_RULES."""

import random
import string

_ALPHABET = string.ascii_letters + string.digits  # Base-62
# The client's own generator, seeded from the system (review P6, 2026-11-20): the module-level `random` is the
# user's — a script that called random.seed(n) between two queued moves gave both the same id, and the second
# reply completed the first. Not for security (an id only names a command), so not `secrets`.
_rng = random.Random()  # nosec B311


def generate_id(length=6):
    """6-char Base-62 random id (62^6 ≈ 56 billion — collision-free in practice).

    Mirrors utility/RTI_utils.generate_short_id so client- and server-generated ids
    share one format. Drawn from the client's own generator, never the global one.
    """
    return "".join(_rng.choices(_ALPHABET, k=length))
