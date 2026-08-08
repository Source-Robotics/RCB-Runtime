"""Command id generation. Command names match communication/protocol.py::COMMAND_RULES."""

import random
import string

_ALPHABET = string.ascii_letters + string.digits  # Base-62


def generate_id(length=6):
    """6-char Base-62 random id (62^6 ≈ 56 billion — collision-free in practice).

    Mirrors utility/RTI_utils.generate_short_id so client- and server-generated ids
    share one format.
    """
    return "".join(random.choices(_ALPHABET, k=length))
