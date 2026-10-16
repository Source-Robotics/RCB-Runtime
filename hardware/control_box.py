"""The control-box PCB: which hardware version is fitted, and what the runtime may rely on (2026-10-04).

The board does not report its version over UART yet (the four registers it sends are power-path numbers; see
TODO/control_box_firmware_wishlist.md), so the version is DECLARED in config/system.xml:

    <control_box><pcb_version>1</pcb_version></control_box>

and this table says what each version can do. The first consumer is the E-stop: PCB v1 has a layout fault on
E-stop channel 2 (GPIO6 always reads pressed), so the software check uses channel 1 only; v2 is expected to
fix it, and then both channels are read and cross-checked (hardware/gpio_handler.configure_estop).

Add a version here when a board changes anything the software can see. An unknown (newer) version gets the
latest known table with a warning — the software never silently assumes a capability a board may not have
beyond what the newest known board has.
"""
import logging

logger = logging.getLogger("io")  # docs/logging.md "Logger names"

PCB_CAPABILITIES = {
    1: {
        "estop_channels": 1,
        "version_report": False,
        "notes": "E-stop channel 2 (GPIO6) unusable — PCB v1 layout fault, it always reads pressed; "
                 "the software E-stop check reads channel 1 (GPIO5) only",
    },
    2: {
        "estop_channels": 2,
        "version_report": False,
        "notes": "dual-channel E-stop, both channels read and cross-checked (any channel pressed = pressed; a "
                 "sustained disagreement is a channel fault, treated as pressed)",
    },
}
LATEST_KNOWN_PCB = max(PCB_CAPABILITIES)


def capabilities(pcb_version) -> dict:
    """The capability table for a declared PCB version (a copy). Unknown versions: below 1 -> v1; above the
    newest known -> the newest known, with a warning (an upgrade of this table is due)."""
    try:
        v = int(pcb_version)
    except (TypeError, ValueError):
        logger.warning(f"control box pcb_version {pcb_version!r} is not a number — assuming PCB v1")
        v = 1
    if v in PCB_CAPABILITIES:
        return dict(PCB_CAPABILITIES[v])
    if v < 1:
        logger.warning(f"control box pcb_version {v} is below 1 — assuming PCB v1")
        return dict(PCB_CAPABILITIES[1])
    logger.warning(f"control box pcb_version {v} is newer than this software knows (latest known v{LATEST_KNOWN_PCB}) "
                   f"— using the v{LATEST_KNOWN_PCB} table; add the new board to hardware/control_box.py")
    return dict(PCB_CAPABILITIES[LATEST_KNOWN_PCB])


def describe(pcb_version) -> str:
    """One line for a log or a dashboard label."""
    c = capabilities(pcb_version)
    return (f"PCB v{pcb_version}: E-stop {c['estop_channels']} channel{'s' if c['estop_channels'] != 1 else ''}"
            f"{', version reported over UART' if c['version_report'] else ', version from config (not reported by the board)'}"
            f" — {c['notes']}")
