"""Two checks of the drives at RTI's start (2026-11-17, user; TODO/driver_110_rtb15_plan.md "The 110 re-check").

- Gains: after the start-up parameter read (utility/drive_params.py) every drive should hold the gains and limits RTI
  sent it (parameters 8-17: Kpp, Kpv, Kiv, Kpiq, Kiiq, KP, KD, the velocity, current and voltage limits). One that
  holds another value lost a config frame (or was changed by someone else since): J<n>:GAINS_MISMATCH_WARNING, with
  both values in the log. A check only — the tick-50 / 150 / 300 re-send block stays exactly as it is.
- Product: the identity reply's product (1 STEPFOC, 2 Spectral Micro) against the robot / gripper XML's <driver_type>.
  A drive that is not the product the XML describes runs other gains in other units: J<n>:DRIVE_PRODUCT_MISMATCH, a
  hard error re-raised every tick — the arm cannot be enabled until the drive or the XML is fixed and RTI restarts.
Pure functions; RTI runs them once (RTI.py _drive_checks_at_start).
"""
from __future__ import annotations

import math

PRODUCT_IDS = {"stepfoc": 1, "spectral-bldc": 2}
PRODUCT_NAMES = {1: "STEPFOC", 2: "Spectral Micro"}
GAIN_NUMBERS = (8, 9, 10, 11, 12, 13, 14, 15, 16, 17)
GAIN_NAMES = {8: "Kpp", 9: "Kpv", 10: "Kiv", 11: "Kpiq", 12: "Kiiq", 13: "KP", 14: "KD", 15: "velocity limit",
              16: "current limit", 17: "voltage limit"}


def held_voltage_limit(sent_mv: float) -> float:
    """What a drive holds after RTI sent `sent_mv` as its voltage limit: 0 = the supply voltage, anything else is
    raised to at least 6000 mV (the firmware's floor)."""
    v = float(sent_mv)
    return 0.0 if v == 0 else max(v, 6000.0)


def sent_values(KPP, KPV, KIV, KPIQ, KIIQ, KP, KD, velocity_limit, Ilim, voltage_limit) -> dict:
    """{parameter number: the value the drive should hold} from what RTI sends one drive."""
    return {8: float(KPP), 9: float(KPV), 10: float(KIV), 11: float(KPIQ), 12: float(KIIQ), 13: float(KP),
            14: float(KD), 15: float(velocity_limit), 16: float(Ilim), 17: held_voltage_limit(voltage_limit)}


def gains_mismatches(sent: dict, held: dict) -> list:
    """[(number, sent, held)] for each parameter the drive holds with another value (float32 on the drive: compared
    to 1e-5 relative). A parameter not read (held None) is not judged."""
    out = []
    for num in GAIN_NUMBERS:
        h = held.get(num)
        if h is None or num not in sent:
            continue
        s = float(sent[num])
        if not math.isclose(float(h), s, rel_tol=1e-5, abs_tol=1e-9):
            out.append((num, s, float(h)))
    return out


def product_mismatch(driver_type, product_id) -> str | None:
    """Why a drive is not the product its XML declares, or None (also when either is unknown: no declaration, no
    identity reply)."""
    want = PRODUCT_IDS.get(str(driver_type or "").strip().lower())
    if want is None or product_id is None:
        return None
    if int(product_id) == want:
        return None
    return (f"the drive reports {PRODUCT_NAMES.get(int(product_id), f'product {product_id}')}, the XML's "
            f"<driver_type> says {driver_type}")
