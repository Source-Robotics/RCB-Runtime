"""Every step of the first-run check, in the order it runs (2026-10-18). Stage 3 is the powered test moves; BY_HAND
replaces it with the by-hand check (`tools/first_run.py --by-hand`). S4 (homing) and S5 (the homed arm) joined in P3
of TODO/first_run_plan.md (2026-10-27)."""
from __future__ import annotations

from first_run import stage0, stage1, stage2, stage3, stage3_hand, stage4, stage5

STEPS = stage0.STEPS + stage1.STEPS + stage2.STEPS + stage3.STEPS + stage4.STEPS + stage5.STEPS
BY_HAND = stage3_hand.STEPS
BY_ID = {s["id"]: s for s in STEPS + BY_HAND}
