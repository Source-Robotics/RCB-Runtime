"""validate_robot_config(): checks the dict load_full_robot_config() returns, one validator per rule.

Split out of the single-file config/xml_parser.py on 2026-10-03 (refactor Phase 4); definitions moved verbatim.
"""

from __future__ import annotations

import logging

import numpy as np


_val_logger = logging.getLogger(__name__)


# Required per-joint XML tags — must exist and be a finite positive number
_REQUIRED_JOINT_TAGS = ["kt", "Ilim", "gear_ratio", "encoder",
                        "joint_limit_negative", "joint_limit_positive"]


# Valid encoder resolutions in bits
_VALID_ENCODER_BITS = set(range(8, 21))   # 8 – 20 bit


def _cfg_array(cfg: dict, name: str) -> np.ndarray | None:
    """A config field as a float array, or None when the field is absent."""
    a = cfg.get(name)
    return np.asarray(a, dtype=float) if a is not None else None


def _check_joint_count(cfg: dict, issues: list[str]) -> int | None:
    """joint_num must be a positive integer — nothing per joint can be checked without it. Returns it, or None."""
    jn = cfg.get("joint_num", 0)
    if not isinstance(jn, int) or jn <= 0:
        issues.append(f"[ERROR] joint_num is {jn!r} — must be a positive integer")
        # Can't continue checking per-joint arrays without a valid count
        return None
    return jn


def _check_required_arrays(cfg: dict, jn: int, issues: list[str]) -> None:
    """Every per-joint required field is present, joint_num long and free of NaN."""
    array_map = {
        "kt":                   cfg.get("motor_kt"),
        "Ilim":                 cfg.get("motor_Ilim"),
        "gear_ratio":           cfg.get("gear_ratio"),
        "encoder":              cfg.get("motor_encoder"),
        "joint_limit_negative": cfg.get("joint_limit_negative"),
        "joint_limit_positive": cfg.get("joint_limit_positive"),
    }
    for tag, arr in array_map.items():
        if arr is None:
            issues.append(f"[ERROR] '{tag}' array is missing from config")
            continue
        arr = np.asarray(arr, dtype=float)
        if len(arr) != jn:
            issues.append(
                f"[ERROR] '{tag}' has {len(arr)} entries but joint_num={jn}"
            )
        if np.any(np.isnan(arr)):
            bad = np.where(np.isnan(arr))[0].tolist()
            issues.append(f"[ERROR] '{tag}' missing on joint(s) {bad}")


# (config field, name in the message, message suffix) — range checks only run when the array is present and full.
_POSITIVE_FIELDS = (
    ("motor_kt",   "kt",         " (must be positive)"),
    ("motor_Ilim", "Ilim",       " (must be positive mA)"),
    ("gear_ratio", "gear_ratio", ""),
)


def _check_positive(cfg: dict, issues: list[str]) -> None:
    """kt, Ilim and gear_ratio must be > 0 on every joint."""
    for name, label, suffix in _POSITIVE_FIELDS:
        arr = _cfg_array(cfg, name)
        if arr is not None and not np.any(np.isnan(arr)):
            bad = np.where(arr <= 0)[0].tolist()
            if bad:
                issues.append(f"[ERROR] {label} <= 0 on joint(s) {bad}{suffix}")


def _check_encoder_bits(cfg: dict, issues: list[str]) -> None:
    """An encoder resolution outside 8-20 bits is unusual (a warning)."""
    enc = _cfg_array(cfg, "motor_encoder")
    if enc is not None and not np.any(np.isnan(enc)):
        for i, e in enumerate(enc):
            if int(e) not in _VALID_ENCODER_BITS:
                issues.append(
                    f"[WARNING] encoder={int(e)} on joint {i} is unusual "
                    f"(expected {min(_VALID_ENCODER_BITS)}–{max(_VALID_ENCODER_BITS)} bits)"
                )


def _check_hard_limits(cfg: dict, jn: int, issues: list[str]) -> tuple[np.ndarray | None, np.ndarray | None]:
    """The hard (endstop) joint limits, in either sign convention. Returns the two arrays for the soft check."""
    lim_neg = _cfg_array(cfg, "joint_limit_negative")
    lim_pos = _cfg_array(cfg, "joint_limit_positive")
    if lim_neg is not None and lim_pos is not None:
        if len(lim_neg) == jn and len(lim_pos) == jn:
            for i, (ln, lp) in enumerate(zip(lim_neg, lim_pos)):
                if np.isnan(ln) or np.isnan(lp):
                    continue
                # Convention A: both positive magnitudes (neg limit applied in code as -ln)
                # Convention B: ln is already negative, lp is positive
                if ln >= 0 and lp >= 0:
                    # magnitude convention — just check they are non-zero
                    if ln == 0 and lp == 0:
                        issues.append(f"[WARNING] joint {i}: both joint limits are 0")
                else:
                    # signed convention — neg must be strictly less than pos
                    if ln >= lp:
                        issues.append(
                            f"[ERROR] joint {i}: joint_limit_negative ({ln}) >= "
                            f"joint_limit_positive ({lp})"
                        )
    return lim_neg, lim_pos


def _check_soft_limits(cfg: dict, jn: int, lim_neg: np.ndarray | None, lim_pos: np.ndarray | None,
                       issues: list[str]) -> None:
    """Software (soft) joint limits — what RTI and JOG hold every joint to; the hard limits are the endstops."""
    # A soft limit outside its hard limit makes both meaningless (PAR6 J6 shipped that way until
    # 2026-09-30), so it is an ERROR. Values are compared SIGNED — the only convention any
    # consumer uses (PAR6 J3's limits are both positive). A robot XML without soft limits still
    # loads, but RTI and JOG then do not limit that joint at all, so say so.
    soft_neg = _cfg_array(cfg, "software_joint_limit_negative")
    soft_pos = _cfg_array(cfg, "software_joint_limit_positive")
    if soft_neg is not None and soft_pos is not None and len(soft_neg) == jn and len(soft_pos) == jn:
        missing = [i for i in range(jn) if np.isnan(soft_neg[i]) or np.isnan(soft_pos[i])]
        if missing:
            issues.append(
                f"[WARNING] no software joint limits on joint(s) {missing} — RTI and JOG "
                f"will not limit them"
            )
        for i in range(jn):
            sn, sp = soft_neg[i], soft_pos[i]
            if np.isnan(sn) or np.isnan(sp):
                continue
            if sn >= sp:
                issues.append(
                    f"[ERROR] joint {i} (J{i + 1}): software_joint_limit_negative ({sn}) >= "
                    f"software_joint_limit_positive ({sp})"
                )
                continue
            if lim_neg is None or lim_pos is None or len(lim_neg) != jn or len(lim_pos) != jn:
                continue
            hn, hp = lim_neg[i], lim_pos[i]
            if np.isnan(hn) or np.isnan(hp):
                continue
            if sn < hn or sp > hp:
                issues.append(
                    f"[ERROR] joint {i} (J{i + 1}): software limits [{sn}, {sp}] are not inside "
                    f"the hard limits [{hn}, {hp}]"
                )


def _check_loop_timing(cfg: dict, issues: list[str]) -> None:
    """RTI_loop_time must exist and look like a loop period."""
    dt = cfg.get("delta_t")
    if dt is None:
        issues.append("[ERROR] RTI_loop_time missing from XML")
    elif not (1e-5 <= float(dt) <= 0.5):
        issues.append(
            f"[WARNING] delta_t={dt} s looks unusual (expected 10 µs – 500 ms)"
        )


def _check_gripper(cfg: dict, issues: list[str]) -> None:
    """A named gripper must have brought its kt and Gear_r along (its XML present and well-named)."""
    gripper_name = cfg.get("gripper_name", "")
    if gripper_name and gripper_name.lower() not in ("none", "unknown", ""):
        gripper_kt = cfg.get("gripper_kt")
        if gripper_kt is None or (isinstance(gripper_kt, float) and np.isnan(gripper_kt)):
            issues.append(
                f"[WARNING] gripper '{gripper_name}' kt not found — "
                f"gripper XML may be missing or mis-named"
            )
        gripper_gear_r = cfg.get("gripper_gear_r")
        if gripper_gear_r is None or (isinstance(gripper_gear_r, float) and np.isnan(gripper_gear_r)):
            issues.append(
                f"[WARNING] gripper '{gripper_name}' Gear_r not found — "
                f"gripper XML may be missing or mis-named"
            )


def _log_issues(issues: list[str]) -> bool:
    """Every issue to the log at its level, the OK line when there is none at all. Returns ok (no ERROR)."""
    ok = not any(i.startswith("[ERROR]") for i in issues)
    for msg in issues:
        if msg.startswith("[ERROR]"):
            _val_logger.error(f"[XML Validation] {msg}")
        else:
            _val_logger.warning(f"[XML Validation] {msg}")
    if ok and not issues:
        _val_logger.info("[XML Validation] OK — all checks passed")
    return ok


def validate_robot_config(cfg: dict) -> tuple[bool, list[str]]:
    """
    Validate the dict returned by load_full_robot_config().

    One validator per rule (Phase 5, 2026-10-03); this is the order they run in.

    Returns
    -------
    ok      : True if no ERROR-level issue was found (WARNINGs are non-fatal)
    issues  : list of human-readable strings, prefixed with [ERROR] or [WARNING]
    """
    issues: list[str] = []

    jn = _check_joint_count(cfg, issues)
    if jn is None:
        return False, issues
    _check_required_arrays(cfg, jn, issues)
    _check_positive(cfg, issues)
    _check_encoder_bits(cfg, issues)
    lim_neg, lim_pos = _check_hard_limits(cfg, jn, issues)
    _check_soft_limits(cfg, jn, lim_neg, lim_pos, issues)
    _check_loop_timing(cfg, issues)
    _check_gripper(cfg, issues)
    return _log_issues(issues), issues
