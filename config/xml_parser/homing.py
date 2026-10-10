"""The robot's homing sequence (config/<robot>_homing.xml).

Split out of the single-file config/xml_parser.py on 2026-10-03 (refactor Phase 4); definitions moved verbatim.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


# ---------------------------------------------------------------------------
# Homing sequence parser
# ---------------------------------------------------------------------------

def parse_homing_sequence(xml_path: str | Path) -> list[dict]:
    """
    Parse a homing sequence XML file and return a list of step dicts.

    Each step dict contains:
        pre_moves   : list of dicts with keys: joint (int), position_rad (float), speed (float)
        home_joints : list of int joint indices (or 'gripper' string for gripper)
        gripper_mode: None | 'motor' | 'firmware'  (only set when 'gripper' is in home_joints)
        post_moves  : list of dicts (same format as pre_moves; only on the last entry)

    Returns:
        list of step dicts, plus an optional trailing post_moves step
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    steps = []

    for step_elem in root.findall('step'):
        step = {'pre_moves': [], 'home_joints': [], 'gripper_mode': None, 'move_to': []}

        # Parse pre_moves
        pre_moves_elem = step_elem.find('pre_moves')
        if pre_moves_elem is not None:
            for move_elem in pre_moves_elem.findall('move'):
                step['pre_moves'].append({
                    'type': 'position',
                    'joint': int(move_elem.get('joint')),
                    'position_rad': float(move_elem.get('position_rad')),
                    'speed': float(move_elem.get('speed')),
                })
            for nudge_elem in pre_moves_elem.findall('nudge'):
                step['pre_moves'].append({
                    'type': 'nudge',
                    'joint': int(nudge_elem.get('joint')),
                    'speed': float(nudge_elem.get('speed')),
                    'duration_s': float(nudge_elem.get('duration_s')),
                })
            for idle_elem in pre_moves_elem.findall('idle'):
                step['pre_moves'].append({
                    'type': 'idle',
                    'joint': int(idle_elem.get('joint')),
                    'duration_s': float(idle_elem.get('duration_s')),
                })
            for gm_elem in pre_moves_elem.findall('gripper_move'):
                # position and speed are 0-255; current is mA (2 bytes).
                # activate, action, ESTOP, release_dir are bitfield flags (0/1).
                step['pre_moves'].append({
                    'type':        'gripper_move',
                    'joint':       'gripper',   # resolved to _arm_count at runtime
                    'position':    int(gm_elem.get('position')),
                    'speed':       int(gm_elem.get('speed')),
                    'current':     int(gm_elem.get('current')),
                    'activate':    int(gm_elem.get('activate',    1)),
                    'action':      int(gm_elem.get('action',      1)),
                    'estop':       int(gm_elem.get('estop',       0)),
                    'release_dir': int(gm_elem.get('release_dir', 0)),
                    'duration_s':  float(gm_elem.get('duration_s')),
                })

        # Parse home joints
        home_elem = step_elem.find('home')
        if home_elem is not None:
            joints_str = home_elem.get('joints', '')
            joint_list = []
            has_gripper = False
            for tok in joints_str.split(','):
                tok = tok.strip()
                if tok.lower() == 'gripper':
                    has_gripper = True
                    joint_list.append('gripper')
                else:
                    joint_list.append(int(tok))
            step['home_joints'] = joint_list

            if has_gripper:
                step['gripper_mode'] = home_elem.get('mode', 'motor')

        # Parse move_to elements (smooth joint-space moves)
        for mt_elem in step_elem.findall('move_to'):
            step['move_to'].append({
                'joint':        int(mt_elem.get('joint')),
                'position_rad': float(mt_elem.get('position_rad')),
                'duration_s':   float(mt_elem.get('duration_s')),
            })

        steps.append(step)

    # Parse optional global post_moves (after all steps)
    post_moves_elem = root.find('post_moves')
    if post_moves_elem is not None:
        post_moves = []
        for move_elem in post_moves_elem.findall('move'):
            post_moves.append({
                'joint': int(move_elem.get('joint')),
                'position_rad': float(move_elem.get('position_rad')),
                'speed': float(move_elem.get('speed')),
            })
        if post_moves:
            steps.append({
                'pre_moves': [],
                'home_joints': [],
                'gripper_mode': None,
                'post_moves': post_moves,
            })

    return steps
