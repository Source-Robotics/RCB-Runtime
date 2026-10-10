#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Filename: data_recipe.py
Author: Petar Crnjak
Company: Source Robotics
Description:
    Loads data recipes from XML and builds serialized telemetry payloads.

    Each recipe is its own XML file in the data_recipe/ directory:

        data_recipe/
            standard.xml
            minimal.xml
            full.xml
            my_custom_recipe.xml       ← just drop a file here, it's auto-detected

    XML format (one recipe per file, root element IS the recipe):

        <?xml version="1.0" encoding="UTF-8"?>
        <recipe name="standard">
            <field tag="j_p" precision="3"/>
            <field tag="tick"/>
        </recipe>

    Usage in RTI loop:

        from data.data_recipe import RecipeManager, build_payload

        recipe_mgr = RecipeManager(DATA_RECIPE_DIR)
        # Build once before the loop — references are stable.
        motor_buffers = {
            "data":        Motor_data,
            "temperature": Motor_temperature_data,
            "voltage":     Motor_voltage_data,
            "errors":      Motor_errors_data,
            "config":      shared_data.motor_config,
        }
        ...
        # resolved once per tick — zero-allocation hot path after caching
        recipe  = recipe_mgr.get(shared_data.active_recipe.get())
        payload = build_payload(recipe, Joint_data, Gripper_data, motor_buffers, joint_num)
        broadcast_udp_telemetry(telemetry_socket, payload)

    The active recipe name is a SharedString so the GUI can switch it live
    without restarting RTI.  Call recipe_mgr.reload() after uploading a new file.

Performance note:
    All per-recipe lists and dicts (_joint_fields, _gripper_fields, _prec_map) are
    computed ONCE at load time and stored as plain attributes.  build_payload() never
    allocates anything — it only reads those cached references, matching the
    performance of a fully hardcoded serialize_optimized() call.
"""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Optional
import logging

from data.robotics_data import TAGS
from utility.serialization import (
    serialize_optimized,
    serialize_motor_data,
    serialize_gripper_motor_data,
    serialize_motor_config,
    serialize_gripper_config,
)

logger = logging.getLogger(__name__)


# ================================================================
# DataRecipe  — all derived data pre-computed at load time
# ================================================================

class DataRecipe:
    """
    One named telemetry recipe.

    All lists and the precision map are computed once from the raw XML data
    and cached as plain attributes so the RTI hot path does zero work.
    """
    __slots__ = (
        "name", "fields",
        "_joint_fields", "_gripper_fields",
        "_motor_fields", "_gripper_motor_fields",
        "_motor_config_fields", "_gripper_config_fields",
        "_prec_map",
    )

    def __init__(self, name: str, fields: list, precision_overrides: dict):
        self.name   = name
        self.fields = fields  # kept for introspection / docs

        # Pre-compute split field lists — used directly by build_payload()
        self._joint_fields          = [t for t in fields if t in TAGS and TAGS[t]["src"] == "joint"]
        self._gripper_fields        = [t for t in fields if t in TAGS and TAGS[t]["src"] == "gripper"]
        self._motor_fields          = [t for t in fields if t in TAGS and TAGS[t]["src"] == "motor"]
        self._gripper_motor_fields  = [t for t in fields if t in TAGS and TAGS[t]["src"] == "gripper_motor"]
        self._motor_config_fields   = [t for t in fields if t in TAGS and TAGS[t]["src"] == "motor_config"]
        self._gripper_config_fields = [t for t in fields if t in TAGS and TAGS[t]["src"] == "gripper_config"]

        # Pre-compute full precision map — used directly by serialize_optimized()
        self._prec_map = {
            tag: precision_overrides.get(tag, TAGS[tag]["prec"])
            for tag in fields
            if tag in TAGS
        }


# ================================================================
# XML loader  — one file = one recipe
# ================================================================

def load_recipe_from_file(xml_path: str | Path) -> Optional[DataRecipe]:
    """
    Parse a single recipe XML file and return a DataRecipe, or None on error.
    The root element must be <recipe name="...">.
    """
    xml_path = Path(xml_path)
    try:
        root = ET.parse(xml_path).getroot()
    except FileNotFoundError:
        logger.error(f"[DataRecipe] File not found: {xml_path}")
        return None
    except ET.ParseError as e:
        logger.error(f"[DataRecipe] XML parse error in '{xml_path}': {e}")
        return None
    return _recipe_from_root(root, xml_path.name)


def load_recipe_from_text(text: str, label: str) -> Optional[DataRecipe]:
    """The same recipe from the file's TEXT (2026-10-04): command_executor reads data_recipe/*.xml and hands RTI
    the texts with reload_recipes, so the RT process parses from memory and never opens a file in the tick.
    `label` names the file in log lines."""
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        logger.error(f"[DataRecipe] XML parse error in '{label}': {e}")
        return None
    return _recipe_from_root(root, label)


def _recipe_from_root(root: ET.Element, label: str) -> Optional[DataRecipe]:
    """One parsed <recipe> root -> DataRecipe, or None (logged) when it is not a usable recipe."""
    if root.tag != "recipe":
        logger.error(
            f"[DataRecipe] '{label}': root element must be <recipe>, got <{root.tag}> — skipped."
        )
        return None

    name = root.get("name", "").strip()
    if not name:
        logger.warning(f"[DataRecipe] '{label}': <recipe> has no name attribute — skipped.")
        return None

    fields: list = []
    overrides: dict = {}

    for field_el in root.findall("field"):
        tag = field_el.get("tag", "").strip()
        if not tag:
            continue
        if tag not in TAGS:
            logger.warning(f"[DataRecipe] Recipe '{name}': unknown tag '{tag}' — skipped.")
            continue
        fields.append(tag)
        prec_str = field_el.get("precision")
        if prec_str is not None:
            try:
                overrides[tag] = int(prec_str)
            except ValueError:
                logger.warning(
                    f"[DataRecipe] Recipe '{name}': bad precision value for '{tag}' — using default."
                )

    if not fields:
        logger.warning(f"[DataRecipe] Recipe '{name}' has no valid fields — skipped.")
        return None

    logger.info(f"[DataRecipe] Loaded '{name}' ({len(fields)} fields) from '{label}'.")
    return DataRecipe(name=name, fields=fields, precision_overrides=overrides)


# ================================================================
# Payload builder  — called every RTI tick — zero allocation
# ================================================================

def build_payload(recipe: DataRecipe, Joint_data, Gripper_data, motor_buffers, joint_num: int) -> str:
    """
    Serialize the recipe into one comma-separated string.

    motor_buffers is a dict bundling the three per-node motor deque dicts:
        {"data": Motor_data, "temperature": Motor_temperature_data,
         "voltage": Motor_voltage_data}

    Uses pre-computed field lists on the recipe and the precision map —
    no dict or list is created here. Cost is identical to calling the
    serializers directly with hardcoded arguments.
    """
    parts = []

    if recipe._joint_fields and Joint_data:
        parts.append(serialize_optimized(Joint_data, recipe._joint_fields, recipe._prec_map))

    if recipe._motor_fields and motor_buffers:
        parts.append(serialize_motor_data(motor_buffers, recipe._motor_fields, recipe._prec_map, joint_num))

    if recipe._gripper_motor_fields and motor_buffers:
        parts.append(serialize_gripper_motor_data(motor_buffers, recipe._gripper_motor_fields, recipe._prec_map, joint_num))

    if recipe._motor_config_fields and motor_buffers:
        parts.append(serialize_motor_config(motor_buffers, recipe._motor_config_fields, recipe._prec_map, joint_num))

    if recipe._gripper_config_fields and motor_buffers:
        parts.append(serialize_gripper_config(motor_buffers, recipe._gripper_config_fields, recipe._prec_map))

    if recipe._gripper_fields and Gripper_data:
        gripper_id = joint_num   # gripper slot is always right after the last joint
        if gripper_id in Gripper_data and Gripper_data[gripper_id]:
            parts.append(
                serialize_optimized(Gripper_data[gripper_id], recipe._gripper_fields, recipe._prec_map)
            )

    return ",".join(p for p in parts if p)


# ================================================================
# RecipeManager  — scans a directory for *.xml recipe files
# ================================================================

class RecipeManager:
    """
    Discovers and owns all recipes found in a directory.

    Drop a new .xml file into the recipe directory and call reload() —
    it appears instantly without restarting RTI.

    The active recipe is resolved by name.  The last resolved recipe object is
    cached so repeated calls with the same name cost only one string comparison,
    not a dict lookup.
    """

    def __init__(self, recipe_dir: str | Path, fallback: Optional[DataRecipe] = None):
        self.recipe_dir = Path(recipe_dir)
        self._recipes:  dict[str, DataRecipe] = {}
        self._fallback: DataRecipe = fallback or _make_fallback()
        # Cache last-used recipe to skip dict lookup when name hasn't changed
        self._cached_name:   str            = ""
        self._cached_recipe: DataRecipe     = self._fallback
        self.reload()

    def reload(self):
        """
        Re-scan the recipe directory and reload all *.xml files.
        Safe to call at any time from any thread. (RTI no longer calls this in its loop — see reload_from_texts.)
        """
        loaded: dict = {}
        if not self.recipe_dir.is_dir():
            logger.error(f"[RecipeManager] Directory not found: {self.recipe_dir}")
        else:
            for xml_file in sorted(self.recipe_dir.glob("*.xml")):
                recipe = load_recipe_from_file(xml_file)
                if recipe is not None:
                    loaded[recipe.name] = recipe
        self._install(loaded)

    def reload_from_texts(self, texts: dict) -> None:
        """Reload from {file name: xml text} (2026-10-04): what command_executor's reload_recipes carries, so the
        RT process never reads the directory itself. The same install rule as reload(): nothing usable -> the
        recipes in force stay (the fallback only if there were none)."""
        loaded: dict = {}
        for label in sorted(texts):
            recipe = load_recipe_from_text(str(texts[label]), str(label))
            if recipe is not None:
                loaded[recipe.name] = recipe
        self._install(loaded)

    def _install(self, loaded: dict) -> None:
        if loaded:
            self._recipes = loaded
            # Invalidate name cache
            self._cached_name   = ""
            self._cached_recipe = self._fallback
            logger.info(
                f"[RecipeManager] {len(loaded)} recipe(s) loaded from '{self.recipe_dir}': "
                f"{list(loaded.keys())}"
            )
        else:
            logger.warning(
                f"[RecipeManager] No recipes found in '{self.recipe_dir}' — fallback active."
            )

    def get(self, name: str) -> DataRecipe:
        """
        Return the DataRecipe for the given name.

        Caches the last result: if the name has not changed since the previous
        call (the common case every RTI tick), returns immediately without a
        dict lookup.
        """
        if name == self._cached_name:
            return self._cached_recipe
        recipe = self._recipes.get(name)
        if recipe is None:
            logger.debug(f"[RecipeManager] Recipe '{name}' not found — using fallback.")
            recipe = self._fallback
        self._cached_name   = name
        self._cached_recipe = recipe
        return recipe

    @property
    def names(self) -> list:
        """Sorted list of loaded recipe names."""
        return sorted(self._recipes.keys())


def _make_fallback() -> DataRecipe:
    """Minimal safe recipe used when the directory is missing or all files are invalid."""
    return DataRecipe(name="fallback", fields=["j_p", "tick"], precision_overrides={"j_p": 3})

