"""
sdf_parser.py
-------------
Parse Gazebo SDF world files and individual model SDF files.

Responsibilities
----------------
- Load and validate world XML.
- Extract <include> stanzas (URI, name, pose, static override).
- Extract inline <model> definitions.
- Parse model SDF files (links, poses, collision elements).
- Pose parsing: always return a 6-element list [x, y, z, roll, pitch, yaw].

This module does NOT resolve URIs or produce CollisionEntry objects —
those responsibilities belong to asset_resolver.py and collision_parser.py.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# Zero pose used as the default when a <pose> element is absent.
ZERO_POSE: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _parse_pose(element: Optional[ET.Element], context: str = "") -> List[float]:
    """
    Parse a <pose>x y z roll pitch yaw</pose> element.

    Returns a 6-element float list.  Missing or malformed values are padded
    with 0.0 and a warning is logged.
    """
    if element is None:
        return list(ZERO_POSE)

    text = (element.text or "").strip()
    parts = text.split()

    if len(parts) == 6:
        try:
            return [float(p) for p in parts]
        except ValueError:
            logger.warning("Non-numeric pose values in %s: %r — using zeros", context, text)
            return list(ZERO_POSE)

    if len(parts) < 6:
        logger.warning(
            "Pose in %s has only %d values (expected 6): %r — padding with 0.0",
            context, len(parts), text,
        )
        values: List[float] = []
        for p in parts:
            try:
                values.append(float(p))
            except ValueError:
                values.append(0.0)
        values.extend([0.0] * (6 - len(values)))
        return values

    # More than 6 parts: use first 6 and warn.
    logger.warning(
        "Pose in %s has %d values (expected 6): %r — using first 6",
        context, len(parts), text,
    )
    try:
        return [float(p) for p in parts[:6]]
    except ValueError:
        return list(ZERO_POSE)


def _parse_static(element: Optional[ET.Element]) -> Optional[bool]:
    """
    Parse a <static> element and return True/False/None.

    Accepts: 'true', '1' → True; 'false', '0' → False; None if absent.
    """
    if element is None:
        return None
    text = (element.text or "").strip().lower()
    if text in ("true", "1"):
        return True
    if text in ("false", "0"):
        return False
    logger.warning("Unrecognised <static> value: %r — treating as None", text)
    return None


# ---------------------------------------------------------------------------
# World-level parsing
# ---------------------------------------------------------------------------

def load_world_xml(world_path: Path) -> Tuple[ET.Element, str]:
    """
    Load and return (world_element, world_name) from an SDF world file.

    Raises
    ------
    FileNotFoundError
        If world_path does not exist.
    ValueError
        If the XML is invalid or the <world> element is missing.
    """
    if not world_path.exists():
        raise FileNotFoundError(f"World file not found: {world_path}")

    try:
        tree = ET.parse(world_path)
    except ET.ParseError as exc:
        raise ValueError(f"Invalid XML in world file {world_path}: {exc}") from exc

    root = tree.getroot()

    # <sdf><world …> or <world …> directly
    world_el = root.find("world")
    if world_el is None:
        if root.tag == "world":
            world_el = root
        else:
            raise ValueError(
                f"No <world> element found in {world_path}. "
                f"Root tag is <{root.tag}>."
            )

    world_name = world_el.get("name", "unnamed_world")
    return world_el, world_name


# ---------------------------------------------------------------------------
# Include stanza parsing
# ---------------------------------------------------------------------------

def _parse_uri(include_el: ET.Element, context: str) -> Optional[str]:
    """Extract and normalise URI text from an <include> element."""
    uri_el = include_el.find("uri")
    if uri_el is None or not (uri_el.text or "").strip():
        logger.warning("Missing or empty <uri> in include at %s", context)
        return None
    return uri_el.text.strip()


def parse_includes(world_el: ET.Element) -> List[Dict]:
    """
    Return a list of dicts, one per <include> element found in *world_el*.

    Each dict has keys:
        uri            : str | None
        name           : str | None
        pose           : List[float]
        static_override: bool | None
    """
    results = []
    for idx, inc in enumerate(world_el.findall("include")):
        ctx = f"include[{idx}]"

        uri = _parse_uri(inc, ctx)
        name_el = inc.find("name")
        name = (name_el.text or "").strip() if name_el is not None else None
        if not name:
            logger.warning("Missing or empty <name> in %s (uri=%s)", ctx, uri)
            name = None

        pose = _parse_pose(inc.find("pose"), context=f"{ctx}/pose")
        static_override = _parse_static(inc.find("static"))

        results.append(
            {
                "uri": uri,
                "name": name,
                "pose": pose,
                "static_override": static_override,
            }
        )

    logger.debug("Parsed %d <include> stanzas from world", len(results))
    return results


# ---------------------------------------------------------------------------
# Direct world-model parsing
# ---------------------------------------------------------------------------

def parse_direct_models(world_el: ET.Element) -> List[Dict]:
    """
    Return a list of dicts for every <model> element *directly* under *world_el*.

    Each dict has keys:
        name         : str
        pose         : List[float]
        static_value : bool | None
        links        : List[Dict]   (see _parse_links)
    """
    results = []
    for model_el in world_el.findall("model"):
        name = model_el.get("name", "unnamed_model")
        pose = _parse_pose(model_el.find("pose"), context=f"model:{name}/pose")
        static_value = _parse_static(model_el.find("static"))
        links = _parse_links(model_el, parent_name=name)
        results.append(
            {
                "name": name,
                "pose": pose,
                "static_value": static_value,
                "links": links,
            }
        )

    logger.debug("Parsed %d direct world models", len(results))
    return results


# ---------------------------------------------------------------------------
# Model SDF parsing
# ---------------------------------------------------------------------------

def load_model_sdf(model_sdf_path: Path) -> ET.Element:
    """
    Load a model.sdf file and return the <model> element.

    Raises
    ------
    FileNotFoundError, ValueError
    """
    if not model_sdf_path.exists():
        raise FileNotFoundError(f"model.sdf not found: {model_sdf_path}")

    try:
        tree = ET.parse(model_sdf_path)
    except ET.ParseError as exc:
        raise ValueError(f"Invalid XML in {model_sdf_path}: {exc}") from exc

    root = tree.getroot()

    # Handle both <sdf><model> and bare <model>
    model_el = root.find("model")
    if model_el is None:
        if root.tag == "model":
            model_el = root
        else:
            raise ValueError(
                f"No <model> element found in {model_sdf_path}. "
                f"Root tag is <{root.tag}>."
            )

    return model_el


def parse_model_sdf(model_sdf_path: Path) -> Dict:
    """
    Parse a model.sdf file and return a structured dict with:

        name        : str
        pose        : List[float]   (model-level pose)
        static_value: bool | None
        links       : List[Dict]
    """
    model_el = load_model_sdf(model_sdf_path)
    name = model_el.get("name", model_sdf_path.parent.name)
    pose = _parse_pose(model_el.find("pose"), context=f"model:{name}/pose")
    static_value = _parse_static(model_el.find("static"))
    links = _parse_links(model_el, parent_name=name)

    return {
        "name": name,
        "pose": pose,
        "static_value": static_value,
        "links": links,
    }


# ---------------------------------------------------------------------------
# Link / collision raw extraction (geometry left as raw XML for collision_parser)
# ---------------------------------------------------------------------------

def _parse_links(model_el: ET.Element, parent_name: str) -> List[Dict]:
    """
    Extract link data from a <model> element.

    Returns a list of dicts, one per <link>:
        link_name         : str
        link_pose         : List[float]
        raw_collisions    : List[Dict]  (raw collision dicts, geometry as ET.Element)
    """
    links = []
    for link_el in model_el.findall("link"):
        link_name = link_el.get("name", "unnamed_link")
        link_pose = _parse_pose(
            link_el.find("pose"),
            context=f"model:{parent_name}/link:{link_name}/pose",
        )
        raw_collisions = _parse_raw_collisions(link_el, parent_name, link_name)
        links.append(
            {
                "link_name": link_name,
                "link_pose": link_pose,
                "raw_collisions": raw_collisions,
            }
        )

    return links


def _parse_raw_collisions(
    link_el: ET.Element, model_name: str, link_name: str
) -> List[Dict]:
    """
    Extract raw collision data from a <link> element.

    Returns a list of dicts:
        collision_name   : str
        collision_pose   : List[float]
        geometry_el      : ET.Element | None   (the <geometry> element)
    """
    raw = []
    for coll_el in link_el.findall("collision"):
        coll_name = coll_el.get("name", "unnamed_collision")
        coll_pose = _parse_pose(
            coll_el.find("pose"),
            context=f"model:{model_name}/link:{link_name}/collision:{coll_name}/pose",
        )
        geometry_el = coll_el.find("geometry")
        if geometry_el is None:
            logger.warning(
                "collision %r in link %r (model %r) has no <geometry> — skipping",
                coll_name, link_name, model_name,
            )
        raw.append(
            {
                "collision_name": coll_name,
                "collision_pose": coll_pose,
                "geometry_el": geometry_el,
            }
        )

    return raw
