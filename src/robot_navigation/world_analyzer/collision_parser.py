"""
collision_parser.py
-------------------
Extract collision geometry from raw link/collision data produced by sdf_parser.py.

Supported geometry types
------------------------
    box       → BoxGeometry
    cylinder  → CylinderGeometry
    sphere    → SphereGeometry
    plane     → PlaneGeometry
    mesh      → MeshGeometry
    <other>   → UnknownGeometry  (logged as a warning)

Mesh URI resolution
-------------------
Relative mesh URIs (e.g. "meshes/foo.stl") are resolved relative to the
model directory supplied by the caller.  Absolute paths and http(s) URIs
are handled gracefully (the raw URI is stored; existence check is skipped
for remote URIs).

This module does NOT perform vertex-level geometry processing.
"""

from __future__ import annotations

import logging
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional

from .models import (
    BoxGeometry,
    CollisionEntry,
    CylinderGeometry,
    Geometry,
    MeshGeometry,
    PlaneGeometry,
    SphereGeometry,
    UnknownGeometry,
)

logger = logging.getLogger(__name__)

ZERO_POSE: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def extract_collisions(
    links: List[dict],
    model_dir: Optional[Path] = None,
) -> List[CollisionEntry]:
    """
    Convert raw link/collision dicts (from sdf_parser) into CollisionEntry objects.

    Parameters
    ----------
    links :
        Output of sdf_parser._parse_links() / parse_model_sdf()["links"].
    model_dir :
        Local model directory used for resolving relative mesh URIs.
        May be None if the model is unresolved.

    Returns
    -------
    List[CollisionEntry]
        One entry per successfully parsed collision.  Entries with missing
        geometry elements are omitted (already warned during sdf_parser pass).
    """
    entries: List[CollisionEntry] = []

    for link in links:
        link_name: str = link["link_name"]
        link_pose: List[float] = link.get("link_pose", list(ZERO_POSE))

        for raw_coll in link.get("raw_collisions", []):
            coll_name: str = raw_coll["collision_name"]
            coll_pose: List[float] = raw_coll.get("collision_pose", list(ZERO_POSE))
            geom_el: Optional[ET.Element] = raw_coll.get("geometry_el")

            if geom_el is None:
                # Already warned in sdf_parser; skip silently here.
                continue

            geometry = _parse_geometry(geom_el, model_dir, context=f"{link_name}/{coll_name}")
            if geometry is None:
                continue

            entries.append(
                CollisionEntry(
                    collision_name=coll_name,
                    link_name=link_name,
                    link_pose=link_pose,
                    collision_pose=coll_pose,
                    geometry=geometry,
                )
            )

    return entries


# ---------------------------------------------------------------------------
# Geometry dispatch
# ---------------------------------------------------------------------------

def _parse_geometry(
    geom_el: ET.Element,
    model_dir: Optional[Path],
    context: str,
) -> Optional[Geometry]:
    """
    Dispatch to the appropriate geometry parser based on the child tag.

    Returns None only if the geometry element has no recognised child tags.
    For unsupported tags returns UnknownGeometry so the caller knows
    *something* was there.
    """
    # Find the first child of <geometry>
    children = list(geom_el)
    if not children:
        logger.warning("Empty <geometry> element in %s — skipping", context)
        return None

    geom_child = children[0]
    tag = geom_child.tag.lower()

    parsers = {
        "box": _parse_box,
        "cylinder": _parse_cylinder,
        "sphere": _parse_sphere,
        "plane": _parse_plane,
        "mesh": lambda el, _ctx: _parse_mesh(el, model_dir, _ctx),
    }

    parser = parsers.get(tag)
    if parser is None:
        logger.warning("Unsupported geometry type %r in %s", tag, context)
        return UnknownGeometry(raw_tag=tag)

    try:
        return parser(geom_child, context)
    except Exception as exc:
        logger.warning("Failed to parse %r geometry in %s: %s", tag, context, exc)
        return UnknownGeometry(raw_tag=tag)


# ---------------------------------------------------------------------------
# Individual geometry parsers
# ---------------------------------------------------------------------------

def _parse_box(el: ET.Element, context: str) -> BoxGeometry:
    size_el = el.find("size")
    if size_el is None or not (size_el.text or "").strip():
        raise ValueError(f"<box> missing <size> in {context}")
    parts = size_el.text.strip().split()
    if len(parts) != 3:
        raise ValueError(f"<box><size> should have 3 values, got {parts!r} in {context}")
    return BoxGeometry(size=[float(p) for p in parts])


def _parse_cylinder(el: ET.Element, context: str) -> CylinderGeometry:
    radius_el = el.find("radius")
    length_el = el.find("length")
    if radius_el is None or length_el is None:
        raise ValueError(f"<cylinder> missing <radius> or <length> in {context}")
    return CylinderGeometry(
        radius=float((radius_el.text or "0").strip()),
        length=float((length_el.text or "0").strip()),
    )


def _parse_sphere(el: ET.Element, context: str) -> SphereGeometry:
    radius_el = el.find("radius")
    if radius_el is None:
        raise ValueError(f"<sphere> missing <radius> in {context}")
    return SphereGeometry(radius=float((radius_el.text or "0").strip()))


def _parse_plane(el: ET.Element, context: str) -> PlaneGeometry:
    normal_el = el.find("normal")
    size_el = el.find("size")

    normal: List[float] = [0.0, 0.0, 1.0]
    size: List[float] = [1.0, 1.0]

    if normal_el is not None and (normal_el.text or "").strip():
        parts = normal_el.text.strip().split()
        if len(parts) == 3:
            normal = [float(p) for p in parts]
        else:
            logger.warning("<plane><normal> has %d parts in %s, expected 3", len(parts), context)

    if size_el is not None and (size_el.text or "").strip():
        parts = size_el.text.strip().split()
        if len(parts) == 2:
            size = [float(p) for p in parts]
        elif len(parts) == 1:
            v = float(parts[0])
            size = [v, v]
        else:
            logger.warning("<plane><size> has %d parts in %s, expected 2", len(parts), context)

    return PlaneGeometry(normal=normal, size=size)


def _parse_mesh(
    el: ET.Element,
    model_dir: Optional[Path],
    context: str,
) -> MeshGeometry:
    uri_el = el.find("uri")
    if uri_el is None or not (uri_el.text or "").strip():
        raise ValueError(f"<mesh> missing <uri> in {context}")

    raw_uri = uri_el.text.strip()

    # Scale (optional; default 1 1 1)
    scale_el = el.find("scale")
    scale: List[float] = [1.0, 1.0, 1.0]
    if scale_el is not None and (scale_el.text or "").strip():
        parts = scale_el.text.strip().split()
        if len(parts) == 3:
            try:
                scale = [float(p) for p in parts]
            except ValueError:
                logger.warning("Non-numeric scale in <mesh> at %s: %r", context, parts)
        else:
            logger.warning("<mesh><scale> should have 3 values in %s, got %r", context, parts)

    resolved_path, file_exists = _resolve_mesh_uri(raw_uri, model_dir, context)

    return MeshGeometry(
        uri=raw_uri,
        scale=scale,
        resolved_path=resolved_path,
        file_exists=file_exists,
    )


# ---------------------------------------------------------------------------
# Mesh URI resolution helper
# ---------------------------------------------------------------------------

def _resolve_mesh_uri(
    uri: str,
    model_dir: Optional[Path],
    context: str,
) -> tuple[Optional[str], bool]:
    """
    Attempt to resolve a mesh URI to an absolute local path.

    Returns
    -------
    (resolved_path_str | None, file_exists)
    """
    # Remote URLs: store as-is, cannot check existence
    if uri.startswith("http://") or uri.startswith("https://"):
        logger.debug("Mesh URI is remote in %s: %s", context, uri)
        return uri, False

    # Absolute paths
    if uri.startswith("/"):
        p = Path(uri)
        return str(p), p.is_file()

    # model:// scheme (Gazebo model-relative URI)
    if uri.startswith("model://"):
        logger.warning(
            "model:// mesh URI cannot be resolved without model path registry in %s: %r",
            context, uri,
        )
        return None, False

    # file:// scheme
    if uri.startswith("file://"):
        p = Path(uri[7:])
        return str(p), p.is_file()

    # Relative path – resolve against model_dir
    if model_dir is not None:
        resolved = (model_dir / uri).resolve()
        return str(resolved), resolved.is_file()

    # No model_dir available
    logger.debug(
        "Cannot resolve relative mesh URI %r in %s (model_dir is None)", uri, context
    )
    return None, False
