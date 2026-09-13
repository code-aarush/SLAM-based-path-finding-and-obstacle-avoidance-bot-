"""
reporter.py
-----------
Generate terminal (human-readable) and JSON reports from a WorldAnalysis result.

Terminal report
---------------
Structured plain-text output suitable for debugging.
Uses no external formatting libraries.

JSON report
-----------
Writes world_analysis.json to the specified output directory.
The JSON schema is stable and intended for consumption by downstream modules
(geometry processor, occupancy grid generator) without requiring Gazebo.
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import List

from .models import CollisionEntry, DirectWorldModel, ExternalModel, WorldAnalysis

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Terminal reporter
# ---------------------------------------------------------------------------

def print_report(analysis: WorldAnalysis) -> None:
    """Print a structured human-readable report to stdout."""
    lines: List[str] = []
    sep = "=" * 70

    lines.append(sep)
    lines.append("GAZEBO WORLD ANALYSIS REPORT")
    lines.append(sep)

    # ---- World ----
    lines.append("")
    lines.append("WORLD")
    lines.append(f"  Name        : {analysis.world.name}")
    lines.append(f"  Source      : {analysis.world.source_path}")

    # ---- Summary ----
    s = analysis.summary
    lines.append("")
    lines.append("SUMMARY")
    lines.append(f"  Included instances      : {s.total_included_instances}")
    lines.append(f"  Direct world models     : {s.total_direct_world_models}")
    lines.append(f"  Total model entities    : {s.total_model_entities}")
    lines.append(f"  Unique external types   : {s.unique_external_model_types}")
    lines.append(f"  Resolved models         : {s.resolved_models}")
    lines.append(f"  Unresolved models       : {s.unresolved_models}")

    # ---- External models ----
    if analysis.external_models:
        lines.append("")
        lines.append(sep)
        lines.append("EXTERNAL MODELS (included via <include>)")
        lines.append(sep)

        for idx, em in enumerate(analysis.external_models, start=1):
            lines.append("")
            lines.append(f"[{idx}] Model Key : {em.model_key}")
            lines.append(f"    URI       : {em.uri}")
            lines.append(f"    Instances : {len(em.instances)}")

            if em.classification:
                lines.append(
                    f"    Static    : {em.classification.static_state}"
                    f"  ({em.classification.source})"
                )

            res = em.resolution
            if res:
                lines.append(f"    Resolution: {res.status.upper()}")
                if res.model_dir:
                    lines.append(f"      Model Dir : {res.model_dir}")
                if res.model_sdf:
                    lines.append(f"      model.sdf : {res.model_sdf}")
                if res.reason:
                    lines.append(f"      Reason    : {res.reason}")

            # Instance list
            lines.append("    Instance poses:")
            for inst in em.instances:
                pose_str = _fmt_pose(inst.pose)
                static_note = ""
                if inst.static_override is not None:
                    static_note = f"  [static_override={inst.static_override}]"
                lines.append(f"      • {inst.name:<30} {pose_str}{static_note}")

            # Collisions
            if em.collisions:
                lines.append(f"    Collisions ({len(em.collisions)}):")
                for cidx, coll in enumerate(em.collisions, start=1):
                    lines.extend(_fmt_collision(coll, cidx))
            else:
                lines.append("    Collisions: none extracted")

    # ---- Direct world models ----
    if analysis.direct_world_models:
        lines.append("")
        lines.append(sep)
        lines.append("DIRECT WORLD MODELS (inline <model> in world file)")
        lines.append(sep)

        for idx, dwm in enumerate(analysis.direct_world_models, start=1):
            lines.append("")
            lines.append(f"[{idx}] Name      : {dwm.name}")
            lines.append(f"    Pose      : {_fmt_pose(dwm.pose)}")
            if dwm.static_classification:
                lines.append(
                    f"    Static    : {dwm.static_classification.static_state}"
                    f"  ({dwm.static_classification.source})"
                )
            if dwm.nav_relevance:
                lines.append(
                    f"    Nav Role  : {dwm.nav_relevance.label}"
                    f"  — {dwm.nav_relevance.reason}"
                )
            if dwm.collisions:
                lines.append(f"    Collisions ({len(dwm.collisions)}):")
                for cidx, coll in enumerate(dwm.collisions, start=1):
                    lines.extend(_fmt_collision(coll, cidx))
            else:
                lines.append("    Collisions: none")

    # ---- Warnings ----
    if analysis.warnings:
        lines.append("")
        lines.append(sep)
        lines.append(f"WARNINGS ({len(analysis.warnings)})")
        lines.append(sep)
        for w in analysis.warnings:
            lines.append(f"  ⚠  {w}")

    # ---- Errors ----
    if analysis.errors:
        lines.append("")
        lines.append(sep)
        lines.append(f"ERRORS ({len(analysis.errors)})")
        lines.append(sep)
        for e in analysis.errors:
            lines.append(f"  ✗  {e}")

    lines.append("")
    lines.append(sep)
    lines.append("END OF REPORT")
    lines.append(sep)

    print("\n".join(lines))


# ---------------------------------------------------------------------------
# JSON reporter
# ---------------------------------------------------------------------------

def write_json_report(analysis: WorldAnalysis, output_dir: Path) -> Path:
    """
    Serialise the analysis to JSON and write to *output_dir/world_analysis.json*.

    Returns the path of the written file.
    """
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / "world_analysis.json"

    data = analysis.to_dict()

    with output_path.open("w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=2, ensure_ascii=False)

    logger.info("JSON report written to: %s", output_path)
    return output_path


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

def _fmt_pose(pose: List[float]) -> str:
    """Format a 6-element pose list as a compact string."""
    if not pose:
        return "[no pose]"
    return "[" + ", ".join(f"{v:.4f}" for v in pose) + "]"


def _fmt_collision(coll: CollisionEntry, idx: int) -> List[str]:
    """Format a single CollisionEntry as indented lines."""
    lines: List[str] = []
    indent = "      "
    lines.append(f"      Collision #{idx}")
    lines.append(f"{indent}  Link       : {coll.link_name}")
    lines.append(f"{indent}  Name       : {coll.collision_name}")
    lines.append(f"{indent}  Link Pose  : {_fmt_pose(coll.link_pose)}")
    lines.append(f"{indent}  Coll Pose  : {_fmt_pose(coll.collision_pose)}")

    geom = coll.geometry
    geom_type = type(geom).__name__.replace("Geometry", "").upper()

    from .models import (
        BoxGeometry,
        CylinderGeometry,
        MeshGeometry,
        PlaneGeometry,
        SphereGeometry,
        UnknownGeometry,
    )

    if isinstance(geom, BoxGeometry):
        lines.append(f"{indent}  Type       : BOX")
        lines.append(f"{indent}  Size       : {geom.size}")
    elif isinstance(geom, CylinderGeometry):
        lines.append(f"{indent}  Type       : CYLINDER")
        lines.append(f"{indent}  Radius     : {geom.radius}")
        lines.append(f"{indent}  Length     : {geom.length}")
    elif isinstance(geom, SphereGeometry):
        lines.append(f"{indent}  Type       : SPHERE")
        lines.append(f"{indent}  Radius     : {geom.radius}")
    elif isinstance(geom, PlaneGeometry):
        lines.append(f"{indent}  Type       : PLANE")
        lines.append(f"{indent}  Normal     : {geom.normal}")
        lines.append(f"{indent}  Size       : {geom.size}")
    elif isinstance(geom, MeshGeometry):
        lines.append(f"{indent}  Type       : MESH")
        lines.append(f"{indent}  URI        : {geom.uri}")
        lines.append(f"{indent}  Scale      : {geom.scale}")
        lines.append(f"{indent}  Resolved   : {geom.resolved_path or 'N/A'}")
        lines.append(f"{indent}  Exists     : {geom.file_exists}")
    elif isinstance(geom, UnknownGeometry):
        lines.append(f"{indent}  Type       : UNKNOWN ({geom.raw_tag})")
    else:
        lines.append(f"{indent}  Type       : {geom_type}")

    return lines
