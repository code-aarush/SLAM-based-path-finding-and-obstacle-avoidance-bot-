"""
models.py
---------
Domain dataclasses for the Gazebo World Analyzer.

All classes are pure data holders (no logic) and are JSON-serializable
via the to_dict() helper on WorldAnalysis.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

@dataclass
class BoxGeometry:
    """Axis-aligned box collision geometry."""
    size: List[float]  # [x, y, z]

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "box", "size": self.size}


@dataclass
class CylinderGeometry:
    """Cylinder collision geometry."""
    radius: float
    length: float

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "cylinder", "radius": self.radius, "length": self.length}


@dataclass
class SphereGeometry:
    """Sphere collision geometry."""
    radius: float

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "sphere", "radius": self.radius}


@dataclass
class PlaneGeometry:
    """Infinite plane collision geometry."""
    normal: List[float]   # [nx, ny, nz]
    size: List[float]     # [sx, sy]

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "plane", "normal": self.normal, "size": self.size}


@dataclass
class MeshGeometry:
    """Mesh collision geometry."""
    uri: str
    scale: List[float]             # [sx, sy, sz]; defaults to [1, 1, 1]
    resolved_path: Optional[str]   # absolute local path, if resolved
    file_exists: bool

    def to_dict(self) -> Dict[str, Any]:
        return {
            "type": "mesh",
            "uri": self.uri,
            "scale": self.scale,
            "resolved_path": self.resolved_path,
            "file_exists": self.file_exists,
        }


@dataclass
class UnknownGeometry:
    """Placeholder for unsupported geometry types."""
    raw_tag: str

    def to_dict(self) -> Dict[str, Any]:
        return {"type": "unknown", "raw_tag": self.raw_tag}


Geometry = BoxGeometry | CylinderGeometry | SphereGeometry | PlaneGeometry | MeshGeometry | UnknownGeometry


# ---------------------------------------------------------------------------
# Collision
# ---------------------------------------------------------------------------

@dataclass
class CollisionEntry:
    """A single <collision> element within a link."""
    collision_name: str
    link_name: str
    link_pose: List[float]       # [x, y, z, roll, pitch, yaw]
    collision_pose: List[float]  # [x, y, z, roll, pitch, yaw]
    geometry: Geometry

    def to_dict(self) -> Dict[str, Any]:
        return {
            "collision_name": self.collision_name,
            "link_name": self.link_name,
            "link_pose": self.link_pose,
            "collision_pose": self.collision_pose,
            "geometry": self.geometry.to_dict(),
        }


# ---------------------------------------------------------------------------
# Model instance (one occurrence in the world file)
# ---------------------------------------------------------------------------

@dataclass
class ModelInstance:
    """A single <include> or direct <model> occurrence in the world file."""
    name: str
    pose: List[float]                 # [x, y, z, roll, pitch, yaw]
    static_override: Optional[bool]   # explicit <static> inside <include>, if present

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "pose": self.pose,
            "static_override": self.static_override,
        }


# ---------------------------------------------------------------------------
# Resolution result
# ---------------------------------------------------------------------------

@dataclass
class ResolutionResult:
    """Outcome of attempting to resolve a Fuel URI to a local path."""
    status: str            # "resolved" | "unresolved"
    reason: Optional[str]  # human-readable failure reason (None on success)
    model_dir: Optional[str]   # local model directory path (str for JSON)
    model_sdf: Optional[str]   # local model.sdf path (str for JSON)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status,
            "reason": self.reason,
            "model_dir": self.model_dir,
            "model_sdf": self.model_sdf,
        }


# ---------------------------------------------------------------------------
# Static classification
# ---------------------------------------------------------------------------

@dataclass
class StaticClassification:
    """Static / dynamic / unknown classification with provenance."""
    static_state: str   # "STATIC" | "DYNAMIC" | "UNKNOWN"
    source: str         # description of evidence used

    def to_dict(self) -> Dict[str, Any]:
        return {"static_state": self.static_state, "source": self.source}


# ---------------------------------------------------------------------------
# Navigation relevance
# ---------------------------------------------------------------------------

@dataclass
class NavRelevance:
    """Conservative navigation relevance classification."""
    label: str    # "TRAVERSABLE" | "OBSTACLE" | "UNKNOWN"
    reason: str

    def to_dict(self) -> Dict[str, Any]:
        return {"label": self.label, "reason": self.reason}


# ---------------------------------------------------------------------------
# External model (shared model definition referenced by ≥1 instances)
# ---------------------------------------------------------------------------

@dataclass
class ExternalModel:
    """
    One unique model type referenced by one or more <include> stanzas.

    model_key  : normalised URI used as the grouping key
    uri        : original URI string from the world file
    instances  : all world instances pointing to this model
    resolution : outcome of local Fuel cache lookup
    classification : static/dynamic/unknown
    collisions : extracted collision entries (empty if unresolved)
    """
    model_key: str
    uri: str
    instances: List[ModelInstance] = field(default_factory=list)
    resolution: Optional[ResolutionResult] = None
    classification: Optional[StaticClassification] = None
    collisions: List[CollisionEntry] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "model_key": self.model_key,
            "uri": self.uri,
            "model_sdf": self.resolution.model_sdf if self.resolution else None,
            "resolution": self.resolution.to_dict() if self.resolution else None,
            "classification": self.classification.to_dict() if self.classification else None,
            "instances": [i.to_dict() for i in self.instances],
            "collisions": [c.to_dict() for c in self.collisions],
        }


# ---------------------------------------------------------------------------
# Direct world model (inline <model> inside <world>)
# ---------------------------------------------------------------------------

@dataclass
class DirectWorldModel:
    """
    A model defined inline inside the world file (not via <include>).
    """
    name: str
    pose: List[float]
    static_classification: Optional[StaticClassification]
    collisions: List[CollisionEntry] = field(default_factory=list)
    nav_relevance: Optional[NavRelevance] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "pose": self.pose,
            "static_classification": (
                self.static_classification.to_dict()
                if self.static_classification else None
            ),
            "nav_relevance": self.nav_relevance.to_dict() if self.nav_relevance else None,
            "collisions": [c.to_dict() for c in self.collisions],
        }


# ---------------------------------------------------------------------------
# Top-level analysis result
# ---------------------------------------------------------------------------

@dataclass
class WorldInfo:
    """High-level metadata about the parsed world."""
    name: str
    source_path: str

    def to_dict(self) -> Dict[str, Any]:
        return {"name": self.name, "source_path": self.source_path}


@dataclass
class Summary:
    """Aggregate counts derived from the analysis."""
    total_included_instances: int = 0
    total_direct_world_models: int = 0
    total_model_entities: int = 0
    unique_external_model_types: int = 0
    resolved_models: int = 0
    unresolved_models: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "total_included_instances": self.total_included_instances,
            "total_direct_world_models": self.total_direct_world_models,
            "total_model_entities": self.total_model_entities,
            "unique_external_model_types": self.unique_external_model_types,
            "resolved_models": self.resolved_models,
            "unresolved_models": self.unresolved_models,
        }


@dataclass
class WorldAnalysis:
    """Root container for a complete world analysis result."""
    metadata: Dict[str, Any]
    world: WorldInfo
    summary: Summary
    external_models: List[ExternalModel] = field(default_factory=list)
    direct_world_models: List[DirectWorldModel] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "metadata": self.metadata,
            "world": self.world.to_dict(),
            "summary": self.summary.to_dict(),
            "external_models": [m.to_dict() for m in self.external_models],
            "direct_world_models": [m.to_dict() for m in self.direct_world_models],
            "warnings": self.warnings,
            "errors": self.errors,
        }
