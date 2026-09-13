"""
analyzer.py
-----------
Orchestrate the full world-analysis pipeline.

Pipeline
--------
1. Load and parse the world SDF (sdf_parser).
2. Collect all <include> stanzas → ModelInstance objects.
3. Group instances by normalised URI → ExternalModel objects.
4. For each unique model: resolve local Fuel path (asset_resolver).
5. For each resolved model: parse model.sdf, extract collision geometry.
6. Parse direct world <model> elements.
7. Compute summary statistics.
8. Return a WorldAnalysis object.

Error isolation
---------------
Every per-model step is wrapped in try/except.  Failures are appended to
WorldAnalysis.warnings / .errors and do not interrupt the analysis of
other models.
"""

from __future__ import annotations

import datetime
import logging
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional

from .asset_resolver import DEFAULT_FUEL_CACHE, normalize_uri, resolve_uri
from .collision_parser import extract_collisions
from .models import (
    DirectWorldModel,
    ExternalModel,
    ModelInstance,
    NavRelevance,
    ResolutionResult,
    StaticClassification,
    Summary,
    WorldAnalysis,
    WorldInfo,
)
from .sdf_parser import parse_direct_models, parse_includes, parse_model_sdf, load_world_xml

logger = logging.getLogger(__name__)

ANALYZER_VERSION = "1.0.0"


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def analyze_world(
    world_path: Path,
    fuel_cache: Path = DEFAULT_FUEL_CACHE,
) -> WorldAnalysis:
    """
    Run the full analysis pipeline on *world_path*.

    Parameters
    ----------
    world_path :
        Absolute path to the .sdf world file.
    fuel_cache :
        Root of the local Gazebo Fuel cache.

    Returns
    -------
    WorldAnalysis
        Complete analysis result.  Always returns (never raises).
    """
    metadata = {
        "analyzer_version": ANALYZER_VERSION,
        "generated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "source_world_file": str(world_path),
    }

    # ------------------------------------------------------------------ #
    # Step 1: Load world XML                                               #
    # ------------------------------------------------------------------ #
    try:
        world_el, world_name = load_world_xml(world_path)
    except (FileNotFoundError, ValueError) as exc:
        error_msg = f"Cannot load world file: {exc}"
        logger.error(error_msg)
        dummy_world = WorldInfo(name="unknown", source_path=str(world_path))
        return WorldAnalysis(
            metadata=metadata,
            world=dummy_world,
            summary=Summary(),
            errors=[error_msg],
        )

    world_info = WorldInfo(name=world_name, source_path=str(world_path))
    warnings: List[str] = []
    errors: List[str] = []

    # ------------------------------------------------------------------ #
    # Step 2: Parse <include> stanzas                                      #
    # ------------------------------------------------------------------ #
    raw_includes = parse_includes(world_el)

    # ------------------------------------------------------------------ #
    # Step 3: Group instances by normalised URI                            #
    # ------------------------------------------------------------------ #
    uri_to_instances: Dict[str, List[dict]] = defaultdict(list)
    uri_to_raw_uri: Dict[str, str] = {}  # normalised → first-seen original

    for raw in raw_includes:
        uri = raw.get("uri")
        if not uri:
            msg = (
                f"Include with name={raw.get('name')!r} has no URI — skipping instance"
            )
            warnings.append(msg)
            logger.warning(msg)
            continue
        key = normalize_uri(uri)
        uri_to_instances[key].append(raw)
        if key not in uri_to_raw_uri:
            uri_to_raw_uri[key] = uri

    # ------------------------------------------------------------------ #
    # Step 4-5: Build ExternalModel objects                                #
    # ------------------------------------------------------------------ #
    external_models: List[ExternalModel] = []

    for norm_uri, instance_dicts in uri_to_instances.items():
        original_uri = uri_to_raw_uri[norm_uri]
        model_key = _uri_to_key(original_uri)

        instances = [
            ModelInstance(
                name=d["name"] or "<unnamed>",
                pose=d["pose"],
                static_override=d.get("static_override"),
            )
            for d in instance_dicts
        ]

        em = ExternalModel(
            model_key=model_key,
            uri=original_uri,
            instances=instances,
        )

        # ---- Resolve local Fuel path ----
        resolution = _safe_resolve(original_uri, fuel_cache, warnings, errors)
        em.resolution = resolution

        # ---- Parse model.sdf if resolved ----
        model_sdf_path: Optional[Path] = None
        raw_model_data: Optional[dict] = None

        if resolution.status == "resolved" and resolution.model_sdf:
            model_sdf_path = Path(resolution.model_sdf)
            raw_model_data = _safe_parse_model_sdf(
                model_sdf_path, model_key, warnings, errors
            )

        # ---- Static classification ----
        em.classification = _classify_static(
            instance_dicts=instance_dicts,
            model_data=raw_model_data,
            model_key=model_key,
        )

        # ---- Extract collisions ----
        if raw_model_data is not None:
            model_dir = model_sdf_path.parent if model_sdf_path else None
            em.collisions = _safe_extract_collisions(
                raw_model_data["links"], model_dir, model_key, warnings, errors
            )

        external_models.append(em)
        logger.debug("Processed external model: %s (%d instances)", model_key, len(instances))

    # ------------------------------------------------------------------ #
    # Step 6: Parse direct world models                                    #
    # ------------------------------------------------------------------ #
    direct_world_models: List[DirectWorldModel] = []
    raw_direct = parse_direct_models(world_el)

    for raw_m in raw_direct:
        dwm = _build_direct_world_model(raw_m, warnings, errors)
        direct_world_models.append(dwm)

    # ------------------------------------------------------------------ #
    # Step 7: Summary                                                      #
    # ------------------------------------------------------------------ #
    total_included = sum(len(em.instances) for em in external_models)
    resolved_count = sum(
        1 for em in external_models
        if em.resolution and em.resolution.status == "resolved"
    )

    summary = Summary(
        total_included_instances=total_included,
        total_direct_world_models=len(direct_world_models),
        total_model_entities=total_included + len(direct_world_models),
        unique_external_model_types=len(external_models),
        resolved_models=resolved_count,
        unresolved_models=len(external_models) - resolved_count,
    )

    return WorldAnalysis(
        metadata=metadata,
        world=world_info,
        summary=summary,
        external_models=external_models,
        direct_world_models=direct_world_models,
        warnings=warnings,
        errors=errors,
    )


# ---------------------------------------------------------------------------
# Helper: safe wrappers
# ---------------------------------------------------------------------------

def _safe_resolve(
    uri: str,
    fuel_cache: Path,
    warnings: List[str],
    errors: List[str],
) -> ResolutionResult:
    """Resolve URI; log and record failures without raising."""
    try:
        result = resolve_uri(uri, fuel_cache)
        if result.status != "resolved":
            msg = f"Unresolved model URI {uri!r}: {result.reason}"
            warnings.append(msg)
            logger.warning(msg)
        return result
    except Exception as exc:
        msg = f"Unexpected error resolving {uri!r}: {exc}"
        errors.append(msg)
        logger.error(msg)
        return ResolutionResult(
            status="unresolved", reason=str(exc), model_dir=None, model_sdf=None
        )


def _safe_parse_model_sdf(
    model_sdf_path: Path,
    model_key: str,
    warnings: List[str],
    errors: List[str],
) -> Optional[dict]:
    """Parse model.sdf; record failures without raising."""
    try:
        return parse_model_sdf(model_sdf_path)
    except FileNotFoundError as exc:
        msg = f"model.sdf not found for {model_key!r}: {exc}"
        warnings.append(msg)
        logger.warning(msg)
    except ValueError as exc:
        msg = f"Invalid model.sdf for {model_key!r}: {exc}"
        errors.append(msg)
        logger.error(msg)
    except Exception as exc:
        msg = f"Unexpected error parsing model.sdf for {model_key!r}: {exc}"
        errors.append(msg)
        logger.error(msg)
    return None


def _safe_extract_collisions(
    links: list,
    model_dir: Optional[Path],
    model_key: str,
    warnings: List[str],
    errors: List[str],
) -> list:
    """Extract collisions; record failures without raising."""
    try:
        return extract_collisions(links, model_dir)
    except Exception as exc:
        msg = f"Error extracting collisions for {model_key!r}: {exc}"
        errors.append(msg)
        logger.error(msg)
        return []


# ---------------------------------------------------------------------------
# Helper: static classification
# ---------------------------------------------------------------------------

def _classify_static(
    instance_dicts: List[dict],
    model_data: Optional[dict],
    model_key: str,
) -> StaticClassification:
    """
    Determine static/dynamic state for an external model.

    Precedence:
    1. Explicit <static> override in at least one instance.
       If all instances with an override agree → use that value.
       If they disagree → UNKNOWN with explanation.
    2. Explicit <static> in model.sdf.
    3. UNKNOWN.
    """
    # Collect instance-level overrides
    overrides = [
        d["static_override"]
        for d in instance_dicts
        if d.get("static_override") is not None
    ]

    if overrides:
        unique_overrides = set(overrides)
        if len(unique_overrides) == 1:
            value = overrides[0]
            return StaticClassification(
                static_state="STATIC" if value else "DYNAMIC",
                source=f"include static override (all {len(overrides)} overriding instances agree)",
            )
        else:
            return StaticClassification(
                static_state="UNKNOWN",
                source=f"conflicting include static overrides among instances of {model_key}",
            )

    # Fall back to model.sdf
    if model_data is not None and model_data.get("static_value") is not None:
        value = model_data["static_value"]
        return StaticClassification(
            static_state="STATIC" if value else "DYNAMIC",
            source="model.sdf <static> element",
        )

    return StaticClassification(
        static_state="UNKNOWN",
        source="no <static> element found in includes or model.sdf",
    )


# ---------------------------------------------------------------------------
# Helper: direct world model builder
# ---------------------------------------------------------------------------

def _build_direct_world_model(
    raw: dict,
    warnings: List[str],
    errors: List[str],
) -> DirectWorldModel:
    """Convert a raw parsed direct-model dict into a DirectWorldModel."""
    name: str = raw["name"]
    pose: list = raw["pose"]
    static_value: Optional[bool] = raw.get("static_value")

    # Static classification
    if static_value is not None:
        classification = StaticClassification(
            static_state="STATIC" if static_value else "DYNAMIC",
            source="world <model> <static> element",
        )
    else:
        classification = StaticClassification(
            static_state="UNKNOWN",
            source="no <static> element in direct world model",
        )

    # Collisions
    collisions = _safe_extract_collisions(
        raw.get("links", []), None, name, warnings, errors
    )

    # Navigation relevance classification
    nav_relevance = _classify_nav_relevance(name, collisions, classification)

    return DirectWorldModel(
        name=name,
        pose=pose,
        static_classification=classification,
        collisions=collisions,
        nav_relevance=nav_relevance,
    )


def _classify_nav_relevance(
    model_name: str,
    collisions: list,
    classification: StaticClassification,
) -> NavRelevance:
    """
    Conservatively classify navigation relevance.

    - If any collision is a PLANE → TRAVERSABLE (likely ground surface).
    - If STATIC and has non-plane collisions → OBSTACLE.
    - Otherwise → UNKNOWN.

    Does NOT rely on name-based heuristics for classification.
    """
    from .models import PlaneGeometry

    if not collisions:
        return NavRelevance(label="UNKNOWN", reason="no collision geometry found")

    geom_types = [type(c.geometry) for c in collisions]

    if all(g is PlaneGeometry for g in geom_types):
        return NavRelevance(
            label="TRAVERSABLE",
            reason="all collisions are plane geometry (likely floor/ground surface)",
        )

    if classification.static_state == "STATIC":
        return NavRelevance(
            label="OBSTACLE",
            reason="static model with non-plane collision geometry",
        )

    return NavRelevance(
        label="UNKNOWN",
        reason="insufficient information for unambiguous navigation classification",
    )


# ---------------------------------------------------------------------------
# Helper: URI → display key
# ---------------------------------------------------------------------------

def _uri_to_key(uri: str) -> str:
    """
    Extract a short human-readable key from a Fuel URI.

    E.g. 'https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf' → 'shelf'
    """
    uri = uri.strip()
    # Take the last non-empty path component
    parts = [p for p in uri.split("/") if p]
    return parts[-1] if parts else uri
