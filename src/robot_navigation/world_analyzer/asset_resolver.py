"""
asset_resolver.py
-----------------
Resolve Gazebo Fuel URIs to local file-system paths.

Supported URI forms
-------------------
    https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse

Local cache layout (Fuel convention)
-------------------------------------
    <fuel_cache>/
    └── <hostname>/          e.g. fuel.ignitionrobotics.org
        └── <owner_lower>/   e.g. movai
            └── models/
                └── <model_lower>/   e.g. shelf
                    └── <version>/   e.g. 1, 2, 3 …
                        └── model.sdf

Resolution algorithm
--------------------
1. Parse the URI to extract: hostname, owner, model name.
2. Map hostname → local cache sub-directory.
3. Try owner name case-insensitively (listdir + lower-compare).
4. Try model name case-insensitively.
5. Select the highest numeric version directory.
6. Verify model.sdf exists.
7. Return a ResolutionResult.

Design principles
-----------------
- No hardcoded model or owner names.
- No assumption of a single Fuel domain.
- Failures are isolated and reported; they do not propagate exceptions.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

from .models import ResolutionResult

logger = logging.getLogger(__name__)

DEFAULT_FUEL_CACHE = Path.home() / ".gz" / "fuel"


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def resolve_uri(uri: str, fuel_cache: Path = DEFAULT_FUEL_CACHE) -> ResolutionResult:
    """
    Attempt to resolve *uri* to a local Fuel cache directory.

    Parameters
    ----------
    uri :
        A Gazebo Fuel HTTP(S) URI such as
        ``https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf``.
    fuel_cache :
        Root of the local Fuel cache (default ``~/.gz/fuel``).

    Returns
    -------
    ResolutionResult
        Always returns a result; never raises.
    """
    try:
        return _resolve(uri.strip(), fuel_cache)
    except Exception as exc:  # pragma: no cover – defensive catch-all
        logger.error("Unexpected error resolving URI %r: %s", uri, exc)
        return ResolutionResult(
            status="unresolved",
            reason=f"Unexpected error: {exc}",
            model_dir=None,
            model_sdf=None,
        )


# ---------------------------------------------------------------------------
# Internal resolution logic
# ---------------------------------------------------------------------------

def _resolve(uri: str, fuel_cache: Path) -> ResolutionResult:
    """Core resolution logic (may raise; callers should catch)."""
    parsed = urlparse(uri)

    # ------------------------------------------------------------------ #
    # 1. Validate URI scheme                                               #
    # ------------------------------------------------------------------ #
    if parsed.scheme not in ("http", "https"):
        return ResolutionResult(
            status="unresolved",
            reason=f"Unsupported URI scheme: {parsed.scheme!r} (expected http/https)",
            model_dir=None,
            model_sdf=None,
        )

    hostname = parsed.netloc.lower()  # e.g. "fuel.ignitionrobotics.org"

    # ------------------------------------------------------------------ #
    # 2. Extract path components: /<api_ver>/<owner>/models/<model_name>  #
    # ------------------------------------------------------------------ #
    path_parts = [p for p in parsed.path.split("/") if p]
    # Expected: ['1.0', 'MovAi', 'models', 'shelf']
    if len(path_parts) < 4 or path_parts[2].lower() != "models":
        return ResolutionResult(
            status="unresolved",
            reason=(
                f"URI path does not match expected Fuel pattern "
                f"'/<api_ver>/<owner>/models/<model_name>': {parsed.path!r}"
            ),
            model_dir=None,
            model_sdf=None,
        )

    owner_from_uri = path_parts[1]     # original case, e.g. "MovAi"
    model_from_uri = path_parts[3]     # original case, e.g. "shelf"

    # ------------------------------------------------------------------ #
    # 3. Locate hostname directory in cache                               #
    # ------------------------------------------------------------------ #
    if not fuel_cache.is_dir():
        return ResolutionResult(
            status="unresolved",
            reason=f"Fuel cache directory does not exist: {fuel_cache}",
            model_dir=None,
            model_sdf=None,
        )

    host_dir = _find_subdir_ci(fuel_cache, hostname)
    if host_dir is None:
        return ResolutionResult(
            status="unresolved",
            reason=(
                f"No cache directory for hostname {hostname!r} "
                f"under {fuel_cache}"
            ),
            model_dir=None,
            model_sdf=None,
        )

    # ------------------------------------------------------------------ #
    # 4. Locate owner directory (case-insensitive)                        #
    # ------------------------------------------------------------------ #
    owner_dir = _find_subdir_ci(host_dir, owner_from_uri)
    if owner_dir is None:
        return ResolutionResult(
            status="unresolved",
            reason=(
                f"Owner {owner_from_uri!r} not found under {host_dir} "
                f"(case-insensitive)"
            ),
            model_dir=None,
            model_sdf=None,
        )

    models_dir = owner_dir / "models"
    if not models_dir.is_dir():
        return ResolutionResult(
            status="unresolved",
            reason=f"No 'models' subdirectory under {owner_dir}",
            model_dir=None,
            model_sdf=None,
        )

    # ------------------------------------------------------------------ #
    # 5. Locate model directory (case-insensitive)                        #
    # ------------------------------------------------------------------ #
    model_dir = _find_subdir_ci(models_dir, model_from_uri)
    if model_dir is None:
        return ResolutionResult(
            status="unresolved",
            reason=(
                f"Model {model_from_uri!r} not found under {models_dir} "
                f"(case-insensitive)"
            ),
            model_dir=None,
            model_sdf=None,
        )

    # ------------------------------------------------------------------ #
    # 6. Select the latest numeric version                                #
    # ------------------------------------------------------------------ #
    version_dir = _latest_version_dir(model_dir)
    if version_dir is None:
        return ResolutionResult(
            status="unresolved",
            reason=f"No numeric version directories found under {model_dir}",
            model_dir=str(model_dir),
            model_sdf=None,
        )

    # ------------------------------------------------------------------ #
    # 7. Verify model.sdf                                                 #
    # ------------------------------------------------------------------ #
    model_sdf = version_dir / "model.sdf"
    if not model_sdf.is_file():
        return ResolutionResult(
            status="unresolved",
            reason=f"model.sdf not found in {version_dir}",
            model_dir=str(version_dir),
            model_sdf=None,
        )

    logger.debug("Resolved %r → %s", uri, version_dir)
    return ResolutionResult(
        status="resolved",
        reason=None,
        model_dir=str(version_dir),
        model_sdf=str(model_sdf),
    )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_subdir_ci(parent: Path, name: str) -> Optional[Path]:
    """
    Find a direct subdirectory of *parent* whose name matches *name*
    case-insensitively.  Returns the first match or None.
    """
    if not parent.is_dir():
        return None
    target = name.lower()
    try:
        for child in parent.iterdir():
            if child.is_dir() and child.name.lower() == target:
                return child
    except PermissionError as exc:
        logger.warning("Cannot list %s: %s", parent, exc)
    return None


def _latest_version_dir(model_dir: Path) -> Optional[Path]:
    """
    Return the subdirectory of *model_dir* with the highest integer name.

    Only considers directories whose names are pure integers (e.g. "1", "2").
    """
    best: Optional[Path] = None
    best_ver = -1

    try:
        for child in model_dir.iterdir():
            if child.is_dir() and child.name.isdigit():
                ver = int(child.name)
                if ver > best_ver:
                    best_ver = ver
                    best = child
    except PermissionError as exc:
        logger.warning("Cannot list %s: %s", model_dir, exc)

    return best


def normalize_uri(uri: str) -> str:
    """
    Return a canonical form of *uri* for use as a grouping key.

    Strips whitespace and lowercases scheme + hostname; preserves path case.
    """
    uri = uri.strip()
    parsed = urlparse(uri)
    # Normalise scheme and host to lowercase, keep path as-is
    normalized = parsed._replace(
        scheme=parsed.scheme.lower(),
        netloc=parsed.netloc.lower(),
    )
    return normalized.geturl()
