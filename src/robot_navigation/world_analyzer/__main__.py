"""
__main__.py
-----------
CLI entry point for the Gazebo World Analyzer.

Usage
-----
    python -m world_analyzer WORLD_FILE [--fuel-cache PATH] [--output-dir PATH] [--verbose]

Or, from the repo root (src-layout):
    python -m robot_navigation.world_analyzer WORLD_FILE ...
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="world_analyzer",
        description=(
            "Gazebo SDF World Analyzer — inspect world models, resolve Fuel URIs, "
            "extract collision geometry, and produce a machine-readable JSON report."
        ),
    )
    parser.add_argument(
        "world_file",
        metavar="WORLD_FILE",
        type=Path,
        help="Path to the Gazebo SDF world file to analyse.",
    )
    parser.add_argument(
        "--fuel-cache",
        metavar="PATH",
        type=Path,
        default=Path.home() / ".gz" / "fuel",
        help="Root of the local Gazebo Fuel model cache (default: ~/.gz/fuel).",
    )
    parser.add_argument(
        "--output-dir",
        metavar="PATH",
        type=Path,
        default=Path("./outputs"),
        help="Directory where world_analysis.json will be written (default: ./outputs).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable DEBUG-level logging.",
    )
    return parser


def _configure_logging(verbose: bool) -> None:
    level = logging.DEBUG if verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)-8s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stderr,
    )


def main(argv=None) -> int:
    """Entry point; returns an exit code."""
    parser = _build_parser()
    args = parser.parse_args(argv)

    _configure_logging(args.verbose)
    logger = logging.getLogger("world_analyzer")

    world_path: Path = args.world_file.expanduser().resolve()
    fuel_cache: Path = args.fuel_cache.expanduser().resolve()
    output_dir: Path = args.output_dir.expanduser().resolve()

    logger.info("World file   : %s", world_path)
    logger.info("Fuel cache   : %s", fuel_cache)
    logger.info("Output dir   : %s", output_dir)

    # Lazy import keeps startup fast and avoids circular-import issues at module
    # level when running tests that import individual sub-modules directly.
    from .analyzer import analyze_world
    from .reporter import print_report, write_json_report

    # ---- Run analysis ----
    analysis = analyze_world(world_path=world_path, fuel_cache=fuel_cache)

    # ---- Terminal report ----
    print_report(analysis)

    # ---- JSON report ----
    json_path = write_json_report(analysis, output_dir)
    print(f"\nJSON report saved to: {json_path}")

    # Exit non-zero if there were hard errors
    return 1 if analysis.errors else 0


if __name__ == "__main__":
    sys.exit(main())
