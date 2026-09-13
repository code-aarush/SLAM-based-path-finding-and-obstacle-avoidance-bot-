"""
robot_navigation.world_analyzer
--------------------------------
Gazebo SDF World Analyzer — Module 1 of the autonomous navigation pipeline.

Public API
----------
    analyze_world(world_path, fuel_cache) -> WorldAnalysis
    print_report(analysis) -> None
    write_json_report(analysis, output_dir) -> Path
"""

from .analyzer import analyze_world
from .models import WorldAnalysis
from .reporter import print_report, write_json_report

__all__ = [
    "analyze_world",
    "WorldAnalysis",
    "print_report",
    "write_json_report",
]
