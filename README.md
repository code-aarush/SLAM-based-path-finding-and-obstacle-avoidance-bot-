# Autonomous Robot Navigation System

Autonomous Robot Navigation System using ROS 2 Jazzy, Gazebo Harmonic, Tugbot, occupancy grids, A* path planning, and dynamic obstacle replanning.

---

## Module 1 — Gazebo SDF World Analyzer

### Purpose

Given a Gazebo SDF world file, the analyzer automatically discovers:

- All model instances (included and inline)
- Unique model types and instance grouping
- Local Fuel cache model definitions
- Collision geometry for every resolved model
- Static / dynamic classification with evidence
- Navigation relevance for inline world models

Output: a structured `world_analysis.json` and a human-readable terminal report.

---

### Architecture

```
world_analyzer/
├── __init__.py          Public API surface
├── __main__.py          CLI entry point (argparse)
├── models.py            Domain dataclasses (WorldAnalysis, ExternalModel, …)
├── sdf_parser.py        XML parsing — world & model SDF
├── asset_resolver.py    Fuel URI → local cache resolution
├── collision_parser.py  Geometry extraction (box/cylinder/sphere/plane/mesh)
├── analyzer.py          Pipeline orchestrator
└── reporter.py          Terminal + JSON output
```

**Pipeline:**

```
SDF World File
  └─ sdf_parser       → includes, direct models (raw XML)
  └─ asset_resolver   → local Fuel paths per unique model URI
  └─ sdf_parser       → model.sdf per resolved model
  └─ collision_parser → typed CollisionEntry objects
  └─ analyzer         → WorldAnalysis
  └─ reporter         → terminal report + world_analysis.json
```

---

### Installation

No extra dependencies — uses Python standard library only.

Requires Python 3.10+ (uses `match`-compatible union types).

```bash
# From repo root
cd robot_navigation
```

---

### Usage

```bash
# src-layout — set PYTHONPATH to expose the package
PYTHONPATH=src python3 -m robot_navigation.world_analyzer WORLD_FILE [OPTIONS]
```

#### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--fuel-cache PATH` | `~/.gz/fuel` | Local Gazebo Fuel cache root |
| `--output-dir PATH` | `./outputs` | Directory for `world_analysis.json` |
| `--verbose` | off | Enable DEBUG logging |

---

### CLI Examples

```bash
# Basic run against the warehouse world
PYTHONPATH=src python3 -m robot_navigation.world_analyzer \
    ../Worlds_models/tugbot_warehouse.sdf \
    --fuel-cache ~/.gz/fuel \
    --output-dir ./data/outputs

# Verbose mode
PYTHONPATH=src python3 -m robot_navigation.world_analyzer \
    ../Worlds_models/tugbot_warehouse.sdf --verbose
```

---

### Example Output

**Terminal Summary (warehouse world):**

```
WORLD
  Name        : world_demo
  Source      : /…/tugbot_warehouse.sdf

SUMMARY
  Included instances      : 26
  Direct world models     : 1
  Total model entities    : 27
  Unique external types   : 7
  Resolved models         : 7
  Unresolved models       : 0
```

**JSON structure (`world_analysis.json`):**

```json
{
  "metadata": {
    "analyzer_version": "1.0.0",
    "generated_at": "…",
    "source_world_file": "…"
  },
  "world": { "name": "world_demo", "source_path": "…" },
  "summary": { … },
  "external_models": [
    {
      "model_key": "shelf",
      "uri": "https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf",
      "resolution": { "status": "resolved", "model_sdf": "…" },
      "classification": { "static_state": "STATIC", "source": "model.sdf <static> element" },
      "instances": [
        { "name": "shelf_0", "pose": [-4.415, 2.307, 0, 0, 0, 0], "static_override": null }
      ],
      "collisions": [
        {
          "link_name": "shelf_base",
          "collision_name": "shelf_chassi_collision",
          "geometry": { "type": "box", "size": [3.6, 0.6, 1.8] }
        }
      ]
    }
  ],
  "direct_world_models": [ … ],
  "warnings": [],
  "errors": []
}
```

---

### Fuel Cache Resolution

The resolver maps Fuel URIs to local directories using this algorithm:

1. Parse URI: `https://<hostname>/<api_ver>/<owner>/models/<model>`
2. Locate `<fuel_cache>/<hostname>/` (exact match, lowercase)
3. Find `<owner>/` subdirectory (case-insensitive)
4. Find `models/<model_name>/` subdirectory (case-insensitive)
5. Select the highest-numbered version directory
6. Verify `model.sdf` exists

Supports multiple Fuel domains (`fuel.ignitionrobotics.org`, `fuel.gazebosim.org`, etc.) without hardcoding.

---

### Supported Collision Geometry

| Type | SDF Tag | Extracted Fields |
|------|---------|-----------------|
| Box | `<box>` | `size: [x, y, z]` |
| Cylinder | `<cylinder>` | `radius`, `length` |
| Sphere | `<sphere>` | `radius` |
| Plane | `<plane>` | `normal: [x,y,z]`, `size: [x,y]` |
| Mesh | `<mesh>` | `uri`, `scale`, `resolved_path`, `file_exists` |
| Unknown | any other | `raw_tag` (logged as warning) |

Mesh URIs are resolved:
- **Relative** (e.g. `meshes/foo.stl`) → resolved against model directory
- **Absolute** (`/path/…`) → checked for existence
- **Remote** (`http://…`) → stored as-is, `file_exists=false`

Vertex-level geometry processing is explicitly out of scope.

---

### Pose Hierarchy

All pose levels are preserved separately — no silent merging:

| Level | Field |
|-------|-------|
| World instance `<pose>` | `instance.pose` |
| Model-level `<pose>` in model.sdf | available via `parse_model_sdf()` |
| Link `<pose>` | `collision.link_pose` |
| Collision `<pose>` | `collision.collision_pose` |

Future modules compute final transforms from these four levels.

---

### Known Limitations

- Mesh geometry is **discovered and referenced, not rasterized**. Vertices are not read.
- `model://` mesh URIs require a model path registry not yet implemented.
- Nested `<include>` inside model SDFs is not recursed.
- Remote Fuel URIs (not cached locally) produce `unresolved` status with a descriptive reason.
- `<static>` conflicts across instances of the same model type result in `UNKNOWN` classification.

---

### Running Tests

```bash
cd robot_navigation
python3 -m pytest tests/test_world_analyzer.py -v
```

45 tests, no external dependencies, no Gazebo required.

---

### Connection to Future Modules

`world_analysis.json` is the **stable interface** between Module 1 and all downstream modules:

```
world_analysis.json
  → Geometry Processor     reads external_models[].collisions + poses
  → Occupancy Grid         inflates obstacle footprints into 2D grid
  → A* Global Planner      plans over the occupancy grid
  → LaserScan Detector     detects dynamic obstacles at runtime
  → Reactive Replanner     triggers A* re-runs on obstacle change
```

Downstream modules never need to parse SDF or access Gazebo.
