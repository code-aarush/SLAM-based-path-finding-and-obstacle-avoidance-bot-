"""
tests/test_world_analyzer.py
Pytest tests for the world_analyzer module.
All tests use synthetic in-memory/temp SDF files — no Gazebo required.
"""
import json
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

# ── src-layout path fix ────────────────────────────────────────────────────
import sys
SRC = Path(__file__).parent.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from robot_navigation.world_analyzer.sdf_parser import (
    _parse_pose, _parse_static, parse_includes, parse_direct_models,
    load_world_xml, parse_model_sdf,
)
from robot_navigation.world_analyzer.asset_resolver import (
    resolve_uri, normalize_uri, _find_subdir_ci, _latest_version_dir,
)
from robot_navigation.world_analyzer.collision_parser import extract_collisions
from robot_navigation.world_analyzer.models import (
    BoxGeometry, CylinderGeometry, SphereGeometry, PlaneGeometry,
    MeshGeometry, UnknownGeometry, WorldAnalysis, WorldInfo, Summary,
)
from robot_navigation.world_analyzer.analyzer import analyze_world


# ══════════════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════════════

def _make_world_file(tmp_path: Path, body: str) -> Path:
    """Write a minimal SDF world file and return its path."""
    content = textwrap.dedent(f"""\
        <?xml version='1.0'?>
        <sdf version='1.7'>
          <world name='test_world'>
            {body}
          </world>
        </sdf>
    """)
    p = tmp_path / "test_world.sdf"
    p.write_text(content)
    return p


def _make_model_sdf(tmp_path: Path, name: str, body: str, static: str = "1") -> Path:
    """Write a minimal model.sdf and return its path."""
    content = textwrap.dedent(f"""\
        <?xml version='1.0'?>
        <sdf version='1.7'>
          <model name='{name}'>
            <static>{static}</static>
            {body}
          </model>
        </sdf>
    """)
    p = tmp_path / "model.sdf"
    p.write_text(content)
    return p


def _raw_collision(geom_xml: str, coll_name="col", link_name="lnk",
                   link_pose=None, coll_pose=None):
    """Build a raw collision dict as sdf_parser would produce."""
    geom_el = ET.fromstring(f"<geometry>{geom_xml}</geometry>")
    return [{
        "link_name": link_name,
        "link_pose": link_pose or [0]*6,
        "raw_collisions": [{
            "collision_name": coll_name,
            "collision_pose": coll_pose or [0]*6,
            "geometry_el": geom_el,
        }]
    }]


# ══════════════════════════════════════════════════════════════════════════════
# 1. Pose parsing
# ══════════════════════════════════════════════════════════════════════════════

class TestPoseParsing:
    def test_full_pose(self):
        el = ET.fromstring("<pose>1 2 3 0.1 0.2 0.3</pose>")
        assert _parse_pose(el) == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]

    def test_none_returns_zeros(self):
        assert _parse_pose(None) == [0.0]*6

    def test_partial_pose_padded(self):
        el = ET.fromstring("<pose>1 2 3</pose>")
        result = _parse_pose(el)
        assert len(result) == 6
        assert result[:3] == [1.0, 2.0, 3.0]
        assert result[3:] == [0.0, 0.0, 0.0]

    def test_extra_values_truncated(self):
        el = ET.fromstring("<pose>1 2 3 4 5 6 7 8</pose>")
        assert len(_parse_pose(el)) == 6

    def test_negative_zero_pose(self):
        el = ET.fromstring("<pose>5.60144 -0.690987 0 0 -0 0</pose>")
        result = _parse_pose(el)
        assert result[0] == pytest.approx(5.60144)
        assert result[1] == pytest.approx(-0.690987)


# ══════════════════════════════════════════════════════════════════════════════
# 2. World include parsing
# ══════════════════════════════════════════════════════════════════════════════

class TestIncludeParsing:
    def test_basic_include(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <include>
              <uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri>
              <name>shelf_0</name>
              <pose>1 2 3 0 0 0</pose>
            </include>
        """)
        world_el, _ = load_world_xml(f)
        includes = parse_includes(world_el)
        assert len(includes) == 1
        inc = includes[0]
        assert inc["name"] == "shelf_0"
        assert inc["uri"] == "https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf"
        assert inc["pose"] == [1.0, 2.0, 3.0, 0.0, 0.0, 0.0]
        assert inc["static_override"] is None

    def test_static_override_true(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <include>
              <uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/pallet_box_mobile</uri>
              <name>pallet_box</name>
              <pose>0 0 0 0 0 0</pose>
              <static>true</static>
            </include>
        """)
        world_el, _ = load_world_xml(f)
        includes = parse_includes(world_el)
        assert includes[0]["static_override"] is True

    def test_missing_uri_is_none(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <include>
              <name>mystery</name>
              <pose>0 0 0 0 0 0</pose>
            </include>
        """)
        world_el, _ = load_world_xml(f)
        includes = parse_includes(world_el)
        assert includes[0]["uri"] is None

    def test_multiple_includes(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <include><uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri><name>s0</name><pose>0 0 0 0 0 0</pose></include>
            <include><uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri><name>s1</name><pose>1 0 0 0 0 0</pose></include>
            <include><uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri><name>s2</name><pose>2 0 0 0 0 0</pose></include>
        """)
        world_el, _ = load_world_xml(f)
        assert len(parse_includes(world_el)) == 3


# ══════════════════════════════════════════════════════════════════════════════
# 3. Duplicate model grouping
# ══════════════════════════════════════════════════════════════════════════════

class TestModelGrouping:
    def test_shelf_instances_grouped(self, tmp_path):
        body = "\n".join(f"""
            <include>
              <uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri>
              <name>shelf_{i}</name><pose>{i} 0 0 0 0 0</pose>
            </include>""" for i in range(5))
        f = _make_world_file(tmp_path, body)
        result = analyze_world(f, fuel_cache=Path("/nonexistent_cache"))
        assert result.summary.unique_external_model_types == 1
        assert result.summary.total_included_instances == 5
        shelf = result.external_models[0]
        assert len(shelf.instances) == 5

    def test_two_types_two_groups(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <include><uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri><name>s0</name><pose>0 0 0 0 0 0</pose></include>
            <include><uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf_big</uri><name>sb0</name><pose>1 0 0 0 0 0</pose></include>
            <include><uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri><name>s1</name><pose>2 0 0 0 0 0</pose></include>
        """)
        result = analyze_world(f, fuel_cache=Path("/nonexistent_cache"))
        assert result.summary.unique_external_model_types == 2
        assert result.summary.total_included_instances == 3


# ══════════════════════════════════════════════════════════════════════════════
# 4. Fuel URI resolution
# ══════════════════════════════════════════════════════════════════════════════

class TestFuelResolution:
    def _make_cache(self, tmp_path: Path, owner: str, model: str, version: int = 1) -> Path:
        """Create a fake Fuel cache with a model.sdf."""
        model_dir = tmp_path / "fuel.ignitionrobotics.org" / owner / "models" / model / str(version)
        model_dir.mkdir(parents=True)
        (model_dir / "model.sdf").write_text("<sdf><model name='x'></model></sdf>")
        return tmp_path

    def test_resolves_known_model(self, tmp_path):
        cache = self._make_cache(tmp_path, "movai", "shelf")
        r = resolve_uri("https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf", cache)
        assert r.status == "resolved"
        assert r.model_sdf is not None
        assert Path(r.model_sdf).exists()

    def test_case_insensitive_owner(self, tmp_path):
        cache = self._make_cache(tmp_path, "movai", "shelf")
        r = resolve_uri("https://fuel.ignitionrobotics.org/1.0/MOVAI/models/shelf", cache)
        assert r.status == "resolved"

    def test_case_insensitive_model(self, tmp_path):
        cache = self._make_cache(tmp_path, "movai", "shelf")
        r = resolve_uri("https://fuel.ignitionrobotics.org/1.0/movai/models/Shelf", cache)
        assert r.status == "resolved"

    def test_selects_latest_version(self, tmp_path):
        for v in (1, 2, 5):
            d = tmp_path / "fuel.ignitionrobotics.org" / "movai" / "models" / "shelf" / str(v)
            d.mkdir(parents=True)
            (d / "model.sdf").write_text("<sdf><model name='x'></model></sdf>")
        r = resolve_uri("https://fuel.ignitionrobotics.org/1.0/movai/models/shelf", tmp_path)
        assert r.status == "resolved"
        assert "/5/" in r.model_sdf

    def test_missing_cache_dir(self, tmp_path):
        r = resolve_uri("https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf",
                        tmp_path / "nonexistent")
        assert r.status == "unresolved"

    def test_missing_model(self, tmp_path):
        self._make_cache(tmp_path, "movai", "shelf")
        r = resolve_uri("https://fuel.ignitionrobotics.org/1.0/movai/models/NOMODEL", tmp_path)
        assert r.status == "unresolved"

    def test_invalid_scheme(self, tmp_path):
        r = resolve_uri("ftp://fuel.ignitionrobotics.org/1.0/movai/models/shelf", tmp_path)
        assert r.status == "unresolved"

    def test_normalize_uri_lowercases_host(self):
        n = normalize_uri("  https://Fuel.IgnitionRobotics.ORG/1.0/MovAi/models/shelf  ")
        assert "fuel.ignitionrobotics.org" in n


# ══════════════════════════════════════════════════════════════════════════════
# 5. Geometry extraction
# ══════════════════════════════════════════════════════════════════════════════

class TestBoxExtraction:
    def test_box_size(self):
        links = _raw_collision("<box><size>3.6 0.6 1.8</size></box>")
        result = extract_collisions(links)
        assert len(result) == 1
        geom = result[0].geometry
        assert isinstance(geom, BoxGeometry)
        assert geom.size == pytest.approx([3.6, 0.6, 1.8])

    def test_box_missing_size_returns_unknown(self):
        links = _raw_collision("<box></box>")
        result = extract_collisions(links)
        assert len(result) == 1
        assert isinstance(result[0].geometry, UnknownGeometry)


class TestCylinderExtraction:
    def test_cylinder(self):
        links = _raw_collision("<cylinder><radius>0.5</radius><length>1.0</length></cylinder>")
        result = extract_collisions(links)
        geom = result[0].geometry
        assert isinstance(geom, CylinderGeometry)
        assert geom.radius == pytest.approx(0.5)
        assert geom.length == pytest.approx(1.0)


class TestSphereExtraction:
    def test_sphere(self):
        links = _raw_collision("<sphere><radius>0.25</radius></sphere>")
        result = extract_collisions(links)
        geom = result[0].geometry
        assert isinstance(geom, SphereGeometry)
        assert geom.radius == pytest.approx(0.25)


class TestPlaneExtraction:
    def test_plane(self):
        links = _raw_collision("<plane><normal>0 0 1</normal><size>1 1</size></plane>")
        result = extract_collisions(links)
        geom = result[0].geometry
        assert isinstance(geom, PlaneGeometry)
        assert geom.normal == pytest.approx([0.0, 0.0, 1.0])
        assert geom.size == pytest.approx([1.0, 1.0])

    def test_plane_defaults_on_missing_elements(self):
        links = _raw_collision("<plane></plane>")
        result = extract_collisions(links)
        geom = result[0].geometry
        assert isinstance(geom, PlaneGeometry)
        assert geom.normal == [0.0, 0.0, 1.0]
        assert geom.size == [1.0, 1.0]


class TestMeshExtraction:
    def test_mesh_relative_uri(self, tmp_path):
        mesh_file = tmp_path / "meshes" / "foo.stl"
        mesh_file.parent.mkdir()
        mesh_file.touch()
        links = _raw_collision("<mesh><uri>meshes/foo.stl</uri><scale>1 1 1</scale></mesh>")
        result = extract_collisions(links, model_dir=tmp_path)
        geom = result[0].geometry
        assert isinstance(geom, MeshGeometry)
        assert geom.uri == "meshes/foo.stl"
        assert geom.file_exists is True
        assert geom.scale == pytest.approx([1.0, 1.0, 1.0])

    def test_mesh_missing_file(self, tmp_path):
        links = _raw_collision("<mesh><uri>meshes/ghost.stl</uri></mesh>")
        result = extract_collisions(links, model_dir=tmp_path)
        geom = result[0].geometry
        assert isinstance(geom, MeshGeometry)
        assert geom.file_exists is False

    def test_mesh_remote_uri(self):
        links = _raw_collision("<mesh><uri>https://example.com/mesh.dae</uri></mesh>")
        result = extract_collisions(links)
        geom = result[0].geometry
        assert isinstance(geom, MeshGeometry)
        assert geom.resolved_path == "https://example.com/mesh.dae"
        assert geom.file_exists is False

    def test_mesh_missing_uri_raises_unknown(self):
        links = _raw_collision("<mesh></mesh>")
        result = extract_collisions(links)
        assert isinstance(result[0].geometry, UnknownGeometry)


class TestMissingGeometry:
    def test_empty_geometry_element(self):
        links = _raw_collision("")
        result = extract_collisions(links)
        assert len(result) == 0

    def test_unsupported_geometry_type(self):
        links = _raw_collision("<heightmap><uri>foo.png</uri></heightmap>")
        result = extract_collisions(links)
        assert isinstance(result[0].geometry, UnknownGeometry)
        assert result[0].geometry.raw_tag == "heightmap"

    def test_no_geometry_el_skipped(self):
        links = [{
            "link_name": "link",
            "link_pose": [0]*6,
            "raw_collisions": [{"collision_name": "c", "collision_pose": [0]*6, "geometry_el": None}]
        }]
        result = extract_collisions(links)
        assert result == []


# ══════════════════════════════════════════════════════════════════════════════
# 6. Direct world model parsing
# ══════════════════════════════════════════════════════════════════════════════

class TestDirectWorldModel:
    def test_ground_plane_parsed(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <model name='ground_plane'>
              <static>true</static>
              <link name='link'>
                <collision name='collision'>
                  <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
                </collision>
              </link>
              <pose>0 0 0 0 0 0</pose>
            </model>
        """)
        result = analyze_world(f, fuel_cache=Path("/nonexistent"))
        assert len(result.direct_world_models) == 1
        dwm = result.direct_world_models[0]
        assert dwm.name == "ground_plane"
        assert dwm.static_classification.static_state == "STATIC"
        assert len(dwm.collisions) == 1
        assert isinstance(dwm.collisions[0].geometry, PlaneGeometry)

    def test_traversable_classification(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <model name='floor'>
              <static>true</static>
              <link name='link'>
                <collision name='c'>
                  <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
                </collision>
              </link>
              <pose>0 0 0 0 0 0</pose>
            </model>
        """)
        result = analyze_world(f, fuel_cache=Path("/nonexistent"))
        dwm = result.direct_world_models[0]
        assert dwm.nav_relevance.label == "TRAVERSABLE"


# ══════════════════════════════════════════════════════════════════════════════
# 7. Static override precedence
# ══════════════════════════════════════════════════════════════════════════════

class TestStaticOverride:
    def _analyze_with_include(self, tmp_path, static_tag: str) -> object:
        f = _make_world_file(tmp_path, f"""
            <include>
              <uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/pallet_box_mobile</uri>
              <name>pallet_box</name>
              <pose>0 0 0 0 0 0</pose>
              {static_tag}
            </include>
        """)
        result = analyze_world(f, fuel_cache=Path("/nonexistent"))
        return result.external_models[0].classification

    def test_include_static_true_wins(self, tmp_path):
        cls = self._analyze_with_include(tmp_path, "<static>true</static>")
        assert cls.static_state == "STATIC"
        assert "include" in cls.source

    def test_include_static_false_wins(self, tmp_path):
        cls = self._analyze_with_include(tmp_path, "<static>false</static>")
        assert cls.static_state == "DYNAMIC"

    def test_no_override_unknown_without_model(self, tmp_path):
        cls = self._analyze_with_include(tmp_path, "")
        # No Fuel cache → can't read model.sdf → UNKNOWN
        assert cls.static_state == "UNKNOWN"

    def test_model_sdf_static_used_when_no_override(self, tmp_path):
        # Build a fake cache with a STATIC model.sdf
        model_dir = (tmp_path / "cache" / "fuel.ignitionrobotics.org" /
                     "movai" / "models" / "mymodel" / "1")
        model_dir.mkdir(parents=True)
        sdf = "<sdf version='1.7'><model name='mymodel'><static>1</static></model></sdf>"
        (model_dir / "model.sdf").write_text(sdf)

        f = _make_world_file(tmp_path, """
            <include>
              <uri>https://fuel.ignitionrobotics.org/1.0/movai/models/mymodel</uri>
              <name>m0</name><pose>0 0 0 0 0 0</pose>
            </include>
        """)
        result = analyze_world(f, fuel_cache=tmp_path / "cache")
        cls = result.external_models[0].classification
        assert cls.static_state == "STATIC"
        assert "model.sdf" in cls.source


# ══════════════════════════════════════════════════════════════════════════════
# 8. JSON serialization
# ══════════════════════════════════════════════════════════════════════════════

class TestJsonSerialization:
    def test_round_trip(self, tmp_path):
        f = _make_world_file(tmp_path, """
            <model name='ground_plane'>
              <static>true</static>
              <link name='link'>
                <collision name='col'>
                  <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
                </collision>
              </link>
              <pose>0 0 0 0 0 0</pose>
            </model>
            <include>
              <uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri>
              <name>shelf_0</name><pose>1 2 3 0 0 0</pose>
            </include>
        """)
        result = analyze_world(f, fuel_cache=Path("/nonexistent"))
        d = result.to_dict()

        # Top-level keys
        for key in ("metadata", "world", "summary", "external_models",
                    "direct_world_models", "warnings", "errors"):
            assert key in d

        # Must be JSON-serializable
        serialized = json.dumps(d)
        reloaded = json.loads(serialized)
        assert reloaded["world"]["name"] == "test_world"
        assert reloaded["summary"]["total_direct_world_models"] == 1

    def test_write_json_report(self, tmp_path):
        from robot_navigation.world_analyzer.reporter import write_json_report
        f = _make_world_file(tmp_path, "")
        result = analyze_world(f, fuel_cache=Path("/nonexistent"))
        out_dir = tmp_path / "outputs"
        out_file = write_json_report(result, out_dir)
        assert out_file.exists()
        data = json.loads(out_file.read_text())
        assert "metadata" in data

    def test_metadata_fields(self, tmp_path):
        f = _make_world_file(tmp_path, "")
        result = analyze_world(f, fuel_cache=Path("/nonexistent"))
        d = result.to_dict()
        assert "analyzer_version" in d["metadata"]
        assert "generated_at" in d["metadata"]
        assert "source_world_file" in d["metadata"]


# ══════════════════════════════════════════════════════════════════════════════
# 9. Error handling / robustness
# ══════════════════════════════════════════════════════════════════════════════

class TestErrorHandling:
    def test_missing_world_file_returns_analysis_with_error(self, tmp_path):
        result = analyze_world(tmp_path / "ghost.sdf")
        assert len(result.errors) > 0
        assert result.summary.total_included_instances == 0

    def test_invalid_xml_returns_error(self, tmp_path):
        bad = tmp_path / "bad.sdf"
        bad.write_text("<sdf><world name='w'><UNCLOSED")
        result = analyze_world(bad)
        assert len(result.errors) > 0

    def test_one_bad_model_does_not_block_others(self, tmp_path):
        # Mix one URI-less include with a valid one
        f = _make_world_file(tmp_path, """
            <include>
              <name>ghost</name><pose>0 0 0 0 0 0</pose>
            </include>
            <include>
              <uri>https://fuel.ignitionrobotics.org/1.0/MovAi/models/shelf</uri>
              <name>shelf_0</name><pose>1 0 0 0 0 0</pose>
            </include>
        """)
        result = analyze_world(f, fuel_cache=Path("/nonexistent"))
        # shelf should still be present
        assert result.summary.unique_external_model_types == 1
        # warning about missing URI
        assert any("URI" in w or "uri" in w for w in result.warnings)

    def test_multiple_collisions_per_link(self, tmp_path):
        links = [
            {
                "link_name": "base",
                "link_pose": [0]*6,
                "raw_collisions": [
                    {"collision_name": "c1", "collision_pose": [0]*6,
                     "geometry_el": ET.fromstring("<geometry><box><size>1 1 1</size></box></geometry>")},
                    {"collision_name": "c2", "collision_pose": [0]*6,
                     "geometry_el": ET.fromstring("<geometry><sphere><radius>0.5</radius></sphere></geometry>")},
                ]
            }
        ]
        result = extract_collisions(links)
        assert len(result) == 2
        assert isinstance(result[0].geometry, BoxGeometry)
        assert isinstance(result[1].geometry, SphereGeometry)
