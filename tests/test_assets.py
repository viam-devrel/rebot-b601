"""Sanity checks for the built mesh assets under src/rebot_b601/assets/.

Run: .venv/bin/pytest tests/test_assets.py
Rebuild the assets with: .venv/bin/python tools/build_assets.py
"""

import json
from pathlib import Path

import pytest
import trimesh

ASSETS = Path(__file__).resolve().parent.parent / "src" / "rebot_b601" / "assets"

STL_LINKS = [
    "base_link",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "gripper_base",
    "left_finger",
    "right_finger",
]
GLB_LINKS = [
    "base_link",
    "link1",
    "link2",
    "link3",
    "link4",
    "link5",
    "link6",
    "end_link",
    "finger_left_link",
    "finger_right_link",
]
STL_CAP = 150 * 1024
GLB_CAP = 300 * 1024


def test_attribution_and_primitives_exist():
    assert (ASSETS / "ATTRIBUTION.md").is_file()
    assert (ASSETS / "primitives.json").is_file()


@pytest.mark.parametrize("link", STL_LINKS)
def test_collision_stl(link):
    p = ASSETS / "meshes" / f"{link}.stl"
    assert p.is_file(), p
    assert p.stat().st_size < STL_CAP, f"{p.name} is {p.stat().st_size} bytes"
    with p.open("rb") as f:
        assert f.read(5) != b"solid", f"{p.name} is not binary STL"
    m = trimesh.load(str(p), force="mesh")
    assert isinstance(m, trimesh.Trimesh)
    assert len(m.faces) > 0
    assert float(m.extents.max()) < 1.0, "collision mesh does not look like metres"


@pytest.mark.parametrize("link", GLB_LINKS)
def test_visual_glb(link):
    p = ASSETS / "models" / f"{link}.glb"
    assert p.is_file(), p
    assert p.stat().st_size < GLB_CAP, f"{p.name} is {p.stat().st_size} bytes"
    with p.open("rb") as f:
        assert f.read(4) == b"glTF"
    scene = trimesh.load(str(p), force="scene")
    assert len(scene.geometry) > 0
    assert all(len(g.faces) > 0 for g in scene.geometry.values())


def test_primitives_json():
    d = json.loads((ASSETS / "primitives.json").read_text())
    assert d["units"] == "m"
    assert d["source_commit"] == "b0acdcfc47843de16a9f018c6ab1de1d31649fdc"
    links = d["links"]
    assert set(links) == set(STL_LINKS)
    for name in STL_LINKS:
        e = links[name]
        assert isinstance(e["center"], list) and len(e["center"]) == 3
        assert isinstance(e["size"], list) and len(e["size"]) == 3
        assert all(isinstance(v, (int, float)) for v in e["center"] + e["size"])
        assert all(s > 0 for s in e["size"]), (name, e["size"])
        assert max(e["size"]) < 1.0, "size does not look like metres"
        tri = d["triangles"][name]
        assert tri["original"] >= tri["decimated"] > 0
