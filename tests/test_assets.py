"""Sanity checks for the built mesh assets under src/rebot_b601/assets/.

Run: .venv/bin/pytest tests/test_assets.py
Rebuild the assets with: .venv/bin/python tools/build_assets.py --variant {dm,rs}
"""

import json
from pathlib import Path

import pytest
import trimesh

ASSETS = Path(__file__).resolve().parent.parent / "src" / "rebot_b601" / "assets"

VARIANTS = {
    "dm": dict(
        dir=ASSETS,
        commit="b0acdcfc47843de16a9f018c6ab1de1d31649fdc",
        stl=[
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
        ],
        glb=[
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
        ],
    ),
    "rs": dict(
        dir=ASSETS / "rs",
        commit="76512eab38ba54f11830e7cdfcba68e11629f902",
        stl=["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper_end",
             "gripper_left", "gripper_right"],
        glb=["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "end_link",
             "finger_left_link", "finger_right_link"],
    ),
}
STL_CAP = 150 * 1024
GLB_CAP = 300 * 1024


def _pairs(kind):
    return [pytest.param(v, link, id=f"{name}-{link}") for name, v in VARIANTS.items() for link in v[kind]]


@pytest.mark.parametrize("variant", VARIANTS.values(), ids=list(VARIANTS))
def test_attribution_and_primitives_exist(variant):
    assert (variant["dir"] / "ATTRIBUTION.md").is_file()
    assert (variant["dir"] / "primitives.json").is_file()


@pytest.mark.parametrize("variant,link", _pairs("stl"))
def test_collision_stl(variant, link):
    p = variant["dir"] / "meshes" / f"{link}.stl"
    assert p.is_file(), p
    assert p.stat().st_size < STL_CAP, f"{p.name} is {p.stat().st_size} bytes"
    with p.open("rb") as f:
        assert f.read(5) != b"solid", f"{p.name} is not binary STL"
    m = trimesh.load(str(p), force="mesh")
    assert isinstance(m, trimesh.Trimesh)
    assert len(m.faces) > 0
    assert float(m.extents.max()) < 1.0, "collision mesh does not look like metres"


@pytest.mark.parametrize("variant,link", _pairs("glb"))
def test_visual_glb(variant, link):
    p = variant["dir"] / "models" / f"{link}.glb"
    assert p.is_file(), p
    assert p.stat().st_size < GLB_CAP, f"{p.name} is {p.stat().st_size} bytes"
    with p.open("rb") as f:
        assert f.read(4) == b"glTF"
    scene = trimesh.load(str(p), force="scene")
    assert len(scene.geometry) > 0
    assert all(len(g.faces) > 0 for g in scene.geometry.values())


@pytest.mark.parametrize("variant,link", _pairs("stl"))
def test_primitives_json(variant, link):
    d = json.loads((variant["dir"] / "primitives.json").read_text())
    assert d["units"] == "m"
    assert d["source_commit"] == variant["commit"]
    assert set(d["links"]) == set(variant["stl"])
    e = d["links"][link]
    assert isinstance(e["center"], list) and len(e["center"]) == 3
    assert isinstance(e["size"], list) and len(e["size"]) == 3
    assert all(isinstance(v, (int, float)) for v in e["center"] + e["size"])
    assert all(s > 0 for s in e["size"]), (link, e["size"])
    assert max(e["size"]) < 1.0, "size does not look like metres"
    tri = d["triangles"][link]
    assert tri["original"] >= tri["decimated"] > 0


def test_rs_glb_parts_carry_material_colours():
    d = json.loads((ASSETS / "rs" / "primitives.json").read_text())
    parts = d["visual_models"]["link2"]["parts"]
    assert parts and all(p["color"] != "#8A8A8A" and str(p["urdf_material"]).startswith("rs_") for p in parts.values())
