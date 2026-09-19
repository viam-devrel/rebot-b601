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
        stl=[
            "base_link",
            "link1",
            "link2",
            "link3",
            "link4",
            "link5",
            "link6",
            "gripper_end",
            "gripper_left",
            "gripper_right",
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


# ---- decimation must thin the shells, not dissolve them
#
# These CAD parts are assemblies of many separate closed shells. Decimated as one mesh
# with a proportional face budget the small shells collapsed into loose triangles: the
# shipped rs link2.stl was 3062 faces in 2762 pieces whose largest was 6 triangles, and
# dm base_link.stl 3062 faces in 2409 pieces whose largest was 7 -- dust the 3D view
# draws as floating shards and a motion planner routes straight through.


def _shells(mesh):
    return [len(c.faces) for c in mesh.split(only_watertight=False, repair=False)]


def _assert_solid(shells, what):
    assert min(shells) >= 4, f"{what}: shell of {min(shells)} face(s) cannot bound a volume"
    assert len(shells) * 10 <= sum(shells), f"{what}: {sum(shells)} faces in {len(shells)} shells is rubble"


@pytest.mark.parametrize("variant,link", _pairs("stl"))
def test_collision_mesh_is_not_shards(variant, link):
    m = trimesh.load(str(variant["dir"] / "meshes" / f"{link}.stl"), force="mesh")
    _assert_solid(_shells(m), f"{link}.stl")


@pytest.mark.parametrize("variant,link", _pairs("glb"))
def test_visual_mesh_is_not_shards(variant, link):
    scene = trimesh.load(str(variant["dir"] / "models" / f"{link}.glb"), force="scene")
    for name, geom in scene.geometry.items():
        _assert_solid(_shells(geom), f"{link}.glb/{name}")


def test_small_shells_survive_decimation():
    """A budget too small to share out must keep the little shells whole, not shred them."""
    from tools.build_assets import decimate_to_faces

    boxes = [trimesh.creation.box((0.01, 0.01, 0.01)).apply_translation((i * 0.1, 0, 0)) for i in range(8)]
    mesh = trimesh.util.concatenate([trimesh.creation.icosphere(subdivisions=4)] + boxes)
    out, _method = decimate_to_faces(mesh, 400)
    shells = _shells(out)
    assert len(shells) == 9, f"lost shells: {shells}"
    assert sorted(shells)[:8] == [12] * 8, f"the 12-face boxes were decimated: {sorted(shells)}"
    assert sum(shells) <= 440
