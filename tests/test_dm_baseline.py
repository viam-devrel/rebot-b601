"""DM outputs must not change while the kinematic model is refactored for two variants.

The arm hashes were captured at 248ca4d and the gripper hashes at 8ff1d41, each before the
refactor it guards. If a DM output changes on purpose, recapture in the same commit and say why.

The three arm payload hashes were re-pinned at the frame split: the served arm chain now stops at
the tool mount (link6), so the mount plate's link and its fixed joint are no longer in the URDF.
The per-link mesh bytes (MESH_SHAS) and the gripper hashes were untouched by that trim.

Note the two naming conventions: mesh keys are gripper_base/left_finger/right_finger while
geometry labels are gripper_base/finger_left_link/finger_right_link."""

import hashlib
from pathlib import Path

import pytest

from src.rebot_b601 import kinematics, spatial

ASSETS = Path(__file__).resolve().parent.parent / "src" / "rebot_b601" / "assets"

PRIMITIVES_SHA = "d044ebfb974253e1c2eacbe39afeff5b57d9bbe255800cfe39768cbc88dc4fdc"
NONE_SHA = "89e40b45c9b85350a256f8645cc7bd624768b38f968c08ed1802de060dbfe30b"
MESHES_URDF_SHA = "335f8b28b4c25eb00b08e6e8b73bb4c728f4c0a2380549dae3a763fbd32c0256"
MESH_SHAS = {  # meshes/<link>.stl -> sha256 of the bytes served in meshes mode
    "meshes/base_link.stl": "d3401304571743d57931895e92322864a7a495ddc5b55cfee8df93fac5c49210",
    "meshes/link1.stl": "e3ec037f5f94578a2857e2ee743c56aec46be1b1724f82b4650a882489be0526",
    "meshes/link2.stl": "2de1c416baa54ca9de406bf5031e012f2ff16d8046710f82e5fdf4777cb17c52",
    "meshes/link3.stl": "3aa13afaabf0e70428d3418814cbd7e1353ef71ae386a38b33c9dcdae02fc036",
    "meshes/link4.stl": "3bbf8dd25d1c9e91940db4eb909021e3c6fb1b919ae2d0b28b438a3ac8d84c23",
    "meshes/link5.stl": "9f7a0ea8d6695e7a7cc964a2c5a7482f323f2f5d6a3134ccd6765aff83b89b52",
    "meshes/link6.stl": "304225f81169354870c0c1e2410c045600fd7401f29320fb255bd8f0abad54d2",
}
GRIPPER_PRIMITIVES_SHA = "101e86cdadee6984996e3c64907cd25f3ac58665f75d5ce051321c8db3f0c2c5"
GRIPPER_NONE_SHA = "b74001660e106099eb218c32405fc0021a1759bf26478868ae668977a5522900"
GRIPPER_MESHES_URDF_SHA = "a1ed74575dd04d29df8f150de7588c682943edefeddfa405017cf3383c22f965"
GRIPPER_MESH_SHAS = {  # meshes/<part>.stl -> sha256 of the bytes served in meshes mode
    "meshes/gripper_base.stl": "eeb2b04d80cdb408c86da4628256e680ae0984756646b11a41ff683ed16cc0cc",
    "meshes/left_finger.stl": "80c73899bfb5e1068a7fd90021881f96f8d97b91160b87769a4e1650478942ab",
    "meshes/right_finger.stl": "a8db240aeb42a513c01cf990a1c314ec45367ff5dd77d30fc55778dac36912c2",
}
GLB_SIZES = {  # models/<link>.glb -> bytes
    "base_link.glb": 200036,
    "end_link.glb": 199328,
    "finger_left_link.glb": 90044,
    "finger_right_link.glb": 90072,
    "link1.glb": 123604,
    "link2.glb": 201004,
    "link3.glb": 201232,
    "link4.glb": 200316,
    "link5.glb": 199672,
    "link6.glb": 198840,
}


def _sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def test_dm_kinematics_payloads_are_unchanged():
    assert _sha(kinematics.arm_kinematics(spatial.MODELS["dm"], "primitives")[1]) == PRIMITIVES_SHA
    assert _sha(kinematics.arm_kinematics(spatial.MODELS["dm"], "none")[1]) == NONE_SHA
    _, urdf, meshes = kinematics.arm_kinematics(spatial.MODELS["dm"], "meshes")
    assert _sha(urdf) == MESHES_URDF_SHA
    assert {k: _sha(v.mesh) for k, v in meshes.items()} == MESH_SHAS


def test_dm_glb_bytes_are_unchanged():
    assert {p.name: p.stat().st_size for p in (ASSETS / "models").glob("*.glb")} == GLB_SIZES


def test_dm_gripper_payloads_are_unchanged():
    assert _sha(kinematics.gripper_kinematics(spatial.MODELS["dm"], "primitives")[1]) == GRIPPER_PRIMITIVES_SHA
    assert _sha(kinematics.gripper_kinematics(spatial.MODELS["dm"], "none")[1]) == GRIPPER_NONE_SHA
    _, urdf, meshes = kinematics.gripper_kinematics(spatial.MODELS["dm"], "meshes")
    assert _sha(urdf) == GRIPPER_MESHES_URDF_SHA
    assert {k: _sha(v.mesh) for k, v in meshes.items()} == GRIPPER_MESH_SHAS


def test_dm_gripper_geometries_are_unchanged():
    geos = kinematics.gripper_geometries(spatial.MODELS["dm"], 0.02)
    assert [g.label for g in geos] == ["gripper_base", "finger_left_link", "finger_right_link"]
    assert geos[1].center.y == pytest.approx(geos[0].center.y + 20.0 + 13.7, abs=0.2)
