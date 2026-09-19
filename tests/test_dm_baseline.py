"""DM outputs must not change while the kinematic model is refactored for two variants.

The hashes below were captured at 248ca4d (before the refactor). If a DM output changes
on purpose, recapture them in the same commit and say why."""

import hashlib
from pathlib import Path

from src.rebot_b601 import kinematics

ASSETS = Path(__file__).resolve().parent.parent / "src" / "rebot_b601" / "assets"

PRIMITIVES_SHA = "fda17b269b403d30b4dfafd377b0540840a4fb504d2a2cf2144d007397c1aa5a"
NONE_SHA = "86b193d24dbb937d619fd37ecabd37ecc92005ba628727901daf5cda9297d9ab"
MESHES_URDF_SHA = "eeee3e00c5234a9de8f3b488374c0d00a3d26746427e83a8fd30271c07664080"
MESH_SHAS = {  # meshes/<link>.stl -> sha256 of the bytes served in meshes mode
    "meshes/base_link.stl": "d3401304571743d57931895e92322864a7a495ddc5b55cfee8df93fac5c49210",
    "meshes/link1.stl": "e3ec037f5f94578a2857e2ee743c56aec46be1b1724f82b4650a882489be0526",
    "meshes/link2.stl": "2de1c416baa54ca9de406bf5031e012f2ff16d8046710f82e5fdf4777cb17c52",
    "meshes/link3.stl": "3aa13afaabf0e70428d3418814cbd7e1353ef71ae386a38b33c9dcdae02fc036",
    "meshes/link4.stl": "3bbf8dd25d1c9e91940db4eb909021e3c6fb1b919ae2d0b28b438a3ac8d84c23",
    "meshes/link5.stl": "9f7a0ea8d6695e7a7cc964a2c5a7482f323f2f5d6a3134ccd6765aff83b89b52",
    "meshes/link6.stl": "304225f81169354870c0c1e2410c045600fd7401f29320fb255bd8f0abad54d2",
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
    assert _sha(kinematics.arm_kinematics("primitives")[1]) == PRIMITIVES_SHA
    assert _sha(kinematics.arm_kinematics("none")[1]) == NONE_SHA
    _, urdf, meshes = kinematics.arm_kinematics("meshes")
    assert _sha(urdf) == MESHES_URDF_SHA
    assert {k: _sha(v.mesh) for k, v in meshes.items()} == MESH_SHAS


def test_dm_glb_bytes_are_unchanged():
    assert {p.name: p.stat().st_size for p in (ASSETS / "models").glob("*.glb")} == GLB_SIZES
