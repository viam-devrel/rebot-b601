"""DM outputs must not change while the kinematic model is refactored for two variants.

The arm hashes were captured at 248ca4d and the gripper hashes at 8ff1d41, each before the
refactor it guards. If a DM output changes on purpose, recapture in the same commit and say why.

The three arm payload hashes were re-pinned at the frame split: the served arm chain now stops at
the tool mount (link6), so the mount plate's link and its fixed joint are no longer in the URDF.
The per-link mesh bytes (MESH_SHAS) were untouched by that trim. The three gripper payload
hashes were then re-pinned when the gripper's model was rerooted at the tool mount: its URDF
gained a tool_mount root link and the fixed tool_mount_joint carrying the arm's end_joint
transform, so the gripper now starts where the served arm chain stops. The gripper mesh bytes
(GRIPPER_MESH_SHAS) and the GLB sizes did not change.

MESH_SHAS, GRIPPER_MESH_SHAS and GLB_SIZES were then re-pinned when the shell-by-shell
decimation fix (b94bb14, RS only at the time) was applied to DM and its assets rebuilt.
The old DM meshes were shards -- base_link.stl was 3062 faces in 2409 disconnected pieces
whose largest was 7 triangles -- because the builder decimated without welding vertices
first. Every DM mesh byte therefore moved, and the visual GLBs grew to spend the byte
budget a wrong bytes-per-face estimate had left unused. The six URDF payload hashes did
not move: the URDF text names mesh files, it does not embed their bytes.

The three gripper payload hashes were re-pinned once more when the gripper's model was folded to
a single chain. It served two leaves -- finger_left_link and a static finger_right_link -- and
viam-server's ParseConfig rejects a URDF model with more than one end effector, so the gripper
never loaded into the frame system at all. finger_right_link is gone from the model and its travel
envelope is unioned into gripper_base's box. GRIPPER_MESH_SHAS shrank with it: gripper_base now
carries that union box in every mode (a mesh cannot also cover the right finger), so meshes mode
serves only the left finger's STL. The surviving mesh's bytes did not change, and neither did the
arm payloads or the GLBs.

The three gripper payload hashes moved a third time when the gripper's link frames were
squared up with the arm's. Every joint in the model now carries translation only and each
vendor rotation -- the mount plate's, then the finger's -- rides on its link's <collision
origin rpy> instead, so the boxes sit exactly where they did (to within _fmt's six significant
digits) while gripper_base and finger_left_link finally report the tool mount's orientation
rather than an arbitrary one. Only the URDF text changed: GRIPPER_MESH_SHAS, the arm payloads,
MESH_SHAS and GLB_SIZES all held.

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
    "meshes/base_link.stl": "5ab2bb444beb1ea85600f25f815705dc57349f0058f57c90c7723ff94ef7fb43",
    "meshes/link1.stl": "fce0c245cb8d91c4d1c4f1ba23bc6593ec0f7dc7930ddeee71f85277b4855f12",
    "meshes/link2.stl": "be660d201c790977423751b018cdc4eb61d3dace81ed497fe863b3ca8cb91c89",
    "meshes/link3.stl": "d124c5af81ff46dc42bf5b3a3c451aa3df0a90dfc0cf219914626b966c5881bc",
    "meshes/link4.stl": "4652d4fe2bd940019f0e806bcbf608ff0d73466f1f2dd7999fb77db9cf53ec5b",
    "meshes/link5.stl": "07baaabd37acf17b3441fddabf8a50d71039bd66cff9110c2ba72844f6234149",
    "meshes/link6.stl": "ef30182a5d6117990641d8cf72b20256ca419a23e28e0af3565573448488b4b7",
}
GRIPPER_PRIMITIVES_SHA = "05a6c13859d24a684f7eaea35f73bea6c2e1e5e94c6ffa24b90f8c64a8bcfacb"
GRIPPER_NONE_SHA = "1e7db0c6ac4016b3fd83f413ce6a50f7b7a5ecbc683ad39b29e36e08581024a8"
GRIPPER_MESHES_URDF_SHA = "09ec8c76ba33f87f0580189c39c82f93399779d9f799a61a67345f76cc90cd04"
GRIPPER_MESH_SHAS = {  # meshes/<part>.stl -> sha256 of the bytes served in meshes mode
    "meshes/left_finger.stl": "d24a8c610c8f342f9aa4a528416f52e5a291c90e52c8d6c0a032cbd804430557",
}
GLB_SIZES = {  # models/<link>.glb -> bytes
    "base_link.glb": 305660,
    "end_link.glb": 215080,
    "finger_left_link.glb": 90044,
    "finger_right_link.glb": 90072,
    "link1.glb": 123320,
    "link2.glb": 300952,
    "link3.glb": 298376,
    "link4.glb": 297676,
    "link5.glb": 297872,
    "link6.glb": 298208,
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
    # The mount transform maps the gripper's local +y onto global -y, so the jaw separation
    # keeps its magnitude and flips its sign. Same axis, negated, not a different axis.
    assert geos[1].center.y == pytest.approx(geos[0].center.y - (20.0 + 13.7), abs=0.2)
