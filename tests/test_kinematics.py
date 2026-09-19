import math
import xml.etree.ElementTree as ET

import pytest
from viam.proto.common import KinematicsFileFormat

from src.rebot_b601 import kinematics, spatial

DM, RS = spatial.MODELS["dm"], spatial.MODELS["rs"]
BOTH = [pytest.param(DM, id="dm"), pytest.param(RS, id="rs")]
# Five cases need assets/rs, which Task 6 builds; strict xfail makes Task 6 remove the marks or fail.
RS_NEEDS_ASSETS = pytest.param(RS, id="rs", marks=pytest.mark.xfail(strict=True, reason="assets/rs is built in Task 6"))
BOTH_RS_XFAIL = [pytest.param(DM, id="dm"), RS_NEEDS_ASSETS]


def _links_with_collision(urdf: bytes):
    root = ET.fromstring(urdf)
    return {l.get("name") for l in root.findall("link") if l.find("collision") is not None}


@pytest.mark.parametrize("model", BOTH_RS_XFAIL)
def test_primitives_mode_adds_box_per_arm_link(model):
    fmt, data = kinematics.arm_kinematics(model, "primitives")
    assert fmt == KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
    assert _links_with_collision(data) == set(model.arm_links)
    root = ET.fromstring(data)
    assert root.get("name") == f"rebot_b601_{model.name}"
    box = root.find("link[@name='link2']/collision/geometry/box")
    assert box is not None and len(box.get("size").split()) == 3


@pytest.mark.parametrize("model", BOTH_RS_XFAIL)
def test_meshes_mode_returns_mesh_map_keyed_by_filename(model):
    fmt, data, meshes = kinematics.arm_kinematics(model, "meshes")
    root = ET.fromstring(data)
    assert {m.get("filename") for m in root.iter("mesh")} == set(meshes)
    for m in meshes.values():
        assert m.content_type == "stl" and len(m.mesh) > 1000


@pytest.mark.parametrize("model", BOTH)
def test_none_mode_has_no_collision(model):
    assert _links_with_collision(kinematics.arm_kinematics(model, "none")[1]) == set()


@pytest.mark.parametrize("model", BOTH_RS_XFAIL)
def test_gripper_geometry_is_opt_in_on_arm(model):
    _, data = kinematics.arm_kinematics(model, "primitives", include_gripper_geometry=True)
    assert "end_link" in _links_with_collision(data)
    root = ET.fromstring(data)
    assert root.find("link[@name='end_link']/collision/geometry/box") is not None


@pytest.mark.parametrize("model", BOTH)
def test_kinematics_preserve_joint_chain(model):
    _, data = kinematics.arm_kinematics(model, "primitives")
    joints = [j.get("name") for j in ET.fromstring(data).findall("joint")]
    assert joints == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "end_joint"]


def test_rs_served_urdf_has_no_prismatic_joint():
    _, data = kinematics.arm_kinematics(RS, "none")
    assert all(j.get("type") != "prismatic" for j in ET.fromstring(data).findall("joint"))


@pytest.mark.parametrize(
    "model,tol",
    [
        pytest.param(DM, 0.5, id="dm"),
        pytest.param(RS, 2.2, id="rs", marks=pytest.mark.xfail(strict=True, reason="assets/rs is built in Task 6")),
    ],
)
def test_arm_geometries_follow_fk(model, tol):
    geos = kinematics.arm_geometries(model, [0.0] * 6)
    assert [g.label for g in geos] == model.arm_links
    assert geos[0].center.z > 0
    a = geos[2].center
    b = kinematics.arm_geometries(model, [90.0, 0, 0, 0, 0, 0])[2].center
    # joint1's axis is offset from the base z-axis (DM 0.084 mm, RS 1.045 mm);
    # 2 * offset bounds the radius drift under a 90 deg yaw
    assert math.isclose(math.hypot(a.x, a.y), math.hypot(b.x, b.y), abs_tol=tol)
    assert not math.isclose(a.x, b.x, abs_tol=1.0)


def test_dm_base_box_dims_unchanged():
    base = kinematics.arm_geometries(DM, [0.0] * 6)[0]
    assert math.isclose(base.box.dims_mm.x, 140.0, abs_tol=1.0)


def test_gripper_urdf_has_one_prismatic_dof():
    fmt, data = kinematics.gripper_kinematics("primitives")
    root = ET.fromstring(data)
    assert [j.get("type") for j in root.findall("joint")].count("prismatic") == 1
    assert _links_with_collision(data) == {"gripper_base", "finger_left_link", "finger_right_link"}
    assert len(kinematics.gripper_geometries(0.02)) == 3


@pytest.mark.parametrize("model", BOTH_RS_XFAIL)
def test_3d_models_glb(model):
    models = kinematics.arm_3d_models(model, include_gripper=True)
    assert set(model.arm_links) <= set(models)
    assert all(m.content_type == "model/gltf-binary" and m.mesh[:4] == b"glTF" for m in models.values())


def test_gravity_torque_sign_and_magnitude():
    # The zero pose is the folded "sit-down" pose: the upper arm points back
    # and the forearm forward, so the elbow (joint3) carries the most load and
    # the shoulder (joint2) is partly counterbalanced.
    g = spatial.gravity_torques([0.0] * 6)
    assert abs(g[2]) == max(abs(v) for v in g) > 5.0
    assert abs(g[1]) > 0 and abs(g[3]) > 0
    # Base yaw joint never sees gravity torque with vertical gravity.
    assert math.isclose(g[0], 0.0, abs_tol=1e-9)
    # Wrist roll/pitch axes are aligned with gravity at this pose.
    assert math.isclose(g[4], 0.0, abs_tol=1e-3) and math.isclose(g[5], 0.0, abs_tol=1e-3)  # URDF uses 1.5708 / 3.1415
    # A payload at the end effector (forward of the elbow) increases the elbow torque.
    assert abs(spatial.gravity_torques([0.0] * 6, extra_payload_kg=1.0)[2]) > abs(g[2])
    # Pointing the arm straight up (shoulder at -90) removes most of the load from the elbow.
    assert abs(spatial.gravity_torques([0.0, math.radians(-90.0), 0.0, 0.0, 0.0, 0.0])[2]) < abs(g[2])
