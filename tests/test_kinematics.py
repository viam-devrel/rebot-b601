import math
import xml.etree.ElementTree as ET

import pytest
from viam.proto.common import KinematicsFileFormat

from src.rebot_b601 import kinematics, spatial

DM, RS = spatial.MODELS["dm"], spatial.MODELS["rs"]
BOTH = [pytest.param(DM, id="dm"), pytest.param(RS, id="rs")]


def _links_with_collision(urdf: bytes):
    root = ET.fromstring(urdf)
    return {l.get("name") for l in root.findall("link") if l.find("collision") is not None}


@pytest.mark.parametrize("model", BOTH)
def test_primitives_mode_adds_box_per_arm_link(model):
    fmt, data = kinematics.arm_kinematics(model, "primitives")
    assert fmt == KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
    assert _links_with_collision(data) == set(model.arm_links)
    root = ET.fromstring(data)
    assert root.get("name") == f"rebot_b601_{model.name}"
    box = root.find("link[@name='link2']/collision/geometry/box")
    assert box is not None and len(box.get("size").split()) == 3


@pytest.mark.parametrize("model", BOTH)
def test_meshes_mode_returns_mesh_map_keyed_by_filename(model):
    fmt, data, meshes = kinematics.arm_kinematics(model, "meshes")
    root = ET.fromstring(data)
    assert {m.get("filename") for m in root.iter("mesh")} == set(meshes)
    for m in meshes.values():
        assert m.content_type == "stl" and len(m.mesh) > 1000


@pytest.mark.parametrize("model", BOTH)
def test_none_mode_has_no_collision(model):
    assert _links_with_collision(kinematics.arm_kinematics(model, "none")[1]) == set()


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
        pytest.param(RS, 2.2, id="rs"),
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


@pytest.mark.parametrize("model", BOTH)
def test_gripper_urdf_has_one_prismatic_dof(model):
    fmt, data = kinematics.gripper_kinematics(model, "primitives")
    root = ET.fromstring(data)
    assert [j.get("type") for j in root.findall("joint")].count("prismatic") == 1
    # Link names are identical across variants so frame-system names stay stable.
    assert _links_with_collision(data) == {"gripper_base", "finger_left_link", "finger_right_link"}
    limit = root.find("joint[@name='finger_left']/limit")
    assert float(limit.get("upper")) == pytest.approx(model.gripper.travel_m)
    assert len(kinematics.gripper_geometries(model, 0.02)) == 3


def test_rs_gripper_fingers_are_set_back_from_the_mount():
    rs = spatial.MODELS["rs"]
    root = ET.fromstring(kinematics.gripper_kinematics(rs, "primitives")[1])
    origin = root.find("joint[@name='finger_left']/origin")
    assert float(origin.get("xyz").split()[0]) == pytest.approx(-0.041939)
    assert origin.get("rpy").split()[0] == "1.5708"


@pytest.mark.parametrize("model", BOTH)
def test_gripper_fingers_travel_in_opposite_directions(model):
    """The jaw must open. A shared travel sign moves both fingers the same way, which a
    distance check can miss entirely."""
    closed = kinematics.gripper_geometries(model, 0.0)
    wide = kinematics.gripper_geometries(model, model.gripper.travel_m)

    def displacement(i):
        a, b = closed[i].center, wide[i].center
        return (b.x - a.x, b.y - a.y, b.z - a.z)

    left, right = displacement(1), displacement(2)
    expected_mm = 1000.0 * model.gripper.travel_m
    assert math.dist((0, 0, 0), left) == pytest.approx(expected_mm, abs=0.01), model.name
    assert math.dist((0, 0, 0), right) == pytest.approx(expected_mm, abs=0.01), model.name
    # Opposite directions is the whole point: the bug makes this dot product positive.
    assert sum(a * b for a, b in zip(left, right)) < 0, (model.name, left, right)


@pytest.mark.parametrize("model", BOTH)
def test_3d_models_glb(model):
    models = kinematics.arm_3d_models(model)
    assert set(model.arm_links) <= set(models)
    assert all(m.content_type == "model/gltf-binary" and m.mesh[:4] == b"glTF" for m in models.values())


def test_gravity_torque_sign_and_magnitude():
    # The zero pose is the folded "sit-down" pose: the upper arm points back
    # and the forearm forward, so the elbow (joint3) carries the most load and
    # the shoulder (joint2) is partly counterbalanced.
    g = DM.gravity_torques([0.0] * 6)
    assert abs(g[2]) == max(abs(v) for v in g) > 5.0
    assert abs(g[1]) > 0 and abs(g[3]) > 0
    # Base yaw joint never sees gravity torque with vertical gravity.
    assert math.isclose(g[0], 0.0, abs_tol=1e-9)
    # Wrist roll/pitch axes are aligned with gravity at this pose.
    assert math.isclose(g[4], 0.0, abs_tol=1e-3) and math.isclose(g[5], 0.0, abs_tol=1e-3)  # URDF uses 1.5708 / 3.1415
    # A payload at the end effector (forward of the elbow) increases the elbow torque.
    assert abs(DM.gravity_torques([0.0] * 6, extra_payload_kg=1.0)[2]) > abs(g[2])
    # Pointing the arm straight up (shoulder at -90) removes most of the load from the elbow.
    assert abs(DM.gravity_torques([0.0, math.radians(-90.0), 0.0, 0.0, 0.0, 0.0])[2]) < abs(g[2])
