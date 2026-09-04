import math
import xml.etree.ElementTree as ET

from viam.proto.common import KinematicsFileFormat

from src.rebot_b601 import kinematics, spatial


def _links_with_collision(urdf: bytes):
    root = ET.fromstring(urdf)
    return {l.get("name") for l in root.findall("link") if l.find("collision") is not None}


def test_primitives_mode_adds_box_per_arm_link():
    fmt, data = kinematics.arm_kinematics("primitives")
    assert fmt == KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
    assert _links_with_collision(data) == set(kinematics.ARM_LINKS)
    root = ET.fromstring(data)
    box = root.find("link[@name='link2']/collision/geometry/box")
    assert box is not None and len(box.get("size").split()) == 3


def test_meshes_mode_returns_mesh_map_keyed_by_filename():
    fmt, data, meshes = kinematics.arm_kinematics("meshes")
    root = ET.fromstring(data)
    filenames = {m.get("filename") for m in root.iter("mesh")}
    assert filenames == set(meshes)
    for m in meshes.values():
        assert m.content_type == "stl" and len(m.mesh) > 1000


def test_none_mode_has_no_collision():
    fmt, data = kinematics.arm_kinematics("none")
    assert _links_with_collision(data) == set()


def test_gripper_geometry_is_opt_in_on_arm():
    _, data = kinematics.arm_kinematics("primitives", include_gripper_geometry=True)
    assert "end_link" in _links_with_collision(data)


def test_kinematics_preserve_joint_chain():
    _, data = kinematics.arm_kinematics("primitives")
    root = ET.fromstring(data)
    joints = [j.get("name") for j in root.findall("joint")]
    assert joints == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "end_joint"]


def test_arm_geometries_follow_fk():
    geos = kinematics.arm_geometries([0.0] * 6)
    assert [g.label for g in geos] == kinematics.ARM_LINKS
    # base box sits above the origin, dims in mm
    base = geos[0]
    assert math.isclose(base.box.dims_mm.x, 140.0, abs_tol=1.0)
    assert base.center.z > 0
    # rotating joint1 by 90 deg swings link2's box centre around z
    a = kinematics.arm_geometries([0.0] * 6)[2].center
    b = kinematics.arm_geometries([90.0, 0, 0, 0, 0, 0])[2].center
    assert math.isclose(math.hypot(a.x, a.y), math.hypot(b.x, b.y), abs_tol=0.5)  # joint1 axis is offset 0.08 mm
    assert not math.isclose(a.x, b.x, abs_tol=1.0)


def test_gripper_urdf_has_one_prismatic_dof():
    fmt, data = kinematics.gripper_kinematics("primitives")
    root = ET.fromstring(data)
    types = [j.get("type") for j in root.findall("joint")]
    assert types.count("prismatic") == 1
    assert _links_with_collision(data) == {"gripper_base", "finger_left_link", "finger_right_link"}
    geos = kinematics.gripper_geometries(0.02)
    assert len(geos) == 3


def test_3d_models_glb():
    models = kinematics.arm_3d_models(include_gripper=True)
    assert set(kinematics.ARM_LINKS) <= set(models)
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
    g2 = spatial.gravity_torques([0.0] * 6, extra_payload_kg=1.0)
    assert abs(g2[2]) > abs(g[2])
    # Pointing the arm straight up (shoulder at -90) removes most of the load from the elbow.
    g3 = spatial.gravity_torques([0.0, math.radians(-90.0), 0.0, 0.0, 0.0, 0.0])
    assert abs(g3[2]) < abs(g[2])
