"""Verification for spatial.py.

1. Round-trip: random rotations -> quat -> orientation vector -> quat (using a
   port of RDK's OV.Quaternion(), the inverse conversion) must reproduce the
   same rotation.
2. FK cross-check: forward kinematics vs pytransform3d parsing the same URDF,
   at random joint configurations within limits.

Run: .venv/bin/python tests/test_spatial.py
"""

import math
import random
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.rebot_b601 import spatial

random.seed(1234)


def quat_from_axis_angle(axis, angle):
    n = math.sqrt(sum(a * a for a in axis))
    axis = [a / n for a in axis]
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), axis[0] * s, axis[1] * s, axis[2] * s)


def quat_to_matrix(q):
    w, x, y, z = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def ov_to_quat(ox, oy, oz, theta):
    """Port of RDK OrientationVector.Quaternion(): ZYZ euler (lon, lat, theta)."""
    n = math.sqrt(ox * ox + oy * oy + oz * oz)
    ox, oy, oz = ox / n, oy / n, oz / n
    lat = math.acos(max(-1.0, min(1.0, oz)))
    lon = 0.0
    if 1 - abs(oz) > 1e-4:
        lon = math.atan2(oy, ox)
    # ZYZ: Rz(lon) * Ry(lat) * Rz(theta)
    qz1 = quat_from_axis_angle([0, 0, 1], lon)
    qy = quat_from_axis_angle([0, 1, 0], lat)
    qz2 = quat_from_axis_angle([0, 0, 1], theta)
    return spatial._quat_mul(spatial._quat_mul(qz1, qy), qz2)


def mat_close(a, b, tol=1e-6):
    return all(abs(a[i][j] - b[i][j]) < tol for i in range(3) for j in range(3))


def test_ov_round_trip():
    for _ in range(500):
        axis = [random.uniform(-1, 1) for _ in range(3)]
        if all(abs(a) < 1e-3 for a in axis):
            continue
        angle = random.uniform(-math.pi, math.pi)
        q = quat_from_axis_angle(axis, angle)
        ox, oy, oz, theta = spatial.quat_to_orientation_vector(q)
        q2 = ov_to_quat(ox, oy, oz, theta)
        m1, m2 = quat_to_matrix(q), quat_to_matrix(q2)
        # Within the RDK's pole radius the OV representation is deliberately
        # gimbal-averse and approximate (~lat rad of error); elsewhere exact.
        tol = 0.03 if 1 - abs(oz) < 1e-4 else 1e-5
        assert mat_close(m1, m2, tol), f"OV round trip failed for {q}: {(ox, oy, oz, theta)}"
    print("OV round-trip: 500 random rotations OK")


@pytest.mark.parametrize("name", ["dm", "rs"])
def test_fk_against_pytransform3d(name):
    import numpy as np
    from pytransform3d.urdf import UrdfTransformManager

    model = spatial.MODELS[name]
    tm = UrdfTransformManager()
    tm.load_urdf(model.urdf_path.read_text())
    joint_names = [j.name for j in model.revolute]
    limits = [(j.lower, j.upper) for j in model.revolute]
    rng = random.Random(1234 if name == "dm" else 5678)
    for trial in range(200):
        q = [0.0] * 6 if trial == 0 else [rng.uniform(lo, hi) for lo, hi in limits]
        for jn, angle in zip(joint_names, q):
            tm.set_joint(jn, angle)
        expected = tm.get_transform(model.end_link, "base_link")
        (x, y, z), rot = model.forward_kinematics(q)
        assert np.allclose(expected[:3, 3], [x, y, z], atol=1e-9), (
            f"{name} FK position mismatch at {q}: {expected[:3, 3]} vs {(x, y, z)}"
        )
        assert np.allclose(expected[:3, :3], np.array(rot), atol=1e-9), f"{name} FK rotation mismatch at {q}"


@pytest.mark.parametrize("name", ["dm", "rs"])
def test_end_position_units(name):
    x, y, z, ox, oy, oz, theta = spatial.MODELS[name].end_position([0, 0, 0, 0, 0, 0])
    assert abs(math.sqrt(ox * ox + oy * oy + oz * oz) - 1.0) < 1e-9
    assert abs(x) + abs(z) > 100  # millimetres, not metres


def test_rs_zero_pose_is_the_folded_rest_posture():
    x, y, z, *_ = spatial.MODELS["rs"].end_position([0] * 6)
    assert math.isclose(x, 135.5, abs_tol=0.1)
    assert math.isclose(y, 0.0, abs_tol=0.1)
    assert math.isclose(z, 217.7, abs_tol=0.1)
    # positive j2/j3 on RS move the arm the way negative ones do on DM; the height is the
    # tool mount's, 166 mm short of the mount plate the old threshold was measured at
    _, _, z_up, *_ = spatial.MODELS["rs"].end_position([0, 30, 50, 0, 0, 0])
    assert z_up > 400


def test_rs_gravity_torques_mirror_dm():
    rs, dm = spatial.MODELS["rs"], spatial.MODELS["dm"]
    g = rs.gravity_torques([0.0] * 6)
    assert math.isclose(g[0], 0.0, abs_tol=1e-9)  # base yaw sees no gravity torque
    assert abs(g[2]) == max(abs(v) for v in g) > 5.0  # elbow carries the most at the rest pose
    assert abs(rs.gravity_torques([0.0] * 6, extra_payload_kg=1.0)[2]) > abs(g[2])
    # arm straight up: RS shoulder at +90 (DM: -90) unloads the elbow
    assert abs(rs.gravity_torques([0, math.radians(90), 0, 0, 0, 0])[2]) < abs(g[2])
    # the two arms are mirrored: rest-pose elbow torques have opposite sign
    assert g[2] * dm.gravity_torques([0.0] * 6)[2] < 0


def test_rs_bundle_is_the_vendor_chain_renamed():
    import xml.etree.ElementTree as ET

    rs = Path(__file__).parent.parent / "src" / "rebot_b601" / "rebot_b601_rs.urdf"
    root = ET.parse(rs).getroot()
    assert root.get("name") == "rebot_b601_rs"
    joints = [(j.get("name"), j.get("type")) for j in root.findall("joint")]
    assert joints == [(f"joint{i}", "revolute") for i in range(1, 7)] + [("end_joint", "fixed")]
    links = [l.get("name") for l in root.findall("link")]
    assert links == ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "end_link"]
    assert root.find("link[@name='end_link']/inertial/mass").get("value") == "0.65"
    assert root.find("joint[@name='end_joint']/origin").get("xyz") == "0 0 0.16621"
    assert root.find("joint[@name='joint2']/limit").get("upper") == "3.14"
    assert not root.findall(".//visual") and not root.findall(".//collision")


def test_each_model_carries_its_gripper_spec():
    dm, rs = spatial.MODELS["dm"], spatial.MODELS["rs"]
    assert (dm.gripper.left_key, dm.gripper.right_key) == ("left_finger", "right_finger")
    assert (rs.gripper.left_key, rs.gripper.right_key) == ("gripper_left", "gripper_right")
    assert dm.gripper.travel_m == 0.05
    assert rs.gripper.travel_m == 0.0715
    # DM's finger frames coincide with the mount; RS's are set back and rotated.
    assert dm.gripper.left_origin == ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert rs.gripper.left_origin[0][0] == pytest.approx(-0.041939)
    assert dm.gripper.axis == (0.0, 1.0, 0.0) and rs.gripper.axis == (0.0, 0.0, 1.0)
    # Which way the right finger travels is per variant, verified against the vendor URDFs:
    # DM's fingers share one axis with mirrored limits, so its right finger takes negative
    # travel. RS's two joints both run 0..travel and are mirrored by their rpy instead, so
    # both take positive travel. Getting this wrong slides the RS jaw sideways as a pair.
    assert dm.gripper.right_travel_sign == -1.0
    assert rs.gripper.right_travel_sign == 1.0
    # Both variants' primitives carry the finger boxes the spec names.
    for m in (dm, rs):
        assert m.gripper.left_key in m.primitives and m.gripper.right_key in m.primitives


def test_model_still_constructs_positionally():
    # tests/test_rs.py builds Models positionally; new params must be keyword with defaults.
    m = spatial.Model("rs", spatial.RS_URDF_PATH, spatial.ASSETS_DIR / "rs", "gripper_end")
    assert m.gripper.travel_m == 0.05  # the default spec, not RS's


# Captured before the frame split. The served arm chain is about to stop at the tool mount
# (link6), but the bundled URDF keeps end_joint and the mount plate (end_link), so these must
# not move: the mount plate's mass (0.5 kg DM, 0.65 kg RS) still loads the joints. A change
# here means the trim reached the physical model, which would under-compensate manual mode
# with no visible symptom.
GRAVITY_AT_ZERO = {
    "dm": [0.0, 1.2831, 7.1806, 1.9798, 0.0, -0.0003],
    "rs": [0.0, -1.8702, -6.0873, -1.6621, 0.0, 0.0008],
}


@pytest.mark.parametrize("name", ["dm", "rs"])
def test_gravity_at_zero_survives_the_frame_split(name):
    got = spatial.MODELS[name].gravity_torques([0.0] * 6)
    assert got == pytest.approx(GRAVITY_AT_ZERO[name], abs=0.01), name


@pytest.mark.parametrize("name", ["dm", "rs"])
def test_end_position_reports_the_tool_mount(name):
    m = spatial.MODELS[name]
    assert m.tool_mount_link == "link6"
    x, y, z, *_ = m.end_position([0] * 6)
    expected = {"dm": (104.9, 191.7), "rs": (135.5, 217.7)}[name]
    assert (x, z) == pytest.approx(expected, abs=0.1)
    assert y == pytest.approx(0.0, abs=0.1)


@pytest.mark.parametrize("name", ["dm", "rs"])
def test_the_tool_mount_is_on_the_approach_axis(name):
    """Pose-independent invariant, and the premise the whole split rests on: end_joint is a
    pure +Z translation in the tool mount's frame, so that axis points at the tool at every
    pose. A vendor URDF re-pin that moved the mount off +Z would invalidate the design, and
    this is the two-line guard that would catch it."""
    end_joint = spatial.MODELS[name].chain[-1]
    assert end_joint.type == "fixed"
    assert end_joint.origin[0][3] == pytest.approx(0.0, abs=1e-9)
    assert end_joint.origin[1][3] == pytest.approx(0.0, abs=1e-9)
    assert end_joint.origin[2][3] > 0.1


if __name__ == "__main__":
    test_ov_round_trip()
    test_end_position_units("dm")
    test_fk_against_pytransform3d("dm")
    print("all spatial tests passed")
