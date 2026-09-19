import math
import xml.etree.ElementTree as ET

import pytest
from viam.proto.common import KinematicsFileFormat

from src.rebot_b601 import kinematics, spatial

DM, RS = spatial.MODELS["dm"], spatial.MODELS["rs"]
BOTH = [pytest.param(DM, id="dm"), pytest.param(RS, id="rs")]


_I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]


def _links_with_collision(urdf: bytes):
    root = ET.fromstring(urdf)
    return {l.get("name") for l in root.findall("link") if l.find("collision") is not None}


def _walk_gripper(urdf: bytes, travel: float):
    """Walk the served gripper chain from tool_mount: (link frames, collision-body frames),
    both as 4x4 transforms in the tool mount's frame, with the finger slid by ``travel``."""
    root = ET.fromstring(urdf)
    joints = {j.find("parent").get("link"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}
    ident = spatial._transform(_I3, [0, 0, 0])

    def origin(el):
        if el is None:
            return ident
        xyz = [float(v) for v in (el.get("xyz") or "0 0 0").split()]
        rpy = [float(v) for v in (el.get("rpy") or "0 0 0").split()]
        return spatial._transform(spatial._rot_rpy(*rpy), xyz)

    frames, bodies, name, t = {}, {}, "tool_mount", ident
    while True:
        frames[name] = t
        col = links[name].find("collision")
        if col is not None:
            bodies[name] = spatial._mat_mul(t, origin(col.find("origin")))
        joint = joints.get(name)
        if joint is None:
            return frames, bodies
        t = spatial._mat_mul(t, origin(joint.find("origin")))
        if joint.get("type") == "prismatic":
            axis = [float(v) for v in joint.find("axis").get("xyz").split()]
            t = spatial._mat_mul(t, spatial._transform(_I3, [a * travel for a in axis]))
        name = joint.find("child").get("link")


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
    assert joints == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


@pytest.mark.parametrize("model", BOTH)
def test_served_arm_chain_ends_at_the_tool_mount(model):
    root = ET.fromstring(kinematics.arm_kinematics(model, "primitives")[1])
    names = {j.get("name") for j in root.findall("joint")}
    assert "end_joint" not in names
    assert {l.get("name") for l in root.findall("link")} == set(model.arm_links)
    children = {j.find("child").get("link") for j in root.findall("joint")}
    parents = {j.find("parent").get("link") for j in root.findall("joint")}
    assert children - parents == {model.tool_mount_link}  # the only leaf


@pytest.mark.parametrize("model", BOTH)
def test_each_served_link_has_exactly_one_collision(model):
    """The RDK keeps only the first <collision> per link and discards the rest silently, so a
    second one would vanish without an error. Counting is the only assertion that catches it."""
    root = ET.fromstring(kinematics.arm_kinematics(model, "primitives")[1])
    for link in root.findall("link"):
        assert len(link.findall("collision")) == 1, link.get("name")


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
    assert _links_with_collision(data) == {"gripper_base", "finger_left_link"}
    limit = root.find("joint[@name='finger_left']/limit")
    assert float(limit.get("upper")) == pytest.approx(model.gripper.travel_m)
    # GetGeometries still reports all three real parts; only the served model is folded to two.
    assert len(kinematics.gripper_geometries(model, 0.02)) == 3


@pytest.mark.parametrize("model", BOTH)
def test_gripper_model_has_exactly_one_leaf(model):
    """viam-server's ParseConfig requires a single end effector of a URDF model (OutputFrames
    are a Viam-JSON concept plain URDF cannot express). Two leaves and the whole gripper drops
    out of the frame system with only `buildCache # of frames: 2` in the logs to show for it."""
    root = ET.fromstring(kinematics.gripper_kinematics(model, "primitives")[1])
    links = {link.get("name") for link in root.findall("link")}
    parents = {j.find("parent").get("link") for j in root.findall("joint")}
    assert links - parents == {"finger_left_link"}


@pytest.mark.parametrize("model", BOTH)
def test_gripper_base_carries_the_right_finger_envelope(model):
    """One <collision> per link, so the right finger's travel envelope is folded into
    gripper_base's box rather than served as a second body or a second link."""
    root = ET.fromstring(kinematics.gripper_kinematics(model, "primitives")[1])
    base_link = root.find("link[@name='gripper_base']")
    assert len(base_link.findall("collision")) == 1
    size = [float(v) for v in base_link.find("collision/geometry/box").get("size").split()]
    # gripper_base's frame now keeps the tool mount's axes, so the box carries the mount plate's
    # rotation on its own origin. Undo it to compare against the plate-frame primitives.
    mount_rpy = model.chain[-1].rpy
    assert [float(v) for v in base_link.find("collision/origin").get("rpy").split()] == pytest.approx(mount_rpy)
    mount = spatial._rot_rpy(*mount_rpy)
    center = list(
        spatial.rotate(
            [[mount[j][i] for j in range(3)] for i in range(3)],
            [float(v) for v in base_link.find("collision/origin").get("xyz").split()],
        )
    )

    g = model.gripper
    body = model.primitives[model.mount_asset_key]
    right = model.primitives[g.right_key]
    ri = max(range(3), key=lambda k: abs(g.axis[k]))
    env_center, env_size = list(right["center"]), list(right["size"])
    env_center[ri] += g.right_travel_sign * g.travel_m / 2  # the envelope over the full travel
    env_size[ri] += g.travel_m
    xyz, rpy = g.right_origin
    corners = kinematics._box_corners(env_center, env_size, spatial._transform(spatial._rot_rpy(*rpy), list(xyz)))

    # the jaw axis in gripper_base's frame: the finger's travel axis through the finger's rpy
    jaw = spatial.rotate(spatial._transform(spatial._rot_rpy(*rpy), [0, 0, 0]), g.axis)
    i = max(range(3), key=lambda k: abs(jaw[k]))
    # 1e-6 m of slack: the served origin and size are _fmt'd to six significant digits
    lo, hi = center[i] - size[i] / 2 - 1e-6, center[i] + size[i] / 2 + 1e-6
    assert lo <= body["center"][i] - body["size"][i] / 2 and hi >= body["center"][i] + body["size"][i] / 2
    assert lo <= min(c[i] for c in corners) and hi >= max(c[i] for c in corners)
    # and the envelope really did widen it, rather than sitting inside the body box
    assert size[i] > body["size"][i]


def test_rs_gripper_fingers_are_set_back_from_the_mount():
    rs = spatial.MODELS["rs"]
    root = ET.fromstring(kinematics.gripper_kinematics(rs, "primitives")[1])
    origin = root.find("joint[@name='finger_left']/origin")
    xyz = [float(v) for v in origin.get("xyz").split()]
    # The vendor's 41.939 mm setback, rotated into the tool mount's frame: it runs back along
    # the approach axis there, not along the finger frame's own -x.
    assert math.dist((0, 0, 0), xyz) == pytest.approx(0.041939, abs=1e-6)
    assert xyz[2] == pytest.approx(-0.041939, abs=1e-6)
    # Translation only. The vendor rpy rides on the finger's collision body instead.
    assert origin.get("rpy") == "0 0 0"


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


@pytest.mark.parametrize("model", BOTH)
def test_gripper_model_roots_at_the_tool_mount(model):
    root = ET.fromstring(kinematics.gripper_kinematics(model, "primitives")[1])
    links = [l.get("name") for l in root.findall("link")]
    assert links[0] == "tool_mount"
    mount = root.find("joint[@name='tool_mount_joint']")
    assert mount.get("type") == "fixed"
    assert mount.find("parent").get("link") == "tool_mount"
    assert mount.find("child").get("link") == "gripper_base"
    xyz = [float(v) for v in mount.find("origin").get("xyz").split()]
    assert xyz == pytest.approx([0.0, 0.0, {"dm": 0.15539, "rs": 0.16621}[model.name]])
    # Pure translation along the approach axis: the mount plate's rpy rides on the box instead.
    assert mount.find("origin").get("rpy") == "0 0 0"


@pytest.mark.parametrize("travel", [0.0, 0.02])
@pytest.mark.parametrize("model", BOTH)
def test_gripper_boxes_did_not_move_in_space(model, travel):
    """The frame refactor moved every vendor rotation off the joints and onto the collision
    origins. Each body must still land exactly where the vendor chain -- the arm's end_joint,
    then the finger joint, then the slide along the finger's own axis -- puts it. This is the
    assertion that makes the refactor safe: break it and the planner's idea of where the jaw is
    drifts quietly off the metal while every other gripper test stays green.
    """
    _, bodies = _walk_gripper(kinematics.gripper_kinematics(model, "primitives")[1], travel)
    g = model.gripper
    mj = model.chain[-1]
    mount = spatial._transform(spatial._rot_rpy(*mj.rpy), list(mj.xyz))
    lxyz, lrpy = g.left_origin
    finger = spatial._mat_mul(mount, spatial._transform(spatial._rot_rpy(*lrpy), list(lxyz)))
    finger = spatial._mat_mul(finger, spatial._transform(_I3, [a * travel for a in g.axis]))
    want = {
        "gripper_base": (mount, kinematics.gripper_base_box(model)[0]),
        "finger_left_link": (finger, model.primitives[g.left_key]["center"]),
    }
    assert set(bodies) == set(want)
    for link, (frame, center) in want.items():
        expected = spatial._mat_mul(frame, spatial._transform(_I3, list(center)))
        for r in range(3):
            # Slack is _fmt's six significant digits on the accumulated rpy -- finer than the
            # five-digit angles the vendor URDFs themselves carry -- not a real displacement.
            assert expected[r][3] == pytest.approx(bodies[link][r][3], abs=1e-7), (link, r)
            for c in range(3):
                assert expected[r][c] == pytest.approx(bodies[link][r][c], abs=1e-5), (link, r, c)


@pytest.mark.parametrize("model", BOTH)
def test_gripper_link_frames_keep_the_tool_mount_axes(model):
    """viam-server reports the gripper component at its model's leaf frame, so link frames that
    carry the vendor rotations make the component's orientation in world unrelated to the arm's
    (it read (0,0,1) th 0 at the base and (0,-1,0) th -180 at the RS leaf). Every joint is a
    pure translation now, so both frames read exactly what the arm's tool mount reads.
    """
    arm = model.end_position([0.0] * 6)
    assert arm[3:6] == pytest.approx((1.0, 0.0, 0.0), abs=1e-4)
    # -179.9996, and +/-180 is where the orientation vector's theta wraps: compare magnitudes.
    assert abs(arm[6]) == pytest.approx(180.0, abs=1e-3)
    mount_t = model.link_transforms([0.0] * 6)[model.link_order.index(model.tool_mount_link)]
    frames, _ = _walk_gripper(kinematics.gripper_kinematics(model, "primitives")[1], 0.0)
    for name in ("gripper_base", "finger_left_link"):
        pose = spatial.transform_to_viam_pose(spatial._mat_mul(mount_t, frames[name]))
        assert pose[3:6] == pytest.approx(arm[3:6], abs=1e-6), name
        assert abs(pose[6]) == pytest.approx(abs(arm[6]), abs=1e-3), name
