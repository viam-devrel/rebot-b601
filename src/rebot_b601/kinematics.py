"""Kinematics files, collision geometry, and 3D models served to viam-server, built
from whichever ``spatial.Model`` the arm holds.

The variant's URDF is the single source of truth for the joint chain (spatial.py
parses the same file for FK). This module decorates it with ``<collision>``
elements at request time, in one of three modes:

- ``primitives`` (default): one axis-aligned box per link, fitted to the
  vendor collision mesh (``<assets_dir>/primitives.json``). Parses from bytes
  with no external files and is cheap for the planner.
- ``meshes``: ``<mesh filename="meshes/<link>.stl">`` per link. The decimated
  STL bytes are returned alongside the URDF; the Python SDK puts them in
  ``GetKinematicsResponse.meshes_by_urdf_filepath`` and the RDK resolves the
  filenames against that map.
- ``none``: kinematics only, the 0.1.0 behaviour.

The gripper gets its own small URDF (one prismatic finger joint) built from the
same primitives so it can be a frame-system link with a collision body.
"""

import math
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Sequence, Tuple

from viam.proto.common import Geometry, KinematicsFileFormat, Mesh, Pose, RectangularPrism, Vector3

from . import spatial

COLLISION_MODES = ("primitives", "meshes", "none")

_STL_CONTENT_TYPE = "stl"
_GLB_CONTENT_TYPE = "model/gltf-binary"


def _fmt(values: Sequence[float]) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def _collision_box(
    center: Sequence[float], size: Sequence[float], rpy: Sequence[float] = (0.0, 0.0, 0.0)
) -> ET.Element:
    col = ET.Element("collision")
    ET.SubElement(col, "origin", xyz=_fmt(center), rpy=_fmt(rpy))
    geom = ET.SubElement(col, "geometry")
    ET.SubElement(geom, "box", size=_fmt(size))
    return col


def _collision_mesh(filename: str, rpy: Sequence[float] = (0.0, 0.0, 0.0)) -> ET.Element:
    col = ET.Element("collision")
    ET.SubElement(col, "origin", xyz="0 0 0", rpy=_fmt(rpy))
    geom = ET.SubElement(col, "geometry")
    ET.SubElement(geom, "mesh", filename=filename)
    return col


def _mesh_filename(link: str) -> str:
    return f"meshes/{link}.stl"


def _read_mesh(model: spatial.Model, key: str) -> Optional[bytes]:
    path = model.assets_dir / "meshes" / f"{key}.stl"
    return path.read_bytes() if path.exists() else None


def arm_kinematics(
    model: spatial.Model,
    mode: str = "primitives",
    joint_limits_deg: Optional[Sequence[Tuple[float, float]]] = None,
):
    """Return the get_kinematics tuple for ``model``: (format, urdf_bytes[, meshes]).

    ``joint_limits_deg`` (one (lo, hi) per revolute joint, base first) replaces the
    URDF's joint limits. viam-server checks joint targets against these, so an arm
    whose motors count differently from its URDF (the B601-RS) must serve its own
    range or it cannot be moved through the API at all.
    """
    if mode not in COLLISION_MODES:
        raise ValueError(f"collision_geometry must be one of {COLLISION_MODES}")
    tree = ET.parse(model.urdf_path)
    root = tree.getroot()
    # Serve the chain up to the tool mount. The bundled URDF runs one fixed joint further, to
    # the mount plate, which stays in the file for its mass and its collision asset: a gripper
    # is described by the gripper component, whose own model starts here.
    for el in root.findall("joint"):
        if el.get("type") == "fixed" and el.find("child").get("link") == model.end_link:
            root.remove(el)
    for el in root.findall("link"):
        if el.get("name") == model.end_link:
            root.remove(el)
    if joint_limits_deg is not None:
        revolute = [j for j in root.findall("joint") if j.get("type") == "revolute"]
        for joint, (lo, hi) in zip(revolute, joint_limits_deg):
            joint.find("limit").set("lower", str(math.radians(lo)))
            joint.find("limit").set("upper", str(math.radians(hi)))
    meshes: Dict[str, Mesh] = {}
    links = list(model.arm_links)
    for link_el in root.findall("link"):
        name = link_el.get("name")
        if name not in links or mode == "none":
            continue
        asset_key = name
        if mode == "primitives":
            prim = model.primitives.get(asset_key)
            if prim:
                link_el.append(_collision_box(prim["center"], prim["size"]))
        else:
            data = _read_mesh(model, asset_key)
            if data is not None:
                filename = _mesh_filename(asset_key)
                link_el.append(_collision_mesh(filename))
                meshes[filename] = Mesh(content_type=_STL_CONTENT_TYPE, mesh=data)
    ET.indent(tree, space="  ")
    data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    fmt = KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
    if meshes:
        return (fmt, data, meshes)
    return (fmt, data)


def _pose_from_transform(t) -> Pose:
    x, y, z, ox, oy, oz, theta = spatial.transform_to_viam_pose(t)
    return Pose(x=x, y=y, z=z, o_x=ox, o_y=oy, o_z=oz, theta=theta)


def _box_geometry(t_link, center_m: Sequence[float], size_m: Sequence[float], label: str) -> Geometry:
    # box center in link frame -> base frame; box axes follow the link frame
    t_box = spatial._mat_mul(t_link, spatial._transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], list(center_m)))
    return Geometry(
        center=_pose_from_transform(t_box),
        box=RectangularPrism(dims_mm=Vector3(x=size_m[0] * 1000, y=size_m[1] * 1000, z=size_m[2] * 1000)),
        label=label,
    )


def arm_geometries(model: spatial.Model, joint_degs: Sequence[float]) -> List[Geometry]:
    """Per-link bounding boxes posed by the current joint state, in the arm's base frame.

    Always uses the primitive boxes, even in ``meshes`` mode: the planner gets
    the meshes through the kinematics model, and callers of GetGeometries
    usually want something cheap.
    """
    rads = [math.radians(d) for d in joint_degs]
    transforms = model.link_transforms(rads)
    out = []
    for idx, name in enumerate(model.link_order):
        if name not in model.arm_links:
            continue
        prim = model.primitives.get(name)
        if not prim:
            continue
        out.append(_box_geometry(transforms[idx], prim["center"], prim["size"], name))
    return out


def arm_3d_models(model: spatial.Model) -> Dict[str, Mesh]:
    """GLB visual meshes keyed by link name, for the app's 3D view."""
    models: Dict[str, Mesh] = {}
    for name in model.arm_links:
        path = model.assets_dir / "models" / f"{name}.glb"
        if path.exists():
            models[name] = Mesh(content_type=_GLB_CONTENT_TYPE, mesh=path.read_bytes())
    return models


# --- gripper ---


def _aabb(points: Sequence[Sequence[float]]) -> Tuple[List[float], List[float]]:
    """(center, size) of the axis-aligned box bounding ``points``."""
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    return ([(lo[i] + hi[i]) / 2 for i in range(3)], [hi[i] - lo[i] for i in range(3)])


def _box_corners(center: Sequence[float], size: Sequence[float], transform=None):
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                p = [center[i] + s * size[i] / 2 for i, s in enumerate((sx, sy, sz))]
                out.append(spatial.apply(transform, p) if transform is not None else p)
    return out


def gripper_base_box(model: spatial.Model) -> Optional[Tuple[List[float], List[float]]]:
    """gripper_base's single collision body: the axis-aligned union, in the vendor mount-plate
    frame the primitives are authored in, of the body box and the right finger's static travel
    envelope. ``gripper_urdf`` rotates it into gripper_base's frame.

    The served model has to be one chain with exactly one leaf, because viam-server's URDF
    parser (``referenceframe.ParseConfig``) rejects a model with more than one end effector,
    and a link carries exactly one ``<collision>``, because ``UnmarshalModelXML`` keeps
    ``Collision[0]`` and drops the rest without a word. So the right finger can be neither its
    own leaf link nor a second box on this one, and its volume is folded in here instead of
    disappearing. Over-approximating is the safe direction for a planner; ``gripper_geometries``
    still reports the three precise boxes.
    """
    g = model.gripper
    base = model.primitives.get(model.mount_asset_key)
    if not base:
        return None
    corners = _box_corners(base["center"], base["size"])
    right = model.primitives.get(g.right_key)
    if right:
        axis_i = max(range(3), key=lambda i: abs(g.axis[i]))
        center, size = list(right["center"]), list(right["size"])
        center[axis_i] += g.right_travel_sign * g.travel_m / 2
        size[axis_i] += g.travel_m
        xyz, rpy = g.right_origin
        corners += _box_corners(center, size, spatial._transform(spatial._rot_rpy(*rpy), list(xyz)))
    return _aabb(corners)


def gripper_urdf(model: spatial.Model, mode: str = "primitives") -> Tuple[bytes, Dict[str, Mesh]]:
    """A one-DoF gripper model: base + left finger on a prismatic joint. Root frame = the
    arm's tool mount, which is where the served arm chain ends, so configure the gripper with
    the arm as frame parent and zero translation.

    Every joint here carries translation only, so gripper_base and finger_left_link keep the
    tool mount's axes and the component's pose in world means something next to the arm's. The
    vendor rotations -- the mount plate's, then the finger's -- do not vanish: each accumulates
    into its link's ``<collision origin rpy>``, which leaves every box exactly where it was
    (``test_gripper_boxes_did_not_move_in_space``). Same trick as SO-101's gripper model.

    The left finger is the model's only leaf and gripper_base carries the right finger's
    travel envelope (see ``gripper_base_box``), so the served model is coarser than the
    geometries ``gripper_geometries`` reports. That asymmetry is deliberate: the model must
    obey the RDK's one-leaf and one-collision-per-link rules, GetGeometries need not.
    """
    robot = ET.Element("robot", name="rebot_b601_gripper")
    meshes: Dict[str, Mesh] = {}
    g = model.gripper

    def add_link(name: str, asset: str, rot, rpy: Sequence[float]):
        link = ET.SubElement(robot, "link", name=name)
        if mode == "none":
            return
        data = _read_mesh(model, asset) if mode == "meshes" else None
        if data is not None:
            filename = _mesh_filename(asset)
            link.append(_collision_mesh(filename, rpy))
            meshes[filename] = Mesh(content_type=_STL_CONTENT_TYPE, mesh=data)
            return
        prim = model.primitives.get(asset)
        if prim:
            link.append(_collision_box(spatial.rotate(rot, prim["center"]), prim["size"], rpy))

    mount_joint = model.chain[-1]  # end_joint: the tool mount -> mount plate transform
    mount_rot = spatial._rot_rpy(*mount_joint.rpy)
    # The vendor rpy of the finger joint, folded into the mount's: the rotation of the finger's
    # own frame relative to the tool mount, which is what its collision body has to carry.
    (lxyz, lrpy) = g.left_origin
    finger_rot = spatial._mat_mul(mount_rot, spatial._rot_rpy(*lrpy))
    finger_rpy = spatial.rpy_from_rot(finger_rot)

    # The gripper's model starts where the arm's ends. This fixed joint carries the mount
    # plate's offset along the approach axis; its rotation stays out of the chain.
    ET.SubElement(robot, "link", name="tool_mount")
    j = ET.SubElement(robot, "joint", name="tool_mount_joint", type="fixed")
    ET.SubElement(j, "origin", xyz=_fmt(mount_joint.xyz), rpy="0 0 0")
    ET.SubElement(j, "parent", link="tool_mount")
    ET.SubElement(j, "child", link="gripper_base")

    # gripper_base always carries the union box, even in meshes mode: a mesh cannot also cover
    # the right finger's envelope, and losing that volume would blind the planner to half the jaw.
    base = ET.SubElement(robot, "link", name="gripper_base")
    box = gripper_base_box(model) if mode != "none" else None
    if box:
        center, size = box
        base.append(_collision_box(spatial.rotate(mount_rot, center), size, mount_joint.rpy))
    add_link("finger_left_link", g.left_key, finger_rot, finger_rpy)

    j = ET.SubElement(robot, "joint", name="finger_left", type="prismatic")
    # Offset and travel axis expressed in the tool mount's frame, the frame both links now use.
    ET.SubElement(j, "origin", xyz=_fmt(spatial.rotate(mount_rot, lxyz)), rpy="0 0 0")
    ET.SubElement(j, "parent", link="gripper_base")
    ET.SubElement(j, "child", link="finger_left_link")
    ET.SubElement(j, "axis", xyz=_fmt(spatial.rotate(finger_rot, g.axis)))
    ET.SubElement(j, "limit", lower="0", upper=f"{g.travel_m}", effort="8", velocity="0.08")

    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True), meshes


def gripper_kinematics(model: spatial.Model, mode: str = "primitives"):
    data, meshes = gripper_urdf(model, mode)
    fmt = KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
    if meshes:
        return (fmt, data, meshes)
    return (fmt, data)


def gripper_geometries(model: spatial.Model, finger_travel_m: float) -> List[Geometry]:
    """Gripper boxes in the gripper's own frame (the tool mount) for the given
    left-finger travel.

    Three boxes, one per real part, where the served model has two links: GetGeometries is not
    a frame system, so the RDK's one-leaf and one-collision-per-link rules do not apply and the
    right finger is reported where it actually is instead of folded into gripper_base's envelope
    (see ``gripper_base_box``). Finer here, coarser there, on purpose.
    """
    g = model.gripper
    mj = model.chain[-1]
    # Same transform the served URDF's tool_mount_joint carries: the primitives are
    # authored in the mount plate frame, the returned frame is the tool mount.
    mount = spatial._transform(spatial._rot_rpy(*mj.rpy), list(mj.xyz))

    def finger_frame(origin, travel):
        (xyz, rpy) = origin
        base = spatial._transform(spatial._rot_rpy(*rpy), list(xyz))
        slide = spatial._transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [a * travel for a in g.axis])
        return spatial._mat_mul(mount, spatial._mat_mul(base, slide))

    out = []
    prim = model.primitives.get(model.mount_asset_key)
    if prim:
        out.append(_box_geometry(mount, prim["center"], prim["size"], "gripper_base"))
    prim = model.primitives.get(g.left_key)
    if prim:
        t = finger_frame(g.left_origin, finger_travel_m)
        out.append(_box_geometry(t, prim["center"], prim["size"], "finger_left_link"))
    prim = model.primitives.get(g.right_key)
    if prim:
        t = finger_frame(g.right_origin, g.right_travel_sign * finger_travel_m)
        out.append(_box_geometry(t, prim["center"], prim["size"], "finger_right_link"))
    return out
