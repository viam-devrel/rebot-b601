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

FINGER_TRAVEL_M = 0.05  # per finger, from the vendor URDF prismatic limits

_STL_CONTENT_TYPE = "stl"
_GLB_CONTENT_TYPE = "model/gltf-binary"


def _fmt(values: Sequence[float]) -> str:
    return " ".join(f"{v:.6g}" for v in values)


def _collision_box(center: Sequence[float], size: Sequence[float]) -> ET.Element:
    col = ET.Element("collision")
    ET.SubElement(col, "origin", xyz=_fmt(center), rpy="0 0 0")
    geom = ET.SubElement(col, "geometry")
    ET.SubElement(geom, "box", size=_fmt(size))
    return col


def _collision_mesh(filename: str) -> ET.Element:
    col = ET.Element("collision")
    ET.SubElement(col, "origin", xyz="0 0 0", rpy="0 0 0")
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
    include_gripper_geometry: bool = False,
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
    if joint_limits_deg is not None:
        revolute = [j for j in root.findall("joint") if j.get("type") == "revolute"]
        for joint, (lo, hi) in zip(revolute, joint_limits_deg):
            joint.find("limit").set("lower", str(math.radians(lo)))
            joint.find("limit").set("upper", str(math.radians(hi)))
    meshes: Dict[str, Mesh] = {}
    links = list(model.arm_links)
    if include_gripper_geometry:
        links.append(model.end_link)
    for link_el in root.findall("link"):
        name = link_el.get("name")
        if name not in links or mode == "none":
            continue
        asset_key = model.mount_asset_key if name == model.end_link else name
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


def arm_geometries(
    model: spatial.Model, joint_degs: Sequence[float], include_gripper_geometry: bool = False
) -> List[Geometry]:
    """Per-link bounding boxes posed by the current joint state, in the arm's base frame.

    Always uses the primitive boxes, even in ``meshes`` mode: the planner gets
    the meshes through the kinematics model, and callers of GetGeometries
    usually want something cheap.
    """
    rads = [math.radians(d) for d in joint_degs]
    transforms = model.link_transforms(rads)
    out = []
    for idx, name in enumerate(model.link_order):
        if name == model.end_link and not include_gripper_geometry:
            continue
        key = model.mount_asset_key if name == model.end_link else name
        prim = model.primitives.get(key)
        if not prim:
            continue
        out.append(_box_geometry(transforms[idx], prim["center"], prim["size"], name))
    return out


def arm_3d_models(model: spatial.Model, include_gripper: bool = False) -> Dict[str, Mesh]:
    """GLB visual meshes keyed by link name, for the app's 3D view."""
    models: Dict[str, Mesh] = {}
    names = list(model.arm_links)
    if include_gripper:
        # finger GLBs exist only for DM; on RS the mount body is served and the fingers are skipped
        names += [model.end_link, "finger_left_link", "finger_right_link"]
    for name in names:
        path = model.assets_dir / "models" / f"{name}.glb"
        if path.exists():
            models[name] = Mesh(content_type=_GLB_CONTENT_TYPE, mesh=path.read_bytes())
    return models


# --- gripper ---


def gripper_urdf(model: spatial.Model, mode: str = "primitives") -> Tuple[bytes, Dict[str, Mesh]]:
    """A one-DoF gripper model: base + left finger on a prismatic joint + a
    right-finger envelope covering its full travel. Root frame = the arm's
    end_link, so configure the gripper with the arm as frame parent and zero
    translation."""
    robot = ET.Element("robot", name="rebot_b601_gripper")
    meshes: Dict[str, Mesh] = {}
    g = model.gripper
    axis_i = max(range(3), key=lambda i: abs(g.axis[i]))

    def add_link(name: str, asset: str, widen: float = 0.0, shift: float = 0.0):
        link = ET.SubElement(robot, "link", name=name)
        if mode == "none":
            return
        data = _read_mesh(model, asset) if mode == "meshes" else None
        if data is not None:
            filename = _mesh_filename(asset)
            link.append(_collision_mesh(filename))
            meshes[filename] = Mesh(content_type=_STL_CONTENT_TYPE, mesh=data)
            return
        prim = model.primitives.get(asset)
        if prim:
            center = list(prim["center"])
            size = list(prim["size"])
            center[axis_i] += shift
            size[axis_i] += widen
            link.append(_collision_box(center, size))

    add_link("gripper_base", model.mount_asset_key)
    add_link("finger_left_link", g.left_key)
    # The right finger mirrors the left; model it as a static envelope over its travel.
    add_link(
        "finger_right_link", g.right_key,
        widen=g.travel_m, shift=g.right_travel_sign * g.travel_m / 2,
    )

    (lxyz, lrpy), (rxyz, rrpy) = g.left_origin, g.right_origin
    j = ET.SubElement(robot, "joint", name="finger_left", type="prismatic")
    ET.SubElement(j, "origin", xyz=_fmt(lxyz), rpy=_fmt(lrpy))
    ET.SubElement(j, "parent", link="gripper_base")
    ET.SubElement(j, "child", link="finger_left_link")
    ET.SubElement(j, "axis", xyz=_fmt(g.axis))
    ET.SubElement(j, "limit", lower="0", upper=f"{g.travel_m}", effort="8", velocity="0.08")

    j = ET.SubElement(robot, "joint", name="finger_right", type="fixed")
    ET.SubElement(j, "origin", xyz=_fmt(rxyz), rpy=_fmt(rrpy))
    ET.SubElement(j, "parent", link="gripper_base")
    ET.SubElement(j, "child", link="finger_right_link")

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
    """Gripper boxes in the gripper's own frame for the given left-finger travel."""
    g = model.gripper
    identity = spatial._transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])

    def finger_frame(origin, travel):
        (xyz, rpy) = origin
        base = spatial._transform(spatial._rot_rpy(*rpy), list(xyz))
        slide = spatial._transform(
            [[1, 0, 0], [0, 1, 0], [0, 0, 1]], [a * travel for a in g.axis]
        )
        return spatial._mat_mul(base, slide)

    out = []
    prim = model.primitives.get(model.mount_asset_key)
    if prim:
        out.append(_box_geometry(identity, prim["center"], prim["size"], "gripper_base"))
    prim = model.primitives.get(g.left_key)
    if prim:
        t = finger_frame(g.left_origin, finger_travel_m)
        out.append(_box_geometry(t, prim["center"], prim["size"], "finger_left_link"))
    prim = model.primitives.get(g.right_key)
    if prim:
        t = finger_frame(g.right_origin, g.right_travel_sign * finger_travel_m)
        out.append(_box_geometry(t, prim["center"], prim["size"], "finger_right_link"))
    return out
