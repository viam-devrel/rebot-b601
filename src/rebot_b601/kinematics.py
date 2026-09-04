"""Kinematics files, collision geometry, and 3D models served to viam-server.

The bundled ``rebot_b601_dm.urdf`` is the single source of truth for the joint
chain (spatial.py parses the same file for FK). This module decorates it with
``<collision>`` elements at request time, in one of three modes:

- ``primitives`` (default): one axis-aligned box per link, fitted to the
  vendor collision mesh (``assets/primitives.json``). Parses from bytes with
  no external files and is cheap for the planner.
- ``meshes``: ``<mesh filename="meshes/<link>.stl">`` per link. The decimated
  STL bytes are returned alongside the URDF; the Python SDK puts them in
  ``GetKinematicsResponse.meshes_by_urdf_filepath`` and the RDK resolves the
  filenames against that map.
- ``none``: kinematics only, the 0.1.0 behaviour.

The gripper gets its own small URDF (one prismatic finger joint) built from the
same primitives so it can be a frame-system link with a collision body.
"""

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from viam.proto.common import Geometry, KinematicsFileFormat, Mesh, Pose, RectangularPrism, Vector3

from . import spatial

ASSETS_DIR = Path(__file__).parent / "assets"
PRIMITIVES_PATH = ASSETS_DIR / "primitives.json"
MESH_DIR = ASSETS_DIR / "meshes"
MODEL_DIR = ASSETS_DIR / "models"

COLLISION_MODES = ("primitives", "meshes", "none")

# Links whose collision geometry belongs to the arm component. end_link is the
# gripper mount; its geometry is served by the gripper component unless the
# arm is configured with include_gripper_geometry.
ARM_LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6"]
GRIPPER_BASE_LINK = "gripper_base"
FINGER_TRAVEL_M = 0.05  # per finger, from the vendor URDF prismatic limits

_STL_CONTENT_TYPE = "stl"
_GLB_CONTENT_TYPE = "model/gltf-binary"


def _load_primitives() -> Dict[str, dict]:
    if not PRIMITIVES_PATH.exists():
        return {}
    data = json.loads(PRIMITIVES_PATH.read_text())
    links = data.get("links", data)
    return {k: v for k, v in links.items() if isinstance(v, dict) and "center" in v and "size" in v}


PRIMITIVES = _load_primitives()


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


def _read_mesh(link: str) -> Optional[bytes]:
    path = MESH_DIR / f"{link}.stl"
    return path.read_bytes() if path.exists() else None


def available_links(mode: str) -> List[str]:
    if mode == "primitives":
        return [l for l in ARM_LINKS if l in PRIMITIVES]
    if mode == "meshes":
        return [l for l in ARM_LINKS if (MESH_DIR / f"{l}.stl").exists()]
    return []


def arm_kinematics(mode: str = "primitives", include_gripper_geometry: bool = False):
    """Return the get_kinematics tuple for the arm: (format, urdf_bytes[, meshes])."""
    if mode not in COLLISION_MODES:
        raise ValueError(f"collision_geometry must be one of {COLLISION_MODES}")
    tree = ET.parse(spatial.URDF_PATH)
    root = tree.getroot()
    meshes: Dict[str, Mesh] = {}
    links = list(ARM_LINKS)
    if include_gripper_geometry:
        links.append("end_link")
    for link_el in root.findall("link"):
        name = link_el.get("name")
        if name not in links or mode == "none":
            continue
        asset_key = GRIPPER_BASE_LINK if name == "end_link" else name
        if mode == "primitives":
            prim = PRIMITIVES.get(asset_key)
            if prim:
                link_el.append(_collision_box(prim["center"], prim["size"]))
        else:
            data = _read_mesh(asset_key)
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


def arm_geometries(joint_degs: Sequence[float], include_gripper_geometry: bool = False) -> List[Geometry]:
    """Per-link bounding boxes posed by the current joint state, in the arm's base frame.

    Always uses the primitive boxes, even in ``meshes`` mode: the planner gets
    the meshes through the kinematics model, and callers of GetGeometries
    usually want something cheap.
    """
    rads = [math.radians(d) for d in joint_degs]
    transforms = spatial.link_transforms(rads)
    out = []
    for idx, name in enumerate(spatial.LINK_ORDER):
        if name in ARM_LINKS:
            key = name
        elif name == "end_link" and include_gripper_geometry:
            key = GRIPPER_BASE_LINK
        else:
            continue
        prim = PRIMITIVES.get(key)
        if not prim:
            continue
        out.append(_box_geometry(transforms[idx], prim["center"], prim["size"], name))
    return out


def arm_3d_models(include_gripper: bool = False) -> Dict[str, Mesh]:
    """GLB visual meshes keyed by link name, for the app's 3D view."""
    models: Dict[str, Mesh] = {}
    names = list(ARM_LINKS)
    if include_gripper:
        names += ["end_link", "finger_left_link", "finger_right_link"]
    for name in names:
        path = MODEL_DIR / f"{name}.glb"
        if path.exists():
            models[name] = Mesh(content_type=_GLB_CONTENT_TYPE, mesh=path.read_bytes())
    return models


# --- gripper ---


def gripper_urdf(mode: str = "primitives") -> Tuple[bytes, Dict[str, Mesh]]:
    """A one-DoF gripper model: base + left finger on a prismatic joint + a
    right-finger envelope covering its full travel. Root frame = the arm's
    end_link, so configure the gripper with the arm as frame parent and zero
    translation."""
    robot = ET.Element("robot", name="rebot_b601_gripper")
    meshes: Dict[str, Mesh] = {}

    def add_link(name: str, asset: str, widen_y: float = 0.0, shift_y: float = 0.0):
        link = ET.SubElement(robot, "link", name=name)
        if mode == "none":
            return
        if mode == "meshes" and _read_mesh(asset) is not None:
            filename = _mesh_filename(asset)
            link.append(_collision_mesh(filename))
            meshes[filename] = Mesh(content_type=_STL_CONTENT_TYPE, mesh=_read_mesh(asset))
            return
        prim = PRIMITIVES.get(asset)
        if prim:
            center = list(prim["center"])
            size = list(prim["size"])
            center[1] += shift_y
            size[1] += widen_y
            link.append(_collision_box(center, size))

    add_link("gripper_base", GRIPPER_BASE_LINK)
    add_link("finger_left_link", "left_finger")
    # The right finger mirrors the left; model it as a static envelope over its travel.
    add_link("finger_right_link", "right_finger", widen_y=FINGER_TRAVEL_M, shift_y=-FINGER_TRAVEL_M / 2)

    j = ET.SubElement(robot, "joint", name="finger_left", type="prismatic")
    ET.SubElement(j, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(j, "parent", link="gripper_base")
    ET.SubElement(j, "child", link="finger_left_link")
    ET.SubElement(j, "axis", xyz="0 1 0")
    ET.SubElement(j, "limit", lower="0", upper=f"{FINGER_TRAVEL_M}", effort="8", velocity="0.08")

    j = ET.SubElement(robot, "joint", name="finger_right", type="fixed")
    ET.SubElement(j, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(j, "parent", link="gripper_base")
    ET.SubElement(j, "child", link="finger_right_link")

    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    return ET.tostring(robot, encoding="utf-8", xml_declaration=True), meshes


def gripper_kinematics(mode: str = "primitives"):
    data, meshes = gripper_urdf(mode)
    fmt = KinematicsFileFormat.KINEMATICS_FILE_FORMAT_URDF
    if meshes:
        return (fmt, data, meshes)
    return (fmt, data)


def gripper_geometries(finger_travel_m: float) -> List[Geometry]:
    """Gripper boxes in the gripper's own frame for the given left-finger travel."""
    identity = spatial._transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
    out = []
    prim = PRIMITIVES.get(GRIPPER_BASE_LINK)
    if prim:
        out.append(_box_geometry(identity, prim["center"], prim["size"], "gripper_base"))
    prim = PRIMITIVES.get("left_finger")
    if prim:
        c = list(prim["center"])
        c[1] += finger_travel_m
        out.append(_box_geometry(identity, c, prim["size"], "finger_left_link"))
    prim = PRIMITIVES.get("right_finger")
    if prim:
        c = list(prim["center"])
        c[1] -= finger_travel_m
        out.append(_box_geometry(identity, c, prim["size"], "finger_right_link"))
    return out
