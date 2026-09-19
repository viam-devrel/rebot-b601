"""Forward kinematics for the reBot B601 (DM and RS models), plus rotation -> Viam orientation
vector conversion.

The kinematic chain is parsed from the bundled URDF so there is a single source
of truth shared with the kinematics file served to viam-server. The quaternion
-> orientation vector conversion is a direct port of Viam RDK's
spatialmath.QuatToOV (spatialmath/quaternion.go) so poses reported here match
what the RDK computes from the same URDF.
"""

import functools
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

URDF_PATH = Path(__file__).parent / "rebot_b601_dm.urdf"
RS_URDF_PATH = Path(__file__).parent / "rebot_b601_rs.urdf"
ASSETS_DIR = Path(__file__).parent / "assets"

_POLE_RADIUS = 1e-4  # orientationVectorPoleRadius in the RDK
_ANGLE_EPSILON = 1e-4  # defaultAngleEpsilon in the RDK


# --- minimal 3x3 / 4x4 helpers (row-major lists; no numpy dependency) ---


def _mat_mul(a, b):
    n = len(a)
    return [[sum(a[i][k] * b[k][j] for k in range(n)) for j in range(n)] for i in range(n)]


def _rot_rpy(roll, pitch, yaw):
    """URDF-convention fixed-axis RPY rotation matrix (R = Rz * Ry * Rx)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return _mat_mul(rz, _mat_mul(ry, rx))


def rpy_from_rot(r):
    """Inverse of ``_rot_rpy``: the URDF fixed-axis (roll, pitch, yaw) of a rotation matrix."""
    sp = max(-1.0, min(1.0, -r[2][0]))
    cp = math.hypot(r[0][0], r[1][0])
    if cp < 1e-12:  # pitch at +/-90: only roll -/+ yaw is determined, so pin yaw at 0
        return (math.atan2(sp * r[0][1], sp * r[0][2]), math.asin(sp), 0.0)
    return (math.atan2(r[2][1], r[2][2]), math.atan2(sp, cp), math.atan2(r[1][0], r[0][0]))


def _rot_axis_angle(axis, angle):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n == 0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / n, y / n, z / n
    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _transform(rot, trans):
    return [
        [rot[0][0], rot[0][1], rot[0][2], trans[0]],
        [rot[1][0], rot[1][1], rot[1][2], trans[1]],
        [rot[2][0], rot[2][1], rot[2][2], trans[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


class _Joint:
    def __init__(self, el):
        self.name = el.get("name")
        self.type = el.get("type")
        origin = el.find("origin")
        xyz = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy") or "0 0 0").split()]
        self.xyz, self.rpy = xyz, rpy
        self.origin = _transform(_rot_rpy(*rpy), xyz)
        axis_el = el.find("axis")
        self.axis = [float(v) for v in axis_el.get("xyz").split()] if axis_el is not None else [0.0, 0.0, 1.0]
        limit = el.find("limit")
        self.lower = float(limit.get("lower")) if limit is not None else 0.0
        self.upper = float(limit.get("upper")) if limit is not None else 0.0
        self.effort = float(limit.get("effort") or 0.0) if limit is not None else 0.0


def load_chain(urdf_path):
    """Return the URDF joints ordered base -> end effector."""
    root = ET.parse(urdf_path).getroot()
    children = {j.find("parent").get("link"): j for j in root.findall("joint")}
    parents = {j.find("child").get("link") for j in root.findall("joint")}
    all_parents = {j.find("parent").get("link") for j in root.findall("joint")}
    base = (all_parents - parents).pop()
    chain, link = [], base
    while link in children:
        j = _Joint(children[link])
        chain.append(j)
        link = children[link].find("child").get("link")
    return chain


def rotation_to_quat(r):
    """3x3 rotation matrix -> quaternion (w, x, y, z)."""
    trace = r[0][0] + r[1][1] + r[2][2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2][1] - r[1][2]) * s
        y = (r[0][2] - r[2][0]) * s
        z = (r[1][0] - r[0][1]) * s
    elif r[0][0] > r[1][1] and r[0][0] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[0][0] - r[1][1] - r[2][2])
        w = (r[2][1] - r[1][2]) / s
        x = 0.25 * s
        y = (r[0][1] + r[1][0]) / s
        z = (r[0][2] + r[2][0]) / s
    elif r[1][1] > r[2][2]:
        s = 2.0 * math.sqrt(1.0 + r[1][1] - r[0][0] - r[2][2])
        w = (r[0][2] - r[2][0]) / s
        x = (r[0][1] + r[1][0]) / s
        y = 0.25 * s
        z = (r[1][2] + r[2][1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + r[2][2] - r[0][0] - r[1][1])
        w = (r[1][0] - r[0][1]) / s
        x = (r[0][2] + r[2][0]) / s
        y = (r[1][2] + r[2][1]) / s
        z = 0.25 * s
    return (w, x, y, z)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def _rotate_vec(q, v):
    """Rotate pure-quaternion v (as (0, x, y, z)) by q; returns (x, y, z)."""
    r = _quat_mul(_quat_mul(q, v), _quat_conj(q))
    return (r[1], r[2], r[3])


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    return math.sqrt(_dot(a, a))


def quat_to_orientation_vector(q):
    """Port of Viam RDK spatialmath.QuatToOV. Returns (ox, oy, oz, theta_rad)."""
    # The RDK uses -X as the reference axis here; keep it identical.
    new_x = _rotate_vec(q, (0.0, -1.0, 0.0, 0.0))
    new_z = _rotate_vec(q, (0.0, 0.0, 0.0, 1.0))
    ox, oy, oz = new_z

    if 1 - abs(new_z[2]) > _POLE_RADIUS:
        norm1 = _cross(new_z, new_x)
        norm2 = _cross(new_z, (0.0, 0.0, 1.0))
        cos_theta = max(-1.0, min(1.0, _dot(norm1, norm2) / (_norm(norm1) * _norm(norm2))))
        theta = math.acos(cos_theta)
        if theta > _POLE_RADIUS:
            # Determine the sign: rotate new_z by -theta around (ox,oy,oz) and
            # test whether we land coplanar with local-x/global-z/origin.
            half = -theta / 2.0
            s = math.sin(half)
            q2 = (math.cos(half), s * ox, s * oy, s * oz)
            test_z = _rotate_vec(q2, (0.0, 0.0, 0.0, 1.0))
            norm3 = _cross(new_z, test_z)
            cos_test = _dot(norm1, norm3) / (_norm(norm1) * _norm(norm3))
            theta = -theta if 1 - cos_test < _ANGLE_EPSILON * _ANGLE_EPSILON else theta
        else:
            theta = 0.0
    else:
        # Pointing along +/-Z: gimbal-lock special case.
        if new_z[2] < 0:
            theta = -math.atan2(new_x[1], new_x[0])
        else:
            theta = -math.atan2(new_x[1], -new_x[0])

    return (ox, oy, oz, theta)


# --- per-link transforms, inertials, gravity, and Viam pose helpers ---

GRAVITY_M_S2 = 9.80665


class _LinkInertial:
    def __init__(self, name, mass, com_xyz):
        self.name = name
        self.mass = mass
        self.com = com_xyz


def _load_links(urdf_path):
    """Return the link names in chain order (base -> end) and their inertials."""
    root = ET.parse(urdf_path).getroot()
    joints = root.findall("joint")
    child_of = {j.find("parent").get("link"): j.find("child").get("link") for j in joints}
    parents = {j.find("parent").get("link") for j in joints}
    children = {j.find("child").get("link") for j in joints}
    base = (parents - children).pop()
    order, link = [base], base
    while link in child_of:
        link = child_of[link]
        order.append(link)
    inertials = {}
    for el in root.findall("link"):
        inertial = el.find("inertial")
        if inertial is None:
            continue
        mass_el = inertial.find("mass")
        origin = inertial.find("origin")
        xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
        inertials[el.get("name")] = _LinkInertial(el.get("name"), float(mass_el.get("value")), xyz)
    return order, inertials


def apply(t, p):
    """Apply a 4x4 transform to a 3-vector point."""
    return (
        t[0][0] * p[0] + t[0][1] * p[1] + t[0][2] * p[2] + t[0][3],
        t[1][0] * p[0] + t[1][1] * p[1] + t[1][2] * p[2] + t[1][3],
        t[2][0] * p[0] + t[2][1] * p[1] + t[2][2] * p[2] + t[2][3],
    )


def rotate(t, v):
    """Apply only the rotation part of a 4x4 transform (or a bare 3x3 rotation) to a 3-vector."""
    return (
        t[0][0] * v[0] + t[0][1] * v[1] + t[0][2] * v[2],
        t[1][0] * v[0] + t[1][1] * v[1] + t[1][2] * v[2],
        t[2][0] * v[0] + t[2][1] * v[1] + t[2][2] * v[2],
    )


def transform_to_viam_pose(t, scale_mm=1000.0):
    """4x4 transform (meters) -> (x_mm, y_mm, z_mm, ox, oy, oz, theta_deg)."""
    rot = [row[:3] for row in t[:3]]
    ox, oy, oz, theta = quat_to_orientation_vector(rotation_to_quat(rot))
    return (t[0][3] * scale_mm, t[1][3] * scale_mm, t[2][3] * scale_mm, ox, oy, oz, math.degrees(theta))


Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class GripperSpec:
    """One variant's parallel-jaw geometry, in the arm's mount-link frame.

    ``left_origin``/``right_origin`` are the finger joints' (xyz, rpy) as the vendor URDF
    gives them. ``axis`` is the travel direction in the finger's OWN frame, which is why it
    differs between variants: DM's finger frames coincide with the mount, RS's are rotated.
    """

    left_key: str  # primitives.json / STL key of the left finger
    right_key: str
    left_origin: Tuple[Vec3, Vec3] = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    right_origin: Tuple[Vec3, Vec3] = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    axis: Vec3 = (0.0, 1.0, 0.0)
    travel_m: float = 0.05
    # Direction the right finger travels relative to the left, in its own frame. DM's two
    # joints share an axis and mirror each other through their limits (0..t and -t..0), so
    # the right one takes negative travel. RS's both run 0..t and are mirrored by their rpy,
    # so both take positive travel. Checked against both vendor URDFs at the pinned commit.
    right_travel_sign: float = -1.0


DM_GRIPPER = GripperSpec(left_key="left_finger", right_key="right_finger")
# Seeed reBotArm_control_py urdf/RS @ 76512eab, gripper_joint1/gripper_joint2. The URDF's
# asymmetric upper limits (0.05 / 0.0715) cannot both describe a symmetric jaw; the CAD export
# (ReBot_Arm_RS.csv) gives 0.0715 for both, so that is used. Verify with calipers.
RS_GRIPPER = GripperSpec(
    left_key="gripper_left",
    right_key="gripper_right",
    left_origin=((-0.041939, -7.3385e-05, 0.0), (1.5708, -1.5708, 0.0)),
    right_origin=((-0.041939, 7.3385e-05, 0.0), (-1.5708, -1.5708, 0.0)),
    axis=(0.0, 0.0, 1.0),
    travel_m=0.0715,
    right_travel_sign=1.0,
)


class Model:
    """One arm variant's kinematic model: chain, limits, inertials and the assets built for it."""

    def __init__(
        self,
        name: str,
        urdf_path: Path,
        assets_dir: Path,
        mount_asset_key: str,
        *,
        gripper: GripperSpec = DM_GRIPPER,
    ):
        self.name = name
        self.urdf_path = urdf_path
        self.assets_dir = assets_dir
        self.mount_asset_key = mount_asset_key  # asset stem of the mount link's body (gripper_base on DM)
        self.gripper = gripper
        self.chain = load_chain(urdf_path)
        self.revolute = [j for j in self.chain if j.type == "revolute"]
        self.effort_nm = [j.effort for j in self.revolute]
        self.link_order, self.link_inertials = _load_links(urdf_path)
        self.end_link = self.link_order[-1]
        # every link but the mount plate; the mount plate's geometry belongs to the gripper
        # component
        self.arm_links = self.link_order[:-1]
        # Where a tool bolts on, and the frame the arm serves as its end effector. The URDF
        # runs one fixed joint further, to the mount plate (end_link), which stays in the
        # model for its mass and its collision asset but is not served.
        self.tool_mount_link = self.arm_links[-1]

    @functools.cached_property
    def primitives(self) -> dict:
        """Axis-aligned collision boxes per asset key from assets_dir/primitives.json."""
        path = self.assets_dir / "primitives.json"
        if not path.exists():
            return {}
        data = json.loads(path.read_text())
        links = data.get("links", data)
        return {k: v for k, v in links.items() if isinstance(v, dict) and "center" in v and "size" in v}

    def link_transforms(self, joint_rads):
        """4x4 transforms of every link frame (in chain order) in the base frame.

        The first entry is the identity (base_link); the last is end_link.
        """
        t = _transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
        out = [t]
        qi = 0
        for joint in self.chain:
            t = _mat_mul(t, joint.origin)
            if joint.type == "revolute":
                rot = _rot_axis_angle(joint.axis, joint_rads[qi])
                t = _mat_mul(t, _transform(rot, [0, 0, 0]))
                qi += 1
            out.append(t)
        return out

    def end_position(self, joint_degs):
        """FK for Viam: joint angles in degrees -> (x_mm, y_mm, z_mm, ox, oy, oz, theta_deg)
        of the tool mount, which is the frame the served kinematics end at."""
        rads = [math.radians(d) for d in joint_degs]
        t = self.link_transforms(rads)[self.link_order.index(self.tool_mount_link)]
        return transform_to_viam_pose(t)

    def gravity_torques(self, joint_rads, gravity=(0.0, 0.0, -GRAVITY_M_S2), extra_payload_kg=0.0):
        """Joint torques (Nm) that gravity exerts on each revolute joint, i.e. the
        torque a motor must *counteract* is the negative of each value.

        Uses the URDF link masses and centers of mass. ``gravity`` is the gravity
        vector expressed in the base frame; change it for non-upright mounts.
        ``extra_payload_kg`` is added at the end_link origin.
        """
        transforms = self.link_transforms(joint_rads)
        # joint i sits at the origin of link i+1's frame; axis expressed in base frame
        joint_frames, joint_axes = [], []
        for idx, joint in enumerate(self.chain):
            if joint.type == "revolute":
                frame = transforms[idx + 1]
                joint_frames.append((frame[0][3], frame[1][3], frame[2][3]))
                joint_axes.append(rotate(frame, joint.axis))
        torques = [0.0] * len(joint_frames)
        masses = []
        for idx, name in enumerate(self.link_order):
            inertial = self.link_inertials.get(name)
            if inertial is None:
                continue
            com_world = apply(transforms[idx], inertial.com)
            masses.append((inertial.mass, com_world, idx))
        if extra_payload_kg:
            end = transforms[-1]
            masses.append((extra_payload_kg, (end[0][3], end[1][3], end[2][3]), len(self.link_order) - 1))
        for mass, com, link_idx in masses:
            force = (mass * gravity[0], mass * gravity[1], mass * gravity[2])
            # every revolute joint upstream of this link feels the torque
            qi = 0
            for j_idx, joint in enumerate(self.chain):
                if joint.type != "revolute":
                    continue
                if j_idx + 1 <= link_idx:
                    r = (com[0] - joint_frames[qi][0], com[1] - joint_frames[qi][1], com[2] - joint_frames[qi][2])
                    torques[qi] += _dot(_cross(r, force), joint_axes[qi])
                qi += 1
        return torques


MODELS = {
    "dm": Model("dm", URDF_PATH, ASSETS_DIR, "gripper_base"),
    "rs": Model("rs", RS_URDF_PATH, ASSETS_DIR / "rs", "gripper_end", gripper=RS_GRIPPER),
}
