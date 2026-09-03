"""Forward kinematics for the reBot B601-DM, plus rotation -> Viam orientation
vector conversion.

The kinematic chain is parsed from the bundled URDF so there is a single source
of truth shared with the kinematics file served to viam-server. The quaternion
-> orientation vector conversion is a direct port of Viam RDK's
spatialmath.QuatToOV (spatialmath/quaternion.go) so poses reported here match
what the RDK computes from the same URDF.
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

URDF_PATH = Path(__file__).parent / "rebot_b601_dm.urdf"

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
        self.origin = _transform(_rot_rpy(*rpy), xyz)
        axis_el = el.find("axis")
        self.axis = (
            [float(v) for v in axis_el.get("xyz").split()] if axis_el is not None else [0.0, 0.0, 1.0]
        )
        limit = el.find("limit")
        self.lower = float(limit.get("lower")) if limit is not None else 0.0
        self.upper = float(limit.get("upper")) if limit is not None else 0.0


def load_chain(urdf_path=URDF_PATH):
    """Return the URDF joints ordered base -> end effector."""
    root = ET.parse(urdf_path).getroot()
    joints = {j.get("name"): _Joint(j) for j in root.findall("joint")}
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


_CHAIN = load_chain()
REVOLUTE_JOINTS = [j for j in _CHAIN if j.type == "revolute"]
JOINT_LIMITS_DEG = [(math.degrees(j.lower), math.degrees(j.upper)) for j in REVOLUTE_JOINTS]


def forward_kinematics(joint_rads):
    """Compute the end-effector transform for the given revolute joint angles.

    Returns ((x, y, z) in meters, 3x3 rotation matrix) of end_link in base_link.
    """
    t = _transform([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
    qi = 0
    for joint in _CHAIN:
        t = _mat_mul(t, joint.origin)
        if joint.type == "revolute":
            rot = _rot_axis_angle(joint.axis, joint_rads[qi])
            t = _mat_mul(t, _transform(rot, [0, 0, 0]))
            qi += 1
    pos = (t[0][3], t[1][3], t[2][3])
    rot = [row[:3] for row in t[:3]]
    return pos, rot


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


def end_position(joint_degs):
    """FK for Viam: joint angles in degrees -> (x_mm, y_mm, z_mm, ox, oy, oz, theta_deg)."""
    rads = [math.radians(d) for d in joint_degs]
    (x, y, z), rot = forward_kinematics(rads)
    ox, oy, oz, theta = quat_to_orientation_vector(rotation_to_quat(rot))
    return (x * 1000.0, y * 1000.0, z * 1000.0, ox, oy, oz, math.degrees(theta))
