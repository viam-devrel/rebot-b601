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


def test_fk_against_pytransform3d():
    import numpy as np
    from pytransform3d.urdf import UrdfTransformManager

    tm = UrdfTransformManager()
    tm.load_urdf(spatial.URDF_PATH.read_text())

    joint_names = [j.name for j in spatial.REVOLUTE_JOINTS]
    limits = [(j.lower, j.upper) for j in spatial.REVOLUTE_JOINTS]

    for trial in range(200):
        if trial == 0:
            q = [0.0] * 6
        else:
            q = [random.uniform(lo, hi) for lo, hi in limits]
        for name, angle in zip(joint_names, q):
            tm.set_joint(name, angle)
        expected = tm.get_transform("end_link", "base_link")
        (x, y, z), rot = spatial.forward_kinematics(q)
        assert np.allclose(expected[:3, 3], [x, y, z], atol=1e-9), (
            f"FK position mismatch at {q}: {expected[:3, 3]} vs {(x, y, z)}"
        )
        assert np.allclose(expected[:3, :3], np.array(rot), atol=1e-9), f"FK rotation mismatch at {q}"
    print("FK vs pytransform3d: 200 random configurations OK")


def test_end_position_units():
    x, y, z, ox, oy, oz, theta = spatial.end_position([0, 0, 0, 0, 0, 0])
    norm = math.sqrt(ox * ox + oy * oy + oz * oz)
    assert abs(norm - 1.0) < 1e-9
    print(f"zero pose: x={x:.1f} y={y:.1f} z={z:.1f} mm, o=({ox:.3f},{oy:.3f},{oz:.3f}), theta={theta:.1f} deg")


if __name__ == "__main__":
    test_ov_round_trip()
    test_end_position_units()
    test_fk_against_pytransform3d()
    print("all spatial tests passed")
