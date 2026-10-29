"""Harness for the forward-kinematics verification in Part A.

Fill in the comparison functions. Run it, paste the output into your report, and explain any
disagreement rather than hiding it.

    python scripts/verify_fk.py
"""

import numpy as np

from kinematics.fk import fk, fk_poe
from robot import widowx
from kinematics.transforms import translation, rot_x, rot_y, rot_z
from kinematics.transforms import translation, rot_x, rot_y, rot_z, dh_transform

TEST_ANGLES = [
    ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "home"),
    ([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], "first joint only"),
    ([0.0, -0.4, 0.8, -0.4, 0.0, 0.0], "arm folded, wrist neutral"),
    ([0.0, 0.0, 0.0, 0.0, 0.7, 0.0], "wrist only"),
    ([0.3, -0.2, 0.5, 0.1, 0.6, 1.2], "general pose"),
]


def from_urdf_chain(thetas):
    """Multiply the URDF joint origins directly, without any DH table.

    Use wxai_follower.urdf from the trossen_arm_description package. Every joint origin in it is
    a pure translation, and each revolute joint contributes a rotation about its own axis. Mind
    the axis signs: they are not all positive, and a sign error here looks exactly like a frame
    assignment error in your DH table.

    Remember the fixed segment after joint 6. Flange and Tool are both part of the chain.
    """

    q = np.asarray(thetas, dtype=float)

    if q.shape != (widowx.N_JOINTS,):
        raise ValueError("Expected six joint angles")

    H = np.eye(4)

    # Joint 0: rotation about +Z
    H = H @ translation(0, 0, 0.05725) @ rot_z(q[0])

    # Joint 1: rotation about +Y
    H = H @ translation(0.020, 0, 0.04625) @ rot_y(q[1])

    # Joint 2: rotation about -Y
    H = H @ translation(-0.264, 0, 0) @ rot_y(-q[2])

    # Joint 3: rotation about -Y
    H = H @ translation(0.245, 0, 0.060) @ rot_y(-q[3])

    # Joint 4: rotation about -Z
    H = H @ translation(0.06775, 0, 0.0455) @ rot_z(-q[4])

    # Joint 5: rotation about +X
    H = H @ translation(0.02895, 0, -0.0455) @ rot_x(q[5])

    # Fixed transformation to ee_gripper_link
    H = H @ translation(0.156062, 0, 0)

    return H



def from_manufacturer_table(thetas):
    """The pose computed from Trossen's published DH parameters rather than yours.

    This is the comparison §3.3 asks for, done numerically rather than row by row. Two tables can
    differ row by row and still produce the same tool pose, which is exactly the distinction
    between a different frame assignment and an error.
    """

    q = np.asarray(thetas, dtype=float)

    if q.shape != (6,):
        raise ValueError("Expected six joint angles")

    # Trossen Robotics' published classic DH parameters
    # (theta_offset, d, a, alpha)
    manufacturer_dh = [
        (0.0, 0.1035, 0.020, -np.pi / 2),
        (np.pi, 0.0, 0.264, np.pi),
        (-np.pi / 2 - np.arctan2(0.245, 0.060),
         0.0, np.hypot(0.245, 0.060), 0.0),
        (-np.arctan2(0.060, 0.245),
         0.0, 0.06775, np.pi / 2),
        (np.pi / 2, 0.0, 0.0, np.pi / 2),
        (0.0, 0.02895, 0.0, np.pi / 2),
    ]

    H = np.eye(4)

    for angle, (offset, d, a, alpha) in zip(q, manufacturer_dh):
        H = H @ dh_transform(angle + offset, d, a, alpha)

    # Manufacturer's fixed Flange transformation
    H = H @ dh_transform(np.pi / 2, 0.0, 0.0, 0.0)

    # Manufacturer's default Tool extension
    H = H @ dh_transform(0.0, 0.0, 0.15606, 0.0)

    return H



def main():
    for thetas, label in TEST_ANGLES:
        print(f"\n=== {label}  {np.round(thetas, 3).tolist()} ===")
        mine = fk(thetas)
        print("DH table:\n", np.round(mine, 5))

        for name, fn in (("URDF chain", from_urdf_chain),
                         ("Trossen table", from_manufacturer_table),
                         ("PoE", fk_poe)):
            try:
                other = fn(thetas)
            except NotImplementedError:
                print(f"{name}: not implemented yet")
                continue
            dp = np.linalg.norm(mine[:3, 3] - other[:3, 3])
            dR = np.abs(mine[:3, :3] - other[:3, :3]).max()
            verdict = "AGREE" if (dp < 1e-5 and dR < 1e-5) else "DISAGREE"
            print(f"{name}: {verdict}   position {dp*1000:.4f} mm   rotation {dR:.2e}")


if __name__ == "__main__":
    main()
