"""Compare RX150 forward kinematics using the DH, URDF, and PoE methods.

Checks four joint configurations and prints the differences between the results.
RViz/tf2 verification is handled separately rather than by this script.

    python scripts/verify_fk.py
"""

import numpy as np

from kinematics.fk import fk, fk_poe
from robot import rx150

TEST_ANGLES = [
    ([0.0, 0.0, 0.0, 0.0, 0.0], "home"),
    ([0.5, 0.0, 0.0, 0.0, 0.0], "waist only"),
    ([0.0, -0.4, 0.8, -0.4, 0.0], "arm folded, wrist level"),
    ([0.3, -0.2, 0.5, 0.1, 1.2], "general pose"),
]


def from_urdf_chain(thetas):
    """Calculate the gripper pose directly from the URDF chain."""

    angles = np.asarray(thetas, dtype=float)

    if angles.shape != (rx150.N_JOINTS,):
        raise ValueError("Expected five joint angles.")

    H = np.eye(4)
    joint_index = 0

    for origin, axis in rx150.URDF_CHAIN:

        # Translation from parent link to child link
        T = np.eye(4)
        T[:3, 3] = origin
        H = H @ T

        # Fixed joints only have a translation
        if axis is None:
            continue

        theta = angles[joint_index]
        joint_index += 1

        # Build the rotation using Rodrigues' formula
        wx, wy, wz = axis

        W = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ], dtype=float)

        R = (np.eye(3)
             + np.sin(theta) * W
             + (1 - np.cos(theta)) * (W @ W))

        T_rot = np.eye(4)
        T_rot[:3, :3] = R

        H = H @ T_rot

    return H


def from_rviz(thetas):
    """Pose reported by the robot state publisher for the same angles.

    Verification 1. Publish the joint states, then look up base_link to ee_gripper_link through
    tf2. If you prefer, record the numbers by hand from RViz and paste them in as a literal
    table; say in your README which you did.
    """
    raise NotImplementedError


def main():
    for thetas, label in TEST_ANGLES:
        print(f"\n=== {label}  {np.round(thetas, 3).tolist()} ===")
        mine = fk(thetas)
        print("DH table:\n", np.round(mine, 5))

        for name, fn in (("URDF chain", from_urdf_chain),
                         ("RViz / tf2", from_rviz),
                         ("PoE", lambda t: fk_poe(t))):
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
