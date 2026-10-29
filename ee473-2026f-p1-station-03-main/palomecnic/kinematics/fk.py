
"""Forward kinematics: joint angles in, tool pose out."""

import numpy as np

from kinematics.transforms import dh_transform
from robot import rx150


def fk(thetas):
    """Pose of the gripper frame in the base frame.

    Args:
        thetas: five joint angles in radians, in robot.rx150.JOINT_NAMES order.

    Returns:
        4x4 numpy array, the gripper frame expressed in the base frame.

    Raises:
        ValueError: if the wrong number of angles is given. Check this. A silent length mismatch
            is a long afternoon.
    """

    # Convert the joint angles to a NumPy array.
    angles = np.asarray(thetas, dtype=float)

    # Verify all five joint angles.
    if angles.shape != (rx150.N_JOINTS,):
        raise ValueError("Expected five joint angles")

    # Start with a 4x4 identity transformation (base frame).
    H = np.eye(4)

    # Calculate each joint transformation using the DH parameters.
    for theta, (offset, d, a, alpha) in zip(angles, rx150.DH_PARAMS):
        A = dh_transform(theta + offset, d, a, alpha)
        H = H @ A

    # Include the fixed transformation from the last joint to the gripper.
    H = H @ rx150.TOOL_OFFSET

    # Return the final 4x4 homogeneous transformation matrix.
    return H


def fk_all_frames(thetas):
    """Every intermediate frame, not just the tool.

    Returns a list of 4x4 matrices: the pose of each joint frame in the base frame, ending with
    the gripper frame.

    You need this for Part B. When your FK disagrees with RViz you have to find out *where* the
    chain diverges, and a single end pose cannot tell you that. Comparing frame by frame can.
    """

    # Convert the joint angles to a NumPy array.
    angles = np.asarray(thetas, dtype=float)

    # Verify all five joint angles.
    if angles.shape != (rx150.N_JOINTS,):
        raise ValueError("Expected five joint angles")

    # Start with a 4x4 identity transformation (base frame).
    H = np.eye(4)

    # Create a list to store each frame's transformation matrix.
    frames = []

    # Calculate each joint transformation using the DH parameters.
    for theta, (offset, d, a, alpha) in zip(angles, rx150.DH_PARAMS):
        A = dh_transform(theta + offset, d, a, alpha)
        H = H @ A

        # Save the current joint frame for comparison with RViz.
        frames.append(H.copy())

    # Include the fixed transformation from the last joint to the gripper.
    H = H @ rx150.TOOL_OFFSET

    # Save the final gripper frame.
    frames.append(H.copy())

    # Return five joint frames plus the gripper frame.
    return frames


def fk_poe(thetas):
    """The same pose, computed by product of exponentials instead of DH.

    This is the third verification in Part B. Write the screw axes in the base frame at the
    home configuration, then

        H = exp([S1]th1) exp([S2]th2) ... exp([S5]th5) M

    The point of doing it twice is that a frame assignment error cannot hide in both methods, so
    agreement is real evidence rather than a repeated mistake.

    You may use modern_robotics.FKinSpace here, or write the exponential yourself. If you use the
    library, say so in your README.
    """

    # Convert the joint angles to a NumPy array.
    angles = np.asarray(thetas, dtype=float)

    # Verify all five joint angles.
    if angles.shape != (rx150.N_JOINTS,):
        raise ValueError("Expected five joint angles")

    # Start with a 4x4 identity transformation (base frame).
    H = np.eye(4)

    # Calculate each joint transformation using its screw axis.
    for theta, screw in zip(angles, rx150.POE_SCREW_AXES):

        # Separate the angular and linear components of the screw axis.
        omega = screw[:3]
        v = screw[3:]

        wx, wy, wz = omega

        # Create the skew-symmetric matrix for the rotation axis.
        W = np.array([
            [0, -wz, wy],
            [wz, 0, -wx],
            [-wy, wx, 0]
        ], dtype=float)

        # Start with a 3x3 identity matrix.
        I = np.eye(3)

        # Calculate the joint rotation using Rodrigues' formula.
        R = I + np.sin(theta) * W \
              + (1 - np.cos(theta)) * (W @ W)

        # Calculate the translation part of the matrix exponential.
        G = theta * I \
            + (1 - np.cos(theta)) * W \
            + (theta - np.sin(theta)) * (W @ W)

        p = G @ v

        # Combine rotation and translation into a 4x4 transformation.
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = p

        # Multiply the current joint transformation.
        H = H @ T

    # Include the gripper's home configuration.
    H = H @ rx150.POE_HOME

    # Return the final 4x4 homogeneous transformation matrix.
    return H
