"""Forward kinematics: joint angles in, tool pose out."""

import numpy as np

from kinematics.transforms import dh_transform
from robot import widowx


def fk(thetas):
    """Pose of the Tool frame in the base frame.

    Args:
        thetas: six joint angles in radians, in robot.widowx.JOINT_NAMES order.

    Returns:
        4x4 numpy array, the Tool frame expressed in the base frame.

    Raises:
        ValueError: if the wrong number of angles is given. Check this. Coming from a five-joint
            arm, a length mismatch is the first mistake waiting to happen.
    """

    # Make sure we have exactly six joint angles
    thetas = np.asarray(thetas, dtype=float)

    if thetas.shape != (widowx.N_JOINTS,):
        raise ValueError("Expected six joint angles")

    # Start with the identity matrix
    H = np.eye(4)

    # Multiply the six DH transformation matrices
    for angle, (offset, d, a, alpha) in zip(thetas, widowx.DH_PARAMS):
        H = H @ dh_transform(angle + offset, d, a, alpha)

    # Include the fixed transformation to the Tool frame
    H = H @ widowx.TOOL_OFFSET

    return H



def fk_all_frames(thetas):
    """Every intermediate frame, not just the tool.

    Returns a list of 4x4 matrices: the pose of each joint frame in the base frame, ending with
    the Tool frame.

    When your FK disagrees with the manufacturer you have to find out *where* the chain diverges,
    and a single end pose cannot tell you that. Comparing frame by frame can.
    """

    # Convert the joint angles to a numpy array
    thetas = np.asarray(thetas, dtype=float)

    # Check that we received six joint angles
    if thetas.shape != (widowx.N_JOINTS,):
        raise ValueError("Expected six joint angles")

    H = np.eye(4)
    frames = []

    # Calculate the transformation after each joint
    for angle, (offset, d, a, alpha) in zip(thetas, widowx.DH_PARAMS):
        H = H @ dh_transform(angle + offset, d, a, alpha)

        # Save a copy of the current frame
        frames.append(H.copy())

    # Add the final Tool frame
    frames.append(H @ widowx.TOOL_OFFSET)

    return frames



def fk_poe(thetas):
    """The same pose, computed by product of exponentials instead of DH.

    An independent check: a frame assignment error cannot hide in both methods, so agreement is
    real evidence rather than a repeated mistake.

    You may use modern_robotics.FKinSpace here, or write the exponential yourself. If you use the
    library, say so in your report.
    """

    thetas = np.asarray(thetas, dtype=float)

    if thetas.shape != (widowx.N_JOINTS,):
        raise ValueError("Expected six joint angles")

    H = np.eye(4)

    for angle, screw in zip(thetas, widowx.POE_SCREWS):
        w = screw[:3]
        v = screw[3:]

        # Skew-symmetric matrix for the rotation axis
        W = np.array([
            [0, -w[2], w[1]],
            [w[2], 0, -w[0]],
            [-w[1], w[0], 0]
        ], dtype=float)

        # Rotation using Rodrigues' formula
        R = (np.eye(3)
             + np.sin(angle) * W
             + (1 - np.cos(angle)) * (W @ W))

        # Translation from the matrix exponential
        V = (angle * np.eye(3)
             + (1 - np.cos(angle)) * W
             + (angle - np.sin(angle)) * (W @ W))

        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = V @ v

        H = H @ T

    # Apply the zero-configuration Tool pose
    return H @ widowx.POE_HOME
