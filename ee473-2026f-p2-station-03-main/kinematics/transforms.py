"""Frame algebra. Nothing in this file knows which robot it is operating on.

Any dimension particular to the WidowX AI belongs in robot/widowx.py.
"""

import numpy as np


def dh_transform(theta, d, a, alpha):
    """One Denavit-Hartenberg link transform, as a 4x4 homogeneous matrix.

    Which four elementary operations this is a product of, and in what order, depends on whether
    the classical or the modified convention is in use. Implement the one declared in
    robot/widowx.py, and implement it once: every joint passes through this function.

    Angles in radians, lengths in metres.
    """

def dh_transform(theta, d, a, alpha):
    """Return the classical DH transformation matrix."""

    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ], dtype=float)




def rot_x(angle):
    """Rotation about x, as a 4x4."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [1, 0,  0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0,  0, 1]
    ], dtype=float)


def rot_y(angle):
    """Rotation about y, as a 4x4."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]
    ], dtype=float)


def rot_z(angle):
    """Rotation about z, as a 4x4."""
    c = np.cos(angle)
    s = np.sin(angle)

    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ], dtype=float)




def translation(x, y, z):
    """Pure translation, as a 4x4."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ], dtype=float)




def inverse(H):
    """Inverse of a homogeneous transform."""

    R = H[:3, :3]
    p = H[:3, 3]

    H_inv = np.eye(4)

    H_inv[:3, :3] = R.T
    H_inv[:3, 3] = -R.T @ p

    return H_inv



def pose_to_xyz_rpy(H):
    """Convenience: pull (x, y, z, roll, pitch, yaw) out of a 4x4 for printing and comparison.

    Observe the convention. Fixed-axis roll-pitch-yaw and Euler angles are distinct
    parameterizations, and whichever is adopted here governs the measurements as well.
    """

    # Get the position from the last column
    x, y, z = H[:3, 3]

    # Get the rotation part of the matrix
    R = H[:3, :3]

    # Calculate pitch
    sy = np.hypot(R[0, 0], R[1, 0])
    pitch = np.arctan2(-R[2, 0], sy)

    if sy > 1e-8:
        # Normal case
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock: roll and yaw are not independent
        roll = 0.0
        yaw = np.arctan2(-R[0, 1], R[1, 1])

    return np.array([x, y, z, roll, pitch, yaw])



def pose_from_xyz_rpy(x, y, z, roll, pitch, yaw):
    """Build a 4x4 pose from a position and three orientation angles.

    This is how a target is specified. `ik()` accepts a 4x4 transformation, and Part C begins
    here: select a position and an approach orientation, construct the matrix, and solve it.

    Compose it from the helpers above rather than writing out the entries directly.

    Fix the convention here and state it in your report. Fixed-axis roll-pitch-yaw and Euler
    angles are different parameterizations, and the order in which the three rotations are
    applied is part of the convention. Whichever is adopted, pose_to_xyz_rpy must be its exact
    inverse, and measurements must be taken under the same one.
    """

    H = translation(x, y, z)

    H = H @ rot_z(yaw)
    H = H @ rot_y(pitch)
    H = H @ rot_x(roll)

    return H



def poses_close(H1, H2, pos_tol=1e-6, rot_tol=1e-6):
    """True if two poses agree to within tolerance.

    Used by the round-trip test. An element-by-element comparison of 4x4 matrices mixes metres
    with dimensionless rotation entries, so separate the two: compare the translation columns in
    metres and the rotation blocks by angle.
    """

    # Calculate the distance between the two positions
    p1 = H1[:3, 3]
    p2 = H2[:3, 3]
    pos_error = np.linalg.norm(p1 - p2)

    # Get the rotation matrices
    R1 = H1[:3, :3]
    R2 = H2[:3, :3]

    # Calculate the difference in orientation
    R_error = R1.T @ R2

    # Convert the rotation difference into an angle
    cos_angle = (np.trace(R_error) - 1.0) / 2.0
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    rot_error = np.arccos(cos_angle)

    # Both errors must be within their tolerances
    return bool(pos_error <= pos_tol and rot_error <= rot_tol)
