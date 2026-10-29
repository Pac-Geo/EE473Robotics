
"""Frame algebra. Nothing in this file knows which robot it is operating on.

If you find yourself wanting to put an RX150 dimension in here, it belongs in robot/rx150.py.
"""

import numpy as np


def dh_transform(theta, d, a, alpha):
    """One Denavit-Hartenberg link transform, as a 4x4 homogeneous matrix.

    Which four elementary operations this is a product of, and in what order, depends on whether
    you are using classic or modified DH. Write the one you declared in robot/rx150.py, and write
    it once: every joint goes through this function.

    Angles in radians, lengths in metres.
    """

    # Calculate the sine and cosine of the DH angles.
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    # Classic DH: Rz(theta) @ Tz(d) @ Tx(a) @ Rx(alpha).
    A = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d],
        [0,   0,        0,       1]
    ], dtype=float)

    return A


def rot_x(angle):
    """Rotation about x, as a 4x4."""

    # Calculate the rotation angle.
    c = np.cos(angle)
    s = np.sin(angle)

    # Rotate about the x-axis.
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ], dtype=float)


def rot_y(angle):
    """Rotation about y, as a 4x4."""

    # Calculate the rotation angle.
    c = np.cos(angle)
    s = np.sin(angle)

    # Rotate about the y-axis.
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]
    ], dtype=float)


def rot_z(angle):
    """Rotation about z, as a 4x4."""

    # Calculate the rotation angle.
    c = np.cos(angle)
    s = np.sin(angle)

    # Rotate about the z-axis.
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=float)


def translation(x, y, z):
    """Pure translation, as a 4x4."""

    # Start with an identity matrix.
    H = np.eye(4)

    # Set the displacement along the x, y, and z axes.
    H[0, 3] = x
    H[1, 3] = y
    H[2, 3] = z

    return H


def inverse(H):
    """Inverse of a homogeneous transform.

    Do not call np.linalg.inv. A homogeneous transform inverts in closed form, using the fact
    that the rotation block is orthogonal. Work out what that gives you and implement it; it is
    both faster and more numerically stable, and the derivation is worth doing once.
    """

    # Separate the rotation matrix and translation vector.
    R = H[:3, :3]
    p = H[:3, 3]

    # Start with an identity matrix.
    H_inv = np.eye(4)

    # Inverse rotation is the transpose of R.
    H_inv[:3, :3] = R.T

    # Inverse translation is -R.T @ p.
    H_inv[:3, 3] = -R.T @ p

    return H_inv


def pose_to_xyz_rpy(H):
    """Convenience: pull (x, y, z, roll, pitch, yaw) out of a 4x4 for printing and comparison.

    Watch the convention. Fixed-axis roll-pitch-yaw and Euler angles are not the same thing, and
    whichever you pick here you must use in your measurements too.
    """

    # Extract the position and rotation matrix.
    x, y, z = H[:3, 3]
    R = H[:3, :3]

    # Fixed-axis XYZ convention:
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    # Calculate pitch from the rotation matrix.
    pitch = np.arctan2(
        -R[2, 0],
        np.hypot(R[0, 0], R[1, 0])
    )

    # Calculate roll and yaw for the normal case.
    if abs(np.cos(pitch)) > 1e-9:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])

    else:
        # At gimbal lock, roll and yaw are not independently defined.
        # Choose yaw = 0 and calculate the matching roll.
        yaw = 0.0

        if pitch > 0:
            roll = np.arctan2(R[0, 1], R[1, 1])
        else:
            roll = np.arctan2(-R[0, 1], R[1, 1])

    return x, y, z, roll, pitch, yaw


def poses_close(H1, H2, pos_tol=1e-6, rot_tol=1e-6):
    """True if two poses agree to within tolerance.

    Used by the round-trip test. Comparing 4x4 matrices element by element mixes metres with
    dimensionless rotation entries, so separate them: compare the translation columns in metres
    and the rotation blocks by angle.
    """

    # Extract the position from both transformations.
    p1 = H1[:3, 3]
    p2 = H2[:3, 3]

    # Calculate the position difference in metres.
    position_error = np.linalg.norm(p1 - p2)

    # Extract the rotation matrices.
    R1 = H1[:3, :3]
    R2 = H2[:3, :3]

    # Calculate the relative rotation.
    R_diff = R1.T @ R2

    # Determine the rotation difference in radians.
    cos_angle = (np.trace(R_diff) - 1) / 2

    # Prevent floating-point errors outside the valid arccos range.
    cos_angle = np.clip(cos_angle, -1.0, 1.0)

    rotation_error = np.arccos(cos_angle)

    # Both position and orientation must satisfy their tolerances.
    return position_error <= pos_tol and rotation_error <= rot_tol
