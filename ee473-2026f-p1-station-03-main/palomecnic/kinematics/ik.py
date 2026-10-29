
"""Inverse kinematics: a reachable target in, every joint solution out.

Read Part C of the project instructions before writing anything here. The RX150 has five
joints and cannot satisfy a general six-constraint pose. This solver takes four constraints,
x, y, z and pitch, and passes roll straight through.
"""

import numpy as np

from robot import rx150


class Solution:
    """One IK solution, with the facts the caller needs to judge it.

    Returning a bare list of angles throws away two things that matter: whether the arm can
    physically adopt the configuration, and which branch it came from. Keep both.
    """

    def __init__(self, thetas, branch, within_limits, violated_joints=None):
        self.thetas = thetas                    # list of five angles, radians
        self.branch = branch                    # e.g. "elbow-up", "elbow-down, waist-flipped"
        self.within_limits = within_limits      # bool
        self.violated_joints = violated_joints or []

    def __repr__(self):
        flag = "ok" if self.within_limits else f"LIMITS: {','.join(self.violated_joints)}"
        angles = ", ".join(f"{t:+.4f}" for t in self.thetas)
        return f"<Solution {self.branch:<28} [{angles}] {flag}>"


def ik(x, y, z, pitch, roll=0.0):
    """All joint solutions reaching (x, y, z) with the given approach pitch.

    Args:
        x, y, z: gripper frame origin in base coordinates, metres.
        pitch: approach angle of the gripper in the vertical plane, radians.
        roll: rotation about the approach axis. Not solved for. Passed through to wrist_rotate.

    Returns:
        A list of Solution objects, possibly empty. Empty means unreachable.

        Return EVERY branch, not one. Elbow up and elbow down are both valid, and the waist has a
        second branch that reaches backwards. A solver that silently picks one is hiding
        information from whoever called it.

        Mark solutions that violate joint limits rather than dropping them. The caller may want
        to know that a target is geometrically reachable but mechanically forbidden; those are
        different failures and deserve different fixes.

        For an unreachable target return an empty list. Never return a nearby pose as though it
        were the answer, and never return NaN.
    """

    # Verify that the target coordinates and angles are valid numbers.
    values = np.array([x, y, z, pitch, roll], dtype=float)

    if not np.all(np.isfinite(values)):
        raise ValueError("Target values must be finite.")

    # Get the robot dimensions and angular offset from rx150.py.
    shoulder_height = rx150.DH_PARAMS[0][1]
    L1 = rx150.DH_PARAMS[1][2]
    L2 = rx150.DH_PARAMS[2][2]
    beta = -rx150.DH_PARAMS[1][0]

    # Include the wrist length and fixed gripper extension.
    tool_length = (
        rx150.DH_PARAMS[4][1]
        + rx150.TOOL_OFFSET[2, 3]
    )

    # Keep joint angles within the range -pi to pi.
    def wrap(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    # Store all possible IK solutions.
    solutions = []

    # Calculate the waist angle and horizontal distance to the target.
    base_angle = np.arctan2(y, x)
    radius = np.hypot(x, y)

    # Check both the normal and waist-flipped configurations.
    waist_options = [
        (base_angle, radius, "normal waist"),
        (wrap(base_angle + np.pi), -radius, "waist-flipped"),
    ]

    for waist, signed_radius, waist_name in waist_options:

        # Subtract the tool extension to locate the wrist center.
        rw = signed_radius - tool_length * np.cos(pitch)
        zw = z - shoulder_height - tool_length * np.sin(pitch)

        # Use the law of cosines to calculate the elbow angle.
        D = (
            (rw**2 + zw**2 - L1**2 - L2**2)
            / (2 * L1 * L2)
        )

        # Skip this waist branch if the wrist center is unreachable.
        if abs(D) > 1.0 + 1e-10:
            continue

        # Keep D within the valid range for arccos.
        D = np.clip(D, -1.0, 1.0)

        elbow_angle = np.arccos(D)

        # Calculate both elbow-up and elbow-down solutions.
        for sign, elbow_name in [
            (1, "elbow-up"),
            (-1, "elbow-down"),
        ]:

            q_elbow = sign * elbow_angle

            # Calculate the shoulder angle using the wrist-center position.
            q_shoulder = (
                np.arctan2(-zw, rw)
                - np.arctan2(
                    L2 * np.sin(q_elbow),
                    L1 + L2 * np.cos(q_elbow)
                )
            )

            # Account for the fixed angular offsets in the DH table.
            shoulder = q_shoulder + beta
            elbow = q_elbow - beta

            # Adjust the wrist angle to maintain the requested pitch.
            wrist_angle = -pitch - shoulder - elbow

            # Combine the five calculated joint angles.
            thetas = [
                wrap(waist),
                wrap(shoulder),
                wrap(elbow),
                wrap(wrist_angle),
                wrap(roll),
            ]

            # Avoid returning the same joint configuration twice.
            if any(
                np.allclose(thetas, s.thetas, atol=1e-8)
                for s in solutions
            ):
                continue

            # Check which solutions satisfy the robot's joint limits.
            allowed, violated = rx150.within_limits(thetas)

            # Save the solution along with its branch and limit status.
            solutions.append(
                Solution(
                    thetas=thetas,
                    branch=f"{waist_name}, {elbow_name}",
                    within_limits=allowed,
                    violated_joints=violated,
                )
            )

    # Return all geometric solutions, including any outside joint limits.
    return solutions


def unreachable_reason(x, y, z, pitch):
    """Why a target cannot be reached, as a short human-readable string.

    "Outside maximum reach: 0.412 m requested, 0.358 m available" is useful.
    "No solution" is not.
    """

    # Verify that the target coordinates and pitch are valid numbers.
    values = np.array([x, y, z, pitch], dtype=float)

    if not np.all(np.isfinite(values)):
        return "Invalid target: values must be finite."

    # Get the robot dimensions from rx150.py.
    shoulder_height = rx150.DH_PARAMS[0][1]
    L1 = rx150.DH_PARAMS[1][2]
    L2 = rx150.DH_PARAMS[2][2]

    # Include the wrist length and fixed gripper extension.
    tool_length = (
        rx150.DH_PARAMS[4][1]
        + rx150.TOOL_OFFSET[2, 3]
    )

    # Calculate the minimum and maximum reach of the two arm links.
    min_reach = abs(L1 - L2)
    max_reach = L1 + L2

    # Calculate the horizontal distance to the target.
    radius = np.hypot(x, y)

    # Calculate the wrist-center height relative to the shoulder.
    zw = z - shoulder_height - tool_length * np.sin(pitch)

    # Store the wrist-center distances for both waist configurations.
    distances = []

    for signed_radius in [radius, -radius]:

        # Remove the tool extension to locate the wrist center.
        rw = signed_radius - tool_length * np.cos(pitch)

        # Calculate the distance from the shoulder to the wrist center.
        distance = np.hypot(rw, zw)

        distances.append(distance)

    # Return None if at least one waist branch is geometrically reachable.
    for distance in distances:
        if min_reach - 1e-10 <= distance <= max_reach + 1e-10:
            return None

    # Find the wrist-center distance closest to the reachable range.
    distance = min(
        distances,
        key=lambda d: max(min_reach - d, d - max_reach, 0)
    )

    # Explain when the target is beyond the maximum reach.
    if distance > max_reach:
        return (
            f"Outside maximum reach: {distance:.3f} m requested, "
            f"{max_reach:.3f} m available"
        )

    # Explain when the target is inside the minimum reach.
    return (
        f"Inside minimum reach: {distance:.3f} m requested, "
        f"{min_reach:.3f} m minimum"
    )
