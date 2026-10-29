"""Closed-form inverse kinematics of the Trossen WidowX AI Tool frame.

The actual WidowX AI geometry has an offset between wrist axes 4 and 5.
Consequently, the usual textbook 3-axis spherical-wrist decoupling is not
exact for the manufacturer's published DH table.  This solution still uses
geometric (not iterative) IK: subtract the tool extension, solve the waist
and wrist-axis directions, then solve a planar 2-link elbow triangle.

The code uses the team's supplied classical DH frame assignment and
TOOL_OFFSET from robot/widowx.py; all lengths remain in that robot module.
It imports no hardware SDK and never commands the robot.
"""

import numpy as np

from kinematics.fk import fk
from robot import widowx


class Solution:
    """One IK branch, including whether the arm can physically adopt it."""

    def __init__(self, thetas, branch, within_limits, violated_joints=None):
        self.thetas = [float(t) for t in thetas]
        self.branch = str(branch)
        self.within_limits = bool(within_limits)
        self.violated_joints = list(violated_joints or [])

    def __repr__(self):
        label = "OK" if self.within_limits else (
            "LIMITS: " + ", ".join(self.violated_joints)
        )
        angles = ", ".join(f"{a:+.4f}" for a in self.thetas)
        return f"<Solution {self.branch}: [{angles}] {label}>"


def _principal(angle):
    """Return a representative in [-pi, pi)."""
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _limit_preferred(angle, idx):
    """Choose a 2pi-equivalent joint angle within limits, if one exists."""
    v = _principal(float(angle))
    limits = getattr(widowx, "JOINT_LIMITS", None)
    if limits is not None:
        lo, hi = limits[idx]
        choices = (v, v + 2.0 * np.pi, v - 2.0 * np.pi)
        for item in choices:
            if lo - 1e-8 <= item <= hi + 1e-8:
                return float(item)
    return v


def _limit_status(angles):
    """Flag every violating joint using the robot-model JOINT_LIMITS."""
    # The supplied starter robot/widowx.py still has a placeholder
    # within_limits() method. Read the canonical joint-limit data here
    # instead, so IK works without editing the robot adapter first.
    limits = getattr(widowx, "JOINT_LIMITS", None)
    if limits is None:
        raise AttributeError("robot.widowx needs JOINT_LIMITS or within_limits().")
    names = getattr(widowx, "JOINT_NAMES", [f"joint_{i}" for i in range(6)])
    bad = [str(names[i]) for i, (q, (lo, hi)) in enumerate(zip(angles, limits))
           if q < lo - 1e-8 or q > hi + 1e-8]
    return not bad, bad


def _geometry():
    """Read all geometry from robot/widowx.py (no hard-coded lengths here).

    DH_PARAMS rows are (theta_offset, d, a, alpha), in meters/radians,
    with the manufacturer's classical frame assignment.
    """
    params = np.asarray(widowx.DH_PARAMS, dtype=float)
    if params.shape != (6, 4):
        raise ValueError("widowx.DH_PARAMS must be 6 rows of (offset, d, a, alpha).")
    # The team's TOOL_OFFSET translates along frame-6 local z, which
    # becomes the Tool x-axis after its fixed orientation change.
    tool_length = float(widowx.TOOL_OFFSET[2, 3])
    base_height = float(params[0, 1])
    base_radius = float(params[0, 2])
    upper_arm = float(params[1, 2])
    forearm = float(params[2, 2])
    wrist_offset = float(params[3, 2])
    gamma = -float(params[3, 0])
    wrist_to_tool = float(params[5, 1]) + tool_length
    return (base_height, base_radius, upper_arm, forearm,
            wrist_offset, gamma, wrist_to_tool)


def _equivalent(existing, angles, tol=1e-6):
    for sol in existing:
        wrapped_difference = (np.asarray(sol.thetas) - angles + np.pi) % (2*np.pi) - np.pi
        if float(np.linalg.norm(wrapped_difference)) < tol:
            return True
    return False


def ik(T_target):
    """Return all isolated geometric IK branches for a 4x4 Tool target pose.

    Args:
        T_target: Homogeneous Tool pose in the Trossen base_link frame.

    Returns:
        List[Solution]; [] means no valid geometric pose solution was found.
        Both limit-compliant and out-of-limit branches are returned and flagged.

    Typical nonsingular poses have up to eight solutions (two waist choices,
    two wrist-direction choices, and two elbow choices). At singularities
    there may be infinitely many branches; a finite representative sample
    is returned instead of claiming to enumerate an infinite solution set.
    """
    T = np.asarray(T_target, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return []
    R = T[:3, :3]
    if (not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(R), 1.0, atol=1e-5)):
        return []

    (height, a0, upper, lower, wrist_offset,
     gamma, reach_after_wrist) = _geometry()

    # Trossen's Tool x-axis points along the final joint's roll axis.
    # Remove d6 and the fixed Flange -> Tool extension to reach the point
    # at which joints 5 and 6 intersect.  It lies along joint 5's axis.
    tool_axis = R[:, 0]
    wrist_center = T[:3, 3] - reach_after_wrist * tool_axis
    radial_distance = float(np.hypot(wrist_center[0], wrist_center[1]))
    yaw_nominal = float(np.arctan2(wrist_center[1], wrist_center[0]))

    solutions = []
    eps = 1e-8

    # The wrist-center point can be reached with the waist facing forward
    # or flipped 180 deg and a signed planar radial coordinate.
    waist_choices = [(yaw_nominal, radial_distance, "waist-forward"),
                     (yaw_nominal + np.pi, -radial_distance, "waist-flipped")]

    for yaw, radial, waist_branch in waist_choices:
        n_radial = float(tool_axis[0]*np.cos(yaw) + tool_axis[1]*np.sin(yaw))
        n_lateral = float(-tool_axis[0]*np.sin(yaw) + tool_axis[1]*np.cos(yaw))

        # The fifth joint tilts the Tool x-axis sideways out of the planar
        # shoulder/elbow/wrist-angle plane.  This directly determines sin(q4).
        sin_q4 = -n_lateral
        if abs(sin_q4) > 1.0 + eps:
            continue
        q4_primary = float(np.arcsin(np.clip(sin_q4, -1.0, 1.0)))
        wrist_choices = [(q4_primary, "wrist-primary"),
                          (np.pi-q4_primary, "wrist-flipped")]

        for q4, wrist_branch in wrist_choices:
            cos_q4 = float(np.cos(q4))
            if abs(cos_q4) > 1e-7:
                # Direction of final axis constrains the planar tool pitch:
                # n_radial = cos(q4)*cos(pitch)
                # n_vertical = cos(q4)*sin(pitch)
                pitches = [float(np.arctan2(tool_axis[2]/cos_q4,
                                             n_radial/cos_q4))]
                singular = False
            else:
                # At wrist gimbal lock, pitch becomes a free parameter.
                # We can return examples, but no finite list is complete.
                pitches = np.linspace(-np.pi, np.pi, 24, endpoint=False)
                singular = True

            for pitch in pitches:
                # The wrist center belongs to a planar 3R chain; once pitch
                # is known, subtract the last link and solve a 2R triangle.
                target_x = radial - a0 - wrist_offset*np.cos(pitch)
                target_z = wrist_center[2] - height - wrist_offset*np.sin(pitch)
                cos_elbow = ((target_x**2 + target_z**2 - upper**2 - lower**2)
                             / (2.0*upper*lower))
                if abs(cos_elbow) > 1.0 + eps:
                    continue
                angle_mag = float(np.arccos(np.clip(cos_elbow, -1.0, 1.0)))
                elbow_choices = [(angle_mag, "elbow-positive"),
                                 (-angle_mag, "elbow-negative")]

                for delta, elbow_branch in elbow_choices:
                    upper_pitch = (np.arctan2(target_z, target_x)
                                   - np.arctan2(lower*np.sin(delta),
                                                upper+lower*np.cos(delta)))
                    lower_pitch = upper_pitch + delta
                    # Convert the 2R geometric angles back to the six driver
                    # joint variables according to the published classic DH.
                    q1 = np.pi-upper_pitch
                    q2 = lower_pitch + q1-gamma
                    q3 = pitch-lower_pitch+gamma
                    angles = [_limit_preferred(q, i) for i, q in enumerate(
                        (yaw, q1, q2, q3, q4, 0.0))]

                    # After q0..q4, the remaining orientation error is a roll
                    # about the Tool x-axis. Read its angle from that rotation.
                    R_without_roll = fk(angles)[:3, :3]
                    relative = R_without_roll.T @ R
                    q5 = float(np.arctan2(relative[2, 1], relative[1, 1]))
                    angles[5] = _limit_preferred(q5, 5)

                    # Check against the team's FK implementation, not against
                    # the same reduced equations used for IK.
                    T_actual = fk(angles)
                    position_error = float(np.linalg.norm(T_actual[:3, 3]-T[:3, 3]))
                    rotation_error = float(np.linalg.norm(T_actual[:3, :3]-R))
                    if position_error > 1e-6 or rotation_error > 1e-6:
                        continue
                    if _equivalent(solutions, np.asarray(angles)):
                        continue
                    allowed, bad = _limit_status(angles)
                    branch = f"{waist_branch}, {elbow_branch}, {wrist_branch}"
                    if singular:
                        branch += " (wrist singularity: representative)"
                    solutions.append(Solution(angles, branch, allowed, bad))

    return solutions


def unreachable_reason(T_target):
    """Explain what an empty solution list might mean (diagnostic only)."""
    T = np.asarray(T_target, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return "Invalid target: expected a finite 4x4 homogeneous transformation."
    if ik(T):
        return "Geometrically reachable; consult each solution's joint-limit flags."
    return ("No geometric branch reproduces this complete Tool pose. "
            "The target may be out of reach or incompatible with the orientation; "
            "check the FK model and frame conventions first.")
