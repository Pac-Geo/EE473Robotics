# Move the RX150 using joint angles and preview the result with forward kinematics.

from pathlib import Path
import subprocess
import sys

import numpy as np

# Allow imports from the individual submission folder, even if this file is
# launched from a different working directory.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from kinematics.fk import fk
from robot import rx150


COMMAND_TOPIC = "/rx150/commands/joint_group"
COMMAND_TYPE = "interbotix_xs_msgs/msg/JointGroupCommand"


def check_joint_limits(angles):
    """Return the names of joints outside their configured limits."""
    angles = np.asarray(angles, dtype=float)
    if angles.shape != (rx150.N_JOINTS,) or not np.all(np.isfinite(angles)):
        raise ValueError("Expected five finite joint angles.")

    bad_joints = []
    for name, angle, (lower, upper) in zip(
        rx150.JOINT_NAMES, angles, rx150.JOINT_LIMITS
    ):
        if not lower <= angle <= upper:
            bad_joints.append(name)
    return bad_joints


def command_arm(angles):
    """Publish one joint-position command using the tested Interbotix topic."""
    message = "{name: arm, cmd: [" + ", ".join(map(str, angles)) + "]}"
    subprocess.run(
        [
            "ros2", "topic", "pub", "--once",
            COMMAND_TOPIC,
            COMMAND_TYPE,
            message,
        ],
        check=True,
        timeout=15,
    )


def main():
    print("\nRX150 Forward Kinematics Control\n")
    print("Enter a desired angle for each joint (degrees).")
    print("Joint order:", ", ".join(rx150.JOINT_NAMES))

    try:
        degrees = [
            float(input(f"{name} (degrees): ")) for name in rx150.JOINT_NAMES
        ]
        angles = np.deg2rad(degrees)
        bad_joints = check_joint_limits(angles)
    except (ValueError, EOFError, KeyboardInterrupt) as error:
        print("\nInvalid or interrupted input. Robot not moved.", error)
        return

    if bad_joints:
        print("\nJoint limit exceeded:", ", ".join(bad_joints))
        print("Robot not moved.")
        return

    try:
        pose = np.asarray(fk(angles), dtype=float)
        if pose.shape != (4, 4) or not np.all(np.isfinite(pose)):
            raise ValueError("FK returned an invalid transformation matrix.")
    except (ValueError, TypeError) as error:
        print("\nForward kinematics failed. Robot not moved:", error)
        return

    print("\nJoint angles (rad):", np.round(angles, 4))
    print("Predicted gripper position (mm):", np.round(pose[:3, 3] * 1000, 2))
    print("Predicted gripper rotation matrix:\n", np.round(pose[:3, :3], 4))

    print("\nCheck the complete motion path in RViz before commanding the arm.")
    print("Ensure the workspace is clear and the emergency/power stop is accessible.")
    print("The script checks joint limits, not collisions or the motion path.")

    try:
        confirm = input("\nCommand physical RX150? (y/n): ").strip().lower()
    except (EOFError, KeyboardInterrupt):
        print("\nMovement cancelled.")
        return

    if confirm != "y":
        print("Movement cancelled.")
        return

    try:
        command_arm(angles)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError) as error:
        print("Movement command failed:", error)
        return

    print("\nMovement command published.")
    print("Confirm actual motion and final joint positions using robot feedback.")


if __name__ == "__main__":
    main()
