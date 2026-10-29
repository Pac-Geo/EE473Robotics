
"""Interactive inverse kinematics control for the RX150."""

import subprocess
import numpy as np

from kinematics.ik import ik, unreachable_reason
from kinematics.fk import fk


def main():
    print("\nRX150 Inverse Kinematics Control\n")

    try:
        x = float(input("Enter X (mm): ")) / 1000
        y = float(input("Enter Y (mm): ")) / 1000
        z = float(input("Enter Z (mm): ")) / 1000
        pitch = np.deg2rad(float(input("Enter Pitch (degrees): ")))
        roll = np.deg2rad(float(input("Enter Roll (degrees): ")))
    except ValueError:
        print("Invalid input. Robot not moved.")
        return

    solutions = ik(x, y, z, pitch, roll)
    valid = [s for s in solutions if s.within_limits]

    if not valid:
        print("\nNo joint-limit-compliant IK solution.")
        print(unreachable_reason(x, y, z, pitch))
        return

    print("\nAvailable valid solutions:")

    for i, solution in enumerate(valid, 1):
        print(f"\n{i}. {solution.branch}")
        print("Joint angles (rad):", np.round(solution.thetas, 4))

    try:
        choice = int(input("\nSelect solution number: ")) - 1
        if not 0 <= choice < len(valid):
            raise ValueError
    except ValueError:
        print("Invalid selection. Robot not moved.")
        return

    selected = valid[choice]
    angles = selected.thetas

    predicted = fk(angles)[:3, 3] * 1000

    print("\nPredicted gripper position (mm):")
    print(np.round(predicted, 2))

    print("\nReview the motion path in RViz.")
    print("Ensure the workspace is clear.")
    print("Have someone ready at the power switch.")

    confirm = input("\nCommand physical RX150? (y/n): ")

    if confirm.strip().lower() != "y":
        print("Movement cancelled.")
        return

    message = "{name: arm, cmd: [" + ", ".join(map(str, angles)) + "]}"

    try:
        subprocess.run(
            [
                "ros2",
                "topic",
                "pub",
                "--once",
                "/rx150/commands/joint_group",
                "interbotix_xs_msgs/msg/JointGroupCommand",
                message,
            ],
            check=True,
            timeout=15,
        )
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError) as error:
        print("Movement command failed:", error)
        return

    print("\nMovement command published.")
    print("Confirm actual robot position using joint feedback.")


if __name__ == "__main__":
    main()
