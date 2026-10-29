
"""Everything that knows this is an RX150.

This is the **only** file in the project that may contain hardware-specific numbers. Joint
limits, link lengths, DH parameters, the gripper frame definition: all of it lives here, and
nothing outside this file may hard-code a dimension.

The reason is Project 3, where the arm changes. If you have kept this boundary, you swap this one
file and your kinematics library keeps working. If you have scattered link lengths through
`fk.py`, you rewrite everything.

Fill in the values yourself from your Part A derivation. The structure is given; the numbers are
the assignment.
"""

import numpy as np

# Joint order. Every list of angles in this project is in this order, without exception.
JOINT_NAMES = [
    "waist",
    "shoulder",
    "elbow",
    "wrist_angle",
    "wrist_rotate",
]

N_JOINTS = len(JOINT_NAMES)


# ----------------------------------------------------------------------------
# DH parameters
# ----------------------------------------------------------------------------
# One row per joint, in JOINT_NAMES order. Use the convention you assigned in Part A and say
# which one it is, because classic and modified DH do not interchange.
#
# Each row is the CONSTANT part of the row. The joint variable is added at evaluation time:
# theta_i for a revolute joint, d_i for a prismatic one. The RX150 is all revolute.
#
#   (theta_offset, d, a, alpha)      angles in radians, lengths in metres

DH_CONVENTION = "classic"  # "classic" or "modified". State which. It changes the transform.

# Calculate the angular offset from the diagonal shoulder-to-elbow link.
beta = np.arctan2(0.15, 0.05)

# DH parameters from the team's frame assignment.
DH_PARAMS = [
    # (theta_offset, d, a, alpha)
    (0.0,        0.10457, 0.0,                 -np.pi / 2),  # waist
    (-beta,      0.0,     np.hypot(0.05, 0.15), 0.0),        # shoulder
    (beta,       0.0,     0.150,                0.0),        # elbow
    (-np.pi / 2, 0.0,     0.0,                -np.pi / 2),  # wrist_angle
    (0.0,        0.065,   0.0,                 0.0),        # wrist_rotate
]


# ----------------------------------------------------------------------------
# Tool frame
# ----------------------------------------------------------------------------
# The URDF puts four FIXED joints between wrist_rotate and the tool frame, adding 0.093575 m in
# total. Decide where your gripper frame sits, write it here once, and use the same definition in
# your derivation, your code, and your tape measure. Most large errors in this project are a
# model and a measurement pointing at two different places.

# Transformation from the final DH frame to ee_gripper_link.
TOOL_OFFSET = np.array([
    [0,  0, 1, 0.0],
    [0, -1, 0, 0.0],
    [1,  0, 0, 0.093575],
    [0,  0, 0, 1.0]
], dtype=float)


# ----------------------------------------------------------------------------
# URDF reference geometry
# ----------------------------------------------------------------------------
# Each row contains the joint origin translation and rotation axis.
# Fixed joints have no rotation axis.

URDF_CHAIN = [
    ((0.0, 0.0, 0.06566),  (0, 0, 1)),  # waist
    ((0.0, 0.0, 0.03891),  (0, 1, 0)),  # shoulder
    ((0.05, 0.0, 0.15),    (0, 1, 0)),  # elbow
    ((0.15, 0.0, 0.0),     (0, 1, 0)),  # wrist_angle
    ((0.065, 0.0, 0.0),    (1, 0, 0)),  # wrist_rotate
    ((0.043, 0.0, 0.0),    None),       # ee_arm
    ((0.0, 0.0, 0.0),      None),       # gripper_bar
    ((0.023, 0.0, 0.0),    None),       # ee_bar
    ((0.027575, 0.0, 0.0), None),      # ee_gripper
]


# ----------------------------------------------------------------------------
# Product of Exponentials
# ----------------------------------------------------------------------------
# Each row contains [wx, wy, wz, vx, vy, vz].
# Screw axes are expressed in the base frame at home.

POE_SCREW_AXES = np.array([
    [0, 0, 1,  0.0,      0.0,     0.0],   # waist
    [0, 1, 0, -0.10457,  0.0,     0.0],   # shoulder
    [0, 1, 0, -0.25457,  0.0,     0.05],  # elbow
    [0, 1, 0, -0.25457,  0.0,     0.20],  # wrist_angle
    [1, 0, 0,  0.0,      0.25457, 0.0],   # wrist_rotate
], dtype=float)

# Gripper pose when all five joint angles are zero.
POE_HOME = np.array([
    [1, 0, 0, 0.358575],
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.254570],
    [0, 0, 0, 1.0]
], dtype=float)


# ----------------------------------------------------------------------------
# Joint limits
# ----------------------------------------------------------------------------
# From the manufacturer, in radians. Do not guess these: read them from the URDF or the Trossen
# documentation. Part C requires you to flag solutions that violate them.

JOINT_LIMITS = [
    # (lower, upper)
    (np.deg2rad(-180), np.deg2rad(180)),  # waist
    (np.deg2rad(-106), np.deg2rad(100)),  # shoulder
    (np.deg2rad(-102), np.deg2rad(95)),   # elbow
    (np.deg2rad(-100), np.deg2rad(123)),  # wrist_angle
    (np.deg2rad(-180), np.deg2rad(180)),  # wrist_rotate
]


def within_limits(thetas):
    """True if every joint angle is inside its limit.

    Returns a (bool, list_of_offending_joint_names) pair so the caller can report *which* joint
    is out of range rather than just that something is.
    """

    # Convert the joint angles to a NumPy array.
    angles = np.asarray(thetas, dtype=float)

    # Verify all five joint angles.
    if angles.shape != (N_JOINTS,):
        raise ValueError("Expected five joint angles.")

    # Store the names of joints that violate their limits.
    violated = []

    # Check each joint angle against its lower and upper limits.
    for name, theta, (lower, upper) in zip(
        JOINT_NAMES, angles, JOINT_LIMITS
    ):
        if not np.isfinite(theta) or theta < lower or theta > upper:
            violated.append(name)

    # Return whether all joints are valid and identify any violations.
    return len(violated) == 0, violated


def home():
    """The all-zeros configuration, as a list of N_JOINTS angles."""
    return [0.0] * N_JOINTS


# ----------------------------------------------------------------------------
# Hardware
# ----------------------------------------------------------------------------
# Part D only. Everything below talks to the physical arm, and this is the only file in the
# project allowed to. Nothing in kinematics/ may import any of it.
#
# Import the Interbotix packages INSIDE the functions, not at the top of this file. The test
# suite has to run on a machine with no arm attached, and a module-level import would make
# `pytest` fail at collection with an ImportError that has nothing to do with your code.
#
# Before any of this will work, the driver has to be running in another terminal:
#
#   ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=rx150
#
# See section 4 of the project instructions for the arm handling rules. The arm collapses when
# the motors are released, so send it to the sleep pose before you disconnect.


class Arm:
    """A connection to the physical RX150.

    Use it as a context manager so the arm is always put away, including when your script
    raises partway through:

        with Arm() as arm:
            arm.send(my_solution)
            measured = arm.read()

    Implement `__enter__` with `InterbotixManipulatorXS` and `robot_startup()`, and `__exit__`
    with `go_to_sleep_pose()` followed by `robot_shutdown()`.
    """

    def __init__(self, moving_time=3.0, accel_time=1.0):
        self.moving_time = moving_time
        self.accel_time = accel_time
        self.bot = None

    def __enter__(self):

        # Import the ROS 2 hardware interface only when connecting.
        from interbotix_common_modules.common_robot.robot import robot_startup
        from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS

        # Connect to the RX150 arm through the running ROS 2 driver.
        self.bot = InterbotixManipulatorXS(
            robot_model="rx150",
            group_name="arm",
            gripper_name="gripper",
        )

        # Initialize the robot interface.
        robot_startup()

        # Set the duration and acceleration time for joint movements.
        self.bot.arm.set_trajectory_time(
            moving_time=self.moving_time,
            accel_time=self.accel_time,
        )

        # Return the connected arm object.
        return self

    def __exit__(self, exc_type, exc_value, traceback):

        # Import the ROS 2 shutdown function.
        from interbotix_common_modules.common_robot.robot import robot_shutdown

        try:
            # Move the arm to its safe resting position.
            if self.bot is not None:
                self.bot.arm.go_to_sleep_pose()

        finally:
            # Shut down the robot interface after the movement attempt.
            robot_shutdown()
            self.bot = None

        # Do not hide exceptions raised inside the context manager.
        return False

    def send(self, thetas):
        """Command N_JOINTS angles, in radians, in JOINT_NAMES order.

        Check `within_limits` first and refuse an out-of-range command rather than letting the
        driver clamp it silently. A clamped joint is the hardest kind of Part D discrepancy to
        track down, because the arm moves and looks fine.
        """

        # Verify that the robot connection is active.
        if self.bot is None:
            raise RuntimeError("RX150 is not connected.")

        # Accept either a list of angles or an IK Solution object.
        if hasattr(thetas, "thetas"):
            thetas = thetas.thetas

        # Convert the joint angles to a NumPy array.
        angles = np.asarray(thetas, dtype=float)

        # Check all five joint angles against the manufacturer limits.
        allowed, violated = within_limits(angles)

        if not allowed:
            raise ValueError(
                f"Joint limits violated: {', '.join(violated)}"
            )

        # Send the requested angles to the physical robot.
        success = self.bot.arm.set_joint_positions(angles.tolist())

        # The driver can also reject a command.
        if success is False:
            raise RuntimeError("RX150 driver rejected the joint command.")

    def read(self):
        """The angles the servos actually reached, as a list of N_JOINTS floats.

        These are not the angles you sent. The difference under gravity load is one of the
        candidate explanations Part D asks you to argue about, so record both.
        """

        # Verify that the robot connection is active.
        if self.bot is None:
            raise RuntimeError("RX150 is not connected.")

        # Read the actual joint positions reported by the servos.
        angles = self.bot.arm.get_joint_positions()

        # Return the five measured angles as Python floats.
        return [float(theta) for theta in angles]

    def driver_ee_pose(self):
        """The manufacturer's own forward kinematics, as a 4x4 matrix.

        A free check on your `fk()` that needs no ruler, and it isolates model error from
        measurement error. It does not replace the physical measurement Part D requires: the
        driver's model and the real arm are two different things, which is the whole point of
        that part.
        """

        # Verify that the robot connection is active.
        if self.bot is None:
            raise RuntimeError("RX150 is not connected.")

        # Get the manufacturer's calculated gripper transformation.
        H = self.bot.arm.get_ee_pose()

        # Return the 4x4 homogeneous transformation matrix.
        return np.asarray(H, dtype=float)
