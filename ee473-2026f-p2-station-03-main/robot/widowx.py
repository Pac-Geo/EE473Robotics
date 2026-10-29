"""Everything that knows this is a WidowX AI.

This is the **only** file in the project that may contain hardware-specific numbers. Joint
limits, link lengths, DH parameters, the tool frame definition, and the network interface: all of
it lives here, and nothing outside this file may hard-code a dimension.

The boundary has a payoff in Project 3, which extends your kinematics/ package to velocity
control on this same arm. Where the boundary has held, that is an addition. Where link lengths
have leaked into kinematics/, it is a rewrite.

The values come from the Part A derivation. The structure is given; the numbers are the
assignment.
"""

import numpy as np

# Joint order. Every list of angles in this project is in this order, without exception.
# These are the names Trossen's description uses; keep them so your code and the URDF agree.
JOINT_NAMES = [
    "joint_0",
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]

N_JOINTS = len(JOINT_NAMES)


# ----------------------------------------------------------------------------
# DH parameters
# ----------------------------------------------------------------------------
# One row per joint, in JOINT_NAMES order. The instructions specify the CLASSICAL convention.
# Trossen publishes both classical and modified tables; their rows do not correspond, so make
# sure you are reading the right one before you compare.
#
# Each row is the CONSTANT part of the row. The joint variable is added at evaluation time.
# This arm is all revolute.
#
#   (theta_offset, d, a, alpha)      angles in radians, lengths in metres

DH_CONVENTION = "classic"


beta = np.arctan2(0.060, 0.245)

DH_PARAMS = [
    # (theta_offset, d, a, alpha)
    (0.0,           0.10350, 0.02000, -np.pi / 2),  # joint_0
    (np.pi,         0.0,     0.26400,  np.pi),      # joint_1
    (-np.pi + beta, 0.0,     np.hypot(0.245, 0.060), 0.0),  # joint_2
    (-beta,         0.0,     0.06775,  np.pi / 2),  # joint_3
    (-np.pi / 2,    0.0,     0.0,     -np.pi / 2),  # joint_4
    (0.0,           0.02895, 0.0,      0.0),       # joint_5
]



# ----------------------------------------------------------------------------
# Tool frame
# ----------------------------------------------------------------------------
# Trossen's chain does not end at joint 6. It continues through a Flange frame and then to Tool,
# which is the frame the instructions prescribe. That is two fixed transformations: a rotation
# carrying frame 6 into the flange, and a translation of 156.06 mm from the flange to the tool
# point. A table that stops at joint 6 is short by that whole segment, and it shows up as a large
# constant position error at every configuration.


TOOL_OFFSET = np.array([
    [0, 1, 0, 0.0],
    [0, 0, 1, 0.0],
    [1, 0, 0, 0.156062],
    [0, 0, 0, 1.0]
], dtype=float)


# Screw axes from the URDF at the zero configuration.
# Each row is [wx, wy, wz, vx, vy, vz].
# The linear component is v = -w x p.

POE_SCREWS = np.array([
    [0,  0,  1,  0.0,      0.0,  0.0],
    [0,  1,  0, -0.10350,  0.0,  0.02000],
    [0, -1,  0,  0.10350,  0.0,  0.24400],
    [0, -1,  0,  0.16350,  0.0, -0.00100],
    [0,  0, -1,  0.0,      0.06875, 0.0],
    [1,  0,  0,  0.0,      0.16350, 0.0],
], dtype=float)

# Tool pose when all six joint angles are zero.
POE_HOME = np.eye(4)
POE_HOME[:3, 3] = [0.253762, 0.0, 0.16350]




# ----------------------------------------------------------------------------
# Joint limits
# ----------------------------------------------------------------------------
# From the manufacturer, in radians. Do not guess these: read them from wxai_follower.urdf in the
# trossen_arm_description package, or from Trossen's documentation. Part B requires you to flag
# solutions that violate them.


JOINT_LIMITS = [
    # (lower, upper) in radians
    (np.deg2rad(-175), np.deg2rad(175)),  # joint_0 - Base
    (0.0, np.pi),                        # joint_1 - Shoulder
    (0.0, 3 * np.pi / 4),                # joint_2 - Elbow
    (-np.pi / 2, np.pi / 2),             # joint_3 - Wrist pitch
    (-np.pi / 2, np.pi / 2),             # joint_4 - Wrist yaw
    (-np.pi, np.pi),                     # joint_5 - Wrist roll
]



def within_limits(thetas):
    """True if every joint angle is inside its limit.

    Returns a (bool, list_of_offending_joint_names) pair so the caller can report *which* joint
    is out of range rather than just that something is.
    """
    raise NotImplementedError


def home():
    """The all-zeros configuration, as a list of N_JOINTS angles.

    Establish what this configuration looks like physically before commanding it.
    """
    return [0.0] * N_JOINTS


# ----------------------------------------------------------------------------
# Hardware
# ----------------------------------------------------------------------------
# Part C only. Everything below talks to the physical arm, and this is the only file in the
# project allowed to. Nothing in kinematics/ may import any of it.
#
# The controller resides on the laboratory network and the trossen_arm driver connects to it
# directly over Ethernet, in contrast to the ROS stack used for the RX150 in Project 1. No launch
# file is involved.
#
# Import trossen_arm WITHIN the methods rather than at module level. The test suite must run on a
# machine with no manipulator attached, and a module-level import would cause `pytest` to fail at
# collection with an ImportError unrelated to the code under test.
#
# The SDK version must match the controller firmware. See requirements.txt.

ARM_IP = "192.168.1.2"


class Arm:
    """A connection to the physical WidowX AI.

    Employ it as a context manager, so that the manipulator is left in a safe state even where a
    script raises partway through:

        with Arm() as arm:
            arm.send(solution.thetas)
            measured = arm.read()

    Implement `__enter__` by constructing a TrossenArmDriver and calling configure() with the
    model, the follower end effector, ARM_IP, and the clear-error flag. Implement `__exit__` by
    bringing the arm somewhere safe and calling cleanup().

    Review the safety note in Part C before connecting. This manipulator is heavier and faster
    than the RX150, and a sign error in a commanded pose carries correspondingly greater
    consequences.
    """

    def __init__(self, goal_time=3.0):
        self.goal_time = goal_time
        self.driver = None

    def __enter__(self):
        raise NotImplementedError

    def __exit__(self, exc_type, exc_value, traceback):
        raise NotImplementedError

    def send(self, thetas):
        """Command N_JOINTS angles, in radians, in JOINT_NAMES order.

        Consult `within_limits` first and refuse an out-of-range command rather than permitting
        the driver to clamp it silently. A clamped joint is among the most difficult discrepancies
        to locate, as the manipulator moves and appears correct.

        Command conservatively at first, increasing goal_time rather than reducing it.
        """
        raise NotImplementedError

    def driver_ee_pose(self):
        """The manufacturer's own forward kinematics, as (x, y, z, rx, ry, rz).

        The driver exposes get_cartesian_positions(), which reports six values: the tool position
        in metres followed by three orientation parameters. Consult the SDK documentation for the
        orientation convention those three use before comparing them against your own.

        This is a check on fk() requiring no measurement, and it is the only practical check on
        the orientation component of a commanded pose, since orientation cannot be measured with
        a rule. It separates model error from measurement error.

        It does not replace the physical measurement Part C requires. The driver's model and the
        physical manipulator are themselves distinct, which is precisely what that part
        investigates.
        """
        raise NotImplementedError

    def read(self):
        """The angles the servos actually reached, as a list of N_JOINTS floats.

        The driver reports more values than this manipulator has arm joints, the gripper being
        included among them. Return the six in JOINT_NAMES order.

        These are not the angles commanded. The difference under gravitational load is among the
        candidate explanations Part C requires, so record both.
        """
        raise NotImplementedError
