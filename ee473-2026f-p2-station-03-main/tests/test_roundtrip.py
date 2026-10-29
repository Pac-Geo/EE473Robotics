"""Project 2 round-trip verification: FK -> IK -> FK.

Every target is constructed by forward kinematics, so the target is known
reachable geometrically.  A passing test verifies returned branches; it does
not prove that IK found every branch or that every branch is within limits.

Run from the repository root with:
    python -m pytest -v tests/test_roundtrip.py
"""

import numpy as np
import pytest

from kinematics.fk import fk
from kinematics.ik import ik
from robot import widowx


# Six joint angles, in widowx.JOINT_NAMES order. Avoid singularities and limits.
CONFIGURATIONS = [
    ([0.0, -0.3, 0.6, 0.0, 0.5, 0.0], "near home, wrist level"),
    ([0.6, -0.5, 0.9, -0.2, 0.4, 0.3], "rotated round, elbow bent"),
    ([-0.4, -0.2, 0.5, 0.3, 0.8, -0.6], "other side, wrist pitched"),
    ([0.2, -0.7, 1.1, -0.3, 0.6, 1.2], "reaching low, wrist rolled"),
    ([-0.8, -0.4, 0.7, 0.1, 0.9, 0.5], "well off axis"),
]

POS_TOL = 1e-4  # 0.1 mm
ROT_TOL = 1e-3  # maximum rotation-matrix entry difference


def _pose_error(H1, H2):
    """Return translation error (m) and max rotation-matrix entry difference."""
    dp = float(np.linalg.norm(H1[:3, 3] - H2[:3, 3]))
    dR = float(np.abs(H1[:3, :3] - H2[:3, :3]).max())
    return dp, dR


@pytest.mark.parametrize("thetas,label", CONFIGURATIONS)
def test_roundtrip(thetas, label):
    """Every returned IK branch must match the target position AND orientation."""
    target = fk(thetas)
    solutions = ik(target)

    assert solutions, (
        f"no solution returned for {label}, but this pose came out of fk() "
        "so one must exist"
    )

    for sol in solutions:
        dp, dR = _pose_error(fk(sol.thetas), target)
        assert dp < POS_TOL, (
            f"{label}, branch {sol.branch}: position off by {dp * 1000:.3f} mm"
        )
        assert dR < ROT_TOL, (
            f"{label}, branch {sol.branch}: orientation off by {dR:.2e}"
        )


@pytest.mark.parametrize("thetas,label", CONFIGURATIONS)
def test_original_configuration_is_among_the_solutions(thetas, label):
    """IK must find the exact configuration used to create each target."""
    solutions = ik(fk(thetas))
    found = any(np.allclose(sol.thetas, thetas, atol=1e-4) for sol in solutions)
    assert found, (
        f"{label}: the generating configuration was not among the "
        f"{len(solutions)} solutions returned. Which branch is missing?"
    )


@pytest.mark.parametrize("thetas,label", CONFIGURATIONS)
def test_multiple_branches(thetas, label):
    """These nonsingular targets should produce more than one IK branch."""
    solutions = ik(fk(thetas))
    assert len(solutions) > 1, (
        f"{label}: only {len(solutions)} solution returned. "
        "Are you returning every branch?"
    )


def test_limits_are_flagged_not_dropped():
    """Out-of-range solutions must be reported rather than silently omitted."""
    for thetas, _ in CONFIGURATIONS:
        for sol in ik(fk(thetas)):
            assert isinstance(sol.within_limits, bool), (
                "every Solution must say whether it is within joint limits"
            )
            if not sol.within_limits:
                assert sol.violated_joints, (
                    "a solution outside limits must name which joints are violated"
                )


def test_unreachable_returns_empty():
    """A target 10 m away is unreachable and must give no IK branches."""
    far = np.eye(4)
    far[:3, 3] = [10.0, 0.0, 0.0]
    assert ik(far) == [], "a target 10 m away should return no solutions"


def test_fk_at_home_is_finite():
    """FK must return a finite homogeneous transform for home angles."""
    H = fk(widowx.home())
    assert H.shape == (4, 4)
    assert np.all(np.isfinite(H)), "FK returned NaN or inf at home"
    assert np.allclose(H[3, :], [0, 0, 0, 1]), (
        "bottom row of a homogeneous transform must be 0 0 0 1"
    )


def test_fk_rejects_the_wrong_joint_count():
    """Reject five-angle input; this is a six-joint arm."""
    with pytest.raises(ValueError):
        fk([0.0, 0.0, 0.0, 0.0, 0.0])


def test_rotation_block_is_a_rotation():
    """FK rotation must be orthogonal and have determinant +1."""
    H = fk([0.1, -0.2, 0.3, -0.4, 0.5, 0.6])
    R = H[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=ROT_TOL), (
        "rotation block is not orthogonal"
    )
    assert np.isclose(np.linalg.det(R), 1.0, atol=ROT_TOL), (
        f"determinant is {np.linalg.det(R):.6f}, should be +1"
    )
