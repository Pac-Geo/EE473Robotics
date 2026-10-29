"""The round-trip test: IK, then FK, and confirm you land back where you asked.

This is written for you, because it is the instrument you check your own work with rather than
part of what is being assessed. It will fail until fk() and ik() are implemented. That is the
point: run it now, watch it fail, and make it pass.

    pytest -v

What a passing run proves: every solution your IK returns is a VALID solution.
What it does not prove: that you found ALL of them, or that the arm can physically adopt them.
Those are the two separate requirements in Part C, and no round trip can check either.
"""

import numpy as np
import pytest

from kinematics.fk import fk
from kinematics.ik import ik
from robot import rx150

# Targets are deliberately modest: well inside the workspace, nothing near a singularity. Add
# your own once these pass, and include at least one you expect to be unreachable.
TARGETS = [
    # (x, y, z, pitch_deg, label)
    (0.25, 0.00, 0.20, 0.0, "straight ahead, level"),
    (0.20, 0.10, 0.15, -30.0, "off to one side, angled down"),
    (0.18, -0.12, 0.25, 15.0, "other side, angled up"),
    (0.30, 0.00, 0.10, -45.0, "reaching low and far"),
    (0.15, 0.00, 0.30, 60.0, "high and close"),
]

POS_TOL = 1e-4   # 0.1 mm
ROT_TOL = 1e-3   # about 0.06 degrees


def _position_error(H, x, y, z):
    return float(np.linalg.norm(H[:3, 3] - np.array([x, y, z])))


@pytest.mark.parametrize("x,y,z,pitch_deg,label", TARGETS)
def test_roundtrip(x, y, z, pitch_deg, label):
    """Every solution IK returns must reproduce the requested position through FK."""
    pitch = np.radians(pitch_deg)
    solutions = ik(x, y, z, pitch)

    assert solutions, f"no solution returned for {label}; if this target really is unreachable, move it"

    for sol in solutions:
        H = fk(sol.thetas)
        err = _position_error(H, x, y, z)
        assert err < POS_TOL, (
            f"{label}, branch {sol.branch}: position off by {err * 1000:.3f} mm"
        )


@pytest.mark.parametrize("x,y,z,pitch_deg,label", TARGETS)
def test_multiple_branches(x, y, z, pitch_deg, label):
    """A reachable target well inside the workspace should give more than one solution.

    If this fails while test_roundtrip passes, your solver works but is throwing branches away.
    """
    solutions = ik(x, y, z, np.radians(pitch_deg))
    assert len(solutions) > 1, (
        f"{label}: only {len(solutions)} solution returned. Elbow up and elbow down are both "
        f"valid here. Are you returning every branch?"
    )


def test_limits_are_flagged_not_dropped():
    """Solutions outside joint limits must be returned and marked, not silently discarded."""
    for x, y, z, pitch_deg, _ in TARGETS:
        for sol in ik(x, y, z, np.radians(pitch_deg)):
            assert isinstance(sol.within_limits, bool), (
                "every Solution must say whether it is within joint limits"
            )
            if not sol.within_limits:
                assert sol.violated_joints, (
                    "a solution outside limits must name which joints are violated"
                )


def test_unreachable_returns_empty():
    """A target far outside the workspace returns an empty list, not an exception and not NaN."""
    solutions = ik(10.0, 0.0, 0.0, 0.0)
    assert solutions == [], "a target 10 m away should return no solutions"


def test_fk_at_home_is_finite():
    """Cheapest possible smoke test. If this fails, nothing downstream is worth debugging."""
    H = fk(rx150.home())
    assert H.shape == (4, 4)
    assert np.all(np.isfinite(H)), "FK returned NaN or inf at the home configuration"
    assert np.allclose(H[3, :], [0, 0, 0, 1]), "bottom row of a homogeneous transform must be 0 0 0 1"


def test_rotation_block_is_a_rotation():
    """FK must return an actual rotation: orthogonal, determinant +1.

    A determinant of -1 means a reflection has crept in, usually a sign error in one transform.
    A determinant that is not +/-1 means a scale has, usually a non-unit axis.
    """
    H = fk([0.1, -0.2, 0.3, -0.4, 0.5])
    R = H[:3, :3]
    assert np.allclose(R.T @ R, np.eye(3), atol=ROT_TOL), "rotation block is not orthogonal"
    assert np.isclose(np.linalg.det(R), 1.0, atol=ROT_TOL), (
        f"determinant is {np.linalg.det(R):.6f}, should be +1"
    )
