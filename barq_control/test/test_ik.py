"""Unit tests for barq_control.leg_kinematics - verifies IK inverts FK exactly."""

from barq_control.leg_kinematics import fk_leg, ik_leg, side_of
import pytest

# BARQ link lengths (robot_params.yaml, derived from the URDF).
L1, L2, L3 = 0.0465, 0.107, 0.100


def test_neutral_pose_is_straight_down():
    """Zero joint angles put the foot straight down, and IK of that recovers zeros."""
    for side in (+1.0, -1.0):
        x, y, z = fk_leg(0.0, 0.0, 0.0, L1, L2, L3, side)
        assert x == pytest.approx(0.0, abs=1e-12)
        assert y == pytest.approx(side * L1, abs=1e-12)
        assert z == pytest.approx(-(L2 + L3), abs=1e-12)
        q1, q2, q3 = ik_leg(x, y, z, L1, L2, L3, side)
        assert (q1, q2, q3) == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_roundtrip_grid():
    """FK then IK recovers the original angles across the workspace (knee_bend=+1 => q3>=0)."""
    max_err = 0.0
    for side in (+1.0, -1.0):
        for q1 in (-0.6, -0.3, 0.0, 0.3, 0.6):
            for q2 in (-0.9, -0.4, 0.0, 0.4, 0.9):
                for q3 in (0.15, 0.5, 0.9, 1.3):
                    x, y, z = fk_leg(q1, q2, q3, L1, L2, L3, side)
                    r1, r2, r3 = ik_leg(x, y, z, L1, L2, L3, side, knee_bend=+1.0)
                    max_err = max(max_err, abs(r1 - q1), abs(r2 - q2), abs(r3 - q3))
                    fx, fy, fz = fk_leg(r1, r2, r3, L1, L2, L3, side)
                    assert (fx, fy, fz) == pytest.approx((x, y, z), abs=1e-9)
    assert max_err < 1e-9, f'max angle round-trip error {max_err}'


def test_unreachable_raises():
    """A foot target inside the coxa radius is rejected."""
    with pytest.raises(ValueError):
        ik_leg(0.0, 0.0, 0.0, L1, L2, L3, side=+1.0)


def test_side_of():
    """side_of maps +y hips to left (+1) and -y hips to right (-1)."""
    assert side_of(0.0171913) == 1.0
    assert side_of(-0.0148092) == -1.0
