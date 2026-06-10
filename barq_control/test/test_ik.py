"""Unit tests for barq_control.leg_kinematics — verifies IK inverts FK exactly."""

import math
import os
import sys

import pytest

# Make the package importable when run directly (pytest from the package dir or workspace).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from barq_control.leg_kinematics import fk_leg, ik_leg, side_of  # noqa: E402

# BARQ link lengths (robot_params.yaml, derived from the URDF).
L1, L2, L3 = 0.0465, 0.107, 0.100


def test_neutral_pose_is_straight_down():
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
                    err = max(abs(r1 - q1), abs(r2 - q2), abs(r3 - q3))
                    max_err = max(max_err, err)
                    # and FK of the recovered angles lands on the same foot point
                    fx, fy, fz = fk_leg(r1, r2, r3, L1, L2, L3, side)
                    assert (fx, fy, fz) == pytest.approx((x, y, z), abs=1e-9)
    assert max_err < 1e-9, f'max angle round-trip error {max_err}'


def test_unreachable_raises():
    with pytest.raises(ValueError):
        # foot at the hip origin is well inside the coxa radius -> unreachable
        ik_leg(0.0, 0.0, 0.0, L1, L2, L3, side=+1.0)


def test_side_of():
    assert side_of(0.0171913) == 1.0
    assert side_of(-0.0148092) == -1.0
