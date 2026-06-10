"""Exact-model tests: fk_exact must equal the raw URDF joint chain; ik_exact must invert it."""

import math

from barq_control.leg_kinematics import fk_exact, ik_exact, kx_of, side_of
import pytest

# Raw URDF origins (barq.urdf.xacro) — independent of the model under test.
KNEE_Y = 0.0430692
ANKLE = (0.018944, 0.0324, -0.1)
FOOT = (0.0, 0.0, -0.1)
LEGS = {  # name -> (kx, side)
    'FL': (+0.01744, +1.0),
    'FR': (+0.01744, -1.0),
    'RL': (-0.01744, +1.0),
    'RR': (-0.01744, -1.0),
}


def _rot_x(a, v):
    x, y, z = v
    return (x, y * math.cos(a) - z * math.sin(a), y * math.sin(a) + z * math.cos(a))


def _rot_y(a, v):
    x, y, z = v
    return (x * math.cos(a) + z * math.sin(a), y, -x * math.sin(a) + z * math.cos(a))


def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def urdf_chain_fk(q1, q2, q3, kx, side):
    """Foot point in hip frame composed directly from URDF origins and joint rotations."""
    p = _rot_y(q3, FOOT)
    p = _add((ANKLE[0], side * ANKLE[1], ANKLE[2]), p)
    p = _rot_y(q2, p)
    p = _add((kx, side * KNEE_Y, 0.0), p)
    return _rot_x(q1, p)


GRID_Q1 = (-0.6, -0.25, 0.0, 0.25, 0.6)
GRID_Q2 = (-0.5, 0.0, 0.5, 1.0, 1.4)
GRID_Q3 = (-2.1, -1.6, -1.0, -0.4, 0.2)


def test_fk_exact_matches_urdf_chain():
    """fk_exact equals the rotation-matrix composition of the raw URDF chain (<1e-12)."""
    for kx, side in LEGS.values():
        for q1 in GRID_Q1:
            for q2 in GRID_Q2:
                for q3 in GRID_Q3:
                    ref = urdf_chain_fk(q1, q2, q3, kx, side)
                    got = fk_exact(q1, q2, q3, kx, side)
                    assert got == pytest.approx(ref, abs=1e-12), (kx, side, q1, q2, q3)


def test_ik_exact_returns_valid_solution_everywhere():
    """ik_exact lands the foot on the target across the full grid (solution validity)."""
    for kx, side in LEGS.values():
        for q1 in GRID_Q1:
            for q2 in GRID_Q2:
                for q3 in (-2.1, -1.6, -1.0, -0.4):
                    x, y, z = urdf_chain_fk(q1, q2, q3, kx, side)
                    r1, r2, r3 = ik_exact(x, y, z, kx, side, knee_bend=-1.0)
                    fx, fy, fz = fk_exact(r1, r2, r3, kx, side)
                    assert (fx, fy, fz) == pytest.approx((x, y, z), abs=1e-9)


def test_ik_exact_roundtrip_operational_envelope():
    """In the gait's working region the IK recovers the exact original angles (<1e-9)."""
    for kx, side in LEGS.values():
        for q1 in GRID_Q1:
            for q2 in (0.3, 0.7, 1.1, 1.4):
                for q3 in (-2.1, -1.7, -1.2):
                    x, y, z = urdf_chain_fk(q1, q2, q3, kx, side)
                    r1, r2, r3 = ik_exact(x, y, z, kx, side, knee_bend=-1.0)
                    assert (r1, r2, r3) == pytest.approx((q1, q2, q3), abs=1e-9)


def test_neutral_stance_angles():
    """Foot below the knee-x at depth 0.13 -> q1=0; angles within limits for all legs."""
    for name, (kx, side) in LEGS.items():
        q1, q2, q3 = ik_exact(kx, side * 0.0754692, -0.13, kx, side)
        assert q1 == pytest.approx(0.0, abs=1e-9), name
        assert abs(q2) < 1.57, name
        assert -2.2 < q3 < 0.0, name
        x, y, z = fk_exact(q1, q2, q3, kx, side)
        assert (x, y, z) == pytest.approx((kx, side * 0.0754692, -0.13), abs=1e-9)


def test_helpers():
    """kx_of follows the front/rear sign; side_of follows the hip y sign."""
    assert kx_of('FL') == kx_of('FR') == +0.01744
    assert kx_of('RL') == kx_of('RR') == -0.01744
    assert side_of(0.017) == 1.0 and side_of(-0.015) == -1.0
