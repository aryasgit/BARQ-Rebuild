"""Unit tests for barq_control.gait (trot foot-trajectory generator, exact-model frame)."""

from barq_control.gait import foot_targets, LEGS
from barq_control.leg_kinematics import ik_exact, kx_of, LAT, side_of
import pytest

HIPS = {
    'FL': [0.108484, 0.0171913, 0.00022012],
    'FR': [0.108484, -0.0148092, 0.00022176],
    'RL': [-0.108371, 0.0167905, 0.00022176],
    'RR': [-0.108371, -0.01521, 0.00022012],
}
PERIOD = 0.5


def _legxyz(out, leg):
    """Slice the 3 coords for one leg out of the 12-value foot-target vector."""
    i = LEGS.index(leg)
    return out[3 * i:3 * i + 3]


def test_zero_command_holds_neutral_stance():
    """No velocity command -> feet at the exact-model neutral stance (no stepping)."""
    out = foot_targets(0.37, 0.0, 0.0, 0.0, HIPS, period=PERIOD, stand_height=0.13,
                       rear_raise=0.0)
    for leg in LEGS:
        hx, hy, hz = HIPS[leg]
        x, y, z = _legxyz(out, leg)
        assert x == pytest.approx(hx + kx_of(leg), abs=1e-12)
        assert y == pytest.approx(hy + side_of(hy) * LAT, abs=1e-12)
        assert z == pytest.approx(hz - 0.13, abs=1e-12)


def test_rear_raise_stance_trim():
    """Rear feet sit rear_raise deeper than front (nose-down load-forward trim, D-016)."""
    out = foot_targets(0.0, 0.0, 0.0, 0.0, HIPS, stand_height=0.13, rear_raise=0.02)
    z = {leg: _legxyz(out, leg)[2] - HIPS[leg][2] for leg in LEGS}
    assert z['FL'] == pytest.approx(-0.13, abs=1e-12)
    assert z['FR'] == pytest.approx(-0.13, abs=1e-12)
    assert z['RL'] == pytest.approx(-0.15, abs=1e-12)
    assert z['RR'] == pytest.approx(-0.15, abs=1e-12)


def test_periodicity():
    """Foot targets repeat every gait period."""
    a = foot_targets(0.13, 0.12, 0.0, 0.0, HIPS, period=PERIOD)
    b = foot_targets(0.13 + PERIOD, 0.12, 0.0, 0.0, HIPS, period=PERIOD)
    assert a == pytest.approx(b, abs=1e-12)


def test_diagonal_pairs_in_sync():
    """FL+RR share a phase and FR+RL share the opposite phase (trot)."""
    for t in (0.0, 0.05, 0.1, 0.18, 0.25, 0.33, 0.45):
        out = foot_targets(t, 0.12, 0.0, 0.0, HIPS, period=PERIOD, rear_raise=0.0)
        z = {leg: _legxyz(out, leg)[2] for leg in LEGS}
        assert z['FL'] == pytest.approx(z['RR'], abs=1e-12)
        assert z['FR'] == pytest.approx(z['RL'], abs=1e-12)


def test_swinging_foot_lifts():
    """Across a cycle each foot lifts above the neutral height (swing arc)."""
    for leg in LEGS:
        neutral_z = HIPS[leg][2] - 0.13
        max_z = max(_legxyz(foot_targets(k / 100.0 * PERIOD, 0.1, 0.0, 0.0, HIPS,
                    period=PERIOD, rear_raise=0.0), leg)[2] for k in range(100))
        assert max_z > neutral_z + 0.015, f'{leg} never lifts'


def test_step_length_scales_with_velocity():
    """Stance sweep amplitude grows with commanded forward velocity."""
    def fl_x_range(vx):
        xs = [_legxyz(foot_targets(k / 50.0 * PERIOD, vx, 0.0, 0.0, HIPS, period=PERIOD),
                      'FL')[0] for k in range(50)]
        return max(xs) - min(xs)
    assert fl_x_range(0.2) > fl_x_range(0.1) > fl_x_range(0.0)


def test_default_gait_stays_within_joint_limits():
    """Default gait params through the EXACT IK keep every joint inside its limits."""
    for k in range(120):
        out = foot_targets(k / 120.0 * PERIOD, 0.15, 0.0, 0.0, HIPS, period=PERIOD)
        for leg in LEGS:
            hx, hy, hz = HIPS[leg]
            fx, fy, fz = _legxyz(out, leg)
            q1, q2, q3 = ik_exact(fx - hx, fy - hy, fz - hz, kx_of(leg), side_of(hy))
            assert -0.785 <= q1 <= 0.785, f'{leg} coxa {q1}'
            assert -1.57 <= q2 <= 1.57, f'{leg} femur {q2}'
            assert -2.2 <= q3 <= 0.0, f'{leg} tibia {q3}'


def test_all_targets_reachable_with_yaw_and_strafe():
    """Every foot target over a mixed-command cycle is solvable by the exact IK."""
    for k in range(60):
        out = foot_targets(k / 60.0 * PERIOD, 0.12, 0.03, 0.3, HIPS, period=PERIOD)
        for leg in LEGS:
            hx, hy, hz = HIPS[leg]
            fx, fy, fz = _legxyz(out, leg)
            ik_exact(fx - hx, fy - hy, fz - hz, kx_of(leg), side_of(hy))  # raises if invalid
