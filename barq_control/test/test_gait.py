"""Unit tests for barq_control.gait (trot foot-trajectory generator)."""

from barq_control.gait import foot_targets, LEGS
from barq_control.leg_kinematics import ik_leg, side_of
import pytest

L1, L2, L3 = 0.0465, 0.107, 0.100
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
    """No velocity command -> feet sit at the neutral stance (no stepping)."""
    out = foot_targets(0.37, 0.0, 0.0, 0.0, HIPS, L1, period=PERIOD, stand_height=0.16)
    for leg in LEGS:
        hx, hy, hz = HIPS[leg]
        x, y, z = _legxyz(out, leg)
        assert x == pytest.approx(hx, abs=1e-12)
        assert y == pytest.approx(hy + side_of(hy) * L1, abs=1e-12)
        assert z == pytest.approx(hz - 0.16, abs=1e-12)


def test_periodicity():
    """Foot targets repeat every gait period."""
    a = foot_targets(0.13, 0.12, 0.0, 0.0, HIPS, L1, period=PERIOD)
    b = foot_targets(0.13 + PERIOD, 0.12, 0.0, 0.0, HIPS, L1, period=PERIOD)
    assert a == pytest.approx(b, abs=1e-12)


def test_diagonal_pairs_in_sync():
    """FL+RR share a phase and FR+RL share the opposite phase (trot)."""
    for t in (0.0, 0.05, 0.1, 0.18, 0.25, 0.33, 0.45):
        out = foot_targets(t, 0.12, 0.0, 0.0, HIPS, L1, period=PERIOD, step_height=0.04)
        z = {leg: _legxyz(out, leg)[2] for leg in LEGS}
        assert z['FL'] == pytest.approx(z['RR'], abs=1e-12)
        assert z['FR'] == pytest.approx(z['RL'], abs=1e-12)


def test_swinging_foot_lifts():
    """Across a cycle each foot lifts above the neutral height (swing arc)."""
    for leg in LEGS:
        neutral_z = HIPS[leg][2] - 0.16
        max_z = max(_legxyz(foot_targets(k / 100.0 * PERIOD, 0.1, 0.0, 0.0, HIPS, L1,
                    period=PERIOD, step_height=0.04, stand_height=0.16), leg)[2]
                    for k in range(100))
        assert max_z > neutral_z + 0.02, f'{leg} never lifts'


def test_step_length_scales_with_velocity():
    """Stance sweep amplitude grows with commanded forward velocity."""
    def fl_x_range(vx):
        xs = [_legxyz(foot_targets(k / 50.0 * PERIOD, vx, 0.0, 0.0, HIPS, L1, period=PERIOD),
                      'FL')[0] for k in range(50)]
        return max(xs) - min(xs)
    assert fl_x_range(0.2) > fl_x_range(0.1) > fl_x_range(0.0)


def test_default_gait_stays_within_tibia_range():
    """With default gait params and the default IK branch, the tibia stays in [-1.571, 0]."""
    for k in range(120):
        out = foot_targets(k / 120.0 * PERIOD, 0.15, 0.0, 0.0, HIPS, L1, period=PERIOD)
        for leg in LEGS:
            hx, hy, hz = HIPS[leg]
            fx, fy, fz = _legxyz(out, leg)
            _, _, q3 = ik_leg(fx - hx, fy - hy, fz - hz, L1, L2, L3, side_of(hy))
            assert -1.571 <= q3 <= 0.0, f'{leg} tibia {q3} outside servo range'


def test_all_targets_reachable():
    """Every foot target over a cycle is solvable by the leg IK (within the workspace)."""
    for k in range(60):
        out = foot_targets(k / 60.0 * PERIOD, 0.12, 0.03, 0.3, HIPS, L1,
                           period=PERIOD, step_height=0.04, stand_height=0.16)
        for leg in LEGS:
            hx, hy, hz = HIPS[leg]
            fx, fy, fz = _legxyz(out, leg)
            ik_leg(fx - hx, fy - hy, fz - hz, L1, L2, L3, side_of(hy))  # raises if unreachable
