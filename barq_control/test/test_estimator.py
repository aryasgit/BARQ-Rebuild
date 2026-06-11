"""Unit tests for the state estimator's pure kinematic-odometry math."""

from barq_control.leg_kinematics import ik_exact, kx_of, LAT, side_of
from barq_control.state_estimator_node import (feet_body_positions, stance_legs,
                                               stance_velocity)
import pytest

HIPS = {
    'FL': [0.108484, 0.0171913, 0.00022012],
    'FR': [0.108484, -0.0148092, 0.00022176],
    'RL': [-0.108371, 0.0167905, 0.00022176],
    'RR': [-0.108371, -0.01521, 0.00022012],
}


def _joints_for_feet(feet):
    """Build a joint_states dict that places each foot at the given body-frame target."""
    jp = {}
    for leg, (x, y, z) in feet.items():
        hx, hy, hz = HIPS[leg]
        q1, q2, q3 = ik_exact(x - hx, y - hy, z - hz, kx_of(leg), side_of(hy))
        jp[f'{leg}_hip_joint'] = q1
        jp[f'{leg}_knee_joint'] = q2
        jp[f'{leg}_ankle_joint'] = q3
    return jp


def _stance_pose(depth=0.13):
    return {leg: (HIPS[leg][0] + kx_of(leg),
                  HIPS[leg][1] + side_of(HIPS[leg][1]) * LAT,
                  HIPS[leg][2] - depth) for leg in HIPS}


def test_feet_positions_roundtrip():
    """feet_body_positions inverts the IK-built joint dict back to the foot targets."""
    target = _stance_pose()
    feet = feet_body_positions(_joints_for_feet(target), HIPS)
    for leg in HIPS:
        assert feet[leg] == pytest.approx(target[leg], abs=1e-9)


def test_stance_detection_picks_lowest_pair():
    """The two deepest feet are the stance pair."""
    feet = _stance_pose()
    feet['FL'] = (feet['FL'][0], feet['FL'][1], feet['FL'][2] + 0.02)   # FL lifted
    feet['RR'] = (feet['RR'][0], feet['RR'][1], feet['RR'][2] + 0.02)   # RR lifted
    assert sorted(stance_legs(feet)) == ['FR', 'RL']


def test_stance_velocity_recovers_body_motion():
    """Body moving +0.1 m/s => stance feet slide -x in body frame => v estimate +0.1."""
    dt = 0.02
    v = 0.10
    a = _stance_pose()
    b = {leg: (x - v * dt, y, z) for leg, (x, y, z) in a.items()}
    vx, vy = stance_velocity(a, b, ['FL', 'RR'], dt)
    assert vx == pytest.approx(v, abs=1e-9)
    assert vy == pytest.approx(0.0, abs=1e-9)


def test_stance_velocity_lateral():
    """Strafe +y shows up identically through the lateral axis."""
    dt = 0.02
    a = _stance_pose()
    b = {leg: (x, y - 0.06 * dt, z) for leg, (x, y, z) in a.items()}
    vx, vy = stance_velocity(a, b, ['FR', 'RL'], dt)
    assert vx == pytest.approx(0.0, abs=1e-9)
    assert vy == pytest.approx(0.06, abs=1e-9)
