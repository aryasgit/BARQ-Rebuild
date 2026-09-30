"""Unit tests for the experimental IMU attitude / heading feedback (attitude.py)."""

import math

from barq_control.attitude import (AxisPID, HeadingHold, leg_feasible, LEGS, limit_to_reach,
                                   nominal_pitch, roll_pitch_of, rotate_feet)
from barq_control.gait import foot_targets

HIPS = {'FL': [0.108484, 0.0171913, 0.00022012], 'FR': [0.108484, -0.0148092, 0.00022176],
        'RL': [-0.108371, 0.0167905, 0.00022176], 'RR': [-0.108371, -0.01521, 0.00022012]}


def _quat(roll, pitch):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    return (sr * cp, cr * sp, -sr * sp, cr * cp)   # x, y, z, w for yaw = 0


def test_roll_pitch_roundtrip():
    for r, p in [(0.0, 0.0), (0.1, -0.05), (-0.2, 0.15)]:
        rr, pp = roll_pitch_of(*_quat(r, p))
        assert abs(rr - r) < 1e-9 and abs(pp - p) < 1e-9


def test_nominal_pitch_matches_measured_stance():
    # Physics settle measured +0.0793 rad nose-down with rear_raise 0.02 (D-016).
    assert abs(nominal_pitch(HIPS, 0.02) - 0.0793) < 5e-4


def test_rotation_is_rigid():
    feet = foot_targets(0.0, 0.0, 0.0, 0.0, HIPS)
    rot = rotate_feet(feet, 0.07, -0.05)
    for i in range(4):
        a = math.dist([0, 0, 0], feet[3 * i:3 * i + 3])
        b = math.dist([0, 0, 0], rot[3 * i:3 * i + 3])
        assert abs(a - b) < 1e-12


def test_pitch_correction_sign():
    """Nose too low (pitch err > 0) -> PID asks nose-up -> front feet go deeper (extend)."""
    pid = AxisPID(1.0, 0.0, 0.0, 0.2, 0.1)
    c = pid.step(0.05, 0.0, 0.02)
    assert c < 0.0
    feet = foot_targets(0.0, 0.0, 0.0, 0.0, HIPS)
    rot = rotate_feet(feet, 0.0, c)
    assert rot[2] < feet[2] and rot[5] < feet[5]      # FL, FR deeper
    assert rot[8] > feet[8] and rot[11] > feet[11]    # RL, RR shallower


def test_roll_correction_sign():
    """Left side too high (roll err > 0) -> left legs shorten, right legs extend."""
    pid = AxisPID(1.0, 0.0, 0.0, 0.2, 0.1)
    c = pid.step(0.05, 0.0, 0.02)
    feet = foot_targets(0.0, 0.0, 0.0, 0.0, HIPS)
    rot = rotate_feet(feet, c, 0.0)
    assert rot[2] > feet[2] and rot[8] > feet[8]      # FL, RL (left) shallower
    assert rot[5] < feet[5] and rot[11] < feet[11]    # FR, RR (right) deeper


def test_gyro_is_damping():
    pid = AxisPID(0.0, 0.0, 1.0, 1.0, 1.0)
    assert pid.step(0.0, 0.5, 0.02) < 0.0              # rolling +: push back


def test_output_clamp_and_windup():
    pid = AxisPID(10.0, 10.0, 0.0, 0.1, 0.05)
    for _ in range(1000):
        c = pid.step(1.0, 0.0, 0.02)
    assert abs(c) <= 0.1 + 1e-12 and abs(pid.i) <= 0.05 + 1e-12


def test_limit_to_reach_always_feasible():
    """Across a whole gait cycle with a large correction, every emitted foot is reachable."""
    for k in range(50):
        t = k * 0.01
        nom = foot_targets(t, 0.15, 0.0, 0.0, HIPS)
        corr = rotate_feet(nom, 0.12, 0.12)            # deliberately aggressive
        out, s = limit_to_reach(nom, corr, HIPS)
        assert 0.0 <= s <= 1.0
        for i, leg in enumerate(LEGS):
            assert leg_feasible(out[3 * i:3 * i + 3], HIPS[leg], leg)


def test_heading_hold_counters_veer():
    h = HeadingHold(0.5, 1.0, 0.4, 0.5)
    wz = h.step(0.0, 0.05, 0.02)                       # robot yawing left while told straight
    assert wz < 0.0
    assert abs(HeadingHold(0.5, 1.0, 0.4, 0.5).step(0.3, 0.3, 0.02) - 0.3) < 1e-12


def test_ilc_cancels_a_delayed_periodic_disturbance():
    """Plant: rock d(phase) plus the correction applied `delay` bins late. ILC must shrink it."""
    from barq_control.attitude import PhaseILC
    n, delay = 25, 3
    ilc = PhaseILC(n, gain=0.3, lead=delay, forget=1.0, limit=1.0)
    hist = [0.0] * delay
    rms = []
    for cycle in range(40):
        acc = 0.0
        for k in range(n):
            ph = k / n
            u = ilc.output(ph)
            hist.append(u)
            e = 0.06 * math.sin(2 * math.pi * ph) + hist[-1 - delay]
            ilc.learn(ph, e)
            acc += e * e
        rms.append(math.sqrt(acc / n))
    assert rms[-1] < 0.25 * rms[0]


def test_abs_heading_hold():
    from barq_control.attitude import AbsHeadingHold
    h = AbsHeadingHold(1.5, 0.0, 0.4)
    h.step(0.0, 0.0, 0.0, 0.02)                        # capture ref = 0
    assert h.step(0.0, 0.1, 0.0, 0.02) < 0.0           # drifted left -> turn right
    h2 = AbsHeadingHold(1.5, 0.0, 0.4)
    yaw = 0.0
    h2.step(0.3, yaw, 0.3, 0.02)                       # ref captured, then advanced one step
    for _ in range(50):                                # commanded turn is not fought
        yaw += 0.3 * 0.02
        wz = h2.step(0.3, yaw, 0.3, 0.02)
    assert abs(wz - 0.3) < 1e-6
