"""
BARQ single-leg kinematics (idealized 3-DOF analytical model).

Frame (per leg, REP-103 body frame): x forward, y left, z up. Origin at the hip (coxa) joint.
Joints:
  q1 = coxa  (abduction, rotates about body +X)
  q2 = femur (rotates about body +Y)
  q3 = tibia (rotates about body +Y; knee bend)

Link lengths come from robot_params.yaml (derived from the URDF):
  L1 = coxa_length  (lateral hip->knee offset)
  L2 = femur_length
  L3 = tibia_length

`side` = +1 for left legs (FL, RL: hip y > 0), -1 for right legs (FR, RR).

At all-zero angles the leg points straight down: foot = (0, side*L1, -(L2+L3)).

NOTE: this is the idealized model the project standardised on (clean link lengths). It ignores
the small mesh-frame offsets in the URDF (sub-cm); foot placement is exact *for this model* and
round-trips to machine precision (see test_ik.py). Knee-bend convention: q3 <= 0 (knee_bend=-1,
legs fold forward; confirmed visually on BARQ and matches the servo tibia range [-1.571, 0]).
"""

import math


def fk_leg(q1, q2, q3, L1, L2, L3, side):
    """Forward kinematics: joint angles -> foot position (x, y, z) in the hip frame."""
    # Femur + tibia in the leg's sagittal plane (x forward, z down-negative).
    foot_x = L2 * math.sin(q2) + L3 * math.sin(q2 + q3)
    foot_z = -(L2 * math.cos(q2) + L3 * math.cos(q2 + q3))
    # Lateral coxa offset, then rotate the (y, z) by the coxa angle about +X.
    y0 = side * L1
    z0 = foot_z
    x = foot_x
    y = y0 * math.cos(q1) - z0 * math.sin(q1)
    z = y0 * math.sin(q1) + z0 * math.cos(q1)
    return (x, y, z)


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def _wrap(a):
    """Normalize an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def ik_leg(x, y, z, L1, L2, L3, side, knee_bend=-1.0):
    """
    Inverse kinematics: foot position (hip frame) -> (q1, q2, q3).

    knee_bend selects the elbow branch (two mirrored solutions reach the same foot):
      -1 (default): q3 <= 0, legs fold FORWARD — BARQ's physical configuration (Q-010),
                    and the only branch inside the servo tibia range [-1.571, 0].
      +1          : q3 >= 0, legs fold backward (mirrored; kept for tests/analysis).
    Raises ValueError if the target is outside the leg's reachable workspace.
    """
    R = math.hypot(y, z)
    if R < abs(L1):
        raise ValueError(f'target inside coxa radius (R={R:.4f} < L1={L1:.4f})')

    # Coxa: cos(q1 - phi) = side*L1 / R, take the downward-hanging branch.
    phi = math.atan2(z, y)
    q1 = phi + math.acos(_clamp(side * L1 / R))

    # Inverse-rotate to recover the sagittal-plane foot (x stays, z0 is the in-plane vertical).
    z0 = -y * math.sin(q1) + z * math.cos(q1)
    D = -z0  # downward reach (positive)

    # Planar 2-link (L2, L3) for the foot at (x forward, D down).
    r2 = x * x + D * D
    cos_q3 = _clamp((r2 - L2 * L2 - L3 * L3) / (2.0 * L2 * L3))
    q3 = knee_bend * math.acos(cos_q3)
    q2 = math.atan2(x, D) - math.atan2(L3 * math.sin(q3), L2 + L3 * math.cos(q3))
    return (_wrap(q1), _wrap(q2), _wrap(q3))


def side_of(hip_offset_y):
    """+1 (left) if the hip is on the +y side, else -1 (right)."""
    return 1.0 if hip_offset_y >= 0.0 else -1.0
