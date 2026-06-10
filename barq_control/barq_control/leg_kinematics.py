"""
BARQ single-leg kinematics.

Two models live here:

EXACT (use this — fk_exact / ik_exact): matches the URDF joint chain to machine precision
(verified against a rotation-matrix composition of the raw URDF origins in
test_exact_kinematics.py). Frame (hip-relative, REP-103): x forward, y left, z up.
  q1 = hip/coxa  (about +X), q2 = knee/femur (about +Y), q3 = ankle/tibia (about +Y)

URDF chain (lengths in m, from barq.urdf.xacro):
  knee origin in coxa frame : (KX, side*0.0430692, 0)   KX = +0.01744 front, -0.01744 rear
  ankle origin in femur frame: (0.018944, side*0.0324, -0.1)
  foot point in tibia frame  : (0, 0, -0.1)
Because the coxa rotates about X, the x-offsets are q1-invariant; the two lateral offsets
combine into LAT = 0.0754692. The femur's (0.018944, -0.1) in-plane vector has magnitude
L2P at fixed angle A2 from straight-down. Note: positive q2 (about +Y) tilts the femur
toward -X — this sign is URDF truth and was the root cause of an early direction bug.

LEGACY (fk_leg / ik_leg): the old idealized 3-DOF model (clean link lengths, q2 sign
mirrored vs the URDF). Kept only for its unit tests; do NOT use for control. Its frame
offsets error is up to 3.4 cm at the front feet — see docs/03_CHANGELOG.md 2026-06-11.
"""

import math

# Exact URDF-derived constants (computed, not rounded; robot_params.yaml mirrors them).
KX_FRONT = 0.01744                    # knee-origin x in coxa frame, front legs (rear = -)
KNEE_Y = 0.0430692                    # knee-origin lateral offset (per side)
ANKLE_X = 0.018944                    # ankle-origin x in femur frame
ANKLE_Y = 0.0324                      # ankle-origin lateral offset (per side)
FEMUR_Z = 0.1                         # femur vertical drop to ankle
LAT = KNEE_Y + ANKLE_Y                # combined q1-plane lateral offset = 0.0754692
L2P = math.hypot(ANKLE_X, FEMUR_Z)    # femur in-plane length
A2 = math.atan2(ANKLE_X, FEMUR_Z)     # femur fixed in-plane angle
L3 = 0.100                            # tibia: ankle -> foot point


def _clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def _wrap(a):
    """Normalize an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def side_of(hip_offset_y):
    """+1 (left) if the hip is on the +y side, else -1 (right)."""
    return 1.0 if hip_offset_y >= 0.0 else -1.0


def kx_of(leg_name, kx_front=KX_FRONT):
    """Knee-origin x offset for a leg name ('FL','FR' front +, 'RL','RR' rear -)."""
    return kx_front if leg_name.startswith('F') else -kx_front


# ───────────────────────────── EXACT MODEL ─────────────────────────────

def fk_exact(q1, q2, q3, kx, side, lat=LAT, l2p=L2P, a2=A2, l3=L3):
    """URDF-true forward kinematics: joint angles -> foot point (x, y, z), hip frame."""
    x = kx - l2p * math.sin(q2 - a2) - l3 * math.sin(q2 + q3)
    zp = -(l2p * math.cos(q2 - a2) + l3 * math.cos(q2 + q3))
    y0 = side * lat
    y = y0 * math.cos(q1) - zp * math.sin(q1)
    z = y0 * math.sin(q1) + zp * math.cos(q1)
    return (x, y, z)


def ik_exact(x, y, z, kx, side, knee_bend=-1.0, lat=LAT, l2p=L2P, a2=A2, l3=L3):
    """
    URDF-true inverse kinematics: foot point (hip frame) -> (q1, q2, q3).

    knee_bend=-1 is BARQ's physical fold (matches the visually-confirmed stance).
    Raises ValueError if the target is outside the workspace laterally.
    """
    R = math.hypot(y, z)
    if R < lat:
        raise ValueError(f'target inside lateral-offset radius (R={R:.4f} < {lat:.4f})')

    phi = math.atan2(z, y)
    q1 = _wrap(phi + math.acos(_clamp(side * lat / R)))

    z0 = -y * math.sin(q1) + z * math.cos(q1)
    D = -z0                       # in-plane downward reach (positive)
    xm = -(x - kx)                # mirrored sagittal coordinate (URDF +q2 tilts femur -X)

    r2 = xm * xm + D * D
    cosd = _clamp((r2 - l2p * l2p - l3 * l3) / (2.0 * l2p * l3))
    d = knee_bend * math.acos(cosd)
    psi = math.atan2(xm, D) - math.atan2(l3 * math.sin(d), l2p + l3 * math.cos(d))
    return (q1, _wrap(psi + a2), _wrap(d - a2))


# ──────────────────────── LEGACY IDEALIZED MODEL ────────────────────────

def fk_leg(q1, q2, q3, L1, L2, L3_, side):
    """LEGACY idealized FK (q2 sign mirrored vs URDF). Not for control use."""
    foot_x = L2 * math.sin(q2) + L3_ * math.sin(q2 + q3)
    foot_z = -(L2 * math.cos(q2) + L3_ * math.cos(q2 + q3))
    y0 = side * L1
    y = y0 * math.cos(q1) - foot_z * math.sin(q1)
    z = y0 * math.sin(q1) + foot_z * math.cos(q1)
    return (foot_x, y, z)


def ik_leg(x, y, z, L1, L2, L3_, side, knee_bend=-1.0):
    """LEGACY idealized IK (q2 sign mirrored vs URDF). Not for control use."""
    R = math.hypot(y, z)
    if R < abs(L1):
        raise ValueError(f'target inside coxa radius (R={R:.4f} < L1={L1:.4f})')
    phi = math.atan2(z, y)
    q1 = phi + math.acos(_clamp(side * L1 / R))
    z0 = -y * math.sin(q1) + z * math.cos(q1)
    D = -z0
    r2 = x * x + D * D
    cos_q3 = _clamp((r2 - L2 * L2 - L3_ * L3_) / (2.0 * L2 * L3_))
    q3 = knee_bend * math.acos(cos_q3)
    q2 = math.atan2(x, D) - math.atan2(L3_ * math.sin(q3), L2 + L3_ * math.cos(q3))
    return (_wrap(q1), _wrap(q2), _wrap(q3))
