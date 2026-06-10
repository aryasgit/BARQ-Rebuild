"""
BARQ trot gait — open-loop foot-trajectory generation (Stage 2D).

Pure functions (no ROS) so they can be unit-tested. Given a gait time `t` and a body velocity
command (vx, vy, wz), produce 12 foot targets in the body frame for the IK node.

Trot = two diagonal pairs alternating swing/stance: (FL, RR) in phase, (FR, RL) half a cycle later.
Each leg foot cycles:
  - stance (phase < duty): foot on the ground, sweeping backward to push the body forward.
  - swing  (phase >= duty): foot lifts (sine arc) and swings forward to the next foothold.

Step size scales with the commanded velocity; at zero command the feet hold the neutral stance.
"""

import math

LEGS = ['FL', 'FR', 'RL', 'RR']
# Diagonal trot pairing: FL+RR together, FR+RL half a cycle offset.
PHASE = {'FL': 0.0, 'RR': 0.0, 'FR': 0.5, 'RL': 0.5}


def _side(hip_y):
    return 1.0 if hip_y >= 0.0 else -1.0


def foot_targets(t, vx, vy, wz, hip_offsets, lateral=0.0754692, kx_front=0.01744,
                 period=0.5, duty=0.5, step_height=0.02, stand_height=0.13, deadband=1e-3):
    """
    Return 12 body-frame foot coords [FLxyz, FRxyz, RLxyz, RRxyz] at gait time t.

    vx, vy: body linear velocity (m/s); wz: yaw rate (rad/s).
    Neutral foot: knee-x offset forward/back of the hip (exact URDF model), `lateral`
    outboard, `stand_height` below. Constraint: stand_height - step_height >= ~0.095 m,
    else the swing apex demands tibia beyond -2.2 (exact-model min in-plane reach ~0.092).
    """
    moving = (abs(vx) + abs(vy) + abs(wz)) > deadband
    out = []
    for leg in LEGS:
        hx, hy, hz = hip_offsets[leg]
        kx = kx_front if leg.startswith('F') else -kx_front
        nx = hx + kx                      # neutral foot under the knee-x (support symmetric)
        ny = hy + _side(hy) * lateral     # ... and the true lateral offset outboard
        nz = hz - stand_height
        if not moving:
            out += [nx, ny, nz]
            continue
        # Per-foot ground velocity needed for the commanded body motion (incl. yaw r x w).
        sx = (vx - wz * ny) * period * duty
        sy = (vy + wz * nx) * period * duty
        phase = ((t / period) + PHASE[leg]) % 1.0
        if phase < duty:                         # stance: sweep + -> - (push back)
            prog = phase / duty
            dx, dy, dz = sx * (0.5 - prog), sy * (0.5 - prog), 0.0
        else:                                    # swing: sweep - -> + and lift
            prog = (phase - duty) / (1.0 - duty)
            dx = sx * (prog - 0.5)
            dy = sy * (prog - 0.5)
            dz = step_height * math.sin(math.pi * prog)
        out += [nx + dx, ny + dy, nz + dz]
    return out
