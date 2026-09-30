"""
BARQ body-attitude + heading feedback (experimental, branch exp/attitude-control).

Pure functions / plain classes (no ROS) so they are unit-testable.

Idea: the open-loop trot places feet assuming the body stays at its designed attitude. Here the
IMU (at base_link origin = model CoM) closes the loop:

  roll/pitch : PID on attitude error -> a corrective BODY rotation c = (c_roll, c_pitch).
               Realised by rigidly rotating every body-frame foot target by R(c)^T about the
               body origin: planted feet stay world-fixed, so the body rotates by c over them;
               swing feet keep a ground-parallel path. Gyro rates are the D term (no numeric
               differentiation of a noisy angle).
  yaw        : heading hold — PI on (commanded yaw rate - measured gyro z), added to wz.
               The integral of a rate error is a heading error, so straight commands stay
               straight (targets the open-loop duty-dependent veer, Q-016).

Reach safety: front feet sit ~2 mm from the tibia fold floor at swing apex (D-019), so every
corrected foot is checked with the exact IK + joint limits and, if infeasible, its correction is
scaled back by bisection. The node never emits a target the leg cannot reach.
"""

import math

from barq_control.leg_kinematics import ik_exact, kx_of, side_of

LEGS = ['FL', 'FR', 'RL', 'RR']
COXA_LIMIT = 0.785
KNEE_LIMIT = 1.57
ANKLE_MIN, ANKLE_MAX = -2.2, 0.0


def roll_pitch_of(qx, qy, qz, qw):
    """Return (roll, pitch) of a world-referenced quaternion (ZYX / REP-103 convention)."""
    roll = math.atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy))
    s = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    return roll, math.asin(s)


def nominal_pitch(hip_offsets, rear_raise, kx_front=0.01744):
    """Designed stance pitch (+ = nose down) produced by the rear_raise trim on flat ground."""
    x_front = 0.5 * (hip_offsets['FL'][0] + hip_offsets['FR'][0]) + kx_front
    x_rear = 0.5 * (hip_offsets['RL'][0] + hip_offsets['RR'][0]) - kx_front
    return math.atan2(rear_raise, x_front - x_rear)


def rotate_feet(feet, c_roll, c_pitch):
    """
    Rotate 12 body-frame foot coords so the BODY rotates by (c_roll, c_pitch) over them.

    feet' = R(c)^T feet, R = Ry(c_pitch) Rx(c_roll). Positive c_pitch = nose down (REP-103).
    """
    cr, sr = math.cos(c_roll), math.sin(c_roll)
    cp, sp = math.cos(c_pitch), math.sin(c_pitch)
    out = []
    for i in range(4):
        x, y, z = feet[3 * i:3 * i + 3]
        # R^T = Rx(c_roll)^T Ry(c_pitch)^T : first undo pitch, then undo roll
        x1 = cp * x - sp * z
        z1 = sp * x + cp * z
        y2 = cr * y + sr * z1
        z2 = -sr * y + cr * z1
        out += [x1, y2, z2]
    return out


def leg_feasible(foot, hip, leg, knee_bend=-1.0):
    """Return True if the body-frame foot target is reachable within the joint limits."""
    hx, hy, hz = hip
    try:
        q1, q2, q3 = ik_exact(foot[0] - hx, foot[1] - hy, foot[2] - hz,
                              kx_of(leg), side_of(hy), knee_bend)
    except ValueError:
        return False
    return (abs(q1) <= COXA_LIMIT and abs(q2) <= KNEE_LIMIT
            and ANKLE_MIN <= q3 <= ANKLE_MAX)


def limit_to_reach(nominal, corrected, hips, iters=8):
    """
    Per leg, keep the largest fraction s in [0, 1] of (corrected - nominal) that is feasible.

    Returns (feet, min_scale). The nominal gait target is assumed feasible (s = 0).
    """
    out, smin = [], 1.0
    for i, leg in enumerate(LEGS):
        n = nominal[3 * i:3 * i + 3]
        c = corrected[3 * i:3 * i + 3]
        if leg_feasible(c, hips[leg], leg):
            out += c
            continue
        lo, hi = 0.0, 1.0
        for _ in range(iters):
            mid = 0.5 * (lo + hi)
            p = [n[k] + mid * (c[k] - n[k]) for k in range(3)]
            if leg_feasible(p, hips[leg], leg):
                lo = mid
            else:
                hi = mid
        out += [n[k] + lo * (c[k] - n[k]) for k in range(3)]
        smin = min(smin, lo)
    return out, smin


class AxisPID:
    """PID on one attitude axis with gyro-rate D term, output clamp and anti-windup."""

    def __init__(self, kp, ki, kd, out_limit, i_limit):
        """Store gains and limits; integrator starts at zero."""
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit, self.i_limit = out_limit, i_limit
        self.i = 0.0

    def step(self, err, rate, dt, freeze_i=False):
        """Return the corrective body rotation for error `err` (rad) and rate (rad/s)."""
        if not freeze_i:
            self.i = max(-self.i_limit, min(self.i_limit, self.i + err * dt))
        u = -(self.kp * err + self.ki * self.i + self.kd * rate)
        return max(-self.out_limit, min(self.out_limit, u))

    def reset(self):
        """Zero the integrator."""
        self.i = 0.0


class HeadingHold:
    """PI on yaw-rate error; the integral term is effectively a heading error."""

    def __init__(self, kp, ki, out_limit, i_limit):
        """Store gains and limits."""
        self.kp, self.ki, self.out_limit, self.i_limit = kp, ki, out_limit, i_limit
        self.i = 0.0

    def step(self, wz_cmd, wz_meas, dt):
        """Return the corrected yaw-rate command."""
        err = wz_cmd - wz_meas
        self.i = max(-self.i_limit, min(self.i_limit, self.i + err * dt))
        corr = self.kp * err + self.ki * self.i
        return wz_cmd + max(-self.out_limit, min(self.out_limit, corr))

    def reset(self):
        """Zero the integrator."""
        self.i = 0.0


class PhaseILC:
    """
    Phase-indexed iterative learning control for the periodic trot rock (one axis).

    A table ff[k] (k = gait-phase bin) holds a feed-forward corrective body rotation. Each
    tick at bin k the measured error e updates the bin applied `lead` bins EARLIER
    (ff[k - lead] -= gain * e), so next cycle the correction arrives ahead of the servo /
    pipeline lag that defeats plain feedback on a 2 Hz disturbance. A circular 3-tap smooth
    (Q-filter) and slight forgetting keep the learning robust to noise and gait changes.
    """

    def __init__(self, bins=25, gain=0.3, lead=3, forget=0.998, limit=0.10):
        """Create an empty (zero) feed-forward table."""
        self.n, self.gain, self.lead = bins, gain, lead
        self.forget, self.limit = forget, limit
        self.ff = [0.0] * bins
        self._last_k = None

    def bin_of(self, phase):
        """Map a gait phase in [0, 1) to a table bin."""
        return int(phase * self.n) % self.n

    def output(self, phase):
        """Return the feed-forward correction for this phase."""
        return self.ff[self.bin_of(phase)]

    def learn(self, phase, err):
        """Update the table with the error measured at this phase."""
        k = self.bin_of(phase)
        j = (k - self.lead) % self.n
        v = self.forget * (self.ff[j] - self.gain * err)
        self.ff[j] = max(-self.limit, min(self.limit, v))
        if self._last_k is not None and k < self._last_k:     # cycle wrapped: Q-filter
            f = self.ff
            self.ff = [0.25 * f[i - 1] + 0.5 * f[i] + 0.25 * f[(i + 1) % self.n]
                       for i in range(self.n)]
        self._last_k = k

    def decay(self, factor=0.95):
        """Fade the table (used while standing still)."""
        self.ff = [factor * v for v in self.ff]
        self._last_k = None
