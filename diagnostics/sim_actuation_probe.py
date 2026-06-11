#!/usr/bin/env python3
"""
Sim actuation honesty probe — is Gazebo moving joints like ST3215s or teleporting them?

Two modes (run inside the sim container with the stack up):

  step   Command a +delta step on one joint, record /joint_states, report
         rise time (10-90%), peak |velocity|, overshoot, settle time.
         An ideal/teleporting sim shows rise <= 1-2 control ticks and peak
         velocity far beyond the servo's physical ceiling (ST3215 @12V:
         4.71 rad/s no-load). A physical sim slews at <= the URDF velocity
         limit and takes ~delta/v_max to arrive.

  track  Record commanded vs actual positions for all 12 joints during
         walking (gait must be running); report per-joint RMS and peak
         |error|. This is the sim twin of the bench step/sweep tests in
         st3215_diag.py — same metrics, so once real servos are measured
         the sim stiffness can be tuned to match.

Examples:
  python3 sim_actuation_probe.py step --joint FL_knee_joint --delta 0.3
  python3 sim_actuation_probe.py track --duration 12
"""

import argparse
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# Order must match ros2_controllers.yaml joint_group_position_controller.joints
JOINTS = [f'{leg}_{part}_joint' for leg in ('FL', 'FR', 'RL', 'RR')
          for part in ('hip', 'knee', 'ankle')]
CMD_TOPIC = '/joint_group_position_controller/commands'
ART_DIR = os.path.expanduser('~/barq_ws/artifacts')


class Probe(Node):

    def __init__(self):
        super().__init__('sim_actuation_probe')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.idx = None                      # JointState name order -> JOINTS order
        self.js = None                       # latest (pos[12], vel[12]) in JOINTS order
        self.js_log = []                     # (t, pos[12], vel[12])
        self.cmd_log = []                    # (t, [12 floats])
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.create_subscription(Float64MultiArray, CMD_TOPIC, self._on_cmd, 50)
        self.pub = self.create_publisher(Float64MultiArray, CMD_TOPIC, 10)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_js(self, msg):
        if self.idx is None:
            if not all(j in msg.name for j in JOINTS):
                return
            self.idx = [msg.name.index(j) for j in JOINTS]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos = [msg.position[i] for i in self.idx]
        vel = [msg.velocity[i] for i in self.idx] if msg.velocity else [0.0] * 12
        self.js = (pos, vel)
        self.js_log.append((t, pos, vel))

    def _on_cmd(self, msg):
        self.cmd_log.append((self._now(), list(msg.data)))

    def spin_for(self, seconds):
        end = self._now() + seconds
        while self._now() < end:
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait_for_state(self, timeout=15.0):
        end = self._now() + timeout
        while self.js is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._now() > end:
                raise RuntimeError('no /joint_states — is the sim up?')

    def send(self, targets):
        self.pub.publish(Float64MultiArray(data=list(targets)))


def run_step(probe, joint, delta, hold):
    probe.wait_for_state()
    probe.spin_for(0.5)
    start = list(probe.js[0])
    probe.send(start)                        # take ownership at current pose: no lurch
    probe.spin_for(1.0)
    idx = JOINTS.index(joint)
    target = start[idx] + delta
    probe.js_log.clear()
    t0 = probe._now()
    stepped = list(start)
    stepped[idx] = target
    probe.send(stepped)
    probe.spin_for(hold)
    probe.send(start)                        # return home
    probe.spin_for(1.0)

    series = [(t - t0, p[idx], v[idx]) for t, p, v in probe.js_log if t >= t0]
    p0 = series[0][1]
    lo, hi = p0 + 0.1 * delta, p0 + 0.9 * delta
    sgn = 1.0 if delta >= 0 else -1.0
    t10 = next((t for t, p, _ in series if sgn * (p - lo) >= 0), None)
    t90 = next((t for t, p, _ in series if sgn * (p - hi) >= 0), None)
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else float('nan')
    vpeak = max(abs(v) for _, _, v in series)
    over = max(sgn * (p - target) for _, p, _ in series) / abs(delta) * 100.0
    settle = next((t for t, p, _ in series
                   if all(abs(q - target) <= 0.02 * abs(delta)
                          for tt, q, _ in series if tt >= t)), float('nan'))

    os.makedirs(ART_DIR, exist_ok=True)
    path = os.path.join(ART_DIR, f'step_{joint}.csv')
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['t', 'pos', 'vel'])
        w.writerows(series)
    print(f'STEP {joint} delta={delta:+.3f} rad  ({len(series)} samples -> {path})')
    print(f'  rise time 10-90% : {rise * 1000.0:7.1f} ms'
          f'   (pure slew at 4.71 rad/s would be {abs(delta) * 0.8 / 4.71 * 1000:.0f} ms)')
    print(f'  peak |velocity|  : {vpeak:7.2f} rad/s  (ST3215 @12V ceiling: 4.71)')
    print(f'  overshoot        : {over:7.1f} %')
    print(f'  settle (2%) at   : {settle * 1000.0:7.1f} ms')
    verdict = ('TELEPORT-LIKE (limits not enforced)' if vpeak > 6.0 or rise < 0.02
               else 'physical (velocity-limited slew)')
    print(f'  verdict          : {verdict}')


def run_track(probe, duration):
    probe.wait_for_state()
    end = probe._now() + 20.0
    while not probe.cmd_log:                 # gait is silent until /cmd_vel flows
        rclpy.spin_once(probe, timeout_sec=0.05)
        if probe._now() > end:
            raise RuntimeError('no commands on %s — is the gait walking?' % CMD_TOPIC)
    probe.js_log.clear()
    probe.cmd_log.clear()
    probe.spin_for(duration)

    n_js, n_cmd = len(probe.js_log), len(probe.cmd_log)
    print(f'TRACK {duration:.0f}s  ({n_js} state samples, {n_cmd} command samples)')
    if n_js < duration * 40 or n_cmd < duration * 25:
        print('  WARNING: subscriber starved (expect ~100 Hz states, ~50 Hz commands) '
              '- errors below pair stale commands and are NOT valid')
    print(f'  {"joint":<16} {"RMS err":>10} {"peak err":>10}')
    worst = (None, 0.0)
    for i, j in enumerate(JOINTS):
        errs = []
        ci = 0
        for t, p, _ in probe.js_log:
            while ci + 1 < n_cmd and probe.cmd_log[ci + 1][0] <= t:
                ci += 1
            errs.append(p[i] - probe.cmd_log[ci][1][i])
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        peak = max(abs(e) for e in errs)
        if peak > worst[1]:
            worst = (j, peak)
        print(f'  {j:<16} {rms * 1000:7.1f}mrad {peak * 1000:7.1f}mrad')
    print(f'  worst joint: {worst[0]} peak {worst[1] * 1000:.1f} mrad')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=['step', 'track'])
    ap.add_argument('--joint', default='FL_knee_joint')
    ap.add_argument('--delta', type=float, default=0.3)
    ap.add_argument('--hold', type=float, default=2.0)
    ap.add_argument('--duration', type=float, default=12.0)
    args = ap.parse_args()

    rclpy.init()
    probe = Probe()
    try:
        if args.mode == 'step':
            run_step(probe, args.joint, args.delta, args.hold)
        else:
            run_track(probe, args.duration)
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
