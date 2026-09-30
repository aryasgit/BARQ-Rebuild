#!/usr/bin/env python3
"""
Drive a straight-line walk in sim and report ground-truth displacement metrics.

Publishes /cmd_vel for --duration sim-seconds (gait must be running), brackets
the run with /odom_gt poses, prints one parseable WALK line:
dx/dy in the odom frame, yaw drift, realized speed vs commanded.

Used for the foot-friction sweep (D-018) and as a walking regression metric.
  python3 sim_walk_metric.py --vx 0.15 --duration 10
"""

import argparse
import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


NOMINAL_PITCH = 0.0793   # designed stance trim (D-016), rad nose-down


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def roll_pitch_of(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    return roll, math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))


def _rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else float('nan')


class WalkMetric(Node):

    def __init__(self):
        super().__init__('sim_walk_metric')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.odom = None
        self.recording = False
        self.samples = []
        self.create_subscription(Odometry, '/odom_gt', self._on_odom, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_odom(self, msg):
        self.odom = msg
        if self.recording:
            r, p = roll_pitch_of(msg.pose.pose.orientation)
            self.samples.append((r, p, msg.pose.pose.position.z))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, t_end, tick=None, tick_dt=0.05):
        next_tick = 0.0
        while self._now() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if tick and self._now() >= next_tick:
                tick()
                next_tick = self._now() + tick_dt

    def run(self, vx, duration, settle):
        while self.odom is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.spin_until(self._now() + settle)
        p0 = self.odom.pose.pose
        x0, y0, yaw0 = p0.position.x, p0.position.y, yaw_of(p0.orientation)

        cmd = Twist()
        cmd.linear.x = vx
        self.recording = True
        self.spin_until(self._now() + duration, tick=lambda: self.pub.publish(cmd))
        self.recording = False
        for _ in range(5):
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_until(self._now() + 1.0)

        p1 = self.odom.pose.pose
        dx_w, dy_w = p1.position.x - x0, p1.position.y - y0
        # displacement in the robot's initial heading frame (forward / lateral)
        fwd = dx_w * math.cos(yaw0) + dy_w * math.sin(yaw0)
        lat = -dx_w * math.sin(yaw0) + dy_w * math.cos(yaw0)
        dyaw = math.atan2(math.sin(yaw_of(p1.orientation) - yaw0),
                          math.cos(yaw_of(p1.orientation) - yaw0))
        print(f'WALK vx={vx:.2f} T={duration:.1f}s  fwd={fwd:+.3f}m lat={lat:+.3f}m '
              f'yaw={dyaw:+.3f}rad  speed={fwd / duration:.3f}m/s '
              f'({fwd / duration / vx * 100.0:.0f}% of commanded)')
        # Attitude from GROUND TRUTH (not the IMU the controller uses): skip first cycle.
        a = self.samples[len(self.samples) // 10:]
        rolls = [x[0] for x in a]
        perr = [x[1] - NOMINAL_PITCH for x in a]
        zs = [x[2] for x in a]
        zm = sum(zs) / len(zs)
        print(f'ATT  n={len(a)} roll_rms={_rms(rolls) * 1e3:.1f}mrad '
              f'roll_max={max(abs(x) for x in rolls) * 1e3:.1f}mrad '
              f'pitch_err_rms={_rms(perr) * 1e3:.1f}mrad '
              f'pitch_err_mean={sum(perr) / len(perr) * 1e3:+.1f}mrad '
              f'pitch_err_max={max(abs(x) for x in perr) * 1e3:.1f}mrad '
              f'heave_rms={_rms([z - zm for z in zs]) * 1e3:.1f}mm z_mean={zm:.4f}m')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--vx', type=float, default=0.15)
    ap.add_argument('--duration', type=float, default=10.0)
    ap.add_argument('--settle', type=float, default=2.0)
    args = ap.parse_args()

    rclpy.init()
    node = WalkMetric()
    try:
        node.run(args.vx, args.duration, args.settle)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
