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


def yaw_of(q):
    return 2.0 * math.atan2(q.z, q.w)


class WalkMetric(Node):

    def __init__(self):
        super().__init__('sim_walk_metric')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.odom = None
        self.create_subscription(Odometry, '/odom_gt', self._on_odom, 20)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_odom(self, msg):
        self.odom = msg

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
        self.spin_until(self._now() + duration, tick=lambda: self.pub.publish(cmd))
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
