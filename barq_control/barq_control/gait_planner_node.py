"""
BARQ gait planner node (Stage 2D).

Subscribes /cmd_vel (geometry_msgs/Twist), generates trot foot trajectories, and streams 12
body-frame foot targets to /foot_targets at a fixed rate. The IK node turns those into joint
commands. At zero /cmd_vel the feet hold the neutral stance (no stepping).

Gait params (period, duty, step_height, stand_height, rate) are ROS parameters.
Geometry (hip offsets, coxa length) is read from barq_description/config/robot_params.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from barq_control.gait import foot_targets, LEGS
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import yaml


class GaitPlanner(Node):
    """Trot gait: /cmd_vel -> body-frame foot trajectories on /foot_targets."""

    def __init__(self):
        """Load geometry/params and start publishing foot targets at the configured rate."""
        super().__init__('gait_planner')
        legs = self._load_params()['legs']
        self.hip = {leg: legs['hip_offsets'][leg] for leg in LEGS}

        self.declare_parameter('period', 0.5)
        # duty >0.5 = stance overlap: calmer load transfer, less heave (D-019)
        self.declare_parameter('duty', 0.6)
        # Exact-model geometry; constraint: stand - step >= ~0.095 m (tibia -2.2 at apex).
        # step 0.02 gives real swing clearance (foot sphere r=0.012 + contact/staircase margins).
        self.declare_parameter('step_height', 0.02)
        self.declare_parameter('stand_height', 0.13)
        # Stance trim (Aryaman): rear legs extended by this much -> nose-down pitch, load
        # shifts to the front feet, prevents backward body roll. ~5.3 deg at 0.02.
        self.declare_parameter('rear_raise', 0.02)
        self.declare_parameter('rate', 50.0)
        # Forward = body +X (Aryaman, watching the PHYSICS walk in Gazebo, 2026-06-11; this is
        # the arc direction approved in the RViz reversal session). +1 => cmd_vel +x drives the
        # body toward +X. Flip to -1 only if the frame convention is ever re-decided (Q-012).
        self.declare_parameter('forward_sign', 1.0)
        self.fwd = float(self.get_parameter('forward_sign').value)
        self.period = float(self.get_parameter('period').value)
        self.duty = float(self.get_parameter('duty').value)
        self.step_height = float(self.get_parameter('step_height').value)
        self.stand_height = float(self.get_parameter('stand_height').value)
        self.rear_raise = float(self.get_parameter('rear_raise').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)

        # Deadman: zero the command if /cmd_vel goes silent (Ctrl-C of a teleop publisher
        # must STOP the robot, not freeze the last velocity forever).
        self.declare_parameter('cmd_timeout', 1.0)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.last_cmd_time = None

        self.vx = self.vy = self.wz = 0.0
        self.t = 0.0

        self.pub = self.create_publisher(Float64MultiArray, '/foot_targets', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info('gait_planner up: trot on /cmd_vel -> /foot_targets')

    def _load_params(self):
        path = os.path.join(get_package_share_directory('barq_description'),
                            'config', 'robot_params.yaml')
        with open(path) as f:
            return yaml.safe_load(f)

    def _on_cmd(self, msg):
        self.vx, self.vy, self.wz = msg.linear.x, msg.linear.y, msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def _tick(self):
        self.t += self.dt
        if self.last_cmd_time is not None and (self.vx or self.vy or self.wz):
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age > self.cmd_timeout:
                self.vx = self.vy = self.wz = 0.0
                self.get_logger().info('cmd_vel silent %.1fs - deadman stop' % age)
        # cmd_vel is robot-centric (+x = forward = body +X per forward_sign above); yaw about
        # Z is unchanged by the mapping.
        ft = foot_targets(self.t, self.fwd * self.vx, self.fwd * self.vy, self.wz, self.hip,
                          period=self.period, duty=self.duty,
                          step_height=self.step_height, stand_height=self.stand_height,
                          rear_raise=self.rear_raise)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in ft]
        self.pub.publish(msg)


def main():
    """Spin the gait planner node."""
    rclpy.init()
    rclpy.spin(GaitPlanner())


if __name__ == '__main__':
    main()
