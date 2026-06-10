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
        self.L1 = legs['coxa_length']
        self.hip = {leg: legs['hip_offsets'][leg] for leg in LEGS}

        self.declare_parameter('period', 0.5)
        self.declare_parameter('duty', 0.5)
        # Deep crouch. Constraint: stand_height - step_height must stay >= ~0.103 m, else the
        # swing apex demands tibia beyond the -2.2 judgment limit (min 2-link reach ~0.094 m).
        self.declare_parameter('step_height', 0.012)
        self.declare_parameter('stand_height', 0.115)
        self.declare_parameter('rate', 50.0)
        self.period = float(self.get_parameter('period').value)
        self.duty = float(self.get_parameter('duty').value)
        self.step_height = float(self.get_parameter('step_height').value)
        self.stand_height = float(self.get_parameter('stand_height').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)

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

    def _tick(self):
        self.t += self.dt
        # Robot FRONT is the body's -X end (head per the mesh; URDF leg labels disagree, Q-012).
        # cmd_vel is robot-centric (+x = head-first), so map into body axes by negating linear
        # x,y; yaw about Z is unchanged. This reverses the gait arc so it steps head-first.
        ft = foot_targets(self.t, -self.vx, -self.vy, self.wz, self.hip, self.L1,
                          period=self.period, duty=self.duty,
                          step_height=self.step_height, stand_height=self.stand_height)
        msg = Float64MultiArray()
        msg.data = [float(v) for v in ft]
        self.pub.publish(msg)


def main():
    """Spin the gait planner node."""
    rclpy.init()
    rclpy.spin(GaitPlanner())


if __name__ == '__main__':
    main()
