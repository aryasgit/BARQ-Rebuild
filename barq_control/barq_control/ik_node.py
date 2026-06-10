"""
BARQ IK node (Stage 2C).

Subscribes foot targets (body frame), runs per-leg analytical IK, and streams 12 joint positions to
the position controller. Publishes a default standing stance until foot targets arrive.

Input  : std_msgs/Float64MultiArray on /foot_targets - 12 values (m), body frame (REP-103):
         [FLx,FLy,FLz, FRx,FRy,FRz, RLx,RLy,RLz, RRx,RRy,RRz]
Output : std_msgs/Float64MultiArray on /joint_group_position_controller/commands - 12 joint
         positions (rad), order FL/FR/RL/RR x (coxa, femur, tibia).

Geometry (link lengths + hip offsets) is read from barq_description/config/robot_params.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from barq_control.leg_kinematics import ik_leg, side_of
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import yaml

LEGS = ['FL', 'FR', 'RL', 'RR']
COXA_LIMIT = 0.785
KNEE_LIMIT = 1.57
ANKLE_LIMIT = 1.57   # Q-001: URDF value; tighten if the tibia turns out to be one-directional


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class IKNode(Node):
    """Foot targets -> per-leg analytical IK -> 12 joint commands for the position controller."""

    def __init__(self):
        """Load geometry, set the default stance, and start streaming joint commands at 50 Hz."""
        super().__init__('ik_node')
        legs = self._load_params()['legs']
        self.L1 = legs['coxa_length']
        self.L2 = legs['femur_length']
        self.L3 = legs['tibia_length']
        self.hip = {leg: legs['hip_offsets'][leg] for leg in LEGS}

        self.declare_parameter('stance_height', 0.15)
        self.declare_parameter('knee_bend', 1.0)
        h = self.get_parameter('stance_height').value
        self.knee_bend = float(self.get_parameter('knee_bend').value)

        # Default stance: each foot L1 outboard of its hip and `h` below it (body frame).
        self.targets = []
        for leg in LEGS:
            hx, hy, hz = self.hip[leg]
            self.targets += [hx, hy + side_of(hy) * self.L1, hz - h]

        self.pub = self.create_publisher(
            Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.create_subscription(Float64MultiArray, '/foot_targets', self._on_targets, 10)
        self.create_timer(0.02, self._tick)   # 50 Hz
        self.get_logger().info('ik_node up: streaming default stance; listening on /foot_targets')

    def _load_params(self):
        path = os.path.join(get_package_share_directory('barq_description'),
                            'config', 'robot_params.yaml')
        with open(path) as f:
            return yaml.safe_load(f)

    def _on_targets(self, msg):
        if len(msg.data) != 12:
            self.get_logger().warn(f'/foot_targets needs 12 values, got {len(msg.data)}')
            return
        self.targets = list(msg.data)

    def _tick(self):
        cmd = []
        for i, leg in enumerate(LEGS):
            hx, hy, hz = self.hip[leg]
            fx, fy, fz = self.targets[3 * i:3 * i + 3]
            try:
                q1, q2, q3 = ik_leg(fx - hx, fy - hy, fz - hz,
                                    self.L1, self.L2, self.L3, side_of(hy), self.knee_bend)
            except ValueError as exc:
                self.get_logger().warn(f'{leg}: unreachable target: {exc}',
                                       throttle_duration_sec=2.0)
                return
            cmd += [_clamp(q1, COXA_LIMIT), _clamp(q2, KNEE_LIMIT), _clamp(q3, ANKLE_LIMIT)]
        msg = Float64MultiArray()
        msg.data = [float(v) for v in cmd]
        self.pub.publish(msg)


def main():
    """Spin the IK node."""
    rclpy.init()
    rclpy.spin(IKNode())


if __name__ == '__main__':
    main()
