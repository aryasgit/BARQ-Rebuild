"""
BARQ state estimator (legged odometry v1).

Fuses IMU orientation with stance-leg kinematic odometry:
  - yaw / angular rate from /imu/data (sim IMU now; BNO085 SH-2 quaternion on hardware)
  - body planar velocity from the feet currently in stance: planted feet are world-fixed,
    so their body-frame motion is the negative of the body's motion (exact-model FK).
Publishes nav_msgs/Odometry on /odom_est (frame odom -> base_link) and, when publish_tf
is true, the odom->base_link transform (replacing the ground-truth stopgap).

v1 limits (documented for the research log): planar (x, y, yaw) only; stance detection is
"two lowest feet" (valid for trot); no slip rejection; drift is EXPECTED - measuring it
against ground truth is the point.
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from barq_control.leg_kinematics import fk_exact, kx_of, side_of
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster
import yaml

LEGS = ['FL', 'FR', 'RL', 'RR']
JOINTS = {leg: (f'{leg}_hip_joint', f'{leg}_knee_joint', f'{leg}_ankle_joint') for leg in LEGS}


def yaw_of(q):
    """Return the yaw of a quaternion (x, y, z, w)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def feet_body_positions(joint_pos, hips):
    """Map joint angles (name->rad) to body-frame foot positions {leg: (x, y, z)}."""
    feet = {}
    for leg in LEGS:
        names = JOINTS[leg]
        if any(n not in joint_pos for n in names):
            return None
        q1, q2, q3 = (joint_pos[n] for n in names)
        hx, hy, hz = hips[leg]
        fx, fy, fz = fk_exact(q1, q2, q3, kx_of(leg), side_of(hy))
        feet[leg] = (hx + fx, hy + fy, hz + fz)
    return feet


def stance_legs(feet):
    """
    Return the stance diagonal: the trot pair (FL+RR or FR+RL) with the lower z-sum.

    Compared per-DIAGONAL, not per-foot: each diagonal holds one front + one rear leg,
    so the rear_raise stance trim (D-016) cancels out of the comparison. (A naive
    two-lowest-feet rule always picks the two trimmed-deeper rear legs - that bug
    zeroed the odometry entirely.)
    """
    a = feet['FL'][2] + feet['RR'][2]
    b = feet['FR'][2] + feet['RL'][2]
    return ['FL', 'RR'] if a <= b else ['FR', 'RL']


def stance_velocity(prev_feet, feet, stance, dt):
    """
    Estimate body planar velocity from stance-foot deltas.

    Planted feet are fixed in the world; if a stance foot moves (dx, dy) in the body
    frame, the body moved (-dx, -dy). Average over the stance set.
    """
    if dt <= 0.0 or not stance:
        return (0.0, 0.0)
    vx = sum(-(feet[s][0] - prev_feet[s][0]) for s in stance) / (len(stance) * dt)
    vy = sum(-(feet[s][1] - prev_feet[s][1]) for s in stance) / (len(stance) * dt)
    return (vx, vy)


class StateEstimator(Node):
    """Legged odometry: IMU yaw + stance-leg FK velocity -> /odom_est (+ optional TF)."""

    def __init__(self):
        """Load geometry, subscribe IMU + joint states, integrate at 50 Hz."""
        super().__init__('state_estimator')
        path = os.path.join(get_package_share_directory('barq_description'),
                            'config', 'robot_params.yaml')
        with open(path) as f:
            legs = yaml.safe_load(f)['legs']
        self.hips = {leg: legs['hip_offsets'][leg] for leg in LEGS}

        self.declare_parameter('publish_tf', False)
        self.declare_parameter('rate', 50.0)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.dt = 1.0 / float(self.get_parameter('rate').value)

        self.joint_pos = {}
        self.yaw = 0.0
        self.wz = 0.0
        self.have_imu = False
        self.prev_feet = None
        self.x = self.y = 0.0
        self.vx_f = self.vy_f = 0.0          # low-passed body velocity

        self.pub = self.create_publisher(Odometry, '/odom_est', 10)
        self.br = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Imu, '/imu/data', self._on_imu, 50)
        self.create_subscription(JointState, '/joint_states', self._on_js, 50)
        self.create_timer(self.dt, self._tick)
        self.get_logger().info(
            f'state_estimator up (publish_tf={self.publish_tf}): legged odometry v1')

    def _on_imu(self, msg):
        self.yaw = yaw_of(msg.orientation)
        self.wz = msg.angular_velocity.z
        self.have_imu = True

    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joint_pos[n] = p

    def _tick(self):
        if not self.have_imu:
            return
        feet = feet_body_positions(self.joint_pos, self.hips)
        if feet is None:
            return
        if self.prev_feet is not None:
            vx, vy = stance_velocity(self.prev_feet, feet, stance_legs(feet), self.dt)
            a = 0.4                                  # low-pass: gait-cycle ripple rejection
            self.vx_f += a * (vx - self.vx_f)
            self.vy_f += a * (vy - self.vy_f)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (c * self.vx_f - s * self.vy_f) * self.dt
            self.y += (s * self.vx_f + c * self.vy_f) * self.dt
        self.prev_feet = feet

        now = self.get_clock().now().to_msg()
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        od.twist.twist.linear.x = self.vx_f
        od.twist.twist.linear.y = self.vy_f
        od.twist.twist.angular.z = self.wz
        self.pub.publish(od)

        if self.br is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = od.pose.pose.orientation
            self.br.sendTransform(t)


def main():
    """Spin the state estimator."""
    rclpy.init()
    rclpy.spin(StateEstimator())


if __name__ == '__main__':
    main()
