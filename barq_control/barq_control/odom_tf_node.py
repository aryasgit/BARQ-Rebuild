"""
Odometry -> TF bridge (STOPGAP).

Re-publishes nav_msgs/Odometry (/odom, currently Gazebo ground truth) as the odom->base_link
transform that slam_toolbox/nav need. Replaced by the real state estimator + legged odometry
later; keeping it a separate node makes that swap a one-line launch change.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTf(Node):
    """Mirror /odom poses onto the TF tree as odom -> base_link."""

    def __init__(self):
        """Subscribe /odom and broadcast the equivalent transform."""
        super().__init__('odom_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/odom', self._on_odom, 20)
        self.get_logger().info('odom_tf up: /odom -> TF odom->base_link (ground-truth stopgap)')

    def _on_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        # Frame names FORCED (Gazebo plugins sometimes model-prefix them, e.g. 'barq/odom',
        # which silently breaks the odom->laser chain slam_toolbox needs).
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    """Spin the odom->TF bridge."""
    rclpy.init()
    rclpy.spin(OdomTf())


if __name__ == '__main__':
    main()
