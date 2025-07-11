#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_msgs.msg import TFMessage
import tf_transformations


class TfFollower(Node):
    def __init__(self):
        super().__init__('tf_follower')

        self.declare_parameter('target_frame', 'vicon/robodog_magnetic/robodog_magnetic')
        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('kp_linear', 1.0)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('tf_topic', '/vicon_tf')
        self.declare_parameter('tf_static_topic', '/vicon_tf_static')

        self.target_frame = self.get_parameter('target_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.tf_topic = self.get_parameter('tf_topic').value
        self.tf_static_topic = self.get_parameter('tf_static_topic').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f"Tracking {self.target_frame} with respect to {self.robot_frame}"
        )

        self.create_subscription(TFMessage, self.tf_topic, self._tf_cb, 10)
        self.create_subscription(TFMessage, self.tf_static_topic, self._tf_static_cb, 10)

        self.target_transform = None

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def _tf_cb(self, msg: TFMessage):
        for tr in msg.transforms:
            if tr.child_frame_id == self.target_frame:
                first = self.target_transform is None
                self.target_transform = tr
                if first:
                    self.get_logger().info('Received first target transform')
            self.tf_buffer.set_transform(tr, 'vicon_bag')

    def _tf_static_cb(self, msg: TFMessage):
        for tr in msg.transforms:
            self.tf_buffer.set_transform_static(tr, 'vicon_bag')

    def control_loop(self):
        if self.target_transform is None:
            return

        try:
            robot = self.tf_buffer.lookup_transform(
                'world',
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().debug('Robot transform not available yet')
            return

        target = self.target_transform

        dx = target.transform.translation.x - robot.transform.translation.x
        dy = target.transform.translation.y - robot.transform.translation.y

        rq = robot.transform.rotation
        tq = target.transform.rotation

        _, _, robot_yaw = tf_transformations.euler_from_quaternion([rq.x, rq.y, rq.z, rq.w])
        _, _, target_yaw = tf_transformations.euler_from_quaternion([tq.x, tq.y, tq.z, tq.w])

        err_x = math.cos(robot_yaw) * dx + math.sin(robot_yaw) * dy
        err_y = -math.sin(robot_yaw) * dx + math.cos(robot_yaw) * dy
        yaw_err = math.atan2(math.sin(target_yaw - robot_yaw), math.cos(target_yaw - robot_yaw))

        twist = Twist()
        twist.linear.x = self.kp_linear * err_x
        twist.linear.y = self.kp_linear * err_y
        twist.angular.z = self.kp_angular * yaw_err

        twist.linear.x = max(min(twist.linear.x, 0.6), -0.6)
        twist.linear.y = max(min(twist.linear.y, 0.6), -0.6)
        twist.angular.z = max(min(twist.angular.z, 1.5), -1.5)

        self.cmd_pub.publish(twist)
        self.get_logger().debug(
            f"cmd_vel: [{twist.linear.x:.2f}, {twist.linear.y:.2f}, {twist.angular.z:.2f}]"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TfFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
