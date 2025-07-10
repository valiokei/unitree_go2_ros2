#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
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
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self,
            tf_topic=self.tf_topic,
            static_tf_topic=self.tf_static_topic,
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

    def control_loop(self):
        try:
            target = self.tf_buffer.lookup_transform('world', self.target_frame, rclpy.time.Time())
            robot = self.tf_buffer.lookup_transform('world', self.robot_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

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
