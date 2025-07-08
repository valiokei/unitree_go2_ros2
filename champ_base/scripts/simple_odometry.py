#!/usr/bin/env python3

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster


class SimpleOdometry(Node):
    def __init__(self):
        super().__init__('simple_odometry')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Current velocities
        self.linear_x = 0.0
        self.linear_y = 0.0  # Add support for lateral movement
        self.angular_z = 0.0
        
        # Timer for odometry integration
        self.timer = self.create_timer(0.05, self.update_odometry)  # 20Hz
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('Simple odometry node started for RViz-only mode')
        
    def cmd_vel_callback(self, msg):
        """Update current velocities from cmd_vel"""
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y  # Capture lateral velocity
        self.angular_z = msg.angular.z
        
    def update_odometry(self):
        """Integrate velocity and publish TF"""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Holonomic integration (supports lateral movement)
        # Transform velocities from robot frame to world frame
        dx_world = self.linear_x * math.cos(self.theta) - self.linear_y * math.sin(self.theta)
        dy_world = self.linear_x * math.sin(self.theta) + self.linear_y * math.cos(self.theta)
        
        self.x += dx_world * dt
        self.y += dy_world * dt
        self.theta += self.angular_z * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_footprint'
        
        # Set translation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0  # base_footprint stays on ground
        
        # Set rotation (quaternion from yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
