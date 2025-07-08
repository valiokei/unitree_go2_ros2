#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from builtin_interfaces.msg import Duration


class JointTrajectoryToJointStates(Node):
    def __init__(self):
        super().__init__('joint_trajectory_to_joint_states')
        
        # Subscribe to joint trajectory commands from CHAMP controller
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_group_effort_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )
        
        # Publish joint states for visualization
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.get_logger().info('Joint trajectory to joint states bridge started')
        
    def trajectory_callback(self, msg):
        if not msg.points:
            return
            
        # Take the first point from the trajectory
        point = msg.points[0]
        
        # Create joint state message
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.header.frame_id = ''
        
        joint_state.name = msg.joint_names
        joint_state.position = point.positions
        
        # Add velocities if available
        if point.velocities:
            joint_state.velocity = point.velocities
        else:
            joint_state.velocity = [0.0] * len(joint_state.name)
            
        # Add efforts if available  
        if point.effort:
            joint_state.effort = point.effort
        else:
            joint_state.effort = [0.0] * len(joint_state.name)
        
        # Publish the joint state
        self.joint_state_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    
    node = JointTrajectoryToJointStates()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
