#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from tf2_msgs.msg import TFMessage


class ViconTFToPose(Node):
    """Convert Vicon TF to body pose setpoint."""
    def __init__(self):
        super().__init__('vicon_tf_to_pose')

        self.declare_parameter('target_frame', 'vicon/robodog_magnetic/robodog_magnetic')
        self.declare_parameter('world_frame', 'vicon/world')
        self.declare_parameter('pose_topic', 'body_pose')
        self.declare_parameter('nominal_height', 0.225)

        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.nominal_height = self.get_parameter('nominal_height').get_parameter_value().double_value

        self.pose_pub = self.create_publisher(Pose, pose_topic, 10)

        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.create_subscription(TFMessage, '/tf_static', self.tf_callback, 10)

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if (transform.child_frame_id == self.target_frame and
                    transform.header.frame_id == self.world_frame):
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = self.nominal_height
                pose.orientation = transform.transform.rotation
                self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = ViconTFToPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
