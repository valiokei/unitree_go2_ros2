#!/usr/bin/env python3
"""
gz_pose_bridge.py  –  Gazebo->ROS2 ground-truth bridge
-----------------------------------------------------
ROS parameters
  • topic        (string)  = "/world/default/pose/info"
  • model_name   (string)  = "go2"
  • child_frame  (string)  = "base_footprint"
  • pub_tf       (bool)    = True
Publishes
  • <model_name>/gt_pose   (geometry_msgs/PoseStamped)
  • TF world → <child_frame>   (optional)
"""

import threading, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

# Gazebo Transport (python3-gz-transportX) -----------------------------

import importlib

# ───────── transport binding ─────────
# for name in ("gz.transport", "gz.transport14", "gz.transport13"):
#     try:
#         gz_transport = importlib.import_module(name)
#         break
#     except ModuleNotFoundError:
#         pass
# else:
#     raise ImportError("Gazebo Python 'gz.transport*' non trovato")

# # ───────── protobuf messages ─────────
# for prefix in ("gz.msgs", "gz.msgs12", "gz.msgs11", "gz.msgs10"):
#     try:
#         Pose    = importlib.import_module(f"{prefix}.pose_pb2").Pose
#         Pose_V  = importlib.import_module(f"{prefix}.pose_v_pb2").Pose_V
#         break
#     except ModuleNotFoundError:
#         continue
# else:
#     raise ImportError("Moduli 'gz.msgs*/pose_pb2' non trovati")


import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2    import Pose        # singola pose
from gz.msgs10.pose_v_pb2  import Pose_V      # array di pose




class GzPoseBridge(Node):
    """Bridge Gazebo Pose(_V) → ROS PoseStamped + TF"""
    def __init__(self):
        super().__init__('gz_pose_bridge')

        # ──────────── Parametri configurabili ────────────
        self.declare_parameter('topic',      '/world/default/pose/info')
        self.declare_parameter('model_name', 'go2')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('pub_tf',     True)

        self.gz_topic     = self.get_parameter('topic').get_parameter_value().string_value
        self.model_name   = self.get_parameter('model_name').get_parameter_value().string_value
        self.child_frame  = self.get_parameter('child_frame').get_parameter_value().string_value
        self.publish_tf   = self.get_parameter('pub_tf').get_parameter_value().bool_value

        # ──────────── Publisher ROS + TF ────────────
        self.pub_pose = self.create_publisher(PoseStamped,
                                              f'/{self.model_name}/gt_pose', 10)
        self.br = TransformBroadcaster(self) if self.publish_tf else None

        # ──────────── Nodo Gazebo Transport ────────────
        self.gz_node = gz_transport.Node()
        self._attach_subscriber()

        self.get_logger().info(
            f'Bridging [{self.gz_topic}] → ROS (/{self.model_name}/gt_pose , TF)')

    # ------------------------------------------------------------------
    def _attach_subscriber(self):
        """Collega il callback corretto al topic Gazebo."""
        print(f"Attaching subscriber to topic: {self.gz_topic}")
        if self.gz_topic.endswith('/pose/info'):
            print("Using Pose_V (vector of poses)")
            # topic vector → Pose_V
            self.gz_node.subscribe(
                Pose_V, self.gz_topic, self._cb_pose_v)           
        else:
            print("Using Pose (single pose)")
            # topic singolo → Pose
            self.gz_node.subscribe(Pose, self.gz_topic, self._cb_pose)
    # ------------------------------------------------------------------
    def _cb_pose(self, msg_bin):
        self.get_logger().info(
            f"Received Pose: {msg_bin[:50]}... (truncated)")
        pose = Pose()
        pose.ParseFromString(msg_bin)
        self._publish_ros(pose)

    def _cb_pose_v(self, msg):
        # self.get_logger().info(
        #     f"Received Pose_V {(msg)} ")
        for p in msg.pose:
            self.get_logger().info(
                f"Received Pose: name={p.name}, position=({p.position.x}, {p.position.y}, {p.position.z}), "
                f"orientation=({p.orientation.x}, {p.orientation.y}, {p.orientation.z}, {p.orientation.w})")
            # Se il nome della posa corrisponde al modello, pubblica
            if p.name == self.model_name:
                self.get_logger().info(
                    f"Publishing pose for model: {self.model_name}")
                # Pubblica la posa su ROS e TF
                self._publish_ros(p)
                break

    # ------------------------------------------------------------------
    def _publish_ros(self, p):
        now = self.get_clock().now().to_msg()

        # PoseStamped
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = 'world'
        ps.pose.position.x = p.position.x
        ps.pose.position.y = p.position.y
        ps.pose.position.z = p.position.z
        ps.pose.orientation.x = p.orientation.x
        ps.pose.orientation.y = p.orientation.y
        ps.pose.orientation.z = p.orientation.z
        ps.pose.orientation.w = p.orientation.w
        self.pub_pose.publish(ps)
        self.get_logger().info(
            f"Published PoseStamped: position=({p.position.x}, {p.position.y}, {p.position.z}), "
            f"orientation=({p.orientation.x}, {p.orientation.y}, {p.orientation.z}, {p.orientation.w})")
        
        # TF (opzionale)
        if self.br:
            self.get_logger().info(
                f"Publishing TF: world -> {self.child_frame} at {now.sec}.{now.nanosec}")

            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = 'world'
            tf.child_frame_id = self.child_frame
            tf.transform.translation.x = p.position.x
            tf.transform.translation.y = p.position.y
            tf.transform.translation.z = p.position.z
            tf.transform.rotation.x = p.orientation.x
            tf.transform.rotation.y = p.orientation.y
            tf.transform.rotation.z = p.orientation.z
            tf.transform.rotation.w = p.orientation.w
            self.get_logger().info(
                f"Transform: translation=({p.position.x}, {p.position.y}, {p.position.z}), "
                f"rotation=({p.orientation.x}, {p.orientation.y}, {p.orientation.z}, {p.orientation.w})")
            self.br.sendTransform(tf)

    # ------------------------------------------------------------------
    # (Nessun event-loop Gazebo necessario: i callback sono asincroni)


def main():
    rclpy.init()
    bridge = GzPoseBridge()
    rclpy.spin(bridge)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
