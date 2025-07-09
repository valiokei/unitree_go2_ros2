#!/usr/bin/env python3

"""
Nodo che legge la posa del robot da Gazebo e pubblica la trasformazione TF world->base_footprint
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
import tf_transformations

class GazeboPose2TF(Node):
    def __init__(self):
        super().__init__('gazebo_pose2tf')
        
        # TF broadcaster
        self.br = TransformBroadcaster(self)
        
        # Subscribe al topic dinamico di Gazebo che contiene le pose
        self.create_subscription(
            TFMessage,
            '/world/default/dynamic_pose/info',
            self.dynamic_pose_callback,
            10
        )
        
        self.get_logger().info('Gazebo Pose2TF node started - subscribing to /world/default/dynamic_pose/info')
        
        # Variabili per tracking posizione
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0

    def dynamic_pose_callback(self, msg):
        """Callback per il topic dynamic_pose/info di Gazebo"""
        
        # Il messaggio contiene un array di transforms
        # Cerchiamo il transform del nostro robot (dovrebbe essere il primo)
        if len(msg.transforms) > 0:
            transform = msg.transforms[0]  # Prendi il primo (dovrebbe essere il robot go2)
            
            # Estrai posizione
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Estrai orientazione (quaternion)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Calcola yaw dall'orientazione quaternion
            _, _, yaw = tf_transformations.euler_from_quaternion([qx, qy, qz, qw])
            
            # Pubblica transform world -> base_footprint
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'world'
            t.child_frame_id = 'base_footprint'
            
            # Usa la posa diretta del robot
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0  # base_footprint è sempre a terra
            
            # Mantieni solo yaw per base_footprint
            q_footprint = tf_transformations.quaternion_from_euler(0, 0, yaw)
            t.transform.rotation.x = q_footprint[0]
            t.transform.rotation.y = q_footprint[1]
            t.transform.rotation.z = q_footprint[2]
            t.transform.rotation.w = q_footprint[3]
            
            self.br.sendTransform(t)
            
            # Log della posizione per debug, solo se è cambiata significativamente
            if (abs(x - self.last_x) > 0.01 or 
                abs(y - self.last_y) > 0.01 or 
                abs(yaw - self.last_yaw) > 0.01):
                self.get_logger().info(f'Robot pose from Gazebo: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
                self.last_x = x
                self.last_y = y
                self.last_yaw = yaw

def main():
    rclpy.init()
    rclpy.spin(GazeboPose2TF())

if __name__ == '__main__':
    main()
