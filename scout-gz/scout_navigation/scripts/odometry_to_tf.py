#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_conjugate, quaternion_matrix
import numpy as np

class OdometryTfPublisher(Node):
    def __init__(self):
        super().__init__('odometry_tf_publisher')
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10)
        
        self.get_logger().info('Node iniciado, publicando TF /odometry/filtered como filho de base_link')

    def odom_callback(self, msg):
        transform = TransformStamped()
        
        # Configurar o cabeçalho da transformada
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'base_link'  # Frame pai
        transform.child_frame_id = 'odometry/filtered'  # Frame filho

        # Extrair posição e orientação da odometria
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        translation = np.array([pos.x, pos.y, pos.z])
        quat = np.array([ori.x, ori.y, ori.z, ori.w])
        
        # Calcular a inversa da orientação (conjugado do quaternion)
        quat_inv = quaternion_conjugate(quat)
        
        # Aplicar a rotação inversa à posição
        rot_matrix = quaternion_matrix(quat_inv)[:3, :3]
        trans_inv = -np.dot(rot_matrix, translation)

        # Preencher a transformação
        transform.transform.translation.x = trans_inv[0]
        transform.transform.translation.y = trans_inv[1]
        transform.transform.translation.z = trans_inv[2]

        transform.transform.rotation.x = quat_inv[0]
        transform.transform.rotation.y = quat_inv[1]
        transform.transform.rotation.z = quat_inv[2]
        transform.transform.rotation.w = quat_inv[3]

        # Enviar a TF
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()