import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_publisher')
        
        # Initialize the StaticTransformBroadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Create a static transform
        transform = TransformStamped()
        
        # Fill in the transform details
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'  # Parent frame
        transform.child_frame_id = 'meleca'    # Child frame
        
        # Translation (x, y, z)
        transform.transform.translation.x = 1.0
        transform.transform.translation.y = 0.5
        transform.transform.translation.z = 0.0
        
        # Rotation (quaternion x, y, z, w)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # No rotation (identity quaternion)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info('Published static transform from parent_frame to child_frame')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()