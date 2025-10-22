#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')

        self.origin = {
            'x': -2.0,
            'y': -0.5,
            'z': 0.0,
        }
        
        # Create subscriber to /tf_gt
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf_gt',
            self.tf_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create publisher to /tf
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10)
        
        self.get_logger().info('TF Republisher node initialized - Looking for default->scout_mini transform')

    def tf_callback(self, msg):
        # Look for the specific transform we want
        target_parent = 'empty_world'
        target_child = 'scout_mini'
        
        found_transform = None
        
        for transform in msg.transforms:
            if transform.header.frame_id == target_parent and transform.child_frame_id == target_child:
                found_transform = transform
                break
        
        if found_transform is None:
            self.get_logger().debug('Transform from {} to {} not found in this message'.format(
                target_parent, target_child), throttle_duration_sec=1)
            return
            
        # Create a new transform with modified frames
        new_transform = TransformStamped()
        
        # Copy header and modify frame_ids
        new_transform.header = found_transform.header
        new_transform.header.frame_id = 'odom'          # new parent frame
        new_transform.child_frame_id = 'base_footprint'  # new child frame
        
        # Copy the transform data (translation and rotation)
        new_transform.transform = found_transform.transform

        # Modify the translation to the origin
        new_transform.transform.translation.x -= self.origin['x']
        new_transform.transform.translation.y -= self.origin['y']
        new_transform.transform.translation.z -= self.origin['z']
        
        # Create and publish the new TFMessage
        new_msg = TFMessage()
        new_msg.transforms = [new_transform]
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()