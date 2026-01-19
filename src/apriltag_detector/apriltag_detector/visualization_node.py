#!/usr/bin/env python3
"""
Visualization Node
Displays AprilTag detections in a window.

This node can be run either inside Docker (with X11 forwarding) or natively.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')
        
        self.declare_parameter('window_name', 'AutonOHM AprilTag Detector')
        self.window_name = self.get_parameter('window_name').value
        
        self.bridge = CvBridge()
        self.latest_image = None
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/apriltag/image_annotated',
            self.image_callback,
            10
        )
        
        # Timer for display update
        self.timer = self.create_timer(0.033, self.display_callback)  # ~30fps
        
        self.get_logger().info('Visualization node started')
        self.get_logger().info('Waiting for annotated images...')
    
    def image_callback(self, msg: Image):
        """Store latest image for display."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def display_callback(self):
        """Display the latest image."""
        if self.latest_image is not None:
            cv2.imshow(self.window_name, self.latest_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit requested')
                rclpy.shutdown()
    
    def destroy_node(self):
        """Clean up OpenCV windows."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
