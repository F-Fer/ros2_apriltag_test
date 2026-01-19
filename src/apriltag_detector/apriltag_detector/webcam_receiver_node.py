#!/usr/bin/env python3
"""
Webcam Receiver Node
Receives webcam frames via ZMQ from Mac host and publishes as ROS 2 images.

This node is needed because Docker cannot directly access Mac webcam.
A separate Python script runs on the Mac to capture webcam frames.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import zmq
import cv2


class WebcamReceiverNode(Node):
    def __init__(self):
        super().__init__('webcam_receiver_node')
        
        # Parameters
        self.declare_parameter('zmq_port', 5555)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30.0)
        
        zmq_port = self.get_parameter('zmq_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        fps = self.get_parameter('fps').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # ZMQ setup (use zmq_context to avoid conflict with ROS Node.context)
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PULL)
        self.socket.bind(f"tcp://*:{zmq_port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout
        
        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        
        # Timer to poll ZMQ
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f'Webcam Receiver initialized on port {zmq_port}')
        self.get_logger().info('Waiting for frames from webcam_sender.py...')
        self.get_logger().info('Run: python3 webcam_sender.py on Mac host')
    
    def timer_callback(self):
        """Poll for new frames from ZMQ."""
        try:
            # Receive frame data
            data = self.socket.recv(flags=zmq.NOBLOCK)
            
            # Decode frame
            nparr = np.frombuffer(data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                # Get current time
                now = self.get_clock().now().to_msg()
                
                # Publish image
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                img_msg.header.stamp = now
                img_msg.header.frame_id = self.frame_id
                self.image_pub.publish(img_msg)
                
                # Publish camera info
                info_msg = self.create_camera_info(frame.shape[1], frame.shape[0])
                info_msg.header.stamp = now
                info_msg.header.frame_id = self.frame_id
                self.camera_info_pub.publish(info_msg)
                
        except zmq.Again:
            # No message available
            pass
        except Exception as e:
            self.get_logger().error(f'Error receiving frame: {e}')
    
    def create_camera_info(self, width, height):
        """Create camera info message with approximate MacBook webcam intrinsics."""
        msg = CameraInfo()
        msg.width = width
        msg.height = height
        
        # Approximate intrinsics for MacBook webcam
        fx = 600.0
        fy = 600.0
        cx = width / 2.0
        cy = height / 2.0
        
        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]
        
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        
        msg.distortion_model = 'plumb_bob'
        
        return msg
    
    def destroy_node(self):
        """Clean up ZMQ resources."""
        self.socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebcamReceiverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
