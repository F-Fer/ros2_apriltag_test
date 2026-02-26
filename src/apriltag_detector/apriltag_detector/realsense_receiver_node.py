#!/usr/bin/env python3
"""
RealSense Receiver Node
Receives color + aligned depth frames from realsense_sender.py (running on Mac host)
via ZMQ and republishes them as ROS 2 topics for lego_detector_node.

Publishes:
  /camera/color/image_raw                  sensor_msgs/Image  (bgr8)
  /camera/aligned_depth_to_color/image_raw sensor_msgs/Image  (16UC1, mm)
  /camera/color/camera_info                sensor_msgs/CameraInfo
"""

import json

import numpy as np
import rclpy
import zmq
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class RealSenseReceiverNode(Node):
    def __init__(self):
        super().__init__('realsense_receiver_node')

        self.declare_parameter('zmq_port', 5556)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('fps', 30.0)

        zmq_port      = self.get_parameter('zmq_port').value
        self.frame_id = self.get_parameter('frame_id').value
        fps           = self.get_parameter('fps').value

        self.bridge = CvBridge()

        # ZMQ PULL socket
        self.zmq_context = zmq.Context()
        self.socket = self.zmq_context.socket(zmq.PULL)
        self.socket.bind(f"tcp://*:{zmq_port}")
        self.socket.setsockopt(zmq.RCVTIMEO, 100)   # ms — non-blocking poll

        # Publishers
        self.color_pub = self.create_publisher(
            Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(
            Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, '/camera/color/camera_info', 10)

        self.timer = self.create_timer(1.0 / fps, self.timer_callback)

        self.get_logger().info(f'RealSense receiver listening on ZMQ port {zmq_port}')
        self.get_logger().info('Waiting for realsense_sender.py on Mac host...')

    # ------------------------------------------------------------------
    def timer_callback(self):
        try:
            parts = self.socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        except Exception as e:
            self.get_logger().error(f'ZMQ recv error: {e}')
            return

        if len(parts) != 3:
            self.get_logger().warn(f'Expected 3-part message, got {len(parts)}')
            return

        meta_bytes, color_bytes, depth_bytes = parts

        try:
            meta = json.loads(meta_bytes.decode())
        except Exception as e:
            self.get_logger().error(f'Metadata parse error: {e}')
            return

        now = self.get_clock().now().to_msg()

        # --- Color image ---
        import cv2
        color_np = cv2.imdecode(np.frombuffer(color_bytes, np.uint8), cv2.IMREAD_COLOR)
        if color_np is None:
            self.get_logger().warn('Failed to decode color JPEG')
            return

        color_msg = self.bridge.cv2_to_imgmsg(color_np, encoding='bgr8')
        color_msg.header.stamp    = now
        color_msg.header.frame_id = self.frame_id
        self.color_pub.publish(color_msg)

        # --- Depth image (raw uint16, units = depth_scale metres, typically mm) ---
        dw = meta['depth_width']
        dh = meta['depth_height']
        depth_np = np.frombuffer(depth_bytes, dtype=np.uint16).reshape((dh, dw))

        # Convert to millimetres if depth_scale != 0.001
        depth_scale = meta.get('depth_scale', 0.001)
        if abs(depth_scale - 0.001) > 1e-6:
            # Rescale so values represent mm (multiply by depth_scale / 0.001)
            factor = depth_scale / 0.001
            depth_np = np.clip(depth_np.astype(np.float32) * factor,
                               0, 65535).astype(np.uint16)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_np, encoding='16UC1')
        depth_msg.header.stamp    = now
        depth_msg.header.frame_id = self.frame_id
        self.depth_pub.publish(depth_msg)

        # --- CameraInfo ---
        info_msg = self._build_camera_info(meta)
        info_msg.header.stamp    = now
        info_msg.header.frame_id = self.frame_id
        self.info_pub.publish(info_msg)

    # ------------------------------------------------------------------
    def _build_camera_info(self, meta: dict) -> CameraInfo:
        msg = CameraInfo()
        w  = meta['width']
        h  = meta['height']
        fx = meta['fx']
        fy = meta['fy']
        cx = meta['cx']
        cy = meta['cy']

        msg.width  = w
        msg.height = h
        msg.distortion_model = 'plumb_bob'
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [fx,  0.0, cx,
                 0.0, fy,  cy,
                 0.0, 0.0, 1.0]
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
        msg.p = [fx,  0.0, cx,  0.0,
                 0.0, fy,  cy,  0.0,
                 0.0, 0.0, 1.0, 0.0]
        return msg

    # ------------------------------------------------------------------
    def destroy_node(self):
        self.socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
