#!/usr/bin/env python3
"""
Lego Duplo Detector Node for AutonOHM RoboCup @Work
Detects Lego Duplo bricks by HSV color thresholding + contour analysis.
Outputs 3D camera-frame positions using aligned RealSense depth.

Subscribes:
  /camera/color/image_raw             sensor_msgs/Image (BGR8)
  /camera/aligned_depth_to_color/image_raw  sensor_msgs/Image (16UC1, mm)
  /camera/color/camera_info           sensor_msgs/CameraInfo

Publishes:
  /lego/detections/{color}            geometry_msgs/PoseArray
  /lego/image_annotated               sensor_msgs/Image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import message_filters
import cv2
import numpy as np


# BGR annotation colors keyed by color name
ANNOTATION_COLORS = {
    'red':    (0,   0,   220),
    'blue':   (220, 80,  0),
    'yellow': (0,   210, 230),
    'green':  (0,   200, 0),
}
DEFAULT_ANNOTATION_COLOR = (200, 200, 200)

MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


class LegoDetectorNode(Node):
    def __init__(self):
        super().__init__('lego_detector_node')

        # --- Parameters ---
        self.declare_parameter('color_names', ['red', 'blue', 'yellow', 'green'])
        self.declare_parameter('min_contour_area', 500)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('sync_queue_size', 5)

        color_names = self.get_parameter('color_names').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        self.camera_frame = self.get_parameter('camera_frame').value
        sync_slop = self.get_parameter('sync_slop').value
        sync_queue_size = int(self.get_parameter('sync_queue_size').value)

        # Load per-color HSV configs
        self.color_configs = {}
        for color in color_names:
            int_array = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
            self.declare_parameter(f'{color}.hsv_lower',  [0,   0,   0])
            self.declare_parameter(f'{color}.hsv_upper',  [180, 255, 255])
            self.declare_parameter(f'{color}.hsv_lower2', [], int_array)
            self.declare_parameter(f'{color}.hsv_upper2', [], int_array)

            lower  = list(self.get_parameter(f'{color}.hsv_lower').value)
            upper  = list(self.get_parameter(f'{color}.hsv_upper').value)
            lower2 = list(self.get_parameter(f'{color}.hsv_lower2').value)
            upper2 = list(self.get_parameter(f'{color}.hsv_upper2').value)

            self.color_configs[color] = {
                'hsv_lower':  np.array(lower,  dtype=np.uint8),
                'hsv_upper':  np.array(upper,  dtype=np.uint8),
                'hsv_lower2': np.array(lower2, dtype=np.uint8) if lower2 else None,
                'hsv_upper2': np.array(upper2, dtype=np.uint8) if upper2 else None,
            }

        # --- State ---
        self.bridge = CvBridge()
        self.camera_info = None  # set once by camera_info callback

        # --- Publishers ---
        self.detection_pubs = {}
        for color in color_names:
            self.detection_pubs[color] = self.create_publisher(
                PoseArray, f'/lego/detections/{color}', 10)

        self.annotated_pub = self.create_publisher(Image, '/lego/image_annotated', 10)

        # --- Subscribers ---
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        color_sub = message_filters.Subscriber(
            self, Image, '/camera/color/image_raw')
        depth_sub = message_filters.Subscriber(
            self, Image, '/camera/aligned_depth_to_color/image_raw')

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [color_sub, depth_sub],
            queue_size=sync_queue_size,
            slop=sync_slop
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info(
            f'Lego detector started | colors: {list(color_names)} | '
            f'min_area: {self.min_contour_area}px²'
        )

    # ------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        """Store intrinsics once (they don't change)."""
        if self.camera_info is None:
            self.camera_info = msg
            self.get_logger().info(
                f'Camera intrinsics received: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} '
                f'cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}'
            )

    # ------------------------------------------------------------------
    def synced_callback(self, color_msg: Image, depth_msg: Image):
        """Main callback: detect bricks in all configured colors."""
        try:
            bgr   = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')
            return

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        annotated = bgr.copy()

        for color, cfg in self.color_configs.items():
            bricks = self.detect_color(hsv, depth, cfg)
            self.publish_detections(color, bricks, color_msg.header)
            self.draw_detections(annotated, color, bricks)

        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = color_msg.header
        self.annotated_pub.publish(annotated_msg)

    # ------------------------------------------------------------------
    def detect_color(self, hsv: np.ndarray, depth: np.ndarray, cfg: dict) -> list:
        """
        Detect bricks matching cfg's HSV range.
        Returns list of dicts with keys: cx, cy, x3d, y3d, z3d, contour, bbox
        """
        mask = cv2.inRange(hsv, cfg['hsv_lower'], cfg['hsv_upper'])
        if cfg['hsv_lower2'] is not None and len(cfg['hsv_lower2']) == 3:
            mask2 = cv2.inRange(hsv, cfg['hsv_lower2'], cfg['hsv_upper2'])
            mask = cv2.bitwise_or(mask, mask2)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  MORPH_KERNEL, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bricks = []
        h_depth, w_depth = depth.shape[:2]

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.min_contour_area:
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            x3d = y3d = z3d = None
            if (0 <= cy < h_depth and 0 <= cx < w_depth):
                depth_mm = int(depth[cy, cx])
                if depth_mm > 0 and self.camera_info is not None:
                    fx = self.camera_info.k[0]
                    fy = self.camera_info.k[4]
                    cx_intr = self.camera_info.k[2]
                    cy_intr = self.camera_info.k[5]
                    z3d = depth_mm / 1000.0
                    x3d = (cx - cx_intr) * z3d / fx
                    y3d = (cy - cy_intr) * z3d / fy

            bricks.append({
                'cx':      cx,
                'cy':      cy,
                'x3d':     x3d,
                'y3d':     y3d,
                'z3d':     z3d,
                'contour': cnt,
                'bbox':    cv2.boundingRect(cnt),
            })

        return bricks

    # ------------------------------------------------------------------
    def publish_detections(self, color: str, bricks: list, header):
        """Publish one PoseArray for the given color."""
        msg = PoseArray()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.camera_frame

        for brick in bricks:
            pose = Pose()
            if brick['x3d'] is not None:
                pose.position.x = brick['x3d']
                pose.position.y = brick['y3d']
                pose.position.z = brick['z3d']
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.detection_pubs[color].publish(msg)

    # ------------------------------------------------------------------
    def draw_detections(self, image: np.ndarray, color: str, bricks: list):
        """Draw semi-transparent fill, outline, bbox, centroid, and labels."""
        ann_color = ANNOTATION_COLORS.get(color, DEFAULT_ANNOTATION_COLOR)

        for brick in bricks:
            cnt = brick['contour']
            cx, cy = brick['cx'], brick['cy']
            bx, by, bw, bh = brick['bbox']

            # Semi-transparent contour fill
            overlay = image.copy()
            cv2.drawContours(overlay, [cnt], -1, ann_color, thickness=cv2.FILLED)
            cv2.addWeighted(overlay, 0.3, image, 0.7, 0, image)

            # Contour outline
            cv2.drawContours(image, [cnt], -1, ann_color, 2)

            # Bounding rect
            cv2.rectangle(image, (bx, by), (bx + bw, by + bh), ann_color, 1)

            # Center dot
            cv2.circle(image, (cx, cy), 4, ann_color, -1)

            # Color label
            cv2.putText(image, color, (bx, by - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, ann_color, 1, cv2.LINE_AA)

            # 3D coord text or "no depth"
            if brick['z3d'] is not None:
                coord_text = f'{brick["x3d"]:.2f},{brick["y3d"]:.2f},{brick["z3d"]:.2f}m'
            else:
                coord_text = 'no depth'
            cv2.putText(image, coord_text, (bx, by + bh + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, ann_color, 1, cv2.LINE_AA)


def main(args=None):
    rclpy.init(args=args)
    node = LegoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
