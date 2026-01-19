#!/usr/bin/env python3
"""
AprilTag Detector Node for AutonOHM atwork robot
RoboCup German Open

Subscribes to camera images, detects AprilTags, and publishes:
- Tag detections with pose estimates
- Annotated images for visualization
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from pupil_apriltags import Detector


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector_node')
        
        # Parameters
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_size', 0.05)  # 5cm tags
        self.declare_parameter('camera_frame', 'camera_link')
        
        tag_family = self.get_parameter('tag_family').value
        self.tag_size = self.get_parameter('tag_size').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
        # AprilTag detector (pupil-apriltags)
        self.detector = Detector(
            families=tag_family,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # Default camera parameters for MacBook webcam (approximate)
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/apriltag/detections',
            10
        )
        
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/apriltag/poses',
            10
        )
        
        self.annotated_image_pub = self.create_publisher(
            Image,
            '/apriltag/image_annotated',
            10
        )
        
        self.get_logger().info('AprilTag Detector initialized')
        self.get_logger().info(f'  Tag family: {tag_family}')
        self.get_logger().info(f'  Tag size: {self.tag_size}m')
        self.get_logger().info('  Waiting for images on /camera/image_raw...')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from camera_info topic."""
        if msg.k[0] > 0:  # fx
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            
            self.camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ], dtype=np.float32)
            
            if len(msg.d) >= 5:
                self.dist_coeffs = np.array(msg.d[:5], dtype=np.float32)
    
    def image_callback(self, msg: Image):
        """Process incoming images and detect AprilTags."""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect AprilTags with pose estimation
            detections = self.detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=(self.fx, self.fy, self.cx, self.cy),
                tag_size=self.tag_size
            )
            
            # Prepare messages
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            pose_array = PoseArray()
            pose_array.header = msg.header
            pose_array.header.frame_id = self.camera_frame
            
            # Process each detection
            for detection in detections:
                # Draw detection on image
                self.draw_detection(cv_image, detection)
                
                # Create Detection2D message
                det_msg = Detection2D()
                det_msg.header = msg.header
                
                # Bounding box from corners
                corners = detection.corners
                x_coords = [c[0] for c in corners]
                y_coords = [c[1] for c in corners]
                
                det_msg.bbox.center.position.x = float(detection.center[0])
                det_msg.bbox.center.position.y = float(detection.center[1])
                det_msg.bbox.size_x = float(max(x_coords) - min(x_coords))
                det_msg.bbox.size_y = float(max(y_coords) - min(y_coords))
                
                # Add hypothesis with tag ID
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(detection.tag_id)
                hypothesis.hypothesis.score = float(detection.decision_margin) / 100.0
                
                # Get pose (from detector if available, otherwise estimate manually)
                if detection.pose_t is not None:
                    pose = self.pose_from_detection(detection)
                else:
                    pose = self.estimate_pose(detection)
                
                if pose is not None:
                    hypothesis.pose.pose = pose
                    pose_array.poses.append(pose)
                
                det_msg.results.append(hypothesis)
                detection_array.detections.append(det_msg)
                
                # Log detection
                self.get_logger().info(
                    f'Detected tag {detection.tag_id} at '
                    f'({detection.center[0]:.1f}, {detection.center[1]:.1f}) '
                    f'margin: {detection.decision_margin:.1f}'
                )
            
            # Publish results
            self.detection_pub.publish(detection_array)
            self.pose_pub.publish(pose_array)
            
            # Add status text
            status = f'AutonOHM AprilTag Detector | Tags: {len(detections)}'
            cv2.putText(cv_image, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def draw_detection(self, image, detection):
        """Draw AprilTag detection on image."""
        corners = detection.corners.astype(int)
        
        # Draw outline
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(image, pt1, pt2, (0, 255, 0), 2)
        
        # Draw center
        center = tuple(detection.center.astype(int))
        cv2.circle(image, center, 5, (0, 0, 255), -1)
        
        # Draw tag ID
        cv2.putText(image, f'ID: {detection.tag_id}',
                   (center[0] - 30, center[1] - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Draw axes if pose available
        # corners_float = detection.corners.reshape(4, 2)
        
        # Draw corner numbers for debugging
        for i, corner in enumerate(corners):
            cv2.putText(image, str(i), tuple(corner),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    def pose_from_detection(self, detection):
        """Create Pose message from pupil-apriltags detection with pose."""
        pose = Pose()
        
        # Translation
        pose.position.x = float(detection.pose_t[0][0])
        pose.position.y = float(detection.pose_t[1][0])
        pose.position.z = float(detection.pose_t[2][0])
        
        # Rotation (convert rotation matrix to quaternion)
        quat = self.rotation_matrix_to_quaternion(detection.pose_R)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def estimate_pose(self, detection):
        """Estimate 3D pose of AprilTag (fallback method)."""
        # 3D coordinates of tag corners (in tag frame)
        half_size = self.tag_size / 2.0
        object_points = np.array([
            [-half_size, -half_size, 0],
            [ half_size, -half_size, 0],
            [ half_size,  half_size, 0],
            [-half_size,  half_size, 0],
        ], dtype=np.float32)
        
        # 2D image coordinates
        image_points = detection.corners.astype(np.float32)
        
        # Camera matrix
        camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        dist_coeffs = np.zeros(5, dtype=np.float32)
        
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            return None
        
        # Convert to pose message
        pose = Pose()
        
        # Translation
        pose.position.x = float(tvec[0][0])
        pose.position.y = float(tvec[1][0])
        pose.position.z = float(tvec[2][0])
        
        # Rotation (convert rodrigues to quaternion)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        
        return pose
    
    def rotation_matrix_to_quaternion(self, R):
        """Convert rotation matrix to quaternion."""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        return [x, y, z, w]


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
