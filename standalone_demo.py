#!/usr/bin/env python3
"""
Standalone AprilTag Detection Demo - NO DOCKER REQUIRED

Usage:
    uv sync
    uv run standalone_demo.py
"""

import cv2
from pupil_apriltags import Detector
import numpy as np
import argparse
import time
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class TagDetection:
    """Detected AprilTag information."""
    tag_id: int
    center: Tuple[float, float]
    corners: np.ndarray
    decision_margin: float
    pose_t: Optional[np.ndarray] = None  # Translation
    pose_R: Optional[np.ndarray] = None  # Rotation matrix


class AprilTagDemo:
    """AprilTag detection demo for AutonOHM."""
    
    TEAM_BANNER = """
    ╔═══════════════════════════════════════════════════════════════╗
    ║     ___        __              ____  __  ____  ___            ║
    ║    /   | __  _/ /_____  ____  / __ \\/ / / /  |/  /           ║
    ║   / /| |/ / / / __/ __ \\/ __ \\/ / / / /_/ / /|_/ /          ║
    ║  / ___ / /_/ / /_/ /_/ / / / / /_/ / __  / /  / /             ║
    ║ /_/  |_\\__,_/\\__/\\____/_/ /_/\\____/_/ /_/_/  /_/          ║
    ║                                                               ║
    ║            AprilTag Detection System                          ║
    ╚═══════════════════════════════════════════════════════════════╝
    """
    
    # Color scheme
    COLORS = {
        'primary': (46, 204, 113),      # Green
        'secondary': (52, 152, 219),    # Blue
        'accent': (241, 196, 15),       # Yellow
        'danger': (231, 76, 60),        # Red
        'text': (236, 240, 241),        # Light gray
        'dark': (44, 62, 80),           # Dark blue-gray
    }
    
    def __init__(self, tag_family: str = 'tag36h11', tag_size: float = 0.05,
                 camera_index: int = 0, width: int = 1280, height: int = 720):
        """Initialize the AprilTag detector."""
        self.tag_size = tag_size
        self.camera_index = camera_index
        self.width = width
        self.height = height
        
        # Initialize detector (pupil-apriltags API)
        self.detector = Detector(
            families=tag_family,
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=True,
            decode_sharpening=0.25,
        )
        
        # Camera intrinsics (approximate for MacBook)
        self.fx = width * 0.8
        self.fy = width * 0.8
        self.cx = width / 2.0
        self.cy = height / 2.0
        
        # Stats
        self.frame_count = 0
        self.detection_count = 0
        self.start_time = time.time()
        self.fps = 0.0
    
    def detect(self, frame: np.ndarray) -> List[TagDetection]:
        """Detect AprilTags in frame."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # pupil-apriltags can estimate pose directly if we pass camera params
        raw_detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.fx, self.fy, self.cx, self.cy),
            tag_size=self.tag_size
        )
        
        detections = []
        for det in raw_detections:
            tag = TagDetection(
                tag_id=det.tag_id,
                center=tuple(det.center),
                corners=det.corners,
                decision_margin=det.decision_margin,
            )
            
            # Use pose from detector if available, otherwise estimate manually
            if det.pose_t is not None:
                tag.pose_t = det.pose_t
                tag.pose_R = det.pose_R
            else:
                pose_t, pose_R = self.estimate_pose(det)
                tag.pose_t = pose_t
                tag.pose_R = pose_R
            
            detections.append(tag)
        
        return detections
    
    def estimate_pose(self, detection) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Estimate 3D pose of tag."""
        half_size = self.tag_size / 2.0
        object_points = np.array([
            [-half_size, -half_size, 0],
            [ half_size, -half_size, 0],
            [ half_size,  half_size, 0],
            [-half_size,  half_size, 0],
        ], dtype=np.float32)
        
        image_points = detection.corners.astype(np.float32)
        
        camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        dist_coeffs = np.zeros(5, dtype=np.float32)
        
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        
        if not success:
            return None, None
        
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        return tvec, rotation_matrix
    
    def draw_detection(self, frame: np.ndarray, tag: TagDetection):
        """Draw a single tag detection with pose visualization."""
        corners = tag.corners.astype(int)
        center = tuple(int(x) for x in tag.center)
        
        # Draw filled polygon for tag area (semi-transparent)
        overlay = frame.copy()
        cv2.fillPoly(overlay, [corners], self.COLORS['primary'])
        cv2.addWeighted(overlay, 0.2, frame, 0.8, 0, frame)
        
        # Draw corner outline
        for i in range(4):
            pt1 = tuple(corners[i])
            pt2 = tuple(corners[(i + 1) % 4])
            cv2.line(frame, pt1, pt2, self.COLORS['primary'], 3)
        
        # Draw corner dots
        for i, corner in enumerate(corners):
            color = self.COLORS['accent'] if i == 0 else self.COLORS['secondary']
            cv2.circle(frame, tuple(corner), 8, color, -1)
            cv2.circle(frame, tuple(corner), 8, self.COLORS['dark'], 2)
        
        # Draw center
        cv2.circle(frame, center, 6, self.COLORS['danger'], -1)
        
        # Draw 3D axes if pose is available
        if tag.pose_t is not None:
            self.draw_axes(frame, tag)
        
        # Draw info box
        self.draw_info_box(frame, tag)
    
    def draw_axes(self, frame: np.ndarray, tag: TagDetection):
        """Draw 3D coordinate axes on tag."""
        axis_length = self.tag_size * 0.8
        
        # Axis points in tag frame
        axis_points = np.array([
            [0, 0, 0],
            [axis_length, 0, 0],  # X - red
            [0, axis_length, 0],  # Y - green
            [0, 0, -axis_length], # Z - blue (pointing out of tag)
        ], dtype=np.float32)
        
        camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        dist_coeffs = np.zeros(5, dtype=np.float32)
        rvec, _ = cv2.Rodrigues(tag.pose_R)
        
        image_points, _ = cv2.projectPoints(
            axis_points, rvec, tag.pose_t, camera_matrix, dist_coeffs
        )
        image_points = image_points.reshape(-1, 2).astype(int)
        
        origin = tuple(image_points[0])
        
        # Draw axes
        cv2.arrowedLine(frame, origin, tuple(image_points[1]), (0, 0, 255), 3, tipLength=0.2)  # X
        cv2.arrowedLine(frame, origin, tuple(image_points[2]), (0, 255, 0), 3, tipLength=0.2)  # Y
        cv2.arrowedLine(frame, origin, tuple(image_points[3]), (255, 0, 0), 3, tipLength=0.2)  # Z
    
    def draw_info_box(self, frame: np.ndarray, tag: TagDetection):
        """Draw info box for detected tag."""
        center = tuple(int(x) for x in tag.center)
        
        # Calculate distance
        distance = None
        if tag.pose_t is not None:
            distance = np.linalg.norm(tag.pose_t)
        
        # Info text
        lines = [f"ID: {tag.tag_id}"]
        if distance is not None:
            lines.append(f"Dist: {distance:.2f}m")
        
        # Calculate box size
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        thickness = 2
        padding = 8
        
        max_width = 0
        total_height = 0
        line_heights = []
        
        for line in lines:
            (w, h), _ = cv2.getTextSize(line, font, font_scale, thickness)
            max_width = max(max_width, w)
            line_heights.append(h)
            total_height += h + 5
        
        # Position box above center
        box_x = center[0] - max_width // 2 - padding
        box_y = center[1] - int(tag.corners[:, 1].max() - tag.corners[:, 1].min()) // 2 - total_height - 20
        box_y = max(10, box_y)
        
        # Draw box background
        cv2.rectangle(frame,
                     (box_x, box_y),
                     (box_x + max_width + 2 * padding, box_y + total_height + padding),
                     self.COLORS['dark'], -1)
        cv2.rectangle(frame,
                     (box_x, box_y),
                     (box_x + max_width + 2 * padding, box_y + total_height + padding),
                     self.COLORS['primary'], 2)
        
        # Draw text
        y_offset = box_y + line_heights[0] + padding // 2
        for i, line in enumerate(lines):
            cv2.putText(frame, line, (box_x + padding, y_offset),
                       font, font_scale, self.COLORS['text'], thickness)
            if i < len(line_heights) - 1:
                y_offset += line_heights[i + 1] + 5
    
    def draw_hud(self, frame: np.ndarray, detections: List[TagDetection]):
        """Draw heads-up display overlay."""
        h, w = frame.shape[:2]
        
        # Semi-transparent header bar
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, 60), self.COLORS['dark'], -1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
        
        # Title
        cv2.putText(frame, "AutonOHM AprilTag Detector", (15, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.0, self.COLORS['primary'], 2)
        
        # Stats on right
        stats_text = f"Tags: {len(detections)}  |  FPS: {self.fps:.1f}"
        (tw, _), _ = cv2.getTextSize(stats_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
        cv2.putText(frame, stats_text, (w - tw - 15, 40),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, self.COLORS['text'], 2)
        
        # Footer bar
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, h - 35), (w, h), self.COLORS['dark'], -1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
        
        footer_text = "Press 'q' to quit | 's' to save screenshot | RoboCup German Open"
        cv2.putText(frame, footer_text, (15, h - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS['text'], 1)
        
        # Draw detection list on left side if tags detected
        if detections:
            self.draw_detection_list(frame, detections)
    
    def draw_detection_list(self, frame: np.ndarray, detections: List[TagDetection]):
        """Draw list of detected tags on left side."""
        h, w = frame.shape[:2]
        
        panel_width = 200
        panel_height = min(70 + len(detections) * 35, h - 120)
        
        # Panel background
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 70), (10 + panel_width, 70 + panel_height),
                     self.COLORS['dark'], -1)
        cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
        cv2.rectangle(frame, (10, 70), (10 + panel_width, 70 + panel_height),
                     self.COLORS['primary'], 2)
        
        # Panel header
        cv2.putText(frame, "Detected Tags", (20, 95),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, self.COLORS['primary'], 2)
        
        # List tags
        y_offset = 125
        for i, tag in enumerate(detections[:8]):  # Max 8 tags displayed
            dist_text = ""
            if tag.pose_t is not None:
                dist = np.linalg.norm(tag.pose_t)
                dist_text = f" ({dist:.2f}m)"
            
            text = f"• Tag {tag.tag_id}{dist_text}"
            cv2.putText(frame, text, (20, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.COLORS['text'], 1)
            y_offset += 30
    
    def run(self):
        """Run the demo."""
        print(self.TEAM_BANNER)
        print("Starting camera...")
        
        cap = cv2.VideoCapture(self.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        if not cap.isOpened():
            print("ERROR: Could not open camera!")
            print("Make sure camera permissions are granted in System Preferences.")
            return
        
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Camera opened: {actual_width}x{actual_height}")
        print("\nControls:")
        print("  'q' - Quit")
        print("  's' - Save screenshot")
        print("  'f' - Toggle fullscreen")
        print("\nLooking for AprilTags (tag36h11 family)...")
        
        # Update camera parameters
        self.cx = actual_width / 2.0
        self.cy = actual_height / 2.0
        self.fx = actual_width * 0.8
        self.fy = actual_width * 0.8
        
        cv2.namedWindow("AutonOHM AprilTag Demo", cv2.WINDOW_NORMAL)
        
        fps_time = time.time()
        fps_counter = 0
        
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Warning: Failed to grab frame")
                    continue
                
                # Detect tags
                detections = self.detect(frame)
                
                # Draw detections
                for tag in detections:
                    self.draw_detection(frame, tag)
                
                # Draw HUD
                self.draw_hud(frame, detections)
                
                # Update FPS
                fps_counter += 1
                if time.time() - fps_time >= 1.0:
                    self.fps = fps_counter / (time.time() - fps_time)
                    fps_counter = 0
                    fps_time = time.time()
                
                self.frame_count += 1
                self.detection_count += len(detections)
                
                # Show frame
                cv2.imshow("AutonOHM AprilTag Demo", frame)
                
                # Handle input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f"apriltag_screenshot_{int(time.time())}.png"
                    cv2.imwrite(filename, frame)
                    print(f"Screenshot saved: {filename}")
                elif key == ord('f'):
                    current = cv2.getWindowProperty("AutonOHM AprilTag Demo", cv2.WND_PROP_FULLSCREEN)
                    cv2.setWindowProperty("AutonOHM AprilTag Demo", cv2.WND_PROP_FULLSCREEN,
                                         cv2.WINDOW_FULLSCREEN if current != cv2.WINDOW_FULLSCREEN else cv2.WINDOW_NORMAL)
                    
        except KeyboardInterrupt:
            pass
        finally:
            cap.release()
            cv2.destroyAllWindows()
            
            # Print stats
            elapsed = time.time() - self.start_time
            print(f"\n{'='*50}")
            print("Session Statistics:")
            print(f"  Runtime: {elapsed:.1f}s")
            print(f"  Frames processed: {self.frame_count}")
            print(f"  Total detections: {self.detection_count}")
            print(f"  Average FPS: {self.frame_count / elapsed:.1f}")
            print(f"{'='*50}")


def main():
    parser = argparse.ArgumentParser(
        description='AutonOHM AprilTag Detection Demo',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 standalone_demo.py
  python3 standalone_demo.py --camera 1 --width 1920 --height 1080
  python3 standalone_demo.py --tag-family tag25h9 --tag-size 0.10
        """
    )
    parser.add_argument('--camera', type=int, default=0,
                       help='Camera index (default: 0)')
    parser.add_argument('--width', type=int, default=1280,
                       help='Frame width (default: 1280)')
    parser.add_argument('--height', type=int, default=720,
                       help='Frame height (default: 720)')
    parser.add_argument('--tag-family', type=str, default='tag36h11',
                       help='AprilTag family (default: tag36h11)')
    parser.add_argument('--tag-size', type=float, default=0.05,
                       help='Tag size in meters (default: 0.05)')
    
    args = parser.parse_args()
    
    demo = AprilTagDemo(
        tag_family=args.tag_family,
        tag_size=args.tag_size,
        camera_index=args.camera,
        width=args.width,
        height=args.height,
    )
    
    demo.run()


if __name__ == '__main__':
    main()
