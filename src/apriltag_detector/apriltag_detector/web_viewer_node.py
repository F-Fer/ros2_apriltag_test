#!/usr/bin/env python3
"""
Web Viewer Node for AutonOHM AprilTag Detector
Streams annotated images to a web browser via HTTP.

Access at: http://localhost:8080
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
import time


class MJPEGHandler(BaseHTTPRequestHandler):
    """HTTP handler for MJPEG streaming."""
    
    latest_frame = None
    frame_lock = threading.Lock()
    
    def log_message(self, format, *args):
        """Suppress default logging."""
        pass
    
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            html = '''<!DOCTYPE html>
<html>
<head>
    <title>AprilTag Detector</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 50%, #0f3460 100%);
            min-height: 100vh;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 20px;
            color: #e8e8e8;
        }
        h1 {
            color: #2ecc71;
            margin-bottom: 10px;
            font-size: 2.5em;
            text-shadow: 0 0 20px rgba(46, 204, 113, 0.3);
        }
        .subtitle {
            color: #7f8c8d;
            margin-bottom: 30px;
            font-size: 1.1em;
        }
        .video-container {
            background: #000;
            border-radius: 12px;
            overflow: hidden;
            box-shadow: 0 20px 60px rgba(0,0,0,0.5), 0 0 40px rgba(46, 204, 113, 0.1);
            border: 2px solid #2ecc71;
        }
        img {
            display: block;
            max-width: 100%;
            height: auto;
        }
        .status {
            margin-top: 20px;
            padding: 10px 20px;
            background: rgba(46, 204, 113, 0.1);
            border: 1px solid #2ecc71;
            border-radius: 20px;
            font-size: 0.9em;
        }
        .status::before {
            content: '...';
            color: #2ecc71;
            margin-right: 8px;
            animation: pulse 1s infinite;
        }
        @keyframes pulse {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }
    </style>
</head>
<body>
    <h1>AprilTag Detector</h1>
    <div class="video-container">
        <img src="/stream" alt="AprilTag Detection Stream">
    </div>
    <div class="status">Live Stream Active</div>
</body>
</html>'''
            self.wfile.write(html.encode())
            
        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            
            while True:
                with MJPEGHandler.frame_lock:
                    frame = MJPEGHandler.latest_frame
                
                if frame is not None:
                    try:
                        self.wfile.write(b'--frame\r\n')
                        self.wfile.write(b'Content-Type: image/jpeg\r\n\r\n')
                        self.wfile.write(frame)
                        self.wfile.write(b'\r\n')
                    except (BrokenPipeError, ConnectionResetError):
                        break
                
                time.sleep(0.033)  # ~30fps
        else:
            self.send_error(404)


class WebViewerNode(Node):
    def __init__(self):
        super().__init__('web_viewer_node')
        
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value
        
        self.bridge = CvBridge()
        
        # Subscribe to annotated images
        self.subscription = self.create_subscription(
            Image,
            '/apriltag/image_annotated',
            self.image_callback,
            10
        )
        
        # Start HTTP server in background thread
        self.server = HTTPServer(('0.0.0.0', port), MJPEGHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        
        self.get_logger().info(f'Web viewer started at http://localhost:{port}')
        self.get_logger().info('Open this URL in your browser to see the detection stream!')
    
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            with MJPEGHandler.frame_lock:
                MJPEGHandler.latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Error encoding image: {e}')
    
    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebViewerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
