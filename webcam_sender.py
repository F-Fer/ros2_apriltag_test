#!/usr/bin/env python3
"""
Webcam Sender for Mac Host
Captures frames from MacBook webcam and sends them via ZMQ to Docker container.

Run this script NATIVELY on your Mac (not in Docker).

Usage:
    python3 webcam_sender.py

Requirements:
    pip3 install opencv-python pyzmq
"""

import cv2
import zmq
import argparse
import time


def main():
    parser = argparse.ArgumentParser(description='Send webcam frames to ROS 2 Docker container')
    parser.add_argument('--port', type=int, default=5555, help='ZMQ port (default: 5555)')
    parser.add_argument('--camera', type=int, default=0, help='Camera index (default: 0)')
    parser.add_argument('--width', type=int, default=640, help='Frame width (default: 640)')
    parser.add_argument('--height', type=int, default=480, help='Frame height (default: 480)')
    parser.add_argument('--fps', type=int, default=30, help='Target FPS (default: 30)')
    parser.add_argument('--quality', type=int, default=80, help='JPEG quality 0-100 (default: 80)')
    parser.add_argument('--show', action='store_true', help='Show local preview window')
    args = parser.parse_args()
    
    # ZMQ setup
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)
    socket.connect(f"tcp://localhost:{args.port}")
    socket.setsockopt(zmq.SNDHWM, 2)  # Only buffer 2 frames
    
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║        AutonOHM AprilTag Detector - Webcam Sender            ║")
    print("║                  RoboCup German Open                         ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print(f"║  Connecting to: tcp://localhost:{args.port:<25}    ║")
    print(f"║  Camera: {args.camera}  Resolution: {args.width}x{args.height}  FPS: {args.fps:<14}         ║")
    print("║                                                              ║")
    print("║  Press 'q' to quit (if preview enabled)                      ║")
    print("║  Press Ctrl+C to stop                                        ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    
    # Open camera
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    
    if not cap.isOpened():
        print("Error: Could not open camera!")
        print("Make sure you've granted camera permissions in System Preferences.")
        return
    
    print("\n✓ Camera opened successfully!")
    print(f"  Actual resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    print("\nSending frames... (Ctrl+C to stop)\n")
    
    frame_count = 0
    start_time = time.time()
    fps_update_time = start_time
    
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Warning: Failed to grab frame")
                continue
            
            # Encode frame as JPEG
            _, encoded = cv2.imencode('.jpg', frame, encode_params)
            
            # Send frame via ZMQ
            try:
                socket.send(encoded.tobytes(), zmq.NOBLOCK)
                frame_count += 1
            except zmq.Again:
                # Socket buffer full, skip frame
                pass
            
            # Show preview if requested
            if args.show:
                cv2.putText(frame, "Sending to ROS 2...", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Webcam Sender Preview", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            # Print FPS every second
            current_time = time.time()
            if current_time - fps_update_time >= 1.0:
                elapsed = current_time - start_time
                fps = frame_count / elapsed
                print(f"\rFrames sent: {frame_count} | FPS: {fps:.1f}", end='', flush=True)
                fps_update_time = current_time
            
            # Rate limiting
            time.sleep(1.0 / args.fps)
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        cap.release()
        socket.close()
        context.term()
        if args.show:
            cv2.destroyAllWindows()
        print("Done!")


if __name__ == '__main__':
    main()
