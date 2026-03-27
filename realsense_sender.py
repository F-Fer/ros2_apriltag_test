#!/usr/bin/env python3
"""
RealSense Sender for Mac Host
Captures color + aligned depth from a D435 and forwards them via ZMQ
to the realsense_receiver_node running inside Docker.

Run NATIVELY on your Mac (not in Docker).

Requirements:
    pip3 install pyrealsense2 pyzmq numpy opencv-python

Usage:
    python3 realsense_sender.py [--port 5556] [--width 640] [--height 480] [--fps 30] [--show]
"""

import argparse
import json
import time

import cv2
import numpy as np
import zmq

try:
    import pyrealsense2 as rs
except ImportError:
    print("ERROR: pyrealsense2 not installed.")
    print("Install with:  pip3 install pyrealsense2")
    raise SystemExit(1)


def build_metadata(color_profile, depth_intr, depth_scale):
    """Build JSON metadata dict from RealSense intrinsics."""
    ci = color_profile.as_video_stream_profile().get_intrinsics()
    return {
        'width':       ci.width,
        'height':      ci.height,
        'fx':          ci.fx,
        'fy':          ci.fy,
        'cx':          ci.ppx,
        'cy':          ci.ppy,
        'depth_width':  depth_intr.width,
        'depth_height': depth_intr.height,
        'depth_scale':  depth_scale,      # metres per unit (typically 0.001)
    }


def main():
    parser = argparse.ArgumentParser(description='Send RealSense D435 frames to ROS 2 Docker')
    parser.add_argument('--port',    type=int, default=5556,  help='ZMQ port (default: 5556)')
    parser.add_argument('--width',   type=int, default=640,   help='Color/depth width  (default: 640)')
    parser.add_argument('--height',  type=int, default=480,   help='Color/depth height (default: 480)')
    parser.add_argument('--fps',     type=int, default=30,    help='Frame rate (default: 30)')
    parser.add_argument('--quality', type=int, default=85,    help='JPEG quality for color 0-100 (default: 85)')
    parser.add_argument('--show',    action='store_true',     help='Show local preview window')
    args = parser.parse_args()

    # --- ZMQ ---
    ctx    = zmq.Context()
    socket = ctx.socket(zmq.PUSH)
    socket.connect(f"tcp://localhost:{args.port}")
    socket.setsockopt(zmq.SNDHWM, 2)   # drop old frames when slow consumer

    print("╔══════════════════════════════════════════════════════════════╗")
    print("║      AutonOHM Lego Detector - RealSense D435 Sender          ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print(f"║  ZMQ → tcp://localhost:{args.port}                               ║")
    print(f"║  Resolution: {args.width}x{args.height} @ {args.fps} fps                          ║")
    print("║  Press Ctrl+C to stop                                        ║")
    print("╚══════════════════════════════════════════════════════════════╝")

    # --- RealSense pipeline ---
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8,  args.fps)
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16,   args.fps)

    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"\nERROR: Could not open RealSense device: {e}")
        print("Make sure the D435 is plugged in and not claimed by another process.")
        socket.close()
        ctx.term()
        raise SystemExit(1)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale  = depth_sensor.get_depth_scale()   # metres per unit

    # Align depth to color
    align = rs.align(rs.stream.color)

    # Build metadata once — intrinsics don't change
    color_profile = profile.get_stream(rs.stream.color)
    aligned_depth_profile = profile.get_stream(rs.stream.depth)
    depth_intr = aligned_depth_profile.as_video_stream_profile().get_intrinsics()
    meta_bytes = json.dumps(build_metadata(color_profile, depth_intr, depth_scale)).encode()

    encode_params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]

    print(f"\nCamera opened. depth_scale={depth_scale:.6f} m/unit")
    print("Sending frames... (Ctrl+C to stop)\n")

    frame_count  = 0
    start_time   = time.time()
    fps_deadline = start_time

    try:
        while True:
            frames  = pipeline.wait_for_frames(timeout_ms=5000)
            aligned = align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # Color: BGR numpy array → JPEG bytes
            color_np = np.asanyarray(color_frame.get_data())   # bgr8
            _, color_jpg = cv2.imencode('.jpg', color_np, encode_params)

            # Depth: uint16 numpy array → raw bytes (lossless — JPEG would corrupt values)
            depth_np = np.asanyarray(depth_frame.get_data())   # uint16, units = depth_scale metres
            depth_bytes = depth_np.tobytes()

            # Send as 3-part multipart message
            try:
                socket.send_multipart(
                    [meta_bytes, color_jpg.tobytes(), depth_bytes],
                    flags=zmq.NOBLOCK
                )
                frame_count += 1
            except zmq.Again:
                pass   # receiver lagging — drop frame

            if args.show:
                cv2.putText(color_np, f"Sending -> Docker:{args.port}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                # Depth visualisation: normalise to 8-bit for display
                depth_vis = cv2.convertScaleAbs(depth_np, alpha=0.03)
                depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
                cv2.imshow("Color", color_np)
                cv2.imshow("Depth", depth_vis)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            now = time.time()
            if now - fps_deadline >= 1.0:
                elapsed = now - start_time
                print(f"\rFrames sent: {frame_count} | FPS: {frame_count/elapsed:.1f}", end='', flush=True)
                fps_deadline = now

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        pipeline.stop()
        socket.close()
        ctx.term()
        if args.show:
            cv2.destroyAllWindows()
        print("Done!")


if __name__ == '__main__':
    main()
