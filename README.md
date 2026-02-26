# AutonOHM AprilTag + Lego Duplo Detection

ROS 2 Jazzy-based detection system for RoboCup @Work (TH Nürnberg).
Solves the macOS Docker webcam/USB constraint using ZMQ message passing.

---

## Architecture

```
Mac Host                              Docker (ROS 2 Jazzy)
────────────────────                  ──────────────────────────────────────
webcam_sender.py   ──ZMQ:5555──▶  webcam_receiver_node
                                       └▶ apriltag_detector_node
                                           └▶ /apriltag/detections|poses
                                           └▶ web_viewer_node :8080

realsense_sender.py ──ZMQ:5556──▶  realsense_receiver_node
                                       └▶ lego_detector_node
                                           └▶ /lego/detections/{color}
                                           └▶ web_viewer_node :8080
```

---

## AprilTag Demo (webcam)

```bash
# Terminal 1 — Docker
docker compose run --rm --service-ports apriltag_detector bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch apriltag_detector apriltag_demo.launch.py

# Terminal 2 — Mac host
python3 webcam_sender.py --show
```

Browser: http://localhost:8080

---

## Lego Duplo Demo (RealSense D435)

**Mac host — one-time pyrealsense2 setup:**
```bash
# Build from source (no ARM64 wheel available)
brew install cmake libusb pkg-config
git clone https://github.com/IntelRealSense/librealsense.git --depth=1
cd librealsense && mkdir build && cd build
cmake .. -DBUILD_PYTHON_BINDINGS=ON \
         -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)") \
         -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF \
         -DBUILD_GRAPHICAL_EXAMPLES=OFF -DFORCE_RSUSB_BACKEND=ON
make -j$(sysctl -n hw.logicalcpu) && sudo make install

# Copy bindings into venv and fix rpath
cp -r /opt/homebrew/lib/python3.14/site-packages/pyrealsense2 \
      .venv/lib/python3.11/site-packages/
install_name_tool -add_rpath /usr/local/lib \
  .venv/lib/python3.11/site-packages/pyrealsense2/pyrealsense2.cpython-311-darwin.so
install_name_tool -add_rpath /usr/local/lib \
  .venv/lib/python3.11/site-packages/pyrealsense2/pyrsutils.cpython-311-darwin.so
```

**Run:**
```bash
# Terminal 1 — Docker (needs service ports for ZMQ + HTTP)
docker compose run --rm --service-ports apriltag_detector bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch apriltag_detector lego_demo.launch.py

# Terminal 2 — Mac host (needs sudo for USB power control on macOS)
sudo .venv/bin/python realsense_sender.py --show
```

Browser: http://localhost:8080

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | RealSense color |
| `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Aligned depth (16UC1, mm) |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RealSense intrinsics |
| `/lego/detections/{color}` | `geometry_msgs/PoseArray` | 3D brick positions per color |
| `/lego/image_annotated` | `sensor_msgs/Image` | Annotated color feed |
| `/apriltag/detections` | `vision_msgs/Detection2DArray` | 2D tag detections |
| `/apriltag/poses` | `geometry_msgs/PoseArray` | 3D tag poses |

---

## Tuning HSV Colors

Edit `src/apriltag_detector/config/apriltag_params.yaml` under `lego_detector.ros__parameters`.
H: 0–180, S/V: 0–255 (OpenCV convention). No rebuild needed — restart the launch.

