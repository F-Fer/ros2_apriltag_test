# AutonOHM AprilTag Detection System

A ROS 2 Jazzy-based AprilTag detection system designed to work with a built-in webcam.

```
   ___        __              ____  __  ____  ___
  /   | __  _/ /_____  ____  / __ \/ / / /  |/  /
 / /| |/ / / / __/ __ \/ __ \/ / / / /_/ / /|_/ / 
/ ___ / /_/ / /_/ /_/ / / / / /_/ / __  / /  / /  
/_/  |_\__,_/\__/\____/_/ /_/\____/_/ /_/_/  /_/   
                                                   
         AprilTag Detection System
```

## Architecture

Since Docker on macOS cannot directly access the webcam, this system uses a hybrid approach:

```
┌─────────────────────┐      ZMQ      ┌─────────────────────────────────────┐
│   Mac Host          │    (5555)     │   Docker Container (ROS 2 Jazzy)    │
│                     │               │                                     │
│ ┌─────────────────┐ │               │ ┌─────────────────┐                 │
│ │  webcam_sender  │ ├──────────────►│ │ webcam_receiver │                 │
│ │     .py         │ │   JPEG frames │ │     _node       │                 │
│ └─────────────────┘ │               │ └────────┬────────┘                 │
│         ▲           │               │          │ /camera/image_raw        │
│         │           │               │          ▼                          │
│    ┌────┴────┐      │               │ ┌─────────────────┐                 │
│    │ Webcam  │      │               │ │ apriltag_detector│                │
│    └─────────┘      │               │ │     _node        │                │
│                     │               │ └────────┬─────────┘                │
└─────────────────────┘               │          │                          │
                                      │          ▼                          │
                                      │ /apriltag/detections                │
                                      │ /apriltag/poses                     │
                                      │ /apriltag/image_annotated           │
                                      └─────────────────────────────────────┘
```

## Quick Start (5 minutes)

### Prerequisites

1. **Docker Desktop for Mac** - Install from https://docker.com
2. **Python 3** - Usually pre-installed on macOS

### Step 1: Install Mac-side dependencies

```bash
cd /Users/finnferchau/dev/apriltag_test
pip3 install -r requirements.txt
```

### Step 2: Build and start the ROS 2 container

```bash
# Build the Docker image (first time takes ~5 min)
docker compose build

# Start the ROS 2 nodes
docker compose up
```

### Step 3: In a NEW terminal, start the webcam sender

```bash
cd /Users/finnferchau/dev/apriltag_test
python3 webcam_sender.py --show
```

The `--show` flag opens a local preview window. Press 'q' to quit.

### Step 4: Print some AprilTags!

Print AprilTags from the **tag36h11** family. You can generate them at:
- https://github.com/AprilRobotics/apriltag-imgs
- https://chev.me/arucogen/ (select AprilTag, tag36h11)

Recommended tag IDs for testing: 0, 1, 2, 3, 4, 5

## ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Raw camera frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |
| `/apriltag/detections` | `vision_msgs/Detection2DArray` | 2D tag detections |
| `/apriltag/poses` | `geometry_msgs/PoseArray` | 3D tag poses |
| `/apriltag/image_annotated` | `sensor_msgs/Image` | Annotated visualization |

## Inspecting Results

### From another terminal, enter the container:

```bash
docker exec -it apriltag_ros2 bash
source /ros2_ws/install/setup.bash

# List topics
ros2 topic list

# See detections
ros2 topic echo /apriltag/detections

# See poses
ros2 topic echo /apriltag/poses
```

## Configuration

### Tag Family

The default tag family is `tag36h11`. To use a different family:

```bash
# In docker-compose.yml or launch command
ros2 launch apriltag_detector apriltag_demo.launch.py tag_family:=tag25h9
```

Supported families: `tag36h11`, `tag25h9`, `tag16h5`, `tagCircle21h7`, `tagStandard41h12`

### Tag Size

Set the physical tag size (in meters) for accurate pose estimation:

```bash
ros2 launch apriltag_detector apriltag_demo.launch.py tag_size:=0.10  # 10cm tags
```

### Webcam Settings

```bash
python3 webcam_sender.py --help

# Example: Higher resolution
python3 webcam_sender.py --width 1280 --height 720 --fps 15
```

## Running Individual Nodes

If you prefer to run nodes individually:

```bash
# Inside Docker container
source /ros2_ws/install/setup.bash

# Terminal 1: Webcam receiver
ros2 run apriltag_detector webcam_receiver_node

# Terminal 2: AprilTag detector  
ros2 run apriltag_detector apriltag_detector_node

# Terminal 3: Visualization (requires X11)
ros2 run apriltag_detector visualization_node
```

## Troubleshooting

### "Could not open camera" error

1. Grant camera permissions: **System Preferences → Security & Privacy → Camera**
2. Try a different camera index: `python3 webcam_sender.py --camera 1`

### No frames received in Docker

1. Check the webcam sender is running
2. Verify port is not blocked: `lsof -i :5555`
3. Try restarting Docker Desktop

### Docker build fails

```bash
# Clean rebuild
docker compose down
docker compose build --no-cache
docker compose up
```

### Poor detection performance

1. Ensure good lighting
2. Print tags at appropriate size (5-10cm works well)
3. Keep tags flat and perpendicular to camera
4. Reduce motion blur by holding tags steady

## Project Structure

```
apriltag_test/
├── docker-compose.yml      # Docker orchestration
├── Dockerfile              # ROS 2 Jazzy container
├── requirements.txt        # Mac-side Python deps
├── webcam_sender.py        # Mac webcam capture script
├── README.md               # This file
└── src/
    └── apriltag_detector/  # ROS 2 package
        ├── package.xml
        ├── setup.py
        ├── apriltag_detector/
        │   ├── __init__.py
        │   ├── apriltag_detector_node.py
        │   ├── webcam_receiver_node.py
        │   └── visualization_node.py
        ├── launch/
        │   └── apriltag_demo.launch.py
        └── config/
            └── apriltag_params.yaml
```

## Next Steps for Robot Integration

1. **Replace webcam receiver** with actual robot camera driver
2. **Connect pose output** to robot navigation/manipulation
3. **Add TF broadcasting** for tag poses in robot frame
4. **Calibrate camera** for accurate pose estimation

## Credits

- **Team:** AutonOHM, TH Nürnberg
- **Competition:** RoboCup German Open - @Work
- **AprilTag:** https://april.eecs.umich.edu/software/apriltag

---

*Good luck at the German Open! 🤖🏆*
