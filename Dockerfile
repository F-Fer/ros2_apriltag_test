# ROS 2 Jazzy AprilTag Detection for AutonOHM
FROM ros:jazzy

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-msgs \
    libopencv-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --break-system-packages \
    pupil-apriltags \
    numpy \
    pyzmq \
    Pillow

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p src

# Copy package
COPY src/apriltag_detector src/apriltag_detector

# Build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install"

# Source workspace on startup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

EXPOSE 5555

CMD ["bash"]
