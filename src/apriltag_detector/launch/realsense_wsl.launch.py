#!/usr/bin/env python3
"""
Launch file for AutonOHM AprilTag Detection natively via RealSense
Uses RealSense ROS driver instead of ZMQ webcams.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare arguments
    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='AprilTag family (tag36h11, tag25h9, tag16h5, etc.)'
    )
    
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.032',
        description='AprilTag size in meters (e.g. 0.032 for 32mm tags)'
    )
    
    # Try to load offsets from demo_config.yaml if available
    # Use the SOURCE path to avoid colcon install caching issues
    try:
        import yaml
        config_path = "/home/ros2/ros2_ws/src/uarm_apriltag_demo/config/demo_config.yaml"
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f)
            # Find the parameters block (usually under global /** or specific node)
            params = cfg.get('/**', {}).get('ros__parameters', {})
    except Exception as e:
        print(f"Error loading config from {config_path}: {e}")
        params = {}

    def get_p(key, default):
        return str(params.get(key, default))

    # RealSense Camera Node
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false', 
            'rgb_camera.profile': '640x480x30'
        }.items()
    )

    # Static TF Publisher from End Effector (Link8) to Tool Center Point (tcp_link)
    tcp_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tcp_static_tf',
        arguments=[
            get_p('tcp_x_offset', '0.05655'),
            get_p('tcp_y_offset', '0.0'),
            get_p('tcp_z_offset', '-0.03'),
            get_p('tcp_roll_offset', '0.0'),
            get_p('tcp_pitch_offset', '0.0'),
            get_p('tcp_yaw_offset', '0.0'),
            'Link8', 'tcp_link'
        ]
    )
    
    # Static TF Publisher from Tool Center Point (tcp_link) to Camera Link
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        arguments=[
            get_p('camera_x_offset', '0.025'),
            get_p('camera_y_offset', '0.010'),
            get_p('camera_z_offset', '0.010'),
            get_p('camera_roll_offset', '0.0'),
            get_p('camera_pitch_offset', '1.5708'),
            get_p('camera_yaw_offset', '0.0'),
            'tcp_link', 'camera_link'
        ]
    )
    
    # AprilTag detector node
    apriltag_detector = Node(
        package='apriltag_detector',
        executable='apriltag_detector_node',
        name='apriltag_detector',
        output='screen',
        parameters=[{
            'tag_family': LaunchConfiguration('tag_family'),
            'tag_size': LaunchConfiguration('tag_size'),
            'camera_frame': 'camera_color_optical_frame',  # Images come from this optical frame
        }],
        remappings=[
            ('/camera/image_raw', '/camera/camera/color/image_raw'),
            ('/camera/camera_info', '/camera/camera/color/camera_info')
        ]
    )
    
    # Web viewer node for browser-based visualization
    web_viewer = Node(
        package='apriltag_detector',
        executable='web_viewer_node',
        name='web_viewer',
        output='screen',
        parameters=[{
            'port': 8080,
            'image_topic': '/apriltag/image_annotated'
        }]
    )
    
    return LaunchDescription([
        tag_family_arg,
        tag_size_arg,
        realsense_node,
        tcp_tf_node,
        camera_tf_node,
        apriltag_detector,
        web_viewer,
    ])
