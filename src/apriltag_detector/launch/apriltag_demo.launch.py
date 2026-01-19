#!/usr/bin/env python3
"""
Launch file for AutonOHM AprilTag Detection Demo
RoboCup German Open
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    tag_family_arg = DeclareLaunchArgument(
        'tag_family',
        default_value='tag36h11',
        description='AprilTag family (tag36h11, tag25h9, tag16h5, etc.)'
    )
    
    tag_size_arg = DeclareLaunchArgument(
        'tag_size',
        default_value='0.05',
        description='AprilTag size in meters'
    )
    
    zmq_port_arg = DeclareLaunchArgument(
        'zmq_port',
        default_value='5555',
        description='ZMQ port for receiving webcam frames'
    )
    
    # Webcam receiver node
    webcam_receiver = Node(
        package='apriltag_detector',
        executable='webcam_receiver_node',
        name='webcam_receiver',
        output='screen',
        parameters=[{
            'zmq_port': LaunchConfiguration('zmq_port'),
            'frame_id': 'camera_link',
            'image_width': 640,
            'image_height': 480,
            'fps': 30.0,
        }]
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
            'camera_frame': 'camera_link',
        }]
    )
    
    return LaunchDescription([
        tag_family_arg,
        tag_size_arg,
        zmq_port_arg,
        webcam_receiver,
        apriltag_detector,
    ])
