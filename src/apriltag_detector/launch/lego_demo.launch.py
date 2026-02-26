#!/usr/bin/env python3
"""
Launch file for AutonOHM Lego Duplo Detection Demo
RoboCup @Work — RealSense D435 on robot arm end-effector.

Run on Mac host first:
  python3 realsense_sender.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    params_file = ParameterFile(
        os.path.join(
            get_package_share_directory('apriltag_detector'),
            'config', 'apriltag_params.yaml'
        ),
        allow_substs=True
    )

    realsense_receiver = Node(
        package='apriltag_detector',
        executable='realsense_receiver_node',
        name='realsense_receiver',
        output='screen',
        parameters=[params_file],
    )

    lego_detector = Node(
        package='apriltag_detector',
        executable='lego_detector_node',
        name='lego_detector',
        output='screen',
        parameters=[params_file],
    )

    web_viewer = Node(
        package='apriltag_detector',
        executable='web_viewer_node',
        name='web_viewer',
        output='screen',
        parameters=[{
            'port': 8080,
            'image_topic': '/lego/image_annotated',
        }],
    )

    return LaunchDescription([
        realsense_receiver,
        lego_detector,
        web_viewer,
    ])
