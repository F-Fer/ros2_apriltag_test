from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'apriltag_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutonOHM',
    maintainer_email='autonomhm@th-nuernberg.de',
    description='AprilTag detection for AutonOHM atwork robot - RoboCup German Open',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detector_node = apriltag_detector.apriltag_detector_node:main',
            'webcam_receiver_node = apriltag_detector.webcam_receiver_node:main',
            'visualization_node = apriltag_detector.visualization_node:main',
            'web_viewer_node = apriltag_detector.web_viewer_node:main',
        ],
    },
)
