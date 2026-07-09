#!/usr/bin/env python3
"""perception.launch.py — lance la perception de la boule."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    perception_node = Node(
        package="so101_perception",
        executable="perception",
        name="perception",
        output="screen",
        respawn=True,
        parameters=[
            {"camera_info_topic": "external_cam/camera_info"},
            {"image_topic": "external_cam/image_raw"},
        ],
    )

    return LaunchDescription([perception_node])
