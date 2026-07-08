#!/usr/bin/env python3
"""brain.launch.py — lance le cerveau du bras."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    brain_node = Node(
        package="so101_brain",
        executable="brain",
        name="brain",
        output="screen",
        parameters=[
            {"use_sim": True},
        ],
    )

    return LaunchDescription([brain_node])
