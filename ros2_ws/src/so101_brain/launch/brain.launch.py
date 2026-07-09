#!/usr/bin/env python3
"""brain.launch.py — lance le cerveau du bras (IK + pick & place)."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="so101_brain",
            executable="brain",
            name="so101_brain",
            output="screen",
        ),
    ])
