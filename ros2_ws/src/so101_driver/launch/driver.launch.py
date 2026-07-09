#!/usr/bin/env python3
"""driver.launch.py — lance le driver du bras SO-ARM101.

    ros2 launch so101_driver driver.launch.py                # simulation
    ros2 launch so101_driver driver.launch.py use_sim:=false # bras réel
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="Backend simulation (true) ou bras réel (false)"),
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0",
                              description="Port série du bras réel"),
        Node(
            package="so101_driver",
            executable="driver",
            name="so101_driver",
            output="screen",
            parameters=[{
                "use_sim": ParameterValue(LaunchConfiguration("use_sim"), value_type=bool),
                "port": LaunchConfiguration("port"),
            }],
        ),
    ])
