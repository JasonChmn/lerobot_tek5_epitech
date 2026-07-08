"""robot_state_publisher pour le SO-ARM101.

    ros2 launch so101_description viz.launch.py            # attend /joint_states
    ros2 launch so101_description viz.launch.py demo:=true # joints figés à 0 (smoke test)

Publie /robot_description et les TF du bras. Visualisation : lancer rviz2
depuis le container control (X11) — voir docs/INSTRUCTION.md.

IMPORTANT : sans nœud publiant /joint_states (votre driver_node !), le TF des
joints n'existe pas → le bras apparaît "en morceaux" dans RViz. C'est normal,
pas un bug : publiez sensor_msgs/JointState depuis votre driver.
demo:=true lance un joint_state_publisher factice (pose zéro) pour vérifier
la chaîne d'affichage indépendamment de votre code.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory("so101_description")) / "urdf" / "so101.urdf"
    robot_description = urdf.read_text()

    return LaunchDescription([
        DeclareLaunchArgument("demo", default_value="false",
                              description="Publier des joint_states factices (pose zéro)"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # Smoke test uniquement — en usage normal, /joint_states vient du driver_node.
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(LaunchConfiguration("demo")),
        ),
    ])
