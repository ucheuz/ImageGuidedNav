import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    """
    RViz + robot_state_publisher only (no joint GUI).
    Use this with kinematics_node + OpenIGTLink so only one node publishes /joint_states.
    For manual sliders use launch/display_with_gui.launch.py (never at the same time as kinematics_node).
    """
    pkg_share = get_package_share_directory('robot_control')
    xacro_file = os.path.join(pkg_share, 'urdf', 'Robotic.xacro')
    robot_desc = {'robot_description': Command(['xacro ', xacro_file])}

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_desc],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
