import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    """End-to-end stack: OpenIGTLink bridge, IK node, robot_state_publisher, RViz (no joint_state GUI)."""
    pkg_share = get_package_share_directory('robot_control')
    xacro_file = os.path.join(pkg_share, 'urdf', 'Robotic.xacro')
    robot_desc = {'robot_description': Command(['xacro ', xacro_file])}

    return LaunchDescription([
        Node(
            package='ros2_igtl_bridge',
            executable='igtl_node',
            name='igtlink_bridge',
            parameters=[{'port': 18944}],
        ),
        Node(
            package='robot_control',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_desc],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
    ])