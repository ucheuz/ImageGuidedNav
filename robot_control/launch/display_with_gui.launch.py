import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    """
    RViz + URDF + joint_state_publisher_gui for manual joint posing.
    Do not run this together with kinematics_node: both publish /joint_states and RViz will look broken.
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
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
        ),
    ])
