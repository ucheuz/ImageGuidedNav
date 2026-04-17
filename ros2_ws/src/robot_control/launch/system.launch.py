import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """End-to-end stack: MoveIt Demo and Custom Listener."""
    
    # 1. Start the MoveIt "Brain" and RViz
    moveit_config_dir = get_package_share_directory('robotic_moveit_config')
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'demo.launch.py')
        )
    )

    # 2. Define your custom Slicer Listener
    listener_node = Node(
        package='robot_control',
        executable='igtl_listener', 
        name='igtl_listener',
        output='screen',
    )

    # Delay the listener by 8 seconds to let MoveIt fully load first
    delayed_listener = TimerAction(
        period=8.0,
        actions=[listener_node]
    )

    return LaunchDescription([
        moveit_launch,
        delayed_listener
    ])
