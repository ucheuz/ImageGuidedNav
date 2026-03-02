import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to your URDF file
    urdf_file = 'src/robot_control/urdf/surgical_robot.urdf'
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. Start the OpenIGTLink Bridge (Listens to 3D Slicer)
        Node(
            package='ros2_igtlink_bridge',
            executable='igtlink_node',
            name='igtlink_bridge',
            parameters=[{'port': 18944}]
        ),
        
        # 2. Start Your Custom Kinematics Node
        Node(
            package='robot_control',
            executable='kinematics_node',
            name='kinematics_node'
        ),
        
        # 3. Start the Robot State Publisher (Loads the URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 4. Start RViz for Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])