from setuptools import find_packages, setup
import os

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Tell ROS2 where to find your launch and urdf files
        (os.path.join('share', package_name, 'launch'), ['launch/system.launch.py']),
        (os.path.join('share', package_name, 'urdf'), ['urdf/surgical_robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KCL Student',
    maintainer_email='student@kcl.ac.uk',
    description='Surgical Robot Control Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This links the executable name to your python file
            'kinematics_node = robot_control.robot_kinematics:main'
        ],
    },
)