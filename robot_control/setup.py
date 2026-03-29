from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # FIXED: Automatically grabs all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # FIXED: Automatically grabs all URDF files (fixes the name mismatch)
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        
        # FIXED: THE MISSING MESHES FOLDER! This is why the robot was invisible.
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kosiasochukwu Uchemudi Uzoka',
    maintainer_email='k22030155@kcl.ac.uk',
    description='Surgical Robot Control Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Make sure your python script in the robot_control folder is actually named robot_kinematics.py!
            'kinematics_node = robot_control.robot_kinematics:main'
        ],
    },
)