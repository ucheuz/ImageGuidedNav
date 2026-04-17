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
        
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # URDF and associated files
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')
            + glob('urdf/*.xacro')
            + glob('urdf/*.trans')
            + glob('urdf/*.gazebo'),
        ),
        
        # Meshes
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools', 'ikpy', 'numpy', 'pyigtl'],
    zip_safe=True,
    maintainer='Kosiasochukwu Uchemudi Uzoka',
    maintainer_email='k22030155@kcl.ac.uk',
    description='Surgical Robot Control Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics_node = robot_control.robot_kinematics:main',

            'igtl_listener = robot_control.igtl_listener:main',
        ],
    },
)
