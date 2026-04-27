import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_conveyor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Required for ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seven',
    maintainer_email='student@example.com',
    description='ROS 2 serial bridge + teleop for TurtleBot3 Conveyor',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge_node=turtlebot3_conveyor_bridge.serial_bridge_node:main',
            'teleop_keyboard_node=turtlebot3_conveyor_bridge.teleop_keyboard_node:main',
            'test_robot=turtlebot3_conveyor_bridge.test_robot:main',
        ],
    },
)
