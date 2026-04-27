import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'swerve_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tankikun',
    maintainer_email='tankhun2547l@gmail.com',
    description='Launch files for the swerve formation system',
    license='TODO: License declaration',
    entry_points={'console_scripts': []},
)
