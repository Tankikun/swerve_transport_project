from setuptools import find_packages, setup

package_name = 'swerve_formation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tankikun',
    maintainer_email='tankhun2547l@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'laplacian_formation_node = swerve_formation.laplacian_formation_node:main',
            'fake_swerve_simulator = swerve_formation.fake_swerve_simulator:main',
            'serial_bridge_node = swerve_formation.serial_bridge_node:main',
        ],
    },
)
