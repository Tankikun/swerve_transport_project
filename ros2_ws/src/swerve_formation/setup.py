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
            'conveyor_base_node = swerve_formation.conveyor_base_node:main',
            'ekf_node = swerve_formation.ekf_node:main',
            'leader_election_node = swerve_formation.leader_election_node:main',
            'navigation_node = swerve_formation.navigation_node:main',
            'formation_size_node = swerve_formation.formation_size_node:main',
            'ai_camera_node = swerve_formation.ai_camera_node:main',
            '3d_slam_node = swerve_formation.slam_3d_node:main',
            # ── Simulation helpers ──────────────────────────────────────────
            'static_leader_publisher = swerve_formation.static_leader_publisher:main',
            'send_goal_node = swerve_formation.send_goal_node:main',
        ],
    },
)
