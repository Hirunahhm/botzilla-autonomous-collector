import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'botzilla_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiruna',
    maintainer_email='hirunamalavipathirana.333@gmail.com',
    description='SLAM and Nav2 bring-up for BotZilla Phase 1 (RTAB-Map primary, slam_toolbox fallback)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_covariance_relay = botzilla_navigation.odom_covariance_relay:main',
            'frontier_explorer_node = botzilla_navigation.frontier_explorer_node:main',
            'executor_node = botzilla_navigation.executor_node:main',
            'mission_metrics_node = botzilla_navigation.mission_metrics_node:main',
        ],
    },
)
