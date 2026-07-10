from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'botzilla_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register with ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # Install the robot URDF description
        (os.path.join('share', package_name, 'description'),
            glob('description/*.urdf')),

        # Install Gazebo world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiruna',
    maintainer_email='hirunamalavipathirana.333@gmail.com',
    description='Launch files for BotZilla simulation and hardware bringup',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
