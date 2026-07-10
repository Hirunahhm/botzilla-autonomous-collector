import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'botzilla_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiruna',
    maintainer_email='hirunamalavipathirana.333@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kobuki_base_node = botzilla_control.kobuki_base_node:main',
            'brain_node = botzilla_control.brain_node:main',
            'cube_collector = botzilla_control.cube_collector:main',
            'perception_simulator = botzilla_control.perception_simulator:main',
            'tag_follower_node = botzilla_control.tag_follower_node:main',
            'encoder_test_node = botzilla_control.encoder_test_node:main',
            'nav_test_node = botzilla_control.nav_test_node:main',
            'final_test_node = botzilla_control.final_test_node:main',
        ],
    },
)
