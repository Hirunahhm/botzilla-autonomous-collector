from setuptools import find_packages, setup

package_name = 'botzilla_perception'

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
    maintainer='hiruna',
    maintainer_email='hiruna@todo.todo',
    description='YOLO and Kinect perception for BotZilla',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinect_bridge = botzilla_perception.kinect_bridge:main',
            'yolo_node = botzilla_perception.yolo_node:main',
            'apriltag_node = botzilla_perception.apriltag_node:main',
            'red_color_follower = botzilla_perception.red_color_follower:main'
        ],
    },
)