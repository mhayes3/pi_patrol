from setuptools import find_packages, setup

package_name = 'pi_patrol'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pi_patrol.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mhayes3',
    maintainer_email='matt.hayes13@yahoo.com',
    description='A ROS2 package for a security robot that uses a camera and YOLO for intruder detection and tracking.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_node = pi_patrol.camera_node:main',
        'detection_node = pi_patrol.detection_node:main',
        'recorder_node = pi_patrol.recorder_node:main',
        'telegram_notifier_node = pi_patrol.telegram_notifier_node:main',
        'server_node = pi_patrol.server_node:main',
        #'motor_control_node = pi_patrol.motor_control_node:main',
        #'follow_target_node = pi_patrol.follow_target_node:main',
        ],
    },
)
