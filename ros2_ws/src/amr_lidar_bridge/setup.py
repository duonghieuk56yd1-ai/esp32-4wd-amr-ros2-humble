from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'amr_lidar_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AMR maintainer',
    maintainer_email='amr@example.com',
    description='Nhận gói LiDAR UDP từ ESP32 (cổng 8889) và xuất bản sensor_msgs/LaserScan /scan',
    license='Apache-2.0',
    # tests_require cho setuptools cũ (Ubuntu 22.04), extras_require cho bản mới
    tests_require=['pytest'],
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'lidar_bridge = amr_lidar_bridge.bridge_node:main',
        ],
    },
)
