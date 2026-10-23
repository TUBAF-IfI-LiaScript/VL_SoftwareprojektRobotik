from setuptools import setup
import os
from glob import glob

package_name = 'cloud_processing'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'scikit-learn'],
    zip_safe=True,
    maintainer='Dein Name',
    maintainer_email='dein.email@example.com',
    description='ROS2 PointCloud Processing Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'filter_node = cloud_processing.range_filter_node:main',
            'ground_detection_node = cloud_processing.ground_detection_node:main',
        ],
    },
)