from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'line_follower'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Softwareprojekt Robotik',
    maintainer_email='student@tu-freiberg.de',
    description='Linienverfolgung mit Hinderniserkennung für TurtleBot 3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_detector_node = line_follower.line_detector_node:main',
            'controller_node = line_follower.controller_node:main',
            'obstacle_detector_node = line_follower.obstacle_detector_node:main',
        ],
    },
)
