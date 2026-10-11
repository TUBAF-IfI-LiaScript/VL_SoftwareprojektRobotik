from glob import glob
from setuptools import setup

paket = 'verkehrswelt'
setup(
    name=paket,
    version='0.2.0',
    packages=[paket],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + paket]),
        ('share/' + paket, ['package.xml']),
        ('share/' + paket + '/config', glob('config/*.yaml')),
        ('share/' + paket + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'image_corrector = verkehrswelt.image_corrector:main',
            'boden_kalibrieren = verkehrswelt.boden_kalibrieren:main',
        ],
    },
)
