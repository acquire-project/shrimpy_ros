from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'waveorder_ros'

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
    maintainer='Justin Eskesen',
    maintainer_email='jeskesen@chanzuckerberg.com',
    description='Nodes for performing phase reconstruction using waveorder',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phase_calibration_node = waveorder_ros.phase_calibration_node:main'
        ],
    },
)
