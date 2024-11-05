from setuptools import find_packages, setup

package_name = 'triggerscope_ros'

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
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'triggerscope_node = triggerscope_ros.triggerscope_node:main',
            'ascii_serial_transport_node = triggerscope_ros.ascii_serial_transport_node:main',
        ],
    },
)
