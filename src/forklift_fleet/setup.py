from glob import glob

from setuptools import find_packages, setup

package_name = 'forklift_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keshavwxyz',
    maintainer_email='keshavbhardwaj5354@gmail.com',
    description='Multi-forklift task allocation, coordination and conflict resolution.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fleet_coordinator = forklift_fleet.coordinator_node:main',
            'fleet_agent = forklift_fleet.fleet_agent:main',
        ],
    },
)
