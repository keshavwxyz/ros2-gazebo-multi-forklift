from glob import glob

from setuptools import find_packages, setup

package_name = 'forklift_task_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'mission_manager = forklift_task_manager.mission_node:main',
        ],
    },
    zip_safe=True,
    maintainer='keshavwxyz',
    maintainer_email='keshavbhardwaj5354@gmail.com',
    description='Mission configuration (single source of truth) and the mission manager.',
    license='Apache-2.0',
    tests_require=['pytest'],
)
