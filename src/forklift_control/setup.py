from glob import glob

from setuptools import find_packages, setup

package_name = 'forklift_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='keshavwxyz',
    maintainer_email='keshavbhardwaj5354@gmail.com',
    description='Forklift drive/lift control: controllers, twist_mux, lift action server.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lift_controller = forklift_control.lift_controller:main',
            'load_handler = forklift_control.load_handler:main',
            'rear_safety = forklift_control.rear_safety:main',
        ],
    },
)
