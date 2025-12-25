from setuptools import setup
import os
from glob import glob

package_name = 'autonomous_car_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS Developer',
    maintainer_email='user@example.com',
    description='Simple autonomous car simulation with planning and control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_simulator = autonomous_car_sim.vehicle_simulator:main',
            'super_state_spy = autonomous_car_sim.super_state_spy:main',
            'path_planner = autonomous_car_sim.path_planner:main',
            'vehicle_controller = autonomous_car_sim.vehicle_controller:main',
        ],
    },
)
