from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_nmpc_tracker'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Soja_the_First',
    maintainer_email='zuoyang061@gmail.com',
    description='Nonlinear Model Predictive Control (NMPC) package for drone person tracking',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmpc_tracker_node = drone_nmpc_tracker.nmpc_node:main',
            'nmpc_test_node = drone_nmpc_tracker.test_node:main',
        ],
    },
)

