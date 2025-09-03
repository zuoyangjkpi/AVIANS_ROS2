from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_nmpc_tracker'

# Debug: Print what we're finding
print(f"Package name: {package_name}")
print(f"Packages found: {find_packages(exclude=['test'])}")
print(f"Current directory: {os.getcwd()}")

# Check if launch files exist
launch_files = glob(os.path.join('launch', '*.launch.py'))
config_files = glob(os.path.join('config', '*.yaml'))
print(f"Launch files found: {launch_files}")
print(f"Config files found: {config_files}")

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), launch_files),
        (os.path.join('share', package_name, 'config'), config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
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

