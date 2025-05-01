from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'auv_nano'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/config', ['config/config.yaml']),
        ('share/' + package_name + '/weights', glob(os.path.join('weights', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adnan Abdullah',
    maintainer_email='adnanabdullah@ufl.edu',
    description='ROS2 Package for Perception and Control of CavePI AUV',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = auv_nano.detector:main',
            'planner = auv_nano.planner:main',
        ],
    },
)
