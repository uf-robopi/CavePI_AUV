from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'auv_rpi'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aqua2',
    maintainer_email='aqua2@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autopilot_util = auv_rpi.autopilot_util:main',
            'autopilot = auv_rpi.autopilot:main',
            'data_receiver = auv_rpi.data_receiver:main',
            'ping2_publisher = auv_rpi.ping2_publisher:main', 
        ],
    },
)
