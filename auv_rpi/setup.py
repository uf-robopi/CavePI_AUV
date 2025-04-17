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
            'autopilot_util = cavepi_controller.autopilot_util:main',
            'autopilot = cavepi_controller.autopilot:main',
            'data_receiver = cavepi_controller.data_receiver:main',
            'ping2_publisher = cavepi_controller.ping2_publisher:main', 
        ],
    },
)
