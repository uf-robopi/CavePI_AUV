from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/config.yaml']),
        ('share/' + package_name + '/weights', glob(os.path.join('weights', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cavepi',
    maintainer_email='cavepi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'caveline_detection_main = detector.caveline_detection_main:main',
        ],
    },
)
