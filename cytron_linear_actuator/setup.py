from setuptools import setup
import os
from glob import glob

package_name = 'cytron_linear_actuator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rama Rao Vencharla',
    maintainer_email='ramarao3@illinois.edu',
    description='ROS2 node for controlling Cytron motor driver for linear actuator',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cytron_lin_actr = cytron_linear_actuator.cytron_linear_actuator_node:main',
        ],
    },
)