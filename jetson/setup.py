from setuptools import setup
import os
from glob import glob

package_name = 'jetson'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iris-autonomous',
    maintainer_email='iris.uiuc@gmail.com',
    description='Jetson package for IRIS-2022',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_comms_node = jetson.arduino_comms_node:main'
        ],
    },
)
