from setuptools import setup
import os
from glob import glob

package_name = 'teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (f'share/{package_name}/config',
            ['resource/xbox.config.yaml']),
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iris-autonomous-team',
    maintainer_email='iris.uiuc@gmail.com',
    description='Teleop package for IRIS (2023)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = teleop.teleop_node:main', # name = package.file:method/
        ],
    },
)
