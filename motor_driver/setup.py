from setuptools import setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iris',
    maintainer_email='teodor.tch@gmail.com',
    description='Python Driver Package for interfacing with the Arduino Motor Controller.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'motor_driver = motor_driver.motor_driver:main'
        ],
    },
)
