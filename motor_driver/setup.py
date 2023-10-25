from setuptools import setup
from setuptools.command.install import install
from setuptools import find_packages
import warnings
import sys
from sys import stderr
import subprocess
import os
from glob import glob

package_name = 'motor_driver'

class PythonProtobufGenerator(install):
    def run(self):
        install.run(self)
        print("Generating Python Protobuf Files:")
        # Example command from within motor_driver package:
        # protoc --plugin=protoc-gen-eams=../MotorDriver/lib/EmbeddedProto/protoc-gen-eams -I./proto -I../MotorDriver/lib/EmbeddedProto/generator --python_out=./motor_driver/generated ./proto/*.proto
        
        embedded_proto_path = "../MotorDriver/lib/EmbeddedProto"

        # It is recommended to use subprocess: https://stackoverflow.com/a/89243
        # Couldn't figure out how to get EmbeddedProto to be visible.
        os.system("protoc" + " "
                  + f"--plugin=protoc-gen-eams={embedded_proto_path}/protoc-gen-eams" + " "
                  + f"-I./proto" + " "
                  + f"-I{embedded_proto_path}/generator" + " "
                  + f"--python_out=pyi_out:./{package_name}/generated" + " "
                  + f"./proto/*.proto"
        )

        # Attempt at using subprocess
        # embedded_proto_path = "~/colcon_ws/src/IRIS/MotorDriver/lib/EmbeddedProto"
        # result = subprocess.run(["protoc"
        #                          , f"--plugin=protoc-gen-eams={embedded_proto_path}/protoc-gen-eams"
        #                          , f"-I./proto"
        #                          , f"-I{embedded_proto_path}/generator"
        #                          , f"--python_out=./{package_name}/generated"
        #                          , f"./proto/example.proto"]
        #                         #  , check=False
        #                          , capture_output=True
        #                         #  , shell=True
        #                          )
        # if result.returncode:
        #     print(" [Fail]")
        #     print(result.stderr.decode("utf-8"), end='', file=stderr)
        #     exit(1)
        # else:
        #     print(" [Success]")

        # Code to print system warnings
        # warnings.warn('My warning.', UserWarning)

# Useful Tutorial: https://roboticsbackend.com/create-a-ros2-python-package/
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'glob'],
    zip_safe=True,
    include_package_data=True, # Might not be necessary https://stackoverflow.com/q/7522250
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
    cmdclass={'install': PythonProtobufGenerator}, #Might need to check if it occurs before the installation procedure is done
)
