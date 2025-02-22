#### Disclaimer
This project is not affiliated with, endorsed by, or in any way connected to REV Robotics. All product names, logos, and brands are property of their respective owners. Any references to REV Robotics products, such as the SPARK MAX or SPARK Flex, are purely for identification purposes and do not imply any endorsement or sponsorship.

#### Setup
A USB to CAN adapter such as a [CANable](https://canable.io/) is required. This package has been tested with the [Spark MAX](https://www.revrobotics.com/rev-11-2158/) and [SPARK Flex](https://www.revrobotics.com/rev-11-2159/).

Make sure to use [REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/gs/install) to set the `device ID` for your controllers and modify the values to match them in the examples.  Ensure the correct motor type is set to either `kBrushed` or `kBrushless` depending on your motor, as it may damage the motor if set to the wrong type.

#### Installation
```bash
sudo apt update
sudo apt upgrade
sudo apt install git cmake

sudo add-apt-repository ppa:graysonarendt/sparkcan
sudo apt update
sudo apt install sparkcan

git clone https://github.com/grayson-arendt/sparkcan-examples.git
cd sparkcan-examples/

chmod +x canable_start.sh
./canable_start.sh

mkdir build
cd build
cmake ..
make
```

#### Running an Example

```bash
./example_control # OR ./example_status
```
