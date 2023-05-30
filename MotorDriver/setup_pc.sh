sudo apt-get install -y python3.8-venv zip unzip g++ autoconf automake libtool curl make

cwd=$(pwd)
cd lib/

# mkdir bazel/
# cd bazel/
# version="6.2.0"
# link="https://github.com/bazelbuild/bazel/releases/download/$version/bazel-$version-installer-linux-x86_64.sh"
# wget $link
# chmod +x bazel-$version-installer-linux-x86_64.sh
# ./bazel-$version-installer-linux-x86_64.sh --user
# export PATH="$PATH:$HOME/bin"
# echo "export PATH="$PATH:$HOME/bin"" >> ~/.bashrc
# cd ..

version="21.5"
link="https://github.com/protocolbuffers/protobuf/archive/refs/tags/v$version.zip"
proto_zip_name=protoc-$version
wget $link -O $proto_zip_name.zip
unzip $proto_zip_name.zip
rm $proto_zip_name.zip
proto_lib_name="protobuf-$version"
cd $proto_lib_name/

sudo apt-get remove -y protobuf-compiler
# Build Instructions: https://github.com/protocolbuffers/protobuf/blob/21.x/src/README.md
./autogen.sh
./configure
make -j$(nproc)
sudo make install
sudo ldconfig
cd ../

git submodule update --init --recursive
cd EmbeddedProto
python3 setup.py
cd $cwd

# Platformio Setup
# Enable linux udev rules for platformio extension
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart
sudo usermod -a -G dialout $USER
sudo usermod -a -G plugdev $USER