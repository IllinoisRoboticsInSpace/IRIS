# This script is needed to compile and install a desired version of opencv with CUDA enabled on the Jetson Nano.
# This script follows the instructions from this link: https://qengineering.eu/install-opencv-4.5-on-jetson-nano.html
# This script has opted to not use zram but instead make an 8GB SD memory swap file.

sudo apt-get -y update
sudo apt-get -y upgrade

sudo swapoff /swapfile
sudo rm /swapfile
sudo dd if=/dev/zero of=/swapfile bs=1M count=8192
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

free -m

sudo apt-get install -y build-essential cmake git unzip pkg-config
sudo apt-get install -y libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libgtk2.0-dev libcanberra-gtk*
sudo apt-get install -y python3-dev python3-numpy python3-pip
sudo apt-get install -y libxvidcore-dev libx264-dev libgtk-3-dev
sudo apt-get install -y libtbb2 libtbb-dev libdc1394-22-dev
sudo apt-get install -y gstreamer1.0-tools libv4l-dev v4l-utils
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
sudo apt-get install -y libavresample-dev libvorbis-dev libxine2-dev
sudo apt-get install -y libfaac-dev libmp3lame-dev libtheora-dev
sudo apt-get install -y libopencore-amrnb-dev libopencore-amrwb-dev
sudo apt-get install -y libopenblas-dev libatlas-base-dev libblas-dev
sudo apt-get install -y liblapack-dev libeigen3-dev gfortran
sudo apt-get install -y libhdf5-dev protobuf-compiler
sudo apt-get install -y libprotobuf-dev libgoogle-glog-dev libgflags-dev
sudo apt-get install -y qt5-default

cwd=$(pwd)
cd ~/
mkdir IRIS_dependencies
cd IRIS_dependencies
version="4.2.0"
wget -O opencv.zip https://github.com/opencv/opencv/archive/$version.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/$version.zip
unzip opencv.zip 
unzip opencv_contrib.zip
mv opencv-$version opencv
mv opencv_contrib-$version opencv_contrib
rm opencv.zip 
rm opencv_contrib.zip

# Build opencv
cd opencv/
mkdir build
cd build/

# QT has been turned on
cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr \
-D OPENCV_EXTRA_MODULES_PATH=~/IRIS_dependencies/opencv_contrib/modules \
-D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
-D WITH_OPENCL=OFF \
-D WITH_CUDA=ON \
-D CUDA_ARCH_BIN=5.3 \
-D CUDA_ARCH_PTX="" \
-D WITH_CUDNN=ON \
-D WITH_CUBLAS=ON \
-D ENABLE_FAST_MATH=ON \
-D CUDA_FAST_MATH=ON \
-D OPENCV_DNN_CUDA=ON \
-D ENABLE_NEON=ON \
-D WITH_QT=ON \
-D WITH_OPENMP=ON \
-D BUILD_TIFF=ON \
-D WITH_FFMPEG=ON \
-D WITH_GSTREAMER=ON \
-D WITH_TBB=ON \
-D BUILD_TBB=ON \
-D BUILD_TESTS=OFF \
-D WITH_EIGEN=ON \
-D WITH_V4L=ON \
-D WITH_LIBV4L=ON \
-D OPENCV_ENABLE_NONFREE=ON \
-D INSTALL_C_EXAMPLES=OFF \
-D INSTALL_PYTHON_EXAMPLES=OFF \
-D BUILD_NEW_PYTHON_SUPPORT=ON \
-D BUILD_opencv_python3=TRUE \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-D BUILD_EXAMPLES=OFF ..

# This is the original build options from rtabmap docker file for comparison.
# DO NOT build with these options.
# https://github.com/introlab/rtabmap/blob/master/docker/focal-foxy/deps/Dockerfile
# cmake -D BUILD_opencv_python3=OFF \
# -D BUILD_opencv_python_bindings_generator=OFF \
# -D BUILD_opencv_python_tests=OFF \
# -D BUILD_PERF_TESTS=OFF \
# -D BUILD_TESTS=OFF \
# -D OPENCV_ENABLE_NONFREE=ON \
# -D OPENCV_EXTRA_MODULES_PATH=/root/opencv_contrib/modules ..

make -j$(nproc)

sudo rm -r /usr/include/opencv4/opencv2
sudo make install
sudo ldconfig
make clean
sudo apt-get update

cd $cwd
