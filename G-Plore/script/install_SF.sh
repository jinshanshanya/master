#!/usr/bin/env bash

set -e

#echo "Installing protobuf..."
wget https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz
tar xzf protobuf-cpp-3.3.0.tar.gz

pushd protobuf-3.3.0
./configure
make -j4
#make check
make install
ldconfig
popd
# Clean up.
rm -fr protobuf-cpp-3.3.0.tar.gz protobuf-3.3.0

# Fix protobuf headers

if [ ! -f /usr/local/include/google/protobuf/stubs/stringprintf.h ];
then
    wget https://github.com/protocolbuffers/protobuf/blob/v3.3.0/src/google/protobuf/stubs/stringprintf.h
    wget https://github.com/protocolbuffers/protobuf/blob/v3.3.0/src/google/protobuf/stubs/strutil.h
    mv stringprintf.h /usr/local/include/google/protobuf/stubs/
    mv strutil.h /usr/local/include/google/protobuf/stubs/
fi

echo "Installing gflags and glog..."
wget https://github.com/gflags/gflags/archive/v2.0.tar.gz
tar xzf v2.0.tar.gz
pushd gflags-2.0
./configure
make -j4
make install
popd
# Install glog which also depends on gflags.
wget https://github.com/google/glog/archive/v0.3.3.tar.gz
tar xzf v0.3.3.tar.gz
pushd glog-0.3.3
./configure
make -j4
make install
popd
# Clean up.
rm -fr v2.0.tar.gz gflags-2.0 v0.3.3.tar.gz glog-0.3.3

# install catkin build tools
apt-get install -y python-catkin-tools

sudo apt-get install ros-melodic-usb-cam
sudo apt-get install ros-melodic-image-*
sudo apt-get install ros-melodic-rqt-image-view