#!/bin/bash
set -e
# Run unit tests for plant_user and ctrl_set

# Set up CAN interface
sudo modprobe vcan || true
sudo ip link set vcan0 down 2>/dev/null || true
sudo ip link add dev vcan0 type vcan || true
sudo ip link set vcan0 up

# Build unit tests
mkdir -p build && cd build
cmake ..
make -j

sudo ctest -I 1,14 --output-on-failure

cd ..
cd controller


make
sudo rmmod controller_kernel 2>/dev/null || true
sudo insmod controller_kernel.ko kunit_no_hw=1
sudo rmmod tests/nodeb_kunit_test 2>/dev/null || true
sudo insmod tests/nodeb_kunit_test.ko
dmesg -w | grep -A3 nodeb-controller-kunit-oot