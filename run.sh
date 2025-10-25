#!/bin/bash
set -e

# Set up CAN interface
sudo modprobe vcan || true
sudo ip link set vcan0 down 2>/dev/null || true
sudo ip link add dev vcan0 type vcan || true
sudo ip link set vcan0 up

# Build the kernel module
echo "Building controller kernel module..."
make -C /lib/modules/$(uname -r)/build M=$PWD/controller modules

# Load the kernel module
echo "Loading controller kernel module..."
sudo rmmod controller_kernel 2>/dev/null || true
sudo insmod controller/controller_kernel.ko ifname=vcan0 period_ms=100 idle_ms=1500

# Build the plant_user.c file
echo "Building plant_user.c..."
gcc -O2 -Wall -o plant_user plant_user.c -lm

# Build the ctrl_set.c file
echo "Building ctrl_set.c..."
gcc -O2 -Wall -o ctrl_set ctrl_set.c -lm

# Run the executables in three different terminals (no GNOME/D-Bus needed)
echo "Launching terminals..."
x-terminal-emulator -T "Plant User"     -e bash -lc " ./plant_user vcan0 --Ts 60 --Th 40 --Tc 20 --v_prev 1200 --dt_ms 15 --mdot 0.25; exec bash" &
x-terminal-emulator -T "Controller Set" -e bash -lc 'echo "./ctrl_set vcan0 30.0 --kp 120 --ki 0.15 --kd 5 --kaw 4 --kpm 150 --kim 0.02 --kawm 8 --kvw -0.1 --kwv -0.03"; echo; read -p "Press Enter to run it (or edit manually): "; ./ctrl_set vcan0 30.0 --kp 120 --ki 0.15 --kd 5 --kaw 4 --kpm 150 --kim 0.02 --kawm 8 --kvw -0.1 --kwv -0.03; exec bash' &
x-terminal-emulator -T "Controller Log"     -e bash -lc "journalctl -k -f; exec bash" &

wait
