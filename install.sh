#!/bin/sh

# Exit immediately if a command exits with a non-zero status.
set -e

# Dependencies
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-robot-state-publisher -y
pip3 install PyQt5

# Build
git clone https://gitee.com/bhswift/swarm_sync_sim.git
cd swarm_sync_sim/
catkin_make
echo "source $PWD/devel/setup.bash" >> ~/.bashrc
