#!/bin/bash
# Source this in every new terminal to set up ROS2 + this workspace:
#   source ~/ros2_ws/setup_env.sh
#
# Must be sourced, not executed (./setup_env.sh or bash setup_env.sh won't
# work) -- a script run as its own process can't change your shell's
# environment, only one that's sourced into your current shell can.

source /opt/ros/humble/setup.bash
source /home/feroj/ros2_ws/install/setup.bash
