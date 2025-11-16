#!/bin/bash
# ROS2 Teleoperation Wrapper Script
# Usage: ./run_teleoperate_ros2.sh [teleoperation arguments]

# Activate conda environment and source ROS2
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash

# Run teleoperation
lerobot-teleoperate-ros2 "$@"
