#!/bin/bash
# ROS2 Calibration Wrapper Script
# Usage: ./run_calibrate_ros2.sh [calibration arguments]

# Activate conda environment and source ROS2
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash

# Run calibration
lerobot-calibrate-ros2 "$@"
