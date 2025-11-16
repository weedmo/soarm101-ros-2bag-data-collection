#!/bin/bash
# Direct calibration script for SO101 Follower (without ROS2 services)

source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash

# Run direct calibration (not ROS2 service based)
python3 << 'PYTHON_SCRIPT'
import sys
sys.path.insert(0, '/home/weed/lerobot/src')

from lerobot.robots import make_robot_from_config
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig

config = SO101FollowerConfig(
    port="/dev/ttyACM0",
    id="follower"
)

robot = make_robot_from_config(config)
robot.connect(calibrate=False)

print("\n" + "="*60)
print("Starting calibration for SO101 Follower")
print("="*60 + "\n")

robot.calibrate()

print("\n" + "="*60)
print("Calibration completed!")
print(f"Calibration saved to: {robot.calibration_fpath}")
print("="*60 + "\n")

robot.disconnect()
PYTHON_SCRIPT
