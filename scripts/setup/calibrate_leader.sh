#!/bin/bash
# Direct calibration script for SO101 Leader (without ROS2 services)

source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash

# Run direct calibration (not ROS2 service based)
python3 << 'PYTHON_SCRIPT'
import sys
sys.path.insert(0, '/home/weed/lerobot/src')

from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig

config = SO101LeaderConfig(
    port="/dev/ttyACM1",
    id="leader"
)

teleop = make_teleoperator_from_config(config)
teleop.connect(calibrate=False)

print("\n" + "="*60)
print("Starting calibration for SO101 Leader")
print("="*60 + "\n")

teleop.calibrate()

print("\n" + "="*60)
print("Calibration completed!")
print(f"Calibration saved to: {teleop.calibration_fpath}")
print("="*60 + "\n")

teleop.disconnect()
PYTHON_SCRIPT
