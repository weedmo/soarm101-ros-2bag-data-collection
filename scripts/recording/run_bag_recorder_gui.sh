#!/bin/bash
# ROS 2 Bag Recorder GUI Launcher

source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash

echo "╔════════════════════════════════════════════════════════════╗"
echo "║           ROS 2 Bag Recorder GUI                           ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Starting ROS 2 Bag Recorder GUI..."
echo ""
echo "Topics to be recorded:"
echo "  • /camera/color/image_raw/compressed"
echo "  • /camera/depth/color/points"
echo "  • /camera/image_raw/compressed"
echo "  • /lerobot/follower/joint_states"
echo ""
echo "Camera preview:"
echo "  • Head View: /camera/camera/color/image_raw/compressed"
echo "  • Wrist View: /camera/image_raw/compressed"
echo ""
echo "Keyboard shortcuts:"
echo "  • A: Start recording"
echo "  • S: Save recording"
echo "  • D: Cancel recording"
echo ""

# Get the directory where this script is located and navigate to LeRobot root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LEROBOT_ROOT="$( cd "$SCRIPT_DIR/../../.." && pwd )"

cd "$LEROBOT_ROOT"
python3 ros2/tools/ros2_bag_recorder_gui_v2.py
