#!/bin/bash
# ROS 2 Bag Recorder GUI with Metadata Launcher

source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash

echo "╔════════════════════════════════════════════════════════════╗"
echo "║    ROS 2 Bag Recorder GUI with Metadata Collection        ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Starting ROS 2 Bag Recorder with Metadata Collection..."
echo ""
echo "Features:"
echo "  • Real-time camera preview (Head + Wrist)"
echo "  • Metadata collection (Task, Tags, Operator, Context)"
echo "  • Topic selection (Choose which topics to record)"
echo "  • Automatic JSON generation (Schema v1.0.0)"
echo "  • Keyboard shortcuts (A/S/D)"
echo "  • Robot models: RB-Y1, SO-ARM100, SO101, UR5e, etc."
echo ""
echo "Default topics (pre-selected):"
echo "  ✓ /camera/color/image_raw/compressed"
echo "  ✓ /camera/depth/color/points"
echo "  ✓ /camera/image_raw/compressed"
echo "  ✓ /lerobot/follower/joint_states"
echo ""
echo "You can select additional topics using the GUI!"
echo ""

# Get the directory where this script is located and navigate to LeRobot root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LEROBOT_ROOT="$( cd "$SCRIPT_DIR/../../.." && pwd )"

cd "$LEROBOT_ROOT"
python3 ros2/tools/ros2_bag_recorder_gui_v2.py
