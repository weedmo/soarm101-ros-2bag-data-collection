#!/bin/bash
#
# LeRobot Complete System Launcher
#
# This script launches all components in separate terminal windows:
# 1. RealSense Camera (PointCloud)
# 2. USB Camera Publisher
# 3. Teleoperation (Leader + Follower)
#

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         LeRobot Complete System Launcher                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "This will launch:"
echo "  1. RealSense Camera (PointCloud)"
echo "  2. USB Camera Publisher"
echo "  3. Teleoperation (Leader + Follower)"
echo ""
echo "Each component will run in a separate terminal window."
echo ""

# Get the LeRobot root directory (3 levels up from this script)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LEROBOT_ROOT="$( cd "$SCRIPT_DIR/../../.." && pwd )"
ROS2_SCRIPTS="$LEROBOT_ROOT/ros2/scripts"

# RealSense Camera configuration
REALSENSE_LAUNCH=${REALSENSE_LAUNCH:-"rs_pointcloud_launch.py"}

# USB Camera configuration
USB_CAMERA_INDEX=${USB_CAMERA_INDEX:-0}
USB_CAMERA_TOPIC=${USB_CAMERA_TOPIC:-"/camera/image_raw"}
USB_CAMERA_WIDTH=${USB_CAMERA_WIDTH:-640}
USB_CAMERA_HEIGHT=${USB_CAMERA_HEIGHT:-480}
USB_CAMERA_FPS=${USB_CAMERA_FPS:-30}

echo "Press ENTER to start all components, or Ctrl+C to cancel..."
read

echo ""
echo "Launching components..."
echo ""

# 1. Launch RealSense Camera in new terminal
echo "→ Launching RealSense Camera (PointCloud)..."
gnome-terminal --title="RealSense Camera" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    echo '╔════════════════════════════════════════════════════════════╗'
    echo '║      RealSense Camera (PointCloud + Compressed)            ║'
    echo '╚════════════════════════════════════════════════════════════╝'
    echo ''
    echo 'Launching RealSense camera with pointcloud...'
    echo ''
    ros2 launch realsense2_camera $REALSENSE_LAUNCH \
        enable_color:=true \
        enable_depth:=true \
        pointcloud.enable:=true \
        enable_sync:=true
    echo ''
    echo 'RealSense camera stopped.'
    read -p 'Press ENTER to close this window...'
" &

sleep 3

# 1-1. Launch compressed image publishers for RealSense
echo "→ Launching RealSense Compressed Image Publishers..."
gnome-terminal --title="RealSense Compressed" -- bash -c "
    source /opt/ros/jazzy/setup.bash
    echo '╔════════════════════════════════════════════════════════════╗'
    echo '║        RealSense Compressed Image Publishers               ║'
    echo '╚════════════════════════════════════════════════════════════╝'
    echo ''
    echo 'Publishing compressed images for RealSense color and depth...'
    echo ''

    # Launch color compressed publisher
    ros2 run image_transport republish raw compressed --ros-args \
        -r in:=/camera/color/image_raw \
        -r out/compressed:=/camera/color/image_raw/compressed &
    COLOR_PID=\$!

    # Launch depth compressed publisher
    ros2 run image_transport republish raw compressed --ros-args \
        -r in:=/camera/depth/image_rect_raw \
        -r out/compressed:=/camera/depth/image_rect_raw/compressed &
    DEPTH_PID=\$!

    echo 'Color compressed: /camera/color/image_raw/compressed'
    echo 'Depth compressed: /camera/depth/image_rect_raw/compressed'
    echo ''

    # Wait for processes
    wait \$COLOR_PID \$DEPTH_PID

    echo ''
    echo 'Compressed publishers stopped.'
    read -p 'Press ENTER to close this window...'
" &

sleep 2

# 2. Launch USB Camera Publisher in new terminal
echo "→ Launching USB Camera Publisher..."
gnome-terminal --title="USB Camera" -- bash -c "
    cd '$ROS2_SCRIPTS/camera'
    echo '╔════════════════════════════════════════════════════════════╗'
    echo '║              USB Camera Publisher                          ║'
    echo '╚════════════════════════════════════════════════════════════╝'
    echo ''
    ./run_camera_ros2.sh $USB_CAMERA_INDEX '$USB_CAMERA_TOPIC' $USB_CAMERA_WIDTH $USB_CAMERA_HEIGHT $USB_CAMERA_FPS
    echo ''
    echo 'USB Camera publisher stopped.'
    read -p 'Press ENTER to close this window...'
" &

sleep 2

# 3. Launch Teleoperation (Leader + Follower) in new terminal
echo "→ Launching Teleoperation (Leader + Follower)..."
gnome-terminal --title="Teleoperation" -- bash -c "
    cd '$ROS2_SCRIPTS/teleop'
    echo '╔════════════════════════════════════════════════════════════╗'
    echo '║         Teleoperation (Leader + Follower)                  ║'
    echo '╚════════════════════════════════════════════════════════════╝'
    echo ''
    ./run_teleop_all.sh
    echo ''
    echo 'Teleoperation stopped.'
    read -p 'Press ENTER to close this window...'
" &

sleep 1

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║              All Components Launched!                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Running components:"
echo "  ✓ RealSense Camera (PointCloud + Compressed)"
echo "  ✓ USB Camera Publisher (Raw + Compressed)"
echo "  ✓ Teleoperation (Leader + Follower)"
echo ""
echo "To stop all components:"
echo "  - Close each terminal window, or"
echo "  - Press Ctrl+C in each window"
echo ""
echo "Monitor ROS2 topics with:"
echo "  ros2 topic list"
echo "  ros2 topic echo /camera/image_raw"
echo "  ros2 topic echo /camera/image_raw/compressed"
echo "  ros2 topic echo /camera/depth/image_rect_raw/compressed"
echo "  ros2 topic echo /lerobot/leader/joint_states"
echo ""
echo "Visualize with:"
echo "  rviz2"
echo ""
