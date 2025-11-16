#!/bin/bash
#
# LeRobot Complete System Launcher (Background Mode)
#
# This script launches all components in the background:
# 1. RealSense Camera (PointCloud)
# 2. USB Camera Publisher
# 3. Teleoperation Leader
# 4. Teleoperation Follower
#
# All processes run in the background and can be stopped with Ctrl+C
#

echo "╔════════════════════════════════════════════════════════════╗"
echo "║    LeRobot Complete System Launcher (Background Mode)     ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Log directory
LOG_DIR="$SCRIPT_DIR/log"
mkdir -p "$LOG_DIR"

# PID file
PID_FILE="$SCRIPT_DIR/.launch_all.pid"

# Configuration
USB_CAMERA_INDEX=${USB_CAMERA_INDEX:-0}
USB_CAMERA_TOPIC=${USB_CAMERA_TOPIC:-"/camera/image_raw"}
USB_CAMERA_WIDTH=${USB_CAMERA_WIDTH:-640}
USB_CAMERA_HEIGHT=${USB_CAMERA_HEIGHT:-480}
USB_CAMERA_FPS=${USB_CAMERA_FPS:-30}

# Array to store PIDs
declare -a PIDS

# Cleanup function
cleanup() {
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║              Shutting Down All Components                 ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""

    # Kill all processes
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "→ Stopping process (PID: $pid)..."
            kill "$pid" 2>/dev/null
        fi
    done

    # Wait for all processes to finish
    sleep 2

    # Force kill if still running
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "→ Force stopping process (PID: $pid)..."
            kill -9 "$pid" 2>/dev/null
        fi
    done

    rm -f "$PID_FILE"

    echo ""
    echo "All components stopped."
    echo ""
    exit 0
}

# Trap signals
trap cleanup SIGINT SIGTERM

echo "Starting all components in background mode..."
echo "Logs will be saved to: $LOG_DIR"
echo ""

# 1. Launch RealSense Camera
echo "→ Launching RealSense Camera (PointCloud)..."
bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 launch realsense2_camera rs_pointcloud_launch.py \
        enable_color:=true \
        enable_depth:=true \
        pointcloud.enable:=true \
        enable_sync:=true
" > "$LOG_DIR/realsense.log" 2>&1 &
REALSENSE_PID=$!
PIDS+=($REALSENSE_PID)
echo "  ✓ Started (PID: $REALSENSE_PID)"
sleep 3

# 1-1. Launch RealSense Color Compressed Publisher
echo "→ Launching RealSense Color Compressed Publisher..."
bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 run image_transport republish raw compressed --ros-args \
        -r in:=/camera/color/image_raw \
        -r out/compressed:=/camera/color/image_raw/compressed
" > "$LOG_DIR/realsense_color_compressed.log" 2>&1 &
REALSENSE_COLOR_COMPRESSED_PID=$!
PIDS+=($REALSENSE_COLOR_COMPRESSED_PID)
echo "  ✓ Started (PID: $REALSENSE_COLOR_COMPRESSED_PID)"

# 1-2. Launch RealSense Depth Compressed Publisher
echo "→ Launching RealSense Depth Compressed Publisher..."
bash -c "
    source /opt/ros/jazzy/setup.bash
    ros2 run image_transport republish raw compressed --ros-args \
        -r in:=/camera/depth/image_rect_raw \
        -r out/compressed:=/camera/depth/image_rect_raw/compressed
" > "$LOG_DIR/realsense_depth_compressed.log" 2>&1 &
REALSENSE_DEPTH_COMPRESSED_PID=$!
PIDS+=($REALSENSE_DEPTH_COMPRESSED_PID)
echo "  ✓ Started (PID: $REALSENSE_DEPTH_COMPRESSED_PID)"
sleep 2

# 2. Launch USB Camera
echo "→ Launching USB Camera Publisher..."
"$SCRIPT_DIR/run_camera_ros2.sh" \
    $USB_CAMERA_INDEX \
    "$USB_CAMERA_TOPIC" \
    $USB_CAMERA_WIDTH \
    $USB_CAMERA_HEIGHT \
    $USB_CAMERA_FPS \
    > "$LOG_DIR/usb_camera.log" 2>&1 &
USB_CAMERA_PID=$!
PIDS+=($USB_CAMERA_PID)
echo "  ✓ Started (PID: $USB_CAMERA_PID)"
sleep 2

# 3. Launch Leader
echo "→ Launching Teleoperation Leader..."
"$SCRIPT_DIR/run_teleoperate_ros2.sh" \
    --mode=leader \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=leader \
    --topic=/lerobot/leader/joint_states \
    --rate=50 \
    > "$LOG_DIR/leader.log" 2>&1 &
LEADER_PID=$!
PIDS+=($LEADER_PID)
echo "  ✓ Started (PID: $LEADER_PID)"
sleep 2

# 4. Launch Follower
echo "→ Launching Teleoperation Follower..."
"$SCRIPT_DIR/run_teleoperate_ros2.sh" \
    --mode=follower \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower \
    --topic=/lerobot/leader/joint_states \
    --rate=50 \
    > "$LOG_DIR/follower.log" 2>&1 &
FOLLOWER_PID=$!
PIDS+=($FOLLOWER_PID)
echo "  ✓ Started (PID: $FOLLOWER_PID)"

# Save PIDs to file
echo "${PIDS[@]}" > "$PID_FILE"

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║              All Components Launched!                      ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
echo "Running components:"
echo "  ✓ RealSense Camera           (PID: $REALSENSE_PID)"
echo "  ✓ RealSense Color Compressed (PID: $REALSENSE_COLOR_COMPRESSED_PID)"
echo "  ✓ RealSense Depth Compressed (PID: $REALSENSE_DEPTH_COMPRESSED_PID)"
echo "  ✓ USB Camera                 (PID: $USB_CAMERA_PID) - Raw + Compressed"
echo "  ✓ Leader                     (PID: $LEADER_PID)"
echo "  ✓ Follower                   (PID: $FOLLOWER_PID)"
echo ""
echo "Log files:"
echo "  • RealSense (raw):            $LOG_DIR/realsense.log"
echo "  • RealSense (color compress): $LOG_DIR/realsense_color_compressed.log"
echo "  • RealSense (depth compress): $LOG_DIR/realsense_depth_compressed.log"
echo "  • USB Camera:                 $LOG_DIR/usb_camera.log"
echo "  • Leader:                     $LOG_DIR/leader.log"
echo "  • Follower:                   $LOG_DIR/follower.log"
echo ""
echo "To view logs in real-time:"
echo "  tail -f $LOG_DIR/realsense.log"
echo "  tail -f $LOG_DIR/realsense_color_compressed.log"
echo "  tail -f $LOG_DIR/realsense_depth_compressed.log"
echo "  tail -f $LOG_DIR/usb_camera.log"
echo "  tail -f $LOG_DIR/leader.log"
echo "  tail -f $LOG_DIR/follower.log"
echo ""
echo "To stop all components:"
echo "  Press Ctrl+C"
echo ""
echo "Monitor ROS2 topics:"
echo "  ros2 topic list"
echo "  ros2 topic hz /camera/image_raw"
echo "  ros2 topic hz /camera/image_raw/compressed"
echo "  ros2 topic hz /camera/depth/image_rect_raw/compressed"
echo "  ros2 topic hz /lerobot/leader/joint_states"
echo ""

# Wait for all processes
echo "All components running. Press Ctrl+C to stop..."
echo ""

wait
