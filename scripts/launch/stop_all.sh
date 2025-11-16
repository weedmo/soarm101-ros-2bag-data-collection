#!/bin/bash
#
# Stop all LeRobot components
#

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PID_FILE="$SCRIPT_DIR/.launch_all.pid"

echo "╔════════════════════════════════════════════════════════════╗"
echo "║           Stopping All LeRobot Components                 ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

if [ ! -f "$PID_FILE" ]; then
    echo "No PID file found. Components may not be running."
    echo "Attempting to find and stop ROS2 nodes..."
    echo ""

    # Try to find and kill ROS2 nodes
    source /opt/ros/jazzy/setup.bash 2>/dev/null

    pkill -f "run_camera_ros2.sh"
    pkill -f "run_teleoperate_ros2.sh"
    pkill -f "realsense2_camera"

    echo "Sent kill signals to any running processes."
    exit 0
fi

# Read PIDs from file
PIDS=($(cat "$PID_FILE"))

echo "Found ${#PIDS[@]} processes to stop..."
echo ""

# Stop each process
for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
        echo "→ Stopping process (PID: $pid)..."
        kill "$pid" 2>/dev/null
    else
        echo "→ Process $pid already stopped"
    fi
done

# Wait a bit
sleep 2

# Force kill if still running
for pid in "${PIDS[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
        echo "→ Force stopping process (PID: $pid)..."
        kill -9 "$pid" 2>/dev/null
    fi
done

# Remove PID file
rm -f "$PID_FILE"

echo ""
echo "╔════════════════════════════════════════════════════════════╗"
echo "║              All Components Stopped                        ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""
