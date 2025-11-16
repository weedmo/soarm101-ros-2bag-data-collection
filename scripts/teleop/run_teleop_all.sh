#!/bin/bash
#
# LeRobot ROS2 Teleoperation Launcher
#
# This script starts both the leader and follower nodes for teleoperation.
# The leader node is run in the background, and the follower node runs in the foreground.
# Pressing Ctrl+C will stop both nodes.

echo "Starting Leader and Follower nodes for teleoperation..."

# Function to handle cleanup when the script is interrupted (Ctrl+C)
cleanup() {
    echo -e "\n\nShutting down teleoperation nodes..."
    # Kill the background leader process
    if kill -0 $LEADER_PID 2>/dev/null; then
        kill $LEADER_PID
        echo "Leader node (PID: $LEADER_PID) stopped."
    fi
    # The follower process is in the foreground and will be terminated by Ctrl+C
    wait $LEADER_PID 2>/dev/null
    echo "All nodes stopped. Exiting."
    exit 0
}

# Trap the SIGINT signal (sent by Ctrl+C) and call the cleanup function
trap cleanup SIGINT

# Start the Leader node in the background
./run_teleoperate_ros2.sh \
    --mode=leader \
    --teleop.type=so101_leader \
    --teleop.port=/dev/ttyACM1 \
    --teleop.id=leader \
    --topic=/lerobot/leader/joint_states \
    --rate=50 &

# Save the Process ID (PID) of the background leader process
LEADER_PID=$!

echo "Leader node started in background with PID: $LEADER_PID"
echo "Starting Follower node in foreground..."

# Start the Follower node in the foreground. The script will wait here.
./run_teleoperate_ros2.sh \
    --mode=follower \
    --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 \
    --robot.id=follower \
    --topic=/lerobot/leader/joint_states \
    --rate=50

# Wait for the leader process to finish if the follower exits normally
wait $LEADER_PID