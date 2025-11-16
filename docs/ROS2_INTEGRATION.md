# ROS2 Integration for LeRobot

This document describes the ROS2 integration for LeRobot, including calibration and teleoperation using ROS2.

## Overview

LeRobot now provides ROS2-based tools for:
1. **Calibration** - ROS2 service-based calibration for robots and teleoperators
2. **Teleoperation** - ROS2 topic-based teleoperation with calibration support

## Prerequisites

### Install ROS2 Dependencies

```bash
# Install LeRobot with ROS2 support
pip install -e ".[ros2]"

# OR install ROS2 Jazzy system packages (Ubuntu 24.04)
sudo apt update
sudo apt install ros-jazzy-desktop python3-colcon-common-extensions

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
```

### Verify ROS2 Installation

```bash
# Test ROS2 is available
conda run -n lerobot bash -c "source /opt/ros/jazzy/setup.bash && python -c 'import rclpy; print(\"ROS2 import successful!\")"
```

## ROS2 Calibration

### Start Calibration Server

The calibration server provides ROS2 services for calibrating robots and teleoperators.

#### For Teleoperator (Leader Arm)

```bash
lerobot-calibrate-ros2 \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --node_name=lerobot_leader_calibration
```

#### For Robot (Follower Arm)

```bash
lerobot-calibrate-ros2 \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.id=black \
    --node_name=lerobot_follower_calibration
```

### Use Calibration Services

Once the calibration server is running, you can use ROS2 services:

```bash
# Start calibration process
ros2 service call /lerobot_leader_calibration/start_calibration std_srvs/srv/Trigger

# Check calibration status
ros2 service call /lerobot_leader_calibration/get_status std_srvs/srv/Trigger

# Save calibration
ros2 service call /lerobot_leader_calibration/save_calibration std_srvs/srv/Trigger

# Monitor calibration status (in another terminal)
ros2 topic echo /lerobot_leader_calibration/calibration_status
```

### Available Services

Each calibration server provides:
- `/{node_name}/start_calibration` - Trigger calibration process
- `/{node_name}/get_status` - Get current calibration status
- `/{node_name}/save_calibration` - Save calibration to file

And publishes status to:
- `/{node_name}/calibration_status` - Current calibration status (std_msgs/String)

## ROS2 Teleoperation

### Architecture

The ROS2 teleoperation system uses a leader-follower architecture:

1. **Leader Node** - Reads from teleoperator and publishes joint states
2. **Follower Node** - Subscribes to leader joint states and controls robot

Both nodes use calibration data automatically when devices are connected.

### Start Leader Node (Teleoperator)

```bash
lerobot-teleoperate-ros2 \
    --mode=leader \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --topic=/lerobot/leader/joint_states \
    --rate=50
```

### Start Follower Node (Robot)

```bash
lerobot-teleoperate-ros2 \
    --mode=follower \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.id=black \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --topic=/lerobot/leader/joint_states \
    --rate=50
```

### Monitor Joint States

```bash
# Monitor leader joint states
ros2 topic echo /lerobot/leader/joint_states

# Monitor follower joint states
ros2 topic echo /lerobot/follower/joint_states

# List all topics
ros2 topic list

# Get topic info
ros2 topic info /lerobot/leader/joint_states
```

### Configuration Parameters

- `--mode` - Either "leader" or "follower"
- `--topic` - ROS2 topic for joint states (default: `/lerobot/leader/joint_states`)
- `--rate` - Publishing/control rate in Hz (default: 50)
- `--teleop.*` - Teleoperator configuration (for leader mode)
- `--robot.*` - Robot configuration (for follower mode)

## Complete Workflow Example

### 1. Calibrate Devices

**Terminal 1 - Calibrate Leader:**
```bash
lerobot-calibrate-ros2 \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue
```

**Terminal 2 - Trigger Calibration:**
```bash
# Wait for server to start, then:
ros2 service call /lerobot_calibration_server/start_calibration std_srvs/srv/Trigger

# After calibration is complete, save it:
ros2 service call /lerobot_calibration_server/save_calibration std_srvs/srv/Trigger
```

**Terminal 3 - Calibrate Follower:**
```bash
lerobot-calibrate-ros2 \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.id=black
```

**Terminal 4 - Trigger Calibration:**
```bash
ros2 service call /lerobot_calibration_server/start_calibration std_srvs/srv/Trigger
ros2 service call /lerobot_calibration_server/save_calibration std_srvs/srv/Trigger
```

### 2. Run Teleoperation

After calibration, close the calibration servers (Ctrl+C) and start teleoperation:

**Terminal 1 - Start Leader:**
```bash
lerobot-teleoperate-ros2 \
    --mode=leader \
    --teleop.type=so101_leader \
    --teleop.port=/dev/tty.usbmodem58760431551 \
    --teleop.id=blue \
    --rate=50
```

**Terminal 2 - Start Follower:**
```bash
lerobot-teleoperate-ros2 \
    --mode=follower \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.id=black \
    --rate=50
```

**Terminal 3 - Monitor (Optional):**
```bash
# Watch joint states
ros2 topic echo /lerobot/leader/joint_states

# Or use rqt for visualization
rqt
```

### 3. Verify Calibration is Being Used

The teleoperation nodes automatically load and use calibration data:
- Calibration files are stored in `~/.cache/lerobot/calibration/`
- When connecting, devices check if calibration matches what's in the motors
- If calibration doesn't match, devices will prompt for recalibration

## Advantages of ROS2 Integration

1. **Distributed System** - Leader and follower can run on different machines
2. **Service-Based Calibration** - Remote calibration via ROS2 services
3. **Topic-Based Communication** - Standard ROS2 JointState messages
4. **Monitoring Tools** - Use standard ROS2 tools (rviz, rqt, ros2 topic, etc.)
5. **Integration with ROS2 Ecosystem** - Easy integration with other ROS2 packages
6. **Network Transparency** - Works across network with ROS2 DDS

## Troubleshooting

### ROS2 Not Found

```bash
# Make sure ROS2 is installed
pip install -e ".[ros2]"

# OR source ROS2 environment
source /opt/ros/jazzy/setup.bash
```

### Calibration File Not Found

Calibration files are stored in `~/.cache/lerobot/calibration/{device_id}.json`

If calibration file is missing, the device will prompt for calibration on connect.

### Connection Issues

- Check device ports: `lerobot-find-port`
- Verify devices are connected: `ls /dev/tty*`
- Check ROS2 topics: `ros2 topic list`
- Check ROS2 nodes: `ros2 node list`

### Rate Issues

If teleoperation is slow:
- Reduce `--rate` (try 30Hz or 20Hz)
- Check network latency with `ros2 topic hz /lerobot/leader/joint_states`
- Monitor CPU usage

## Advanced Usage

### Custom Topics

```bash
# Leader publishes to custom topic
lerobot-teleoperate-ros2 \
    --mode=leader \
    --topic=/my_robot/leader_commands \
    ...

# Follower subscribes to same custom topic
lerobot-teleoperate-ros2 \
    --mode=follower \
    --topic=/my_robot/leader_commands \
    ...
```

### Multiple Robot Arms

You can run multiple leader-follower pairs with different topics:

```bash
# Left arm
lerobot-teleoperate-ros2 --mode=leader --topic=/left_arm/commands ...
lerobot-teleoperate-ros2 --mode=follower --topic=/left_arm/commands ...

# Right arm
lerobot-teleoperate-ros2 --mode=leader --topic=/right_arm/commands ...
lerobot-teleoperate-ros2 --mode=follower --topic=/right_arm/commands ...
```

### Integration with Existing ROS2 System

The joint states are published as standard `sensor_msgs/JointState` messages and can be:
- Recorded with `ros2 bag record`
- Visualized with `rviz2` or `rqt`
- Integrated with MoveIt2 or other ROS2 motion planning
- Used with ROS2 control frameworks

## Related Documentation

- [CLAUDE.md](CLAUDE.md) - Main LeRobot documentation
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [LeRobot SO-101 Guide](https://huggingface.co/docs/lerobot/robots/so100)
