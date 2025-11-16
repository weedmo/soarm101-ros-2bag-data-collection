# LeRobot ROS2 Integration

Complete ROS2 (Jazzy) integration for LeRobot, enabling teleoperation, data collection, and robotic manipulation with full ROS2 ecosystem support.

[![ROS2 Version](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://docs.ros.org/en/jazzy/)
[![Python Version](https://img.shields.io/badge/Python-3.10+-green)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-Apache%202.0-yellow)](LICENSE)

---

## 📋 Table of Contents

- [Features](#-features)
- [Quick Start](#-quick-start)
- [Installation](#-installation)
- [Usage](#-usage)
  - [Calibration](#1-calibration)
  - [Teleoperation](#2-teleoperation)
  - [Data Collection](#3-data-collection)
  - [Complete System](#4-complete-system-launch)
- [Directory Structure](#-directory-structure)
- [Documentation](#-documentation)
- [Troubleshooting](#-troubleshooting)
- [Contributing](#-contributing)

---

## 🚀 Features

### Core Features
- **ROS2 Jazzy Integration**: Full support for ROS2 Jazzy Jalisco
- **Real-time Teleoperation**: Leader-follower control with joint state publishing
- **Multi-Camera Support**: RealSense D435i (head + wrist) with compressed image topics
- **Data Collection**: Advanced ROS2 bag recording with metadata
- **Complete Launch System**: One-command system startup with all components

### Advanced Features
- **Metadata Collection**: Structured metadata schema (v1.0.0) for datasets
- **GUI-based Recorder**: Interactive bag recorder with live camera preview
- **Topic Selection**: Choose which topics to record dynamically
- **Automatic Compression**: Compressed image republishing for efficient storage
- **Calibration Tools**: Automated calibration for leader and follower arms

---

## ⚡ Quick Start

### Prerequisites
```bash
# ROS2 Jazzy installed
source /opt/ros/jazzy/setup.bash

# Conda environment with LeRobot
conda activate lerobot_ros2
```

### Launch Everything
```bash
# Navigate to LeRobot directory
cd /path/to/lerobot

# Launch all components (cameras + teleoperation)
./ros2/scripts/launch/launch_all.sh
```

This launches:
- ✅ RealSense Camera (PointCloud + Compressed Images)
- ✅ USB Camera Publisher (Raw + Compressed)
- ✅ Teleoperation (Leader + Follower)

### Start Recording Data
```bash
# In a new terminal
./ros2/scripts/recording/run_bag_recorder_with_metadata.sh
```

**That's it!** You're ready to collect robot manipulation data with full metadata tracking.

---

## 📦 Installation

### 1. Install ROS2 Jazzy

```bash
# Ubuntu 24.04 (Noble)
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-jazzy-realsense2-camera ros-jazzy-image-transport-plugins
```

### 2. Install LeRobot

```bash
# Create conda environment
conda create -y -n lerobot_ros2 python=3.10
conda activate lerobot_ros2

# Install LeRobot
cd /path/to/lerobot
pip install -e .
```

### 3. Install ROS2 Dependencies

```bash
# Install ROS2 Python bindings
pip install rclpy sensor_msgs std_msgs geometry_msgs

# Install GUI dependencies (for bag recorder GUI)
pip install PySide6 opencv-python numpy

# Install RealSense SDK (if using RealSense cameras)
sudo apt install -y ros-jazzy-realsense2-camera

# Install image transport plugins
sudo apt install -y ros-jazzy-image-transport-plugins
```

### 4. Setup Serial Ports (for robots)

```bash
# Add user to dialout group for serial port access
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect
# Or temporarily grant permissions:
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

### 5. Build LeRobot ROS2 Package (Optional)

```bash
# Build the ROS2 package
cd /path/to/lerobot/src/lerobot_ros2
colcon build
source install/setup.bash
```

---

## 🎯 Usage

### 1. Calibration

Calibrate your robot arms before first use:

```bash
# Calibrate leader arm
./ros2/scripts/setup/calibrate_leader.sh

# Calibrate follower arm
./ros2/scripts/setup/calibrate_follower.sh

# Or use the unified calibration script
./ros2/scripts/setup/run_calibrate_ros2.sh
```

### 2. Teleoperation

#### Option A: Quick Launch (Recommended)
```bash
./ros2/scripts/teleop/run_teleop_all.sh
```

#### Option B: Manual Launch
```bash
# Terminal 1: Leader + Follower Teleoperation
./ros2/scripts/teleop/run_teleoperate_ros2.sh

# Terminal 2: Verify joint states are publishing
source /opt/ros/jazzy/setup.bash
ros2 topic echo /lerobot/follower/joint_states
```

#### Published Topics
- `/lerobot/leader/joint_states` - Leader arm joint states
- `/lerobot/follower/joint_states` - Follower arm joint states

### 3. Data Collection

#### Option A: GUI-based Recorder (Recommended)
```bash
./ros2/scripts/recording/run_bag_recorder_with_metadata.sh
```

**Features:**
- 📹 Live camera preview (head + wrist)
- 📝 Metadata collection (task, tags, operator, context)
- ✅ Topic selection (choose which topics to record)
- 💾 Automatic metadata.json generation (schema v1.0.0)
- ⌨️ Keyboard shortcuts (A/S/D for Start/Save/Cancel)

**Metadata Schema:**
```json
{
  "schema_version": "1.0.0",
  "collection_uuid": "unique-id",
  "task_info": {
    "task_id": "pick_and_place_red_cube_001",
    "task_name": "Pick and Place (Red Cube)",
    "instruction": "Pick the red cube and place it in the blue bowl",
    "task_type": "pick_and_place",
    "tags": ["manipulation", "object_transfer", "red_cube"]
  },
  "collection_context": {
    "operator": "junmo",
    "robot_model": "SO-ARM100",
    "location": "alchera_lab_01",
    "environment_notes": "Natural lighting, clear workspace"
  },
  "timestamps": {
    "start_utc": "2025-01-16T10:30:00Z",
    "end_utc": "2025-01-16T10:32:30Z",
    "duration_sec": 150.0
  },
  "data_provenance": {
    "rosbag_filename": "20250116_103000_pick_and_place",
    "rosbag_size_mb": 245.6
  },
  "hardware_config": {
    "cameras": [...],
    "gripper": {...}
  },
  "recorded_topics": [
    {
      "name": "/camera/color/image_raw/compressed",
      "message_type": "sensor_msgs/msg/CompressedImage"
    },
    ...
  ],
  "custom_fields": {
    "is_success": true,
    "failure_reason": null
  }
}
```

The `metadata.json` file is saved **inside the rosbag directory**, making it easy to keep data and metadata together.

#### Option B: Command-line Recorder
```bash
# Record specific topics
ros2 bag record -o my_dataset \
  /camera/color/image_raw/compressed \
  /camera/depth/color/points \
  /camera/image_raw/compressed \
  /lerobot/follower/joint_states

# Record all topics
ros2 bag record -a
```

### 4. Complete System Launch

Launch all components with a single command:

```bash
./ros2/scripts/launch/launch_all.sh
```

**Launches:**
1. **RealSense Camera** - PointCloud + Compressed Images
2. **USB Camera** - Raw + Compressed
3. **Teleoperation** - Leader + Follower

**Configuration (Environment Variables):**
```bash
# Customize before running
export USB_CAMERA_INDEX=0
export USB_CAMERA_TOPIC="/camera/image_raw"
export USB_CAMERA_WIDTH=640
export USB_CAMERA_HEIGHT=480
export USB_CAMERA_FPS=30
export REALSENSE_LAUNCH="rs_pointcloud_launch.py"

./ros2/scripts/launch/launch_all.sh
```

**Stop All Components:**
```bash
./ros2/scripts/launch/stop_all.sh
```

---

## 📁 Directory Structure

```
ros2/
├── README.md                          # This file
├── docs/                              # Documentation
│   ├── ROS2_INTEGRATION.md           # Detailed integration guide
│   ├── ROS2_QUICK_START.md           # Quick start tutorial
│   ├── BAG_RECORDER_GUI_GUIDE.md     # GUI recorder documentation
│   ├── METADATA_COLLECTION_GUIDE.md  # Metadata schema & best practices
│   └── LAUNCH_GUIDE.md               # Launch system documentation
├── scripts/                           # Executable scripts
│   ├── setup/                        # Calibration scripts
│   │   ├── calibrate_leader.sh
│   │   ├── calibrate_follower.sh
│   │   └── run_calibrate_ros2.sh
│   ├── teleop/                       # Teleoperation scripts
│   │   ├── run_teleoperate_ros2.sh
│   │   └── run_teleop_all.sh
│   ├── camera/                       # Camera scripts
│   │   └── run_camera_ros2.sh
│   ├── recording/                    # Data collection scripts
│   │   ├── run_bag_recorder_gui.sh
│   │   └── run_bag_recorder_with_metadata.sh
│   └── launch/                       # System launch scripts
│       ├── launch_all.sh             # Launch all components
│       ├── launch_all_background.sh  # Background mode
│       └── stop_all.sh               # Stop all components
├── tools/                             # Python tools
│   └── ros2_bag_recorder_gui_v2.py   # GUI bag recorder
└── examples/                          # Usage examples (coming soon)
```

**Related Directories:**
```
src/
├── lerobot/
│   ├── scripts/
│   │   ├── lerobot_calibrate_ros2.py      # ROS2 calibration script
│   │   └── lerobot_teleoperate_ros2.py    # ROS2 teleoperation script
│   └── utils/
│       └── ros2_utils.py                   # ROS2 utility functions
└── lerobot_ros2/                           # ROS2 package (colcon buildable)
    ├── package.xml
    ├── setup.py
    ├── lerobot_ros2/
    │   └── so101_teleop_node.py           # SO-101 teleoperation node
    ├── launch/
    │   └── so101_teleop.launch.py         # Launch file
    └── config/
        └── so101_params.yaml              # Parameters
```

---

## 📚 Documentation

### Quick Guides
- [**Quick Start Guide**](docs/ROS2_QUICK_START.md) - Get started in 5 minutes
- [**Launch System Guide**](docs/LAUNCH_GUIDE.md) - System launch documentation

### Detailed Guides
- [**ROS2 Integration Guide**](docs/ROS2_INTEGRATION.md) - Complete integration details
- [**Bag Recorder GUI Guide**](docs/BAG_RECORDER_GUI_GUIDE.md) - GUI recorder tutorial
- [**Metadata Collection Guide**](docs/METADATA_COLLECTION_GUIDE.md) - Metadata schema & best practices

### API Documentation
- [**LeRobot ROS2 Package**](../src/lerobot_ros2/README.md) - ROS2 package documentation
- [**ROS2 Utils**](../src/lerobot/utils/ros2_utils.py) - Utility functions

---

## 🔧 Troubleshooting

### Common Issues

#### 1. Serial Port Permission Denied
```bash
# Solution 1: Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and log back in

# Solution 2: Temporary fix
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

#### 2. ROS2 Topics Not Publishing
```bash
# Check if nodes are running
ros2 node list

# Check if topics exist
ros2 topic list

# Check topic data
ros2 topic echo /lerobot/follower/joint_states

# Check topic publishing rate
ros2 topic hz /lerobot/follower/joint_states
```

#### 3. Camera Not Found
```bash
# List connected cameras
ls /dev/video*

# Check RealSense cameras
rs-enumerate-devices

# Test camera
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
```

#### 4. Python Module Not Found
```bash
# Ensure conda environment is activated
conda activate lerobot_ros2

# Reinstall dependencies
pip install rclpy sensor_msgs std_msgs PySide6 opencv-python
```

#### 5. Motors Not Responding
```bash
# Check motor power
# Verify correct serial ports
lerobot-find-port

# Test motor connection
python -c "from lerobot.motors.feetech import FeetechMotorsBus; bus = FeetechMotorsBus('/dev/ttyACM0'); print(bus.read('present_position', 1))"
```

---

## 🐛 Known Issues

- **Background launch mode**: `launch_all_background.sh` may not work on all desktop environments
- **GUI display**: Requires X11 or Wayland display server
- **RealSense on USB 2.0**: May have reduced performance; use USB 3.0+

---

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines
- Follow ROS2 naming conventions
- Add documentation for new features
- Test on ROS2 Jazzy before submitting
- Update README and relevant docs

---

## 📄 License

Apache 2.0 - See [LICENSE](../LICENSE) for details.

---

## 🙏 Acknowledgments

- [**LeRobot**](https://github.com/huggingface/lerobot) by Hugging Face
- [**ROS2 Jazzy**](https://docs.ros.org/en/jazzy/) by Open Robotics
- [**RealSense SDK**](https://github.com/IntelRealSense/librealsense) by Intel

---

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/huggingface/lerobot/issues)
- **Discussions**: [GitHub Discussions](https://github.com/huggingface/lerobot/discussions)
- **Discord**: [LeRobot Discord](https://discord.gg/s3KuuzsPFb)
- **Documentation**: [LeRobot Docs](https://huggingface.co/docs/lerobot)

---

## 🗺️ Roadmap

- [x] ROS2 Jazzy integration
- [x] Teleoperation with joint states
- [x] Multi-camera support
- [x] Metadata collection system
- [x] GUI-based bag recorder
- [ ] URDF/XACRO robot models
- [ ] MoveIt2 integration
- [ ] Gazebo simulation
- [ ] Multi-robot support
- [ ] Real-time visualization (RViz2)
- [ ] Behavior tree integration (BehaviorTree.CPP)

---

**Made with ❤️ by the LeRobot community**
