# Changelog - LeRobot ROS2 Integration

All notable changes to the ROS2 integration will be documented in this file.

## [1.0.0] - 2025-01-16

### Added - Initial Release

#### Core Features
- **ROS2 Jazzy Integration**: Full support for ROS2 Jazzy Jalisco
- **Teleoperation System**: Leader-follower robot control with joint state publishing
- **Multi-Camera Support**: RealSense D435i and USB camera integration
- **Data Collection**: Advanced ROS2 bag recording with metadata tracking
- **Complete Launch System**: One-command system startup

#### Tools & Scripts
- **GUI Bag Recorder** (`ros2_bag_recorder_gui_v2.py`)
  - Live camera preview (head + wrist)
  - Interactive metadata collection
  - Topic selection
  - Keyboard shortcuts (A/S/D)
  - Metadata schema v1.0.0 with in-bag storage

- **Setup Scripts**
  - `calibrate_leader.sh` - Leader arm calibration
  - `calibrate_follower.sh` - Follower arm calibration
  - `run_calibrate_ros2.sh` - Unified calibration script

- **Teleoperation Scripts**
  - `run_teleoperate_ros2.sh` - ROS2 teleoperation launcher
  - `run_teleop_all.sh` - Complete teleoperation system

- **Camera Scripts**
  - `run_camera_ros2.sh` - USB camera publisher

- **Recording Scripts**
  - `run_bag_recorder_gui.sh` - GUI recorder launcher
  - `run_bag_recorder_with_metadata.sh` - Metadata-enabled recorder

- **Launch Scripts**
  - `launch_all.sh` - Launch complete system (cameras + teleoperation)
  - `launch_all_background.sh` - Background mode launcher
  - `stop_all.sh` - Stop all components

#### Documentation
- **Main Documentation**
  - `README.md` - Complete integration guide
  - `ROS2_INTEGRATION.md` - Detailed integration documentation
  - `ROS2_QUICK_START.md` - Quick start guide

- **Specialized Guides**
  - `BAG_RECORDER_GUI_GUIDE.md` - GUI recorder tutorial
  - `METADATA_COLLECTION_GUIDE.md` - Metadata schema & best practices
  - `LAUNCH_GUIDE.md` - Launch system documentation

#### Examples
- `example_subscriber.py` - Joint state subscriber example
- `example_image_subscriber.py` - Camera image subscriber example
- `examples/README.md` - Examples documentation

#### ROS2 Package
- **lerobot_ros2** - Colcon-buildable ROS2 package
  - SO-101 teleoperation node
  - Launch files
  - Configuration files
  - Full ROS2 package structure

#### Metadata Schema
- **Version 1.0.0** - Structured metadata format
  - Task information (ID, name, type, instruction, tags)
  - Collection context (operator, robot model, location)
  - Timestamps (start, end, duration)
  - Data provenance (filename, size)
  - Hardware configuration (cameras, gripper)
  - Recorded topics (with message types)
  - Custom fields (success, failure reason)
  - **Metadata stored inside rosbag directory** for easy management

#### Directory Structure
```
ros2/
├── README.md
├── CHANGELOG.md
├── docs/               # Documentation
├── examples/           # Example scripts
├── scripts/            # Executable scripts
│   ├── setup/         # Calibration
│   ├── teleop/        # Teleoperation
│   ├── camera/        # Camera
│   ├── recording/     # Data collection
│   └── launch/        # System launch
└── tools/             # GUI tools
```

### Changed
- **Metadata Location**: Metadata JSON now saved **inside** rosbag directory (not outside)
- **Script Paths**: All scripts updated to use relative paths from ros2/ directory
- **File Organization**: Consolidated all ROS2 files into dedicated `ros2/` directory

### Removed
- Old bag recorder GUI (`ros2_bag_recorder_gui.py`) - replaced by v2

---

## Future Plans

### Upcoming Features
- [ ] URDF/XACRO robot models for visualization
- [ ] MoveIt2 integration for motion planning
- [ ] Gazebo simulation support
- [ ] Multi-robot coordination
- [ ] RViz2 real-time visualization
- [ ] BehaviorTree.CPP integration
- [ ] Advanced data processing tools
- [ ] Dataset conversion utilities (ROS2 bag → LeRobot format)
- [ ] Automated testing framework
- [ ] Docker containers for easy deployment

### Known Issues
- Background launch mode may not work on all desktop environments
- GUI requires X11 or Wayland display server
- RealSense performance reduced on USB 2.0 (use USB 3.0+)

---

## Version History

- **1.0.0** (2025-01-16): Initial release with complete ROS2 integration
