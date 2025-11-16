# ROS2 Integration Examples

Example scripts demonstrating how to use LeRobot's ROS2 integration.

## Prerequisites

```bash
# Activate conda environment
conda activate lerobot_ros2

# Source ROS2
source /opt/ros/jazzy/setup.bash
```

## Examples

### 1. Joint State Subscriber

Subscribe to and display robot joint states in real-time.

**Usage:**
```bash
# Terminal 1: Start teleoperation
cd /path/to/lerobot
./ros2/scripts/teleop/run_teleop_all.sh

# Terminal 2: Run subscriber
python3 ros2/examples/example_subscriber.py
```

**What it does:**
- Subscribes to `/lerobot/follower/joint_states`
- Prints joint positions in real-time
- Demonstrates basic ROS2 subscriber pattern

**File:** [example_subscriber.py](example_subscriber.py)

---

### 2. Image Subscriber

Subscribe to and display camera images from compressed topics.

**Usage:**
```bash
# Terminal 1: Start cameras
cd /path/to/lerobot
./ros2/scripts/launch/launch_all.sh

# Terminal 2: Run image subscriber
python3 ros2/examples/example_image_subscriber.py
```

**What it does:**
- Subscribes to compressed image topics
- Displays head and wrist camera feeds using OpenCV
- Press 'q' to quit

**File:** [example_image_subscriber.py](example_image_subscriber.py)

---

## Creating Your Own Subscribers

### Basic Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/my/topic',
            self.callback,
            10
        )

    def callback(self, msg):
        # Process message
        self.get_logger().info(f'Received: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Available Topics

You can discover available topics using:

```bash
# List all topics
ros2 topic list

# Show topic info
ros2 topic info /lerobot/follower/joint_states

# Echo topic data
ros2 topic echo /lerobot/follower/joint_states

# Check publishing rate
ros2 topic hz /lerobot/follower/joint_states
```

### Common Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lerobot/leader/joint_states` | `sensor_msgs/JointState` | Leader arm joint states |
| `/lerobot/follower/joint_states` | `sensor_msgs/JointState` | Follower arm joint states |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | Head camera (RealSense) |
| `/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | Wrist camera |
| `/camera/depth/color/points` | `sensor_msgs/PointCloud2` | RealSense point cloud |

---

## Additional Resources

- [ROS2 Python Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [sensor_msgs Documentation](https://docs.ros.org/en/jazzy/p/sensor_msgs/)
- [LeRobot ROS2 Guide](../README.md)
