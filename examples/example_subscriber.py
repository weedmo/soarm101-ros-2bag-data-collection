#!/usr/bin/env python3
"""
Example ROS2 Subscriber for Joint States

This example demonstrates how to subscribe to joint states published by
the LeRobot teleoperation system.

Usage:
    python3 example_subscriber.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateListener(Node):
    """Example ROS2 node that listens to joint states."""

    def __init__(self):
        super().__init__('joint_state_listener')

        # Subscribe to follower joint states
        self.subscription = self.create_subscription(
            JointState,
            '/lerobot/follower/joint_states',
            self.listener_callback,
            10
        )

        self.get_logger().info('Joint State Listener started')
        self.get_logger().info('Subscribing to: /lerobot/follower/joint_states')

    def listener_callback(self, msg: JointState):
        """Callback function when joint state message is received."""
        # Create dictionary mapping joint names to positions
        joint_dict = dict(zip(msg.name, msg.position))

        # Log joint positions
        self.get_logger().info('Joint States:')
        for name, position in joint_dict.items():
            self.get_logger().info(f'  {name}: {position:.4f}')
        self.get_logger().info('---')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    listener = JointStateListener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
