#!/usr/bin/env python3
"""
Example ROS2 Subscriber for Compressed Images

This example demonstrates how to subscribe to compressed camera images
and display them using OpenCV.

Usage:
    python3 example_image_subscriber.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class ImageListener(Node):
    """Example ROS2 node that listens to compressed images."""

    def __init__(self):
        super().__init__('image_listener')

        # Subscribe to head camera compressed image
        self.head_sub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.head_callback,
            10
        )

        # Subscribe to wrist camera compressed image
        self.wrist_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.wrist_callback,
            10
        )

        self.get_logger().info('Image Listener started')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /camera/color/image_raw/compressed (Head Camera)')
        self.get_logger().info('  - /camera/image_raw/compressed (Wrist Camera)')
        self.get_logger().info('Press "q" to quit')

    def head_callback(self, msg: CompressedImage):
        """Callback for head camera images."""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                # Display image
                cv2.imshow('Head Camera', cv_image)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing head image: {e}')

    def wrist_callback(self, msg: CompressedImage):
        """Callback for wrist camera images."""
        try:
            # Decode compressed image
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                # Display image
                cv2.imshow('Wrist Camera', cv_image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    raise KeyboardInterrupt
        except Exception as e:
            self.get_logger().error(f'Error processing wrist image: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    listener = ImageListener()

    try:
        rclpy.spin(listener)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
