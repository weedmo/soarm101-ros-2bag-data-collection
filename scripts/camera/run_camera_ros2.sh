#!/bin/bash
# ROS2 Camera Publisher
# Publishes USB camera feed to ROS2 topic

# Default parameters
CAMERA_INDEX=${1:-0}
TOPIC_NAME=${2:-"/camera/image_raw"}
WIDTH=${3:-640}
HEIGHT=${4:-480}
FPS=${5:-30}

source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot_ros2
source /opt/ros/jazzy/setup.bash

echo "Starting ROS2 Camera Publisher"
echo "Camera Index: $CAMERA_INDEX"
echo "Topic: $TOPIC_NAME"
echo "Resolution: ${WIDTH}x${HEIGHT}"
echo "FPS: $FPS"
echo ""

python3 << PYTHON_SCRIPT
import sys
sys.path.insert(0, '/home/weed/lerobot/src')

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class CameraPublisher(Node):
    def __init__(self, camera_index=$CAMERA_INDEX, topic_name="$TOPIC_NAME",
                 width=$WIDTH, height=$HEIGHT, fps=$FPS):
        super().__init__('camera_publisher')

        # Create publishers for raw and compressed images
        self.publisher = self.create_publisher(Image, topic_name, 10)
        self.compressed_publisher = self.create_publisher(
            CompressedImage, f"{topic_name}/compressed", 10
        )

        # Initialize camera
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            logger.error(f"Failed to open camera {camera_index}")
            raise RuntimeError(f"Cannot open camera {camera_index}")

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # Verify actual settings
        actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))

        logger.info(f"Camera opened successfully")
        logger.info(f"Actual resolution: {actual_width}x{actual_height}")
        logger.info(f"Actual FPS: {actual_fps}")
        logger.info(f"Publishing raw to: {topic_name}")
        logger.info(f"Publishing compressed to: {topic_name}/compressed")

        # Create timer for publishing at specified rate
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.frame_count = 0

    def cv2_to_imgmsg(self, cv_image):
        """Convert OpenCV image to ROS Image message (without cv_bridge)"""
        msg = Image()

        # Convert BGR to RGB (ROS standard)
        if len(cv_image.shape) == 3:
            height, width, channels = cv_image.shape
            if channels == 3:
                # BGR to RGB
                cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                msg.encoding = "rgb8"
                msg.step = width * 3
                msg.data = cv_image_rgb.tobytes()
            else:
                raise ValueError(f"Unsupported number of channels: {channels}")
        else:
            # Grayscale
            height, width = cv_image.shape
            msg.encoding = "mono8"
            msg.step = width
            msg.data = cv_image.tobytes()

        msg.height = height
        msg.width = width
        msg.is_bigendian = 0

        return msg

    def cv2_to_compressed_imgmsg(self, cv_image, quality=90):
        """Convert OpenCV image to ROS CompressedImage message"""
        msg = CompressedImage()
        msg.format = "jpeg"

        # Encode image as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        result, encoded = cv2.imencode('.jpg', cv_image, encode_param)

        if result:
            msg.data = encoded.tobytes()
        else:
            logger.error("Failed to encode image")

        return msg

    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret:
            timestamp = self.get_clock().now().to_msg()

            # Convert to ROS Image message (raw)
            msg = self.cv2_to_imgmsg(frame)
            msg.header.stamp = timestamp
            msg.header.frame_id = "camera_frame"

            # Convert to CompressedImage message
            compressed_msg = self.cv2_to_compressed_imgmsg(frame)
            compressed_msg.header.stamp = timestamp
            compressed_msg.header.frame_id = "camera_frame"

            # Publish both raw and compressed
            self.publisher.publish(msg)
            self.compressed_publisher.publish(compressed_msg)

            self.frame_count += 1
            if self.frame_count % 30 == 0:
                logger.info(f"Published {self.frame_count} frames (raw + compressed)")
        else:
            logger.warning("Failed to read frame from camera")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main():
    rclpy.init()

    try:
        camera_publisher = CameraPublisher()
        logger.info("Camera publisher running. Press Ctrl+C to stop...")
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        logger.info("Shutting down camera publisher...")
    except Exception as e:
        logger.error(f"Error: {e}")
    finally:
        if 'camera_publisher' in locals():
            camera_publisher.destroy_node()
        rclpy.shutdown()
        logger.info("Camera publisher shutdown complete")

if __name__ == '__main__':
    main()
PYTHON_SCRIPT
