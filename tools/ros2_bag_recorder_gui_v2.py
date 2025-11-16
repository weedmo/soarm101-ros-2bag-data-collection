#!/usr/bin/env python3
"""
ROS 2 Bag Recorder GUI with Metadata Collection

Records ROS 2 topics to bag files with live camera preview and metadata generation.
"""

import sys
import os
import subprocess
import shutil
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import JointState

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QStatusBar, QFrame, QLineEdit, QTextEdit,
    QComboBox, QGroupBox, QFormLayout, QScrollArea, QSplitter,
    QCheckBox, QListWidget, QListWidgetItem
)
from PySide6.QtCore import QTimer, Qt, Signal, QThread
from PySide6.QtGui import QImage, QPixmap, QFont


# Metadata Schema Version
METADATA_SCHEMA_VERSION = "1.0.0"


class ROSSpinThread(QThread):
    """Thread for spinning ROS 2 node without blocking GUI."""

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.running = True

    def run(self):
        while self.running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def stop(self):
        self.running = False


class BagRecorderNode(Node):
    """ROS 2 node for subscribing to camera topics."""

    def __init__(self):
        super().__init__('bag_recorder_gui_node')

        # Subscribe to camera topics
        self.head_sub = self.create_subscription(
            CompressedImage,
            '/camera/camera/color/image_raw/compressed',
            self.head_image_callback,
            10
        )

        self.wrist_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.wrist_image_callback,
            10
        )

        self.get_logger().info('Bag Recorder Node initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /camera/camera/color/image_raw/compressed (Head)')
        self.get_logger().info('  - /camera/image_raw/compressed (Wrist)')

    def head_image_callback(self, msg: CompressedImage):
        """Callback for head camera images."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                if hasattr(self, 'gui'):
                    self.gui.update_head_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error decoding head image: {e}')

    def wrist_image_callback(self, msg: CompressedImage):
        """Callback for wrist camera images."""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if cv_image is not None:
                if hasattr(self, 'gui'):
                    self.gui.update_wrist_image(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error decoding wrist image: {e}')


class BagRecorderGUI(QMainWindow):
    """Main GUI window for ROS 2 bag recorder with metadata collection."""

    def __init__(self, node: BagRecorderNode):
        super().__init__()
        self.node = node
        self.node.gui = self

        # Recording state
        self.bag_process = None
        self.bag_file_name = None
        self.collection_uuid = None
        self.recording_start_time = None
        self.recording_end_time = None
        self.timer_seconds = 0

        # Setup UI
        self.setup_ui()

        # Timer for recording duration
        self.recording_timer = QTimer()
        self.recording_timer.timeout.connect(self.update_timer)

        # ROS spin thread
        self.ros_thread = ROSSpinThread(self.node)
        self.ros_thread.start()

    def setup_ui(self):
        """Setup the user interface."""
        self.setWindowTitle('ROS 2 Bag Recorder with Metadata')
        self.setMinimumSize(1600, 900)

        # Central widget with splitter
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QHBoxLayout(central_widget)

        # Splitter for resizable panels
        splitter = QSplitter(Qt.Horizontal)

        # === Left Panel: Camera Views and Controls ===
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        # Video Display Area
        video_layout = QHBoxLayout()

        # Head camera view
        head_frame = self.create_camera_frame('Head View', 480, 360)
        self.head_label = head_frame[1]
        video_layout.addWidget(head_frame[0])

        # Wrist camera view
        wrist_frame = self.create_camera_frame('Wrist View', 480, 360)
        self.wrist_label = wrist_frame[1]
        video_layout.addWidget(wrist_frame[0])

        left_layout.addLayout(video_layout)

        # Control Panel
        control_frame = QFrame()
        control_frame.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        control_layout = QVBoxLayout(control_frame)

        # Timer display
        timer_layout = QHBoxLayout()
        timer_label_text = QLabel('Recording Time:')
        timer_label_text.setFont(QFont('Arial', 12))
        self.timer_label = QLabel('00:00')
        self.timer_label.setFont(QFont('Arial', 20, QFont.Bold))
        self.timer_label.setAlignment(Qt.AlignCenter)
        timer_layout.addStretch()
        timer_layout.addWidget(timer_label_text)
        timer_layout.addWidget(self.timer_label)
        timer_layout.addStretch()
        control_layout.addLayout(timer_layout)

        # Buttons
        button_layout = QHBoxLayout()

        self.start_button = QPushButton('시작 (Start) [A]')
        self.start_button.setMinimumHeight(50)
        self.start_button.setFont(QFont('Arial', 12))
        self.start_button.clicked.connect(self.start_recording)
        self.start_button.setStyleSheet('background-color: #4CAF50; color: white;')

        self.save_button = QPushButton('저장 (Save) [S]')
        self.save_button.setMinimumHeight(50)
        self.save_button.setFont(QFont('Arial', 12))
        self.save_button.clicked.connect(self.save_recording)
        self.save_button.setEnabled(False)
        self.save_button.setStyleSheet('background-color: #2196F3; color: white;')

        self.cancel_button = QPushButton('취소 (Cancel) [D]')
        self.cancel_button.setMinimumHeight(50)
        self.cancel_button.setFont(QFont('Arial', 12))
        self.cancel_button.clicked.connect(self.cancel_recording)
        self.cancel_button.setEnabled(False)
        self.cancel_button.setStyleSheet('background-color: #f44336; color: white;')

        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.save_button)
        button_layout.addWidget(self.cancel_button)

        control_layout.addLayout(button_layout)

        left_layout.addWidget(control_frame)

        # === Right Panel: Metadata Input ===
        right_panel = self.create_metadata_panel()

        # Add panels to splitter
        splitter.addWidget(left_panel)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 2)  # Left panel 2x
        splitter.setStretchFactor(1, 1)  # Right panel 1x

        main_layout.addWidget(splitter)

        # Status Bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage('대기 중 (Ready)')

    def create_camera_frame(self, title: str, width: int, height: int):
        """Create a camera view frame with title."""
        frame = QFrame()
        frame.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)

        layout = QVBoxLayout(frame)

        # Title
        title_label = QLabel(title)
        title_label.setFont(QFont('Arial', 14, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # Image label
        image_label = QLabel()
        image_label.setMinimumSize(width, height)
        image_label.setAlignment(Qt.AlignCenter)
        image_label.setStyleSheet('background-color: black; color: white; font-size: 16px;')
        image_label.setText('신호 없음 (No Signal)')
        layout.addWidget(image_label)

        return (frame, image_label)

    def create_metadata_panel(self):
        """Create metadata input panel."""
        # Scroll area for metadata inputs
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)

        panel = QWidget()
        layout = QVBoxLayout(panel)

        # Title
        title_label = QLabel('📋 Metadata Collection')
        title_label.setFont(QFont('Arial', 16, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)

        # === Task Information ===
        task_group = QGroupBox('Task Information')
        task_layout = QFormLayout()

        self.task_id_input = QLineEdit()
        self.task_id_input.setPlaceholderText('e.g., pick_and_place_red_cube_001')
        task_layout.addRow('Task ID:', self.task_id_input)

        self.task_name_input = QLineEdit()
        self.task_name_input.setPlaceholderText('e.g., Pick and Place (Red Cube)')
        task_layout.addRow('Task Name:', self.task_name_input)

        self.task_type_combo = QComboBox()
        self.task_type_combo.addItems([
            'pick_and_place',
            'grasp',
            'push',
            'pull',
            'pour',
            'stack',
            'unstack',
            'open_drawer',
            'close_drawer',
            'other'
        ])
        task_layout.addRow('Task Type:', self.task_type_combo)

        self.instruction_input = QTextEdit()
        self.instruction_input.setPlaceholderText('Enter natural language instruction\ne.g., "Place the red cube into the blue bowl."')
        self.instruction_input.setMaximumHeight(80)
        task_layout.addRow('Instruction:', self.instruction_input)

        self.tags_input = QLineEdit()
        self.tags_input.setPlaceholderText('e.g., manipulation, object_transfer, red_cube (comma-separated)')
        task_layout.addRow('Tags:', self.tags_input)

        task_group.setLayout(task_layout)
        layout.addWidget(task_group)

        # === Collection Context ===
        context_group = QGroupBox('Collection Context')
        context_layout = QFormLayout()

        self.operator_input = QLineEdit()
        self.operator_input.setPlaceholderText('e.g., junmo')
        context_layout.addRow('Operator:', self.operator_input)

        self.robot_model_combo = QComboBox()
        self.robot_model_combo.addItems([
            'RB-Y1',
            'SO-ARM100',
            'SO101',
            'UR5e',
            'Franka Emika',
            'Other'
        ])
        context_layout.addRow('Robot Model:', self.robot_model_combo)

        self.location_input = QLineEdit()
        self.location_input.setPlaceholderText('e.g., alchera_lab_01')
        context_layout.addRow('Location:', self.location_input)

        self.environment_notes_input = QTextEdit()
        self.environment_notes_input.setPlaceholderText('Optional: Environment conditions\ne.g., "Sunny day, strong natural light"')
        self.environment_notes_input.setMaximumHeight(60)
        context_layout.addRow('Environment Notes:', self.environment_notes_input)

        context_group.setLayout(context_layout)
        layout.addWidget(context_group)

        # === Hardware Config ===
        hardware_group = QGroupBox('Hardware Configuration')
        hardware_layout = QFormLayout()

        self.camera_head_name = QLineEdit()
        self.camera_head_name.setText('realsense_d435i_external')
        hardware_layout.addRow('Head Camera Name:', self.camera_head_name)

        self.camera_wrist_name = QLineEdit()
        self.camera_wrist_name.setText('realsense_d435i_wrist')
        hardware_layout.addRow('Wrist Camera Name:', self.camera_wrist_name)

        self.gripper_model_combo = QComboBox()
        self.gripper_model_combo.addItems([
            'rg2',
            'robotiq_2f_85',
            'schunk_gripper',
            'parallel_gripper',
            'other'
        ])
        hardware_layout.addRow('Gripper Model:', self.gripper_model_combo)

        hardware_group.setLayout(hardware_layout)
        layout.addWidget(hardware_group)

        # === Recording Topics ===
        topics_group = QGroupBox('Recording Topics')
        topics_layout = QVBoxLayout()

        # Refresh button
        refresh_button = QPushButton('🔄 Refresh Topics')
        refresh_button.clicked.connect(self.refresh_topics)
        topics_layout.addWidget(refresh_button)

        # Topics list with checkboxes
        self.topics_list = QListWidget()
        self.topics_list.setMaximumHeight(150)
        topics_layout.addWidget(self.topics_list)

        topics_group.setLayout(topics_layout)
        layout.addWidget(topics_group)

        # Initialize topics
        self.refresh_topics()

        # === Custom Fields ===
        custom_group = QGroupBox('Custom Fields (Optional)')
        custom_layout = QFormLayout()

        self.success_combo = QComboBox()
        self.success_combo.addItems(['true', 'false'])
        custom_layout.addRow('Task Success:', self.success_combo)

        self.failure_reason_input = QLineEdit()
        self.failure_reason_input.setPlaceholderText('If failed, reason...')
        custom_layout.addRow('Failure Reason:', self.failure_reason_input)

        custom_group.setLayout(custom_layout)
        layout.addWidget(custom_group)

        layout.addStretch()

        scroll.setWidget(panel)
        return scroll

    def update_head_image(self, cv_image: np.ndarray):
        """Update head camera view."""
        self.update_image_label(self.head_label, cv_image)

    def update_wrist_image(self, cv_image: np.ndarray):
        """Update wrist camera view."""
        self.update_image_label(self.wrist_label, cv_image)

    def update_image_label(self, label: QLabel, cv_image: np.ndarray):
        """Convert OpenCV image to QPixmap and update label."""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Get label size
            label_size = label.size()

            # Resize image to fit label while maintaining aspect ratio
            h, w = rgb_image.shape[:2]
            label_w, label_h = label_size.width(), label_size.height()

            scale = min(label_w / w, label_h / h)
            new_w, new_h = int(w * scale), int(h * scale)

            resized_image = cv2.resize(rgb_image, (new_w, new_h))

            # Convert to QImage
            height, width, channel = resized_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(resized_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

            # Convert to QPixmap and display
            pixmap = QPixmap.fromImage(q_image)
            label.setPixmap(pixmap)
        except Exception as e:
            self.node.get_logger().error(f'Error updating image label: {e}')

    def refresh_topics(self):
        """Refresh the list of available ROS2 topics."""
        try:
            # Get list of topics
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                topics = [line.strip() for line in result.stdout.split('\n') if line.strip()]

                # Clear existing list
                self.topics_list.clear()

                # Default topics to check
                default_topics = [
                    '/camera/color/image_raw/compressed',
                    '/camera/depth/color/points',
                    '/camera/image_raw/compressed',
                    '/lerobot/follower/joint_states'
                ]

                # Add topics with checkboxes
                for topic in sorted(topics):
                    item = QListWidgetItem(topic)
                    item.setFlags(item.flags() | Qt.ItemIsUserCheckable)

                    # Check default topics
                    if topic in default_topics:
                        item.setCheckState(Qt.Checked)
                    else:
                        item.setCheckState(Qt.Unchecked)

                    self.topics_list.addItem(item)

                self.node.get_logger().info(f'Refreshed topics: {len(topics)} topics found')
            else:
                self.node.get_logger().error(f'Failed to get topics: {result.stderr}')

        except Exception as e:
            self.node.get_logger().error(f'Error refreshing topics: {e}')

    def get_selected_topics(self):
        """Get list of selected topics."""
        selected = []
        for i in range(self.topics_list.count()):
            item = self.topics_list.item(i)
            if item.checkState() == Qt.Checked:
                selected.append(item.text())
        return selected

    def get_topic_type(self, topic_name):
        """Get the message type of a topic."""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'info', topic_name, '-t'],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                # Output format: "sensor_msgs/msg/CompressedImage"
                return result.stdout.strip()
            else:
                return "unknown"
        except Exception as e:
            self.node.get_logger().error(f'Error getting topic type for {topic_name}: {e}')
            return "unknown"

    def start_recording(self):
        """Start recording ROS 2 bag."""
        try:
            # Get selected topics
            topics = self.get_selected_topics()

            if not topics:
                self.status_bar.showMessage('에러: 녹화할 토픽을 선택하세요 (Error: No topics selected)')
                self.node.get_logger().error('No topics selected for recording')
                return

            # Generate collection UUID
            self.collection_uuid = str(uuid.uuid4())

            # Generate unique bag file name
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            task_name_abbr = self.task_name_input.text().replace(' ', '_').lower()[:30]
            if not task_name_abbr:
                task_name_abbr = 'recording'

            self.bag_file_name = f'data/{timestamp}_{task_name_abbr}'

            # Record start time
            self.recording_start_time = datetime.now(timezone.utc)

            # Build ros2 bag record command
            cmd = ['ros2', 'bag', 'record', '-o', self.bag_file_name] + topics

            # Start recording process
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                cwd=os.path.expanduser('~/lerobot')
            )

            # Update UI
            self.start_button.setEnabled(False)
            self.save_button.setEnabled(True)
            self.cancel_button.setEnabled(True)

            # Start timer
            self.timer_seconds = 0
            self.recording_timer.start(1000)

            # Update status
            self.status_bar.showMessage(f'녹화 중... (Recording {len(topics)} topics to {self.bag_file_name})')

            self.node.get_logger().info(f'Started recording to {self.bag_file_name}')
            self.node.get_logger().info(f'Collection UUID: {self.collection_uuid}')
            self.node.get_logger().info(f'Recording {len(topics)} topics: {", ".join(topics)}')

        except Exception as e:
            self.status_bar.showMessage(f'Error starting recording: {e}')
            self.node.get_logger().error(f'Error starting recording: {e}')

    def save_recording(self):
        """Save recording by stopping the bag process and generating metadata."""
        try:
            # Stop timer
            self.recording_timer.stop()

            # Record end time
            self.recording_end_time = datetime.now(timezone.utc)

            # Send SIGINT to bag process (graceful shutdown)
            if self.bag_process:
                self.bag_process.send_signal(subprocess.signal.SIGINT)
                self.bag_process.wait(timeout=5)

                self.node.get_logger().info(f'Recording saved to {self.bag_file_name}')

            # Generate metadata
            self.generate_metadata()

            # Reset UI
            self.reset_ui()

            # Update status
            self.status_bar.showMessage(f'저장 완료 (Saved: {self.bag_file_name})')

        except Exception as e:
            self.status_bar.showMessage(f'Error saving recording: {e}')
            self.node.get_logger().error(f'Error saving recording: {e}')

    def generate_metadata(self):
        """Generate metadata JSON file."""
        try:
            # Calculate duration
            duration_sec = (self.recording_end_time - self.recording_start_time).total_seconds()

            # Get bag file info
            bag_dir = Path.home() / 'lerobot' / self.bag_file_name
            bag_size_mb = 0
            if bag_dir.exists():
                bag_size_mb = sum(f.stat().st_size for f in bag_dir.rglob('*') if f.is_file()) / (1024 * 1024)

            # Parse tags (comma-separated)
            tags_text = self.tags_input.text().strip()
            tags = [tag.strip() for tag in tags_text.split(',') if tag.strip()] if tags_text else []

            # Get recorded topics with their types
            selected_topics = self.get_selected_topics()
            recorded_topics_info = []
            for topic in selected_topics:
                topic_type = self.get_topic_type(topic)
                recorded_topics_info.append({
                    "name": topic,
                    "message_type": topic_type
                })

            # Construct metadata
            metadata = {
                "schema_version": METADATA_SCHEMA_VERSION,
                "collection_uuid": self.collection_uuid,
                "task_info": {
                    "task_id": self.task_id_input.text(),
                    "task_name": self.task_name_input.text(),
                    "instruction": self.instruction_input.toPlainText(),
                    "task_type": self.task_type_combo.currentText(),
                    "tags": tags
                },
                "collection_context": {
                    "operator": self.operator_input.text(),
                    "robot_model": self.robot_model_combo.currentText(),
                    "location": self.location_input.text(),
                    "environment_notes": self.environment_notes_input.toPlainText()
                },
                "timestamps": {
                    "start_utc": self.recording_start_time.isoformat(),
                    "end_utc": self.recording_end_time.isoformat(),
                    "duration_sec": round(duration_sec, 2)
                },
                "data_provenance": {
                    "rosbag_filename": self.bag_file_name.split('/')[-1],
                    "rosbag_size_mb": round(bag_size_mb, 2)
                },
                "hardware_config": {
                    "cameras": [
                        {
                            "cam_name": self.camera_head_name.text(),
                            "type": "depth_camera",
                            "position": "external"
                        },
                        {
                            "cam_name": self.camera_wrist_name.text(),
                            "type": "depth_camera",
                            "position": "wrist"
                        }
                    ],
                    "gripper": {
                        "model": self.gripper_model_combo.currentText(),
                        "type": "2_finger"
                    }
                },
                "recorded_topics": recorded_topics_info,
                "custom_fields": {
                    "is_success": self.success_combo.currentText() == 'true',
                    "failure_reason": self.failure_reason_input.text() if self.failure_reason_input.text() else None
                }
            }

            # Save to JSON file inside the rosbag directory
            json_filename = bag_dir / "metadata.json"
            with open(json_filename, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, ensure_ascii=False, indent=4)

            self.node.get_logger().info(f'Metadata saved to {json_filename}')
            self.status_bar.showMessage(f'메타데이터 저장 완료 (Metadata saved: {json_filename.name})')

        except Exception as e:
            self.node.get_logger().error(f'Error generating metadata: {e}')
            self.status_bar.showMessage(f'메타데이터 저장 실패 (Metadata save failed): {e}')

    def cancel_recording(self):
        """Cancel recording and delete bag files."""
        try:
            # Stop timer
            self.recording_timer.stop()

            # Terminate bag process
            if self.bag_process:
                self.bag_process.terminate()
                self.bag_process.wait(timeout=5)

            # Delete bag file directory
            bag_path = Path.home() / 'lerobot' / self.bag_file_name
            if bag_path.exists():
                shutil.rmtree(bag_path)
                self.node.get_logger().info(f'Deleted bag file: {bag_path}')

            # Reset UI
            self.reset_ui()

            # Update status
            self.status_bar.showMessage('녹화 취소됨 (Recording cancelled)')

        except Exception as e:
            self.status_bar.showMessage(f'Error cancelling recording: {e}')
            self.node.get_logger().error(f'Error cancelling recording: {e}')

    def reset_ui(self):
        """Reset UI to initial state."""
        self.start_button.setEnabled(True)
        self.save_button.setEnabled(False)
        self.cancel_button.setEnabled(False)

        self.timer_seconds = 0
        self.timer_label.setText('00:00')

        self.bag_process = None
        self.bag_file_name = None
        self.collection_uuid = None
        self.recording_start_time = None
        self.recording_end_time = None

    def update_timer(self):
        """Update recording timer display."""
        self.timer_seconds += 1
        minutes = self.timer_seconds // 60
        seconds = self.timer_seconds % 60
        self.timer_label.setText(f'{minutes:02d}:{seconds:02d}')

    def keyPressEvent(self, event):
        """Handle keyboard shortcuts."""
        key = event.key()

        # A key - Start recording
        if key == Qt.Key_A:
            if self.start_button.isEnabled():
                self.start_recording()
        # S key - Save recording
        elif key == Qt.Key_S:
            if self.save_button.isEnabled():
                self.save_recording()
        # D key - Cancel recording
        elif key == Qt.Key_D:
            if self.cancel_button.isEnabled():
                self.cancel_recording()
        else:
            # Pass other keys to parent
            super().keyPressEvent(event)

    def closeEvent(self, event):
        """Handle window close event."""
        # If recording is in progress, cancel it
        if self.bag_process and self.bag_process.poll() is None:
            self.cancel_recording()

        # Stop ROS thread
        self.ros_thread.stop()
        self.ros_thread.wait()

        event.accept()


def main():
    """Main entry point."""
    # Initialize ROS 2
    rclpy.init()

    # Create Qt application
    app = QApplication(sys.argv)

    # Create ROS node
    node = BagRecorderNode()

    # Create and show GUI
    window = BagRecorderGUI(node)
    window.show()

    # Run application
    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
