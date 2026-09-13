"""Publish confirmed shelf labels/rows and save annotated camera evidence."""

from datetime import datetime, timezone
import json
from pathlib import Path
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from erc_perception.selection import COLOURS, StableResult
from erc_perception.shelf_detector import ShelfDetector


CAMERA_TOPIC = '/head_front_camera/head_front_camera/color/image_raw'
COLUMN_TOPIC = '/erc/shelf_column_identification'
ROW_TOPIC = '/erc/shelf_row_identification'


class CameraSubscriber(Node):
    """Process RGB frames, withholding ambiguous or incomplete results."""

    def __init__(self):
        """Configure inputs, appearance templates, publishers and evidence."""
        super().__init__('camera_subscriber')
        defaults = {
            'shelf_column_number': 1,
            'book_colour': 'red',
            'output_directory': '/opt/erc_ws/erc_images',
            'template_directory': '',
            'camera_topic': CAMERA_TOPIC,
            'confirmation_frames': 5,
            'processing_rate': 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.target_column = self.get_parameter('shelf_column_number').value
        self.target_colour = self.get_parameter('book_colour').value.lower()
        count = self.get_parameter('confirmation_frames').value
        rate = self.get_parameter('processing_rate').value
        if self.target_column not in range(1, 6):
            raise ValueError('shelf_column_number must be 1 to 5')
        if self.target_colour not in COLOURS:
            raise ValueError('book_colour must be red, green, yellow or blue')
        if count < 1 or not 0.5 <= rate <= 30:
            raise ValueError('Invalid confirmation_frames or processing_rate')
        self.period = 1.0 / rate
        self.output_directory = Path(
            self.get_parameter('output_directory').value).expanduser()
        self.output_directory.mkdir(parents=True, exist_ok=True)
        templates = self.get_parameter('template_directory').value
        if not templates:
            templates = (Path(get_package_share_directory('erc_description'))
                         / 'models' / 'number_marker' / 'textures')
        self.detector = ShelfDetector(templates)
        self.bridge = CvBridge()
        self.column_gate = StableResult(count, max(1.5, self.period * 3))
        self.row_gate = StableResult(count, max(1.5, self.period * 3))
        self.last_processed = float('-inf')
        self.last_stamp = None
        self.last_log = float('-inf')
        self.last_publish = {'column': float('-inf'), 'row': float('-inf')}
        self.last_key = {'column': None, 'row': None}
        self.saved = set()
        self.column_publisher = self.create_publisher(Int32, COLUMN_TOPIC, 10)
        self.row_publisher = self.create_publisher(Int32, ROW_TOPIC, 10)
        self.subscription = self.create_subscription(
            Image, self.get_parameter('camera_topic').value,
            self.image_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Target: column {self.target_column}, '
            f'colour {self.target_colour}')
        self.get_logger().info(
            'Reading visible digit labels; waiting for camera images. '
            f'Evidence directory: {self.output_directory}')

    def publish_value(self, kind, value, key, now):
        """Publish new confirmed answers immediately, then at most 1 Hz."""
        if key == self.last_key[kind] and now - self.last_publish[kind] < 1:
            return
        publisher = (self.column_publisher if kind == 'column'
                     else self.row_publisher)
        publisher.publish(Int32(data=int(value)))
        if key != self.last_key[kind]:
            self.get_logger().info(f'Confirmed {kind}: {value}')
        self.last_key[kind] = key
        self.last_publish[kind] = now

    def save_evidence(self, image, column, book, message):
        """Save an annotated frame and source timestamp once per result."""
        row = book['row_number'] if book else None
        key = (column['digit'], self.target_colour, row)
        if key in self.saved:
            return
        timestamp = datetime.now(timezone.utc)
        stamp_text = timestamp.strftime('%Y-%m-%d %H:%M:%S UTC')
        annotated = image.copy()
        x, y, width, height = column['bbox']
        cv2.rectangle(annotated, (x, y), (x + width, y + height),
                      (0, 255, 0), 2)
        lx, ly, lw, lh = column['label']['bbox']
        cv2.rectangle(annotated, (lx, ly), (lx + lw, ly + lh),
                      (255, 255, 0), 1)
        if book:
            bx, by, bw, bh = book['bbox']
            cv2.rectangle(annotated, (bx, by), (bx + bw, by + bh),
                          (0, 255, 255), 2)
        caption = f"Column {column['digit']} | {self.target_colour}"
        caption += f' | row {row}' if book else ' | row unconfirmed'
        for text, baseline in ((stamp_text, 20), (caption, 42)):
            cv2.putText(annotated, text, (8, baseline),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3)
            cv2.putText(annotated, text, (8, baseline),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        stem = (f"camera_{timestamp.strftime('%Y%m%d_%H%M%S_%f')}_"
                f"column{column['digit']}_{self.target_colour}_"
                f"row{row if row else 'pending'}_{uuid4().hex[:8]}")
        path = self.output_directory / f'{stem}.jpg'
        metadata = {
            'captured_at_utc': timestamp.isoformat(),
            'source': 'ROS camera topic',
            'camera_topic': self.get_parameter('camera_topic').value,
            'frame_id': message.header.frame_id,
            'stamp_sec': int(message.header.stamp.sec),
            'stamp_nanosec': int(message.header.stamp.nanosec),
            'column_label': column['digit'],
            'visible_labels': [item['digit'] for item in column['labels']],
            'book_colour': self.target_colour,
            'row_number': row,
            'column_box_xywh': column['bbox'],
            'book_box_xywh': book['bbox'] if book else None,
        }
        try:
            if not cv2.imwrite(str(path), annotated):
                raise OSError('OpenCV could not write the image')
            path.with_suffix('.json').write_text(
                json.dumps(metadata, indent=2) + '\n', encoding='utf-8')
        except (OSError, cv2.error) as error:
            self.get_logger().error(f'Could not save evidence: {error}')
            return
        self.saved.add(key)
        self.get_logger().info(f'Saved camera evidence: {path}')

    def image_callback(self, message):
        """Confirm consistent answers using distinct camera frames."""
        now = time.monotonic()
        if now - self.last_processed < self.period:
            return
        stamp = (int(message.header.stamp.sec),
                 int(message.header.stamp.nanosec))
        if stamp == self.last_stamp:
            return
        if self.last_stamp is not None and stamp < self.last_stamp:
            self.column_gate.reset()
            self.row_gate.reset()
        self.last_stamp = stamp
        self.last_processed = now
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            column, book = self.detector.detect(
                image, self.target_column, self.target_colour)
        except Exception as error:
            self.column_gate.reset()
            self.row_gate.reset()
            if now - self.last_log >= 5:
                self.get_logger().error(f'Image processing failed: {error}')
                self.last_log = now
            return
        column_key = (tuple(item['digit'] for item in column['labels'])
                      if column else None)
        row_key = ((column_key, self.target_colour, book['row_number'])
                   if book else None)
        column_ready = self.column_gate.update(column_key, now)
        row_ready = self.row_gate.update(row_key, now)
        if column_ready:
            self.publish_value('column', column['digit'], column_key, now)
            self.save_evidence(image, column, None, message)
        if column_ready and row_ready:
            self.publish_value('row', book['row_number'], row_key, now)
            self.save_evidence(image, column, book, message)
        elif now - self.last_log >= 5:
            state = ('Need clear target and neighbouring labels'
                     if column is None else
                     'Need all four books in the target column')
            if book is not None:
                state = 'Confirming consecutive camera frames'
            self.get_logger().info(state)
            self.last_log = now


def main(args=None):
    """Start the perception node and clean up on shutdown."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraSubscriber()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
