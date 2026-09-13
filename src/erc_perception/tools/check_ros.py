"""Check ROS wiring with synthetic images on isolated topics."""

from pathlib import Path
import sys
import tempfile
import time
from uuid import uuid4

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from erc_perception.camera_subscriber import CameraSubscriber

# Source-package fixtures are never saved as trial evidence.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'test'))
from test_vision import scene, template_directory  # noqa: E402


def main():
    """Check no-output-on-blank, both Int32 answers, and evidence saving."""
    prefix = '/erc_perception_check_' + uuid4().hex[:10]
    images_topic = prefix + '/images'
    column_topic = prefix + '/column'
    row_topic = prefix + '/row'
    with tempfile.TemporaryDirectory(prefix='erc-synthetic-check-') as output:
        rclpy.init(args=[
            '--ros-args',
            '-p', 'shelf_column_number:=5', '-p', 'book_colour:=blue',
            '-p', f'camera_topic:={images_topic}',
            '-p', f'output_directory:={output}',
            '-r', '/erc/shelf_column_identification:=' + column_topic,
            '-r', '/erc/shelf_row_identification:=' + row_topic,
            '-r', '__node:=perception_check_' + uuid4().hex[:8],
        ])
        node = None
        monitor = None
        executor = SingleThreadedExecutor()
        try:
            node = CameraSubscriber()
            monitor = Node('perception_check_monitor',
                           use_global_arguments=False)
            publisher = monitor.create_publisher(
                Image, images_topic, qos_profile_sensor_data)
            columns = []
            rows = []
            monitor.create_subscription(
                Int32, column_topic, lambda msg: columns.append(msg.data), 10)
            monitor.create_subscription(
                Int32, row_topic, lambda msg: rows.append(msg.data), 10)
            executor.add_node(node)
            executor.add_node(monitor)
            bridge = CvBridge()
            counter = 0

            def feed(image, duration):
                """Send distinct frames while servicing all test nodes."""
                nonlocal counter
                deadline = time.monotonic() + duration
                next_frame = 0.0
                while time.monotonic() < deadline:
                    now = time.monotonic()
                    if now >= next_frame:
                        counter += 1
                        message = bridge.cv2_to_imgmsg(image, encoding='bgr8')
                        message.header.stamp.sec = counter
                        message.header.frame_id = 'synthetic_test_camera'
                        publisher.publish(message)
                        next_frame = now + 0.25
                    executor.spin_once(timeout_sec=0.03)

            blank = np.full((460, 1000, 3), 240, dtype=np.uint8)
            feed(blank, 2.0)
            assert not columns and not rows, 'Published on blank input'
            image = scene(template_directory())
            feed(image, 6.0)
            assert columns and set(columns) == {5}, f'Wrong columns: {columns}'
            assert rows and set(rows) == {4}, f'Wrong rows: {rows}'
            assert list(Path(output).glob('*_row4_*.jpg')), 'No book image'
            assert list(Path(output).glob('*_row4_*.json')), 'No metadata'
            # Drain queued answers, then check for cached publication.
            feed(blank, 1.0)
            before = (len(columns), len(rows))
            feed(blank, 1.5)
            assert (len(columns), len(rows)) == before, 'Stale publication'
            print('ROS synthetic check PASSED: column 5, row 4, saved image.')
            print('Synthetic files were temporary; live testing is required.')
        finally:
            executor.shutdown()
            if node is not None:
                node.destroy_node()
            if monitor is not None:
                monitor.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
