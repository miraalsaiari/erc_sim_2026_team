from datetime import datetime, timezone
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


CAMERA_TOPIC = (
    '/head_front_camera/head_front_camera/color/image_raw'
)

DEFAULT_OUTPUT_DIRECTORY = '/opt/erc_ws/erc_images'


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.declare_parameter(
            'output_directory',
            DEFAULT_OUTPUT_DIRECTORY,
        )

        output_directory = self.get_parameter(
            'output_directory'
        ).value

        self.output_directory = Path(output_directory)
        self.output_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.bridge = CvBridge()
        self.frame_count = 0
        self.image_saved = False

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Waiting for images on {CAMERA_TOPIC}'
        )

        self.get_logger().info(
            f'Images will be saved to {self.output_directory}'
        )

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        except Exception as error:
            self.get_logger().error(
                f'Failed to convert camera image: {error}'
            )
            return

        self.frame_count += 1

        if self.frame_count % 30 == 0:
            height, width = image.shape[:2]

            self.get_logger().info(
                f'Received frame {self.frame_count}: '
                f'{width}x{height} pixels'
            )

        if not self.image_saved:
            self.save_test_image(image)

    def save_test_image(self, image):
        timestamp = datetime.now(timezone.utc)
        timestamp_text = timestamp.strftime(
            '%Y-%m-%d %H:%M:%S UTC'
        )
        filename_timestamp = timestamp.strftime(
            '%Y%m%d_%H%M%S_%f'
        )

        annotated_image = image.copy()

        cv2.putText(
            annotated_image,
            f'Captured: {timestamp_text}',
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        image_path = (
            self.output_directory
            / f'camera_test_{filename_timestamp}.jpg'
        )

        saved = cv2.imwrite(
            str(image_path),
            annotated_image,
        )

        if saved:
            self.image_saved = True
            self.get_logger().info(
                f'Saved live camera image: {image_path}'
            )
        else:
            self.get_logger().error(
                f'Failed to save image: {image_path}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = CameraSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()