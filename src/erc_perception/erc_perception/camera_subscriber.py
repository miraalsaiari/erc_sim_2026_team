import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


CAMERA_TOPIC = (
    '/head_front_camera/head_front_camera/color/image_raw'
)


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.bridge = CvBridge()
        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image,
            CAMERA_TOPIC,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Waiting for images on {CAMERA_TOPIC}'
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