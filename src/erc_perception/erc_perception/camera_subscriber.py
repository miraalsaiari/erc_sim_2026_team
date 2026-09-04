from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32


CAMERA_TOPIC = (
    '/head_front_camera/head_front_camera/color/image_raw'
)
COLUMN_TOPIC = '/erc/shelf_column_identification'
ROW_TOPIC = '/erc/shelf_row_identification'
DEFAULT_OUTPUT_DIRECTORY = '/opt/erc_ws/erc_images'

COLOUR_RANGES = {
    'red': [
        ((0, 100, 80), (12, 255, 255)),
        ((165, 100, 80), (179, 255, 255)),
    ],
    'green': [
        ((35, 70, 60), (90, 255, 255)),
    ],
    'yellow': [
        ((18, 100, 100), (35, 255, 255)),
    ],
    'blue': [
        ((90, 80, 60), (135, 255, 255)),
    ],
}

DRAW_COLOURS = {
    'red': (0, 0, 255),
    'green': (0, 255, 0),
    'yellow': (0, 255, 255),
    'blue': (255, 0, 0),
}


class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.declare_parameter(
            'output_directory',
            DEFAULT_OUTPUT_DIRECTORY,
        )
        self.declare_parameter('shelf_column_number', 1)
        self.declare_parameter('book_colour', 'red')

        output_directory = self.get_parameter(
            'output_directory'
        ).value
        self.target_column = int(
            self.get_parameter('shelf_column_number').value
        )
        self.target_colour = str(
            self.get_parameter('book_colour').value
        ).lower()

        if self.target_column not in range(1, 6):
            raise ValueError(
                'shelf_column_number must be between 1 and 5'
            )

        if self.target_colour not in COLOUR_RANGES:
            raise ValueError(
                'book_colour must be red, green, yellow, or blue'
            )

        self.output_directory = Path(output_directory)
        self.output_directory.mkdir(
            parents=True,
            exist_ok=True,
        )

        self.bridge = CvBridge()
        self.frame_count = 0
        self.result_saved = False
        self.success_count = 0

        self.column_publisher = self.create_publisher(
            Int32,
            COLUMN_TOPIC,
            10,
        )
        self.row_publisher = self.create_publisher(
            Int32,
            ROW_TOPIC,
            10,
        )

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
            'Target: column '
            f'{self.target_column}, colour {self.target_colour}'
        )
        self.get_logger().info(
            f'Results will be saved to {self.output_directory}'
        )

    @staticmethod
    def make_mask(hsv_image, ranges):
        mask = np.zeros(
            hsv_image.shape[:2],
            dtype=np.uint8,
        )

        for lower, upper in ranges:
            mask = cv2.bitwise_or(
                mask,
                cv2.inRange(
                    hsv_image,
                    np.array(lower, dtype=np.uint8),
                    np.array(upper, dtype=np.uint8),
                ),
            )

        kernel = np.ones((3, 3), dtype=np.uint8)
        return cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
        )

    @staticmethod
    def find_books(mask):
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        books = []

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < 3:
                continue

            x, y, width, height = cv2.boundingRect(contour)

            if height < 2 or width < 1:
                continue

            books.append({
                'x': x,
                'y': y,
                'width': width,
                'height': height,
                'centre_x': x + width / 2.0,
                'centre_y': y + height / 2.0,
                'area': area,
            })

        return books

    @staticmethod
    def cluster_positions(values, expected_count):
        if len(values) < expected_count:
            return []

        points = np.float32(values).reshape(-1, 1)

        criteria = (
            cv2.TERM_CRITERIA_EPS
            + cv2.TERM_CRITERIA_MAX_ITER,
            100,
            0.2,
        )

        _, _, centres = cv2.kmeans(
            points,
            expected_count,
            None,
            criteria,
            10,
            cv2.KMEANS_PP_CENTERS,
        )

        return sorted(float(value) for value in centres.flatten())

    @staticmethod
    def nearest_index(value, centres):
        return min(
            range(len(centres)),
            key=lambda index: abs(centres[index] - value),
        )

    def detect_shelf(self, image):
        hsv_image = cv2.cvtColor(
            image,
            cv2.COLOR_BGR2HSV,
        )

        detected_books = []

        for colour_name, ranges in COLOUR_RANGES.items():
            mask = self.make_mask(hsv_image, ranges)

            for book in self.find_books(mask):
                book['colour'] = colour_name
                detected_books.append(book)

        if len(detected_books) < 8:
            return None

        x_centres = self.cluster_positions(
            [book['centre_x'] for book in detected_books],
            5,
        )
        y_centres = self.cluster_positions(
            [book['centre_y'] for book in detected_books],
            4,
        )

        if len(x_centres) != 5 or len(y_centres) != 4:
            return None

        for book in detected_books:
            book['column_index'] = self.nearest_index(
                book['centre_x'],
                x_centres,
            )
            book['row_number'] = (
                self.nearest_index(
                    book['centre_y'],
                    y_centres,
                )
                + 1
            )

        return detected_books, x_centres, y_centres

    def find_target_book(self, detected_books, x_centres):
        # The requested column is published directly. Until OCR is added,
        # its left-to-right position is used to locate the target book.
        column_index = self.target_column - 1
        target_x = x_centres[column_index]

        candidates = [
            book for book in detected_books
            if book['colour'] == self.target_colour
        ]

        if not candidates:
            return None

        return min(
            candidates,
            key=lambda book: abs(
                book['centre_x'] - target_x
            ),
        )

    def annotate_image(
        self,
        image,
        target_book,
        x_centres,
        y_centres,
    ):
        annotated = image.copy()
        image_height, image_width = image.shape[:2]

        column_index = self.target_column - 1
        target_x = x_centres[column_index]

        if column_index == 0:
            left_x = int(
                target_x
                - (x_centres[1] - target_x) / 2
            )
        else:
            left_x = int(
                (x_centres[column_index - 1] + target_x)
                / 2
            )

        if column_index == 4:
            right_x = int(
                target_x
                + (target_x - x_centres[3]) / 2
            )
        else:
            right_x = int(
                (target_x + x_centres[column_index + 1])
                / 2
            )

        left_x = max(0, left_x)
        right_x = min(image_width - 1, right_x)

        top_y = max(0, int(min(y_centres) - 35))
        bottom_y = min(
            image_height - 1,
            int(max(y_centres) + 25),
        )

        cv2.rectangle(
            annotated,
            (left_x, top_y),
            (right_x, bottom_y),
            (255, 255, 0),
            2,
        )

        x = target_book['x']
        y = target_book['y']
        width = target_book['width']
        height = target_book['height']

        cv2.rectangle(
            annotated,
            (x, y),
            (x + width, y + height),
            DRAW_COLOURS[self.target_colour],
            3,
        )

        timestamp = datetime.now(timezone.utc)
        timestamp_text = timestamp.strftime(
            '%Y-%m-%d %H:%M:%S UTC'
        )

        cv2.putText(
            annotated,
            f'Captured: {timestamp_text}',
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        result_text = (
            f'Column {self.target_column} | '
            f'{self.target_colour} book | '
            f'Row {target_book["row_number"]}'
        )

        cv2.putText(
            annotated,
            result_text,
            (10, image_height - 15),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

        return annotated, timestamp

    def publish_result(self, row_number):
        column_message = Int32()
        column_message.data = self.target_column
        self.column_publisher.publish(column_message)

        row_message = Int32()
        row_message.data = row_number
        self.row_publisher.publish(row_message)

        self.get_logger().info(
            f'Published column {self.target_column} '
            f'to {COLUMN_TOPIC}'
        )
        self.get_logger().info(
            f'Published row {row_number} to {ROW_TOPIC}'
        )

    def save_result(self, image, timestamp):
        filename_timestamp = timestamp.strftime(
            '%Y%m%d_%H%M%S_%f'
        )
        image_path = self.output_directory / (
            f'perception_result_{filename_timestamp}.jpg'
        )

        if cv2.imwrite(str(image_path), image):
            self.result_saved = True
            self.get_logger().info(
                f'Saved annotated image: {image_path}'
            )
        else:
            self.get_logger().error(
                f'Failed to save image: {image_path}'
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

        result = self.detect_shelf(image)

        if result is None:
            self.success_count = 0

            if self.frame_count % 30 == 0:
                self.get_logger().info(
                    'Waiting until the shelf is visible...'
                )
            return

        detected_books, x_centres, y_centres = result
        target_book = self.find_target_book(
            detected_books,
            x_centres,
        )

        if target_book is None:
            self.success_count = 0
            return

        self.success_count += 1

        if self.success_count < 5:
            return

        row_number = target_book['row_number']
        self.publish_result(row_number)

        if not self.result_saved:
            annotated, timestamp = self.annotate_image(
                image,
                target_book,
                x_centres,
                y_centres,
            )
            self.save_result(annotated, timestamp)


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
