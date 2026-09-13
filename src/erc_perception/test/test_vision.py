"""Synthetic image regressions; these are not live competition evidence."""

from itertools import permutations
import os
from pathlib import Path
import unittest

import cv2
import numpy as np

from erc_perception.digit_recognizer import DigitRecognizer
from erc_perception.shelf_detector import ShelfDetector


BGR = {'red': (0, 0, 230), 'green': (0, 230, 0),
       'yellow': (0, 230, 230), 'blue': (230, 0, 0)}


def template_directory():
    """Locate static templates through an override or the installed package."""
    override = os.environ.get('ERC_DIGIT_TEMPLATES')
    if override:
        return Path(override)
    from ament_index_python.packages import get_package_share_directory
    return (Path(get_package_share_directory('erc_description'))
            / 'models' / 'number_marker' / 'textures')


def scene(directory, labels=(4, 1, 5, 2, 3),
          colours=('red', 'green', 'yellow', 'blue'), dark=False):
    """Draw a synthetic frontal shelf using real marker artwork."""
    image = np.full((460, 1000, 3), 40 if dark else 240, dtype=np.uint8)
    for index, digit in enumerate(labels):
        centre = 100 + 200 * index
        reference = cv2.imread(str(directory / f'{digit}.png'))
        if reference is None:
            raise RuntimeError(f'Missing reference image for {digit}')
        image[30:90, centre - 30:centre + 30] = cv2.resize(
            reference, (60, 60), interpolation=cv2.INTER_AREA)
        for row, colour in enumerate(colours):
            x = centre + (50 if row % 2 else -50)
            y = 180 + row * 66
            cv2.rectangle(image, (x - 20, y - 16), (x + 20, y + 16),
                          BGR[colour], -1)
    return image


class VisionTests(unittest.TestCase):
    """Check pixel extraction as well as downstream selection."""

    @classmethod
    def setUpClass(cls):
        """Load the same templates used by the perception node."""
        cls.directory = template_directory()
        cls.reader = DigitRecognizer(cls.directory)
        cls.detector = ShelfDetector(cls.directory)

    def test_reference_labels_in_every_order(self):
        """Recognize all 120 permutations without a positional shortcut."""
        for order in permutations(range(1, 6)):
            with self.subTest(order=order):
                image = scene(self.directory, labels=order)
                actual = self.reader.detect(image, (0, 0, 1000, 110))
                self.assertEqual([item['digit'] for item in actual],
                                 list(order))

    def test_full_pipeline_on_light_and_dark_backgrounds(self):
        """Find the blue book in the column labelled 5 on either background."""
        for dark in (False, True):
            with self.subTest(dark=dark):
                image = scene(self.directory, dark=dark)
                column, book = self.detector.detect(image, 5, 'blue')
                self.assertIsNotNone(column)
                self.assertEqual(column['digit'], 5)
                self.assertLess(abs(column['label']['centre_x'] - 500), 5)
                self.assertIsNotNone(book)
                self.assertEqual(book['row_number'], 4)
                self.assertGreater(book['centre_x'], 400)
                self.assertLess(book['centre_x'], 600)

    def test_image_colour_permutations(self):
        """The target row follows its colour through every vertical order."""
        for colours in permutations(BGR):
            with self.subTest(colours=colours):
                column, book = self.detector.detect(
                    scene(self.directory, colours=colours), 5, 'blue')
                self.assertIsNotNone(column)
                self.assertIsNotNone(book)
                self.assertEqual(book['row_number'], colours.index('blue') + 1)

    def test_missing_target_does_not_select_neighbour_pixels(self):
        """Erasing one blue book must not select blue in another column."""
        image = scene(self.directory)
        image[361:396, 529:572] = 240
        column, book = self.detector.detect(image, 5, 'blue')
        self.assertIsNotNone(column)
        self.assertIsNone(book)

    def test_blank_frames_are_rejected(self):
        """Uniform camera images cannot produce a label or a row."""
        for value in (0, 127, 255):
            image = np.full((460, 1000, 3), value, dtype=np.uint8)
            self.assertEqual(self.detector.detect(image, 5, 'blue'),
                             (None, None))

    def test_unreadable_label_prevents_column_result(self):
        """A blank target plate cannot be replaced by its requested number."""
        image = scene(self.directory)
        image[30:90, 470:530] = 255
        self.assertEqual(self.detector.detect(image, 5, 'blue'), (None, None))

    def test_clipped_shelf_does_not_guess_row(self):
        """The same scene cropped above the last row produces no row."""
        image = scene(self.directory)[:330]
        column, book = self.detector.detect(image, 5, 'blue')
        self.assertIsNotNone(column)
        self.assertIsNone(book)

    def test_wrong_roi_and_empty_image(self):
        """Invalid regions and empty frames return no digit candidates."""
        image = scene(self.directory)
        self.assertEqual(self.reader.detect(image, (0, 0, 0, 10)), [])
        self.assertEqual(self.reader.detect(image, (2000, 0, 10, 10)), [])
        self.assertEqual(self.reader.detect(None, (0, 0, 10, 10)), [])


if __name__ == '__main__':
    unittest.main()
