"""Extract visible shelf labels and coloured books from BGR camera images."""

import cv2
import numpy as np

from erc_perception.digit_recognizer import DigitRecognizer
from erc_perception.selection import book_from_candidates, column_from_labels


COLOUR_RANGES = {
    'red': [((0, 100, 80), (12, 255, 255)),
            ((165, 100, 80), (179, 255, 255))],
    'green': [((36, 70, 60), (89, 255, 255))],
    'yellow': [((18, 100, 100), (35, 255, 255))],
    'blue': [((90, 80, 60), (135, 255, 255))],
}


class ShelfDetector:
    """Find a labelled target without consulting simulator object states."""

    def __init__(self, template_directory):
        """Load the simulator's static digit appearance templates."""
        self.reader = DigitRecognizer(template_directory)

    def find_labels(self, image):
        """Search overlapping horizontal bands and merge repeat detections."""
        height, width = image.shape[:2]
        detections = []
        # Larger bands preserve complete digits when a short image would
        # place the same glyph across the edges of every smaller band.
        band_heights = {min(height, max(40, int(height * scale)))
                        for scale in (0.24, 0.48, 1.0)}
        for band_height in sorted(band_heights):
            step = max(1, band_height // 2)
            tops = list(range(0, height - band_height + 1, step))
            tops.append(height - band_height)
            for top in sorted(set(tops)):
                detections.extend(self.reader.detect(
                    image, (0, top, width, band_height)))
        merged = []
        ambiguous = []
        for item in sorted(detections, key=lambda d: -d['score']):
            nearby = [old for old in merged
                      if abs(old['centre_x'] - item['centre_x']) < 3
                      and abs(old['centre_y'] - item['centre_y']) < 3]
            if nearby:
                if nearby[0]['digit'] != item['digit']:
                    ambiguous.append(nearby[0])
                continue
            merged.append(item)
        return [item for item in merged if item not in ambiguous]

    @staticmethod
    def find_books(image):
        """Keep colour components; geometry is checked during selection."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        books = []
        for colour, ranges in COLOUR_RANGES.items():
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            for lower, upper in ranges:
                mask |= cv2.inRange(hsv, np.array(lower, dtype=np.uint8),
                                    np.array(upper, dtype=np.uint8))
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                x, y, width, height = cv2.boundingRect(contour)
                area = cv2.contourArea(contour)
                if area < 3 or height < 3:
                    continue
                if area / (width * height) < 0.35:
                    continue
                books.append({
                    'colour': colour,
                    'bbox': (x, y, width, height),
                    'centre_x': x + width / 2,
                    'centre_y': y + height / 2,
                    'area': area,
                })
        return books

    def detect(self, image, target_column, target_colour):
        """Return (column, book); either can be None when unconfirmed."""
        height, width = image.shape[:2]
        column = column_from_labels(
            self.find_labels(image), target_column, (width, height))
        if column is None:
            return None, None
        book = book_from_candidates(
            self.find_books(image), column, target_colour, (width, height))
        return column, book
