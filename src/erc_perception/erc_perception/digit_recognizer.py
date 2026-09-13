"""Recognize shelf digits using the simulator's number-marker images."""

from pathlib import Path

import cv2
import numpy as np


class DigitRecognizer:
    """Match digits 1-5 and reject weak or ambiguous matches."""

    def __init__(
        self,
        template_directory,
        min_score=0.85,
        min_margin=0.06,
    ):
        """Load templates and set conservative matching thresholds."""
        self.min_score = min_score
        self.min_margin = min_margin
        self.templates = {}

        directory = Path(template_directory)

        for digit in range(1, 6):
            path = directory / f'{digit}.png'

            if not path.is_file():
                raise FileNotFoundError(
                    f'Missing digit template: {path}'
                )

            image = cv2.imread(
                str(path),
                cv2.IMREAD_GRAYSCALE,
            )

            if image is None:
                raise ValueError(
                    f'Cannot read digit template: {path}'
                )

            template = self._normalise(
                self._foreground(image)
            )

            if template is None:
                raise ValueError(
                    f'Empty digit template: {path}'
                )

            self.templates[digit] = template

    @staticmethod
    def _foreground(image):
        """Convert dark shapes on a light background into a mask."""
        if image.ndim == 3:
            image = cv2.cvtColor(
                image,
                cv2.COLOR_BGR2GRAY,
            )

        if int(image.max()) - int(image.min()) < 15:
            return np.zeros(
                image.shape,
                dtype=np.uint8,
            )

        _, mask = cv2.threshold(
            image,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
        )

        return mask

    @staticmethod
    def _normalise(mask):
        """Crop and resize a foreground shape for comparison."""
        points = cv2.findNonZero(mask)

        if points is None:
            return None

        x, y, width, height = cv2.boundingRect(points)

        if width < 2 or height < 8:
            return None

        crop = mask[y:y + height, x:x + width]

        resized = cv2.resize(
            crop,
            (24, 40),
            interpolation=cv2.INTER_AREA,
        )

        canvas = np.zeros(
            (48, 32),
            dtype=np.float32,
        )
        canvas[4:44, 4:28] = (
            resized.astype(np.float32) / 255.0
        )

        return cv2.GaussianBlur(canvas, (3, 3), 0)

    def _classify(self, mask):
        """Return (digit, similarity), or None if uncertain."""
        candidate = self._normalise(mask)

        if candidate is None:
            return None

        scores = []

        for digit, template in self.templates.items():
            numerator = 2.0 * float(
                np.sum(candidate * template)
            )
            denominator = float(
                np.sum(candidate ** 2 + template ** 2)
            )
            score = numerator / max(denominator, 1e-9)

            scores.append((score, digit))

        scores.sort(reverse=True)
        best_score, digit = scores[0]
        second_score = scores[1][0]

        if best_score < self.min_score:
            return None

        if best_score - second_score < self.min_margin:
            return None

        return digit, best_score

    def detect(self, image, roi):
        """
        Find digits inside a shelf number-label band.

        roi is (x, y, width, height), measured in image pixels.

        Return dictionaries containing digit, bbox, centre_x,
        centre_y, and score. Similarity scores are not probabilities.

        The caller must restrict the ROI to the number-label band,
        excluding books, floor markings, and unrelated objects.
        """
        if image is None or image.size == 0:
            return []

        image_height, image_width = image.shape[:2]
        x, y, width, height = (
            int(value) for value in roi
        )

        if width <= 0 or height <= 0:
            return []

        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(image_width, x + width)
        y1 = min(image_height, y + height)

        if x1 <= x0 or y1 <= y0:
            return []

        mask = self._foreground(
            image[y0:y1, x0:x1]
        )

        contours, hierarchy = cv2.findContours(
            mask,
            cv2.RETR_CCOMP,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        boxes = []

        for index, contour in enumerate(contours):
            # Keep dark components, including digits inside white plates,
            # while excluding contours describing holes in those components.
            if hierarchy[0][index][3] != -1:
                continue
            bx, by, bw, bh = cv2.boundingRect(contour)

            if bh < 8 or bw < 2:
                continue

            if not 0.1 <= bw / bh <= 1.3:
                continue

            # Reject shapes cut off by the region boundary.
            if bx == 0 or by == 0:
                continue

            if (
                bx + bw >= mask.shape[1]
                or by + bh >= mask.shape[0]
            ):
                continue

            crop = mask[by:by + bh, bx:bx + bw]
            fill = (
                cv2.countNonZero(crop) / float(bw * bh)
            )

            if 0.1 <= fill <= 0.9:
                boxes.append((bx, by, bw, bh))

        # RETR_CCOMP also finds digits surrounded by a dark shelf/wall.
        # A very crowded region probably includes unrelated objects.
        if len(boxes) > 120:
            return []

        detections = []

        for bx, by, bw, bh in boxes:
            result = self._classify(
                mask[by:by + bh, bx:bx + bw]
            )

            if result is None:
                continue

            digit, score = result

            detections.append({
                'digit': digit,
                'bbox': (
                    x0 + bx,
                    y0 + by,
                    bw,
                    bh,
                ),
                'centre_x': x0 + bx + bw / 2.0,
                'centre_y': y0 + by + bh / 2.0,
                'score': score,
            })

        return sorted(
            detections,
            key=lambda item: item['centre_x'],
        )
