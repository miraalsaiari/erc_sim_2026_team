"""Exercise label/row decisions without needing ROS, OpenCV or navigation."""

from itertools import permutations
import unittest

from erc_perception.selection import (
    book_from_candidates, COLOURS, column_from_labels, StableResult,
)


def labels_for(order):
    """Make measured label candidates at fixed shelf-column centres."""
    return [dict(digit=digit, centre_x=100 + index * 100, centre_y=40,
                 bbox=(90 + index * 100, 28, 20, 24), score=0.98)
            for index, digit in enumerate(order)]


def books_for(centre, order=COLOURS, jitter=0):
    """Make one visible book per active row, with horizontal jitter."""
    return [dict(colour=colour, centre_x=centre + (-1) ** row * jitter,
                 centre_y=100 + row * 33,
                 bbox=(centre + (-1) ** row * jitter - 10,
                       92 + row * 33, 20, 16))
            for row, colour in enumerate(order)]


class SelectionTests(unittest.TestCase):
    """Check randomized labels, incomplete shelves and ambiguous colours."""

    def test_every_label_permutation_and_target(self):
        """Select the requested label's physical column in all 120 orders."""
        for order in permutations(range(1, 6)):
            for target in range(1, 6):
                with self.subTest(order=order, target=target):
                    result = column_from_labels(
                        labels_for(order), target, (600, 300))
                    self.assertIsNotNone(result)
                    self.assertEqual(result['digit'], target)
                    self.assertEqual(result['bbox'][0],
                                     50 + 100 * order.index(target))

    def test_every_colour_order_and_horizontal_jitter(self):
        """All colour permutations retain top-to-bottom active rows 1 to 4."""
        column = column_from_labels(labels_for([4, 1, 5, 2, 3]),
                                    5, (600, 300))
        for order in permutations(COLOURS):
            for colour in COLOURS:
                with self.subTest(order=order, colour=colour):
                    result = book_from_candidates(
                        books_for(300, order, 25), column, colour, (600, 300))
                    self.assertIsNotNone(result)
                    self.assertEqual(result['row_number'],
                                     order.index(colour) + 1)

    def test_missing_target_book_cannot_use_neighbour(self):
        """A neighbour's red book never fills a missing target red book."""
        column = column_from_labels(labels_for([4, 1, 5, 2, 3]),
                                    5, (600, 300))
        books = books_for(300)[1:] + books_for(200) + books_for(400)
        self.assertIsNone(book_from_candidates(
            books, column, 'red', (600, 300)))

    def test_missing_other_colour_does_not_shift_rows(self):
        """A visible target cannot be numbered from an incomplete column."""
        column = column_from_labels(labels_for([4, 1, 5, 2, 3]),
                                    5, (600, 300))
        self.assertIsNone(book_from_candidates(
            books_for(300)[:-1], column, 'red', (600, 300)))

    def test_duplicate_book_is_ambiguous(self):
        """Two plausible books of one colour produce no row."""
        column = column_from_labels(labels_for([4, 1, 5, 2, 3]),
                                    5, (600, 300))
        books = books_for(300) + [books_for(320)[0]]
        self.assertIsNone(book_from_candidates(
            books, column, 'red', (600, 300)))

    def test_partial_view_needs_both_target_neighbours(self):
        """A central target can use three labels; an unbounded edge cannot."""
        labels = labels_for([4, 1, 5])
        self.assertIsNotNone(column_from_labels(labels, 1, (600, 300)))
        self.assertIsNone(column_from_labels(labels, 4, (600, 300)))
        self.assertIsNone(column_from_labels(labels[:2], 1, (600, 300)))

    def test_absent_or_duplicate_target_labels(self):
        """No column is invented from absent or duplicate label candidates."""
        self.assertIsNone(column_from_labels([], 1, (600, 300)))
        self.assertIsNone(column_from_labels(
            labels_for([2, 3, 4]), 1, (600, 300)))
        self.assertIsNone(column_from_labels(
            labels_for([1, 1, 1, 1, 1]), 1, (600, 300)))

    def test_missing_intermediate_marker_is_not_a_wide_column(self):
        """Skipped marker positions fail the known marker-to-column scale."""
        labels = labels_for([1, 2, 3, 4, 5])[::2]
        self.assertIsNone(column_from_labels(labels, 3, (600, 300)))
        labels = labels_for([1, 2, 3, 4, 5])
        self.assertIsNone(column_from_labels(
            [labels[0], labels[1], labels[3]], 2, (600, 300)))

    def test_unaligned_labels_are_rejected(self):
        """Unrelated numerals at different heights cannot define a shelf."""
        labels = labels_for([1, 2, 3])
        labels[1]['centre_y'] = 100
        self.assertIsNone(column_from_labels(labels, 2, (600, 300)))

    def test_cropped_column_cannot_produce_row(self):
        """Clipping the lower shelf prevents a guessed active-row number."""
        column = column_from_labels(labels_for([4, 1, 5, 2, 3]),
                                    5, (600, 200))
        self.assertIsNotNone(column)
        self.assertIsNone(book_from_candidates(
            books_for(300), column, 'red', (600, 200)))

    def test_large_red_bin_is_not_a_book(self):
        """An oversized red region does not replace the target book."""
        column = column_from_labels(labels_for([4, 1, 5, 2, 3]),
                                    5, (600, 300))
        bin_candidate = dict(colour='red', bbox=(260, 75, 80, 70),
                             centre_x=300, centre_y=110)
        self.assertIsNone(book_from_candidates(
            books_for(300)[1:] + [bin_candidate], column, 'red', (600, 300)))


class StabilityTests(unittest.TestCase):
    """Check fresh consecutive agreement and reset conditions."""

    def test_five_consecutive_frames_required(self):
        """A result becomes publishable only on the fifth observation."""
        gate = StableResult()
        self.assertEqual([gate.update((3, 'blue', 2), i / 5)
                          for i in range(5)],
                         [False, False, False, False, True])

    def test_alternating_rows_never_confirm(self):
        """Detecting something in five frames is insufficient."""
        gate = StableResult()
        self.assertFalse(any(gate.update(i % 2, i / 5)
                             for i in range(20)))

    def test_missing_frame_resets_confirmation(self):
        """An absent observation breaks the consecutive sequence."""
        gate = StableResult(required=2)
        self.assertFalse(gate.update(3, 0))
        self.assertFalse(gate.update(None, 0.2))
        self.assertFalse(gate.update(3, 0.4))
        self.assertTrue(gate.update(3, 0.6))

    def test_long_gap_or_clock_reset_restarts_confirmation(self):
        """Old detections do not count after a pause or clock reset."""
        for next_time in (5, -1, 0):
            gate = StableResult(required=2)
            self.assertFalse(gate.update(3, 0))
            self.assertFalse(gate.update(3, next_time))


if __name__ == '__main__':
    unittest.main()
