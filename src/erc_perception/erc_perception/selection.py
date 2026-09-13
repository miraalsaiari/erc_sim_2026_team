"""Conservative decisions from visible labels and book candidates."""

from statistics import median


COLOURS = ('red', 'green', 'yellow', 'blue')


def column_from_labels(labels, target, image_size):
    """
    Locate a labelled column, rejecting incomplete or ambiguous geometry.

    A partial view needs both neighbours of the target. Outer boundaries
    are extrapolated only when all five distinct labels are visible.
    Geometry assumes a roughly frontal view of the competition shelf.
    """
    width, height = image_size
    ordered = sorted(labels, key=lambda item: item['centre_x'])
    candidates = []
    for count in range(3, min(5, len(ordered)) + 1):
        for start in range(len(ordered) - count + 1):
            group = ordered[start:start + count]
            digits = [item['digit'] for item in group]
            if target not in digits or len(set(digits)) != count:
                continue
            if not set(digits).issubset({1, 2, 3, 4, 5}):
                continue
            index = digits.index(target)
            if count < 5 and index in (0, count - 1):
                continue
            xs = [item['centre_x'] for item in group]
            ys = [item['centre_y'] for item in group]
            hs = [item['bbox'][3] for item in group]
            gaps = [b - a for a, b in zip(xs, xs[1:])]
            spacing = median(gaps)
            glyph_height = median(hs)
            if min(hs) < 8 or max(hs) > 1.8 * min(hs):
                continue
            # The marker plate is 0.3 m; column centres are 1 m apart.
            if not 2.5 * glyph_height <= spacing <= 6.5 * glyph_height:
                continue
            if min(gaps) < 0.65 * spacing or max(gaps) > 1.5 * spacing:
                continue
            if max(gaps) > 1.4 * min(gaps):
                continue
            slope = (ys[-1] - ys[0]) / (xs[-1] - xs[0])
            residual = max(abs(y - ys[0] - slope * (x - xs[0]))
                           for x, y in zip(xs, ys))
            if abs(slope) > 0.4 or residual > 0.45 * glyph_height:
                continue
            left = ((xs[index - 1] + xs[index]) / 2 if index
                    else xs[0] - gaps[0] / 2)
            right = ((xs[index] + xs[index + 1]) / 2
                     if index < count - 1 else xs[-1] + gaps[-1] / 2)
            if left < 0 or right > width:
                continue
            local_spacing = right - left
            label = group[index]
            cy = label['centre_y']
            top = max(0, int(cy - 0.65 * label['bbox'][3]))
            bottom = min(height, int(cy + 2.25 * local_spacing))
            candidates.append({
                'digit': target,
                'label': label,
                'labels': group,
                'spacing': local_spacing,
                'bbox': (int(left), top, int(right - left), bottom - top),
                'book_bounds': (left, cy + 0.30 * local_spacing,
                                right, cy + 1.95 * local_spacing),
            })
    if not candidates:
        return None
    best = max(candidates, key=lambda item: len(item['labels']))
    # Two different visible targets cannot both be the requested column.
    for item in candidates:
        if (abs(item['label']['centre_x'] - best['label']['centre_x'])
                > best['label']['bbox'][3] / 2 or
                abs(item['label']['centre_y'] - best['label']['centre_y'])
                > best['label']['bbox'][3] / 2):
            return None
    return best


def book_from_candidates(books, column, colour, image_size):
    """Return a row only when all four colours are visible in this column."""
    if column is None or colour not in COLOURS:
        return None
    left, top, right, bottom = column['book_bounds']
    image_width, image_height = image_size
    if left < 0 or right > image_width or top < 0 or bottom > image_height:
        return None
    spacing = column['spacing']
    by_colour = {name: [] for name in COLOURS}
    for book in books:
        x, y, width, height = book['bbox']
        if book['colour'] not in by_colour:
            continue
        if not (left < x and x + width < right and
                top < y and y + height < bottom):
            continue
        if not (max(3, 0.06 * spacing) <= height <= 0.32 * spacing):
            continue
        if not (1 <= width <= 0.45 * spacing):
            continue
        by_colour[book['colour']].append(book)
    if any(len(items) != 1 for items in by_colour.values()):
        return None
    ordered = sorted((items[0] for items in by_colour.values()),
                     key=lambda item: item['centre_y'])
    gaps = [b['centre_y'] - a['centre_y']
            for a, b in zip(ordered, ordered[1:])]
    if min(gaps) < 0.17 * spacing or max(gaps) > 0.50 * spacing:
        return None
    if max(gaps) > 1.8 * min(gaps):
        return None
    for row, book in enumerate(ordered, start=1):
        if book['colour'] == colour:
            return dict(book, row_number=row)
    return None


class StableResult:
    """Require consecutive matching observations from fresh camera frames."""

    def __init__(self, required=5, max_gap=1.5):
        """Set the confirmation count and maximum gap in seconds."""
        if required < 1 or max_gap <= 0:
            raise ValueError('Invalid stability settings')
        self.required = required
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        """Forget an old or missing observation."""
        self.key = None
        self.count = 0
        self.last_time = None

    def update(self, key, now):
        """Return true after the same nonempty key persists long enough."""
        if key is None:
            self.reset()
            return False
        expired = (self.last_time is not None and
                   (now <= self.last_time or
                    now - self.last_time > self.max_gap))
        if expired or key != self.key:
            self.count = 0
        self.key = key
        self.last_time = now
        self.count += 1
        return self.count >= self.required
