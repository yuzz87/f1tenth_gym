"""Small scan-analysis helpers used by validation nodes and tests."""

import math


SECTOR_CENTERS_RAD = {
    "front": 0.0,
    "left": 0.5 * math.pi,
    "rear": -math.pi,
    "right": -0.5 * math.pi,
}


def _angular_distance(first, second):
    return abs((float(first) - float(second) + math.pi) % (2.0 * math.pi) - math.pi)


def mask_near_field_sector(
    ranges,
    angle_min_rad,
    angle_increment_rad,
    center_rad,
    half_width_rad,
    maximum_range_m,
):
    """Mask near-field returns in one angular sector and return its count."""

    masked = [float(value) for value in ranges]
    masked_count = 0
    for index, value in enumerate(masked):
        if not math.isfinite(value) or value > float(maximum_range_m):
            continue
        angle = float(angle_min_rad) + index * float(angle_increment_rad)
        if _angular_distance(angle, center_rad) <= float(half_width_rad):
            masked[index] = math.inf
            masked_count += 1
    return masked, masked_count


def sector_minima(
    ranges,
    angle_min_rad,
    angle_increment_rad,
    range_min_m,
    range_max_m,
    half_width_rad=math.radians(15.0),
):
    """Return the nearest valid point in front, left, rear, and right sectors."""

    minima = {name: math.inf for name in SECTOR_CENTERS_RAD}
    for index, value in enumerate(ranges):
        value = float(value)
        if not math.isfinite(value) or value < range_min_m or value > range_max_m:
            continue
        angle = float(angle_min_rad) + index * float(angle_increment_rad)
        for name, center in SECTOR_CENTERS_RAD.items():
            if _angular_distance(angle, center) <= float(half_width_rad):
                minima[name] = min(minima[name], value)
    return minima
