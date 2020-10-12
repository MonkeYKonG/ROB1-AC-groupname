import math
from Shapes.point import Point
import numpy as np


def get_closest_range_index(ranges):
    min_index = 0
    min_range = ranges[min_index]
    for index, r in enumerate(ranges[1:]):
        if r < min_range:
            min_range = r
            min_index = index
    return min_index


def get_sign(a):
    return -1 if a < 0 else 1


def get_position(angle, dist):
    x = math.cos(math.radians(angle)) * dist
    y = math.sin(math.radians(angle)) * dist
    return Point(x, y)


def get_angle(direction):
    return math.degrees(np.arccos(np.dot([1, 0], direction.normalized.to_array()))) * get_sign(direction.y)


def are_close_directions(direction, last_direction, dist, self=None, angles_diffs=[]) -> bool:
    direction_angle = get_angle(direction)
    last_direction_angle = get_angle(last_direction)
    if self and len(angles_diffs) and (max(angles_diffs) < direction_angle or min(angles_diffs) > direction_angle):
        self.get_logger().info(f'New extemity')
    angles_diffs.append(direction_angle)
    if self:
        self.get_logger().info(f'AreClose: {direction_angle}, {last_direction_angle}, {abs(direction_angle) - abs(last_direction_angle)}, {dist}')
        self.get_logger().info(f'Angle diffs mean {np.array(angles_diffs).mean()}')
    if abs(abs(direction_angle) - abs(last_direction_angle)) < 0.5 if dist > 2 else 8:
        return True
    return False


def get_intersection(first_point, first_direction, second_point, second_direction):
    a1 = first_direction.y / first_direction.x
    b1 = first_point.y - a1 * first_point.x
    a2 = second_direction.y / second_direction.x
    b2 = second_point.y - a2 * second_point.x
    x = (b2 - b1) / (a1 - a2)
    return Point(x, a1 * x + b1)


def get_angles_diff(first_angle, second_angle):
    max_angle = max(first_angle, second_angle)
    min_angle = min(first_angle, second_angle)
    diff = abs(max_angle - min_angle)
    return diff if diff < 180 else 360 - diff


class DataBuffer:
    def __init__(self, max_size=10):
        self.max_size = max_size
        self._buffer = []

    def __iter__(self):
        return self.mean_data.__iter__()

    def append(self, data):
        self._buffer.append(data)
        if len(self._buffer) > self.max_size:
            self._buffer.pop(0)

    def clear(self):
        self._buffer.clear()

    @property
    def length(self):
        return len(self._buffer)

    @property
    def mean_data(self):
        return [np.array([data[i] for data in self._buffer]).mean() for i in range(len(self._buffer[0]))]
