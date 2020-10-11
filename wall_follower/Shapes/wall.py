from Shapes.point import Point, rotate_vector
from tools import get_intersection, get_angle


class Wall:
    def __init__(self, begin_point, end_point):
        self._begin_point = begin_point
        self._end_point = end_point

    def __str__(self):
        return f'Wall<begin_point: {self.begin_point}, end_point: {self.end_point}, dist: {self.dist}>'

    def __repr__(self):
        return str(self)

    @property
    def begin_point(self):
        return self._begin_point

    @property
    def end_point(self):
        return self._end_point

    @property
    def dist(self):
        return (self.end_point - self.begin_point).dist

    @property
    def direction(self):
        return (self.end_point - self.begin_point).normalized

    @property
    def middle(self):
        return self.begin_point + self.direction * (self.dist / 2)

    def closer_point_from_origin(self) -> Point:
        closer_point_on_line = get_intersection(Point.zero(), rotate_vector(self.direction, 90), self.begin_point,
                                                self.direction)
        closer_point_diff = closer_point_on_line - self.begin_point
        angles_diff = round(abs(abs(get_angle(self.direction)) - abs(get_angle(closer_point_diff.normalized))), 4)
        if closer_point_diff.dist < 0 or closer_point_diff.dist > self.dist or angles_diff != 0:
            if self.begin_point.dist < self.end_point.dist:
                return self.begin_point
            else:
                return self.end_point
        return closer_point_on_line

    def closer_dist_from_origin(self) -> float:
        return self.closer_point_from_origin().dist
