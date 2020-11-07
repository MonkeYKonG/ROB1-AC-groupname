from wall_follower.Shapes.point import Point, rotate_vector
from wall_follower.tools import get_intersection, get_angle, get_angles_diff, get_position, DataBuffer


def interpret_points_sequence(points):
    if len(points) == 0:
        return []
    if len(points) < 3:
        return [Wall(points[0], points[-1])]
    middle_index = len(points) // 2
    begin_point = points[0]
    middle_point = points[middle_index]
    end_point = points[-1]
    first_part_vector = middle_point - begin_point
    second_part_vector = end_point - middle_point
    first_part_variance = get_angle(Point(first_part_vector.dist, 0.01))
    second_part_variance = get_angle(Point(second_part_vector.dist, 0.01))
    diff = get_angles_diff(get_angle(first_part_vector), get_angle(second_part_vector))
    if abs(diff) > first_part_variance + second_part_variance:
        first_part_walls = interpret_points_sequence(points[:middle_index])
        second_part_walls = interpret_points_sequence(points[middle_index:])
        return first_part_walls + second_part_walls
    return [Wall(begin_point, end_point)]


def get_points_sequences(ranges: DataBuffer) -> list:
    sequences = []
    cur_suit = []
    for index, r in enumerate(ranges):
        if r > 12:
            if len(cur_suit):
                sequences.append(cur_suit)
                cur_suit = []
            continue
        position = get_position(index, r)
        cur_suit.append((index, Point(position.x, position.y)))
    if len(cur_suit):
        if len(sequences) and sequences[0][0][0] == 0:
            sequences[0] = cur_suit + sequences[0]
        else:
            sequences.append(cur_suit)
    return [[t[1] for t in s] for s in sequences]


def join_walls(walls: list) -> list:
    joined_walls = []
    cur_wall = walls.pop(0)
    while len(walls):
        wall = walls.pop(0)
        diff = get_angles_diff(get_angle(cur_wall.direction), get_angle(wall.direction))
        cur_wall_angle_variance = get_angle(Point(cur_wall.dist, 0.01))
        wall_angle_variance = get_angle(Point(wall.dist, 0.01))
        if cur_wall.begin_point == cur_wall.end_point or wall.begin_point == wall.end_point or diff < (cur_wall_angle_variance + wall_angle_variance) * 2:
            cur_wall = Wall(cur_wall.begin_point, wall.end_point)
        else:
            joined_walls.append(cur_wall)
            cur_wall = wall
    joined_walls.append(cur_wall)
    return joined_walls


def get_walls(ranges: DataBuffer) -> list:
    points_sequence = get_points_sequences(ranges)
    walls = []
    for points in points_sequence:
        walls += join_walls(interpret_points_sequence(points))
    return [w for w in walls if w.dist > 0.1]


class Wall:
    def __init__(self, begin_point, end_point):
        self._begin_point = begin_point
        self._end_point = end_point

    def __str__(self):
        return f'Wall<begin_point: {self.begin_point}, end_point: {self.end_point}, dist: {self.dist}, angle: {get_angle(self.direction)}>'

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
    def perpendicular_direction(self):
        return rotate_vector(self.direction, -90)

    @property
    def middle(self):
        return self.begin_point + (self.direction * (self.dist / 2))

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

    def get_target_extremity(self) -> Point:
        best_angle = get_angle(self.direction)
        rotated_begin_point = rotate_vector(self.begin_point, best_angle)
        rotated_end_point = rotate_vector(self.end_point, best_angle)
        if rotated_begin_point.x > rotated_end_point.x:
            return self.begin_point
        else:
            return self.end_point

    def intersect(self, point: Point) -> bool:
        from_begin_dist = (self.begin_point - point).dist
        from_end_dist = (self.end_point - point).dist
        return from_begin_dist < self.dist and from_end_dist < self.dist
