import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
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
    return math.degrees(np.arccos(np.dot([1, 0], direction.to_array()))) * get_sign(direction.y)


def rotate_vector(vector, angle):
    radian = math.radians(angle)
    cos = math.cos(radian)
    sin = math.sin(radian)
    x = vector.x * cos + vector.y * sin
    y = vector.x * -sin + vector.y * cos
    return Point(x, y)


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
    a1 = first_direction.x / first_direction.y
    b1 = -first_point.x / first_direction.x
    a2 = second_direction.x / second_direction.y
    b2 = -second_point.x / second_direction.x
    x = (b2 - b1) / (a1 - a2)
    return Point((b2 - b1) / (a1 - a2), a1 * x + b1)


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Point(self.x * other, self.y * other)
        return Point(self.x * other.x, self.y * other.y)

    def __truediv__(self, other):
        if isinstance(other, (int, float)):
            return Point(self.x / other, self.y / other)
        return Point(self.x / other.x, self.y / other.y)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self):
        return f'Point<x: {self.x}, y: {self.y}>'

    def __repr__(self):
        return str(self)

    @property
    def normalized(self):
        normalizer = self.dist
        if normalizer == 0:
            return Point(0, 0)
        return Point(self.x / normalizer, self.y / normalizer)

    @property
    def dist(self):
        return math.sqrt(self.x ** 2 + self.y ** 2)

    def to_array(self) -> list:
        return [self.x, self.y]

    def rotate(self, angle):
        new_point = rotate_vector(self, angle)
        self.x = new_point.x
        self.y = new_point.y


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
        return self.begin_point + self.direction * self.dist / 2

    def dist_to_origin(self) -> float:
        lambda_s = self.begin_point.dist * self.end_point.dist / self.dist ** 2
        if lambda_s >= 1:
            return self.end_point.dist
        elif lambda_s <= 0:
            return self.begin_point.dist
        return (self.begin_point + self.direction * lambda_s).dist


class WallFollower(Node):
    MAX_LINEAR = 0.1
    MIN_LINEAR = 0.0
    MAX_ANGULAR = 0.1
    MIN_ANGULAR = 0.0

    STATE_SEARCH_WALL = 0
    STATE_MOVE_TO_WALL = 1
    STATE_FOLLOW_WALL = 2

    def __init__(self):
        super().__init__('wall_follower')

        """ Subscriber / Publisher / Timer """
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_sensor_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            rclpy.qos.QoSProfile(depth=10))
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.update_delay = 0.010
        self.update_timer = self.create_timer(self.update_delay, self.update_callback)

        """ Private attributes """
        self._current_ranges = None
        self._location = None
        self._rotation = None
        self._target_location = None
        self._target_rotation = None
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        self._cur_state = self.STATE_SEARCH_WALL

        self._safety_distance = 0.5

    def __reset_location_and_rotation(self):
        self._location = Point(0, 0)
        self._rotation = 0.0

    def __log_change_state(self, previous_state, new_state):
        self.get_logger().info(f'State {previous_state} finish -> Start state {new_state}')

    @staticmethod
    def __valid_found_on_suit(points, last_direction, begin_point):
        for point in points:
            direction = (point - begin_point).normalized
            if are_close_directions(direction, last_direction, (point - begin_point).dist):
                return True
        return False

    def are_same_wall(self, direction, last_angles):
        angle = get_angle(direction)
        if len(last_angles) and (min(last_angles) > angle or max(last_angles) < angle):
            return False
        return True

    def __interpret_suit_of_points(self, points):
        if len(points) == 1:
            return points
        walls = []
        angles = []
        begin_index = 0
        while begin_index < len(points):
            begin_index += 1
        exit(1)
        for index, point in enumerate(points[1:]):
            cur_angle = get_angle((point - points[index]).normalized)
            angles.append(abs(cur_angle))
            angles_diff = abs(np.array(angles).mean() - abs(cur_angle))
            self.get_logger().info(f'{cur_angle} - {np.array(angles).mean()} - {angles_diff}')
            if angles_diff > 50:
                self.get_logger().info(f'Big angle diff ({index}) -> {angles_diff} - {np.array(angles).mean()}')
                ok, not_ok = 0, 0
                for i, p in enumerate(points[index+2:index+7]):
                    a = abs(np.array(angles).mean() - abs(get_angle((p - points[index+1+i]).normalized)))
                    self.get_logger().info(f'Projection -> {a}')
                    if a > 50:
                        not_ok += 1
                    else:
                        ok += 1
                self.get_logger().info(f'END Projection -> ok: {ok} - not_ok: {not_ok}')
                if not_ok > ok:
                    self.get_logger().info(f'WALL FOUND')
                    walls.append(Wall(points[begin_index], point))
                    begin_index = index + 2
                    angles.clear()
                    angles.append(abs(cur_angle))
        if begin_index < len(points):
            walls.append(Wall(points[begin_index], points[-1]))
        self.get_logger().info(f'Walls count: {len(walls)}\n{walls}')
        exit(1)
            # direction = (point - points[begin_index]).normalized
        #
        #     if last_direction is not None and not are_close_directions(direction, last_direction,
        #                                                                (point - points[begin_index]).dist, self, angle_diffs):
        #         if not self.__valid_found_on_suit(points[index + 1:index + 6], last_direction, points[begin_index]):
        #             self.get_logger().info(f'WALLS FOUND {Wall(points[begin_index], points[index])}')
        #             walls.append(Wall(points[begin_index], points[index]))
        #             begin_index = index
        #             last_direction = None
        #             continue
        #     last_direction = direction
        # if begin_index != len(points) - 1:
        #     walls.append(Wall(points[begin_index], points[len(points) - 1]))
        return walls

    def __get_suits_of_points(self):
        suits = []
        cur_suit = []
        for index, r in enumerate(self._current_ranges):
            if r > 12:
                if len(cur_suit):
                    suits.append(cur_suit)
                    cur_suit = []
                continue
            position = get_position(index, r)
            cur_suit.append((index, Point(position.x, position.y)))
        if len(cur_suit):
            if len(suits) and suits[0][0][0] == 0:
                suits[0] = cur_suit + suits[0]
            else:
                suits.append(cur_suit)
        return [[t[1] for t in s] for s in suits]

    def __get_walls(self):
        points_suits = self.__get_suits_of_points()
        # self.get_logger().info(f'{len(points_suits)}')
        walls = []
        for points in points_suits:
            walls += self.__interpret_suit_of_points(points)
        return walls

    @staticmethod
    def __get_longer_wall(walls):
        longer_wall = None
        for wall in walls:
            if longer_wall is None or longer_wall.dist < wall.dist:
                longer_wall = wall
        return longer_wall

    @staticmethod
    def __get_closer_wall(walls):
        closer_wall = None
        for wall in walls:
            if closer_wall is None or closer_wall.dist_to_origin() > wall.dist_to_origin():
                closer_wall = wall
        return closer_wall

    def __search_for_wall(self):
        walls = self.__get_walls()
        self.get_logger().info(f'Walls count: {len(walls)}')
        for w in walls:
            self.get_logger().info(f'Walls : {w}')
        exit(1)
        best_wall = self.__get_longer_wall(walls)
        self._cur_linear = self.MAX_LINEAR
        self._cur_angular = self.MAX_ANGULAR
        self.__reset_location_and_rotation()
        self.get_logger().info(f'best wall direction: {best_wall.direction}')
        self.get_logger().info(f'best wall direction rotated: {rotate_vector(best_wall.direction, -90)}')
        self._target_location = best_wall.middle + rotate_vector(best_wall.direction, -90) * self._safety_distance
        self._target_rotation = math.radians(get_angle(self._target_location.normalized))
        self.get_logger().info(f'Next Location: {self._target_location}')
        self.get_logger().info(f'Next rotation: {self._target_rotation}')
        self.__log_change_state(self._cur_state, self.STATE_MOVE_TO_WALL)
        self._cur_state = self.STATE_MOVE_TO_WALL

    def __move_to_wall(self):
        if self._rotation != self._target_rotation:
            diff = self._target_rotation - self._rotation
            move = self.MAX_ANGULAR
            next_rotation = move * self.update_delay
            if next_rotation > diff:
                move = diff * self.update_delay
                next_rotation = diff
            self._cur_angular = move
            self._cur_linear = self.MIN_LINEAR
            self._rotation += next_rotation
        elif self._location != self._target_location:
            diff = self._target_location - self._location
            move = self.MAX_LINEAR
            next_movement = diff.normalized * move * self.update_delay
            if next_movement.dist > diff.dist:
                move = diff.dist * self.update_delay
                next_movement = diff
            self._cur_angular = self.MIN_ANGULAR
            self._cur_linear = move
            self._location += next_movement
        else:
            self._cur_angular = self.MIN_ANGULAR
            self._cur_linear = self.MIN_LINEAR
            self.__log_change_state(self._cur_state, self.STATE_FOLLOW_WALL)
            self._cur_state = self.STATE_FOLLOW_WALL

    def __follow_wall(self):
        walls = self.__get_walls()
        self.get_logger().info(f'Walls count: {len(walls)}')
        for w in walls:
            self.get_logger().info(f'Walls: {w}')
        best_wall = self.__get_closer_wall(walls)
        self.get_logger().info(f'Best wall: {best_wall}')
        self.__reset_location_and_rotation()
        best_angle = get_angle(best_wall.direction)
        self.get_logger().info(f'best wall angle {best_angle}')
        rotated_begin_point = rotate_vector(best_wall.begin_point, best_angle)
        rotated_end_point = rotate_vector(best_wall.end_point, best_angle)
        if rotated_begin_point.x > rotated_end_point.x:
            best_wall_extremity = best_wall.begin_point
        else:
            best_wall_extremity = best_wall.end_point
        self.get_logger().info(f'Best wall extremity: {best_wall_extremity}')
        best_direction = rotate_vector(best_wall.direction, -best_angle)
        next_point = best_wall_extremity + best_direction * self._safety_distance
        for wall in (w for w in walls if w != best_wall):
            intersection = get_intersection(Point(0, 0), best_wall.direction.normalized, wall.begin_point, wall.direction)
            self.get_logger().info(f'Intersection {intersection}')
            self.get_logger().info(f'intersec dist: {(intersection - wall.begin_point).dist} -- wall dist: {wall.dist}')
            if (intersection - wall.begin_point).dist < wall.dist:
                self.get_logger().info(f'Intersection point on wall')
        self.get_logger().info(f'NEXT POINT {next_point}')
        self._target_location = next_point
        self._target_rotation = math.radians(get_angle(self._target_location.normalized))
        self.__log_change_state(self._cur_state, self.STATE_MOVE_TO_WALL)
        self._cur_state = self.STATE_MOVE_TO_WALL

    def __publish(self):
        twist = Twist()
        twist.linear.x = self._cur_linear
        twist.angular.z = self._cur_angular
        self.publisher.publish(twist)

    def scan_sensor_callback(self, sensor_data):
        self._current_ranges = sensor_data.ranges

    def odom_callback(self, odometry):
        return
        # self.get_logger().info(f'CMD VEL ROW -> {odometry}')

    def update_callback(self):
        if self._current_ranges is not None:
            self.STATES[self._cur_state](self)
        self.__publish()

    STATES = {
        STATE_SEARCH_WALL: __search_for_wall,
        STATE_MOVE_TO_WALL: __move_to_wall,
        STATE_FOLLOW_WALL: __follow_wall
    }


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = WallFollower()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
