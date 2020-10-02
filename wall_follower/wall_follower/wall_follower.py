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
    return math.degrees(np.arccos(np.dot([1, 0], direction.to_array())))


def rotate_vector(vector, angle):
    radian = math.radians(angle)
    cos = math.cos(radian)
    sin = math.sin(radian)
    x = vector.x * cos + vector.y * sin
    y = vector.x * -sin + vector.y * cos
    return Point(x, y)


def are_close_directions(direction, last_direction, dist) -> bool:
    direction_angle = get_angle(direction)
    last_direction_angle = get_angle(last_direction)
    if abs(direction_angle - last_direction_angle) < 0.5 if dist > 2 else 8:
        return True
    return False


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
        lambda_s = self.begin_point.dist * self.end_point.dist / self.dist**2
        if lambda_s >= 1:
            return self.end_point
        elif lambda_s <= 0:
            return self.begin_point
        return self.begin_point + self.direction * lambda_s


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

    def __interpret_suit_of_points(self, points):
        if len(points) == 1:
            return points
        walls = []
        begin_index = 0
        last_direction = (points[1] - points[begin_index]).normalized
        for index, point in list(enumerate(points))[2:]:
            direction = (point - points[begin_index]).normalized
            if last_direction is not None and not are_close_directions(direction, last_direction,
                                                                              (point - points[begin_index]).dist):
                if not self.__valid_found_on_suit(points[index + 1:index + 6], last_direction, points[begin_index]):
                    walls.append(Wall(points[begin_index], points[index]))
                    begin_index = index
                    last_direction = None
                    continue
            last_direction = direction
        if begin_index != len(points) - 1:
            walls.append(Wall(points[begin_index], points[len(points) - 1]))
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
        best_wall = self.__get_longer_wall(self.__get_walls())
        self.__reset_location_and_rotation()
        self._target_location = best_wall.middle + rotate_vector(best_wall.direction, -90) * self._safety_distance
        self._target_rotation = math.radians(get_angle(self._target_location.normalized))
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
        best_wall = self.__get_closer_wall(self.__get_walls())
        self.__reset_location_and_rotation()
        self.get_logger().info(f'Follow wall! best_wall {best_wall}')
        exit(1)
        # TODO Search for farther safe point

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
