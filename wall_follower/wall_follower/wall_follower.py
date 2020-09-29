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
    return math.degrees(np.arccos(np.dot([0, 1], direction.to_array())))


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Point(self.x * other.x, self.y * other.y)

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
        self.update_timer = self.create_timer(0.010, self.update_callback)

        """ Private attributes """
        self._current_ranges = None
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        self._cur_state = self.STATE_SEARCH_WALL

        self._safety_distance = 0.3

    def __log_change_state(self, previous_state, new_state):
        self.get_logger().info(f'State {previous_state} finish -> Start state {new_state}')

    def __get_continuous_points(self, closest, ranges):
        start_point = get_position(closest, self._current_ranges[closest])
        wall_direction = None
        points = []
        for i in ranges:
            dist = self._current_ranges[i]
            if dist > 12.0:
                self.get_logger().info(f'To far {dist}')
                break
            point_position = get_position(i, dist)
            direction = point_position - start_point
            if wall_direction is None:
                wall_direction = direction
            if wall_direction.normalized != direction.normalized:
                self.get_logger().info(f'BREAK! {i} -> {wall_direction.normalized} --> {direction.normalized}')
                # break
            points.append(i)
        return points

    def __get_wall_points(self, closest):
        left_points = self.__get_continuous_points(closest, range(closest)[::-1])
        right_points = self.__get_continuous_points(closest, range(closest + 1, 360))
        self.get_logger().info(f'Let points: {left_points}')
        self.get_logger().info(f'Right points: {right_points}')
        return left_points + [closest] + right_points

    def __are_close_directions(self, direction, last_direction, dist) -> bool:
        direction_angle = get_angle(direction)
        last_direction_angle = get_angle(last_direction)
        self.get_logger().info(f'angle {direction_angle} -> last_angle: {last_direction_angle}')
        if abs(direction_angle - last_direction_angle) < 0.5:
            return True
        self.get_logger().info(f'dist {dist}')
        return False

    def __valid_found_on_suit(self, points, last_direction, begin_point):
        for point in points:
            self.get_logger().info(f'{point}')
            direction = (point - begin_point).normalized
            if self.__are_close_directions(direction, last_direction, (point - begin_point).dist):
                return True
        return False

    def __interpret_suit_of_points(self, points):
        if len(points) == 1:
            return points
        walls = []
        begin_index = 0
        last_direction = (points[1] - points[begin_index]).normalized
        for index, point in enumerate(points[2:]):
            direction = (point - points[begin_index]).normalized
            # self.get_logger().info(f'd: {direction}')
            if last_direction is not None and not self.__are_close_directions(direction, last_direction, (point - points[begin_index]).dist):
                self.get_logger().info(f'PREDICTION -> {index} -> {point}')
                if not self.__valid_found_on_suit(points[index + 1:index + 6], last_direction, points[begin_index]):
                    self.get_logger(f'ALL FAIL -> WALL CORNER FOUND')
                    walls.append(points[:index])
                    begin_index = index
                    last_direction = None
                    self.get_logger().info(f'END PREDICTION')
                    continue
                self.get_logger().info(f'END PREDICTION')
            last_direction = direction
        if begin_index != len(points) - 1:
            walls.append(points[begin_index:])
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

    def __search_for_wall(self):
        points_suits = self.__get_suits_of_points()
        self.get_logger().info(f'{len(points_suits)}')
        walls = []
        for points in points_suits:
            walls += self.__interpret_suit_of_points(points)
        self.get_logger().info(f'Found {len(walls)} walls')
        exit(666)
        # walls = []
        # current_wall = []
        # for index, r in enumerate(self._current_ranges):
        #     self.__interpret_point(index, r current_wall, walls)
        # cur_direction = (cur_pos - point).normalized
        # if point != cur_pos:
        #     if base_direction:
        #         dir_diff = base_direction - cur_direction
        #         if abs(dir_diff.x) > 0.15 or abs(dir_diff.y) > 0.15:
        #             self.get_logger().info(f'To big diff:\n{dir_diff}\n{base_direction}\n{cur_direction}\n')
        #             point = None
        #             base_direction = None
        #             if len(current_wall):
        #                 walls.append(Wall(current_wall[0], current_wall[len(current_wall) - 1]))
        #                 current_wall = []
        #             continue
        #     base_direction = cur_direction
        if len(current_wall):
            walls.append(Wall(current_wall[0], current_wall[len(current_wall) - 1]))
        jumpline = '\n'
        self.get_logger().info(f'{len(walls)} --\n{jumpline.join((str(w) for w in walls))}')
        exit(1)
        closest = get_closest_range_index(self._current_ranges)
        self.get_logger().info(f'CLOSEST! -> {closest} - {self._current_ranges[closest]}')
        wall = self.__get_wall_points(closest)
        exit(1)
        degrees_to_rotate = closest if closest < 90 else 360 - closest
        self.get_logger().info(f'{closest}')
        if degrees_to_rotate > 0:
            self._cur_angular = self.MAX_LINEAR * (1 if closest < 90 else -1)
        else:
            self._cur_angular = self.MIN_ANGULAR
            if self._current_ranges[closest] > self._safety_distance:
                self._cur_linear = self.MAX_LINEAR
            else:
                self._cur_linear = self.MIN_LINEAR
                self._cur_state = self.STATE_FOLLOW_WALL
                self.__log_change_state(self.STATE_SEARCH_WALL, self.STATE_FOLLOW_WALL)

    def __move_to_wall(self):
        pass

    def __follow_wall(self):
        self.get_logger().info(f'Step follow wall, {self._current_ranges[80:90]}')
        right_ranges = self._current_ranges[80:90]
        closest_right = get_closest_range_index(right_ranges)
        front_ranges = self._current_ranges[:10]
        closest_front = get_closest_range_index(front_ranges)

        if right_ranges[closest_right] < self._safety_distance:
            self.get_logger().info('Wall on right!')
        else:
            self.get_logger().info('Wall no on right')
            self.get_logger().info(f'{self._current_ranges[:80]}')

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
