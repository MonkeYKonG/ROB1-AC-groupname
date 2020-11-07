import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from wall_follower.tools import *
from wall_follower.Shapes.point import Point, rotate_vector
from wall_follower.Shapes.wall import Wall, get_walls


class WallFollower(Node):
    MAX_LINEAR = 0.1
    MIN_LINEAR = 0.0
    MAX_ANGULAR = 0.1
    MIN_ANGULAR = 0.0

    STATE_SEARCH_WALL = 0
    STATE_MOVE_TO_WALL = 1
    STATE_FOLLOW_WALL = 2
    STATE_TEST_MOVING = 3

    def __init__(self, debug=False):
        super().__init__('wall_follower')

        """ Subscriber / Publisher / Timer """
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.__scan_sensor_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.cmd_vel_raw_sub = self.create_subscription(
            Odometry,
            'odom',
            self.__odom_callback,
            rclpy.qos.QoSProfile(depth=10))
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.update_delay = 0.010
        self.update_timer = self.create_timer(self.update_delay, self.__update_callback)

        """ Private attributes """
        self._debug = debug
        self._current_ranges = DataBuffer()
        self._location = None
        self._rotation = None
        self._target_location = None
        self._target_rotation = None
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        self._cur_state = self.STATE_SEARCH_WALL if not self._debug else self.STATE_TEST_MOVING

        self._safety_distance = 0.3
        self._target_max_dist = 1.0
        self.last_odom_twist = None

    @property
    def info(self):
        return self.get_logger().info

    def __reset_location_and_rotation(self):
        self._location = Point.zero()
        self._rotation = 0.0

    def __log_change_state(self, previous_state, new_state):
        self.info(f'State {previous_state} finish -> Start state {new_state}')

    @staticmethod
    def __get_longer_wall(walls):
        longer_wall = None
        for wall in walls:
            if longer_wall is None or longer_wall.dist < wall.dist:
                longer_wall = wall
        return longer_wall

    @staticmethod
    def __get_closer_wall(walls) -> Wall:
        closer_wall = None
        for wall in walls:
            if closer_wall is None or closer_wall.closer_dist_from_origin() > wall.closer_dist_from_origin():
                closer_wall = wall
        return closer_wall

    def __rotate_to_wall(self):
        diff = abs(self._target_rotation - self._rotation)
        move = self.MAX_ANGULAR * get_sign(self._target_rotation)
        next_rotation = move * self.update_delay
        if abs(next_rotation) > diff:
            move = diff * self.update_delay * get_sign(self._target_rotation)
            next_rotation = diff * get_sign(self._target_rotation)
        self._cur_angular = move
        self._cur_linear = self.MIN_LINEAR
        self._rotation += next_rotation

    def __forward_to_wall(self):
        diff = self._target_location - self._location
        move = self.MAX_LINEAR
        next_movement = diff.normalized * move * self.update_delay
        if next_movement.dist > diff.dist:
            move = diff.dist * self.update_delay
            next_movement = diff
        self._cur_angular = self.MIN_ANGULAR
        self._cur_linear = move
        self._location += next_movement

    def __validate_wall_reach(self):
        self._cur_angular = self.MIN_ANGULAR
        self._cur_linear = self.MIN_LINEAR
        self.__log_change_state(self._cur_state, self.STATE_FOLLOW_WALL)
        self._cur_state = self.STATE_FOLLOW_WALL if not self._debug else self.STATE_TEST_MOVING
        self._current_ranges.clear()

    def __is_safe_spot(self, next_point: Point, intersection: Point):
        return next_point.dist + self._safety_distance > intersection.dist

    def __verify_intersections(self, next_point: Point, best_wall: Wall, walls: list):
        for wall in walls:
            intersection = get_intersection(Point.zero(), best_wall.direction.normalized, wall.begin_point,
                                            wall.direction)
            angles_diff = get_angles_diff(get_angle(intersection), get_angle(next_point))
            if self.__is_safe_spot(next_point, intersection) and angles_diff < 45 and wall.intersect(next_point):
                return self.__find_next_point(wall, [w for w in walls if w != best_wall])
        return next_point

    def __find_next_point(self, best_wall: Wall, walls: list):
        next_point = best_wall.get_target_extremity() + (best_wall.perpendicular_direction * self._safety_distance)
        if next_point.dist > self._target_max_dist:
            next_point = next_point.normalized * self._target_max_dist
        next_point = self.__verify_intersections(next_point, best_wall, walls)
        if next_point.dist < 0.1:
            next_point = best_wall.end_point + (rotate_vector(best_wall.direction, 30) * self._safety_distance)
        return next_point

    def __state_search_for_wall(self):
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        if self._current_ranges.length < self._current_ranges.max_size:
            return
        walls = get_walls(self._current_ranges)
        best_wall = self.__get_closer_wall(walls)
        self.__reset_location_and_rotation()
        self._target_location = best_wall.closer_point_from_origin() + (rotate_vector(best_wall.direction, -90) * self._safety_distance)
        self._target_rotation = math.radians(get_angle(self._target_location.normalized))
        self.__log_change_state(self._cur_state, self.STATE_MOVE_TO_WALL)
        self._cur_state = self.STATE_MOVE_TO_WALL

    def __state_move_to_wall(self):
        if self._rotation != self._target_rotation:
            self.__rotate_to_wall()
        elif self._location != self._target_location:
            self.__forward_to_wall()
        else:
            self.__validate_wall_reach()

    def __state_follow_wall(self):
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        if self._current_ranges.length < self._current_ranges.max_size:
            return
        walls = get_walls(self._current_ranges)
        self.__reset_location_and_rotation()
        best_wall = self.__get_closer_wall(walls)
        next_point = self.__find_next_point(best_wall, [w for w in walls if w != best_wall])
        self._target_location = next_point
        self._target_rotation = math.radians(get_angle(self._target_location.normalized))
        self.__log_change_state(self._cur_state, self.STATE_MOVE_TO_WALL)
        self._cur_state = self.STATE_MOVE_TO_WALL

    def __state_test_moving(self):
        self.__reset_location_and_rotation()
        self._target_location = Point.up() * 4
        self._target_rotation = math.radians(get_angle(self._target_location))
        self._cur_state = self.STATE_MOVE_TO_WALL

    def __publish(self):
        twist = Twist()
        twist.linear.x = self._cur_linear if self._cur_linear != 0.0 else -0.00095
        twist.angular.z = self._cur_angular
        self.publisher.publish(twist)

    def __scan_sensor_callback(self, sensor_data):
        self._current_ranges.append(sensor_data.ranges)

    def __odom_callback(self, odometry):
        self.last_odom_twist = odometry.twist.twist

    def __update_callback(self):
        if self._current_ranges is not None:
            self.STATES[self._cur_state](self)
        self.__publish()

    STATES = {
        STATE_SEARCH_WALL: __state_search_for_wall,
        STATE_MOVE_TO_WALL: __state_move_to_wall,
        STATE_FOLLOW_WALL: __state_follow_wall,
        STATE_TEST_MOVING: __state_test_moving
    }


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = WallFollower()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
