import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tools import *
from Shapes.point import Point, rotate_vector
from Shapes.wall import Wall


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
        self._current_ranges = DataBuffer()
        self._location = None
        self._rotation = None
        self._target_location = None
        self._target_rotation = None
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        self._cur_state = self.STATE_SEARCH_WALL

        self._safety_distance = 0.3

    @property
    def info(self):
        return self.get_logger().info

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

    def are_same_wall(self, angle, angle_from_begin, angles, angles_from_begin):
        angles_mean = np.array(angles).mean()
        angle_diff = abs(angle) - abs(angles_mean)
        if abs(angle_diff) > 70:
            self.get_logger().info(f'NEXT WALL?')
            return False
        return True

    def __search_for_corner(self, points, index, angles, angles_from_begin):
        point = points[index]
        last_point = points[index - 1]
        angle = get_angle(point - last_point)
        angle_from_begin = get_angle(point - points[0])
        if len(angles):
            if not self.are_same_wall(angle, angle_from_begin, angles, angles_from_begin):
                return True
        angles.append(abs(angle))
        angles_from_begin.append(angle_from_begin)
        return False

    def __get_next_wall(self, points):
        index = 0
        angles = []
        angles_from_begin = []
        while index < len(points) - 1:
            if index > 0 and self.__search_for_corner(points, index, angles, angles_from_begin):
                break
            index += 1
        return Wall(points[0], points[index]), index + 1

    def __interpret_suit_of_points(self, points):
        if len(points) == 0:
            return []
        if len(points) < 3:
            return [Wall(points[0], points[-1])]
        middle_index = len(points) // 2
        begin_point = points[0]
        middle_point = points[middle_index]
        end_point = points[-1]
        first_half_angle = get_angle(middle_point - begin_point)
        second_half_angle = get_angle(end_point - middle_point)
        diff = abs(max(first_half_angle, second_half_angle) - min(first_half_angle, second_half_angle))
        if diff > 180:
            diff = 360 - diff
        if abs(diff) > 10:
            first_part_walls = self.__interpret_suit_of_points(points[:middle_index])
            second_part_walls = self.__interpret_suit_of_points(points[middle_index:])
            return first_part_walls + second_part_walls
        return [Wall(begin_point, end_point)]

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

    def __join_walls(self, walls):
        joined_walls = []
        cur_wall = walls.pop(0)
        while len(walls):
            wall = walls.pop(0)
            cur_angle = get_angle(cur_wall.direction)
            angle = get_angle(wall.direction)
            diff = abs(max(cur_angle, angle) - min(cur_angle, angle))
            if diff > 180:
                diff = 360 - diff
            if cur_wall.begin_point == cur_wall.end_point or wall.begin_point == wall.end_point or diff < 45:
                cur_wall = Wall(cur_wall.begin_point, wall.end_point)
            else:
                joined_walls.append(cur_wall)
                cur_wall = wall
        joined_walls.append(cur_wall)
        return joined_walls

    def __get_walls(self):
        points_suits = self.__get_suits_of_points()
        walls = []
        for points in points_suits:
            walls += self.__join_walls(self.__interpret_suit_of_points(points))
        return walls

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

    def __search_for_wall(self):
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        if self._current_ranges.length < self._current_ranges.max_size:
            return
        walls = self.__get_walls()
        best_wall = self.__get_closer_wall(walls)
        self.__reset_location_and_rotation()
        self._target_location = best_wall.closer_point_from_origin() + (rotate_vector(best_wall.direction, -90) * self._safety_distance)
        self._target_rotation = math.radians(get_angle(self._target_location.normalized))
        self.__log_change_state(self._cur_state, self.STATE_MOVE_TO_WALL)
        self._cur_state = self.STATE_MOVE_TO_WALL

    def __move_to_wall(self):
        if self._rotation != self._target_rotation:
            diff = abs(self._target_rotation - self._rotation)
            move = self.MAX_ANGULAR * get_sign(self._target_rotation)
            next_rotation = move * self.update_delay
            if abs(next_rotation) > diff:
                move = diff * self.update_delay * get_sign(self._target_rotation)
                next_rotation = diff * get_sign(self._target_rotation)
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
            self._current_ranges.clear()

    def __find_next_point(self, best_wall: Wall, walls: list):
        best_angle = get_angle(best_wall.direction)
        rotated_begin_point = rotate_vector(best_wall.begin_point, best_angle)
        rotated_end_point = rotate_vector(best_wall.end_point, best_angle)
        if rotated_begin_point.x > rotated_end_point.x:
            best_wall_extremity = best_wall.begin_point
        else:
            best_wall_extremity = best_wall.end_point
        best_direction = rotate_vector(best_wall.direction, -90)
        next_point = best_wall_extremity + (best_direction * self._safety_distance)
        if next_point.dist > 8:
            next_point = next_point.normalized * 8
        for wall in (w for w in walls if w != best_wall):
            intersection = get_intersection(Point.zero(), best_wall.direction.normalized, wall.begin_point,
                                            wall.direction)
            if next_point.dist + self._safety_distance > intersection.dist:
                return self.__find_next_point(wall, [w for w in walls if w != best_wall])
        return next_point

    def __follow_wall(self):
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        if self._current_ranges.length < self._current_ranges.max_size:
            return
        self.__reset_location_and_rotation()
        walls = self.__get_walls()
        self.get_logger().info(f'Walls count: {len(walls)}')
        best_wall = self.__get_closer_wall(walls)

        next_point = self.__find_next_point(best_wall, walls)

        # best_angle = get_angle(best_wall.direction)
        # rotated_begin_point = rotate_vector(best_wall.begin_point, best_angle)
        # rotated_end_point = rotate_vector(best_wall.end_point, best_angle)
        # if rotated_begin_point.x > rotated_end_point.x:
        #     best_wall_extremity = best_wall.begin_point
        # else:
        #     best_wall_extremity = best_wall.end_point
        # best_direction = rotate_vector(best_wall.direction, -90)
        # next_point = best_wall_extremity + (best_direction * self._safety_distance)
        # if next_point.dist > 8:
        #     next_point = next_point.normalized * 8
        # for wall in (w for w in walls if w != best_wall):
        #     intersection = get_intersection(Point.zero(), best_wall.direction.normalized, wall.begin_point, wall.direction)
        #     if next_point.dist + self._safety_distance > intersection.dist:
        #         self.info("Tu vas te prendre un mur?")
        #         # TODO Split la fonction, fait en une qui prend le best wall et qui calcule le point, si on arrive ici tu lance la recursive avec wall toucher comme best wall et ainsi de suite mouhahaha
        #         next_point = next_point.normalized * (intersection.dist - self._safety_distance)
        #         break


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
        # self.info(f'{sensor_data}')
        self._current_ranges.append(sensor_data.ranges)

    def odom_callback(self, odometry):
        # self.get_logger().info(f'CMD VEL ROW -> {odometry}')
        return

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
