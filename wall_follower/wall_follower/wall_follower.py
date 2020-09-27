import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math


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


class WallFollower(Node):
    MAX_LINEAR = 1.0
    MIN_LINEAR = 0.0
    MAX_ANGULAR = 1.0
    MIN_ANGULAR = 0.0

    STATE_SEARCH_WALL = 0
    STATE_MOVE_TO_WALL = 1
    STATE_FOLLOW_WALL = 2

    def __init__(self):
        super().__init__('wall_follower')

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_sensor_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self._cur_linear = self.MIN_LINEAR
        self._cur_angular = self.MIN_ANGULAR
        self._cur_state = self.STATE_SEARCH_WALL

    def __compute_angular(self, angle):
        return self.MAX_ANGULAR * angle / 180

    def __compute_linear(self, dist):
        dist_factor = min(1, abs(dist))
        return self.MAX_LINEAR * get_sign(dist) * dist_factor

    def scan_sensor_callback(self, sensor_data):
        if self._cur_state == self.STATE_SEARCH_WALL:
            self.search_for_wall(sensor_data.ranges)
        elif self._cur_state == self.STATE_MOVE_TO_WALL:
            self.move_to_wall(sensor_data.ranges)
        elif self._cur_state == self.STATE_FOLLOW_WALL:
            self.follow_wall(sensor_data.ranges)

    def search_for_wall(self, ranges):
        closest_sensor_index = get_closest_range_index(ranges)
        if closest_sensor_index in [359, 0, 1]:
            if closest_sensor_index == 0:
                self._cur_angular = self.MIN_ANGULAR
                self.publish()
                self._cur_state = self.STATE_MOVE_TO_WALL
        elif closest_sensor_index < 180:
            self._cur_angular = self.__compute_angular(closest_sensor_index)
            self.publish()
        else:
            index_from_left = 360 - closest_sensor_index
            self._cur_angular = self.__compute_angular(-index_from_left)
            self.publish()

    def move_to_wall(self, ranges):
        dist_to_wall = ranges[0]
        expected_dist = 0.5
        if dist_to_wall > expected_dist:
            self._cur_linear = self.__compute_linear(dist_to_wall - expected_dist)
        elif dist_to_wall < expected_dist:
            self._cur_linear = self.__compute_linear(dist_to_wall - expected_dist)
        self.publish()

    def follow_wall(self, ranges):
        pass

    def publish(self):
        twist = Twist()
        twist.linear.x = self._cur_linear
        twist.angular.z = self._cur_angular
        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = WallFollower()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
