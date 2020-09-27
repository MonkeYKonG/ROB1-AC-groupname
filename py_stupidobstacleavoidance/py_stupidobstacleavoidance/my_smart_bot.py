import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class TheSmartestBotIveEverSeen(Node):
    STATE_SEARCH_WALL = 0
    STATE_ROTATION_LEFT = 1
    STATE_ROTATION_RIGHT = 2
    STATE_FOLLOW = 3

    def __init__(self):
        super().__init__('the_smartest_bot_i_ve_ever_seen')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish)
        self.linear_factor = 0.0
        self.angular_factor = 0.0
        self.sensor_angular_range = 30
        self.state = self.STATE_SEARCH_WALL

    def ranges_is_closer(self, ranges, min_range=0.5):
        for range in ranges:
            if range < min_range:
                return True
        return False

    def compute_angular_factor(self, ranges):
        ranges_len = int(len(ranges) / 3)
        left_wall = self.ranges_is_closer(ranges[:ranges_len])
        right_wall = self.ranges_is_closer(ranges[ranges_len * 2:ranges_len])
        angular_return = -1.0
        if left_wall and right_wall is False:
            angular_return = 1.0
        return angular_return

    def search_for_wall(self, sensor_data):
        if sensor_data.ranges[0] < 0.4:
            self.state = self.STATE_ROTATION_LEFT
            self.get_logger().info(f'{sensor_data.ranges[315]} - {sensor_data.ranges[0]} - {sensor_data.ranges[45]}')
        else:
            dist_to_reach = sensor_data.ranges[0] - 0.4
            self.linear_factor = 1.0 * (1 if dist_to_reach > 1 else dist_to_reach)

    def rotate_to_left(self, sensor_data):
        diff = abs(sensor_data.ranges[45] - sensor_data.ranges[135])
        self.angular_factor = 0.1
        self.get_logger().info(f'{sensor_data.ranges[45]} - {sensor_data.ranges[90]} - {sensor_data.ranges[135]} = {diff}')

    def rotate_to_right(self, sensor_data):
        pass

    def listener_callback(self, sensor_data):
        sensor_data_len = len(sensor_data.ranges)
        half_angular_range = int(self.sensor_angular_range / 2)
        front_ranges = sensor_data.ranges[sensor_data_len - half_angular_range:] + sensor_data.ranges[:half_angular_range]
        front_right_ranges = sensor_data.ranges[half_angular_range:90-half_angular_range]
        right_ranges = sensor_data.ranges[90-half_angular_range:90+half_angular_range]
        right_bottom_ranges = sensor_data.ranges[90+half_angular_range:180-half_angular_range]
        front_range_closer = self.ranges_is_closer(front_ranges)
        front_right_range_closer = self.ranges_is_closer(front_right_ranges)
        right_range_closer = self.ranges_is_closer(right_ranges)
        right_bottom_range_closer = self.ranges_is_closer(right_bottom_ranges)

        self.linear_factor = 0.0
        self.angular_factor = 0.0
        #self.get_logger().info('salut')

        if self.state == self.STATE_SEARCH_WALL:
            self.search_for_wall(sensor_data)
        elif self.state == self.STATE_ROTATION_LEFT:
            self.rotate_to_left(sensor_data)
        elif self.state == self.STATE_ROTATION_RIGHT:
            self.rotate_to_right(sensor_data)
        else:
            pass

    def publish(self):
        msg = Twist()
        msg.linear.x = self.linear_factor
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_factor
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = TheSmartestBotIveEverSeen()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()