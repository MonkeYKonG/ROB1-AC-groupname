import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class TheSmartestBotIveEverSeen(Node):
    def __init__(self):
        super().__init__('my_smart_bot')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            rclpy.qos.qos_profile_sensor_data)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish)
        
        self.linear_factor = 0.3
        self.angular_factor = 0.0
        self.sensor_angular_range = 30
        self.on_rotation = False

    def ranges_is_closer(self, ranges, min_range=0.5):
        for r in ranges:
            if r < min_range:
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

    def listener_callback(self, sensor_data):
        sensor_data_len = len(sensor_data.ranges)
        half_angular_range = int(self.sensor_angular_range / 2)
        ranges = sensor_data.ranges[sensor_data_len - half_angular_range:] + sensor_data.ranges[:half_angular_range]
        if self.ranges_is_closer(ranges):
            if self.on_rotation is False:
                self.linear_factor = 0.0
                self.angular_factor = self.compute_angular_factor(ranges)
                self.on_rotation = True
        elif self.on_rotation:
            self.linear_factor = 0.3
            self.angular_factor = 0.0
            self.on_rotation = False

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
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()