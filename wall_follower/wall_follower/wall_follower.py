import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollower(Node):
    MAX_LINEAR = 1.0
    MIN_LINEAR = 0.0
    MAX_ANGULAR = 1.0
    MIN_ANGULAR = 0.0

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

    def scan_sensor_callback(self, sensor_data):
        # TODO Strat is to always have a wall on my right
        # TODO Normal step is wall on right
        # TODO Detect virage a gauche
        # TODO Detect virage a droite
        # TODO Faire un parcours main droite
        pass


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = WallFollower()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
