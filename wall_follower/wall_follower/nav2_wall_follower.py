import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from .Shapes.point import Point
from .tools import rotation_vector_to_quaternion
import math


def is_unknown(cell_value):
    return cell_value == -1


def is_wall(cell_value):
    return cell_value > 0


def is_empty(cell_value):
    return cell_value == 0


class Nav2WallFollower(Node):
    def __init__(self, debug=False):
        super().__init__('nav2_wall_follower')

        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.__map_callback,
            rclpy.qos.qos_profile_sensor_data
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.__odom_callback,
            rclpy.qos.QoSProfile(depth=10)
        )
        self.tf_subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.__tf_callback,
            rclpy.qos.QoSProfile(depth=10)
        )
        self.nav2_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.update_delay = 10
        self.update_timer = self.create_timer(self.update_delay, self.__update)
        self.map_info = None
        self.map_data = None
        self.real_world_location = None
        self.map_size = None
        self.map_origin = None
        self.map_resolution = None
        self.next_target_location = None
        self.odom_estimated_location = None
        self.footprint_estimated_location = None

    def map_location_to_map_indexes(self, world_location: Point) -> Point:
        index_x = int((world_location.x - self.map_origin.x) // self.map_resolution)
        index_y = int((world_location.y - self.map_origin.y) // self.map_resolution)
        return Point(index_x, index_y)

    def map_indexes_to_map_location(self, map_indexes: Point) -> Point:
        return (map_indexes * self.map_resolution) + self.map_origin

    def get_map_personal_index(self) -> Point:
        return self.map_location_to_map_indexes(self.footprint_estimated_location)

    def __get_next_cells(self, cell_index):
        next_cells = []
        if cell_index.x > 0:
            next_cells.append(Point(cell_index.x - 1, cell_index.y))
        if cell_index.x < self.map_size.x - 1:
            next_cells.append(Point(cell_index.x + 1, cell_index.y))
        if cell_index.y > 0:
            next_cells.append(Point(cell_index.x, cell_index.y - 1))
        if cell_index.y < self.map_size.y - 1:
            next_cells.append(Point(cell_index.x, cell_index.y + 1))
        return next_cells

    def __search_point_on_map(self, first_map_index: Point, match_function, min_dist=40):
        cells_to_evaluate = [first_map_index]
        cells_evaluated = [first_map_index]
        index = 0
        while len(cells_to_evaluate):
            cell_index = cells_to_evaluate.pop(0)
            if match_function(cell_index) and (cell_index - first_map_index).dist > min_dist:
                return cell_index
            if self.map_data[cell_index.y][cell_index.x] > 50:
                continue
            for cell in self.__get_next_cells(cell_index):
                if cell not in cells_evaluated:
                    cells_evaluated.append(cell)
                    cells_to_evaluate.append(cell)
            index += 1
        print('No point found')
        return None

    def __get_closer_point(self, match_function) -> Point:
        map_index = self.get_map_personal_index()
        if map_index.x < 0 or map_index.x >= self.map_data.shape[1] or map_index.y < 0 or map_index.y >= self.map_data.shape[0]:
            return Point.zero()
        return self.__search_point_on_map(map_index, match_function)

    def is_square(self, square_center, square_size, match_function):
        half_square_size = square_size // 2
        if square_center.x < half_square_size or square_center.x > self.map_size.x - half_square_size:
            return False
        if square_center.y < half_square_size or square_center.y > self.map_size.y - half_square_size:
            return False
        if not match_function(self.map_data[square_center.y][square_center.x]):
            return False
        for i in range(square_size):
            for j in range(square_size):
                if not match_function(self.map_data[square_center.y + i - half_square_size][square_center.x + j - half_square_size]):
                    return False
        return True

    def __update(self):
        if not self.map_info or not self.footprint_estimated_location:
            return
        print('self:', self.get_map_personal_index(), '->', self.map_size)
        closer_point_index = self.__get_closer_point(lambda cell_index: self.is_square(cell_index, 10, is_unknown))
        if not closer_point_index:
            return
        print('next:', closer_point_index)
        next_location = self.map_indexes_to_map_location(closer_point_index)
        self.__publish_next_location(next_location)

    def __map_callback(self, map_result):
        self.map_info = map_result.info
        self.map_size = Point(map_result.info.width, map_result.info.height)
        self.map_data = np.array(map_result.data).reshape(self.map_size.to_array()[::-1])
        self.map_origin = Point(map_result.info.origin.position.x, map_result.info.origin.position.y)
        self.map_resolution = map_result.info.resolution

    def __odom_callback(self, odometry):
        self.real_world_location = Point(odometry.pose.pose.position.x, odometry.pose.pose.position.y)

    def __tf_callback(self, tf):
        for transform in tf.transforms:
            if transform.header.frame_id == 'map':
                if transform.child_frame_id == 'odom':
                    location = transform.transform.translation
                    self.odom_estimated_location = Point(location.x, location.y)
            elif transform.header.frame_id == 'odom':
                if transform.child_frame_id == 'base_footprint' and self.odom_estimated_location:
                    location = transform.transform.translation
                    self.footprint_estimated_location = Point(location.x, location.y) + self.odom_estimated_location

    def __publish_next_location(self, next_location):
        next_loc = PoseStamped()
        next_loc.header.frame_id = 'map'
        next_loc.pose.position.x = next_location.x
        next_loc.pose.position.y = next_location.y
        diff = next_location - self.footprint_estimated_location
        angle = (math.atan2(diff.x, diff.y) + 3 * math.pi / 2)
        next_loc.pose.orientation.z = math.sin(angle / 2)
        next_loc.pose.orientation.w = math.cos(angle / 2)
        self.nav2_publisher.publish(next_loc)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = Nav2WallFollower()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
