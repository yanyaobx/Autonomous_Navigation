import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
from .rrt import find_path_RRT

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile)
        self.start_goal_subscriber = self.create_subscription(
            Float64MultiArray,
            '/start_goal',
            self.start_goal_callback,
            qos_profile)
        self.trajectory_publisher = self.create_publisher(
            Float64MultiArray,
            '/trajectory',
            qos_profile)
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None

    def map_callback(self, msg):
        self.get_logger().info('Received map message')
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        self.get_logger().info(f'Map dimensions: {msg.info.width}x{msg.info.height}')
        self.get_logger().info(f'Map resolution: {msg.info.resolution}')
        self.get_logger().info(f'Map origin: {self.map_origin[0]}, {self.map_origin[1]}')
        self.get_logger().info(f'Map data received: {self.map_data.shape}')

    def start_goal_callback(self, msg):
        if self.map_data is None:
            self.get_logger().error('Map data has not been received yet.')
            return

        start = [msg.data[0], msg.data[1]]
        goal = [msg.data[2], msg.data[3]]

        # Convert start and goal from meters to pixels
        start_px = [(coord - self.map_origin[i % 2]) / self.map_resolution for i, coord in enumerate(start)]
        goal_px = [(coord - self.map_origin[i % 2]) / self.map_resolution for i, coord in enumerate(goal)]

        self.get_logger().info(f'Received start and goal points: {msg.data}')
        self.get_logger().info(f'Start index: {start_px}, Goal index: {goal_px}')
        path, graph = self.find_path_rrt(start_px, goal_px)
        
        if path:
            self.get_logger().info(f'Path found: {path}')
            trajectory_msg = Float64MultiArray()
            trajectory_msg.data = [coord for point in path for coord in point]
            self.trajectory_publisher.publish(trajectory_msg)
            self.get_logger().info('Published trajectory')
        else:
            self.get_logger().warn('Path not found')

    def find_path_rrt(self, start, goal):
        if self.map_data is None:
            self.get_logger().error('No map data available')
            return None, None
        path, graph = find_path_RRT(start, goal, self.map_data)
        return path, graph

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

