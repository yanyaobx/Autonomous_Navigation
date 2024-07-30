#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
import numpy as np
import cv2
from TurtleBot.scripts.rrt import find_path_RRT  

class RRTNode(Node):
    def __init__(self):
        super().__init__('rrt_node')
        self.subscription_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.subscription_start_goal = self.create_subscription(Float64MultiArray, '/start_goal', self.start_goal_callback, 10)
        self.publisher_trajectory = self.create_publisher(Float64MultiArray, '/trajectory', 10)
        self.current_map = None
        self.resolution = None
        self.origin = None

    def map_callback(self, msg):
        self.resolution = msg.info.resolution
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.current_map = map_data
        self.get_logger().info('Map received')

    def start_goal_callback(self, msg):
        start = msg.data[:2]
        goal = msg.data[2:]
        if self.current_map is not None:
            path = self.compute_rrt_path(start, goal)
            self.publish_trajectory(path)

    def compute_rrt_path(self, start, goal):
        # Convert real-world coordinates to map indices
        start_idx = self.coord_to_index(start)
        goal_idx = self.coord_to_index(goal)
        # Convert map to an image
        map_img = self.map_to_image(self.current_map)
        # Use RRT to find a path
        path, _ = find_path_RRT(start_idx, goal_idx, map_img)
        # Convert path back to real-world coordinates
        path_coords = [self.index_to_coord(p) for p in path]
        return path_coords

    def coord_to_index(self, coord):
        x_idx = int((coord[0] - self.origin[0]) / self.resolution)
        y_idx = int((coord[1] - self.origin[1]) / self.resolution)
        return [x_idx, y_idx]

    def index_to_coord(self, index):
        x = index[0] * self.resolution + self.origin[0]
        y = index[1] * self.resolution + self.origin[1]
        return [x, y]

    def map_to_image(self, map_data):
        img = np.zeros_like(map_data, dtype=np.uint8)
        img[map_data == 0] = 255  # Free space
        img[map_data == 100] = 0  # Occupied space
        img[map_data == -1] = 128  # Unknown space
        return img

    def publish_trajectory(self, path):
        msg = Float64MultiArray()
        msg.data = [coord for point in path for coord in point]
        self.publisher_trajectory.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    rrt_node = RRTNode()
    rclpy.spin(rrt_node)
    rrt_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

