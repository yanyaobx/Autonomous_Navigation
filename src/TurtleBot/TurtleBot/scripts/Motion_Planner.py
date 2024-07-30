#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import numpy as np

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.trajectory_subscriber = self.create_subscription(Float64MultiArray, '/trajectory', self.trajectory_callback, 10)
        self.target_pose_subscriber = self.create_subscription(Float64MultiArray, '/target_pose', self.target_pose_callback, 10)
        self.start_goal_publisher = self.create_publisher(Float64MultiArray, '/start_goal', 10)
        self.reference_pose_publisher = self.create_publisher(Float64MultiArray, '/reference_pose', 10)
        self.current_pose = None
        self.target_pose = None
        self.trajectory = None
        self.trajectory_index = 0
        
        self.start_position = [200.0, 230.0]

        
        

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.trajectory:
            self.follow_trajectory()

    def target_pose_callback(self, msg):
        self.target_pose = msg.data
        if self.current_pose and self.target_pose:
            self.send_start_goal()

    def send_start_goal(self):
        start_goal_msg = Float64MultiArray()
        start_goal_msg.data = [
            self.current_pose.position.x, self.current_pose.position.y,
            self.target_pose[0], self.target_pose[1]
        ]
        self.start_goal_publisher.publish(start_goal_msg)
        self.get_logger().info(f'Sent start and goal: {start_goal_msg.data}')

    def trajectory_callback(self, msg):
        self.trajectory = np.array(msg.data).reshape(-1, 2)
        self.trajectory_index = 0
        if self.trajectory.size > 0:
            self.send_reference_pose()

    def follow_trajectory(self):
        if self.trajectory_index < len(self.trajectory):
            target = self.trajectory[self.trajectory_index]
            distance = self.get_distance(self.current_pose.position, target)
            if distance < 0.1:
                self.trajectory_index += 1
                self.send_reference_pose()

    def send_reference_pose(self):
        if self.trajectory_index < len(self.trajectory):
            next_pose = self.trajectory[self.trajectory_index]
            reference_pose_msg = Float64MultiArray()
            reference_pose_msg.data = [next_pose[0], next_pose[1], 0.0]
            self.reference_pose_publisher.publish(reference_pose_msg)
            self.get_logger().info(f'Sent reference pose: {reference_pose_msg.data}')

    def get_distance(self, position, target):
        return np.sqrt((position.x - target[0])**2 + (position.y - target[1])**2)

def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanner()
    rclpy.spin(motion_planner)
    motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

