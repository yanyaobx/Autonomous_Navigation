#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(Float64MultiArray, '/reference_pose', self.listener_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.reference_pose = None
        self.current_pose = None

        # PID constants
        self.kp_linear = 1.0
        self.ki_linear = 0.0
        self.kd_linear = 0.0
        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        # PID errors
        self.prev_linear_error = 0.0
        self.prev_angular_error = 0.0
        self.integral_linear_error = 0.0
        self.integral_angular_error = 0.0

    def listener_callback(self, msg):
        self.reference_pose = msg.data

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        if self.reference_pose:
            self.control_loop()

    def control_loop(self):
        if self.reference_pose and self.current_pose:
            x_ref, y_ref, theta_ref = self.reference_pose
            x_cur, y_cur = self.current_pose.position.x, self.current_pose.position.y
            theta_cur = self.get_yaw_from_quaternion(self.current_pose.orientation)

            # Linear error
            distance_error = np.sqrt((x_ref - x_cur) ** 2 + (y_ref - y_cur) ** 2)

            # Angular error
            target_angle = np.arctan2(y_ref - y_cur, x_ref - x_cur)
            angle_error = self.normalize_angle(target_angle - theta_cur)

            # PID control
            linear_vel = self.kp_linear * distance_error
            angular_vel = self.kp_angular * angle_error

            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.publisher.publish(cmd)

    def get_yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


