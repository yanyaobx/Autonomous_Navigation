import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np

class MotionPlanner(Node):
    def __init__(self):
        super().__init__('motion_planner')
        qos_profile = self.get_qos_profile()
        self.start_goal_publisher = self.create_publisher(
            Float64MultiArray,
            '/start_goal',
            qos_profile)
        self.create_subscription(
            Float64MultiArray,
            '/trajectory',
            self.trajectory_callback,
            10)

    def get_qos_profile(self):
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos

    def trajectory_callback(self, msg):
        print("Received trajectory")
        trajectory = np.array(msg.data).reshape(-1, 2)
        print("Received trajectory:")
        print(trajectory)

    def publish_start_goal(self, start, goal):
        start_goal_msg = Float64MultiArray()
        start_goal_msg.data = start + goal
        self.start_goal_publisher.publish(start_goal_msg)
        print(f"Published start {start} and goal {goal} points")

def main(args=None):
    rclpy.init(args=args)
    motion_planner = MotionPlanner()

    start_x = float(input("Enter start x: "))
    start_y = float(input("Enter start y: "))
    goal_x = float(input("Enter goal x: "))
    goal_y = float(input("Enter goal y: "))

    start = [start_x, start_y]
    goal = [goal_x, goal_y]

    motion_planner.publish_start_goal(start, goal)

    rclpy.spin(motion_planner)
    motion_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

