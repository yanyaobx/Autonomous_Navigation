import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
import numpy as np
import matplotlib.pyplot as plt

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        qos_profile = self.get_qos_profile()
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile)
        self.trajectory_subscription = self.create_subscription(
            Float64MultiArray,
            '/trajectory',
            self.trajectory_callback,
            10)
        self.current_map = None
        self.map_resolution = None
        self.map_origin = None

    def get_qos_profile(self):
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        return qos
        
    def map_callback(self, msg):
        print("Received map message")
        width = msg.info.width
        height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        print("Map Info: Width={}, Height={}, Resolution={}".format(width, height, self.map_resolution))
        data = msg.data

        # Convert 1D occupancy grid data to 2D numpy array
        self.current_map = np.array(data).reshape((height, width))

        # Plot the occupancy grid
        self.plot_map()

    def trajectory_callback(self, msg):
        print("Received trajectory")
        trajectory = np.array(msg.data).reshape(-1, 2)
        print("Received trajectory:")
        print(trajectory)
        self.plot_trajectory(trajectory)

    def plot_map(self):
        if self.current_map is not None:
            plt.imshow(self.current_map, cmap='binary', origin='lower')
            plt.colorbar()
            plt.title('Occupancy Grid')
            plt.xlabel('X (cells)')
            plt.ylabel('Y (cells)')
            plt.show()

    def plot_trajectory(self, trajectory):
        if self.current_map is not None:
            plt.imshow(self.current_map, cmap='binary', origin='lower')
            plt.colorbar()
            plt.title('Occupancy Grid with Trajectory')
            plt.xlabel('X (cells)')
            plt.ylabel('Y (cells)')
            
            # Convert trajectory from pixels back to meters
            trajectory_m = [(point[0] * self.map_resolution + self.map_origin[0], 
                             point[1] * self.map_resolution + self.map_origin[1]) for point in trajectory]
            trajectory_m = np.array(trajectory_m)
            
            plt.plot(trajectory_m[:, 0], trajectory_m[:, 1], 'o-', color='red')
            plt.show()

def main(args=None):
    rclpy.init(args=args)
    map_subscriber = MapSubscriber()
    rclpy.spin(map_subscriber)
    map_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

