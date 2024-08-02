# Autonomous Systems Project

---

## Project Overview

This project involves the development of an autonomous navigation system using ROS2 (Robot Operating System) and TurtleBot3. The project integrates various components, including motion planning, path planning, and control algorithms, to create a fully functional autonomous robot. The final implementation is capable of navigating a simulated environment using a combination of the Rapidly-exploring Random Tree (RRT) algorithm and a PID controller.

## Objectives

- Develop a complete autonomous navigation system.
- Implement and integrate motion planning, path planning, and control algorithms.
- Use ROS2 and Gazebo for simulation and testing.
- Demonstrate the robot's ability to navigate to various target points autonomously.

## Implementation Details

### Motion Planner Node

The `Motion_Planner.py` script implements the core motion planning functionality. It handles the interaction between the robot's current position, the target pose, and the computed trajectory to guide the robot towards its goal.

**Features:**
- Subscribes to `/odom`, `/trajectory`, and `/target_pose` topics.
- Publishes to `/start_goal` and `/reference_pose` topics.
- Handles receiving the current robot position, computing the trajectory using the RRT algorithm, and guiding the robot towards its goal using a PID controller.

**Key Methods:**
- `odom_callback`: Updates the robot's current pose.
- `target_pose_callback`: Processes the target pose and initiates path planning.
- `send_start_goal`: Publishes the start and goal positions to the RRT node.
- `trajectory_callback`: Receives and processes the computed trajectory.
- `follow_trajectory`: Sends reference poses to the PID controller to guide the robot along the trajectory.
- `send_reference_pose`: Publishes the next point in the trajectory.
- `get_distance`: Calculates the distance between the robot's current position and the target point.

### PID Controller Node

The `PID_Controller.py` script implements a PID controller for TurtleBot3 in ROS2. It computes and publishes velocity commands to guide the robot to the desired pose.

**Features:**
- Subscribes to `/reference_pose` and `/odom` topics.
- Publishes to `/cmd_vel` topic.
- Computes and publishes velocity commands based on the PID control algorithm.

**Key Methods:**
- `listener_callback`: Updates the reference pose.
- `odom_callback`: Updates the current pose.
- `control_loop`: Calculates and publishes velocity commands based on the current and reference poses.
- `get_yaw_from_quaternion`: Extracts the yaw angle from the robot's orientation.
- `normalize_angle`: Normalizes the angular error.

### RRT Node

The `RRT_Node.py` script implements the RRT algorithm for path planning in ROS2. It computes the trajectory from the start to the goal position and publishes it for the motion planner to follow.

**Features:**
- Subscribes to `/map` and `/start_goal` topics.
- Publishes to `/trajectory` topic.
- Computes and publishes the trajectory using the RRT algorithm.

**Key Methods:**
- `map_callback`: Processes the occupancy grid map.
- `start_goal_callback`: Computes the trajectory upon receiving start and goal positions.
- `compute_rrt_path`: Generates the path using the RRT algorithm.
- `coord_to_index` and `index_to_coord`: Handle conversion between real-world coordinates and map indices.
- `publish_trajectory`: Publishes the computed trajectory.

## How to Run

1. **Set up the environment:**
   - Install Ubuntu and ROS2 Humble.
   - Set up the TurtleBot3 simulator.
   - Create a ROS workspace and clone the project repository.

2. **Start the map server node:**
   ```bash
   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=warehouse.yaml
   ros2 lifecycle set /map_server configure
   ros2 lifecycle set /map_server activate

3. **Start the PID controller node:**
   ```bash
   ros2 run TurtleBot PID_Controller

4. **Start the motion planner node:**
   ```bash
   ros2 run TurtleBot Motion_Planner

5. **Run the RRT node:**
   ```bash
   ros2 run TurtleBot rrt_node


## Evaluation

The project was evaluated by testing the robot's ability to navigate to multiple target points. The system's performance was measured based on the time taken for the robot to move along the computed trajectory. Plots of the generated paths were included in the report, demonstrating the effectiveness of the implemented algorithms.

## Result Demonstration

The results of this project, including example trajectories and performance metrics, are documented in the `Autonomous_Navigation_Result_Show.pdf`. This document provides visual evidence of the robot's navigation capabilities and showcases the successful integration of the developed system.

## Conclusion

This project showcases a comprehensive understanding of autonomous systems, ROS2, and robotic navigation. By successfully integrating motion planning and control algorithms, the system demonstrates the ability to perform autonomous navigation in a simulated environment. The skills and knowledge gained through this project are directly applicable to real-world autonomous systems development.


