cmake_minimum_required(VERSION 3.5)
project(TurtleBot)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(PROGRAMS
  scripts/PID_Controller.py
  scripts/Motion_Planner.py
  scripts/RRT_Node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
