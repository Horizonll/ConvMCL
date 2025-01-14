cmake_minimum_required(VERSION 3.5)
project(nav2_system_tests)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
#find_package(gazebo_ros_pkgs REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(navigation2)
find_package(angles REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

nav2_package()

set(dependencies
  rclcpp
  nav2_util
  nav2_map_server
  nav2_msgs
  nav_msgs
  visualization_msgs
  nav2_amcl
  nav2_lifecycle_manager
  nav2_behavior_tree
  gazebo_ros_pkgs
  geometry_msgs
  std_msgs
  tf2_geometry_msgs
  rclpy
  nav2_planner
  nav2_navfn_planner
  angles
  behaviortree_cpp_v3
)



ament_package()
