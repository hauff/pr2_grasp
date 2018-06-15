cmake_minimum_required(VERSION 2.8.3)
project(pr2_grasp)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  visualization_msgs
  #pcl_conversions
  eigen_conversions

  #moveit_core
  #moveit_ros_planning
  moveit_ros_planning_interface
  gpd

  planning_scene_manager
  move_group_manager
)

find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES
  CATKIN_DEPENDS roscpp
# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pr2_grasp_node src/pr2_grasp_node.cpp)
target_link_libraries(pr2_grasp_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})