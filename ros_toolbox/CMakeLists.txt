cmake_minimum_required(VERSION 2.8.3)
project(ros_toolbox)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  #INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(transform_clouds_node src/transform_clouds_node.cpp)
target_link_libraries(transform_clouds_node ${catkin_LIBRARIES})
