cmake_minimum_required(VERSION 2.8)
project(table_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  eigen_conversions
)

find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES table_detection
# CATKIN_DEPENDS roscpp
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(table_detection src/table_detection.cpp)
target_link_libraries(table_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(test_table_detection_node src/test_table_detection_node.cpp)
target_link_libraries(test_table_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} table_detection)