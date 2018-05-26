cmake_minimum_required(VERSION 2.8.3)
project(pr2_grasp)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  eigen_conversions

  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  gpd

  table_detection
)

find_package(PCL 1.8 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(Caffe REQUIRED)
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS})

find_library(GENERATOR_LIB grasp_candidates_generator)
find_path(GENERATOR_LIB_INCLUDE_DIR gpg/grasp.h)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES table_detection
  CATKIN_DEPENDS cmake_modules roscpp gpd table_detection
# DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GENERATOR_LIB_INCLUDE_DIR}
)

add_executable(pr2_grasp_node src/pr2_grasp_node.cpp)
target_link_libraries(pr2_grasp_node gpd_grasp_detector ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${GENERATOR_LIB})