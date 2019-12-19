cmake_minimum_required(VERSION 2.8.3)
project(pr2_grasp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_eigen
  sensor_msgs
  geometry_msgs
  sensor_msgs
  geometry_msgs
  shape_msgs
  moveit_msgs
  eigen_conversions
  pcl_conversions
  gpd
  planning_scene_manager
  move_group_manager
  #pr2_gripper_sensor_action
)

find_library(GENERATOR_LIB grasp_candidates_generator)
if (NOT GENERATOR_LIB)
  message(FATAL_ERROR "Could not find grasp_candidates_generator library.")
endif ()

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES
  CATKIN_DEPENDS roscpp gpd planning_scene_manager move_group_manager
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GENERATOR_LIB_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(grasp_demo_node src/grasp_demo_node.cpp)
target_link_libraries(grasp_demo_node ${catkin_LIBRARIES} ${GENERATOR_LIB} ${PCL_LIBRARIES})