#ifndef PLANNING_SCENE_MANAGER_PLANNING_SCENE_MANAGER_H
#define PLANNING_SCENE_MANAGER_PLANNING_SCENE_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <std_msgs/Header.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>

namespace planning_scene_manager
{

class PlanningSceneManager
{

public:

  PlanningSceneManager();

  void allowCollision(const std::string& id);

  void addBoxCollisionObject(const std::string& frame_id, const std::string& id,
    const Eigen::Vector3d& dimensions, const Eigen::Affine3d& pose);

private:

  void getAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& acm);

  void setPlanningScene(const moveit_msgs::PlanningScene& scene);

  void addCollisionObject(const moveit_msgs::CollisionObject& object);

  void initHeader(const ros::Time& stamp, const std::string& frame_id, std_msgs::Header& header);

  void initSolidPrimitive(const int& type, const std::vector<double>& dimensions,
    shape_msgs::SolidPrimitive& primitive);

  ros::NodeHandle nh_;
  ros::ServiceClient client_get_scene_;
  ros::ServiceClient client_set_scene_;

};

}

#endif