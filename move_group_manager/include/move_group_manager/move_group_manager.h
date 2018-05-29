#ifndef MOVE_GROUP_MANAGER_MOVE_GROUP_MANAGER_H
#define MOVE_GROUP_MANAGER_MOVE_GROUP_MANAGER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>

namespace move_group_manager
{

class MoveGroupManager
{

public:

  MoveGroupManager();

  bool plan(const geometry_msgs::Pose& pose, const std::string& eef_link,
    moveit::planning_interface::MoveGroup::Plan& plan);

  bool planCartesianPath(const std::vector<geometry_msgs::Pose>& poses,
    moveit::planning_interface::MoveGroup::Plan& plan);

  bool execute(const moveit::planning_interface::MoveGroup::Plan& plan);

  bool pick(const geometry_msgs::Pose& grasp_pose, const geometry_msgs::Vector3& approach);

private:

  ros::NodeHandle nh_;
  ros::CallbackQueue call_back_queue_;
  boost::shared_ptr<ros::AsyncSpinner> async_spinner_ptr_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> group_ptr_;

};

}

#endif