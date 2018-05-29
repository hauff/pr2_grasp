#include <move_group_manager/move_group_manager.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace move_group_manager
{

MoveGroupManager::MoveGroupManager()
{
  //nh_.setCallbackQueue(&call_back_queue_);
  //async_spinner_ptr_.reset(new ros::AsyncSpinner(0, &call_back_queue_));
  async_spinner_ptr_.reset(new ros::AsyncSpinner(8));
  group_ptr_.reset(new moveit::planning_interface::MoveGroup("right_arm"));
  group_ptr_->setGoalTolerance(0.01);
  //group_ptr_->setNumPlanningAttempts(3);
  group_ptr_->setPlanningTime(5.0);
}

bool MoveGroupManager::plan(const geometry_msgs::Pose& pose, const std::string& eef_link,
  moveit::planning_interface::MoveGroup::Plan& plan)
{
  group_ptr_->setPoseTarget(pose, eef_link);

  async_spinner_ptr_->start();
  bool result = group_ptr_->plan(plan);
  async_spinner_ptr_->stop();

  return result;
}

bool MoveGroupManager::planCartesianPath(const std::vector<geometry_msgs::Pose>& poses,
  moveit::planning_interface::MoveGroup::Plan& plan)
{
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group_ptr_->computeCartesianPath(poses, 0.01, 0.0, trajectory);
  plan.trajectory_ = trajectory;

  async_spinner_ptr_->start();
  bool result = group_ptr_->plan(plan);
  async_spinner_ptr_->stop();

  return result;
}

bool MoveGroupManager::execute(const moveit::planning_interface::MoveGroup::Plan& plan)
{
  async_spinner_ptr_->start();
  bool result = group_ptr_->execute(plan);
  async_spinner_ptr_->stop();

  return result;
}

bool MoveGroupManager::pick(const geometry_msgs::Pose& grasp_pose, const geometry_msgs::Vector3& approach)
{
  moveit_msgs::Grasp grasp;

  grasp.grasp_pose.header.frame_id = "odom_combined";
  grasp.grasp_pose.pose = grasp_pose;

  grasp.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
  grasp.pre_grasp_approach.direction.vector = approach;
  grasp.pre_grasp_approach.min_distance = 0.1;
  grasp.pre_grasp_approach.desired_distance = 0.25;

  grasp.post_grasp_retreat.direction.header.frame_id = "odom_combined";
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.1;
  grasp.post_grasp_retreat.desired_distance = 0.25;

  grasp.allowed_touch_objects.push_back("object");

  async_spinner_ptr_->start();
  bool result = group_ptr_->pick("object", grasp);
  async_spinner_ptr_->stop();

  return result;
}

}