#include <planning_scene_manager/planning_scene_manager.h>
#include <move_group_manager/move_group_manager.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <gpg/cloud_camera.h>
#include <gpd/grasp_detector.h>
#include <gpd/grasp_plotter.h>

// pr2 gripper sensor ------------------------------------------------------------------------------
/*
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>
#include <actionlib/client/simple_action_client.h>
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction> GrabClient;
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperReleaseAction> ReleaseClient;
*/
//--------------------------------------------------------------------------------------------------

void set_gpd_params(ros::NodeHandle& nh, const moveit_msgs::CollisionObject& table,
  const moveit_msgs::CollisionObject& placing_bin)
{
  const double THRESH = 0.02;

  geometry_msgs::Pose table_pose = table.primitive_poses.at(0);
  std::vector<double> table_dims = table.primitives.at(0).dimensions;

  geometry_msgs::Pose placing_bin_pose = placing_bin.primitive_poses.at(0);
  std::vector<double> placing_bin_dims = placing_bin.primitives.at(0).dimensions;

  std::vector<double> workspace(6);
  workspace[0] = table_pose.position.x - 0.5 * table_dims[0];
  workspace[1] = table_pose.position.x + 0.5 * table_dims[0];
  workspace[2] = placing_bin_pose.position.y + 0.5 * placing_bin_dims[1] + THRESH;
  workspace[3] = table_pose.position.y + 0.5 * table_dims[1];
  workspace[4] = table_pose.position.z - 0.5 * table_dims[2] - THRESH;
  workspace[5] = table_pose.position.z + 1.0;

  nh.setParam("workspace", workspace);
  nh.setParam("workspace_grasps", workspace);
  nh.setParam("table_height", table.primitive_poses.at(0).position.z);
}

void callback_cloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr,
  pcl::PointCloud<pcl::PointXYZRGB>& cloud_out)
{
  pcl::fromROSMsg(*cloud_msg_ptr, cloud_out);
}

bool transform_cloud(const pcl::PointCloud<pcl::PointXYZRGB>& cloud_in,
  const tf2_ros::Buffer& tf_buffer, const std::string& frame_id, Eigen::Affine3d& transform,
  pcl::PointCloud<pcl::PointXYZRGB>& cloud_out)
{
  try
  {
    geometry_msgs::TransformStamped transform_msg = tf_buffer.lookupTransform(
      frame_id, cloud_in.header.frame_id, pcl_conversions::fromPCL(cloud_in.header.stamp));

    transform = tf2::transformToEigen(transform_msg);
    pcl::transformPointCloud(cloud_in, cloud_out, transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  return true;
}

bool detect_grasps(const Eigen::Vector3d& view_point, const pcl::PointCloud<pcl::PointXYZRGB> cloud,
  GraspDetector& grasp_detector, std::vector<Grasp>& grasps)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::copyPointCloud(cloud, *cloud_ptr);

  Eigen::Matrix3Xd view_points(3, 1);
  view_points.col(0) = view_point;
  CloudCamera cloud_camera(cloud_ptr, 0, view_points);

  grasp_detector.preprocessPointCloud(cloud_camera);
  grasps = grasp_detector.detectGrasps(cloud_camera);

  if (grasps.empty())
  {
    ROS_WARN("Could not detect any grasps.");
    return false;
  }

  return true;
}

bool grasp(planning_scene_manager::PlanningSceneManager scene_mgr,
  move_group_manager::MoveGroupManager& group_mgr, const std::vector<Grasp> grasps)
{
  for (size_t i = 0; i < grasps.size(); i++)
  {
    Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
    object_pose.translation() = grasps[i].getGraspBottom();
    
    Eigen::Affine3d grasp_pose = Eigen::Affine3d::Identity();
    grasp_pose.translation() = -0.14 * grasps[i].getApproach() + grasps[i].getGraspBottom();
    grasp_pose.linear() << grasps[i].getApproach(), grasps[i].getBinormal(), grasps[i].getAxis();

    geometry_msgs::Pose grasp_pose_msg;
    tf::poseEigenToMsg(grasp_pose, grasp_pose_msg);

    geometry_msgs::Vector3 approach_msg;
    tf::vectorEigenToMsg(grasps[i].getApproach(), approach_msg);

    scene_mgr.addSphereCollisionObject("odom_combined", "object", object_pose, 0.10);
    scene_mgr.allowCollision("object");

    if (group_mgr.pickGently(grasp_pose_msg, approach_msg))
    {
      return true;
    }
    else
    {
      scene_mgr.removeCollisionObject("r_wist_roll_link", "object");
      scene_mgr.removeCollisionObject("odom_combined", "object");
    }
  }

  ROS_WARN("Tried all grasps without success.");
  return false;
}

void place(planning_scene_manager::PlanningSceneManager scene_mgr,
  move_group_manager::MoveGroupManager& group_mgr, const moveit_msgs::CollisionObject& placing_bin)
{
  geometry_msgs::Pose place_pose_msg;
  place_pose_msg.position.x = placing_bin.primitive_poses.at(0).position.x - 0.2;
  place_pose_msg.position.y = placing_bin.primitive_poses.at(0).position.y;
  place_pose_msg.position.z = placing_bin.primitive_poses.at(0).position.z + 0.4;
  place_pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0.25 * M_PI, 0);

  moveit::planning_interface::MoveGroup::Plan plan;
  if (group_mgr.plan("odom_combined", "r_wrist_roll_link", place_pose_msg, plan))
  {
    group_mgr.execute(plan);
  }
  else
    ROS_WARN("Place object failed.");

  //group_mgr.openGripper();
  scene_mgr.removeCollisionObject("r_wist_roll_link", "object");
  scene_mgr.removeCollisionObject("odom_combined", "object");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp_demo");
  ros::NodeHandle nh_public, nh_private("~");

  std::string topic_in, base_frame_id = "odom_combined";
  nh_private.param<std::string>("topic_in", topic_in, "");

  if (topic_in.empty())
  {
    ROS_ERROR("Parameter missing: 'topic_in'");
    return EXIT_FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  ros::Subscriber sub_cloud = nh_public.subscribe<sensor_msgs::PointCloud2>(
    topic_in, 1, boost::bind(callback_cloud, _1, boost::ref(cloud)));

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  planning_scene_manager::PlanningSceneManager scene_mgr;
  move_group_manager::MoveGroupManager group_mgr;

  moveit_msgs::CollisionObject table, placing_bin;
  if (!scene_mgr.getCollisionObject("table", table) ||
      !scene_mgr.getCollisionObject("placing_bin", placing_bin))
    return EXIT_FAILURE;

  geometry_msgs::Pose place_pose_msg;
  place_pose_msg.position.x = placing_bin.primitive_poses.at(0).position.x - 0.2;
  place_pose_msg.position.y = placing_bin.primitive_poses.at(0).position.y;
  place_pose_msg.position.z = placing_bin.primitive_poses.at(0).position.z + 0.4;
  place_pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0.25 * M_PI, 0);

  //scene_mgr.allowCollision("table");

  set_gpd_params(nh_private, table, placing_bin);

  GraspDetector grasp_detector(nh_private);
  GraspPlotter grasp_plotter(nh_private, grasp_detector.getHandSearchParameters());

  // pr2 gripper sensor ----------------------------------------------------------------------------
  /*
  GrabClient grab_client("r_gripper_sensor_controller/grab", true);
  while(!grab_client.waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the r_gripper_sensor_controller/grab action server to come up.");

  ReleaseClient release_client("r_gripper_sensor_controller/release", true);
  while(!release_client.waitForServer(ros::Duration(5.0)))
    ROS_INFO("Waiting for the r_gripper_sensor_controller/release action server to come up.");
    */
  //------------------------------------------------------------------------------------------------

  ROS_WARN("Moving to place pose.");
  moveit::planning_interface::MoveGroup::Plan plan;
  if (!group_mgr.plan("odom_combined", "r_wrist_roll_link", place_pose_msg, plan) ||
      !group_mgr.execute(plan) ||
      !group_mgr.openGripperGently())
  {
    ROS_ERROR("Failure while moving to place pose.");
    return EXIT_FAILURE;
  }

  ros::Rate rate(10);
  while (ros::ok())
  {
    ROS_INFO("Wait for new point cloud ...");
    ros::Time time = ros::Time::now();
    while (ros::ok())
    {
      ros::spinOnce();
      if (cloud.header.stamp >= pcl_conversions::toPCL(time))
        break;

      rate.sleep();
    }

    ROS_INFO("Transform point cloud to frame '%s'.", base_frame_id.c_str());
    Eigen::Affine3d transform;
    if (!transform_cloud(cloud, tf_buffer, base_frame_id, transform, cloud))
      continue;

    ROS_INFO("Detect grasps.");
    std::vector<Grasp> grasps;
    if (!detect_grasps(transform.translation(), cloud, grasp_detector, grasps))
    {
      /*
      ROS_WARN("Moving back to place pose.");
      moveit::planning_interface::MoveGroup::Plan plan;
      if (!group_mgr.plan("odom_combined", "r_wrist_roll_link", place_pose_msg, plan) ||
          !group_mgr.execute(plan))
        ROS_ERROR("Failure while moving to place pose.");
      */
      continue;
    }

    ROS_INFO("Publish grasps to Rviz.");
    grasp_plotter.drawGrasps(grasps, base_frame_id);

    ROS_INFO("Grasp object.");
    if (!grasp(scene_mgr, group_mgr, grasps))
    {
      ROS_WARN("Open gripper.");
      group_mgr.openGripperGently();

      ROS_WARN("Moving back to place pose.");
      moveit::planning_interface::MoveGroup::Plan plan;
      if (!group_mgr.plan("odom_combined", "r_wrist_roll_link", place_pose_msg, plan) ||
          !group_mgr.execute(plan))
        ROS_ERROR("Failure while moving to place pose.");

      continue;
    }

    // pr2 gripper sensor --------------------------------------------------------------------------
    /*
    pr2_gripper_sensor_msgs::PR2GripperGrabGoal grip;
    grip.command.hardness_gain = 0.03;
    ROS_INFO("Sending grab goal.");
    grab_client.sendGoal(grip);
    grab_client.waitForResult(ros::Duration(20.0));
    if (grab_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully completed Grab");
    }
    else
    {
      ROS_INFO("Grab Failed");
      continue;
    }
    */
    //----------------------------------------------------------------------------------------------

    ROS_INFO("Place object.");
    if (!group_mgr.plan("odom_combined", "r_wrist_roll_link", place_pose_msg, plan) ||
      !group_mgr.execute(plan) ||
      !group_mgr.openGripperGently())
    {
      ROS_ERROR("Failure while moving to place pose.");
    }
    //place(scene_mgr, group_mgr, placing_bin);

    // pr2 gripper sensor --------------------------------------------------------------------------
    /*
    pr2_gripper_sensor_msgs::PR2GripperReleaseGoal place;
    place.command.event.trigger_conditions = place.command.event.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC;
    place.command.event.acceleration_trigger_magnitude = 0;
    place.command.event.slip_trigger_magnitude = 0;
    ROS_INFO("Waiting for object placement contact...");
    release_client.sendGoal(place);
    release_client.waitForResult();
    if(release_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Release Success");
    }
    else
    {
      ROS_INFO("Place Failure");
    }
    */
    //----------------------------------------------------------------------------------------------

    rate.sleep();
  }
}