#include <table_detection/table_detection.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <eigen_conversions/eigen_msg.h>

#include <gpd/SetParameters.h>
#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>

const std::string ns = "pr2_grasp";
const std::string name = "pr2_grasp_node";


bool transform_cloud(tf2_ros::Buffer& tf_buffer, const sensor_msgs::PointCloud2& cloud_msg_in,
  sensor_msgs::PointCloud2& cloud_msg_out, Eigen::Vector3d& sensor_pos)
{
  try
  {
    geometry_msgs::TransformStamped ts = tf_buffer.lookupTransform(
      "odom_combined", cloud_msg_in.header.frame_id, cloud_msg_in.header.stamp);

    tf2::doTransform(cloud_msg_in, cloud_msg_out, ts);
    tf::vectorMsgToEigen(ts.transform.translation, sensor_pos);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }

  return true;
}

void callback_clouds(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr,
  tf2_ros::Buffer& tf_buffer, sensor_msgs::PointCloud2& cloud_msg,
  Eigen::Vector3d& sensor_pos)
{
  //ROS_INFO("Recived point cloud.");
  transform_cloud(tf_buffer, *cloud_msg_ptr, cloud_msg, sensor_pos);
}

void callback_grasps(const gpd::GraspConfigList::ConstPtr& grasp_config_list_ptr,
  gpd::GraspConfigList& grasp_config_list)
{
  //ROS_INFO("Recived grasps.");
  grasp_config_list = *grasp_config_list_ptr;
}

void set_gpd_params(ros::NodeHandle& nh_public, const Eigen::Vector3d& sensor_pos,
  const table_detection::Workspace& workspace)
{
  Eigen::Vector3d ws_min = workspace.pose.translation() - 0.5 * workspace.scale;
  Eigen::Vector3d ws_max = workspace.pose.translation() + 0.5 * workspace.scale;

  gpd::SetParameters set_params_srv;

  set_params_srv.request.set_workspace = true;
  set_params_srv.request.workspace[0] = ws_min[0];
  set_params_srv.request.workspace[1] = ws_max[0];
  set_params_srv.request.workspace[2] = ws_min[1];
  set_params_srv.request.workspace[3] = ws_max[1];
  set_params_srv.request.workspace[4] = ws_min[2];
  set_params_srv.request.workspace[5] = ws_max[2];

  set_params_srv.request.set_camera_position = true;
  set_params_srv.request.camera_position[0] = sensor_pos[0];
  set_params_srv.request.camera_position[1] = sensor_pos[1];
  set_params_srv.request.camera_position[2] = sensor_pos[2];

  ros::ServiceClient client_gpd = nh_public.serviceClient<gpd::SetParameters>("/gpd/set_params");
  bool success = client_gpd.call(set_params_srv);
}

void add_collision_object(ros::NodeHandle& nh_public, const geometry_msgs::Point& position)
{
  moveit_msgs::CollisionObject object;
  object.header.frame_id = "odom_combined";
  object.id = "object";
  object.operation = moveit_msgs::CollisionObject::ADD;

  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(3);
  object.primitives[0].dimensions[0] = 0.2;
  object.primitives[0].dimensions[1] = 0.2;
  object.primitives[0].dimensions[2] = 0.2;

  object.primitive_poses.resize(1);
  object.primitive_poses[0].orientation.w = 1;
  object.primitive_poses[0].position = position;
  //tf::poseEigenToMsg(workspace.pose, object.primitive_poses[0]);

  ros::ServiceClient client_get_ps = nh_public.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  moveit_msgs::GetPlanningScene get_ps_srv;
  get_ps_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  bool success = client_get_ps.call(get_ps_srv);
  moveit_msgs::AllowedCollisionMatrix acm = get_ps_srv.response.scene.allowed_collision_matrix;


  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(object);
  planning_scene.is_diff = true;

  if (acm.default_entry_names.size() == 0)
  {
    acm.default_entry_names.push_back("object");
    acm.default_entry_values.push_back(1);
  }
  planning_scene.allowed_collision_matrix = acm;

  ros::ServiceClient client_ps_diff = nh_public.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  client_ps_diff.waitForExistence();
  moveit_msgs::ApplyPlanningScene service;
  service.request.scene = planning_scene;
  client_ps_diff.call(service);
}

bool quaternion_from_column_vectors(const geometry_msgs::Vector3& col_0, const geometry_msgs::Vector3& col_1,
  const geometry_msgs::Vector3& col_2, geometry_msgs::Quaternion& quaternion)
{
  Eigen::Matrix3d m;
  m <<
    col_0.x, col_1.x, col_2.x,
    col_0.y, col_1.y, col_2.y,
    col_0.z, col_1.z, col_2.z;

  tf::quaternionEigenToMsg(Eigen::Quaterniond(m), quaternion);
}

void grasp(const gpd::GraspConfigList& grasp_config_list, ros::NodeHandle& nh_public)
{
  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //group.setPlanningTime(10);
  group.setGoalTolerance(0.005);

  for (size_t i = 0; i < grasp_config_list.grasps.size(); i++)
  {

    ros::AsyncSpinner spinner(8);

    const gpd::GraspConfig& grasp = grasp_config_list.grasps[i];

    add_collision_object(nh_public, grasp.bottom);

    geometry_msgs::Pose target_pose;
    quaternion_from_column_vectors(grasp.approach, grasp.binormal, grasp.axis, target_pose.orientation);

    Eigen::Vector3f dir = Eigen::Vector3f(grasp.approach.x, grasp.approach.y, grasp.approach.z).normalized();
    Eigen::Vector3f p = Eigen::Vector3f(grasp.bottom.x, grasp.bottom.y, grasp.bottom.z);
    p = p + -dir * 0.2;

    target_pose.position.x = p.x();
    target_pose.position.y = p.y();
    target_pose.position.z = p.z();
    group.setPoseTarget(target_pose, "r_wrist_roll_link");

    moveit::planning_interface::MoveGroup::Plan plan_1, plan_2;
    spinner.start();
    bool success = group.plan(plan_1);
    spinner.stop();

    ROS_INFO("Plan 1: %s", success ? "SUCCESS" : "FAILED");

    if (!success)
      continue;

    ROS_INFO("Wait for key press.");
    std::cin.ignore();

    spinner.start();
    ROS_INFO("Execution of plan 1: %s", group.execute(plan_1) ? "SUCCESS" : "FAILED");

    std::vector<geometry_msgs::Pose> waypoints;
    //waypoints.push_back(target_pose);
    p = p + dir * 0.08;
    target_pose.position.x = p.x();
    target_pose.position.y = p.y();
    target_pose.position.z = p.z();
    waypoints.push_back(target_pose);
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    plan_2.trajectory_ = trajectory;
    ROS_INFO("Plan 2: %.2f", fraction);

    //ROS_INFO("Wait for key press.");
    //std::cin.ignore();

    ROS_INFO("Execution of plan 2: %s", group.execute(plan_2) ? "SUCCESS" : "FAILED");
    spinner.stop();

    exit(EXIT_SUCCESS);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_grasp");
  ros::NodeHandle nh_public, nh_private("~");

  //std::string topic_clouds_in = "/head_mount_kinect/depth_registered/points";
  std::string topic_clouds_in = "/camera/depth_registered/points";
  //std::string topic_clouds_out = "/head_mount_kinect/depth_registered/points/odom_combined";
  std::string topic_clouds_out = "/filtered_cloud/odom_combined";
  std::string topic_grasps_in = "/detect_grasps/clustered_grasps";

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  Eigen::Vector3d sensor_pos;
  sensor_msgs::PointCloud2 cloud_msg;
  ros::Subscriber sub_clouds = nh_public.subscribe<sensor_msgs::PointCloud2>(
    topic_clouds_in, 1, boost::bind(callback_clouds, _1, boost::ref(tf_buffer),
    boost::ref(cloud_msg), boost::ref(sensor_pos)));

  gpd::GraspConfigList grasp_config_list;
  ros::Subscriber sub_grasps = nh_public.subscribe<gpd::GraspConfigList>(
    topic_grasps_in, 1, boost::bind(callback_grasps, _1, boost::ref(grasp_config_list)));

  ros::Publisher pub_clouds = nh_public.advertise<sensor_msgs::PointCloud2>(topic_clouds_out, 1);

  table_detection::TableDetection table_detection;

  ros::Rate rate(10);
  while (ros::ok())
  {
    ROS_INFO("Wait for point cloud.");
    cloud_msg.data.clear();
    while (ros::ok() && cloud_msg.data.empty())
    {
      ros::spinOnce();
      rate.sleep();
    }

    ROS_INFO("Try to detect table.");
    table_detection.detect(cloud_msg);
    table_detection::Workspace workspace = table_detection.getWorkspace();

    ROS_INFO("Set gpd parameters and publish point cloud.");
    set_gpd_params(nh_public, sensor_pos, workspace);
    pub_clouds.publish(cloud_msg);

    ROS_INFO("Wait for grasps.");
    grasp_config_list.grasps.clear();
    while (ros::ok() && grasp_config_list.grasps.empty())
    {
      ros::spinOnce();
      rate.sleep();
    }

    ROS_INFO("Move gripper to object.");
    grasp(grasp_config_list, nh_public);

    rate.sleep();
  }

  return EXIT_SUCCESS;
}