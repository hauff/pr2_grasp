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

void add_workspace(ros::NodeHandle& nh_public, const table_detection::Workspace& workspace)
{
  moveit_msgs::CollisionObject object;
  object.header.frame_id = workspace.frame_id;
  object.id = "workspace";
  object.operation = moveit_msgs::CollisionObject::ADD;

  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions.resize(3);
  object.primitives[0].dimensions[0] = workspace.scale[0]; 
  object.primitives[0].dimensions[1] = workspace.scale[1];
  object.primitives[0].dimensions[2] = workspace.scale[2];

  object.primitive_poses.resize(1);
  tf::poseEigenToMsg(workspace.pose, object.primitive_poses[0]);


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
    acm.default_entry_names.push_back("workspace");
    acm.default_entry_values.push_back(1);
  }
  planning_scene.allowed_collision_matrix = acm;

  ros::ServiceClient client_ps_diff = nh_public.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  client_ps_diff.waitForExistence();
  moveit_msgs::ApplyPlanningScene service;
  service.request.scene = planning_scene;
  client_ps_diff.call(service);
}

bool quat_from_column_vectors(const geometry_msgs::Vector3d& col0,
  const geometry_msgs::Vector3d& col1, const geometry_msgs::Vector3d& col2, geometry_msgs::Quaternionf)
{

}

void grasp(const gpd::GraspConfigList& grasp_config_list)
{
  ros::AsyncSpinner spinner(8);
  spinner.start();

  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  //group.setPlanningTime(10);
  group.setGoalTolerance(0.005);

  for (size_t i = 0; i < grasp_config_list.grasps.size(); i++)
  {
    const gpd::GraspConfig& grasp = grasp_config_list.grasps[i];

    Eigen::Matrix3f m;
    m <<
      grasp.approach.x, grasp.binormal.x, grasp.axis.x,
      grasp.approach.y, grasp.binormal.y, grasp.axis.y,
      grasp.approach.z, grasp.binormal.z, grasp.axis.z;
    Eigen::Quaternionf q(m);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    Eigen::Vector3f dir = Eigen::Vector3f(grasp.approach.x, grasp.approach.y, grasp.approach.z).normalized();
    Eigen::Vector3f p = Eigen::Vector3f(grasp.bottom.x, grasp.bottom.y, grasp.bottom.z);
    p = p + -dir * 0.2;

    target_pose.position.x = p.x();
    target_pose.position.y = p.y();
    target_pose.position.z = p.z();
    group.setPoseTarget(target_pose, "r_wrist_roll_link");


    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = group.plan(plan);
    ROS_INFO("Plan: %s", success? "SUCCESS" : "FAILED");

    //geometry_msgs::PoseStamped pose_stamped;
    //pose_stamped.header.stamp = ros::Time::now();
    //pose_stamped.header.frame_id = "odom_combined";
    //pose_stamped.pose = target_pose;

    //pub_pose.publish(pose_stamped);
    //sleep(10);

    if (success)
    {
      ROS_INFO("Wait for key press.");
      std::cin.ignore();
      success = group.execute(plan);
      ROS_INFO("Execution: %s", success? "SUCCESS" : "FAILED");


      std::vector<geometry_msgs::Pose> waypoints;
      p = p + dir * 0.05;
      target_pose.position.x = p.x();
      target_pose.position.y = p.y();
      target_pose.position.z = p.z();
      waypoints.push_back(target_pose);
      moveit_msgs::RobotTrajectory trajectory;
      double fraction = group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
      plan.trajectory_ = trajectory;

      success = group.execute(plan);

      if (success)
        exit(EXIT_SUCCESS);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_grasp");
  ros::NodeHandle nh_public, nh_private("~");

  std::string topic_clouds_in = "/head_mount_kinect/depth_registered/points";
  std::string topic_grasps_in = "/detect_grasps/clustered_grasps";
  std::string topic_clouds_out = "/head_mount_kinect/depth_registered/points/odom_combined";

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

    //ROS_INFO("Add workspace to planning scene.");
    //add_workspace(nh_public, workspace);

    ROS_INFO("Move gripper to object.");
    grasp(grasp_config_list);

    rate.sleep();
  }

  return EXIT_SUCCESS;
}

/*
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gpd/GraspConfigList.h>

#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/ApplyPlanningScene.h>

void callback(const gpd::GraspConfigList::ConstPtr& gpd_ptr, tf2_ros::Buffer& tf_buffer,
  gpd::GraspConfigList& gpd)
{
  gpd = *gpd_ptr;

  ROS_INFO("Recived grasps: %lu", gpd.grasps.size());

  ROS_INFO("Transorming grasps to 'odom_combined'.");

  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer.lookupTransform(
      "odom_combined",
      "head_mount_kinect_rgb_optical_frame",
      gpd.header.stamp);

    for (size_t i = 0; i < gpd_ptr->grasps.size(); i++)
    {
      tf2::doTransform(gpd_ptr->grasps[i].bottom, grasp.bottom, transform);
      tf2::doTransform(gpd_ptr->grasps[i].approach, grasp.approach, transform);
      tf2::doTransform(gpd_ptr->grasps[i].binormal, grasp.binormal, transform);
      tf2::doTransform(gpd_ptr->grasps[i].axis, grasp.axis, transform);

      //std::cout << gpd_ptr->grasps[i].bottom << std::endl;
      //std::cout << grasp.bottom << std::endl;

      //exit(EXIT_FAILURE);
    }
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    gpd.grasps.clear();
    return;
  }
}

void add_workspace(ros::NodeHandle& nh_public)
{
  moveit_msgs::CollisionObject object;
  object.header.frame_id = "odom_combined";
  object.id = "workspace";

  geometry_msgs::Pose pose;
  pose.position.x = 0.560794;
  pose.position.y = 0.0119765;
  pose.position.z = 0.832663;
  pose.orientation.x = 2.99047e-05;
  pose.orientation.y = 1.54574e-06;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.772718;
  primitive.dimensions[1] = 1.43515;
  primitive.dimensions[2] = 0.495594;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);

  object.operation = moveit_msgs::CollisionObject::ADD;

  ROS_INFO("Obtaining planning scene.");
  ros::ServiceClient client_get_ps = nh_public.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
  moveit_msgs::GetPlanningScene get_ps_srv;
  get_ps_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  bool success = client_get_ps.call(get_ps_srv);
  moveit_msgs::AllowedCollisionMatrix acm = get_ps_srv.response.scene.allowed_collision_matrix;


  ROS_INFO("Adding workspace to planning scene.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(object);
  planning_scene.is_diff = true;

  if (acm.default_entry_names.size() == 0)
  {
    acm.default_entry_names.push_back("workspace");
    acm.default_entry_values.push_back(1);
  }
  planning_scene.allowed_collision_matrix = acm;

  ros::ServiceClient client_ps_diff = nh_public.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  client_ps_diff.waitForExistence();
  moveit_msgs::ApplyPlanningScene service;
  service.request.scene = planning_scene;
  client_ps_diff.call(service);

  //std::cout << acm << std::endl;
  //exit(EXIT_SUCCESS);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grasp");
  ros::NodeHandle nh_public;

  std::string topic_in = "/detect_grasps/clustered_grasps";

  add_workspace(nh_public);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  gpd::GraspConfigList gpd;

  ros::Publisher pub_pose = nh_public.advertise<geometry_msgs::PoseStamped>("grasps", 100);

  ros::Subscriber sub_grasps = nh_public.subscribe<gpd::GraspConfigList>(
    topic_in, 100, boost::bind(callback, _1, boost::ref(tf_buffer), boost::ref(gpd)));

  while (ros::ok())
  {
    ros::Rate(5).sleep();
    ros::spinOnce();

    if (gpd.grasps.size() > 0)
    {
      ROS_INFO("Grasps: %lu", gpd.grasps.size());
      sub_grasps.shutdown();
      break;
    }
  }

  ROS_INFO("Start planning ...");

  ros::AsyncSpinner spinner(8);
  spinner.start();

  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  //ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  //ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  group.setPlanningTime(10);
  group.setGoalTolerance(0.01);

  std::cout << "planning time: " << group.getPlanningTime() << std::endl;
  std::cout << "goal joint tolerance: " << group.getGoalJointTolerance() << std::endl;
  std::cout << "goal position tolerance: " << group.getGoalPositionTolerance() << std::endl;
  std::cout << "goal orientation tolerance: " << group.getGoalOrientationTolerance() << std::endl;

  for (size_t i = 0; i < gpd.grasps.size(); i++)
  {
    //Eigen::Vector3f x_axis = Eigen::Vector3f(grasp.approach.x, grasp.approach.y, grasp.approach.z);
    //Eigen::Vector3f y_axis = Eigen::Vector3f(grasp.binormal.x, grasp.binormal.y, grasp.binormal.z);
    //Eigen::Vector3f z_axis = x_axis.cross(y_axis).normalized();

    Eigen::Matrix3f m;
    m <<
      grasp.approach.x, grasp.binormal.x, grasp.axis.x,
      grasp.approach.y, grasp.binormal.y, grasp.axis.y,
      grasp.approach.z, grasp.binormal.z, grasp.axis.z;
    Eigen::Quaternionf q(m);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    Eigen::Vector3f dir = Eigen::Vector3f(grasp.approach.x, grasp.approach.y, grasp.approach.z).normalized();
    Eigen::Vector3f p = Eigen::Vector3f(grasp.bottom.x, grasp.bottom.y, grasp.bottom.z);
    p = p + -dir * 0.2;

    target_pose.position.x = p.x();
    target_pose.position.y = p.y();
    target_pose.position.z = p.z();
    group.setPoseTarget(target_pose, "r_wrist_roll_link");

    //std::cout << target_pose.position << std::endl;
    //std::cout << grasp.approach << std::endl;
    //std::cout << grasp.binormal << std::endl;
    //std::cout << grasp.axis << std::endl;

    //system("rosservice call /clear_octomap");
    //sleep(1);
    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = group.plan(plan);
    ROS_INFO("Plan: %s", success? "SUCCESS" : "FAILED");
    
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "odom_combined";
    pose_stamped.pose = target_pose;

    pub_pose.publish(pose_stamped);
    //sleep(10);

    if (success)
    {
      success = group.execute(plan);
      ROS_INFO("Execution: %s", success? "SUCCESS" : "FAILED");
      //break;
    }
  }

  return EXIT_SUCCESS;
}
*/