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

#include <gpd/SetParameters.h>
//#include <gpd/CloudSources.h>
//#include <gpd/CloudIndexed.h>
//#include <gpd/GraspConfigList.h>

//#include <gpg/cloud_camera.h>
//#include <gpd/grasp_detector.h>
//#include <gpg/grasp.h>

const std::string ns = "pr2_grasp";
const std::string name = "pr2_grasp_node";


bool transform_cloud(tf2_ros::Buffer& tf_buffer, const sensor_msgs::PointCloud2& cloud_msg_in,
  sensor_msgs::PointCloud2& cloud_msg_out, geometry_msgs::Vector3& sensor_pos)
{
  try
  {
    geometry_msgs::TransformStamped ts = tf_buffer.lookupTransform(
      "odom_combined", cloud_msg_in.header.frame_id, cloud_msg_in.header.stamp);

    tf2::doTransform(cloud_msg_in, cloud_msg_out, ts);
    sensor_pos = ts.transform.translation;
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
  geometry_msgs::Vector3& sensor_pos)
{
  ROS_INFO("[%s::%s]: Recived point cloud.", ns.c_str(), name.c_str());
  transform_cloud(tf_buffer, *cloud_msg_ptr, cloud_msg, sensor_pos);
}

/*
void detect_grasps(ros::NodeHandle& nh_private, const sensor_msgs::PointCloud2& cloud_msg)
{
  nh_private.setParam("hand_outer_diameter", 0.12);
  nh_private.setParam("num_threads", 4);
  double tmp[] = {1,1,1,1,1,1};
  std::vector<double> ws(tmp, tmp + sizeof(tmp) / sizeof(tmp[0]));
  nh_private.setParam("workspace", ws);
  nh_private.setParam("workspace_grasps", ws);

  std::string gpd_path = ros::package::getPath("gpd");
  nh_private.setParam("model_file", gpd_path + "/caffe/15channels/lenet_15_channels.prototxt");
  nh_private.setParam("trained_file", gpd_path + "/caffe/15channels/two_views_15_channels_53_deg.caffemodel");

  nh_private.setParam("min_inliers", 1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
  pcl::fromROSMsg(cloud_msg, *cloud_ptr);
  Eigen::Matrix3Xd view_point(3,1);
  view_point.col(0) << 0, 0, 1.5;
  CloudCamera cloud_camera(cloud_ptr, 0, view_point);

  GraspDetector grasp_detector(nh_private);
  grasp_detector.preprocessPointCloud(cloud_camera);
  //std::vector<Grasp> grasps = grasp_detector.detectGrasps(cloud_camera);

  //std::cout << grasps.size() << std::endl;
}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_grasp");
  ros::NodeHandle nh_public, nh_private("~");

  std::string topic_clouds_in = "/head_mount_kinect/depth_registered/points";
  std::string topic_clouds_out = "/head_mount_kinect/depth_registered/points/odom_combined";

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  geometry_msgs::Vector3 sensor_pos;
  sensor_msgs::PointCloud2 cloud_msg;
  ros::Subscriber sub_clouds = nh_public.subscribe<sensor_msgs::PointCloud2>(topic_clouds_in, 1,
    boost::bind(callback_clouds, _1, boost::ref(tf_buffer), boost::ref(cloud_msg),
    boost::ref(sensor_pos)));

  ros::Publisher pub_clouds = nh_public.advertise<sensor_msgs::PointCloud2>(topic_clouds_out, 1);

  ros::ServiceClient client_gpd_set_params = nh_public.serviceClient<gpd::SetParameters>("/gpd/set_params");

  table_detection::TableDetection table_detection;

  ros::Rate rate(10);
  while (ros::ok())
  {
    cloud_msg.data.clear();
    ros::spinOnce();

    if (cloud_msg.data.empty())
      continue;

    gpd::SetParameters set_params_srv;
    set_params_srv.request.set_camera_position = true;
    set_params_srv.request.camera_position[0] = 0;
    set_params_srv.request.camera_position[1] = 0;
    set_params_srv.request.camera_position[2] = 1.5;
    bool success = client_gpd_set_params.call(set_params_srv);

    //table_detection.detect(cloud_msg);

    //detect_grasps(nh_private, cloud_msg);

    pub_clouds.publish(cloud_msg);


    sleep(60);

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
      tf2::doTransform(gpd_ptr->grasps[i].bottom, gpd.grasps[i].bottom, transform);
      tf2::doTransform(gpd_ptr->grasps[i].approach, gpd.grasps[i].approach, transform);
      tf2::doTransform(gpd_ptr->grasps[i].binormal, gpd.grasps[i].binormal, transform);
      tf2::doTransform(gpd_ptr->grasps[i].axis, gpd.grasps[i].axis, transform);

      //std::cout << gpd_ptr->grasps[i].bottom << std::endl;
      //std::cout << gpd.grasps[i].bottom << std::endl;

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
    //Eigen::Vector3f x_axis = Eigen::Vector3f(gpd.grasps[i].approach.x, gpd.grasps[i].approach.y, gpd.grasps[i].approach.z);
    //Eigen::Vector3f y_axis = Eigen::Vector3f(gpd.grasps[i].binormal.x, gpd.grasps[i].binormal.y, gpd.grasps[i].binormal.z);
    //Eigen::Vector3f z_axis = x_axis.cross(y_axis).normalized();

    Eigen::Matrix3f m;
    m <<
      gpd.grasps[i].approach.x, gpd.grasps[i].binormal.x, gpd.grasps[i].axis.x,
      gpd.grasps[i].approach.y, gpd.grasps[i].binormal.y, gpd.grasps[i].axis.y,
      gpd.grasps[i].approach.z, gpd.grasps[i].binormal.z, gpd.grasps[i].axis.z;
    Eigen::Quaternionf q(m);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();

    Eigen::Vector3f dir = Eigen::Vector3f(gpd.grasps[i].approach.x, gpd.grasps[i].approach.y, gpd.grasps[i].approach.z).normalized();
    Eigen::Vector3f p = Eigen::Vector3f(gpd.grasps[i].bottom.x, gpd.grasps[i].bottom.y, gpd.grasps[i].bottom.z);
    p = p + -dir * 0.2;

    target_pose.position.x = p.x();
    target_pose.position.y = p.y();
    target_pose.position.z = p.z();
    group.setPoseTarget(target_pose, "r_wrist_roll_link");

    //std::cout << target_pose.position << std::endl;
    //std::cout << gpd.grasps[i].approach << std::endl;
    //std::cout << gpd.grasps[i].binormal << std::endl;
    //std::cout << gpd.grasps[i].axis << std::endl;

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