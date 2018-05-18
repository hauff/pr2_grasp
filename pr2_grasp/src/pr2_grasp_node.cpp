#include <table_detection/table_detection.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_grasp");
  ros::NodeHandle nh_public;

  std::string topic_in = "/detect_grasps/clustered_grasps";

  //table_detection::TableDetection table_detection;

  //ros::spin();

  ROS_INFO("hello world");
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