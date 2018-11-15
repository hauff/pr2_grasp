#include <planning_scene_manager/planning_scene_manager.h>
#include <move_group_manager/move_group_manager.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>

#include <eigen_conversions/eigen_msg.h>

#include <gpd/GraspConfig.h>
#include <gpd/GraspConfigList.h>
#include <gpd/SetParameters.h>

#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
//#include <moveit_msgs/GetPlanningScene.h>
//#include <moveit_msgs/ApplyPlanningScene.h>
//#include <moveit_msgs/AllowedCollisionMatrix.h>


bool transform_cloud(tf2_ros::Buffer& tf_buffer, const sensor_msgs::PointCloud2& cloud_msg_in,
  sensor_msgs::PointCloud2& cloud_msg_out, Eigen::Vector3d& sensor_pos)
{
  try
  {
    geometry_msgs::TransformStamped ts = tf_buffer.lookupTransform("odom_combined", cloud_msg_in.header.frame_id,
      cloud_msg_in.header.stamp);

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

void callback_clouds(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr, tf2_ros::Buffer& tf_buffer,
  sensor_msgs::PointCloud2& cloud_msg, Eigen::Vector3d& sensor_pos)
{
  //ROS_INFO("Recived point cloud.");
  transform_cloud(tf_buffer, *cloud_msg_ptr, cloud_msg, sensor_pos);
}

void callback_grasps(const gpd::GraspConfigList::ConstPtr& grasp_config_list_ptr,
  gpd::GraspConfigList& grasp_config_list)
{
  ROS_INFO("Recived %lu grasp proposals.", grasp_config_list_ptr->grasps.size());
  grasp_config_list = *grasp_config_list_ptr;
}

bool set_gpd_params(ros::NodeHandle& nh_public, const Eigen::Vector3d& sensor_pos,
  const geometry_msgs::Pose& table_pose, const std::vector<double>& table_dimensions)
{
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(table_pose, pose);
  Eigen::Vector3d dimensions(table_dimensions.at(0), table_dimensions.at(1), table_dimensions.at(2));

  Eigen::Vector3d ws_min = pose.translation() - 0.5 * dimensions + Eigen::Vector3d(0, 0, dimensions.z() + 0.03);
  Eigen::Vector3d ws_max = pose.translation() + 0.5 * dimensions + Eigen::Vector3d(0, 0, dimensions.z() + 0.5);

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
  bool result = client_gpd.call(set_params_srv);

  return result;
}

bool grasp(const gpd::GraspConfigList& grasp_config_list, planning_scene_manager::PlanningSceneManager scene_mgr,
  move_group_manager::MoveGroupManager& group_mgr)
{
  for (size_t i = 0; i < grasp_config_list.grasps.size(); i++)
  {
    const gpd::GraspConfig& grasp = grasp_config_list.grasps[i];

    Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
    object_pose.translation() << grasp.bottom.x, grasp.bottom.y, grasp.bottom.z;
    //scene_mgr.addBoxCollisionObject("odom_combined", "object", object_pose, Eigen::Vector3d(0.1, 0.1, 0.2));
		scene_mgr.addSphereCollisionObject("odom_combined", "object", object_pose, 0.10);
    scene_mgr.allowCollision("object");

    Eigen::Matrix3d m;
    m <<
      grasp.approach.x, grasp.binormal.x, grasp.axis.x,
      grasp.approach.y, grasp.binormal.y, grasp.axis.y,
      grasp.approach.z, grasp.binormal.z, grasp.axis.z;

    geometry_msgs::Pose grasp_pose;
    tf::quaternionEigenToMsg(Eigen::Quaterniond(m), grasp_pose.orientation);

    Eigen::Vector3d wrist =
      Eigen::Vector3d(grasp.approach.x, grasp.approach.y, grasp.approach.z) * -0.14 +
      Eigen::Vector3d(grasp.bottom.x, grasp.bottom.y, grasp.bottom.z);
    grasp_pose.position.x = wrist.x();
    grasp_pose.position.y = wrist.y();
    grasp_pose.position.z = wrist.z();

    int result = group_mgr.pick(grasp_pose, grasp.approach);

    if (result == 1)
      return true;

    ROS_WARN("Grasp failed with error code: %d", result);
    group_mgr.openGripper();
    scene_mgr.removeCollisionObject("odom_combined", "object");
    scene_mgr.removeCollisionObject("r_wist_roll_link", "object");
  }

  ROS_WARN("Tried all grasp proposals without success.");
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_grasp");
  ros::NodeHandle nh_public, nh_clouds, nh_grasps;
  ros::CallbackQueue queue_clouds, queue_grasps;
  nh_clouds.setCallbackQueue(&queue_clouds);
  nh_grasps.setCallbackQueue(&queue_grasps);

  //std::string topic_clouds_in = "/filtered_cloud";
  std::string topic_clouds_in = "/head_mount_kinect2/depth_registered/points";
  std::string topic_clouds_out = "/pr2_grasp/point_cloud";
  std::string topic_grasps_in = "/detect_grasps/clustered_grasps";

  // Create tf listener.
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Create subscriber for point clouds.
  Eigen::Vector3d sensor_pos;
  sensor_msgs::PointCloud2 cloud_msg;
  ros::Subscriber sub_clouds = nh_clouds.subscribe<sensor_msgs::PointCloud2>(topic_clouds_in, 1,
    boost::bind(callback_clouds, _1, boost::ref(tf_buffer), boost::ref(cloud_msg), boost::ref(sensor_pos)));

  // Create subscriber for grasp proposals.
  gpd::GraspConfigList grasp_config_list;
  ros::Subscriber sub_grasps = nh_grasps.subscribe<gpd::GraspConfigList>(topic_grasps_in, 1,
    boost::bind(callback_grasps, _1, boost::ref(grasp_config_list)));

  // Create publisher for point clouds.
  ros::Publisher pub_clouds = nh_public.advertise<sensor_msgs::PointCloud2>(topic_clouds_out, 1);

  planning_scene_manager::PlanningSceneManager scene_mgr;
  move_group_manager::MoveGroupManager group_mgr;

  ros::Rate rate(5);
  while (ros::ok())
  {
    ROS_INFO("Wait for point cloud.");
    ros::Time time = ros::Time::now();
    while (ros::ok() && cloud_msg.header.stamp < time)
    {
      queue_clouds.callAvailable(ros::WallDuration(0));
      rate.sleep();
    }

    ROS_INFO("Set gpd parameters.");
    moveit_msgs::CollisionObject object;
    if(!scene_mgr.getCollisionObject("table", object))
      return EXIT_FAILURE;

    set_gpd_params(nh_public, sensor_pos, object.primitive_poses.at(0), object.primitives.at(0).dimensions);

    ROS_INFO("Publish point cloud.");
    pub_clouds.publish(cloud_msg);

    ROS_INFO("Wait for grasp proposals.");
    while (ros::ok() && grasp_config_list.header.stamp != cloud_msg.header.stamp)
    {
      queue_grasps.callAvailable(ros::WallDuration(0));
      rate.sleep();
    }

    ROS_INFO("Try to grasp object.");
    if (grasp(grasp_config_list, scene_mgr, group_mgr))
    {
      ROS_INFO("Try to place object.");
      moveit_msgs::CollisionObject object;
      if(!scene_mgr.getCollisionObject("placing_bin", object))
        return EXIT_FAILURE;

      geometry_msgs::Pose pose = object.primitive_poses.at(0);
      pose.position.z += object.primitives.at(0).dimensions.at(2) / 2 + 0.3;
      pose.position.x -= 0.15;
      // Rotate end pose by 90 DEGREES around pitch axis
      tf::Quaternion pitch_rotation = tf::createQuaternionFromRPY(0.0, M_PI/4.0, 0);
      pitch_rotation.normalize();
      tf::Quaternion pose_orientation;
      tf::quaternionMsgToTF(pose.orientation, pose_orientation); 
      tf::Quaternion rotated_pose_orientation = pitch_rotation*pose_orientation;
      tf::quaternionTFToMsg(rotated_pose_orientation, pose.orientation);

      moveit::planning_interface::MoveGroup::Plan plan;
      if (group_mgr.plan("odom_combined", "r_wrist_roll_link", pose, plan))
        group_mgr.execute(plan);

      group_mgr.openGripper();
      scene_mgr.removeCollisionObject("odom_combined", "object");
      scene_mgr.removeCollisionObject("r_wist_roll_link", "object");
    }

    rate.sleep();
  }

  return EXIT_SUCCESS;
}