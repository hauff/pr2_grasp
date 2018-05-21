#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

const std::string ns = "ros_toolbox";
const std::string name = "transform_clouds";

bool compareVectors(const geometry_msgs::Vector3& vec1, const geometry_msgs::Vector3& vec2,
  double epsilon = 1e-3)
{
  return
    fabs(vec1.x - vec2.x) < epsilon &&
    fabs(vec1.y - vec2.y) < epsilon &&
    fabs(vec1.z - vec2.z) < epsilon;
}

void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr, tf2_ros::Buffer& tf_buffer,
  const std::string& target_frame, geometry_msgs::Vector3& prev_sensor_position,
  ros::Publisher& pub_cloud)
{
  sensor_msgs::PointCloud2 cloud_msg;
  geometry_msgs::Vector3 sensor_position;

  try
  {
    geometry_msgs::TransformStamped ts = tf_buffer.lookupTransform(
      target_frame, cloud_msg_ptr->header.frame_id, cloud_msg_ptr->header.stamp);
    tf2::doTransform(*cloud_msg_ptr, cloud_msg, ts);
    sensor_position = ts.transform.translation;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    return;
  }

  if (!compareVectors(prev_sensor_position, sensor_position))
  {
    ROS_INFO("[%s/%s]: Sensor position: %.3f, %.3f, %.3f", ns.c_str(), name.c_str(),
     sensor_position.x, sensor_position.y, sensor_position.z);

    prev_sensor_position = sensor_position;
  }

  pub_cloud.publish(cloud_msg);
}

void fetch_params(ros::NodeHandle& nh_private, std::string& topic_in, std::string& topic_out,
  std::string& target_frame)
{
  if (!nh_private.getParam("topic_in", topic_in))
  {
    ROS_ERROR("Parameter 'topic_in' not found on server.");
    EXIT_FAILURE;
  }

  if (!nh_private.getParam("topic_out", topic_out))
  {
    ROS_ERROR("Parameter 'topic_out' not found on server.");
    EXIT_FAILURE;
  }

  if (!nh_private.getParam("target_frame", target_frame))
  {
    ROS_ERROR("Parameter 'target_frame' not found on server.");
    EXIT_FAILURE;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_clouds");
  ros::NodeHandle nh_public, nh_private("~");

  std::string topic_in, topic_out, target_frame;
  fetch_params(nh_private, topic_in, topic_out, target_frame);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  geometry_msgs::Vector3 prev_sensor_position;
  ros::Publisher pub_cloud = nh_public.advertise<sensor_msgs::PointCloud2>(topic_out, 100);

  ros::Subscriber sub_cloud = nh_public.subscribe<sensor_msgs::PointCloud2>(topic_in, 100,
    boost::bind(callback, _1, boost::ref(tf_buffer), boost::ref(target_frame),
    boost::ref(prev_sensor_position), boost::ref(pub_cloud)));

  ros::spin();

  EXIT_SUCCESS;
};