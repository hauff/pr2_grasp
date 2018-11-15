#include <table_detection/table_detection.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_detection");
  ros::NodeHandle nh_public, nh_private("~");


  ROS_ERROR("Running ....");

  std::string param_topic_in = "topic_in";
  if (!nh_private.hasParam(param_topic_in))
  {
    ROS_ERROR("Parameter missing: %s", param_topic_in.c_str());
    return EXIT_FAILURE;
  }

  std::string topic_in;
  nh_private.param<std::string>(param_topic_in, topic_in, "");

  table_detection::TableDetection table_detection;
  table_detection.run(topic_in);

  ros::spin();

  return EXIT_SUCCESS;
}
