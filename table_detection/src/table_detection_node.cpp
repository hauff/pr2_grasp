#include <table_detection/table_detection.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_detection");
  ros::NodeHandle nh_public, nh_private("~");

  table_detection::TableDetection table_detection;
  //table_detection.run("/head_mount_kinect2/depth_registered/points");
  table_detection.run("/camera/depth_registered/points");

  ros::spin();

  return EXIT_SUCCESS;
}
