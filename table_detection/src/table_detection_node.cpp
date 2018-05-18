#include <table_detection/table_detection.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_detection");

  table_detection::TableDetection table_detection;

  ros::spin();
}