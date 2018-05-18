#ifndef TABLE_DETECTION_TABLE_DETECTION_H
#define TABLE_DETECTION_TABLE_DETECTION_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace table_detection
{

class TableDetection
{

public:

  TableDetection();

private:

  void cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  void transform(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  void downsample();

  void cropBox();

  void estimatePlaneCoeffs();

  void extractCluster();

  void projectPointCloudToModel();

  void computeConvexHull();

  void computeWorkspace();

  void publishMarkers();


  std::string cloud_topic_in_;
  float voxel_grid_size_;
  Eigen::VectorXf crop_box_;
  Eigen::Vector3f up_vector_;
  float up_vector_thresh_;
  float inlier_thresh_;
  size_t min_cluster_size_;
  float cluster_tolerance_;

  pcl::PointIndices::Ptr inlier_ptr_;
  pcl::ModelCoefficients model_coeffs_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bounds_ptr_;
  Eigen::Vector3f workspace_min_;
  Eigen::Vector3f workspace_max_;
  Eigen::Affine3d transform_;

  ros::NodeHandle nh_public_;
  ros::Subscriber sub_cloud_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rviz_visual_tools::RvizVisualToolsPtr visual_tools_ptr_;

  ros::Publisher pub_marker_;

};

}

#endif