#ifndef TABLE_DETECTION_TABLE_DETECTION_H
#define TABLE_DETECTION_TABLE_DETECTION_H

#include <planning_scene_manager/planning_scene_manager.h>
#include <rviz_visualizer/rviz_visualizer.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

namespace table_detection
{

struct Table
{
  std::string frame_id;
  Eigen::Affine3d pose;
  Eigen::Vector3d dimensions;
};

class TableDetection
{

public:

  TableDetection();

  Table getTable() { return table_; }

  void run(const std::string& topic);

  void stop();

  void detect(const sensor_msgs::PointCloud2& cloud_msg);

private:

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

  bool transform(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr,
    sensor_msgs::PointCloud2& cloud_msg);

  void downsample();

  void cropBox();

  void estimatePlaneCoeffs();

  void extractCluster();

  void projectPointCloudToModel();

  void computeConvexHull();

  void computeBounds();

  void publish();

  std::string nameSpace() { return "table_detection"; };

  std::string className() { return "TableDetection"; };

  std::string name() { return nameSpace() + "::" + className(); };

  float voxel_grid_size_;
  std::vector<float> crop_box_;
  std::vector<float> up_vector_;
  float up_vector_thresh_;
  float inlier_thresh_;
  int min_cluster_size_;
  float cluster_tolerance_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bounds_ptr_;
  pcl::PointIndices::Ptr inlier_ptr_;
  pcl::ModelCoefficients model_coeffs_;
  Table table_;

  ros::NodeHandle nh_public_, nh_private_;
  ros::Subscriber sub_cloud_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  planning_scene_manager::PlanningSceneManager scene_mgr_;
  rviz_visualizer::RvizVisualizer::Ptr rviz_visualizer_ptr_;

};

}

#endif