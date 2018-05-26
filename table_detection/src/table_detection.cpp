#include <table_detection/table_detection.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>

namespace table_detection
{

TableDetection::TableDetection() : nh_private_("~"), tf_listener_(tf_buffer_)
{
  cloud_topic_in_ = "/head_mount_kinect/depth_registered/points";
  //cloud_topic_in_ = "/camera/depth_registered/points";
  voxel_grid_size_ = 0.02;
  crop_box_.resize(6);
  crop_box_ << 0, 2, -1, 1, 0.1, 1;
  up_vector_ << 0, 0, 1;
  up_vector_thresh_ = DEG2RAD(5);
  inlier_thresh_ = 0.02;
  min_cluster_size_ = 100;
  cluster_tolerance_ = 0.04;

  inlier_ptr_.reset(new pcl::PointIndices());
  cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_filtered_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_bounds_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
/*
  sub_cloud_ = nh_public_.subscribe<sensor_msgs::PointCloud2>(
    cloud_topic_in_, 10, &TableDetection::cloudCallback, this);
*/
  rviz_visualizer_ptr_.reset(new rviz_visualizer::RvizVisualizer(
    "odom_combined", "markers", nh_private_));
  rviz_visualizer_ptr_->setAlpha(0.5);
}

void TableDetection::detect(const sensor_msgs::PointCloud2& cloud_msg)
{
  cloud_ptr_->clear();
  cloud_filtered_ptr_->clear();
  cloud_bounds_ptr_->clear();
  inlier_ptr_->indices.clear();

  pcl::fromROSMsg(cloud_msg, *cloud_ptr_);
  downsample();
  cropBox();
  estimatePlaneCoeffs();
  extractCluster();
  projectPointCloudToModel();
  computeConvexHull();
  computeWorkspace();

  publishMarkers();
}

void TableDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
  cloud_ptr_->clear();
  cloud_filtered_ptr_->clear();
  cloud_bounds_ptr_->clear();
  inlier_ptr_->indices.clear();

  transform(cloud_msg_ptr);
  downsample();
  cropBox();
  estimatePlaneCoeffs();
  extractCluster();
  projectPointCloudToModel();
  computeConvexHull();
  computeWorkspace();

  publishMarkers();
}

void TableDetection::transform(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
      "odom_combined", cloud_msg_ptr->header.frame_id, cloud_msg_ptr->header.stamp);

    sensor_msgs::PointCloud2 cloud_msg;
    tf2::doTransform(*cloud_msg_ptr, cloud_msg, transform);
    pcl::fromROSMsg(cloud_msg, *cloud_ptr_);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
  }
}

void TableDetection::downsample()
{
  if (cloud_ptr_->empty())
    return;

  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_ptr_);
  vg.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
  vg.filter(*cloud_filtered_ptr_);
}

void TableDetection::cropBox()
{
  if (cloud_filtered_ptr_->empty())
    return;

  pcl::CropBox<pcl::PointXYZRGB> cb;
  cb.setInputCloud(cloud_filtered_ptr_);
  cb.setMin(Eigen::Vector4f(crop_box_[0], crop_box_[2], crop_box_[4], 0));
  cb.setMax(Eigen::Vector4f(crop_box_[1], crop_box_[3], crop_box_[5], 0));
  cb.filter(inlier_ptr_->indices);
}

void TableDetection::estimatePlaneCoeffs()
{
  if (inlier_ptr_->indices.empty() || cloud_filtered_ptr_->empty())
    return;

  pcl::SACSegmentation<pcl::PointXYZRGB> sac;
  sac.setInputCloud(cloud_filtered_ptr_);
  sac.setIndices(inlier_ptr_);
  sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setDistanceThreshold(inlier_thresh_);
  sac.setAxis(up_vector_);
  sac.setEpsAngle(up_vector_thresh_);
  sac.segment(*inlier_ptr_, model_coeffs_);
}

void TableDetection::extractCluster()
{
  if (inlier_ptr_->indices.empty() || cloud_filtered_ptr_->empty())
    return;

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
  ece.setInputCloud(cloud_filtered_ptr_);
  ece.setIndices(inlier_ptr_);
  ece.setClusterTolerance(cluster_tolerance_);
  ece.setMinClusterSize(min_cluster_size_);

  std::vector<pcl::PointIndices> clusters;
  ece.extract(clusters);

  inlier_ptr_->indices.clear();
  for (size_t i = 0; i < clusters.size(); i++)
  {
    inlier_ptr_->indices.insert(inlier_ptr_->indices.end(), clusters[i].indices.begin(),
      clusters[i].indices.end());
  }
}

void TableDetection::projectPointCloudToModel()
{
  if (inlier_ptr_->indices.empty() || cloud_filtered_ptr_->empty())
    return;

  pcl::ProjectInliers<pcl::PointXYZRGB> pi;
  pi.setInputCloud(cloud_filtered_ptr_);
  pi.setIndices(inlier_ptr_);
  pi.setModelType(pcl::SACMODEL_PLANE);
  pi.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(model_coeffs_));
  pi.filter(*cloud_bounds_ptr_);
}

void TableDetection::computeConvexHull()
{
  if (cloud_bounds_ptr_->empty())
    return;

  pcl::ConvexHull<pcl::PointXYZRGB> ch;
  ch.setInputCloud(cloud_bounds_ptr_);

  pcl::PointCloud<pcl::PointXYZRGB> cloud_tmp;
  ch.reconstruct(cloud_tmp);
  pcl::copyPointCloud(cloud_tmp, *cloud_bounds_ptr_);
}

void TableDetection::computeWorkspace()
{
  if (inlier_ptr_->indices.empty() || cloud_filtered_ptr_->empty())
    return;

  Eigen::Vector4f min, max;
  pcl::getMinMax3D(*cloud_filtered_ptr_, *inlier_ptr_, min, max);

  workspace_min_ << min.x(), min.y(), min.z() - inlier_thresh_;
  workspace_max_ << max.x(), max.y(), max.z() + 0.5;
}

void TableDetection::publishMarkers()
{
  std::vector<Eigen::Vector3d> bounds(cloud_bounds_ptr_->size());
  for (size_t i = 0; i < cloud_bounds_ptr_->size(); i++)
  {
    bounds[i] <<
      cloud_bounds_ptr_->at(i).x,
      cloud_bounds_ptr_->at(i).y,
      cloud_bounds_ptr_->at(i).z;
  }

  Eigen::Vector3d ws_position = (workspace_min_ + workspace_max_) / 2;
  Eigen::Vector3d ws_scale = workspace_max_ - workspace_min_;

  rviz_visualizer_ptr_->publishPolygon(bounds, Eigen::Quaterniond::Identity(), 0.02,
    rviz_visualizer::GREEN, "Bounds");

  rviz_visualizer_ptr_->publishCube(ws_position, Eigen::Quaterniond::Identity(), ws_scale,
    rviz_visualizer::BLUE, "Workspace");

  rviz_visualizer_ptr_->publish();
}

}