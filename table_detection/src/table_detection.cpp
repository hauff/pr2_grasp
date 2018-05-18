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
#include <geometry_msgs/Polygon.h>

#include <eigen_conversions/eigen_msg.h>

namespace table_detection
{

TableDetection::TableDetection() : tf_listener_(tf_buffer_)
{
  cloud_topic_in_ = "/head_mount_kinect/depth_registered/points";
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

  visual_tools_ptr_.reset(new rviz_visual_tools::RvizVisualTools("head_mount_kinect_rgb_optical_frame", "/table_detection"));
  visual_tools_ptr_->setLifetime(2);
  visual_tools_ptr_->setAlpha(0.5);

  sub_cloud_ = nh_public_.subscribe<sensor_msgs::PointCloud2>(
    cloud_topic_in_, 100, &TableDetection::cloud_callback, this);
}

void TableDetection::cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
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

    std::cout << cloud_msg_ptr->header.frame_id.c_str() << std::endl;

    sensor_msgs::PointCloud2 cloud_msg;
    tf2::doTransform(*cloud_msg_ptr, cloud_msg, transform);
    pcl::fromROSMsg(cloud_msg, *cloud_ptr_);

    tf::transformMsgToEigen(transform.transform, transform_);
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


  Eigen::Vector4d ws_min, ws_max;
  ws_min << min.x(), min.y(), min.z() - inlier_thresh_, 1.0;
  ws_max << max.x(), max.y(), max.z() + 0.5, 1.0;

  Eigen::Vector4d ws_above_min, ws_above_max;
  ws_above_min << min.x(), min.y(), min.z() + inlier_thresh_ * 2, 1.0;
  ws_above_max << max.x(), max.y(), max.z() + 0.5, 1.0;

  ws_min = transform_.inverse() * ws_min;
  ws_max = transform_.inverse() * ws_max;

  std::cout << transform_.inverse().matrix() << std::endl;

  ROS_INFO("ws:       %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
    ws_min.x(), ws_max.x(), ws_min.y(), ws_max.y(), ws_min.z(), ws_max.z());

  ROS_INFO("ws above: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f",
    ws_above_min.x(), ws_above_max.x(), ws_above_min.y(), ws_above_max.y(), ws_above_min.z(), ws_above_max.z());

  workspace_min_ = ws_above_min.head(3).cast<float>();
  workspace_max_ = ws_above_max.head(3).cast<float>();
}

void TableDetection::publishMarkers()
{
  geometry_msgs::Polygon bounds;
  for (size_t i = 0; i < cloud_bounds_ptr_->size(); i++)
  {
    geometry_msgs::Point32 p;
    p.x = cloud_bounds_ptr_->at(i).x;
    p.y = cloud_bounds_ptr_->at(i).y;
    p.z = cloud_bounds_ptr_->at(i).z;
    bounds.points.push_back(p);
  }

  visual_tools_ptr_->enableBatchPublishing();

  visual_tools_ptr_->publishPolygon(bounds, rviz_visual_tools::GREEN, rviz_visual_tools::REGULAR,
    "bounds");

  visual_tools_ptr_->publishCuboid(workspace_min_.cast<double>(), workspace_max_.cast<double>(),
    rviz_visual_tools::BLUE);

  visual_tools_ptr_->triggerBatchPublishAndDisable();
}

}