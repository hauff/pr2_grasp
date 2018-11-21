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
  float cb_arr[] = {0.0, 2.0, -1.0, 1.0, 0.1, 1.0};
  std::vector<float> crop_box_default(cb_arr, cb_arr + sizeof(cb_arr) / sizeof(float));

  float uv_arr[] = {0.0, 0.0, 1.0};
  std::vector<float> up_vector_default(uv_arr, uv_arr + sizeof(uv_arr) / sizeof(float));

  nh_private_.param("voxel_grid_size", voxel_grid_size_, 0.02f);
  nh_private_.param("crop_box", crop_box_, crop_box_default);
  nh_private_.param("up_vector", up_vector_, up_vector_default);
  nh_private_.param("up_vector_thresh", up_vector_thresh_, 0.087f);
  nh_private_.param("inlier_thresh", inlier_thresh_, 0.02f);
  nh_private_.param("min_cluster_size", min_cluster_size_, 100);
  nh_private_.param("cluster_tolerance", cluster_tolerance_, 0.04f);

  cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_filtered_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_bounds_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  inlier_ptr_.reset(new pcl::PointIndices());

  rviz_visualizer_ptr_.reset(new rviz_visualizer::RvizVisualizer(
    "odom_combined", "markers", nh_private_));
  rviz_visualizer_ptr_->setAlpha(0.5);
}

void TableDetection::run(const std::string& topic)
{
  if (topic.empty())
  {
    ROS_ERROR("[%s]: Invalid topic: '%s'.", name().c_str(), topic.c_str());
    return;
  }

  ROS_INFO("[%s]: Subscribe to topic '%s'.", name().c_str(), topic.c_str());
  sub_cloud_ = nh_public_.subscribe<sensor_msgs::PointCloud2>(
    topic, 1, &TableDetection::cloudCallback, this);
}

void TableDetection::stop()
{
  sub_cloud_.shutdown();
}

void TableDetection::detect(const sensor_msgs::PointCloud2& cloud_msg)
{
  if (cloud_msg.data.size() == 0)
  {
    ROS_WARN("[%s]: Point cloud is empty.", name().c_str());
    return;
  }

  pcl::fromROSMsg(cloud_msg, *cloud_ptr_);
  
  cloud_filtered_ptr_->header = cloud_ptr_->header;
  *cloud_filtered_ptr_ += *cloud_ptr_;
  
  cloud_bounds_ptr_->clear();
  inlier_ptr_->indices.clear();

  downsample();
  cropBox();
  estimatePlaneCoeffs();
  extractCluster();
  projectPointCloudToModel();
  computeConvexHull();
  computeBounds();

  publish();
}

void TableDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr)
{
  ROS_INFO("[%s]: Recived point cloud with frame id '%s'.", name().c_str(),
    cloud_msg_ptr->header.frame_id.c_str());

  sensor_msgs::PointCloud2 cloud_msg;
  if (transform(cloud_msg_ptr, cloud_msg))
    detect(cloud_msg);
}

bool TableDetection::transform(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr,
  sensor_msgs::PointCloud2& cloud_msg)
{
  try
  {
    geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
      "odom_combined", cloud_msg_ptr->header.frame_id, cloud_msg_ptr->header.stamp);
    tf2::doTransform(*cloud_msg_ptr, cloud_msg, transform);
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("[%s]: %s", name().c_str(), ex.what());
    return false;
  }

  return true;
}

void TableDetection::downsample()
{
  if (cloud_filtered_ptr_->empty())
    return;

  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_filtered_ptr_);
  vg.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  vg.filter(*tmp_ptr);

  if (tmp_ptr->size() < min_cluster_size_)
  {
    ROS_WARN("[%s]: Point cloud too small after downsampling. Size: %lu",
      name().c_str(), tmp_ptr->size());
    return;
  }

  cloud_filtered_ptr_ = tmp_ptr;
}

void TableDetection::cropBox()
{
  if (cloud_filtered_ptr_->empty())
    return;

  pcl::CropBox<pcl::PointXYZRGB> cb;
  cb.setInputCloud(cloud_filtered_ptr_);
  cb.setMin(Eigen::Vector4f(crop_box_[0], crop_box_[2], crop_box_[4], 0));
  cb.setMax(Eigen::Vector4f(crop_box_[1], crop_box_[3], crop_box_[5], 0));

  pcl::PointIndices::Ptr tmp_ptr(new pcl::PointIndices());
  cb.filter(tmp_ptr->indices);

  if (tmp_ptr->indices.size() < min_cluster_size_)
  {
    ROS_WARN("[%s]: Point cloud too small after cropping. Size: %lu",
      name().c_str(), tmp_ptr->indices.size());
    return;
  }

  inlier_ptr_ = tmp_ptr;
  inlier_ptr_->header = cloud_filtered_ptr_->header;
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
  sac.setAxis(Eigen::Vector3f(up_vector_.at(1), up_vector_.at(1), up_vector_.at(2)));
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

  pcl::PointCloud<pcl::PointXYZRGB> tmp;

  pcl::ConvexHull<pcl::PointXYZRGB> ch;
  ch.setInputCloud(cloud_bounds_ptr_);
  ch.reconstruct(tmp);

  pcl::copyPointCloud(tmp, *cloud_bounds_ptr_);
}

void TableDetection::computeBounds()
{
	if (inlier_ptr_->indices.empty() || cloud_filtered_ptr_->empty())
    return;

  Eigen::Vector4f min, max;
  pcl::getMinMax3D(*cloud_filtered_ptr_, *inlier_ptr_, min, max);

  table_.frame_id = cloud_filtered_ptr_->header.frame_id;
  table_.pose = Eigen::Affine3d::Identity();
  table_.pose.translation() = ((min + max) / 2).head(3).cast<double>();
  table_.dimensions = (max - min).head(3).cast<double>();

  //table_.dimensions.x() += 0.02;
  //table_.dimensions.y() += 1.5;
  table_.dimensions.z() = 0.005;
}

void TableDetection::publish()
{
 if (cloud_bounds_ptr_->empty())
  return;
  
  // Publish collision object.
  scene_mgr_.addBoxCollisionObject(table_.frame_id, "table", table_.pose, table_.dimensions);

  // Publish rviz markers.
  std::vector<Eigen::Vector3d> bounds(cloud_bounds_ptr_->size());
  for (size_t i = 0; i < cloud_bounds_ptr_->size(); i++)
  {
    bounds[i] <<
      cloud_bounds_ptr_->at(i).x,
      cloud_bounds_ptr_->at(i).y,
      cloud_bounds_ptr_->at(i).z;
  }

  rviz_visualizer_ptr_->publishPolygon(bounds, Eigen::Quaterniond::Identity(), 0.02,
    rviz_visualizer::GREEN, "Bounds");

  rviz_visualizer_ptr_->publish();

  ROS_INFO("[%s]: Published table with frame id '%s'.", name().c_str(),
    table_.frame_id.c_str());
}

}
