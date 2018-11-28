# pr2_grasp

An experimental playground.

"Invalid sizes when resizing a matrix or array."

void CloudCamera::calculateNormalsOrganized()
{
  if (!cloud_processed_->isOrganized())
  {
    std::cout << "Error: point cloud is not organized!\n";
    return;
  }

  std::cout << "Using integral images for surface normals estimation ...\n";
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  ne.setInputCloud(cloud_processed_);
  ne.setViewPoint(view_points_(0,0), view_points_(1,0), view_points_(2,0));
  ne.setNormalEstimationMethod(ne.COVARIANCE_MATRIX);
  ne.setNormalSmoothingSize(20.0f);
  ne.compute(*cloud_normals);

  //normals_ = cloud_normals->getMatrixXfMap().cast<double>();
  Eigen::Matrix3Xf float_points = cloud_normals->getMatrixXfMap();
  normals_ = float_points.cast<double>();
}


## Compile

in grasp.h remove const getApproach, getBinormal, getAxis

1. Create catkin workspace.
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

    source devel/setup.bash
    ```

2. Clone [pcl_conversion](https://github.com/ros-perception/pcl_conversions) and
[perception_pcl](https://github.com/ros-perception/perception_pcl.git).
(only needed if your PCL version doesn't match the one shipped with ROS)
    ```
    git clone https://github.com/ros-perception/pcl_conversions.git -b indigo-devel
    git clone https://github.com/ros-perception/perception_pcl.git -b indigo-devel
    ```

3. Install [Grasp Pose Detection (GPD)](https://github.com/atenpas/gpd).
    ```
    git clone https://github.com/atenpas/gpg.git
    mkdir gpg/build && cd gpg/build
    cmake ..
    make

    sudo make install
    ```

    ```
    git clone https://github.com/atenpas/gpd.git

    catkin_make -USE_CAFFE=OFF
    ```


    If want to install GPD on a machine with GPU, you also have to clone only GPD
    ```
    <build_depend>std_msgs</build_depend>
    <build_depend>sensor_msgs</build_depend>
    <build_depend>geometry_msgs</build_depend>
    <build_depend>message_generation</build_depend>

    <buildtool_depend>catkin</buildtool_depend>
    ```

    ```
    find_package(catkin REQUIRED COMPONENTS
        std_msgs
        sensor_msgs
        geometry_msgs
        message_generation
    )

    add_message_files(
        FILES
        CloudIndexed.msg
        CloudSamples.msg
        CloudSources.msg
        GraspConfig.msg
        GraspConfigList.msg
        SamplesMsg.msg
    )

    add_service_files(
        FILES
        SetParameters.srv
    )

    generate_messages(
        DEPENDENCIES
        std_msgs
        sensor_msgs
        geometry_msgs
    )

    catkin_package(

    )
    ```

4. Clone this repository.
    ```
    git clone https://github.com/hauff/pr2_grasp.git -b indigo-devel
    ```

## Run

TODO: Fill this section.

## Troubleshooting

* Compiler error "Undefined reference to 'pcl::search::KdTree<pcl::PointXYZRGB>::KdTree(bool)'".  
    Your PCL version doesn't match the one shipped with ROS. Solution: [Step 2](#Compile).
