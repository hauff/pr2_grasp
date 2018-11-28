# pr2_grasp

An experimental playground.

## Compile

1. Install [Grasp Pose Generator (GPG)](https://github.com/atenpas/gpg).
    ```
    git clone https://github.com/atenpas/gpg.git
    mkdir gpg/build && cd gpg/build
    cmake ..
    make

    sudo make install
    ```

2. Create catkin workspace.
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/
    catkin_make

    source devel/setup.bash
    ```

3. Clone [pcl_conversion](https://github.com/ros-perception/pcl_conversions) and
[perception_pcl](https://github.com/ros-perception/perception_pcl.git).
(only needed if your PCL version doesn't match the one shipped with ROS)
    ```
    git clone https://github.com/ros-perception/pcl_conversions.git -b indigo-devel
    git clone https://github.com/ros-perception/perception_pcl.git -b indigo-devel
    ```
  
4. Clone [Grasp Pose Detection (GPD)](https://github.com/hauff/gpd).
    ```
    git clone https://github.com/hauff/gpd.git -b forward
    ```

5. Clone this repository.
    ```
    git clone https://github.com/hauff/pr2_grasp.git -b indigo-devel
    ```

## Run

TODO: Fill this section.

## Troubleshooting

* **Compiler error "Undefined reference to pcl::search::KdTree<pcl::PointXYZRGB>::KdTree(bool)"**  
  Your PCL version doesn't match the one shipped with ROS.  
  Solution: [Step 2](#Compile).
  
* **Runtime error "Invalid sizes when resizing a matrix or array."**  
  Solution: Modify "cloud_camera.cpp" in [Grasp Pose Generator (GPG)](https://github.com/atenpas/gpg).
    ```
    void CloudCamera::calculateNormalsOrganized()
    {
      ...

      //normals_ = cloud_normals->getMatrixXfMap().cast<double>();
      Eigen::Matrix3Xf float_points = cloud_normals->getMatrixXfMap();
      normals_ = float_points.cast<double>();
    }
    ```
