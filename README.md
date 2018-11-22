# pr2_grasp

An experimental playground.

## Compile

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

4. Clone this repository.
    ```
    git clone https://github.com/hauff/pr2_grasp.git -b indigo-devel
    ```
