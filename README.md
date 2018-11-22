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
    git clone -b indigo-devel https://github.com/ros-perception/pcl_conversions.git
    git clone -b indigo-devel https://github.com/ros-perception/perception_pcl.git
    ```

3. Install [Grasp Pose Detection (GPD)](https://github.com/atenpas/gpd).


```
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2_teleop_general pr2_teleop_general_keyboard.launch
roslaunch pr2_moveit_config warehouse.launch moveit_warehouse_database_path:=~/warehouse_db
roslaunch pr2_moveit_config move_group.launch
roslaunch gpd tutorial1.launch
```

```
roslaunch planning_scene_manager add_collision_box.launch
rosrun table_detection table_detection_node
rosrun pr2_grasp pr2_grasp_node
```

```
<param name="$(arg node)/finger_width" value="0.01" />
<!-- <param name="$(arg node)/hand_outer_diameter" value="0.09" /> -->
<param name="$(arg node)/hand_outer_diameter" value="0.12" /> 
<param name="$(arg node)/hand_depth" value="0.06" />
<param name="$(arg node)/hand_height" value="0.02" />
<param name="$(arg node)/init_bite" value="0.01" />
```

```
<launch>
  
  <!-- Load hand geometry parameters -->  
  <include file="$(find gpd)/launch/hand_geometry.launch">
    <arg name="node" value="detect_grasps" />
  </include>
  
  <!-- Load classifier parameters -->  
  <include file="$(find gpd)/launch/caffe/classifier_15channels.launch">
    <arg name="node" value="detect_grasps" />
  </include>
  
  <node name="detect_grasps" pkg="gpd" type="detect_grasps" output="screen">
    
    <!-- If sequential importance sampling is used (default: false) -->
    <param name="use_importance_sampling" value="false" />
        
    <!-- What type of point cloud is used and what ROS topic it comes from -->
    <param name="cloud_type" value="0" /> <!-- 0: PointCloud2, 1: CloudSized, 2: CloudIndexed, 3: CloudSamples -->
    <!--<param name="cloud_topic" value="/camera/depth_registered/points" />-->
    <param name="cloud_topic" value="/pr2_grasp/point_cloud"/>
    
    <!-- (optional) The ROS topic that the samples come from (default: an empty string) -->
    <param name="samples_topic" value="" />
    
    <!-- Plotting parameters -->
    <param name="plot_normals" value="false" />
    <param name="plot_samples" value="false" />    
    <param name="plot_candidates" value="false" />
    <param name="plot_filtered_grasps" value="false" />
    <param name="plot_valid_grasps" value="false" />
    <param name="plot_clusters" value="false" />
    <param name="plot_selected_grasps" value="false" />
    <param name="rviz_topic" value="grasps_rviz" />
    
    <!-- Preprocessing of point cloud -->
    <param name="voxelize" value="true"/>
    <param name="remove_outliers" value="false"/>
    <rosparam param="workspace"> [-1, 1, -1, 1, -1, 1] </rosparam>
    <rosparam param="camera_position"> [0, 0, 0] </rosparam>
            
    <!-- General parameters -->
    <param name="num_samples" value="100" />
    <param name="num_threads" value="4" />
    
    <!-- Parameters for local grasp candidate search -->
    <param name="nn_radius" value="0.01" />
    <param name="num_orientations" value="8" /> <!-- Number of orientations to consider -->
    
    <!-- Filtering of grasp candidates --> 
    <param name="filter_grasps" value="false" /> <!-- on workspace and robot hand aperture -->
    <rosparam param="workspace_grasps"> [-1, 1, -1, 1, -1, 1] </rosparam>
    <param name="filter_half_antipodal" value="false"/> <!-- on half antipodal -->
    
    <!-- Grasp image creation -->
    <param name="create_image_batches" value="false" /> <!-- creates grasp images in batches (less memory usage) -->
    <param name="remove_plane_before_image_calculation" value="false" /> <!-- removes table plane from point cloud to speed up shadow computations -->
    
    <!-- Clustering of grasps -->
    <param name="min_inliers" value="1" />
        
    <!-- Grasp selection -->
    <param name="min_score_diff" value="0" />
    <param name="min_aperture" value="0.029" />
    <param name="max_aperture" value="0.072" />
    <param name="num_selected" value="10" />
        
  </node>
  
</launch>
```
