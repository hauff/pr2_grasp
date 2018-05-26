# pr2_grasp

An experimental playground.

'''
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch pr2_moveit_config warehouse.launch moveit_warehouse_database_path:=~/.moveit_db
roslaunch pr2_moveit_config move_group.launch
rviz
roslaunch gpd tutorial1.launch
roslaunch pr2_teleop_general pr2_teleop_general_keyboard.launch
'''