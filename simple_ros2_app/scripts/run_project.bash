#!/usr/bin/env bash
#TODO: find out how to use properly sh script to run the project
source $ROS_FOLDER/ros_labs/src/install/setup.bash
colcon build --symlink-install
source $ROS_FOLDER/ros_labs/src/install/setup.bash
ros2 launch simple_ros2_app multri_drone_launch.xml