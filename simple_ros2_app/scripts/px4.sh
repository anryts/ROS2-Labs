#!/bin/bash
cd ~/.ros
$ROS_FOLDER/ros_labs/PX4-Autopilot/build/px4_sitl_default/bin/px4 $ROS_FOLDER/ros_labs/PX4-Autopilot/build/px4_sitl_default/etc -s etc/init.d-posix/rcS -i $1 -w sitl_iris_$1
exit 0
