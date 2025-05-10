#!/usr/bin/env bash
cd $ROS_FOLDER/ros_labs/ardupilot/ArduCopter/ || exit
../Tools/autotest/sim_vehicle.py -f $1 --console -I$2
exit 0