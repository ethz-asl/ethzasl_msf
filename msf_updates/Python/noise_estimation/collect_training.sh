#!/bin/bash
# this script takes the following argumens
# $1 is the path to the ros launch file
# $2 is the path to the rosbag file
# and runs msf using the specified launch file
#it assumes a roscore is running and /use_sim_time is set to true
timeout 102.5 rosbag play $2 --clock &
timeout 102.5 roslaunch msf_updates $1 &
#timeout 3.5 rosbag play $2 --clock &
#timeout 3.5 roslaunch msf_updates $1 &
sleep 2.5
#this is init call, might need to escape ""
rosrun dynamic_reconfigure dynparam set /msf_position_pose_sensor/position_pose_sensor "{'core_init_filter':'true'}"
