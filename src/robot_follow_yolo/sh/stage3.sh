#!/bin/bash

roslaunch xtark_nav_depthcamera xtark_depthcamera.launch &
# sleep 9
# roslaunch /home/xtark/ros_ws/src/xtark_nav_depthcamera/launch/include/omni/teb_move_base.launch &
sleep 8
rosrun robot_follow_yolo robot_follow_yolo5n.py &
# sleep 30
# rosrun robot_follow_yolo predict_destination.py &
# sleep 3

wait
exit 0