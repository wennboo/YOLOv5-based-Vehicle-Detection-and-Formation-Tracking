#!/bin/bash

rosrun  robot_follow_yolo color_observer.py &
sleep 1
rosrun  robot_follow_yolo depth_observer.py &
sleep 1

wait
exit 0