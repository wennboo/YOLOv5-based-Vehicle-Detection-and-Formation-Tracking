#!/bin/bash

#1.在hostrobot 首先启动深度相机驱动（因为会用到避障节点）以及机器人底盘驱动,同时设置命名空间
rosrun robot_follow_yolo robot_follow_yolo5n.py &
sleep 15
rosrun robot_follow_yolo  yolo5n_driver.py &
sleep 5

wait
exit 0