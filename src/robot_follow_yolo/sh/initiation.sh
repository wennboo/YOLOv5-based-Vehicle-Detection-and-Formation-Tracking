#!/bin/bash

#1.在hostrobot 首先启动深度相机驱动（因为会用到避障节点）以及机器人底盘驱动,同时设置命名空间
roslaunch xtark_driver xtark_driver.launch &
sleep 5
roslaunch xtark_nav_depthcamera xtark_depthcamera.launch &
sleep 5

#2.启动参数初始化节点，初始化参数为0
roslaunch robot_follow_yolo param_init.launch&
sleep 3

# #3.在vmpc 启动robot_follow_yolo5n节点，发布前方小车运动信息
# roslaunch robot_follow_yolo robot_follow_yolo5n.launch &
# sleep 5

# # #4.在hostrobot 端启动yolo5n_driver节点
# roslaunch robot_follow_yolo yolo5n_driver.launch &
# sleep 5

#5.启动避障系统
roslaunch robot_follow_yolo obstacle_avoid.launch &


wait
exit 0