#!/bin/bash

#1.在hostrobot 首先启动深度相机驱动以及机器人底盘驱动,同时设置命名空间
roslaunch robot_follow xtark_driver.launch &
sleep 5
roslaunch robot_follow xtark_nav_depthcamera.launch &
sleep 5

#2.在hostrobot 启动destination_pub节点,以及hostrobot的里程计订阅节点host_robot
roslaunch robot_follow destination_pub.launch &
sleep 5
roslaunch robot_follow host_robot.launch &
sleep 5

#3.启动避障系统
roslaunch robot_follow obstacle_avoid.launch &
sleep 5

#4.启动move_to_destination节点,控制机器人移动
roslaunch robot_follow move_to_destination.launch &
sleep 5

#5.启动乌龟控制节点
#roslaunch turtlesim turtle_teleop_key &
#sleep 5


wait
exit 0