#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    利用anchor_box检测到的目标框在深度相机中判断小车的距离变化
"""

# 1.导包
import rospy
from robot_follow_yolo.msg import anchor_box
from sensor_msgs.msg import Image,CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
import time
import math
import tf
import statistics
from geometry_msgs.msg import PoseStamped


cor = []

def doPose1(anchor):
    global cor
    cor = []
    if anchor.proportion!=0:
        #获取参数,若参数还未设定，默认设定为0
        #计算面积
        cor.append(anchor.x1)
        cor.append(anchor.y1)
        cor.append(anchor.x2)
        cor.append(anchor.y2)      

def doPose2(image):
    global cor
    global distance_dic
    global posestamped
    if len(cor)!=0:    
        #获得深度信息
        cv_image = bridge.imgmsg_to_cv2(image)
        #获得深度信息对应目标框元素
        cor = [int(num) for num in cor]  
        depth_pixels_in_anchor = cv_image[cor[0]:cor[2],cor[1]:cor[3]]
        #将截取的深度图像再转换为Image发送
        if(depth_pixels_in_anchor.shape[0]!=0):
            #去掉矩阵中的nan元素，nan元素的产生源于距离过远
            non_empty_values = [float(value) for row in depth_pixels_in_anchor for value in row if not math.isnan(value)]
            try:    
                #只取1σ以内的值，提升程序的鲁棒性  
                # 计算平均值  
                mean = statistics.mean(non_empty_values)  
                # 计算标准差  
                std_dev = statistics.stdev(non_empty_values)  
                # 确定1sigma的范围  
                lower_bound = mean - std_dev  
                upper_bound = mean + std_dev  
                # 找出1sigma以内的值，提高检测的稳定性
                values_within_1sigma = [x for x in non_empty_values if lower_bound <= x <= upper_bound] 
                mean_extra = statistics.mean(values_within_1sigma)
                var_extra = statistics.stdev(values_within_1sigma)**2
                # print("平均值：{}，方差：{}".format(mean_extra,var_extra))
                # 预测小车目标点
                distance_dic.append(mean_extra) #首项distance_dic[0]为初始目标点距离
                d_r_now = mean_extra #这是实时目标
                #当前目标框的中心的坐标
                x_mean = (cor[0]+cor[2])/2 
                y_mean = (cor[1]+cor[3])/2 
                #坐标变换
                
                #计算sigma0，这里假设单位像素点与单位距离线性相关，x_real和y_real是被跟踪小车当前的预测位置
                sigma0 = math.atan(y_mean/x_mean)
                x_real = d_r_now*math.cos(sigma0)
                y_real = d_r_now*math.sin(sigma0)
                #小车需要时刻保持和被跟踪小车初始距离一样的距离
                d_r = distance_dic[0]
                #如果距离的绝对值大于0.1,就开始移动
                if(abs((d_r_now-d_r)/d_r)>0.1):
                    if mean_extra<d_r:
                        print("后退")
                    else:
                        print("前进")
                    x_dest = (d_r_now-d_r)/d_r_now*x_real
                    y_dest = (d_r_now-d_r)/d_r_now*y_real
                    sigma_dest = sigma0
                    #转换成geometry_msgs/PoseStamped发布
                    posestamped.header.frame_id = ""
                    posestamped.pose.position.x = x_dest
                    posestamped.pose.position.y = y_dest
                    qtn = tf.transformations.quaternion_from_euler(0,0,sigma_dest)
                    posestamped.pose.orientation.x = qtn[0]
                    posestamped.pose.orientation.y = qtn[1]
                    posestamped.pose.orientation.z = qtn[2]
                    posestamped.pose.orientation.w = qtn[3]
                    #发布消息
                    # pub2.publish(posestamped)
            except:
                print("设备不稳定！")
            depth_image = bridge.cv2_to_imgmsg(depth_pixels_in_anchor)
            pub1.publish(depth_image)


if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("predict_destination")
    bridge = CvBridge()
    anchor_depth = Image()
    #设置一个记录小车实际位置变化的列表
    distance_dic = []
    #要发布的目标点坐标
    posestamped = PoseStamped()
    #订阅机器人的模拟雷达信息，并在回调函数中转换为
    sub = rospy.Subscriber("/anchor_box",anchor_box,doPose1,queue_size=100)
    sub = rospy.Subscriber("/camera/depth/image",Image,doPose2,queue_size=100)
    pub1 = rospy.Publisher("/anchor_depth",Image,queue_size=100)
    pub2 = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=100)
    
    rospy.spin()