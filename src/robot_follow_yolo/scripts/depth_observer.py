#! /usr/bin/env python
# -*- coding: utf-8 -*-

from robot_follow_yolo.msg import anchor_box
import cv2 as cv
import rospy
from sensor_msgs.msg import CompressedImage,Image
from cv_bridge import CvBridge

def doPose1(anchor):
    global cor
    cor = []
    cor.append(anchor.x1)
    cor.append(anchor.y1)
    cor.append(anchor.x2)
    cor.append(anchor.y2)


def doPose2(image):
    global cor
    matrix = bridge.imgmsg_to_cv2(image)
    cor = list(map(int, cor))
    matrix = cv.cvtColor(matrix, cv.COLOR_GRAY2BGR)

    if len(cor)!=0:
        x_min, y_min, x_max, y_max = cor
        # 使用cv2.rectangle绘制锚框  
        # 参数分别为：图像，左上角坐标（x_min, y_min），右下角坐标（x_max, y_max），线条颜色，线条宽度  
        cv.rectangle(matrix, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  
        # 显示图像  
        cv.imshow('Image with anchor box', matrix)  
        cv.waitKey(1)  


if __name__=="__main__":
    cor = []
    rospy.init_node("depth_observer")
    anchor_cor = anchor_box()
    bridge = CvBridge()
    sub = rospy.Subscriber("/anchor_box",anchor_box,doPose1,queue_size=1)
    sub = rospy.Subscriber("/camera/depth/image",Image,doPose2,queue_size=1)
    rospy.spin()