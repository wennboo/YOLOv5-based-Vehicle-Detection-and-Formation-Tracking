#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2 as cv
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import torch
import numpy as np
import time
from car_following.msg import Anchor

def preprocess(image):
    matrix = bridge.compressed_imgmsg_to_cv2(image)
    matrix_t = cv.resize(matrix,(640,640))
    matrix_t = torch.from_numpy(matrix_t.astype(np.float32))
    matrix_t = torch.permute(matrix_t,(2,0,1)).unsqueeze(0).to("cuda")
    return matrix_t,matrix


    
def doPose(image):
    start = time.time()
    matrix_t,matrix = preprocess(image)
    res = model(matrix_t)
    end = time.time()
    delay = end-start
    fps = 1 / delay
    pred = res.squeeze(0)

    x1 = int(pred[0] * 640)
    y1 = int(pred[1] * 480)
    x2 = int(pred[2] * 640)
    y2 = int(pred[3] * 480)
    print(matrix.shape)
    # 使用cv2.rectangle绘制锚框  
    # 参数分别为：图像，左上角坐标（x_min, y_min），右下角坐标（x_max, y_max），线条颜色，线条宽度 
    
    cv.rectangle(matrix, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv.putText(matrix,"fps:{:.3f} delay:{:.3f}".format(fps,delay),(30,30),cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)  
    # 显示图像  
    cv.imshow('Image with anchor box', matrix)  
    # print(f"fps:{fps}")
    print("delay:{:3f}ms".format((end-start)*1000))
    cv.waitKey(1)  

    #发送anchor
    anchor = Anchor()
    anchor.x1 = x1
    anchor.y1 = y1
    anchor.x2 = x2
    anchor.y2 = y2
    print(anchor)
    pub.publish(anchor)
   


if __name__=="__main__":
    cor = []
    rospy.init_node("color_observer")
    #model loading
    model = torch.load("/home/lab/pose_estimation_follower/src/car_following/scripts/model_csp3_1000.pth")
    bridge = CvBridge()
    sub = rospy.Subscriber("/follower/camera/image_raw/compressed",CompressedImage,doPose,queue_size=1)
    pub = rospy.Publisher("/anchor",Anchor,queue_size=1)
    rospy.spin()