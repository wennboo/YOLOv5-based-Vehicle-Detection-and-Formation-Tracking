#! /usr/bin/env python3
# -*- coding: utf-8 -*-


from pathlib import Path

# 从摄像头中读入图片
import numpy as np
import torch
from ultralytics.utils.plotting import Annotator
from  ultralytics.utils.plotting import colors
from models.common import DetectMultiBackend
from utils.augmentations import letterbox
from utils.general import check_img_size
from utils.general import non_max_suppression
from utils.torch_utils import select_device
from  utils.general import scale_boxes
from geometry_msgs.msg import PoseArray
import cv2
import rospy
from sensor_msgs.msg import Image,CompressedImage
from PIL import Image
from cv_bridge import CvBridge
from io import BytesIO
import time
import math
import pickle
from robot_follow_yolo.msg import anchor_box

# 打开摄像头，0表示使用默认的摄像头
# 导入模型
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # yolopractice/YoloComputerCameraObjectDetection.py
device = 0
device = select_device(device)
dnn = False
imgsz = (640, 640)
#注意这里要用绝对路径
weights = ROOT / 'last.pt'
# data = ROOT / 'data/coco128.yaml',  # dataset.yaml path
model = DetectMultiBackend(weights=weights,device=device,dnn=dnn, fp16=False)
stride, names, pt = model.stride, model.names, model.pt
imgsz = check_img_size(imgsz, s=stride)  # check image size
#预定义目标框的面积
anchor_size = []
#默认小车不动
distance_flag = 0
#跳帧检测计数
count = 4
# 创建一个VideoWriter对象
def doPose(image):
    global distance_flag
    global count
    #程序开始时间
    start = time.time()
    
    if count==0:
        im0 = bridge.compressed_imgmsg_to_cv2(image)
        # 将RGB图像数据转换为NumPy数组
        
        #图片预处理
        im = letterbox(im0)[0]  # padded resize
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)  # contiguous
        
        # 预测
        im = torch.from_numpy(im).to(model.device)
        im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
        im /= 255  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim
        
        pred = model(im)
        pred = non_max_suppression(pred, 0.25, 0.45, None, False, max_det=1000)
        # #后面的交给虚拟机
        # #倘若没检测到，那么就不作改变
        if len(pred[0])!=0:
            #首先去掉外面包围的list
            pred_ = pred[0]
            #取倒数第二列最大的值的行最为检测的目标
            index = torch.argmax(pred_[:,-2])
            #作直线预测，首先计算目标框的面积
            result = pred_[index]  
            rospy.set_param("yolo_x1",int(result[0]))
            rospy.set_param("yolo_y1",int(result[1]))
            rospy.set_param("yolo_x2",int(result[2]))
            rospy.set_param("yolo_y2",int(result[3]))
            rospy.set_param("yolo_proportion",float(result[4]))
            print("ready!")
        count = 4
    else:
        count -= 1
    
    #程序结束时间
    end = time.time()
    print("fps: %.2f" % (1 / (end - start)))
    

# 释放摄像头资源并关闭窗口

cv2.destroyAllWindows()

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("robot_detection")
    bridge = CvBridge()
    #用Image存储坐标点信息，后期再优化
    anchor_cor = anchor_box()
    #订阅机器人的模拟雷达信息，并在回调函数中转换为
    sub = rospy.Subscriber("/camera/image_raw/compressed",CompressedImage,doPose,queue_size=100)
    #     4.创建订阅对象
    rospy.spin()
