#! /usr/bin/env python3

from car_following.msg import Anchor
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import queue
import time
import math
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf

"""
    订阅方:
        订阅消息

"""

def point2xyz(p_x,p_y,depth_value):
    """
    P = K-1*Z*Puv
    K: [381.36246688113556, 0.0, 320.5, 0.0, 381.36246688113556, 240.5, 0.0, 0.0, 1.0] 为内参矩阵 3*3
    Z: 深度值 1*1
    Puv: [u,v,1].T 3*1
    P: [x,y,z].T 3*1
    """
    
    K = np.array([  
    [381.36246688113556, 0.0, 320.5],  
    [0.0, 381.36246688113556, 240.5],  
    [0.0, 0.0, 1.0]  
    ])
    Z = depth_value
    Puv = np.array([[p_x],[p_y],[1]])
    #numpy的dot是矩阵乘法，multiply和*是点乘
    P = np.dot(np.linalg.inv(K)*Z,Puv)
    x = P[2,0]
    z = -P[1,0]
    y = -P[0,0]
    return x,y,z

def curve_fit(calculate_way):
    #让长度宽的作决策面
    points_np = np.array(calculate_way)  
    slope, intercept = np.polyfit(points_np[:, 0], points_np[:, 1], 1)  
    
    return np.degrees(np.arctan(slope))

def distance(p1,p2):
    return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2)

def direction(point_left,point_right):
    #决定小车朝向
    stable_factor = 1
    left_len = len(point_left)
    right_len = len(point_right)
    if(left_len<=stable_factor or right_len<=stable_factor):
        return "s"
    # 使用列表推导式提取第一个元素  
    
    residual_left = sum(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  
                    for (x1, y1), (x2, y2) in zip(point_left[:-1], point_left[1:]))  
    # 计算残差和  
    residual_right = sum(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)  
                    for (x1, y1), (x2, y2) in zip(point_right[:-1], point_right[1:]))  
    return "l" if residual_left>residual_right else "r"

def doPose1(p):
    global cor
    cor = [p.x1, p.y1, p.x2, p.y2]  
    

def doPose2(image):
    start = time.time()
    global cor

    depth_image = bridge.imgmsg_to_cv2(image)
    if depth_image is None:  
        print("Image is not loaded")  
    depth_image = np.nan_to_num(depth_image,nan=8)

    #模拟加入噪声
    mean = 0
    stddev = 0.1
    noise = np.random.normal(mean,stddev,depth_image.shape).astype("uint8")*255
    depth_image = depth_image

    #找到一条距离最近的点，拟合一条直线
    car_depth = depth_image[cor[1]:cor[3],cor[0]:cor[2]] #小车深度切片
    min_indices = np.argmin(car_depth,axis=1)  

    mid_x = int((cor[0]+cor[2])/2)
    mid_y = int((cor[1]+cor[3])/2)

    x_m,y_m,z_m = point2xyz(mid_x,mid_y,depth_image[mid_y,mid_x])
    depth_image_color = cv.cvtColor(depth_image,cv.COLOR_GRAY2BGR)
    
    cv.rectangle(depth_image_color, (cor[0], cor[1]), (cor[2], cor[3]), (0, 255, 0), 2)
    depth_image_color = cv.normalize(depth_image_color,None,0,16,cv.NORM_MINMAX)
    for cord in enumerate(min_indices):
        cord_ = (cord[1]+cor[0],cord[0]+cor[1])
        cv.circle(depth_image_color, cord_, 1, (0, 0, 255), 4)
    #取中间位置的值
    min_indices_len = len(min_indices)
    start_index = int(min_indices_len/3)
    end_index = start_index*2 #1/3到2/3之间的分区
    mid_value = min_indices[start_index:end_index]
    unstable_factor = np.std(mid_value)
    predict_line = int(np.mean(mid_value))
   
    #判断角度
    check_points = []
    x1 = predict_line
    y1 = int(car_depth.shape[0]/2)
    left_point = car_depth[y1,:]
    index_not_8 =  np.where((left_point-car_depth[y1,x1])<=0.3) #0.3是小车长度,0.2是小车宽度
    left_board = index_not_8[0][0]
    right_board = index_not_8[0][-1]
    #拐角处的值
    x_predict,y_predict,z_predict = point2xyz(x1+cor[0],y1+cor[1],car_depth[y1,x1])

    detect_point = car_depth[y1,left_board:right_board] #一行x的值
    #采样
    step = 2
    index = np.arange(0, len(detect_point), step).tolist()
    sample_data = detect_point[index]

    x = left_board+cor[0]
    y = y1+cor[1]
    point_left = []
    point_right = []
    
    for data in sample_data:
        x_,y_,z_ = point2xyz(x,y,data)
        if x<predict_line+cor[0]:
            point_left.append((x_,y_))
        else:
            point_right.append((x_,y_))
        x = x+step
    
    flag = direction(point_left,point_right)
    
    if flag=="l":
        print("left")
        if predict_line>car_depth.shape[1]/2:
            #用左面计算
            alpha = curve_fit(point_left)
        else:
            #用右面计算
            alpha = 90+curve_fit(point_right)
    elif flag=="r":
        print("right")
        if predict_line>car_depth.shape[1]/2:
            #用左面计算
            alpha = curve_fit(point_left)-90
        else:
            #用右面计算
            alpha = curve_fit(point_right)
    else:
        alpha = 0
        
    alpha = alpha if (alpha<87 and alpha>-87) else 0
   
    #小车位姿估计矫正
    
    l = 0.3
    w = l/2
    
    if alpha == 0:
        #如果角度为0，那么检测为中间点向前半个小车长度
        x_real = x_m+l/2
        y_real = y_m
    
    elif alpha > 0:
        alpha_ = alpha*math.pi/180
        xl = l*math.cos(alpha_)
        yl = l*math.sin(alpha_)
        xr = w*math.sin(alpha_)
        yr = -w*math.cos(alpha_)
        x_real = x_predict + (xl+xr)/2
        y_real = y_predict + (yl+yr)/2
    else:
        alpha_ = -alpha*math.pi/180
        xl = w*math.sin(alpha_)
        yl = w*math.cos(alpha_)
        xr = l*math.cos(alpha_)
        yr = -l*math.sin(alpha_)
        x_real = x_predict + (xl+xr)/2
        y_real = y_predict + (yl+yr)/2

    cv.putText(depth_image_color,"x:{:.4f}  y:{:.4f}".format(x_real,y_real),(30,30),cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)  
    
    
    
    broadcaster = tf2_ros.TransformBroadcaster()
    #         4-2.创建 广播的数据(通过 pose 设置)
    tfs = TransformStamped()
    tfs.header.frame_id = "follower_depthcamera"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "leader_prediction"
    tfs.transform.translation.x = x_real
    tfs.transform.translation.y = y_real
    tfs.transform.translation.z = 0.0
    qtn = tf.transformations.quaternion_from_euler(0,0,alpha*math.pi/180)
    tfs.transform.rotation.x = qtn[0]
    tfs.transform.rotation.y = qtn[1]
    tfs.transform.rotation.z = qtn[2]
    tfs.transform.rotation.w = qtn[3]
    #         4-3.广播器发布数据
    broadcaster.sendTransform(tfs)

    cv.putText(depth_image_color,"cita:{:.3f}".format(alpha),(30,120),cv.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 2)
    cv.imshow("depth",depth_image_color)
    cv.waitKey(1)
    

if __name__ == "__main__":

    plt.figure()
    cor = []
    #1.初始化节点
    rospy.init_node("car_depth")
    bridge = CvBridge()
    #2.创建订阅者对象

    sub1 = rospy.Subscriber("/anchor",Anchor,doPose1,queue_size=1)
    sub2 = rospy.Subscriber("/follower/depthcamera/depth/image_raw",Image,doPose2,queue_size=1)
    
    rospy.spin() #4.循环
