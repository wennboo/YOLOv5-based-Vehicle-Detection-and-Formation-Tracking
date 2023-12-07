#! /usr/bin/env python3
# -*- coding: utf-8 -*-

"""
    利用anchor_box检测到的目标框在深度相机中判断小车的距离变化
"""

# 1.导包
import rospy
from robot_follow_yolo.msg import anchor_box
from sensor_msgs.msg import Image,PointCloud2
from cv_bridge import CvBridge
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped
import tf2_ros
import time


def convert_to_matrix(lst, rows, cols):  
    # 检查输入列表长度是否与指定的长和宽匹配  
    if len(lst) != rows * cols:  
        raise ValueError("Invalid input list length for the specified rows and columns.")  
      
    # 使用列表切片操作将一维列表转换为二维矩阵  
    matrix = [lst[i:i+cols] for i in range(0, len(lst), cols)]  
      
    # 返回结果矩阵  
    return matrix

def get_2d_slice(matrix, row_start, col_start, row_end, col_end):  
    # 切片操作需要确保索引不越界  
    if row_start < 0 or col_start < 0 or row_end > len(matrix) or col_end > len(matrix[0]) or row_end <= row_start or col_end <= col_start:  
        raise IndexError("Invalid slice indices.")  
      
    # 对每一行进行切片操作，获取二维切片  
    slice_matrix = [row[col_start:col_end] for row in matrix[row_start:row_end]]  
      
    return slice_matrix

def doPose(data):
    start = time.time()
    x1 = rospy.get_param("/yolo_x1")
    y1 = rospy.get_param("/yolo_y1")
    x2 = rospy.get_param("/yolo_x2")
    y2 = rospy.get_param("/yolo_y2")
    points_list = []
    global p1,p2
    assert isinstance(data, PointCloud2)
    points = point_cloud2.read_points(data)
    points_list = list(points)
    print(type(points_list[0]))
    result = convert_to_matrix(points_list,480,640)
    slice = get_2d_slice(result,x2,y2,x1,y1)
    print(type(x1))
    # fist_element = int(x1*y1)
    # elements_in_line = int(x2-x1)
    # lines_sum = int(y2-y1)
    # object_list = []
    # for i in range(lines_sum+1):
    #     start = fist_element+i*640
    #     end = start+elements_in_line
    #     object_list.append(points_list[start:end])
    #     print(len(object_list))
    

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("predict_destination")
    bridge = CvBridge()
    anchor_depth = Image()
    #设置一个记录小车实际位置变化的列表
    distance_dic = []
    #要发布的目标点坐标
    posestamped = PoseStamped()
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    #订阅机器人的模拟雷达信息，并在回调函数中转换为
    sub = rospy.Subscriber("/camera/depth/points",PointCloud2,doPose,queue_size=100)
    pub2 = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=100)
    
    rospy.spin()