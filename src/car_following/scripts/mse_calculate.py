#! /usr/bin/env python3
#-- coding:UTF-8 --

'''
该脚本用于计算里程计和预测目标的方差
'''

# 1.导包
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
import math
import tf_conversions
import numpy as np
from nav_msgs.msg import Odometry
from xtark_urdf.msg import Predict

def dopose1(odometry):
    global x_real,y_real,cita_real,flag
    #未加噪声的真实里程计数据
    #flag用于保持两线程的同步
    if flag:
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        (R, P, Y) = tf_conversions.transformations.euler_from_quaternion([odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w])
        x_real.append(x)
        y_real.append(y)
        cita_real.append(Y)
        # print("1")
        flag = False

def dopose2(predict):
    global x_predict,y_predict,cita_predict,flag,x_real,y_real,cita_real
    if not flag:
        x_predict.append(predict.x)
        y_predict.append(predict.y)
        cita_predict.append(predict.cita/180*math.pi)
        # print("2")
        flag = True

    #x_real和x_predict在主进程调用就会失效，不知原因
    if len(x_real)%100==0:
        x_real_a = np.array(x_real)
        x_predict_a = np.array(x_predict)
        x_mean = np.mean(x_real_a-x_predict_a)
        x_var = np.var(x_real_a-x_predict_a)
        y_real_a = np.array(y_real)
        y_predict_a = np.array(y_predict)
        y_mean = np.mean(y_real_a-y_predict_a)
        y_var = np.var(y_real_a-y_predict_a)
        cita_real_a = np.array(cita_real)
        cita_predict_a = np.array(cita_predict)
        cita_mean = np.mean(cita_real_a-cita_predict_a)
        cita_var = np.var(cita_real_a-cita_predict_a)
        print("x_mean:{},x_var:{}".format(x_mean,x_var))
        print("y_mean:{},y_var:{}".format(y_mean,y_var))
        print("cita_mean:{},cita_var:{}".format(cita_mean,cita_var))
        x_real = []
        y_real = []
        cita_real = []
        x_predict = []
        y_predict = []
        cita_predict = []

    

if __name__ == "__main__":

    x_real = []
    y_real = []
    cita_real = []
    x_predict = []
    y_predict = []
    cita_predict = []
    flag = True
    
    # 2.初始化 ros 节点
    rospy.init_node("mse_calculate")

    # 3.创建 TF 订阅对象
    sub1 = rospy.Subscriber("/leader_odom",Odometry,dopose1,queue_size=1000)
    sub2 = rospy.Subscriber("/predict_pose",Predict,dopose2,queue_size=1000)

    rospy.spin()