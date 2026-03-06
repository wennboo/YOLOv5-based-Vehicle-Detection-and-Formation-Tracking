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
from car_following.msg import Predict
import sys


def kalman_filter(miut_1,sigmat_1,ut,zt,B):
    # 里程计误差矩阵
    Rt_values = [0.01, 0.01, 0.01]  
    Rt = np.diag(Rt_values) 
    # 观测器噪声矩阵
    Qt_values = [1, 1, 1]  
    Rt = np.diag(Rt_values) 
    
    
    miut_ =  miut_1+np.dot(B,ut)
    sigmat_ = sigmat_1 + Rt
    Kt = np.dot(sigmat_,np.linalg.inv(sigmat_+Qt_values))

    miut = miut_+np.dot(Kt,zt-miut_)
    sigmat = np.dot((np.eye(3)-Kt),sigmat_)
    return miut,sigmat
    

def dopose1(odometry):
    global flag,x_noise,y_noise,cita_noise,miu_list,sigma_list,v_x_noise,v_y_noise,v_cita_noise
    global x,y,Y
    if flag:
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y
        (R, P, Y) = tf_conversions.transformations.euler_from_quaternion([odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w])
        v_x = odometry.twist.twist.linear.x
        v_y = odometry.twist.twist.linear.y
        v_cita = odometry.twist.twist.angular.z
        
        #里程计生成噪声
        x_noise = x + np.random.normal(0, 0.1) 
        y_noise = y + np.random.normal(0, 0.1) 
        cita_noise = Y + np.random.normal(0, 0.1) 
        v_x_noise = v_x + np.random.normal(0, 0.1) 
        v_y_noise = v_y + np.random.normal(0, 0.1) 
        v_cita_noise = v_cita + np.random.normal(0, 0.1)   

        flag = False

def dopose2(predict):
    global flag,x,y,Y
    if not flag:
        x_predict = predict.x
        y_predict = predict.y
        cita_predict = predict.cita/180*math.pi

        dt = 1/hz
        B = np.array([[math.cos(cita_noise)*dt,0,0],[math.sin(cita_noise)*dt,0,0],[0,0,dt]])
        ut = np.array([[v_x_noise],[v_y_noise],[v_cita_noise]])
        if len(miu_list)==0:
            miut_1 = np.array([[0],[0],[0]])
            sigmat_1 = np.zeros((3,3))
        else:
            miut_1 = miu_list[-1]
            sigmat_1 = sigma_list[-1]
        zt = np.array([[x_predict],[y_predict],[cita_predict]])
        miut,sigmat = kalman_filter(miut_1,sigmat_1,ut,zt,B)
        miu_list.append(miut)
        sigma_list.append(sigmat)
        # rate.sleep()
        kalman_x = miu_list[-1][0][0]
        kalman_y = miu_list[-1][1][0]
        kalman_theta = miu_list[-1][2][0]
        # print("希望的x:{:.2f},y:{:.2f},cita:{:.2f}".format(x,y,np.degrees(Y)))
        print("kalman预测的x:{:.2f},y:{:.2f},cita:{:.2f}".format(kalman_x,kalman_y,kalman_theta))
        kalman_fit = Predict()
        kalman_fit.x = kalman_x
        kalman_fit.y = kalman_y
        kalman_fit.cita = kalman_theta
        pub.publish(kalman_fit)
        # print("加入噪声的x:{:.2f},y:{:.2f},cita:{:.2f}".format(x_noise,y_noise,np.degrees(cita_noise)))
        # print("摄像头预测的x:{:.2f},y:{:.2f},cita:{:.2f}".format(x_predict,y_predict,np.degrees(cita_predict)))
        
        #保存数据
        
        # idea.append([x,y,np.degrees(Y)])
        # noise.append([x_noise,y_noise,np.degrees(cita_noise)])
        # prediction.append([x_predict,y_predict,np.degrees(cita_predict)])
        # print(len(idea))
        # if len(idea)==time_limit*hz:
        #     print("数据开始保存")
        #     idea_ = np.array(idea)
        #     np.save("src/xtark_urdf/data/idea.npy",idea_)
        #     noise_ = np.array(noise)
        #     np.save("src/xtark_urdf/data/noise.npy",noise_)
        #     prediction_ = np.array(prediction)
        #     np.save("src/xtark_urdf/data/prediction.npy",prediction_)
        #     miu_list_ = np.array(miu_list)
        #     np.save("src/xtark_urdf/data/miu_list.npy",miu_list_)
        #     sigma_list_ = np.array(sigma_list)
        #     np.save("src/xtark_urdf/data/sigma_list.npy",sigma_list_)
        #     print("数据保存成功")
        #     rospy.signal_shutdown("fuck me!")
        # flag = True

    

if __name__ == "__main__":
    idea  = [] #理想结果
    noise = []
    prediction = []
    miu_list = []
    sigma_list = []

    flag = True
    # 2.初始化 ros 节点
    rospy.init_node("mse_calculate")
    time_limit = 30
    hz = 100
    rate = rospy.Rate(hz)
    # 3.创建 TF 订阅对象
    sub1 = rospy.Subscriber("/leader_odom",Odometry,dopose1,queue_size=1)
    sub2 = rospy.Subscriber("/predict_pose",Predict,dopose2,queue_size=1)
    pub = rospy.Publisher("/kalman_filt",Predict,queue_size=1)
    rospy.spin()