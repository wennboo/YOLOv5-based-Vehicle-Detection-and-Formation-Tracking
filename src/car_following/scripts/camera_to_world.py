#! /usr/bin/env python3
#-- coding:UTF-8 --

'''
该脚本用于发布转换后的预测目标位置
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


if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("camera_to_world")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    #发布消息
    pub = rospy.Publisher("/predict_pose",Predict,queue_size=1)
    predict = Predict()
    
    #测量观测的方差
    x_list = []
    y_list = []
    cita_list = []

    while not rospy.is_shutdown():
        try:
            #def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            trans = buffer.lookup_transform("world","leader_prediction",rospy.Time(0))
            x = trans.transform.translation.x
            #命名问题！！！！！！！！！！！！！
            y = trans.transform.translation.y
            z = 0.0
            (r, p, Y) = tf_conversions.transformations.euler_from_quaternion([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])
            R = 0
            P = 0
            Y = math.degrees(Y)
            predict.x = x
            predict.y = y
            predict.cita = Y
            pub.publish(predict)
            print(x,y,Y)
        except Exception as e:
            rospy.logwarn("warning:%s",e)
    
    # x_var = np.var(x_list)
    # y_var = np.var(y_list)
    # cita_var = np.var(cita_list)
    # print("统计数目为:{},x_方差为:{},y_方差为:{},cita_方差为:{}".format(len(x_list),x_var,y_var,cita_var))


