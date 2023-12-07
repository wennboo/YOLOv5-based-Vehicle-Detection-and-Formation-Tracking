#! /usr/bin/env python3
# -*- coding: utf-8 -*-

'''
    这里的x,z为twist中的x,z轴
    x_flag=0 小车没动
    x_flag=1 小车前进
    x_flag=-1 小车后退
    z_flag=0 小车没转
    z_flag=1 小车向左转
    z_flag=-1 小车向右转
'''
from geometry_msgs.msg import Twist
import rospy
from robot_follow_yolo.msg import anchor_box

def doPose(anchor):
    #x轴方向上检测小车前后移动
    x_distance_flag = 0
    x_flag = 0
    #z轴方向上检测小车左右移动
    z_distance_flag = 0
    z_flag = 0
    if anchor.proportion!=0:
        #获取参数,若参数还未设定，默认设定为0
        #计算面积
        x1 = anchor.x1
        y1 = anchor.y1
        x2 = anchor.x2
        y2 = anchor.y2
        ##考虑前后距离方面的内容
        ##***********************************************
        anchor_size = ((y1-y2)**2+(x1-x2)**2)/2
        x_dis.append(anchor_size)
       #计算小车之间的距离，距离变化过大就要开始移动了
        x_diff = x_dis[0]-anchor_size
        if abs(x_diff)/x_dis[0]>0.1: #和初始距离相比，波动大于0.1视为移动
            x_distance_flag = x_diff #小车前进
        #若x_distance_flag为正数，小车前进
        if x_distance_flag>0:
            x_flag = 1
            rospy.loginfo("forward")
        elif x_distance_flag<0:
            x_flag = -1
            rospy.loginfo("backward")
        else:
            x_flag = 0
            rospy.loginfo("x stop")
        ##***********************************************
        ##考虑左右方面的内容
        #计算矩阵的中心点(x_c,y_c)
        x_c = (x1+x2)/2 #这里不考虑小车y轴的移动
        z_dis.append(x_c)
        #计算小车中心点距离，距离变化要开始转向
        z_diff = z_dis[0]-x_c
        if abs(z_diff)/z_dis[0]>0.1: #和初始距离相比，波动大于0.1视为移动
            z_distance_flag = z_diff
        #若z_distance_flag为正数，小车正弧度方向旋转
        if z_distance_flag>0:
            z_flag = 1
            rospy.loginfo("left")
        elif z_distance_flag<0:
            z_flag = -1
            rospy.loginfo("right")
        else:
            z_flag = 0
            rospy.loginfo("z stop")
        
        #考虑向前还是向后
        twist.linear.x = x_flag*0.1
        #若x轴方向停止,小车停止转向，不然会一直自转
        if(x_flag!=0):
            twist.angular.z = z_flag*0.1

        #倘若没有检测到障碍物或者线速度为负数（倒车），就发布指令
        if not rospy.get_param("/obstacle_detected"):
            pub.publish(twist)
            rospy.loginfo("start following")
        else:
            rospy.loginfo("obstacle_detected")
            twist.linear.x=-0.1
            twist.angular.z=0
            pub.publish(twist)
    else:
        rospy.loginfo("没检测到")
    
if __name__ == "__main__":
    rospy.init_node("yolo5n_driver")
    #列表用于存储小车的运行轨迹
    x_dis = [] 
    z_dis = []  
    #创建运动消息
    twist = Twist()
    #创建运动控制节点
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    sub = rospy.Subscriber("/anchor_box",anchor_box,doPose,queue_size=100)
    rospy.spin()

        
