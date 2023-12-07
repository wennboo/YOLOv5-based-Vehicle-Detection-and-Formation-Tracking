#! /usr/bin/env python
# -*- coding: utf-8 -*-


"""  
    订阅 目标点 和 机器人 的 TF 广播信息，查找并转换时间最近的 TF 信息
    将 目标点 转换成相对 机器人 的坐标，在计算线速度和角速度并发布

    实现流程:
        1.导包
        2.初始化 ros 节点
        3.创建 TF 订阅对象
        4.处理订阅到的 TF
            4-1.查找坐标系的相对关系
            4-2.生成速度信息，然后发布
"""
# 1.导包
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
import math
from nav_msgs.msg import Path

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("move_to_destination")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    # 4.处理订阅到的 TF
    rate = rospy.Rate(1)
    # 创建速度发布对象
    pub_verb = rospy.Publisher("/host/cmd_vel",Twist,queue_size=1000)

    while not rospy.is_shutdown():
        
        rate.sleep()
        try:
            #def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            trans = buffer.lookup_transform("robot1","destination",rospy.Time(0))

            # 根据转变后的坐标计算出速度和角速度信息 * /rosversion: 1.14.10

            twist = Twist()
            # 间距 = x^2 + y^2  然后开方
            # 设置相对距离有危险，若小车跟随速度过慢，将导致小车距离目标点越远，速度会变快，变得难以控制,手就因此被小车划伤
            linear_velocity = 0.5* math.sqrt(math.pow(trans.transform.translation.x,2) + math.pow(trans.transform.translation.y,2))
            
            if(linear_velocity>0,5):
                twist.linear.x=0.5
            else:
                twist.linear.x=linear_velocity
            
            twist.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
            rospy.loginfo("线速度：%.2f, 角速度：%.2f",
                twist.linear.x, twist.angular.z
            )   
            rospy.loginfo("相对坐标:(%.2f,%.2f,%.2f)",
                         trans.transform.translation.x,
                         trans.transform.translation.y,
                         trans.transform.translation.z
                         )   
            
            #倘若没有检测到障碍物或者线速度为负数（倒车），就发布指令
            if not rospy.get_param("/obstacle_detected"):
                pub_verb.publish(twist)
            else:
                twist.linear.x=-0.1
                twist.angular.z=0
                pub_verb.publish(twist)

            

        except Exception as e:
            rospy.logwarn("警告:%s",e)
