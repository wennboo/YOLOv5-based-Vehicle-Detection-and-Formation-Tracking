#! /usr/bin/env python3

# 1.导包
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, Twist
import math
import numpy as np
import time 
import tf_conversions

if __name__ == "__main__":
    # 2.初始化 ros 节点
    rospy.init_node("save_prediction")
    # 3.创建 TF 订阅对象
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    # 4.处理订阅到的 TF
    rate = rospy.Rate(10)
    # 创建速度发布对象
    start_time = time.time()
    end_time = 1.0

    alg1_list = []
    alg2_list = []
    idea_list = []
    while end_time-start_time<10: #测试时间为20s

        rate.sleep()
        try:
            #def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            trans_alg1 = buffer.lookup_transform("leader_prediction","world",rospy.Time(0))
            trans_alg2 = buffer.lookup_transform("kalman_predict_past","world",rospy.Time(0))
            trans_idea = buffer.lookup_transform("leader_base_link","world",rospy.Time(0))
            rospy.loginfo("相对坐标:(%.2f,%.2f,%.2f)",
                        trans_alg2.transform.translation.x,
                        trans_alg2.transform.translation.y,
                        trans_alg2.transform.translation.z
                        ) 
            (r_1, p_1, Y_1) = tf_conversions.transformations.euler_from_quaternion([trans_alg1.transform.rotation.x,trans_alg1.transform.rotation.y,trans_alg1.transform.rotation.z,trans_alg1.transform.rotation.w])  
            (r_2, p_2, Y_2) = tf_conversions.transformations.euler_from_quaternion([trans_alg2.transform.rotation.x,trans_alg2.transform.rotation.y,trans_alg2.transform.rotation.z,trans_alg2.transform.rotation.w])
            (r_i, p_i, Y_i) = tf_conversions.transformations.euler_from_quaternion([trans_idea.transform.rotation.x,trans_idea.transform.rotation.y,trans_idea.transform.rotation.z,trans_idea.transform.rotation.w])
            
            Y_1 = math.degrees(Y_1)
            Y_2 = math.degrees(Y_2)
            Y_i = math.degrees(Y_i)

            alg1_list.append((trans_alg1.transform.translation.x,trans_alg1.transform.translation.y,Y_1))
            alg2_list.append((trans_alg2.transform.translation.x,trans_alg2.transform.translation.y,Y_2))
            idea_list.append((trans_idea.transform.translation.x,trans_idea.transform.translation.y,Y_i))

        except Exception as e:
            rospy.logwarn("警告:%s",e)

        end_time = time.time()

    print("数据保存中，请稍等……")
    np.save("src/car_following/scripts/alg1_list",np.array(alg1_list))
    np.save("src/car_following/scripts/alg2_list",np.array(alg2_list))
    np.save("src/car_following/scripts/idea_list",np.array(idea_list))
    print("数据保存成功")