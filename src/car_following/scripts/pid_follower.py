#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from math import cos, sin, atan2, sqrt, pi
import threading

class PIDPathTracker:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('pid_follower_tracker', anonymous=True)
        
        # 订阅话题
        self.leader_odom_sub = rospy.Subscriber("/leader_odom", Odometry, self.leader_odom_callback)
        self.follower_odom_sub = rospy.Subscriber("/follower_odom", Odometry, self.follower_odom_callback)
        
        # 发布话题
        self.cmd_vel_pub = rospy.Publisher("/follower/cmd_vel", Twist, queue_size=10)
        self.follower_path_pub = rospy.Publisher("/follower_pose", Path, queue_size=100)
        
        # 机器人状态变量
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        
        # 领航者实时位姿
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_yaw = 0.0
        self.leader_v = 0.0
        self.leader_odom_received = False
        
        # 安全距离阈值（0.7米）
        self.safe_distance = 0.2
        
        # ========== 核心修改：PID参数配置 ==========
        # 横向PID（控制角速度，跟踪航向/位置偏差）
        self.kp_angular = 1.8   # 比例系数
        self.ki_angular = 0.05  # 积分系数
        self.kd_angular = 0.1   # 微分系数
        
        # 纵向PID（控制线速度，跟踪距离）
        self.kp_linear = 0.8    # 比例系数
        self.ki_linear = 0.02   # 积分系数
        self.kd_linear = 0.08   # 微分系数
        
        # PID误差缓存
        self.last_angular_error = 0.0
        self.angular_error_integral = 0.0
        self.last_linear_error = 0.0
        self.linear_error_integral = 0.0
        
        # 控制约束（与原MPC一致）
        self.v_min = 0.0
        self.v_max = 0.7
        self.w_min = -pi/4
        self.w_max = pi/4
        
        # 轨迹缓存相关
        self.follower_path = Path()
        self.follower_path.header.frame_id = "world"
        
        self.data_lock = threading.Lock()
        self.last_control_time = rospy.Time.now()
        
        # 控制循环（频率与原MPC一致：10Hz）
        self.timer = rospy.Timer(rospy.Duration(0.1), self.pid_control_loop)
        rospy.loginfo("PID Follower Node Initialized (Safe Distance: %.1f m, Publish /follower_pose as Path)", self.safe_distance)

    def leader_odom_callback(self, msg):
        with self.data_lock:
            self.leader_x = msg.pose.pose.position.x
            self.leader_y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self.leader_yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
            self.leader_v = msg.twist.twist.linear.x
            self.leader_odom_received = True

    def follower_odom_callback(self, msg):
        with self.data_lock:
            # 更新跟随者状态变量
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self.current_yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
            self.current_v = msg.twist.twist.linear.x
            self.current_w = msg.twist.twist.angular.z
            
            # 添加当前位姿到轨迹
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.follower_path.header.frame_id
            pose_stamped.pose = msg.pose.pose
            
            self.follower_path.poses.append(pose_stamped)

    def calculate_leader_follower_distance(self):
        """计算领航者和跟随者之间的欧式距离"""
        with self.data_lock:
            dx = self.leader_x - self.current_x
            dy = self.leader_y - self.current_y
            distance = sqrt(dx**2 + dy**2)
            return distance

    def calculate_heading_error(self):
        """计算跟随者到领航者的航向误差（带安全距离的目标点）"""
        with self.data_lock:
            # 计算领航者后方安全距离处的目标点
            target_x = self.leader_x - self.safe_distance * cos(self.leader_yaw)
            target_y = self.leader_y - self.safe_distance * sin(self.leader_yaw)
            
            # 计算跟随者到目标点的期望航向
            desired_yaw = atan2(target_y - self.current_y, target_x - self.current_x)
            
            # 航向误差（归一化到[-pi, pi]）
            heading_error = desired_yaw - self.current_yaw
            heading_error = (heading_error + pi) % (2 * pi) - pi
            return heading_error

    def pid_controller(self, error, last_error, integral, kp, ki, kd, dt):
        """通用PID控制器实现"""
        # 比例项
        p_term = kp * error
        # 积分项（带积分限幅，防止积分饱和）
        integral += error * dt
        integral = np.clip(integral, -1.0, 1.0)  # 积分限幅
        i_term = ki * integral
        # 微分项
        d_term = kd * (error - last_error) / dt if dt > 0 else 0.0
        # 总输出
        output = p_term + i_term + d_term
        return output, integral

    def publish_follower_path(self):
        """发布跟随者轨迹（nav_msgs/Path格式）"""
        with self.data_lock:
            self.follower_path.header.stamp = rospy.Time.now()
            self.follower_path_pub.publish(self.follower_path)

    def pid_control_loop(self, event):
        # 检查是否收到领航者数据
        with self.data_lock:
            if not self.leader_odom_received:
                rospy.logwarn_throttle(1.0, "No /leader_odom received yet!")
                return
        
        # 计算时间间隔（用于PID微分/积分）
        current_time = rospy.Time.now()
        dt = (current_time - self.last_control_time).to_sec()
        self.last_control_time = current_time
        if dt <= 0:
            dt = 0.1  # 防止除以0
        
        # 1. 计算核心误差
        distance = self.calculate_leader_follower_distance()
        linear_error = distance - self.safe_distance  # 距离误差（目标：保持safe_distance）
        heading_error = self.calculate_heading_error()  # 航向误差
        
        # 2. 安全距离判断：小于阈值则停止
        if distance < self.safe_distance:
            linear_output = 0.0
            angular_output = 0.0
            rospy.logwarn("Distance to leader (%.2f m) < safe distance (%.1f m)! Stop motion.", 
                          distance, self.safe_distance)
        else:
            # 3. 纵向PID控制（线速度）
            linear_output, self.linear_error_integral = self.pid_controller(
                linear_error, self.last_linear_error, self.linear_error_integral,
                self.kp_linear, self.ki_linear, self.kd_linear, dt
            )
            # 4. 横向PID控制（角速度）
            angular_output, self.angular_error_integral = self.pid_controller(
                heading_error, self.last_angular_error, self.angular_error_integral,
                self.kp_angular, self.ki_angular, self.kd_angular, dt
            )
            
            # 更新误差缓存
            self.last_linear_error = linear_error
            self.last_angular_error = heading_error
        
        # 5. 控制量限幅（与原MPC约束一致）
        linear_output = np.clip(linear_output, self.v_min, self.v_max)
        angular_output = np.clip(angular_output, self.w_min, self.w_max)
        
        # 6. 发布控制指令
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_output
        cmd_vel.angular.z = angular_output
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 7. 发布轨迹（保留原有功能）
        self.publish_follower_path()
        
        # 日志输出（保留原有格式）
        rospy.loginfo_throttle(1.0, 
            "PID Control: v=%.2f m/s, w=%.2f rad/s | Leader Pose: (%.2f, %.2f) | Distance: %.2f m (Safe: %.1f m) | Path points: %d",
            cmd_vel.linear.x, cmd_vel.angular.z, self.leader_x, self.leader_y, distance, self.safe_distance,
            len(self.follower_path.poses)
        )

if __name__ == '__main__':
    try:
        tracker = PIDPathTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("PID Follower Node Shutdown!")
        pass