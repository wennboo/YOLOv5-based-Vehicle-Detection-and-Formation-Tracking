#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
基于ros1操作系统，该文件用于从话题中订阅领航者的位姿/leader_pose和跟随者的位姿/follower_pose，
按10Hz采样频率同步记录数据，保存在data文件夹的单个txt文件中：
- 文件名包含自定义方法名（无时间戳）
- 单个文件中，前三行是领航者的x/y/theta所有数据，后三行是跟随者的x/y/theta所有数据
- 10Hz同步采样：同一时间点采集两者位姿，保证序列长度完全一致

数据样式：
# 领航者
x1 x2 x3 ...
y1 y2 y3 ...
theta1 theta2 theta3 ...
# 跟随者
x1 x2 x3 ...
y1 y2 y3 ...     
theta1 theta2 theta3 ...

先将数据保存在内存中，然后在程序结束时一次性写入文件中。
'''
import rospy
import os
from nav_msgs.msg import Path
import math
from threading import Lock  # 引入线程锁保证数据安全


class PoseRecorder:
    def __init__(self, data_dir="data", method="mpc", mode="circle", sample_freq=10):
        rospy.init_node('pose_recorder', anonymous=True)
        
        # 基础配置
        self.method = method
        self.mode = mode
        self.data_dir = data_dir
        self.sample_freq = sample_freq  # 采样频率(Hz)
        self.sample_interval = 1.0 / sample_freq  # 采样时间间隔(秒)
        
        # 确保数据目录存在
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        
        # 数据缓存：x、y、theta分别存储在独立列表中
        self.leader_x = []
        self.leader_y = []
        self.leader_theta = []
        
        self.follower_x = []
        self.follower_y = []
        self.follower_theta = []
        
        # 新增：缓存最新的位姿数据 + 线程锁（防止多线程读写冲突）
        self.latest_leader_pose = None  # 缓存领航者最新位姿
        self.latest_follower_pose = None  # 缓存跟随者最新位姿
        self.pose_lock = Lock()  # 线程锁
        
        # 订阅话题：只缓存最新位姿，不直接采样保存
        rospy.Subscriber('/leader_pose', Path, self.leader_callback, queue_size=10)
        rospy.Subscriber('/follower_pose', Path, self.follower_callback, queue_size=10)
        
        # 新增：启动定时采样定时器（核心：同步采样）
        self.sample_timer = rospy.Timer(rospy.Duration(self.sample_interval), self.sync_sample_pose)
        
        # 注册关闭钩子：程序结束时保存数据
        rospy.on_shutdown(self.save_data)
        
        rospy.loginfo(f"位姿记录器已启动，采样频率：{self.sample_freq}Hz")
        rospy.loginfo(f"数据将保存至：{os.path.join(self.data_dir, f'{self.method}_{self.mode}_pose_data.txt')}")

    def leader_callback(self, msg):
        """仅缓存领航者最新位姿（不做采样判断）"""
        with self.pose_lock:  # 加锁保证数据读写安全
            try:
                if msg.poses and len(msg.poses) > 0:  # 双重校验确保有位姿数据
                    pose_stamped = msg.poses[-1]
                    x = pose_stamped.pose.position.x
                    y = pose_stamped.pose.position.y
                    theta = self.get_yaw(pose_stamped.pose.orientation)
                    # 缓存最新位姿
                    self.latest_leader_pose = (x, y, theta)
            except Exception as e:
                rospy.logwarn(f"处理领航者位姿数据出错：{e}")
                self.latest_leader_pose = None

    def follower_callback(self, msg):
        """仅缓存跟随者最新位姿（不做采样判断）"""
        with self.pose_lock:  # 加锁保证数据读写安全
            try:
                if msg.poses and len(msg.poses) > 0:  # 双重校验确保有位姿数据
                    pose_stamped = msg.poses[-1]
                    x = pose_stamped.pose.position.x
                    y = pose_stamped.pose.position.y
                    theta = self.get_yaw(pose_stamped.pose.orientation)
                    # 缓存最新位姿
                    self.latest_follower_pose = (x, y, theta)
            except Exception as e:
                rospy.logwarn(f"处理跟随者位姿数据出错：{e}")
                self.latest_follower_pose = None

    def get_yaw(self, orientation):
        """从四元数提取yaw角（朝向）"""
        try:
            x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
            yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
            return yaw
        except Exception as e:
            rospy.logwarn(f"提取yaw角出错：{e}")
            return 0.0

    def sync_sample_pose(self, event):
        """定时同步采样：同一时间点采集两者位姿（核心修改）"""
        with self.pose_lock:  # 加锁防止读写冲突
            # 必须同时获取到两者的最新位姿才保存，保证序列长度一致
            if self.latest_leader_pose is not None and self.latest_follower_pose is not None:
                # 读取领航者最新位姿并保存
                l_x, l_y, l_theta = self.latest_leader_pose
                self.leader_x.append(l_x)
                self.leader_y.append(l_y)
                self.leader_theta.append(l_theta)
                
                # 读取跟随者最新位姿并保存
                f_x, f_y, f_theta = self.latest_follower_pose
                self.follower_x.append(f_x)
                self.follower_y.append(f_y)
                self.follower_theta.append(f_theta)
            else:
                # 记录采样失败的情况，便于调试
                missing = []
                if self.latest_leader_pose is None:
                    missing.append("领航者")
                if self.latest_follower_pose is None:
                    missing.append("跟随者")
                rospy.logdebug(f"采样失败：{','.join(missing)}位姿未更新，跳过本次采样")

    def validate_data_length(self):
        """校验所有数据序列长度是否一致，返回校验结果"""
        # 定义需要校验的所有序列
        sequences = [
            self.leader_x, self.leader_y, self.leader_theta,
            self.follower_x, self.follower_y, self.follower_theta
        ]
        
        # 获取基准长度（取第一个非空序列的长度）
        base_len = len(sequences[0]) if sequences else 0
        
        # 校验所有序列长度是否等于基准长度
        all_same = all(len(seq) == base_len for seq in sequences)
        
        if not all_same:
            rospy.logwarn("数据序列长度不一致！各序列长度：")
            rospy.logwarn(f"领航者 - x:{len(self.leader_x)}, y:{len(self.leader_y)}, theta:{len(self.leader_theta)}")
            rospy.logwarn(f"跟随者 - x:{len(self.follower_x)}, y:{len(self.follower_y)}, theta:{len(self.follower_theta)}")
            
            # 自动修正：截断所有序列到最短长度
            min_len = min(len(seq) for seq in sequences)
            self.leader_x = self.leader_x[:min_len]
            self.leader_y = self.leader_y[:min_len]
            self.leader_theta = self.leader_theta[:min_len]
            self.follower_x = self.follower_x[:min_len]
            self.follower_y = self.follower_y[:min_len]
            self.follower_theta = self.follower_theta[:min_len]
            
            rospy.logwarn(f"已自动修正：所有序列截断至最短长度 {min_len}")
        
        return all_same

    def save_data(self):
        """保存数据到单个文件（增加长度校验）"""
        # 第一步：先校验并修正数据长度
        self.validate_data_length()
        
        # 拼接文件名：方法名 + _pose_data.txt（无时间戳）
        file_name = f"{self.method}_{self.mode}_pose_data.txt"
        file_path = os.path.join(self.data_dir, file_name)
        
        try:
            # 写入数据：领航者前三行，跟随者后三行
            with open(file_path, 'w') as f:
                # 领航者数据
                f.write(' '.join(map(str, self.leader_x)) + '\n')
                f.write(' '.join(map(str, self.leader_y)) + '\n')
                f.write(' '.join(map(str, self.leader_theta)) + '\n')
                # 跟随者数据
                f.write(' '.join(map(str, self.follower_x)) + '\n')
                f.write(' '.join(map(str, self.follower_y)) + '\n')
                f.write(' '.join(map(str, self.follower_theta)) + '\n')
            
            rospy.loginfo(f"数据已保存到文件：{file_path}")
            rospy.loginfo(f"采样频率：{self.sample_freq}Hz，共记录数据{len(self.leader_x)}条")
            rospy.loginfo(f"领航者序列长度：{len(self.leader_x)}，跟随者序列长度：{len(self.follower_x)}")
            
        except Exception as e:
            rospy.logerr(f"保存数据失败：{e}")


if __name__ == '__main__':
    try:
        # 可自定义采样频率，比如改为sample_freq=5（5Hz）
        recorder = PoseRecorder(method="mpc", mode="sin_real", sample_freq=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("程序被中断，正在保存数据...")
    except Exception as e:
        rospy.logerr(f"程序异常退出：{e}")