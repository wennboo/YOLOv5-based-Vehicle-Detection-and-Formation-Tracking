#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from scipy.optimize import minimize
from math import cos, sin, atan2, sqrt
import threading
from collections import deque

class MPCPathTracker:
    def __init__(self):
        # ROS节点初始化
        rospy.init_node('mpc_follower_tracker', anonymous=True)
        
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
        
        # 领航者位姿相关（核心修改：改用缓存队列存储历史位姿）
        # 队列元素格式：(时间戳, x, y, yaw, v)
        self.leader_pose_buffer = deque(maxlen=200)  # 最多存储200个历史位姿（约20秒）
        self.leader_odom_received = False
        
        # 核心参数：跟踪领航车过去 DELAY_TIME 秒的位姿（可根据需要调整）
        self.DELAY_TIME = 2.8  # 延迟2.8秒，对应领航车以0.3m/s行驶时约0.84米的安全距离
        self.MIN_BUFFER_LEN = 10  # 缓存最少需要10个点才开始跟踪
        
        # MPC参数
        self.Np = 10
        self.Nc = 5
        self.dt = 0.1
        self.Q = np.diag([300.0, 300.0, 800.0])
        self.R = np.diag([1.0, 1.0])
        self.R_delta = np.diag([0.5, 0.05])
        
        # 控制约束
        self.v_min = 0.0
        self.v_max = 0.6
        self.w_min = -2*np.pi
        self.w_max = 2*np.pi
        self.delta_v_max = 0.1
        self.delta_w_max = 1.0
        
        # 轨迹缓存相关
        self.follower_path = Path()
        self.follower_path.header.frame_id = "world"
        self.MAX_PATH_POINTS = 1e6  # 限制轨迹点数量，避免内存溢出
        self.last_u = np.array([0.0, 0.0])
        self.data_lock = threading.Lock()
        
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.mpc_control_loop)
        rospy.loginfo("MPC Follower Node Initialized (Tracking leader's pose with %.2f s delay)", self.DELAY_TIME)

    def leader_odom_callback(self, msg):
        """订阅领航车位姿，存入历史缓存队列"""
        with self.data_lock:
            # 记录当前位姿和时间戳
            current_time = rospy.Time.now().to_sec()
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            leader_yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
            
            # 添加到缓存队列
            self.leader_pose_buffer.append((
                current_time,
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                leader_yaw,
                msg.twist.twist.linear.x
            ))
            self.leader_odom_received = True

    def get_leader_delayed_pose(self):
        """从缓存中获取领航车 DELAY_TIME 秒前的位姿"""
        with self.data_lock:
            if not self.leader_odom_received or len(self.leader_pose_buffer) < self.MIN_BUFFER_LEN:
                return None
            
            target_time = rospy.Time.now().to_sec() - self.DELAY_TIME
            
            # 从后往前找第一个时间戳小于等于target_time的位姿
            for i in range(len(self.leader_pose_buffer)-1, -1, -1):
                ts, x, y, yaw, v = self.leader_pose_buffer[i]
                if ts <= target_time:
                    return {
                        'x': x,
                        'y': y,
                        'yaw': yaw,
                        'v': v,
                        'time': ts
                    }
            
            # 如果没找到，返回最早的位姿
            ts, x, y, yaw, v = self.leader_pose_buffer[0]
            return {
                'x': x,
                'y': y,
                'yaw': yaw,
                'v': v,
                'time': ts
            }

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
            
            # 添加当前位姿到Path轨迹中，并限制数量
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.follower_path.header.frame_id
            pose_stamped.pose = msg.pose.pose
            
            self.follower_path.poses.append(pose_stamped)
            # 限制轨迹点数量，避免内存溢出
            if len(self.follower_path.poses) > self.MAX_PATH_POINTS:
                self.follower_path.poses.pop(0)

    def get_reference_sequence(self, current_x):
        """生成参考序列：基于领航车历史位姿预测未来轨迹"""
        # 获取领航车延迟后的位姿
        delayed_pose = self.get_leader_delayed_pose()
        if delayed_pose is None:
            return np.zeros((self.Np, 3))
        
        # 基于延迟位姿生成参考序列
        leader_current = np.array([delayed_pose['x'], delayed_pose['y'], delayed_pose['yaw']])
        leader_v = delayed_pose['v']
        leader_w = 0.0
    
        ref_seq = []
        for i in range(self.Np):
            t = i * self.dt
            x = leader_current[0] + leader_v * cos(leader_current[2]) * t
            y = leader_current[1] + leader_v * sin(leader_current[2]) * t
            yaw = leader_current[2] + leader_w * t
            ref_seq.append([x, y, yaw])
        return np.array(ref_seq)

    def diff_drive_model(self, x, u):
        v, w = u
        yaw = x[2]
        dx = np.array([
            v * cos(yaw),
            v * sin(yaw),
            w
        ])
        return dx

    def linearize_model(self, x_ref, u_ref):
        v_ref, w_ref = u_ref
        yaw_ref = x_ref[2]
        
        A = np.array([
            [0.0, 0.0, -v_ref * sin(yaw_ref)],
            [0.0, 0.0,  v_ref * cos(yaw_ref)],
            [0.0, 0.0,  0.0]
        ])
        
        B = np.array([
            [cos(yaw_ref), 0.0],
            [sin(yaw_ref), 0.0],
            [0.0,          1.0]
        ])
        return A, B

    def discrete_linear_model(self, A, B):
        Ad = np.eye(3) + A * self.dt
        Bd = B * self.dt
        return Ad, Bd

    def mpc_cost_function(self, delta_u_flat, *args):
        Ad, Bd, x0, x_ref_seq = args
        delta_u = delta_u_flat.reshape((self.Nc, 2))
        
        J = 0.0
        x_current = x0.copy()
        
        for k in range(self.Np):
            if k < self.Nc:
                u_k = self.last_u + delta_u[k]
            else:
                u_k = self.last_u + delta_u[-1]
            
            x_current = np.matmul(Ad, x_current) + np.matmul(Bd, u_k)
            
            x_ref = x_ref_seq[k]
            error = x_current - x_ref
            J += np.matmul(np.matmul(error.T, self.Q), error)
            J += np.matmul(np.matmul(u_k.T, self.R), u_k)
            
            if k < self.Nc:
                J += np.matmul(np.matmul(delta_u[k].T, self.R_delta), delta_u[k])
        
        return J

    def mpc_constraints(self, delta_u_flat, *args):
        delta_u = delta_u_flat.reshape((self.Nc, 2))
        constraints = []
        
        for k in range(self.Nc):
            delta_v, delta_w = delta_u[k]
            constraints.append(self.delta_v_max - abs(delta_v))
            constraints.append(self.delta_w_max - abs(delta_w))
            
            v_k = self.last_u[0] + delta_v
            w_k = self.last_u[1] + delta_w
            constraints.append(v_k - self.v_min)
            constraints.append(self.v_max - v_k)
            constraints.append(w_k - self.w_min)
            constraints.append(self.w_max - w_k)
        
        return np.array(constraints)

    def publish_follower_path(self):
        """发布跟随者轨迹"""
        with self.data_lock:
            self.follower_path.header.stamp = rospy.Time.now()
            self.follower_path_pub.publish(self.follower_path)

    def mpc_control_loop(self, event):
        with self.data_lock:
            if not self.leader_odom_received or len(self.leader_pose_buffer) < self.MIN_BUFFER_LEN:
                rospy.logwarn_throttle(1.0, "Leader pose buffer not enough! Current len: %d", len(self.leader_pose_buffer))
                return
            x0 = np.array([self.current_x, self.current_y, self.current_yaw])
        
        # 获取参考序列（基于领航车历史位姿）
        x_ref_seq = self.get_reference_sequence(x0)
        
        # 获取延迟后的领航车速度作为参考速度
        delayed_pose = self.get_leader_delayed_pose()
        u_ref = np.array([delayed_pose['v'], 0.0]) if delayed_pose else np.array([0.0, 0.0])
        
        A, B = self.linearize_model(x_ref_seq[0], u_ref)
        Ad, Bd = self.discrete_linear_model(A, B)
        
        delta_u_init = np.zeros(self.Nc * 2)
        args = (Ad, Bd, x0, x_ref_seq)
        
        cons = {
            'type': 'ineq',
            'fun': self.mpc_constraints,
            'args': args
        }
        
        result = minimize(
            self.mpc_cost_function,
            delta_u_init,
            args=args,
            constraints=cons,
            method='SLSQP',
            options={'maxiter': 100, 'disp': False}
        )
        
        # 新增：初始化损失值变量
        final_cost_J = 0.0
        
        if result.success:
            delta_u_opt = result.x.reshape((self.Nc, 2))
            u_opt = self.last_u + delta_u_opt[0]
            if abs(u_opt[0]) < 0.02:
                u_opt[0] = 0.0
            self.last_u = u_opt
            
            # 新增：计算并获取最终的损失值J
            final_cost_J = self.mpc_cost_function(result.x, *args)
        else:
            rospy.logerr_throttle(1.0, "MPC Optimization Failed! Using last command.")
            u_opt = self.last_u
        
        # 发布控制指令（移除了安全距离强制停止逻辑）
        cmd_vel = Twist()
        cmd_vel.linear.x = np.clip(u_opt[0], self.v_min, self.v_max)
        cmd_vel.angular.z = np.clip(u_opt[1], self.w_min, self.w_max)
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 发布轨迹
        self.publish_follower_path()
        
        # 日志输出（包含延迟位姿信息和新增的损失值J）
        if delayed_pose:
            rospy.loginfo_throttle(1.0, 
                "MPC Control: v=%.2f m/s, w=%.2f rad/s | Cost J=%.2f | Tracking leader's pose (%.2f, %.2f) [%.2f s ago] | Path points: %d",
                cmd_vel.linear.x, cmd_vel.angular.z, 
                final_cost_J,  # 新增：打印损失值J
                delayed_pose['x'], delayed_pose['y'],
                rospy.Time.now().to_sec() - delayed_pose['time'],
                len(self.follower_path.poses)
            )

if __name__ == '__main__':
    try:
        tracker = MPCPathTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MPC Follower Node Shutdown!")
        pass