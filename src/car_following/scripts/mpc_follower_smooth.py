#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import rospy
import numpy as np
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
from scipy.optimize import minimize
from math import cos, sin, atan2, sqrt
import threading

class MPCPathTracker:
    def __init__(self):
        rospy.init_node('mpc_follower_tracker', anonymous=True)
        self.leader_odom_sub = rospy.Subscriber("/leader_odom", Odometry, self.leader_odom_callback)
        self.follower_odom_sub = rospy.Subscriber("/follower_odom", Odometry, self.follower_odom_callback)
        self.cmd_vel_pub = rospy.Publisher("/follower/cmd_vel", Twist, queue_size=10)
        self.follower_path_pub = rospy.Publisher("/follower_pose", Path, queue_size=100)
        
        # 机器人状态变量
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        
        # 领航者状态变量
        self.leader_x = 0.0
        self.leader_y = 0.0
        self.leader_yaw = 0.0
        self.leader_v = 0.0
        self.leader_odom_received = False
        
        self.safe_distance = 0.7  # 安全距离阈值（0.7米）
        
        # MPC参数
        self.Np = 10  # 预测步数
        self.Nc = 5   # 控制步数
        self.dt = 0.1  # 采样时间间隔
        self.Q = np.diag([10.0, 10.0, 1.0])  # 状态误差权重
        self.R = np.diag([1.0, 0.1])  # 控制量的权重
        self.R_delta = np.diag([0.5, 0.05])  # 控制增量的变化权重
        
        # 控制约束
        self.v_min = 0.0
        self.v_max = 0.3
        self.w_min = -np.pi/4
        self.w_max = np.pi/4
        self.delta_v_max = 0.1
        self.delta_w_max = 0.05
        
        # 轨迹缓存
        self.follower_path = Path()
        self.follower_path.header.frame_id = "world"
        
        self.last_u = np.array([0.0, 0.0])
        self.data_lock = threading.Lock()
        
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.mpc_control_loop)
        rospy.loginfo("MPC Follower Node Initialized (Safe Distance: %.1f m, Publish /follower_pose as Path)", self.safe_distance)

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
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            qx = msg.pose.pose.orientation.x
            qy = msg.pose.pose.orientation.y
            qz = msg.pose.pose.orientation.z
            qw = msg.pose.pose.orientation.w
            self.current_yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
            self.current_v = msg.twist.twist.linear.x
            self.current_w = msg.twist.twist.angular.z
            
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.follower_path.header.frame_id
            pose_stamped.pose = msg.pose.pose
            self.follower_path.poses.append(pose_stamped)

    def calculate_leader_follower_distance(self):
        with self.data_lock:
            dx = self.leader_x - self.current_x
            dy = self.leader_y - self.current_y
            distance = sqrt(dx**2 + dy**2)
            return distance

    def publish_follower_path(self):
        with self.data_lock:
            self.follower_path.header.stamp = rospy.Time.now()
            self.follower_path_pub.publish(self.follower_path)

    def get_reference_sequence(self, current_x):
        with self.data_lock:
            if not self.leader_odom_received:
                return np.zeros((self.Np, 3))
            leader_current = np.array([self.leader_x, self.leader_y, self.leader_yaw])
            leader_v = self.leader_v
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

    def mpc_control_loop(self, event):
        with self.data_lock:
            if not self.leader_odom_received:
                rospy.logwarn_throttle(1.0, "No /leader_odom received yet!")
                return
            x0 = np.array([self.current_x, self.current_y, self.current_yaw])
        
        current_distance = self.calculate_leader_follower_distance()
        
        x_ref_seq = self.get_reference_sequence(x0)
        
        with self.data_lock:
            u_ref = np.array([self.leader_v, 0.0])
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
        
        if result.success:
            delta_u_opt = result.x.reshape((self.Nc, 2))
            u_opt = self.last_u + delta_u_opt[0]
            if abs(u_opt[0]) < 0.02:
                u_opt[0] = 0.0
            
            if current_distance < self.safe_distance:
                u_opt[0] = 0.0
                u_opt[1] = 0.0
                rospy.logwarn("Distance to leader (%.2f m) < safe distance (%.1f m)! Stop motion.", 
                              current_distance, self.safe_distance)
            
            self.last_u = u_opt
        else:
            rospy.logerr_throttle(1.0, "MPC Optimization Failed! Using last command.")
            u_opt = self.last_u
        
        cmd_vel = Twist()
        if current_distance < self.safe_distance:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            cmd_vel.linear.x = np.clip(u_opt[0], self.v_min, self.v_max)
            cmd_vel.angular.z = np.clip(u_opt[1], self.w_min, self.w_max)
        
        self.cmd_vel_pub.publish(cmd_vel)
        
        self.publish_follower_path()
        
        rospy.loginfo_throttle(1.0, 
            "MPC Control: v=%.2f m/s, w=%.2f rad/s | Leader Pose: (%.2f, %.2f) | Distance: %.2f m (Safe: %.1f m) | Path points: %d",
            cmd_vel.linear.x, cmd_vel.angular.z, self.leader_x, self.leader_y, current_distance, self.safe_distance,
            len(self.follower_path.poses)
        )

if __name__ == '__main__':
    try:
        tracker = MPCPathTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("MPC Follower Node Shutdown!")
        pass
