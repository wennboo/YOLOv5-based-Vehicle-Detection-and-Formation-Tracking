'''
比较mpc和pid算法的推理时间
============================================================
PERFORMANCE TEST RESULTS (Time in milliseconds)
============================================================
Test conditions: 1000 points, warmup 100 points

MPC Controller:
  Average time: 17.5561 ms (±9.9292)
  Min time:     4.6771 ms
  Max time:     57.7386 ms
  10Hz requirement: 100.00 ms (MPC: True)

PID Controller:
  Average time: 0.0409 ms (±0.0051)
  Min time:     0.0387 ms
  Max time:     0.0849 ms
  10Hz requirement: 100.00 ms (PID: True)

Comparison:
  MPC is 429.26x slower than PID
  PID is 0.00x faster than MPC
============================================================
'''
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import numpy as np
import rospy
from math import cos, sin, pi, atan2, sqrt
from collections import deque
from scipy.optimize import minimize

# ===================== 通用工具函数 =====================
def calculate_yaw_from_quaternion(qx, qy, qz, qw):
    """从四元数计算偏航角"""
    return atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))

def generate_test_trajectory(num_points=1000, v=0.3, w=0.1):
    """生成测试用的轨迹序列（模拟领航者运动）"""
    trajectory = []
    x, y, yaw = 0.0, 0.0, 0.0
    dt = 0.1
    
    for i in range(num_points):
        # 模拟匀速圆周运动
        x += v * cos(yaw) * dt
        y += v * sin(yaw) * dt
        yaw += w * dt
        
        # 构造位姿数据（时间戳, x, y, yaw, v）
        timestamp = i * dt
        trajectory.append({
            'time': timestamp,
            'x': x,
            'y': y,
            'yaw': yaw,
            'v': v
        })
    
    return trajectory

# ===================== MPC核心计算模块（剥离ROS） =====================
class MPCCore:
    def __init__(self):
        # MPC参数（与原代码一致）
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
        
        # 延迟参数
        self.DELAY_TIME = 2.8
        self.leader_pose_buffer = deque(maxlen=200)
        
        # 状态变量
        self.last_u = np.array([0.0, 0.0])

    def add_leader_pose(self, pose):
        """添加领航者位姿到缓存"""
        self.leader_pose_buffer.append((
            pose['time'],
            pose['x'],
            pose['y'],
            pose['yaw'],
            pose['v']
        ))

    def get_leader_delayed_pose(self):
        """获取延迟后的领航者位姿"""
        if len(self.leader_pose_buffer) < 10:
            return None
        
        current_time = self.leader_pose_buffer[-1][0]  # 使用最后一个点的时间作为当前时间
        target_time = current_time - self.DELAY_TIME
        
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
        
        # 返回最早的位姿
        ts, x, y, yaw, v = self.leader_pose_buffer[0]
        return {
            'x': x,
            'y': y,
            'yaw': yaw,
            'v': v,
            'time': ts
        }

    def get_reference_sequence(self, x0):
        """生成参考序列"""
        delayed_pose = self.get_leader_delayed_pose()
        if delayed_pose is None:
            return np.zeros((self.Np, 3))
        
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

    def linearize_model(self, x_ref, u_ref):
        """线性化模型"""
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
        """离散化模型"""
        Ad = np.eye(3) + A * self.dt
        Bd = B * self.dt
        return Ad, Bd

    def mpc_cost_function(self, delta_u_flat, *args):
        """代价函数"""
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
        """约束函数"""
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

    def compute_control(self, follower_pose):
        """核心计算函数（输入跟随者位姿，输出控制量）"""
        # 跟随者当前状态
        x0 = np.array([follower_pose['x'], follower_pose['y'], follower_pose['yaw']])
        
        # 获取参考序列
        x_ref_seq = self.get_reference_sequence(x0)
        
        # 获取延迟后的领航车速度作为参考速度
        delayed_pose = self.get_leader_delayed_pose()
        u_ref = np.array([delayed_pose['v'], 0.0]) if delayed_pose else np.array([0.0, 0.0])
        
        # 线性化+离散化
        A, B = self.linearize_model(x_ref_seq[0], u_ref)
        Ad, Bd = self.discrete_linear_model(A, B)
        
        # 优化求解
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
        
        # 计算控制输出
        if result.success:
            delta_u_opt = result.x.reshape((self.Nc, 2))
            u_opt = self.last_u + delta_u_opt[0]
            if abs(u_opt[0]) < 0.02:
                u_opt[0] = 0.0
            self.last_u = u_opt
        else:
            u_opt = self.last_u
        
        # 控制量限幅
        v = np.clip(u_opt[0], self.v_min, self.v_max)
        w = np.clip(u_opt[1], self.w_min, self.w_max)
        
        return v, w

# ===================== PID核心计算模块（剥离ROS） =====================
class PIDCore:
    def __init__(self):
        # PID参数（与原代码一致）
        self.kp_angular = 1.8
        self.ki_angular = 0.05
        self.kd_angular = 0.1
        
        self.kp_linear = 0.8
        self.ki_linear = 0.02
        self.kd_linear = 0.08
        
        # 误差缓存
        self.last_angular_error = 0.0
        self.angular_error_integral = 0.0
        self.last_linear_error = 0.0
        self.linear_error_integral = 0.0
        
        # 安全距离
        self.safe_distance = 0.7
        
        # 控制约束
        self.v_min = 0.0
        self.v_max = 0.7
        self.w_min = -pi/4
        self.w_max = pi/4
        
        # 状态变量
        self.last_control_time = 0.0

    def set_leader_pose(self, leader_pose):
        """设置领航者位姿"""
        self.leader_x = leader_pose['x']
        self.leader_y = leader_pose['y']
        self.leader_yaw = leader_pose['yaw']
        self.leader_v = leader_pose['v']

    def calculate_leader_follower_distance(self, follower_pose):
        """计算距离"""
        dx = self.leader_x - follower_pose['x']
        dy = self.leader_y - follower_pose['y']
        return sqrt(dx**2 + dy**2)

    def calculate_heading_error(self, follower_pose):
        """计算航向误差"""
        # 计算领航者后方安全距离处的目标点
        target_x = self.leader_x - self.safe_distance * cos(self.leader_yaw)
        target_y = self.leader_y - self.safe_distance * sin(self.leader_yaw)
        
        # 期望航向
        desired_yaw = atan2(target_y - follower_pose['y'], target_x - follower_pose['x'])
        
        # 航向误差归一化
        heading_error = desired_yaw - follower_pose['yaw']
        heading_error = (heading_error + pi) % (2 * pi) - pi
        return heading_error

    def pid_controller(self, error, last_error, integral, kp, ki, kd, dt):
        """通用PID控制器"""
        p_term = kp * error
        integral += error * dt
        integral = np.clip(integral, -1.0, 1.0)
        i_term = ki * integral
        d_term = kd * (error - last_error) / dt if dt > 0 else 0.0
        return p_term + i_term + d_term, integral

    def compute_control(self, follower_pose, current_time):
        """核心计算函数"""
        # 计算时间间隔
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        if dt <= 0:
            dt = 0.1
        
        # 计算误差
        distance = self.calculate_leader_follower_distance(follower_pose)
        linear_error = distance - self.safe_distance
        heading_error = self.calculate_heading_error(follower_pose)
        
        # 安全距离判断
        if distance < self.safe_distance:
            linear_output = 0.0
            angular_output = 0.0
        else:
            # 纵向PID
            linear_output, self.linear_error_integral = self.pid_controller(
                linear_error, self.last_linear_error, self.linear_error_integral,
                self.kp_linear, self.ki_linear, self.kd_linear, dt
            )
            
            # 横向PID
            angular_output, self.angular_error_integral = self.pid_controller(
                heading_error, self.last_angular_error, self.angular_error_integral,
                self.kp_angular, self.ki_angular, self.kd_angular, dt
            )
            
            # 更新误差缓存
            self.last_linear_error = linear_error
            self.last_angular_error = heading_error
        
        # 控制量限幅
        linear_output = np.clip(linear_output, self.v_min, self.v_max)
        angular_output = np.clip(angular_output, self.w_min, self.w_max)
        
        return linear_output, angular_output

# ===================== 性能测试主程序 =====================
def run_performance_test():
    # 1. 配置测试参数
    num_test_points = 1000  # 测试点数
    warmup_points = 100     # 预热点数（排除初始化耗时）
    
    # 2. 生成测试轨迹
    print("Generating test trajectory...")
    test_trajectory = generate_test_trajectory(num_points=num_test_points + warmup_points)
    
    # 3. 初始化MPC和PID核心模块
    print("Initializing controllers...")
    mpc_core = MPCCore()
    pid_core = PIDCore()
    
    # 4. 预热阶段（填充MPC缓存，初始化PID状态）
    print(f"Warming up with {warmup_points} points...")
    for i in range(warmup_points):
        pose = test_trajectory[i]
        mpc_core.add_leader_pose(pose)
        pid_core.set_leader_pose(pose)
        
        # 模拟跟随者位姿（滞后领航者0.5米）
        follower_pose = {
            'x': pose['x'] - 0.5 * cos(pose['yaw']),
            'y': pose['y'] - 0.5 * sin(pose['yaw']),
            'yaw': pose['yaw'],
            'v': pose['v']
        }
        
        # 预热计算
        mpc_core.compute_control(follower_pose)
        pid_core.compute_control(follower_pose, pose['time'])
    
    # 5. 正式测试阶段
    print(f"Starting performance test with {num_test_points} points...")
    
    # MPC性能测试
    mpc_times = []
    for i in range(warmup_points, warmup_points + num_test_points):
        pose = test_trajectory[i]
        mpc_core.add_leader_pose(pose)
        
        # 模拟跟随者位姿
        follower_pose = {
            'x': pose['x'] - 0.5 * cos(pose['yaw']),
            'y': pose['y'] - 0.5 * sin(pose['yaw']),
            'yaw': pose['yaw'],
            'v': pose['v']
        }
        
        # 计时
        start_time = time.perf_counter()
        mpc_core.compute_control(follower_pose)
        end_time = time.perf_counter()
        
        # 记录耗时（毫秒）
        elapsed = (end_time - start_time) * 1000
        mpc_times.append(elapsed)
    
    # PID性能测试
    pid_times = []
    for i in range(warmup_points, warmup_points + num_test_points):
        pose = test_trajectory[i]
        pid_core.set_leader_pose(pose)
        
        # 模拟跟随者位姿
        follower_pose = {
            'x': pose['x'] - 0.5 * cos(pose['yaw']),
            'y': pose['y'] - 0.5 * sin(pose['yaw']),
            'yaw': pose['yaw'],
            'v': pose['v']
        }
        
        # 计时
        start_time = time.perf_counter()
        pid_core.compute_control(follower_pose, pose['time'])
        end_time = time.perf_counter()
        
        # 记录耗时（毫秒）
        elapsed = (end_time - start_time) * 1000
        pid_times.append(elapsed)
    
    # 6. 计算统计结果
    mpc_avg = np.mean(mpc_times)
    mpc_std = np.std(mpc_times)
    mpc_min = np.min(mpc_times)
    mpc_max = np.max(mpc_times)
    
    pid_avg = np.mean(pid_times)
    pid_std = np.std(pid_times)
    pid_min = np.min(pid_times)
    pid_max = np.max(pid_times)
    
    # 7. 输出测试报告
    print("\n" + "="*60)
    print("PERFORMANCE TEST RESULTS (Time in milliseconds)")
    print("="*60)
    print(f"Test conditions: {num_test_points} points, warmup {warmup_points} points")
    print("\nMPC Controller:")
    print(f"  Average time: {mpc_avg:.4f} ms (±{mpc_std:.4f})")
    print(f"  Min time:     {mpc_min:.4f} ms")
    print(f"  Max time:     {mpc_max:.4f} ms")
    print(f"  10Hz requirement: {100.0:.2f} ms (MPC: {mpc_avg < 100.0})")
    print("\nPID Controller:")
    print(f"  Average time: {pid_avg:.4f} ms (±{pid_std:.4f})")
    print(f"  Min time:     {pid_min:.4f} ms")
    print(f"  Max time:     {pid_max:.4f} ms")
    print(f"  10Hz requirement: {100.0:.2f} ms (PID: {pid_avg < 100.0})")
    print("\nComparison:")
    print(f"  MPC is {mpc_avg/pid_avg:.2f}x slower than PID")
    print(f"  PID is {pid_avg/mpc_avg:.2f}x faster than MPC")
    print("="*60)

if __name__ == '__main__':
    # 可选：如果需要在ROS环境中运行，初始化节点（非必须）
    try:
        rospy.init_node('controller_performance_test', anonymous=True)
    except:
        pass
    
    # 运行性能测试
    run_performance_test()