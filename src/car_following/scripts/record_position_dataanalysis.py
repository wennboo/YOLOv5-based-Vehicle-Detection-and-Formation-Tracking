#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
位姿轨迹可视化脚本：
- 读取data文件夹中指定方法名的位姿数据文件
- 提取领航者和跟随者的x、y坐标，忽略theta
- 在xoy平面绘制两者的轨迹，添加图例、坐标轴标签、标题等
'''
import os
import argparse
import matplotlib.pyplot as plt
import numpy as np

def load_pose_data(file_path):
    """
    从txt文件加载位姿数据
    :param file_path: 数据文件路径
    :return: leader_xy (np.array), follower_xy (np.array)
             格式：leader_xy = [[x1,x2,...], [y1,y2,...]]
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"数据文件不存在：{file_path}")
    
    # 读取文件所有行
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    # 按格式解析：前3行是领航者（x/y/theta），后3行是跟随者（x/y/theta）
    # 只提取x和y，忽略theta
    if len(lines) < 6:
        raise ValueError("数据文件格式错误，至少需要6行数据（领航者3行+跟随者3行）")
    
    # 解析领航者x、y
    leader_x = np.array([float(num) for num in lines[0].strip().split() if num])
    leader_y = np.array([float(num) for num in lines[1].strip().split() if num])
    
    # 解析跟随者x、y
    follower_x = np.array([float(num) for num in lines[3].strip().split() if num])
    follower_y = np.array([float(num) for num in lines[4].strip().split() if num])
    
    # 验证数据长度一致
    if len(leader_x) != len(leader_y):
        raise ValueError(f"领航者x/y数据长度不一致：x={len(leader_x)}, y={len(leader_y)}")
    if len(follower_x) != len(follower_y):
        raise ValueError(f"跟随者x/y数据长度不一致：x={len(follower_x)}, y={len(follower_y)}")
    
    return (leader_x, leader_y), (follower_x, follower_y)

def plot_trajectory(leader_xy, follower_xy, method_name="mpc", save_fig=False):
    """
    绘制xoy平面轨迹图
    :param leader_xy: 领航者(x,y)坐标元组 (x_array, y_array)
    :param follower_xy: 跟随者(x,y)坐标元组 (x_array, y_array)
    :param method_name: 方法名（用于标题）
    :param save_fig: 是否保存图片
    """
    # 设置中文字体（避免中文乱码）
    plt.rcParams['font.sans-serif'] = ['DejaVu Sans']  # 英文无乱码，如需中文可替换为 ['SimHei']
    plt.rcParams['axes.unicode_minus'] = False
    
    # 创建画布
    plt.figure(figsize=(10, 8))
    
    # 绘制轨迹
    leader_x, leader_y = leader_xy
    follower_x, follower_y = follower_xy
    
    # 领航者轨迹：红色实线，带起点/终点标记
    plt.plot(leader_x, leader_y, 'r-', linewidth=2, label='Leader (领航者)', alpha=0.8)
    plt.scatter(leader_x[0], leader_y[0], color='red', s=80, marker='o', label='Leader Start', zorder=5)
    plt.scatter(leader_x[-1], leader_y[-1], color='darkred', s=80, marker='*', label='Leader End', zorder=5)
    
    # 跟随者轨迹：蓝色实线，带起点/终点标记
    plt.plot(follower_x, follower_y, 'b-', linewidth=2, label='Follower (跟随者)', alpha=0.8)
    plt.scatter(follower_x[0], follower_y[0], color='blue', s=80, marker='o', label='Follower Start', zorder=5)
    plt.scatter(follower_x[-1], follower_y[-1], color='darkblue', s=80, marker='*', label='Follower End', zorder=5)
    
    # 设置图表属性
    plt.xlabel('X Coordinate (m)', fontsize=12)
    plt.ylabel('Y Coordinate (m)', fontsize=12)
    plt.title(f'Leader-Follower Trajectory (Method: {method_name})', fontsize=14, fontweight='bold')
    plt.legend(loc='best', fontsize=10)
    plt.grid(True, alpha=0.3, linestyle='--')
    plt.axis('equal')  # 等比例坐标轴，避免轨迹变形
    
    # 保存图片（可选）
    if save_fig:
        fig_dir = "figures"
        if not os.path.exists(fig_dir):
            os.makedirs(fig_dir)
        fig_path = os.path.join(fig_dir, f"{method_name}_trajectory.png")
        plt.savefig(fig_path, dpi=300, bbox_inches='tight')
        print(f"轨迹图已保存到：{fig_path}")
    
    # 显示图表
    plt.show()

def main():
    # 命令行参数解析
    parser = argparse.ArgumentParser(description='Leader-Follower 轨迹可视化工具')
    parser.add_argument('--method', '-m', default='mpc', help='方法名（对应数据文件名前缀，默认mpc）')
    parser.add_argument('--mode', default='circle', help='模式名（对应数据文件名，默认normal）')
    parser.add_argument('--data_dir', '-d', default='data', help='数据文件夹路径（默认data）')
    parser.add_argument('--save', '-s', action='store_true', help='是否保存轨迹图到figures文件夹')
    
    args = parser.parse_args()
    
    # 拼接数据文件路径
    # file_name = f"{args.method}_{args.mode}_pose_data.txt"
    file_name = f"mpc_sin_real_pose_data_.txt"
    file_path = os.path.join(args.data_dir, file_name)
    
    try:
        # 加载数据
        print(f"正在加载数据：{file_path}")
        leader_xy, follower_xy = load_pose_data(file_path)
        print(f"数据加载成功！")
        print(f"领航者轨迹点数量：{len(leader_xy[0])}")
        print(f"跟随者轨迹点数量：{len(follower_xy[0])}")
        
        # 绘制轨迹
        plot_trajectory(leader_xy, follower_xy, method_name=args.method, save_fig=args.save)
        
    except Exception as e:
        print(f"错误：{e}")
        exit(1)

if __name__ == '__main__':
    main()