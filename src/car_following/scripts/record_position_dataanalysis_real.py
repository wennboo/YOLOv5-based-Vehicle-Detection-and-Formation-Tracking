import os
import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd  # 用于读取CSV文件

def load_follower_data_from_txt(file_path):
    """
    从指定txt文件的第4、5行加载跟随者位姿数据（仅x/y）
    :param file_path: txt数据文件路径
    :return: follower_xy (tuple) - (follower_x, follower_y)
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"跟随者数据文件不存在：{file_path}")
    
    # 读取文件所有行
    with open(file_path, 'r') as f:
        lines = f.readlines()
    
    # 验证文件行数（至少5行，因为要取第4、5行，索引从0开始是3、4）
    if len(lines) < 5:
        raise ValueError(f"跟随者数据文件行数不足，至少需要5行（当前：{len(lines)}行）")
    
    # 解析第4行（索引3）为跟随者x，第5行（索引4）为跟随者y
    follower_x = np.array([float(num) for num in lines[3].strip().split() if num])
    follower_y = np.array([float(num) for num in lines[4].strip().split() if num])
    
    # 验证数据长度一致
    if len(follower_x) != len(follower_y):
        raise ValueError(f"跟随者x/y数据长度不一致：x={len(follower_x)}, y={len(follower_y)}")
    
    print(f"跟随者数据加载成功！轨迹点数量：{len(follower_x)}")
    return (follower_x, follower_y)

def load_leader_data_from_csv(file_path):
    """
    从指定CSV文件加载领航者位姿数据（Fused_X/Fused_Y列）
    :param file_path: CSV数据文件路径
    :return: leader_xy (tuple) - (leader_x, leader_y)
    """
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"领航者数据文件不存在：{file_path}")
    
    # 读取CSV文件，仅提取Fused_X和Fused_Y列
    df = pd.read_csv(file_path, usecols=['Fused_X', 'Fused_Y'])
    
    # 转换为numpy数组
    leader_x = df['Fused_X'].values
    leader_y = df['Fused_Y'].values
    
    # 验证数据长度一致
    if len(leader_x) != len(leader_y):
        raise ValueError(f"领航者x/y数据长度不一致：x={len(leader_x)}, y={len(leader_y)}")
    
    print(f"领航者数据加载成功！轨迹点数量：{len(leader_x)}")
    return (leader_x, leader_y)

def generate_theta_data(length, init_theta=0.0, step=0.0):
    """
    生成theta（角度）数据（默认全0，也可自定义步进）
    :param length: theta数组长度（与x/y一致）
    :param init_theta: 初始角度
    :param step: 角度步进值
    :return: theta_array (np.array)
    """
    theta_array = np.full(length, init_theta)  # 默认全0，简化处理
    # 如需线性变化的theta，可替换为：theta_array = np.linspace(init_theta, init_theta + step*(length-1), length)
    return theta_array

def save_pose_data_to_txt(leader_xy, follower_xy, save_filename="mpc_line_real_pose_data.txt"):
    """
    将领航者+跟随者位姿数据保存为txt，格式：
    前三行：领航者x、y、theta
    后三行：跟随者x、y、theta
    文本内无注释，数值用空格分隔
    :param leader_xy: 领航者(x,y)元组
    :param follower_xy: 跟随者(x,y)元组
    :param save_filename: 保存的文件名
    """
    # 提取x/y数据
    leader_x, leader_y = leader_xy
    follower_x, follower_y = follower_xy
    
    # 生成theta数据（长度与x/y一致，默认全0）
    leader_theta = generate_theta_data(len(leader_x))
    follower_theta = generate_theta_data(len(follower_x))
    
    # 转换为空格分隔的字符串
    leader_x_str = ' '.join([f"{num:.6f}" for num in leader_x])
    leader_y_str = ' '.join([f"{num:.6f}" for num in leader_y])
    leader_theta_str = ' '.join([f"{num:.6f}" for num in leader_theta])
    
    follower_x_str = ' '.join([f"{num:.6f}" for num in follower_x])
    follower_y_str = ' '.join([f"{num:.6f}" for num in follower_y])
    follower_theta_str = ' '.join([f"{num:.6f}" for num in follower_theta])
    
    # 拼接所有行（无注释）
    lines = [
        leader_x_str,
        leader_y_str,
        leader_theta_str,
        follower_x_str,
        follower_y_str,
        follower_theta_str
    ]
    
    # 保存到当前文件夹
    save_path = os.path.join(os.getcwd(), save_filename)
    with open(save_path, 'w') as f:
        f.write('\n'.join(lines))
    
    print(f"\n位姿数据已保存到当前文件夹：{save_path}")
    print(f"文件格式验证：")
    print(f"  第1行：领航者x（{len(leader_x)}个点）")
    print(f"  第2行：领航者y（{len(leader_y)}个点）")
    print(f"  第3行：领航者theta（{len(leader_theta)}个点）")
    print(f"  第4行：跟随者x（{len(follower_x)}个点）")
    print(f"  第5行：跟随者y（{len(follower_y)}个点）")
    print(f"  第6行：跟随者theta（{len(follower_theta)}个点）")

def plot_trajectory(leader_xy, follower_xy, method_name="mpc", save_fig=False):
    """
    绘制xoy平面轨迹图（保留原有绘图逻辑，仅微调注释）
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
        print(f"\n轨迹图已保存到：{fig_path}")
    
    # 显示图表
    plt.show()

def main():
    # 命令行参数解析（简化，保留核心参数）
    parser = argparse.ArgumentParser(description='Leader-Follower 轨迹可视化+数据保存工具（指定数据源）')
    parser.add_argument('--method', '-m', default='mpc', help='方法名（用于标题，默认mpc）')
    parser.add_argument('--save', '-s', action='store_true', help='是否保存轨迹图到figures文件夹')
    parser.add_argument('--save_data', '-sd', default=True, action='store_true', help='是否保存位姿数据到当前文件夹的txt文件')
    
    args = parser.parse_args()
    
    # ************************ 核心：固定数据源路径 ************************
    # 跟随者数据路径（txt文件，取第4、5行）
    follower_txt_path = "/home/thr/pose_estimation_follower/src/car_following/scripts/data/mpc_sin_real_pose_data.txt"
    # 领航者数据路径（CSV文件，Fused_X/Fused_Y列）
    leader_csv_path = "/home/thr/pose_estimation_follower/src/car_following/scripts/data/ch5/data/fused_xy_data_sin.csv"
    
    try:
        # 加载跟随者数据（从txt第4、5行）
        print(f"\n===== 加载跟随者数据 =====")
        follower_xy = load_follower_data_from_txt(follower_txt_path)
        
        # 加载领航者数据（从CSV）
        print(f"\n===== 加载领航者数据 =====")
        leader_xy = load_leader_data_from_csv(leader_csv_path)
        
        # 保存位姿数据到txt（当前文件夹）
        if args.save_data:
            print(f"\n===== 保存位姿数据 =====")
            save_pose_data_to_txt(leader_xy, follower_xy, save_filename="mpc_sin_real_pose_data.txt")
        
        # 绘制轨迹
        print(f"\n===== 绘制轨迹图 =====")
        plot_trajectory(leader_xy, follower_xy, method_name=args.method, save_fig=args.save)
        
    except Exception as e:
        print(f"\n错误：{e}")
        exit(1)

if __name__ == '__main__':
    main()