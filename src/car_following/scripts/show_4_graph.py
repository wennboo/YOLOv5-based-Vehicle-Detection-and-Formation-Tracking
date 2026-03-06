import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from scipy.interpolate import interp1d

# ===================== 配置参数 =====================
BASE_DIR = os.path.dirname(os.path.abspath(sys.argv[0]))
DATA_DIR = os.path.join(BASE_DIR, "data")  
PIC_DIR = os.path.join(DATA_DIR, "pic")

# 要处理的文件列表 + 绘图配置
FILE_CONFIGS = [
    {
        "filename": "mpc_circle_pose_data.txt",
        "is_circular": True,
        "algorithm": "MPC",
        "trajectory": "Circle"
    },
    {
        "filename": "mpc_sin_pose_data.txt",
        "is_circular": False,
        "algorithm": "MPC",
        "trajectory": "Sine"
    },
    {
        "filename": "pid_circle_pose_data.txt",
        "is_circular": True,
        "algorithm": "PID",
        "trajectory": "Circle"
    },
    {
        "filename": "pid_sin_pose_data.txt",
        "is_circular": False,
        "algorithm": "PID",
        "trajectory": "Sine"
    }
]

# 插值配置（固定时间步，单位：秒）
TIME_STEP = 0.1  # 与仿真控制周期一致
METRICS = ["RMSE", "MAE", "Max Error"]  # 统一误差指标名称
# 绘图统一配置（轨迹图/偏航角图尺寸、清晰度完全一致）
FIG_SIZE = (8, 6)       # 统一画布尺寸 (宽, 高)
DPI = 300               # 统一保存清晰度
YAW_Y_LIM = (-10, 10)   # 偏航角纵轴固定范围
ANGLE_UNIT = "rad"      # 偏航角单位：rad(弧度)/deg(角度)，仅标注

# ===================== 核心函数 =====================
def read_pose_data(file_path):
    """读取单个位姿数据文件，返回leader_data/follower_data（含x/y/theta）或(None, None)"""
    leader_data = {'x': [], 'y': [], 'theta': []}
    follower_data = {'x': [], 'y': [], 'theta': []}
    
    try:
        if not os.path.exists(file_path):
            print(f"⚠️ 文件不存在：{file_path}")
            print(f"  提示：当前脚本目录为 {BASE_DIR}，请确认data文件夹在此目录下")
            return None, None
        
        with open(file_path, 'r', encoding='utf-8') as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]
        
        if len(lines) < 6:
            print(f"⚠️ 文件数据不足：{file_path}（至少需要6行）")
            return None, None
        
        # 解析领航者/跟随者数据（x/y/theta各3行）
        leader_lines, follower_lines = [], []
        is_leader, is_follower = False, False
        for line in lines:
            if '# 领航者' in line or '#leader' in line:
                is_leader, is_follower = True, False
                continue
            elif '# 跟随者' in line or '#follower' in line:
                is_leader, is_follower = False, True
                continue
            
            if is_leader and len(leader_lines) < 3:
                leader_lines.append(line)
            elif is_follower and len(follower_lines) < 3:
                follower_lines.append(line)
        
        # 兼容纯数据格式：前3行领航者(x/y/theta)，后3行跟随者(x/y/theta)
        if not leader_lines and not follower_lines:
            leader_lines = lines[:3]
            follower_lines = lines[3:6]
        
        # 转换为数值数组（x/y/theta全部读取）
        leader_data['x'] = np.array([float(num) for num in leader_lines[0].split()])
        leader_data['y'] = np.array([float(num) for num in leader_lines[1].split()])
        leader_data['theta'] = np.array([float(num) for num in leader_lines[2].split()])  # 领航者偏航角
        follower_data['x'] = np.array([float(num) for num in follower_lines[0].split()])
        follower_data['y'] = np.array([float(num) for num in follower_lines[1].split()])
        follower_data['theta'] = np.array([float(num) for num in follower_lines[2].split()])  # 跟随者偏航角
        
        # 对齐数据长度（原始数据初步截断，保证x/y/theta点数一致）
        min_len = min(len(leader_data['x']), len(follower_data['x']))
        for key in ['x', 'y', 'theta']:
            leader_data[key] = leader_data[key][:min_len]
            follower_data[key] = follower_data[key][:min_len]
        
        print(f"✅ 成功读取：{file_path}（原始数据点数：{min_len}）")
        return leader_data, follower_data
    
    except Exception as e:
        print(f"❌ 读取文件出错 {file_path}：{str(e)}")
        return None, None

def interpolate_data(x, y, time_step=TIME_STEP):
    """对离散位姿数据插值，生成等时间步的对齐序列"""
    t_original = np.arange(len(x)) * time_step
    t_target = np.arange(0, t_original[-1], time_step)
    
    f_x = interp1d(t_original, x, kind='linear', fill_value='extrapolate')
    f_y = interp1d(t_original, y, kind='linear', fill_value='extrapolate')
    
    x_interp = f_x(t_target)
    y_interp = f_y(t_target)
    
    return x_interp, y_interp, t_target

def interpolate_yaw(yaw, time_step=TIME_STEP):
    """对偏航角数据插值，生成等时间步的平滑序列（匹配位置插值的时间轴）"""
    t_original = np.arange(len(yaw)) * time_step
    t_target = np.arange(0, t_original[-1], time_step)
    
    f_yaw = interp1d(t_original, yaw, kind='linear', fill_value='extrapolate')
    yaw_interp = f_yaw(t_target)
    
    return yaw_interp, t_target

def calculate_pos_error(leader_x, leader_y, follower_x, follower_y):
    """计算位置（XOY）误差指标：RMSE(米)、MAE(米)、Max Error(米)"""
    # 计算位置欧式距离误差
    pos_errors = np.sqrt((leader_x - follower_x)**2 + (leader_y - follower_y)**2)
    rmse = np.sqrt(np.mean(pos_errors**2))
    mae = np.mean(np.abs(pos_errors))
    max_error = np.max(np.abs(pos_errors))
    return {
        "RMSE": rmse,
        "MAE": mae,
        "Max Error": max_error
    }

def calculate_yaw_error(leader_yaw, follower_yaw):
    """计算偏航角误差指标：RMSE、MAE、Max Error（与角度单位一致，rad/deg）"""
    # 计算偏航角绝对误差（直接差值，因已插值对齐）
    yaw_errors = np.abs(leader_yaw - follower_yaw)
    rmse = np.sqrt(np.mean(yaw_errors**2))
    mae = np.mean(yaw_errors)
    max_error = np.max(yaw_errors)
    return {
        "RMSE": rmse,
        "MAE": mae,
        "Max Error": max_error
    }

def plot_single_trajectory(config):
    """绘制单个轨迹图并保存（统一尺寸/清晰度）"""
    filename = config["filename"]
    is_circular = config["is_circular"]
    algo = config["algorithm"]
    traj = config["trajectory"]
    
    file_path = os.path.join(DATA_DIR, filename)
    pic_filename = filename.replace(".txt", "_trajectory.png")
    pic_path = os.path.join(PIC_DIR, pic_filename)
    
    leader_data, follower_data = read_pose_data(file_path)
    if leader_data is None or follower_data is None:
        print(f"❌ 跳过绘制轨迹：{filename}（数据读取失败）")
        return None, None
    
    # 插值对齐数据（用于绘图，保证轨迹平滑）
    leader_x_interp, leader_y_interp, _ = interpolate_data(leader_data['x'], leader_data['y'])
    follower_x_interp, follower_y_interp, _ = interpolate_data(follower_data['x'], follower_data['y'])
    
    # 配置绘图样式（全局统一）
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['font.size'] = 10
    plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
    plt.rcParams['figure.facecolor'] = 'white'
    
    fig, ax = plt.subplots(figsize=FIG_SIZE)
    ax.plot(leader_x_interp, leader_y_interp, 'b-', linewidth=2.5, label='Leader', alpha=0.9)
    ax.plot(follower_x_interp, follower_y_interp, 'r-', linewidth=2, label='Follower', alpha=0.8)
    
    # 标记起点
    ax.scatter(leader_x_interp[0], leader_y_interp[0], color='blue', s=80, marker='o', zorder=10)
    ax.scatter(follower_x_interp[0], follower_y_interp[0], color='red', s=80, marker='s', zorder=10)
    
    ax.set_xlabel('X (m)', fontsize=16)
    ax.set_ylabel('Y (m)', fontsize=16)
    ax.legend(loc='upper left', fontsize=16)
    ax.grid(True, alpha=0.3)
    
    # 圆形轨迹强制等比例
    if is_circular:
        ax.axis('equal')
    
    plt.tight_layout()
    plt.savefig(pic_path, dpi=DPI, bbox_inches='tight')
    print(f"✅ 轨迹图已保存至：{pic_path}")
    plt.close(fig)
    
    # 计算位置误差指标
    pos_metrics = calculate_pos_error(leader_x_interp, leader_y_interp, follower_x_interp, follower_y_interp)
    return leader_data, follower_data, pos_metrics

def plot_single_yaw(config, leader_data, follower_data):
    """绘制单个偏航角对比曲线并保存，同时计算偏航角误差指标"""
    filename = config["filename"]
    algo = config["algorithm"]
    traj = config["trajectory"]
    
    # 偏航角图片保存路径
    pic_filename = filename.replace(".txt", "_yaw_angle.png")
    pic_path = os.path.join(PIC_DIR, pic_filename)
    
    try:
        # 对偏航角插值平滑（匹配位置数据的时间轴，保证数据对齐）
        leader_yaw_interp, t_target = interpolate_yaw(leader_data['theta'])
        follower_yaw_interp, _ = interpolate_yaw(follower_data['theta'])
        
        # 配置绘图样式（与轨迹图全局统一）
        plt.rcParams['axes.unicode_minus'] = False
        plt.rcParams['font.size'] = 10
        plt.rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
        plt.rcParams['figure.facecolor'] = 'white'
        
        # 绘制偏航角对比图（使用统一画布尺寸）
        fig, ax = plt.subplots(figsize=FIG_SIZE)
        unit_str = "弧度" if ANGLE_UNIT == "rad" else "角度"
        
        # 绘制偏航角曲线
        ax.plot(t_target, leader_yaw_interp, 'b-', linewidth=2.5, label='Leader', alpha=0.9)
        ax.plot(t_target, follower_yaw_interp, 'r-', linewidth=2, label='Follower', alpha=0.8)
        
        # 核心设置：固定纵轴范围 + 统一标注样式
        ax.set_ylim(YAW_Y_LIM)
        ax.set_xlabel('Time (s)', fontsize=16)
        ax.set_ylabel(f'Yaw Angle ({unit_str})', fontsize=16)
        ax.legend(loc='best', fontsize=16)
        ax.grid(True, alpha=0.3, linestyle='-')
        
        # 与轨迹图统一保存参数（dpi/bbox_inches）
        plt.tight_layout()
        plt.savefig(pic_path, dpi=DPI, bbox_inches='tight')
        print(f"✅ 偏航角图已保存至：{pic_path}")
        plt.close(fig)
        
        # 计算偏航角误差指标（插值后数据，和位置误差时间轴匹配）
        yaw_metrics = calculate_yaw_error(leader_yaw_interp, follower_yaw_interp)
        return yaw_metrics
    
    except Exception as e:
        print(f"❌ 绘制偏航角图失败 {filename}：{str(e)}")
        return None

def print_error_table(metrics_list):
    """打印位置+偏航角双误差指标表格，分轨迹类型展示"""
    # 整理数据：key=(算法, 轨迹)，value=(位置指标, 偏航角指标)
    error_data = {}
    for item in metrics_list:
        key = (item["algorithm"], item["trajectory"])
        error_data[key] = (item["pos_metrics"], item["yaw_metrics"])
    
    # 打印表格标题
    print("\n" + "="*100)
    print("📊 轨迹跟踪 + 偏航角跟踪 双误差指标定量分析结果")
    print(f"💡 位置单位：米 | 偏航角单位：{ANGLE_UNIT} | 指标：RMSE / MAE / Max Error")
    print("="*100)
    
    # 遍历轨迹类型（圆形/正弦）分别打印
    for traj in ["Circle", "Sine"]:
        print(f"\n【{traj}轨迹】")
        # 打印表头：算法 | 位置-RMSE | 位置-MAE | 位置-Max Error | 偏航角-RMSE | 偏航角-MAE | 偏航角-Max Error
        header = f"{'算法':<8}{'位置-RMSE':<12}{'位置-MAE':<12}{'位置-Max':<12}{'偏航角-RMSE':<15}{'偏航角-MAE':<15}{'偏航角-Max':<15}"
        print(header)
        print("-" * len(header.replace(' ', '')) * 2)  # 分隔线，整数倍数，解决浮点数报错
        
        # 打印PID和MPC的指标
        for algo in ["PID", "MPC"]:
            if (algo, traj) in error_data:
                pos_m, yaw_m = error_data[(algo, traj)]
                # 保留4位小数，对齐打印
                print(f"{algo:<8}{pos_m['RMSE']:<12.4f}{pos_m['MAE']:<12.4f}{pos_m['Max Error']:<12.4f}"
                      f"{yaw_m['RMSE']:<15.4f}{yaw_m['MAE']:<15.4f}{yaw_m['Max Error']:<15.4f}")
    
    # 性能总结（位置+偏航角的RMSE提升率）
    print("\n" + "="*80)
    print("📈 性能提升总结（MPC对比PID，RMSE降低率）")
    print("-"*40)
    for traj in ["Circle", "Sine"]:
        if ("PID", traj) in error_data and ("MPC", traj) in error_data:
            pid_pos, pid_yaw = error_data[("PID", traj)]
            mpc_pos, mpc_yaw = error_data[("MPC", traj)]
            # 计算RMSE降低率（(PID-MPC)/PID *100）
            pos_improve = (pid_pos['RMSE'] - mpc_pos['RMSE']) / pid_pos['RMSE'] * 100 if pid_pos['RMSE'] !=0 else 0
            yaw_improve = (pid_yaw['RMSE'] - mpc_yaw['RMSE']) / pid_yaw['RMSE'] * 100 if pid_yaw['RMSE'] !=0 else 0
            print(f"{traj}轨迹 - 位置RMSE：{pos_improve:.2f}% | 偏航角RMSE：{yaw_improve:.2f}%")
    print("="*80)

# ===================== 主程序 =====================
if __name__ == "__main__":
    print(f"📌 当前脚本所在目录：{BASE_DIR}")
    print(f"📌 数据目录路径：{DATA_DIR}")
    print(f"📌 图片保存目录：{PIC_DIR}")
    print(f"📌 绘图统一配置：尺寸{FIG_SIZE} | 清晰度{DPI}dpi | 偏航角纵轴{YAW_Y_LIM}")
    print(f"📌 误差计算配置：位置(米) | 偏航角({ANGLE_UNIT}) | 指标RMSE/MAE/Max Error\n")
    
    # 检查并创建目录
    if not os.path.exists(DATA_DIR):
        print(f"❌ 数据目录不存在：{DATA_DIR}")
        print(f"  请在 {BASE_DIR} 目录下创建data文件夹，并放入相关txt文件")
        exit(1)
    if not os.path.exists(PIC_DIR):
        os.makedirs(PIC_DIR)
        print(f"📁 已创建图片保存目录：{PIC_DIR}\n")
    
    # 绘制轨迹图+偏航角图，收集位置+偏航角双误差指标
    metrics_list = []
    for config in FILE_CONFIGS:
        print(f"\n{'='*60}")
        print(f"开始处理：{config['filename']}")
        print(f"{'='*60}")
        # 绘制轨迹图并获取位置误差
        leader_data, follower_data, pos_metrics = plot_single_trajectory(config)
        if leader_data is None:
            continue
        # 绘制偏航角图并获取偏航角误差
        yaw_metrics = plot_single_yaw(config, leader_data, follower_data)
        if yaw_metrics is None:
            continue
        # 收集双指标数据
        metrics_list.append({
            "algorithm": config["algorithm"],
            "trajectory": config["trajectory"],
            "pos_metrics": pos_metrics,
            "yaw_metrics": yaw_metrics
        })
    
    # 打印双误差指标表格
    if metrics_list:
        print_error_table(metrics_list)
    
    print("\n🎉 所有图片绘制完成 + 位置/偏航角双误差指标计算完成！")
    print(f"📁 图片保存路径：{PIC_DIR}（尺寸{FIG_SIZE} | 清晰度{DPI}dpi）")
    print(f"📊 误差指标已在终端打印，包含RMSE/MAE/Max Error及MPC性能提升率")