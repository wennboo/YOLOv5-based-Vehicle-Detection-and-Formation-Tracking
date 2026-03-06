#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/Path.h"
#include <cmath>
#include "geometry_msgs/PoseStamped.h"

// 轨迹类型选择枚举（新增直线轨迹）
enum TrajectoryType {
    CIRCLE_TRAJ,    // 圆形轨迹
    SIN_TRAJ,       // 正弦曲线轨迹
    LINE_TRAJ       // 直线轨迹
};

// 全局配置参数 - 确保圆经过(0,0)
#define CIRCLE_CENTER_X 2.0        // 圆心x坐标（(0,0)在圆上，半径2.0）
#define CIRCLE_CENTER_Y 0.0        // 圆心y坐标
#define CIRCLE_RADIUS 2.0          // 圆半径（圆心(2,0)，半径2，刚好经过(0,0)）
#define SIN_AMPLITUDE 1.0          // 正弦曲线振幅
#define SIN_FREQUENCY 1            // 正弦曲线频率
#define LINE_END_X 10.0            // 直线轨迹终点x坐标（从(0,0)到(10,0)）
#define LINE_FIXED_Y 0.0           // 直线轨迹固定y坐标（保证沿x轴直线）
#define LINE_THETA 0.0             // 直线轨迹固定朝向（沿x轴正方向）

// 发布路径点的函数
void pub_Path(nav_msgs::Path &path, double x, double y, double theta, ros::Publisher pub) {
    geometry_msgs::PoseStamped pose_stamped;
    
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;

    tf2::Quaternion qtn;
    qtn.setRPY(0, 0, theta); // 设置偏航角
    pose_stamped.pose.orientation.x = qtn.getX();
    pose_stamped.pose.orientation.y = qtn.getY();
    pose_stamped.pose.orientation.z = qtn.getZ();
    pose_stamped.pose.orientation.w = qtn.getW();
    
    path.poses.push_back(pose_stamped);
    pub.publish(path);
}

// 正弦曲线函数（从(0,0)开始）
double sin_curve(double x) {
    return SIN_AMPLITUDE * sin(SIN_FREQUENCY * x);
}

// 极坐标生成整圆的x坐标 - 从(0,0)开始
double circle_x(double angle) {
    // 圆心(2,0)，半径2，极角π时对应(0,0)，调整角度偏移让起始点为(0,0)
    return CIRCLE_CENTER_X + CIRCLE_RADIUS * cos(angle + M_PI);
}

// 极坐标生成整圆的y坐标 - 从(0,0)开始
double circle_y(double angle) {
    return CIRCLE_CENTER_Y + CIRCLE_RADIUS * sin(angle + M_PI);
}

// 计算轨迹点的切线角（朝向）
double get_theta(TrajectoryType type, double param) {
    if (type == CIRCLE_TRAJ) {
        // 圆形轨迹的切线角，适配(0,0)起始点的朝向
        return param + M_PI / 2.0;
    } else if (type == SIN_TRAJ) {
        // 正弦曲线的导数，确保(0,0)点的朝向正确
        double deriv = SIN_AMPLITUDE * SIN_FREQUENCY * cos(SIN_FREQUENCY * param);
        return atan(deriv);
    } else { // LINE_TRAJ
        // 直线轨迹固定朝向（沿x轴正方向）
        return LINE_THETA;
    }
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    
    ros::init(argc, argv, "pose_pub");
    ros::NodeHandle nh;
    ros::Rate rate(20);  // 20Hz发布频率，轨迹更平滑
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/pose_pub", 1000);

    // 初始化路径消息
    nav_msgs::Path path;
    path.header.seq = 1;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    // 选择要绘制的轨迹类型：CIRCLE_TRAJ / SIN_TRAJ / LINE_TRAJ
    // TrajectoryType traj_type = SIN_TRAJ;
    // TrajectoryType traj_type = CIRCLE_TRAJ;
    TrajectoryType traj_type = LINE_TRAJ;  // 切换为直线轨迹
    
    // 轨迹参数初始化（确保从(0,0)开始）
    double param = 0.0;       // 圆形：极角从0开始；正弦/直线：x从0开始
    double step = 0.01;       // 步进值

    // 强制发布第一个点(0,0)，确保起始位置准确
    pub_Path(path, 0.0, 0.0, get_theta(traj_type, param), pub);

    while (ros::ok()) {
        // 1. 计算当前轨迹点的x、y坐标
        double x, y;
        if (traj_type == CIRCLE_TRAJ) {
            x = circle_x(param);
            y = circle_y(param);
        } else if (traj_type == SIN_TRAJ) {
            x = param;
            y = sin_curve(param);
        } else { // LINE_TRAJ
            x = param;                // x轴匀速前进
            y = LINE_FIXED_Y;         // y轴固定，保证直线
        }

        // 2. 计算当前点的朝向角
        double theta = get_theta(traj_type, param);

        // 3. 发布TF变换
        static tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped tfs;
        
        tfs.header.frame_id = "world";
        tfs.header.stamp = ros::Time::now();
        tfs.child_frame_id = "pose_pub";

        // 设置平移量
        tfs.transform.translation.x = x;
        tfs.transform.translation.y = y;
        tfs.transform.translation.z = 0.0;

        // 设置四元数（姿态）
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, theta);
        tfs.transform.rotation.x = qtn.getX();
        tfs.transform.rotation.y = qtn.getY();
        tfs.transform.rotation.z = qtn.getZ();
        tfs.transform.rotation.w = qtn.getW();

        // 发布TF变换
        broadcaster.sendTransform(tfs);

        // 4. 发布路径消息（跳过重复的初始点）
        if (!(fabs(x) < 1e-6 && fabs(y) < 1e-6 && param < step)) {
            pub_Path(path, x, y, theta, pub);
        }

        // 5. 更新轨迹参数，实现循环
        param += step;
        if (traj_type == CIRCLE_TRAJ) {
            // 圆形轨迹：极角超过2π后重置，循环绘制整圆
            if (param > 2 * M_PI) {
                param = 0.0;
                path.poses.clear();  // 清空路径，重新从(0,0)开始
                pub_Path(path, 0.0, 0.0, get_theta(traj_type, param), pub);
            }
        } else if (traj_type == SIN_TRAJ) {
            // 正弦曲线：x坐标超过20后重置，从(0,0)重新开始
            if (param > 20.0) {
                param = 0.0;
                path.poses.clear();
                pub_Path(path, 0.0, 0.0, get_theta(traj_type, 0.0), pub);
            }
        } else { // LINE_TRAJ
            // 直线轨迹：x坐标超过终点后重置，从(0,0)重新开始
            if (param > LINE_END_X) {
                param = 0.0;
                path.poses.clear();
                pub_Path(path, 0.0, 0.0, get_theta(traj_type, 0.0), pub);
            }
        }

        rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}