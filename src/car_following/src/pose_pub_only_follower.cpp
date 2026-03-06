#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>          
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>

// 领航者位姿结构体
struct LeaderPose {
    double x;
    double y;
    double yaw;
    LeaderPose() : x(0.0), y(0.0), yaw(0.0) {}
    LeaderPose(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
};

class LeaderPosePublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher leader_odom_pub_;    
    ros::Publisher leader_path_pub_;    
    ros::Publisher leader_pose_stamped_pub_;  
    ros::Timer waypoint_timer_;         // 0.5秒切换一个航点的定时器
    ros::Timer tf_publish_timer_;       // tf（坐标轴）独立低频发布定时器
    tf::TransformBroadcaster tf_broadcaster_;
    
    // 配置参数
    std::string csv_file_path_;       
    double tf_publish_rate_;           // tf发布频率（默认5Hz，可视化足够）
    double leader_linear_vel_;        
    const double WAYPOINT_INTERVAL = 0.8; // 航点更新间隔，固定0.5秒
    bool is_pose_finished_;            // 标记是否到达最后一个航点
    
    // 位姿数据
    std::vector<LeaderPose> leader_poses_;  
    int current_pose_idx_;                  // 当前航点索引
    int total_poses_;                       
    nav_msgs::Path leader_full_path_;       
    LeaderPose current_tf_pose_;        // 缓存当前位姿给tf发布
    
    // 线程锁
    std::mutex data_mutex_;

public:
    LeaderPosePublisher() : current_pose_idx_(0), total_poses_(0), is_pose_finished_(false) {
        // 1. 读取ROS参数
        nh_.param<std::string>("csv_file_path", csv_file_path_,
            "/home/thr/pose_estimation_follower/src/car_following/scripts/data/ch5/data/fused_xy_data_sin.csv");
        nh_.param<double>("tf_publish_rate", tf_publish_rate_, 5.0);       // tf发布频率，可自定义
        nh_.param<double>("leader_linear_vel", leader_linear_vel_, 0.5);   // 领航者线速度（物理意义）
        
        // 2. 初始化发布器
        leader_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/leader_odom", 10);
        leader_path_pub_ = nh_.advertise<nav_msgs::Path>("/leader_pose", 10);
        leader_pose_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/leader_pose_stamped", 10);
        
        // 3. 加载CSV航点并初始化完整轨迹
        loadLeaderPosesFromCSV();
        initFullLeaderPath();  
        
        // 4. 初始化定时器（核心：0.5秒切换航点）
        if (total_poses_ > 0) {
            // 航点定时器：固定0.5秒触发一次，切换下一个航点
            waypoint_timer_ = nh_.createTimer(ros::Duration(WAYPOINT_INTERVAL),
                &LeaderPosePublisher::publishCurrentWaypoint, this);
            // tf定时器：低频发布坐标轴，降低资源消耗
            tf_publish_timer_ = nh_.createTimer(ros::Duration(1.0/tf_publish_rate_),
                &LeaderPosePublisher::publishTFTransform, this);
            
            // 初始化缓存位姿为第一个航点
            current_tf_pose_ = leader_poses_[0];

            ROS_INFO("Leader Pose Publisher Initialized:");
            ROS_INFO("  - CSV File: %s", csv_file_path_.c_str());
            ROS_INFO("  - Total Waypoints: %d", total_poses_);
            ROS_INFO("  - Waypoint Update Interval: %.1f s", WAYPOINT_INTERVAL);
            ROS_INFO("  - TF (Axes) Publish Rate: %.1f Hz", tf_publish_rate_);
            ROS_INFO("  - Published Topics: /leader_odom /leader_pose /leader_pose_stamped /tf");
        } else {
            ROS_ERROR("No valid leader waypoints loaded! Node will exit.");
            ros::shutdown();
        }
    }

    // 加载CSV航点并计算偏航角
    void loadLeaderPosesFromCSV() {
        std::ifstream csv_file(csv_file_path_);
        if (!csv_file.is_open()) {
            ROS_ERROR("Failed to open CSV file: %s", csv_file_path_.c_str());
            return;
        }
        
        std::string line;
        std::vector<double> x_vals, y_vals;
        bool is_header = true;
        
        while (std::getline(csv_file, line)) {
            if (is_header) {
                is_header = false;
                continue;
            }
            
            std::stringstream ss(line);
            std::string token;
            std::vector<std::string> columns;
            
            while (std::getline(ss, token, ',')) {
                columns.push_back(token);
            }
            
            if (columns.size() >= 3) {
                try {
                    double x = std::stod(columns[1]);
                    double y = std::stod(columns[2]);
                    x_vals.push_back(x);
                    y_vals.push_back(y);
                } catch (...) {
                    ROS_WARN("Invalid data line: %s (skip)", line.c_str());
                    continue;
                }
            }
        }
        csv_file.close();
        
        // 计算每个航点的偏航角（沿轨迹前进方向）
        total_poses_ = x_vals.size();
        leader_poses_.resize(total_poses_);
        
        for (int i = 0; i < total_poses_; ++i) {
            double dx, dy;
            // 第一个航点用下一个航点计算方向，后续用当前-前一个
            if (i == 0) {
                dx = (total_poses_ > 1) ? (x_vals[1] - x_vals[0]) : 1.0;
                dy = (total_poses_ > 1) ? (y_vals[1] - y_vals[0]) : 0.0;
            } else {
                dx = x_vals[i] - x_vals[i-1];
                dy = y_vals[i] - y_vals[i-1];
            }
            double yaw = atan2(dy, dx);
            leader_poses_[i] = LeaderPose(x_vals[i], y_vals[i], yaw);
        }
        ROS_INFO("Successfully loaded %d leader waypoints from CSV", total_poses_);
    }

    // 预生成完整的领航者轨迹（nav_msgs/Path），RVIZ显示全局轨迹
    void initFullLeaderPath() {
        leader_full_path_.header.frame_id = "world";
        leader_full_path_.poses.resize(total_poses_);
        
        for (int i = 0; i < total_poses_; ++i) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "world";
            pose_stamped.header.seq = i;
            pose_stamped.pose.position.x = leader_poses_[i].x;
            pose_stamped.pose.position.y = leader_poses_[i].y;
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation = yawToQuaternion(leader_poses_[i].yaw);
            leader_full_path_.poses[i] = pose_stamped;
        }
    }

    // 欧拉角（偏航）转四元数，适配ROS位姿格式
    geometry_msgs::Quaternion yawToQuaternion(double yaw) {
        tf::Quaternion tf_quat;
        tf_quat.setRPY(0.0, 0.0, yaw); // 仅偏航角，滚转/俯仰为0
        geometry_msgs::Quaternion ros_quat;
        tf::quaternionTFToMsg(tf_quat, ros_quat);
        return ros_quat;
    }

    // 核心：0.5秒触发一次，发布当前航点+更新索引（到终点则停止切换）
    void publishCurrentWaypoint(const ros::TimerEvent& event) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ros::Time current_time = ros::Time::now();

        // 到达终点：持续发布最后一个航点，速度/角速度置0，不再切换索引
        if (is_pose_finished_) {
            publishFinalWaypoint(current_time);
            return;
        }

        // 发布当前航点的所有位姿数据
        LeaderPose curr_pose = leader_poses_[current_pose_idx_];
        publishSingleWaypoint(curr_pose, current_time, false);

        // 更新缓存位姿（供tf低频发布）
        current_tf_pose_ = curr_pose;

        // 检查是否到达最后一个航点
        if (current_pose_idx_ >= total_poses_ - 1) {
            is_pose_finished_ = true;
            ROS_INFO("===== Leader reach final waypoint (idx=%d)! Keep static at end =====", current_pose_idx_);
        } else {
            current_pose_idx_++; // 未到终点，切换下一个航点
            ROS_DEBUG("Switch to next waypoint: idx=%d, (x=%.2f, y=%.2f)", current_pose_idx_, curr_pose.x, curr_pose.y);
        }
    }

    // 发布单个航点的Odometry/PoseStamped/Path
    void publishSingleWaypoint(LeaderPose& pose, ros::Time& time, bool is_final) {
        // 1. 发布/leader_odom（给PID控制器）
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = time;
        odom_msg.header.frame_id = "world";
        odom_msg.child_frame_id = "leader_link";
        odom_msg.pose.pose.position.x = pose.x;
        odom_msg.pose.pose.position.y = pose.y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = yawToQuaternion(pose.yaw);
        // 终点速度/角速度置0，非终点保留设定线速度
        odom_msg.twist.twist.linear.x = is_final ? 0.0 : leader_linear_vel_;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.z = is_final ? 0.0 : 0.0; // 航点间匀速，角速度置0
        leader_odom_pub_.publish(odom_msg);

        // 2. 发布/leader_pose_stamped（实时单点位姿）
        geometry_msgs::PoseStamped pose_stamped_msg;
        pose_stamped_msg.header.stamp = time;
        pose_stamped_msg.header.frame_id = "world";
        pose_stamped_msg.pose.position.x = pose.x;
        pose_stamped_msg.pose.position.y = pose.y;
        pose_stamped_msg.pose.position.z = 0.0;
        pose_stamped_msg.pose.orientation = yawToQuaternion(pose.yaw);
        leader_pose_stamped_pub_.publish(pose_stamped_msg);

        // 3. 发布完整轨迹（Path），仅更新时间戳，全局轨迹不变
        nav_msgs::Path path_msg = leader_full_path_;
        path_msg.header.stamp = time;
        leader_path_pub_.publish(path_msg);
    }

    // 发布最后一个航点（终点静止）
    void publishFinalWaypoint(ros::Time& time) {
        LeaderPose final_pose = leader_poses_[total_poses_ - 1];
        publishSingleWaypoint(final_pose, time, true); // is_final=true，速度置0
    }

    // 独立低频发布tf变换（leader_odom -> world），RVIZ显示坐标轴
    void publishTFTransform(const ros::TimerEvent& event) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        ros::Time current_time = ros::Time::now();

        // 构造tf变换，使用缓存的当前航点位姿
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(current_tf_pose_.x, current_tf_pose_.y, 0.0));
        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, current_tf_pose_.yaw);
        transform.setRotation(quat);

        // 发布tf：父坐标系world，子坐标系leader_odom
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, current_time, "world", "leader_odom"));
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "leader_pose_publisher");
    try {
        LeaderPosePublisher publisher;
        ros::spin();
    } catch (std::exception& e) {
        ROS_ERROR("Leader Pose Publisher Error: %s", e.what());
        return 1;
    }
    ROS_INFO("Leader Pose Publisher Shutdown Normally");
    return 0;
}