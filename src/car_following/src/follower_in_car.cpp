#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "car_following/PID.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/Path.h"

using namespace std;

PID::PID(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), ep(0.0), ei(0.0), ed(0.0), dt(0.05) {}

void PID::update_e(double e) {
    ed = e - ep;
    ei += e;
    ep = e;
}

double PID::get_u() {
    double u = kp * ep + ki * ei + kd * ed;
    
    if (u > M_PI / 6) u = M_PI / 6;
    if (u < -M_PI / 6) u = -M_PI / 6;
    return u;
}

double PID::get_t(){
    return dt;
}

void pub_Path(nav_msgs::Path & path,double x,double y,double theta,ros::Publisher pub){
    geometry_msgs::PoseStamped pose_stamped;
    
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;

    tf2::Quaternion qtn;
    qtn.setRPY(0,0,theta); //slope
    pose_stamped.pose.orientation.x = qtn.getX();
    pose_stamped.pose.orientation.y = qtn.getY();
    pose_stamped.pose.orientation.z = qtn.getZ();
    pose_stamped.pose.orientation.w = qtn.getW();
    path.poses.push_back(pose_stamped);
    
    pub.publish(path);
}

class diff_pose{
    public:
    double e_x;
    double e_y;
    double e_theta;
    diff_pose(double e_x,double e_y,double e_theta){
        this->e_x = e_x;
        this->e_y = e_y;
        this->e_theta = e_theta;
    }
};

int main(int argc, char *argv[])
{   setlocale(LC_ALL,"");

    ros::init(argc,argv,"follower_in_car");

    ros::NodeHandle nh;

    //initialize path
    
    nav_msgs::Path path;
    path.header.seq = 1;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/follower/cmd_vel",1000);
    ros::Publisher pub_path = nh.advertise<nav_msgs::Path>("/follower_pose",1000);
    geometry_msgs::Twist twist;

    tf2_ros::Buffer buffer; 
    tf2_ros::TransformListener listener(buffer);
    
    //generate PID_controller for x,y,theta
    PID pid_x = PID(1,0.01,20.0);
    PID pid_y = PID(1,0.01,20.0); //useless
    PID pid_theta = PID(1,0.01,20.0);

    int hz = 20;
    ros::Rate rate(hz);


    while (ros::ok())
    {
        try{
        //   解析 son1 中的点相对于 son2 的坐标
            geometry_msgs::TransformStamped tfs = buffer.lookupTransform("follower_base_link","leader_past",ros::Time(0));
            double e_x = tfs.transform.translation.x;
            double e_y = tfs.transform.translation.y;
            tf::Quaternion quat1;
            tf::quaternionMsgToTF(tfs.transform.rotation,quat1);
            double e_roll, e_pitch, e_yaw;//定义存储r\p\y的容器
            tf::Matrix3x3(quat1).getRPY(e_roll, e_pitch, e_yaw);//进行转换
            ROS_INFO("e_x:%lf,e_y:%lf,e_yaw:%lf",e_x,e_y,e_yaw);

            pid_x.update_e(e_x);
            pid_y.update_e(e_y);
            pid_theta.update_e(e_yaw);
            double u_x = pid_x.get_u();
            double u_y = pid_y.get_u();
            double u_theta = pid_theta.get_u();
            ROS_INFO("u_x:%lf,u_y:%lf,u_yaw:%lf",u_x,u_y,u_theta);
            double dt = pid_x.get_t();
            twist.linear.x = u_x;
            twist.linear.y = u_y;
            //u_y is used for rectify the deviation in y axis.
            //while y_leader-y_follower>0, then u_theta will increase the value of theta.
            twist.angular.z = u_theta+u_y*5; 

            pub.publish(twist);

            geometry_msgs::TransformStamped follower_pos = buffer.lookupTransform("world","follower_base_link",ros::Time(0));
            double f_x = follower_pos.transform.translation.x;
            double f_y = follower_pos.transform.translation.y;
            tf::Quaternion quat2;
            tf::quaternionMsgToTF(follower_pos.transform.rotation,quat2);
            double f_roll, f_pitch, f_theta;//定义存储r\p\y的容器
            tf::Matrix3x3(quat2).getRPY(f_roll, f_pitch, f_theta);//进行转换

            pub_Path(path,f_x,f_y,f_theta,pub_path);
        }
        catch(const std::exception& e)
        {
            ROS_INFO("异常信息:%s",e.what());
        }
        rate.sleep();
        
    }
    
        
    return 0;
}
