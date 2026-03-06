/*
改脚本用于利用kalman输出话题，输出预测目标位姿，和过去位姿
*/

/*
    需求: 订阅人的信息

*/

#include "ros/ros.h"
#include "car_following/Predict.h"
#include "geometry_msgs/TransformStamped.h"
#include <queue>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>

using namespace std;

class kalman_past_pose{
    public:
    double f_x;
    double f_y;
    double f_theta;
    kalman_past_pose(double f_x,double f_y, double f_theta){
        this->f_x = f_x;
        this->f_y = f_y;
        this->f_theta = f_theta;
    }
};

queue<kalman_past_pose> q;



void doPose(const car_following::Predict::ConstPtr& predict_p){

    static tf2_ros::TransformBroadcaster broadcaster;
    double kalman_x = predict_p->x;
    double kalman_y = predict_p->y;
    double kalman_theta = predict_p->cita;
    
    geometry_msgs::TransformStamped tfs;

    ROS_INFO("abs(kalman_x-q.back().f_x):%lf",abs(kalman_x-q.back().f_x));
    //如果和序列最后一个元素相差过大，则不进行后续工作
    if(abs(kalman_x-q.back().f_x)<0.1||
        abs(kalman_y-q.back().f_y)<0.05){
        tfs.header.frame_id = "world";
        tfs.header.stamp = ros::Time::now();
        tfs.child_frame_id = "kalman_predict";

        tfs.transform.translation.x = kalman_x;

        tfs.transform.translation.y = kalman_y;
        tfs.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
        //  |--------- 四元数设置
        tf2::Quaternion qtn;
        double theta = kalman_theta;
        qtn.setRPY(0,0,theta); //slope
        tfs.transform.rotation.x = qtn.getX();
        tfs.transform.rotation.y = qtn.getY();
        tfs.transform.rotation.z = qtn.getZ();
        tfs.transform.rotation.w = qtn.getW();

        //  5-3.广播器发布数据
        broadcaster.sendTransform(tfs);
        
    
        kalman_past_pose instance(kalman_x,kalman_y,kalman_theta);
        q.push(instance);    
        q.pop();
        
        kalman_past_pose past_pose = q.front();
        
        geometry_msgs::TransformStamped tfs_;
            
        tfs_.header.frame_id = "world";
        tfs_.header.stamp = ros::Time::now();
        tfs_.child_frame_id = "kalman_predict_past";

        tfs_.transform.translation.x = past_pose.f_x;

        tfs_.transform.translation.y = past_pose.f_y;
        tfs_.transform.translation.z = 0.0; // 二维实现，pose 中没有z，z 是 0
        //  |--------- 四元数设置
        tf2::Quaternion qtn_;
        double theta_ = past_pose.f_theta;
        qtn_.setRPY(0,0,theta_); //slope
        tfs_.transform.rotation.x = qtn_.getX();
        tfs_.transform.rotation.y = qtn_.getY();
        tfs_.transform.rotation.z = qtn_.getZ();
        tfs_.transform.rotation.w = qtn_.getW();

        //  5-3.广播器发布数据
        broadcaster.sendTransform(tfs_);

        // 打印预测数据
        ROS_INFO("x:%lf, y:%lf, theta:%lf",past_pose.f_x,past_pose.f_y,past_pose.f_theta);
    }
    
}

int main(int argc, char *argv[])
{   
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"predict_leader_pub");

    
    int q_len = 1200;
    for(int i=0;i<q_len;i++){
        kalman_past_pose instance(0,0,0);
        q.push(instance);
    }

    //2.创建 ROS 句柄
    ros::NodeHandle nh;
    //3.创建订阅对象
    ros::Subscriber sub = nh.subscribe<car_following::Predict>("/kalman_filt",10,doPose);
    
    ros::spin();    
   
   return 0;
}


