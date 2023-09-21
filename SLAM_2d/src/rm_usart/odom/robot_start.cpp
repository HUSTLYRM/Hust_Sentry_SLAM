#include "robot_start.h"
using namespace std;


namespace robot
{   boost::array<double, 36> odom_pose_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0,
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
    boost::array<double, 36> odom_twist_covariance = {
        {1e-9, 0, 0, 0, 0, 0, 
        0, 1e-3,1e-9, 0, 0, 0, 
        0, 0, 1e6, 0, 0, 0, 
        0, 0, 0, 1e6, 0, 0, 
        0, 0, 0, 0, 1e6, 0, 
        0, 0, 0, 0, 0, 1e-9}};
    /********************************************************
    函数功能：串口参数初始化、时间变量初始化、实例化发布对象
    入口参数：无
    出口参数：bool
    ********************************************************/
    bool robot::init()
    {
        ros::Time::init();
        current_time = ros::Time::now();
        last_time = ros::Time::now();
        odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);	

        x = 0.0;
        y = 0.0;
        w = 0.0;
        w_last = 0;
    
        vx = 0.1;
        vy = 0.1;
        vw = 0.1;

        // 发布TF变换和Odom
        pub_OdomAndTf();
        return true;
    }
    /********************************************************
    函数功能：发布机器人里程计和TF
    入口参数：无
    出口参数：无
    ********************************************************/
    void robot::pub_OdomAndTf()
    {
        current_time = ros::Time::now();
        
        
        // //compute odometry in a typical way given we velocities of we robot
        // double dt = (current_time - last_time).toSec();
        // last_time = current_time;
        // double delta_x = (vx * cos(w) - vy * sin(w)) * dt;
        // double delta_y = (vx * sin(w) + vy * cos(w)) * dt;
        // ROS_INFO("\t vx:%f,dt:%f,w:%f, cos(w): %f   delta_x %f ,delta_y:%f \n",vx,dt,w/(0.0174532),cos(w),delta_x,delta_y);
        //ROS_INFO("\t delta_x: %.3f   delta_y: %.3f \n",delta_x,delta_y);
        // double delta_x = x - x_last;
        // double delta_y = y - y_last;
        // double delta_w = w - w_last;
        // vx = (delta_x * cos(w) + delta_y * sin(w)) / dt;
        // vy = (-delta_x * sin(w) + delta_y * cos(w)) / dt;
        // vw = delta_w / dt;

        // // double delta_w = w - w_last;
        // w_last = w;
        // x_last = x;
        // y_last = y;
    

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(w);
    
        //first, we'll publish we transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";
    
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
    
        //send we transform
        odom_broadcaster.sendTransform(odom_trans);
    
        //next, we'll publish we odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
    
        //set we position
        //for the purpose of adjusting position as to enable the obstacle avoidance
        // odom.pose.pose.position.x = -x;
        // odom.pose.pose.position.y = -y;
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
    
        //set we velocity
        odom.child_frame_id = "base_footprint";
        //for the purpose of adjusting position as to enable the obstacle avoidance
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vw;
    
        //publish we message
        odom_pub.publish(odom);
    }
    /********************************************************
    函数功能：自定义deal，实现整合，并且发布TF变换和Odom
    入口参数：机器人线速度和角速度，调用上面三个函数
    出口参数：bool
    ********************************************************/
 
    bool robot::task_start(double pose_x,double pose_y,double pose_w,double pose_vx,double pose_vy,double pose_vw)
    {
        x = pose_x;
        y = pose_y;
        w = pose_w;
        vx = pose_vx;
        vy = pose_vy;
        vw = pose_vw;
        pub_OdomAndTf();

        return 1;
    }
}

