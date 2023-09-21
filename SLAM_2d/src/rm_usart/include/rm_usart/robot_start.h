#ifndef ROBOT_START
#define ROBOT_START

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"//use data struct of std_msgs/String  
#include "std_msgs/Float32.h" 
#include "turtlesim/Pose.h"  
#include <vector>
#include <rm_bringup/F405Data.h>
#include <rm_bringup/Chassis_Control.h>

#define speed_limit 35 //cm/s
#define angle_speed_limit 6 //cm/s
#define angle_change_rate 88 //角度从底盘以degree单位数值*88发送上来
#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))


namespace robot
{
    class robot
    {
        public:
            bool init( );                  
            // bool task_start( double v_x,double v_y,double v_z,double yaw_now);
            bool task_start(double pose_x,double pose_y,double pose_w,double pose_vx,double pose_vy,double pose_vw);
            double wheel_distance;        //机器人底盘宽度
            double speed_radio;           //速度标定系数
            double sampling_time;         //速度采集频率
        
        private:
            void cal_Odom();               //里程计计算
            void pub_OdomAndTf( );           //发布Odom和tf
        
        public:
            ros::Time current_time, last_time; //时间
            double x_last;
            double y_last;
            double w_last;

            double x;
            double y;
            double w;
            
            double vx;
            double vy;
            double vw;
            ros::NodeHandle n;
            ros::Publisher odom_pub;
            tf::TransformBroadcaster odom_broadcaster;
    };
    
}

#endif