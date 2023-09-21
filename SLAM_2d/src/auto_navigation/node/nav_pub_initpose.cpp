/***********************nav_pub_initpose**********************************
*fun: 发布初始车的初始位置和姿态
*author: 郭嘉豪
*Time: 2023-3
*version: 1.0
*********************************************************/
#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/asio.hpp>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"//use data struct of std_msgs/String  
#include "std_msgs/Float32.h" 
#include "turtlesim/Pose.h"  
#include <vector>

int flag=0;
geometry_msgs::PoseWithCovarianceStamped pub_msg;


void odom_Callback(const nav_msgs::Odometry msg) {

    pub_msg.header = msg.header;
    pub_msg.header.frame_id = "map";
    pub_msg.pose = msg.pose;
    flag ++;
}

int main(int argc, char** argv)
{
    //初始化ROS节点
    ros::init(argc, argv, "pub_initpose");
    ros::NodeHandle nh;
    sleep(5);
    //创建一个订阅者订阅键盘控制节点
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odom_Callback);//subscribe("/cmd_vel", 10, speed_vel_Callback);  
    ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);	

    //循环运行
    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        ros::spinOnce();
        // 机器人控制
        if(flag){
            pub.publish(pub_msg);
            if(flag ==2 ) break;
        }
        loop_rate.sleep();
    }

    return 0;
}




