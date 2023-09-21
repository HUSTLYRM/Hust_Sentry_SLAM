/********************rviz_goal************************
 *fun: 调用rviz_goal，取得目标点位置
 *author: Hahalim
 *Time: 2023-7
 *version: 1.0
 *********************************************************/

#include <ros/ros.h>
#include "SerialPort.h"
#include <ros/time.h>
#include <boost/asio.hpp>
#include "tf/transform_datatypes.h" 
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h" 
#include "std_msgs/Float32.h"
#include <sensor_msgs/Imu.h>
#include <vector>
#include "MYAPI.h"
#include <string>
#include <serial/serial.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>
#include <rm_bringup/Gimbal_Control.h>
#include <rm_bringup/F405Data.h>
#include <rm_bringup/Chassis_Control.h>
#include "robot_start.h"
#include <math.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace ly;

 
void rviz_goal_callback(geometry_msgs::PoseStamped goal){
  ROS_DEBUG("\n\n\n rviz_goal: goal_x=%f, goal_y=%f\n\n", goal.pose.position.x, goal.pose.position.y);
}



int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "move_base_node");
    ros::NodeHandle nh("~");

    bool debug_flag;

    nh.param<bool>("debug_flag", debug_flag, "false");
    string usart_port = getParam("usart_port", ((string) "/dev/lidar"));
    

    if (debug_flag)
    {
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        ROS_INFO_STREAM("DEBUG has Open\n");
    }
    else
    {
        ROS_INFO_STREAM("DEBUG has Closed\n");
    }

    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, rviz_goal_callback);



    // 循环运行
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
    
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
