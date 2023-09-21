/***********************rm_usart*****************************
 *fun: 获取云台板发送上来的串口数据并发布话题
 *author: Hahalim
 *Time: 2023-7
 *version: 2.0
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
SerialPortWriteData data_write;
SerialPortData data_read;
rm_bringup::F405Data F405Data_mag;
move_base_msgs::MoveBaseGoal goal;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;
 

/********************************************************
函数功能：订阅/cmd_vel发送速度信息，控制底盘运动
入口参数：
出口参数：
********************************************************/
// 让车沿着激光雷达的方向走
// void ChassisCallback(const geometry_msgs::Twist msg)
// {
//     data_write.x_set = -msg.linear.x * 1000;
//     data_write.y_set = -msg.linear.y * 1000;          //?
//     data_write.w_set = msg.angular.z * 1000;
//     //ROS_DEBUG("Received Chassis_Control Message: x_set=%d, w_set=%f", data_write.x_set, data_write.w_set);
// }

void ChassisCallback(const geometry_msgs::Twist msg)
{
    data_write.x_set = msg.linear.x * 1000;
    data_write.y_set = msg.linear.y * 1000; //?
    data_write.w_set = msg.angular.z * 1000;
    // data_write.w_set = msg.angular.z * 1000 +1000;
    ROS_DEBUG("Write: x_set=%f, y_set=%f, w_set=%f", data_write.x_set, data_write.y_set, data_write.w_set);
}
// void ChassisCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
// {
//     data_write.x_set = msg->twist.linear.x * 1000;
//     data_write.y_set = msg->twist.linear.y * 1000;         //?
//     data_write.w_set = msg->twist.angular.z * 1000;
//     // ROS_INFO("Write: x_set=%f, w_set=%f", data_write.x_set, data_write.w_set);
// }

void PLIO_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    F405Data_mag.x_now = msg->pose.pose.position.x;
    F405Data_mag.y_now = msg->pose.pose.position.y;

    float yaww = msg->pose.pose.orientation.w;
    float yawx = msg->pose.pose.orientation.x;
    float yawy = msg->pose.pose.orientation.y;
    float yawz = msg->pose.pose.orientation.z;
    F405Data_mag.w_now = atan2(2 * (yaww * yawz + yawx * yawy), 1 - 2 * (yawy * yawy + yawz * yawz));
    ROS_INFO("Subcribe: x=%f,y=%f,w=%f", F405Data_mag.x_now, F405Data_mag.y_now, F405Data_mag.w_now); // 提示消息
}
void DLO_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    F405Data_mag.x_now = msg->pose.pose.position.x;
    F405Data_mag.y_now = msg->pose.pose.position.y;
    F405Data_mag.vx = msg->twist.twist.linear.x;
    F405Data_mag.vy = msg->twist.twist.linear.y;
    F405Data_mag.vw = msg->twist.twist.angular.z;

    float yaww = msg->pose.pose.orientation.w;
    float yawx = msg->pose.pose.orientation.x;
    float yawy = msg->pose.pose.orientation.y;
    float yawz = msg->pose.pose.orientation.z;

    F405Data_mag.w_now = atan2(2 * (yaww * yawz + yawx * yawy), 1 - 2 * (yawy * yawy + yawz * yawz));
    ROS_INFO("Subcribe: x=%f,y=%f,w=%f", F405Data_mag.x_now, F405Data_mag.y_now, F405Data_mag.w_now); // 提示消息
}
// void DLO_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) // /robot/dlo/odom_node/pose
// {
//     F405Data_mag.x_now = msg->pose.position.x;
//     F405Data_mag.y_now = msg->pose.position.y;

//     float yaww = msg->pose.orientation.w;
//     float yawx = msg->pose.orientation.x;
//     float yawy = msg->pose.orientation.y;
//     float yawz = msg->pose.orientation.z;
//     F405Data_mag.w_now = atan2( 2*(yaww*yawz + yawx*yawy ) , 1 - 2*(yawy*yawy +yawz*yawz) );
//     // F405Data_mag.w_now = F405data_serial.w_now*4;    //?
//     ROS_INFO("Subcribe: x=%f,y=%f,w=%f", F405Data_mag.x_now,F405Data_mag.y_now,F405Data_mag.w_now);  //提示消息
// }

void status_callback(const move_base_msgs::MoveBaseActionResult& msg)
{	
	if(msg.status.status == 3)
	{
		  std::cout<<"the goal was achieved successfully!"<<std::endl;
      std::cout<<"data_write.flag='1'"<<std::endl;
	    data_write.flag=1;
    }
    else{
      data_write.flag=0;
    }
}



int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "robotStart");
    ros::NodeHandle nh("~");

    bool debug_flag;

    nh.param<bool>("debug_flag", debug_flag, "false");
    string Chassis_topic = getParam("Chassis_Control_topic", ((string) "/cmd_vel"));
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

    robot::robot myrobot;

    if (!myrobot.init())
        ROS_ERROR("myrobot start failed.");
    ROS_INFO("myrobot start successful");

    // 初始化串口
    SerialPort *serial_port = new SerialPort(usart_port); // 如果不填写目标USB，即可搜索所有USB并开启   
    data_write.head = '!';
    data_write.x_set = 0; // mm/s
    data_write.y_set = 0; // mm/s
    data_write.w_set = 0; // rad/s
    

    ros::Subscriber sub_cmd = nh.subscribe(Chassis_topic, 1, ChassisCallback);
    ros::Subscriber sub_dlo = nh.subscribe("/robot/dlo/odom_node/odom", 10, DLO_Callback);
    // ros::Subscriber sub_dlo = nh.subscribe("/robot/dlo/odom_node/pose", 10, DLO_Callback);
    // ros::Subscriber sub_dlo = nh.subscribe("/aft_mapped_to_init", 10, PLIO_Callback);
    //ros::Subscriber goal_sub = nh.subscribe("/move_base/result", 10, status_callback);


    // 循环运行
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        // ROS_INFO("Subcribe: x=%f,y=%f,w=%f", F405Data_mag.x_now,F405Data_mag.y_now,F405Data_mag.w_now);  //提示消息
        // ROS_INFO("Write: vx=%f,vy=%f,vw=%f", data_write.x_set,data_write.y_set,data_write.w_set);  //提示消息
        try
        {
            serial_port->writeData(&data_write);
            serial_port->readData(&data_read);
            char flag_goal = data_read.goal;


            myrobot.task_start(F405Data_mag.x_now, F405Data_mag.y_now, F405Data_mag.w_now,
                                F405Data_mag.vx,F405Data_mag.vy,F405Data_mag.vw);
                                
        }
        catch (exception e)
        {
            ROS_ERROR_STREAM("Some error in pub_node file ");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
