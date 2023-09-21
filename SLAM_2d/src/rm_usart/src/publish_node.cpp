/***********************rm_usart**********************************
 *fun: 获取云台版发送上来的串口数据并发布话题
 *author: 郭嘉豪
 *Time: 2022-9
 *version: 1.0
 *********************************************************/

#include <ros/ros.h>
#include "SerialPort.h"
#include <ros/time.h>
#include <boost/asio.hpp>
#include "tf/transform_datatypes.h" //转换函数头文件
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "std_msgs/String.h" //use data struct of std_msgs/String
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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
using namespace ly;
SerialPortWriteData data_write;
void gimbalCallback(const rm_bringup::Gimbal_Control::ConstPtr& msg)
{
   data_write.RCPitch = msg->pitch_set;
   data_write.RCYaw = msg->yaw_set;
   data_write.FrictionWheel_speed = msg->flag; //[9][10]
   ROS_DEBUG("Received Gimbal Control Message: Pitch=%d, Yaw=%d, Flag=%d", msg->pitch_set, msg->yaw_set, msg->flag);
  // 在终端中打印接收到的消息
}

/********************************************************
函数功能：订阅/cmd_vel控制 计算两个轮子的分别速度，计算控制位
入口参数：
出口参数：
********************************************************/  
void ChassisCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)  
{  
    data_write.x_set = msg->twist.linear.x * 1000;
    data_write.y_set = msg->twist.linear.y * 1000;
    data_write.w_set = msg->twist.angular.x;
    ROS_DEBUG("Received Chassis_Control Message: x_set=%d, w_set=%f", data_write.x_set, data_write.w_set);
}  

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "publish_usart");
    ros::NodeHandle nh("~");
    bool debug_flag;

    nh.param<bool>("debug_flag", debug_flag, "false");
    string Gimbal_topic = getParam("Gimbal_Control_topic", ((string) "/Gimbal_Control"));
    string Chassis_topic = getParam("Chassis_Control_topic", ((string) "/cmd_vel"));
    string usart_port = getParam("usart_port", ((string) "/dev/ttyUSB2"));

    if (debug_flag)
    {
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        ROS_INFO_STREAM("DEBUG has Open\n");
    }
    else
    {
        ROS_INFO_STREAM("DEBUG has Closed\n");
    }

    ros::Subscriber sub = nh.subscribe(Gimbal_topic, 1, gimbalCallback); 
    ros::Subscriber sub1 = nh.subscribe(Chassis_topic, 1, ChassisCallback); 
    // 发布imu数据
    ros::Publisher F405data_pub = nh.advertise<rm_bringup::F405Data>("/F405Data", 1);
    rm_bringup::F405Data F405Data_mag;
    // 初始化串口
    SerialPort *serial_port = new SerialPort(usart_port); // 如果不填写目标USB，即可搜索所有USB并开启
    SerialPortData F405data_serial;
    
    data_write.head = 0xaa;
    // data_write.tail = 0xbb;
    data_write.x_set = 0;//mm/s
    data_write.w_set = 0;//rad/s
    // data_write.FrictionWheel_speed = 1;
    int last_yaw = 0;
    int i = 0;
    // 循环运行
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        try
        {
            serial_port->writeData(&data_write);
            serial_port->readData(&F405data_serial);
            F405Data_mag.x_now = F405data_serial.x_now/1000;
            F405Data_mag.y_now = F405data_serial.y_now;
            F405Data_mag.w_now = F405data_serial.w_now;
            F405data_pub.publish(F405Data_mag);
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


