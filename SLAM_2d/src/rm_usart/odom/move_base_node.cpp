/********************move_base_node************************
 *fun: 调用move_base action，发送目标点
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
SerialPortWriteData data_write;
SerialPortData data_read;
rm_bringup::F405Data F405Data_mag;
move_base_msgs::MoveBaseGoal goal;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>  MoveBaseClient;
 
 
 
void rviz_goal_callback(geometry_msgs::PoseStamped goal){
  ROS_INFO("\n\n\n rviz_goal: goal_x=%f, goal_y=%f\n\n", goal.pose.position.x, goal.pose.position.y);
}


void DLO_Callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    F405Data_mag.x_now = msg->pose.pose.position.x;
    F405Data_mag.y_now = msg->pose.pose.position.y;
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

    //ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 10, rviz_goal_callback);
    ros::Subscriber sub_dlo = nh.subscribe("/robot/dlo/odom_node/odom", 10, DLO_Callback);
    MoveBaseClient ac("move_base", true); 
	  //ac.waitForServer(ros::Duration(60));
  	ROS_INFO("Connected to move base server");
    
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();



    // 初始化串口
    SerialPort *serial_ports = new SerialPort(usart_port); // 如果不填写目标USB，即可搜索所有USB并开启
    ROS_INFO("serial_port ok!");
    data_read.goal=0;
    data_write.head = '!';
 

    bool GOAL_ENABLE = 1;
    char flag_goal = 0;
    F405Data_mag.goal_x = 0;
    F405Data_mag.goal_y = 0;
    F405Data_mag.goal_w = 1;
    
    // 循环运行
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
       try
        {
            serial_ports->readData(&data_read);
            serial_ports->writeData(&data_write);

            //get_usart_goal();
            //ac.sendGoal(goal); //呼叫ac.sendGoal实际上会将目标推送到move_base节点进行处理
            //ac.waitForResult();
            flag_goal = data_read.goal;
            ROS_INFO("flag_goal=%d",flag_goal);
            //ROS_INFO("flag_goal_old=%d\n",flag_goal_old);
            
            
            if(GOAL_ENABLE){
                //去高地(3.39,2.69)
                if(flag_goal==1){
               //if(1){
        
                //goal.target_pose.pose.position.x = 3.5;
                //goal.target_pose.pose.position.y = 2.78;
                F405Data_mag.goal_x = 5.5;
                F405Data_mag.goal_y = 8.85;
                goal.target_pose.pose.position.x = F405Data_mag.goal_x;
                goal.target_pose.pose.position.y = F405Data_mag.goal_y;
                goal.target_pose.pose.orientation.w = -1;
                
                //ROS_INFO("flag_goal=%d",flag_goal);
                ROS_INFO("GOAL1: goal_x=%f, goal_y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
                
                if((fabs(F405Data_mag.x_now-F405Data_mag.goal_x)<0.15) && (fabs(F405Data_mag.y_now-F405Data_mag.goal_y)<0.15) && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
                   data_write.flag=1;
                   ROS_INFO("reach goal successful!");
                   ac.cancelGoal();
                }
                else{
                   data_write.flag=0;
                   //flag_goal_old=1;
                   ac.sendGoal(goal);
                   
                  //等10s到达那里
                  bool finished_within_time = ac.waitForResult(ros::Duration(10));
                  //如果我们没有及时赶到那里，就会中止目标
                  if(!finished_within_time)
                  {
                      ac.cancelGoal();
                      ROS_INFO("Timed out achieving goal");
                  }
                }
      
                //ac.waitForResult(); 
                  
            }
            //去资源岛(8.9,7.1) (8.77,6.4) (8.65,7.1)
            else if(flag_goal==2){
                //goal.target_pose.pose.position.x = 8.34;
                //goal.target_pose.pose.position.y = 6.7;
                F405Data_mag.goal_x = 8.34;
                F405Data_mag.goal_y = 6.7;
                goal.target_pose.pose.position.x = F405Data_mag.goal_x;
                goal.target_pose.pose.position.y = F405Data_mag.goal_y;
                goal.target_pose.pose.orientation.w = 1;
                //ac.sendGoal(goal);
                ROS_INFO("GOAL2: goal_x=%f, goal_y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y); 
                //ac.waitForResult();

                if((fabs(F405Data_mag.x_now-F405Data_mag.goal_x)<0.15) && (fabs(F405Data_mag.y_now-F405Data_mag.goal_y)<0.15) && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
                   data_write.flag=2;
                   ROS_INFO("reach goal successful!");
                   ac.cancelGoal();
                }
                else{
                   data_write.flag=0;
                   ac.sendGoal(goal);
                   
                  //等1分钟到达那里
                  bool finished_within_time = ac.waitForResult(ros::Duration(10));
                  //如果我们没有及时赶到那里，就会中止目标
                  if(!finished_within_time)
                  {
                      ac.cancelGoal();
                      ROS_INFO("Timed out achieving goal");
                  }
                }             
            }   
            //去巡逻区(-0.6,2) (-0.9,1.89)
            else if(flag_goal==3){
                //goal.target_pose.pose.position.x = -0.43;
                //goal.target_pose.pose.position.y = 2.42;
                F405Data_mag.goal_x = -0.43;
                F405Data_mag.goal_y = 2.42;
                goal.target_pose.pose.position.x = F405Data_mag.goal_x;
                goal.target_pose.pose.position.y = F405Data_mag.goal_y;
                goal.target_pose.pose.orientation.w = 1;
                //ac.sendGoal(goal);
                ROS_INFO("GOAL3: goal_x=%f, goal_y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
                //ac.waitForResult();    
   
                if((fabs(F405Data_mag.x_now-F405Data_mag.goal_x)<0.15) && (fabs(F405Data_mag.y_now-F405Data_mag.goal_y)<0.15) && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
                   data_write.flag=3;
                   ROS_INFO("reach goal successful!");
                   ac.cancelGoal();
                }
                else{
                   data_write.flag=0;
                   ac.sendGoal(goal);
                   ROS_INFO("sendGoal!");
                   
                  //等1分钟到达那里
                  bool finished_within_time = ac.waitForResult(ros::Duration(10));
                  //如果我们没有及时赶到那里，就会中止目标
                  if(!finished_within_time)
                  {
                      ac.cancelGoal();
                      ROS_INFO("Timed out achieving goal");
                  }
                }
            }  
            //去前哨站(6.5,-2.8) (6.5,-3.17) (7.0,-2.66)
            else if(flag_goal==4){
                //goal.target_pose.pose.position.x = 7.0;  
                //goal.target_pose.pose.position.y = -2.66;
                F405Data_mag.goal_x = 6.61;
                F405Data_mag.goal_y = -3.3;
                goal.target_pose.pose.position.x = F405Data_mag.goal_x;
                goal.target_pose.pose.position.y = F405Data_mag.goal_y;
                goal.target_pose.pose.orientation.w = -1;
               // ac.sendGoal(goal);
                ROS_INFO("GOAL4: goal_x=%f, goal_y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);   
                //ac.waitForResult(); 

                if((fabs(F405Data_mag.x_now-F405Data_mag.goal_x)<0.15) && (fabs(F405Data_mag.y_now-F405Data_mag.goal_y)<0.15) && (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
                   data_write.flag=4;
                   ROS_INFO("reach goal successful!");
                   ac.cancelGoal();
                }
                else{
                   data_write.flag=0;
                   ac.sendGoal(goal);
                   
                  //等1分钟到达那里
                  bool finished_within_time = ac.waitForResult(ros::Duration(10));
                  //如果我们没有及时赶到那里，就会中止目标
                  if(!finished_within_time)
                  {
                      ac.cancelGoal();
                      ROS_INFO("Timed out achieving goal");
                  }
                }
            }  
            else{
                data_write.flag=0;
            }

          }


        }
        catch (exception e)
        {
            //ROS_ERROR_STREAM("Some error in pub_node file ");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
