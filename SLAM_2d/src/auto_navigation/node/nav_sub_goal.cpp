/***********************nav_sub_goal**********************************
*fun: 获取/goal_pose的目标地点并导航
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
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#define WAIT_TIME 300 //s


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal goal;
int send_flag =0;//让发送信息从回调函数传到while内
int arrive_flag = 0;//通知到达信息只有一次
ros::Time last_time;
ros::Time current_time;
double time_d = 0;//计算受到控制后的空闲时间，和waiting_time比较
int waiting_time=0;//设定的闲置等待时间
void goal_Callback(const geometry_msgs::Pose msg){
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    // goal.target_pose.pose.orientation.w = 0.68;
    // goal.target_pose.pose.orientation.z = 0.7;

    goal.target_pose.pose.position = msg.position;
    goal.target_pose.pose.orientation = msg.orientation;

    ros::param::set("Run_state",1);//将机器人状态修改为控制模式
    ros::param::get("waitTime",waiting_time);//获取闲职等待时间s,不能为0
    ROS_INFO("Sending goal\n");
    send_flag = 1; 
    time_d = 0;
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"sub_goal");

    ros::NodeHandle nh;
    ros::param::set("Run_state",0);
    MoveBaseClient ac("move_base", true);

    //创建一个订阅者订阅键盘控制节点
    ros::Subscriber goal_sub = nh.subscribe("/goal_pose", 1, goal_Callback);

    current_time = ros::Time::now();
    last_time = ros::Time::now();
    //tell the action client that we want to spin a thread by default
    	
    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }


    //循环运行
    ros::Rate loop_rate(50);
    while (ros::ok()) 
    {
        ros::spinOnce();

        //如果五分钟没有app或者语音控制

        if(arrive_flag == 1)//如果到达目标了
            current_time = ros::Time::now();
        
        time_d = (current_time-last_time).toSec();
        //ROS_INFO("time_d = %f",time_d);
        if(time_d >= waiting_time && waiting_time!=0){
            ros::param::set("/Run_state",0);
            waiting_time = 0;
            //time_d置0
            last_time = ros::Time::now();;
            current_time = ros::Time::now(); 
        }


        //接收到app传来的pose信息发送目标
        if(send_flag){
            ac.sendGoal(goal);
            ac.waitForResult();
            arrive_flag = 0;
            send_flag = 0;
            ROS_INFO("Send a new goal\n");
        }

    

        if((ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) && (arrive_flag == 0))
        {
            ROS_INFO("Arrive goal\n");
            arrive_flag = 1;
            last_time = ros::Time::now();
            current_time = ros::Time::now();
        }    
        // else if(ac.getState() == actionlib::SimpleClientGoalState::LOST){
        //     ROS_INFO("fail to goal");
        // }
            


        loop_rate.sleep();
    }


    return 0;
}


