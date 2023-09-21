/**************************
 * 使用该代码以标定线速度走1m
 * 修改参数
 * robot_start.cpp文件下的init函数内的wheel_distance
 * 底盘代码中encoder.c文件下的计算motor_speed函数内的速度计算表达式
 * 
 * ************************/
	   
#include <ros/ros.h>  
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h> 
#define ABS(x) ((x)>0? (x):(-(x))) 
#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

typedef struct PID{
		float SetPoint;			//设定目标值
	  float SetPointLast;
		
		float P;						//比例常数
		float I;						//积分常数
		float D;						//微分常数
		
		float LastError;		//前次误差
		float PreError;			//当前误差
		float SumError;			//积分误差
		float dError;
		
		float ErrorMax;			//偏差上限 超出偏差则不计算积分作用
		float IMax;					//积分限制
		
		float POut;					//比例输出
		float IOut;					//积分输出
		float DOut;					//微分输出
	  float OutMax;       //限幅
}Pid_Typedef;
Pid_Typedef anglar;
	//只输出正负都会
float position_PID_Calc(Pid_Typedef *P, float ActualValue)
{
		P->PreError = P->SetPoint - ActualValue;
		P->dError = P->PreError - P->LastError;
	
	  P->SetPointLast = P->SetPoint;
	
		if(P->PreError > -P->ErrorMax && P->PreError < P->ErrorMax)
		{
			P->SumError += P->PreError; 
		}
		
		P->LastError = P->PreError;
		
		if(P->SumError >= P->IMax)
			P->SumError = P->IMax;
		else if(P->SumError <= -P->IMax)
			P->SumError = -P->IMax;
		
		P->POut = P->P * P->PreError;
		P->IOut = P->I * P->SumError;
		P->DOut = P->D * P->dError;
		
		
		return LIMIT_MAX_MIN(P->POut+P->IOut+P->DOut,P->OutMax,-P->OutMax);
}
// 初始化geometry_msgs::Twist类型的消息  
geometry_msgs::Twist vel_msg;   

double dest=1;//m
static int set_dest=0;
static int error_D=0;//错误累计

void position_Callback(const nav_msgs::Odometry msg) {

    double position_now = msg.pose.pose.position.x;
    if(set_dest == 0){
        set_dest++;
        anglar.SetPoint = position_now + 1;
    }

    vel_msg.linear.x = position_PID_Calc(&anglar ,position_now);

    ROS_INFO("\t[error: %f m]",   
         anglar.SetPoint-position_now); 

} 
int main(int argc, char **argv)  
{  
    // ROS节点初始化  
    ros::init(argc, argv, "velocity_publisher");  
    
    // 创建节点句柄  
    ros::NodeHandle n;  
    
    // 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10  
    ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);  
    //创建一个订阅者订阅键盘控制节点
    ros::Subscriber position = n.subscribe("/odom", 1, position_Callback); 
    vel_msg.angular.z = 0;
    // 设置循环的频率  
    ros::Rate loop_rate(50);  
    anglar.P = 1.5f;
    anglar.I = 0;
    anglar.D = 0.5;
    anglar.ErrorMax = 1000.0f;
    anglar.IMax = 1000.0f;
    anglar.SetPoint = 0.0f;	
    anglar.OutMax = 0.7;

    int count = 0;  
    while (ros::ok())  
    {  

         ros::spinOnce();
    
        // 发布消息  
        turtle_vel_pub.publish(vel_msg);  
        ROS_INFO("Publsh turtle velocity command[%0.4f m/s]",   
                vel_msg.linear.x);  
    
        // 按照循环频率延时  
        loop_rate.sleep();  
    }  
    
    return 0;  
}  
