/*********************************
 * 串口节点，订阅cmd_vel话题控制底盘进行运动
 * 从决策节点的数据cmd_vel话题中分解出速度值通过串口送到移动底盘进行控制 
 * 代码修改需要删除CAN形式的轮式里程计
 * @ZSJ
 * *******************************/
#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/String.h>       //发布串口指令
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>  //订阅决策层的cmd_vel消息数据
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
#include <boost/thread.hpp>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>  //轨迹
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <math.h>

#define	sBUFFERSIZE	14//send buffer size 串口发送缓存长度 
#define	rBUFFERSIZE	11//receive buffer size 串口接收缓存长度
#define	lBUFFERSIZE	29
#define	mBUFFERSIZE	4
using namespace std;

double linear_x_amplifier = 800;
double angular_z_amplifier = 100;

unsigned char s_buffer[sBUFFERSIZE];
unsigned char m_buffer[mBUFFERSIZE];
unsigned char r_buffer[rBUFFERSIZE];
unsigned char l_buffer[lBUFFERSIZE];

double roll;
double yaw;
double yawl;
double yawll=0;
double pitch;
double pitchl=0;
//nav_msgs::Odometry odom;
int np1=0;
int np2=0;
int lun1=0;
int lun2=0;
int flag=1;
string str="";
double n1=0;
double n2=0;
double n1l=0;
double n2l=0;
double x=0;
double y=0;
double xl=0;
double yl=0;
double wx=0;
double wy=0;
double wz=0;
int ct=1;
int flag_first= 1;
geometry_msgs::Quaternion quat;
serial::Serial ser;

int flag2=1;
int first=0;
int last=1;
const double pi = 3.1415926535898;
double a=6378160;   
double b=6356775;  
double c=(a-b)/a; 
double origin_x;
double origin_y;
int p;
double dx=0;
double dy=0;
double dyaw=0;
double openyaw=0;
double openroll=0;
double openpitch=0;
double origin_yaw=0;

/**********************************************************
 * 检测决策层的控制数据打包，将获取的cmd_vel信息打包并通过串口发送
 * ********************************************************/
void get_command ( const int& velocity_1, const int& velocity_2 )
{    
    vector <unsigned char> ucv;
    char c1[6];
	char c2[6];
	memset(s_buffer,0,sizeof(s_buffer));

    ucv.push_back('!');
    ucv.push_back('M');
    ucv.push_back(' ');
    
    // velocity 1 angular
    sprintf(c1, "%d", velocity_1);
    ucv.push_back(c1[0]);
    if (velocity_1 >9)
      ucv.push_back(c1[1]);
	      if (velocity_1 >99)
      ucv.push_back(c1[2]); 
	  // minus 0
	  if (velocity_1 < 0)
	        ucv.push_back(c1[1]);
				  if (velocity_1 < -9)
	        ucv.push_back(c1[2]);
				  if (velocity_1 < -99)
	        ucv.push_back(c1[3]);
    ucv.push_back(' ');
    // velocity 2
    sprintf(c2, "%d", velocity_2);
    ucv.push_back(c2[0]);
    if (velocity_2 >9)
      ucv.push_back(c2[1]);
	      if (velocity_2 >99)
      ucv.push_back(c2[2]); 
	  // minus 0
	  if (velocity_2 < 0)
	        ucv.push_back(c2[1]);
				  if (velocity_2 < -9)
	        ucv.push_back(c2[2]);
				  if (velocity_2 < -99)
	        ucv.push_back(c2[3]);
    
    
    int index = 0;
    for (auto uc:ucv)
    {
      s_buffer[index] = uc;
      index ++;
    }
	ser.write(s_buffer,sBUFFERSIZE);
	ros::Duration(0.03).sleep();
    
    
}

void data_pack(const geometry_msgs::Twist &cmd_vel)
{
	unsigned char i;
	
	int velocity_1  = int(angular_z_amplifier* cmd_vel.angular.z);	
	int velocity_2  = int(linear_x_amplifier* cmd_vel.linear.x);

	get_command(velocity_1, velocity_2);
	
}



//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel){
	/*ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);
	std::cout << "Twist Received" << std::endl;	*/
	data_pack(cmd_vel);
	if(flag)
	{
		flag=0;
        pitchl=pitch;
	}
    
	
}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "control_node0");
    ros::NodeHandle nh;
    /*ros::Publisher pub_= nh.advertise<nav_msgs::Odometry>("/odom",1000);*/
	ros::Subscriber write_sub = nh.subscribe("/cmd_vel",1,cmd_vel_callback);
    /*tf::TransformListener listener;*/
	ros::Rate loop_rate(10);

     try
    {
        ser.setPort("/dev/car");  //串口驱动号需要修改
        ser.setBaudrate(115200);     //波特率设置
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port_1 ");
        return -1;
    }

    if(ser.isOpen())
	{
        ROS_INFO_STREAM("Serial Port_1 initialized");
    }
	else
	{
        return -1;
    }

    
	while(ros::ok())
	{
        ros::spinOnce(); 
        loop_rate.sleep();
    }
}

