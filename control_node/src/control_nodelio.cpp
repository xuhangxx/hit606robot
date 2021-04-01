/*********************************
 * 串口节点，订阅cmd_vel话题控制底盘进行运动
 * lio-sam激光定位版本小车串口驱动代码
 * @XH
 * *******************************/
#include <sys/stat.h>
#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/String.h>       
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/Pose2D.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/Imu.h"
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <velodyne_datatest/velodyne_obstacle.h>
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
#include <nav_msgs/Path.h> 
#include <sensor_msgs/NavSatFix.h>   
#include <math.h>
#include <indoor_navigation/soc.h>

#define	sBUFFERSIZE	14
using namespace std;

double linear_x_amplifier = 400;
double angular_z_amplifier = 200;

unsigned char s_buffer[sBUFFERSIZE];

double roll;
double yaw;
double yawl;
double dis=0;
double yaw_angle=0;
double yawll=0;
double yawlll=0;
int flag=1;
string str="";
double xl2=0;
double yl2=0;
double yawl2=0;
int ct=1;
geometry_msgs::Quaternion quat;
serial::Serial ser;
nav_msgs::Odometry odom;
const double pi = 3.1415926535898;



string mulu0;
int mode=7;
int initialflag=1; 


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



//订阅/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel){
	unsigned char i;
    int velocity_1;
    int velocity_2;
	if(dis==0)
    {
        velocity_1  = int(angular_z_amplifier* cmd_vel.angular.z);	
	    velocity_2  = int(linear_x_amplifier* cmd_vel.linear.x);
        get_command(velocity_1, velocity_2);
    }
	else
    {
        velocity_1  = 0;	
	    velocity_2  = 0;
        dis=0;
        yaw_angle=0.0;
        get_command(velocity_1, velocity_2);
    }
	if(flag)
	{
        flag=0;
        initialflag=1;
	}
    
	
}


void velodyne_callback(const velodyne_datatest::velodyne_obstacle& ve)
{
    dis=ve.distance;
    yaw_angle=ve.yaw_angle;
}

void soc_callback(const indoor_navigation::soc& soc)
{
    if(soc.code == 1 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        mode=1;
        flag=1;
    }
    else if(soc.code == 1 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        mode=2;
        flag=1;
    }
    else if(soc.code == 2 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        mode=3;
        flag=1;
    }
    else if(soc.code == 2 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        mode=4;
        flag=1;
    }
    else if(soc.code == 3 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        mode=5;
        flag=1;
    }
    else if(soc.code == 3 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        mode=6;
        flag=1;
    }
    else if(soc.code == 4 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        mode=7;
        flag=1;
    }
    else if(soc.code == 4 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        mode=8;
        flag=1;
    }
}

inline bool exist(const std::string& name){
    struct stat buffer;
    return ( stat( name.c_str(), &buffer ) == 0);
}


int main (int argc, char** argv)
{
    ros::init(argc, argv, "control_nodelio");
    ros::NodeHandle nh;
	ros::Subscriber write_sub = nh.subscribe("/cmd_vel",1,cmd_vel_callback);
    ros::Publisher pubinitial= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);
    ros::Subscriber write_sub3 = nh.subscribe("/Velodyne_obstacle",2,velodyne_callback);
    ros::Subscriber soc_sub = nh.subscribe("/soc",1,soc_callback); 
    
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
        if(!flag)
        {

        }
        else
        {


            if(mode ==1 && initialflag)
            {
                std::stringstream mulu2;
                mulu2<<mulu0;
                if(exist(mulu2.str()))
                {
                    std::ifstream myfile("/home/x805/catkin_ws/src/indoor_navigation/maps/chushi.txt");
	                myfile>>xl2>>yl2>>yawl2;
	                myfile.close();
                }
                else
                {
                    xl2=0;
                    yl2=0;
                    yawl2=0;
                }       
                geometry_msgs::PoseWithCovarianceStamped initial;
                initial.header.stamp = ros::Time::now();
                initial.header.frame_id = "map";
                initial.header.seq= 0;
                initial.pose.pose.position.x= xl2;
                initial.pose.pose.position.y= yl2;
                initial.pose.pose.position.z= 0;
                initial.pose.pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawl2);
                pubinitial.publish(initial);
                initialflag=0;
            }
        }
        ct++; 
        ros::spinOnce(); 
        loop_rate.sleep();
    }
}


