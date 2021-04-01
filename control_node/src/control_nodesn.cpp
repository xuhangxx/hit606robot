/*********************************
 * 串口节点，订阅cmd_vel话题控制底盘进行运动
 * 从决策节点的数据cmd_vel话题中分解出速度值通过串口送到移动底盘进行控制 
 * 代码修改需要删除CAN形式的轮式里程计
 * @XH
 * *******************************/
#include <sys/stat.h>
#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/String.h>       //发布串口指令
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>  //订阅决策层的cmd_vel消息数据
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
#include <nav_msgs/Path.h>  //轨迹
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <math.h>
#include <indoor_navigation/soc.h>
#include <indoor_navigation/ctrmode.h>

#define	sBUFFERSIZE	14//send buffer size 串口发送缓存长度 
//#define	rBUFFERSIZE	11//receive buffer size 串口接收缓存长度
#define	lBUFFERSIZE	40
#define	mBUFFERSIZE	4
using namespace std;

indoor_navigation::ctrmode ctrmode;
double linear_x_amplifier = 400;
double angular_z_amplifier = 200;

unsigned char s_buffer[sBUFFERSIZE];
unsigned char m_buffer[mBUFFERSIZE];
//unsigned char r_buffer[rBUFFERSIZE];
unsigned char l_buffer[lBUFFERSIZE];

double roll;
double yaw;
double yawl;
double dis=0;
double yaw_angle=0;
double yawll=0;
double yawlll=0;
double pitch;
double pitchl=0;
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
double xl2=0;
double yl2=0;
double yawl2=0;
int ct=1;
int flag_first= 1;
geometry_msgs::Quaternion quat;
serial::Serial ser;
nav_msgs::Odometry odom;
const double pi = 3.1415926535898;

string mulu;
string mulu0;
int flagmulu0=0;
int initialflag=1; 

double gpsx=0;
double gpsy=0;

double imutimel;
int flagimu=1;
double vx=0;
double vy=0;
double vz=0;
double imux=0;
double imuy=0;
double imuz=0;
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


    memset(m_buffer,0,sizeof(m_buffer));	
	m_buffer[0] = '?';
	m_buffer[1] = 'C'; 
	ser.write(m_buffer,mBUFFERSIZE);
    memset(l_buffer,0,lBUFFERSIZE);
    ser.read(l_buffer,lBUFFERSIZE);
	str=(char*)l_buffer;
	np1 = str.find_first_of('=');
	np2 = str.find_first_of(':');
	lun1 = atoi(str.substr(np1 + 1, np2 - np1 - 1).c_str());
	np1 = str.find_first_of(':');
	np2 = str.find_first_of('\r');
    lun2 = atoi(str.substr(np1 + 1, np2 - np1 - 1).c_str());
    
    
}

void data_pack(const geometry_msgs::Twist &cmd_vel)
{
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
}



//订阅turtle1/cmd_vel话题的回调函数，用于显示速度以及角速度
void cmd_vel_callback(const geometry_msgs::Twist &cmd_vel){
	data_pack(cmd_vel);
	if(flag)
	{
        flag=0;
        pitchl=pitch;
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
         ctrmode.ctrmodeback=1;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 1 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=2;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 2 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=3;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 2 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=4;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 3 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=5;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 3 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=6;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 4 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=7;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 4 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=8;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 7 && soc.action ==3)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=9;
        flag=1;
        flag_first=1;
        flagmulu0=0;
    }
    else if(soc.code == 5 && soc.action ==4)
    {
        ros::Duration(0.07).sleep();
        x=0;
        y=0;
    }
}

inline bool exist(const std::string& name){
    struct stat buffer;
    return ( stat( name.c_str(), &buffer ) == 0);
}

void imu_callback(const sensor_msgs::Imu& imu){
	tf::Transform transform;
	transform.setRotation( tf::Quaternion(imu.orientation.x,
                                          imu.orientation.y,
                                          imu.orientation.z,
                                          imu.orientation.w) );
    transform.getBasis().getEulerYPR(yaw, pitch, roll);	
    quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    double imuTime=imu.header.stamp.toSec();
    if(flagimu)
    {
        imutimel=imuTime;
        flagimu=0;
    }
    float accX = imu.linear_acceleration.x ;
    float accY = imu.linear_acceleration.y ;
    float accZ = imu.linear_acceleration.z ;

    float x1 = cos(roll) * accX - sin(roll) * accY;
    float y1 = sin(roll) * accX + cos(roll) * accY;
    float z1 = accZ;

    float x2 = x1;
    float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    float z2 = sin(pitch) * y1 + cos(pitch) * z1;

    accX = cos(yaw) * x2 + sin(yaw) * z2;
    accY = y2-9.7058;
    accZ = -sin(yaw) * x2 + cos(yaw) * z2;
    imux=imux+vx*(imuTime-imutimel)+0.5*accX*(imuTime-imutimel)*(imuTime-imutimel);
    imuy=imuy+vy*(imuTime-imutimel)+0.5*accY*(imuTime-imutimel)*(imuTime-imutimel);
    imuz=imuz+vz*(imuTime-imutimel)+0.5*accZ*(imuTime-imutimel)*(imuTime-imutimel);
    vx=vx+accX*(imuTime-imutimel);
    vy=vy+accY*(imuTime-imutimel);
    vz=vz+accZ*(imuTime-imutimel);

    imutimel=imuTime;
    std::ofstream ofile;
    ofile.open("/home/x805/catkin_ws/src/indoor_navigation/maps/imuodom.txt",std::ios::app);
    ofile<<imux<<" "<<imuy<<" "<<yaw<<std::endl;
    ofile.close();   
    std::cout<<" saassssssssssssssssss"<<std::endl;
}

int main (int argc, char** argv)
{	
    ros::init(argc, argv, "control_nodesn");
    ros::NodeHandle nh;
	ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel",1,cmd_vel_callback);//1.14报错 缓冲多少处理多少
    ros::Subscriber obstacle_sub = nh.subscribe("/Velodyne_obstacle",1,velodyne_callback);
    ros::Subscriber soc_sub = nh.subscribe("/soc",1,soc_callback); 

    ros::Subscriber imu_sub = nh.subscribe("/imu",1,imu_callback);
    ros::Publisher initial_pub= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
    ros::Publisher ctrmode_pub= nh.advertise<indoor_navigation::ctrmode>("/ctrmode",10);


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
		    n1=-lun1/583.48;//*cos(pitch-pitchl)/583.48;
            n2=-lun2/583.48;//*cos(pitch-pitchl)/583.48;
			if (flag_first)
            {
                n2l=n2;
                yawl=yaw;
                n1l=n1;
                flag_first=0;
            }
            else
            {    

                if (yaw == yawl)                      //<0.001 || yaw-yawl>-0.001)
                {
		            x=x-(n1-n1l+n2-n2l)*sin(yawl)/2;
		            y=y+(n1-n1l+n2-n2l)*cos(yawl)/2;

                }
                else
                {   
                    if(yaw-yawl<-pi)
                    {

                        x=x+(n1-n1l+n2-n2l)*(cos(yaw)-cos(yawl))/(2*(2*pi+yaw-yawl));
                        y=y+(n1-n1l+n2-n2l)*(sin(yaw)-sin(yawl))/(2*(2*pi+yaw-yawl));
                    }
                    else if(yaw-yawl> pi)
                    {
                        x=x+(n1-n1l+n2-n2l)*(cos(yaw)-cos(yawl))/(2*(yaw-yawl));
                        y=y+(n1-n1l+n2-n2l)*(sin(yaw)-sin(yawl))/(2*(yaw-yawl));
                    }
                    else
                    {
                        x=x+(n1-n1l+n2-n2l)*(cos(yaw)-cos(yawl))/(2*(yaw-yawl));
                        y=y+(n1-n1l+n2-n2l)*(sin(yaw)-sin(yawl))/(2*(yaw-yawl));
                    }
          
                }
                n1l=n1;
                n2l=n2;
                yawl=yaw;      
            }
            std::ofstream ofile;
            ofile.open("/home/x805/catkin_ws/src/indoor_navigation/maps/guijiodom.txt",std::ios::app);
            ofile<<y/100<<" "<< -x/100<<" "<<yaw<<std::endl;
            ofile.close();    
        }
        else
        {
            if(ctrmode.ctrmodeback ==1 && initialflag)
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
                initial_pub.publish(initial);
                initialflag=0;
            }
        }
        ct++; 
        ros::spinOnce(); 
        ctrmode_pub.publish(ctrmode);
        loop_rate.sleep();
    }
}


