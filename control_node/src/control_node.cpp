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

#include <nav_msgs/Path.h>
#define	sBUFFERSIZE	14//send buffer size 串口发送缓存长度 
//#define	rBUFFERSIZE	11//receive buffer size 串口接收缓存长度
#define	lBUFFERSIZE	40
#define	mBUFFERSIZE	4
using namespace std;

indoor_navigation::ctrmode ctrmode;

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

double cp0,cp7,cp8,cp14,cp21,cp28,cp35,ct0,ct7,ct8,ct14,ct21,ct28,ct35,cpv0,cpv8,cpv35,ctv0,ctv8,ctv35,dex,dey,decx,decy,linear_x_amplifier,angular_z_amplifier;

string mulu;
string mulu0;
int flagmulu0=0;
int initialflag=1; 

double gpsx=0;
double gpsy=0;

nav_msgs::Path path;
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
    /*if( velocity_1==0 && velocity_2==0)
    {
        odom.pose.covariance[0]=1e-9;
        odom.pose.covariance[8]=1e-9;
        odom.pose.covariance[35]=1e-9;
        odom.twist.covariance[0]=1e-9;
        odom.twist.covariance[8]=1e-9;
        odom.twist.covariance[35]=1e-9;
    }
    else
    {*/
        odom.pose.covariance[0]=cpv0;
        odom.pose.covariance[8]=cpv8;
        odom.pose.covariance[35]=cpv35;
        odom.twist.covariance[0]=ctv0;
        odom.twist.covariance[8]=ctv8;
        odom.twist.covariance[35]=ctv35;
    //}
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

void imu_callback(const sensor_msgs::Imu& imu){
	tf::Transform transform;
	transform.setRotation( tf::Quaternion(imu.orientation.x,
                                          imu.orientation.y,
                                          imu.orientation.z,
                                          imu.orientation.w) );
    transform.getBasis().getEulerYPR(yaw, pitch, roll);	
    quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
    //std::cout<<"  dddddddddddddddddddddddddddddddddddddddddddddddddddddddd"<<imu.header.stamp<<std::endl;
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
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 1 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=2;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 2 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=3;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 2 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=4;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 3 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=5;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 3 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=6;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 4 && soc.action ==1)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=7;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 4 && soc.action ==2)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=8;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
    }
    else if(soc.code == 7 && soc.action ==3)
    {
        ros::Duration(0.07).sleep();
        ctrmode.ctrmodeback=9;
        //flag=1;
        //flag_first=1;
        //flagmulu0=0;
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

void gps_callback(const nav_msgs::Odometry& gps)
{ 
    
    //std::cout<<"  gpsx "<<gpsx<<"    "<<odom.pose.pose.position.x<<"ssss"<<gps.pose.pose.position.x<<std::endl;
    if((odom.pose.pose.position.x-gpsx)>dex || (odom.pose.pose.position.x-gpsx)<-dex || (odom.pose.pose.position.y-gpsy)>dey || (odom.pose.pose.position.y-gpsy)<-dey)
    {
        if(gps.pose.covariance[0]<decx && gps.pose.covariance[7]<decy&& ct>10)
        {

    //std::cout<<"  dddddddddddddddddddddddddddddddddddddddddddddddddddddddd"<<std::endl;
            gpsx=gps.pose.pose.position.x;
            gpsy=gps.pose.pose.position.y;
            y=100*gps.pose.pose.position.x;
            x=-100*gps.pose.pose.position.y;
        }
    }
}


int main (int argc, char** argv)
{	
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;
	ros::Subscriber cmd_sub = nh.subscribe("/cmd_vel",1,cmd_vel_callback);//1.14报错 缓冲多少处理多少
	ros::Subscriber imu_sub = nh.subscribe("/imu",5,imu_callback);
	ros::Subscriber gps_sub = nh.subscribe("/gps",1,gps_callback);
    ros::Subscriber obstacle_sub = nh.subscribe("/Velodyne_obstacle",1,velodyne_callback);
    ros::Subscriber soc_sub = nh.subscribe("/soc",1,soc_callback); 

    ros::Publisher odom_pub= nh.advertise<nav_msgs::Odometry>("/odom",1);
    ros::Publisher initial_pub= nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10);
    ros::Publisher ctrmode_pub= nh.advertise<indoor_navigation::ctrmode>("/ctrmode",10);
    ros::Publisher pathodom_pub= nh.advertise<nav_msgs::Path>("/trajectoryodom",1);

    odom.header.frame_id = "odom";
    odom.pose.pose.position.z = 0.0;
    odom.child_frame_id = "base_footprint";
    ctrmode.ctrmodeback=7;
    nh_private.param<double>("cp0",  cp0, 1e-3);
    nh_private.param<double>("cp7",  cp7, 1e-3);
    nh_private.param<double>("cp8",  cp8, 1e-9);
    nh_private.param<double>("cp14",  cp14, 1e6);
    nh_private.param<double>("cp21",  cp21, 1e6);
    nh_private.param<double>("cp28",  cp28, 1e6);
    nh_private.param<double>("cp35",  cp35, 1e-9);
    nh_private.param<double>("ct0",  ct0, 1e-9);
    nh_private.param<double>("ct7",  ct7, 1e-3);
    nh_private.param<double>("ct8",  ct8, 1e-9);
    nh_private.param<double>("ct14",  ct14, 1e6);
    nh_private.param<double>("ct21",  ct21, 1e6);
    nh_private.param<double>("ct28",  ct28, 1e6);
    nh_private.param<double>("ct35",  ct35, 1e-9);
    nh_private.param<double>("cpv0",  cpv0, 1e-3);
    nh_private.param<double>("cpv8",  cpv8, 0);
    nh_private.param<double>("cpv35",  cpv35, 1e3);
    nh_private.param<double>("ctv0",  ctv0, 1e-3);
    nh_private.param<double>("ctv8",  ctv8, 0);
    nh_private.param<double>("ctv35",  ctv35, 1e3);
    nh_private.param<double>("dey",  dey, 5);
    nh_private.param<double>("dex",  dex, 5);
    nh_private.param<double>("decy",  decy, 100);
    nh_private.param<double>("decx",  decx, 100);
    nh_private.param<double>("angular_z_amplifier",  angular_z_amplifier, 200);
    nh_private.param<double>("linear_x_amplifier",  linear_x_amplifier, 400);
    for (int i=0; i<36; ++i)
    {
        odom.pose.covariance[i]=0;
        odom.twist.covariance[i]=0;
    }
    odom.pose.covariance[0]=cp0;
    odom.pose.covariance[7]=cp7;
    odom.pose.covariance[8]=cp8;
    odom.pose.covariance[14]=cp14;
    odom.pose.covariance[21]=cp21;
    odom.pose.covariance[28]=cp28;
    odom.pose.covariance[35]=cp35;
    odom.twist.covariance[0]=ct0;
    odom.twist.covariance[7]=ct7;
    odom.twist.covariance[8]=ct8;
    odom.twist.covariance[14]=ct14;
    odom.twist.covariance[21]=ct21;
    odom.twist.covariance[28]=ct28;
    odom.twist.covariance[35]=ct35;
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
            
            odom.header.stamp = ros::Time::now();
            odom.header.seq = ct;
            odom.pose.pose.position.x = y/100;
            odom.pose.pose.position.y = -x/100;
            odom.pose.pose.orientation = quat;
            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = 0;

            odom_pub.publish(odom);


            path.header.stamp=ros::Time::now();
    	    path.header.frame_id="odom";

    	    geometry_msgs::PoseStamped this_pose_stamped;
    	    this_pose_stamped.pose.position.x=y/100;
    	    this_pose_stamped.pose.position.y=-x/100;
    	    this_pose_stamped.pose.orientation=quat;

    	    this_pose_stamped.header.stamp=ros::Time::now();
    	    this_pose_stamped.header.frame_id="odom";
    	    path.poses.push_back(this_pose_stamped);
            pathodom_pub.publish(path);
        }
        else
        {
            mulu0="/home/x805/catkin_ws/src/indoor_navigation/maps/chushi.txt";
            while(!flagmulu0)
            {
                stringstream mulu2;
                mulu2<<mulu0;
                if(exist(mulu2.str()))
                {
                    ifstream myfile("/home/x805/catkin_ws/src/indoor_navigation/maps/chushi.txt");
	                myfile>>xl2>>yl2>>yawlll;
	                myfile.close();
                    flagmulu0=1;
                    break;
                }
                else
                {
                    flagmulu0=1;
                }
            }
            x=-100*yl2; 
            y=100*xl2;

            odom.header.stamp = ros::Time::now();
            odom.header.seq = ct;
            odom.pose.pose.position.x = xl2;
            odom.pose.pose.position.y = yl2;
            odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawlll);
            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.angular.z = 0;

            odom_pub.publish(odom);


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


