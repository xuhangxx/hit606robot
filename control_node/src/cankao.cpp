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
#include <nav_msgs/Path.h>
#include <signal.h>
#include <tf/transform_listener.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#define	sBUFFERSIZE	14//send buffer size 串口发送缓存长度 
//#define	rBUFFERSIZE	11//receive buffer size 串口接收缓存长度
#define	lBUFFERSIZE	40
#define	mBUFFERSIZE	4
using namespace std;
double a,b,c;
nav_msgs::Path path;
string filename;
geometry_msgs::Quaternion setQuaternion(double _angleRan);

int main (int argc, char** argv)
{	
    ros::init(argc, argv, "cankao");
    ros::NodeHandle nh;

    ros::Publisher path_pub= nh.advertise<nav_msgs::Path>("/trajectory0",1);

    path.header.stamp=ros::Time::now();
    path.header.frame_id="odom";
    filename="/home/x805/catkin_ws/src/indoor_navigation/scripts/guiji/guiji0.txt";
    ifstream myfile(filename);
	while(!myfile.eof())
	{
		myfile>>a>>b>>c;
    		geometry_msgs::PoseStamped this_pose_stamped;
    		this_pose_stamped.pose.position.x=a;
    		this_pose_stamped.pose.position.y=b;
    		this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0, 0, c);

    		this_pose_stamped.header.stamp=ros::Time::now();
    		this_pose_stamped.header.frame_id="odom";
    		path.poses.push_back(this_pose_stamped);
	}
	//std::cout<<"e"<<std::endl;
	myfile.close();
    /*for (int i=0;i<540;i++)
    {
    	geometry_msgs::PoseStamped this_pose_stamped;
    	this_pose_stamped.pose.position.x=i*0.01;
    	this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    	this_pose_stamped.header.stamp=ros::Time::now();
    	this_pose_stamped.header.frame_id="odom";
    	path.poses.push_back(this_pose_stamped);
    }

    for (int i=0;i<566;i++)
    {
    	geometry_msgs::PoseStamped this_pose_stamped;
    	this_pose_stamped.pose.position.x=5.4;
    	this_pose_stamped.pose.position.y=i*0.01;
    	this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,1.5708);

    	this_pose_stamped.header.stamp=ros::Time::now();
    	this_pose_stamped.header.frame_id="odom";
    	path.poses.push_back(this_pose_stamped);
    }
    for (int i=0;i<5400;i++)
    {
    	geometry_msgs::PoseStamped this_pose_stamped;
    	this_pose_stamped.pose.position.x=5.4+i*0.01;
    	this_pose_stamped.pose.position.y=5.66;
    	this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    	this_pose_stamped.header.stamp=ros::Time::now();
    	this_pose_stamped.header.frame_id="odom";
    	path.poses.push_back(this_pose_stamped);
    }
    for (int i=0;i<500;i++)
    {
    	geometry_msgs::PoseStamped this_pose_stamped;
    	this_pose_stamped.pose.position.x=i*0.01;
    	this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    	this_pose_stamped.header.stamp=ros::Time::now();
    	this_pose_stamped.header.frame_id="odom";
    	path.poses.push_back(this_pose_stamped);
    }

    for (int i=0;i<500;i++)
    {
    	geometry_msgs::PoseStamped this_pose_stamped;
    	this_pose_stamped.pose.position.x=5;
    	this_pose_stamped.pose.position.y=i*0.01;
    	this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,1.5708);

    	this_pose_stamped.header.stamp=ros::Time::now();
    	this_pose_stamped.header.frame_id="odom";
    	path.poses.push_back(this_pose_stamped);
    }

    for (int i=0;i<500;i++)
    {
    	geometry_msgs::PoseStamped this_pose_stamped;
    	this_pose_stamped.pose.position.x=5-0.0*i;
    	this_pose_stamped.pose.position.y=5;
    	this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,3.1415);

    	this_pose_stamped.header.stamp=ros::Time::now();
    	this_pose_stamped.header.frame_id="odom";
    	path.poses.push_back(this_pose_stamped);
    }


    for (int i=0;i<500;i++)
    {
    	geometry_msgs::PoseStamped this_pose_stamped;
    	this_pose_stamped.pose.position.x=0;
    	this_pose_stamped.pose.position.y=5-0.01*i;
    	this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,-1.5708);

    	this_pose_stamped.header.stamp=ros::Time::now();
    	this_pose_stamped.header.frame_id="odom";
    	path.poses.push_back(this_pose_stamped);
    }*/
    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce(); 
        path_pub.publish(path);
        loop_rate.sleep();
    }
}


