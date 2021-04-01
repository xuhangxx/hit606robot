/*********************************
 * 决策层
 * 从激光雷达读数据决策控制
 * 从惯性导航组合设备读数据决策控制
 * @ZSJ
 * *******************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include "decision_node/decision_node.h"
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
namespace decision_node
{
  
 DecisionNodeTest::DecisionNodeTest(ros::NodeHandle &nh) : nh_(nh)
 {

    //订阅/scan话题中的激光雷达信息
	Velodyne_sub = nh_.subscribe("/velodyne_points",10,&DecisionNodeTest::Velodyne_callback,this);
    std::cout<<"Subscribe：/velodyne_points"<<std::endl;
    //订阅/IMU_DATA话题中的惯导信息
	IMU_sub = nh_.subscribe("/IMU_data",20,&DecisionNodeTest::IMUdata_callback,this);
    std::cout<<"Subscribe：/IMU_data"<<std::endl;
    //订阅/GPS_DATA话题中的经纬高信息
	GPS_sub = nh_.subscribe("/GPS_data",20,&DecisionNodeTest::GPSdata_callback,this);
    std::cout<<"Subscribe：/GPS_data"<<std::endl;
    //订阅/CMDVEL_DATA话题中的北向速度信息向前
	CmdVel_sub = nh_.subscribe("/CmdVel_data",20,&DecisionNodeTest::CmdVeldata_callback,this);
    std::cout<<"Subscribe：/CmdVel_data"<<std::endl;
	//发布控制信号cmd_vel 包括xy线速度和角速度
	cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    std::cout<<"Publisher：/cmd_vel"<<std::endl;

  }

  


//订阅/scan话题的回调函数，采集激光雷达检测的障碍物信息
void DecisionNodeTest::Velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  //velodyne_points数据转换为laserscan的scan数据
    const float RESOLUTION = 0.007;//分辨率2PI/2016  0.001~0.5
    const size_t SIZE = 2016;  //一圈2016个点
    sensor_msgs::LaserScanPtr scan(new sensor_msgs::LaserScan());

    scan->header = laserCloudMsg->header;
    scan->angle_increment = RESOLUTION;
    scan->angle_min = -M_PI;
    scan->angle_max = M_PI;
    scan->range_min = 0.0;
    scan->range_max = 200.0;
    scan->time_increment = 0.0;
    scan->ranges.resize(SIZE, INFINITY);
    scan->intensities.resize(SIZE);

      for (sensor_msgs::PointCloud2ConstIterator<float> it(*laserCloudMsg, "x"); it != it.end(); ++it)
      {   
          //把对应域中的的变量解析存放在数组中
          const uint16_t r = *((const uint16_t*)(&it[5]));  // ring 4
          const float x = it[0];  // x
          const float y = it[1];  // y
          const float i = it[4];  // intensity 3
          const float yaw_angle=atan2f(y, x);//激光雷达坐标系向前为y，右为x，上为z;
          const float a=(yaw_angle*180)/M_PI; //弧度制转换为角度
          const float distance=sqrtf(x * x + y * y);
        if (r==6||r==7||r==8||r==9||r==10||r==11||r==12)     //判断环数 确定当前点属于那个水平激光柱
         {  
          const int bin = (yaw_angle + static_cast<float>(M_PI)) / RESOLUTION;   //加PI的原因是因为从X负半轴起始值
         if ((bin >= 0) && (bin < static_cast<int>(SIZE)))
          {
            if(yaw_angle>=(-M_PI/4)&&yaw_angle<=(M_PI/4))    //设置可视化区域的角度范围 18
             { 
               
             if (distance>=0&&distance<=10)                  //障碍物点距离激光雷达坐标系远点的距离
              { 
                ROS_INFO("I heard obstacle distance: [%f]", distance);
                ROS_INFO("I heard yaw_angle: [%f]", a); 
        
                scan->ranges[bin] = distance;    
                scan->intensities[bin] = i;  //当前点数组中存储的强度值 障碍物点的显示
              }

             }
           }
         }

       }
        //添加障碍物检测的决策控制程序
  

       //pubLaserScan.publish(scan);
	   std::cout << "Obstacle Received" << std::endl;	
}



//订阅/IMU_data话题的回调函数，采集IMU检测的姿态信息
void DecisionNodeTest::IMUdata_callback(const sensor_msgs::Imu& imu_data)
{
	//double roll,pitch,yaw;
	//tf::Quaternion quat;
    //quat[0]=imu_data.orientation.x;
	//quat[1]=imu_data.orientation.y;
	//quat[2]=imu_data.orientation.z;
	//quat[3]=imu_data.orientation.w;

    //tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);

	ROS_INFO("I heard yaw: [%f]",imu_data.orientation.z);  //偏航角姿态
    ROS_INFO("I heard x linear_acceleration: [%f]", imu_data.linear_acceleration.x);  //x轴加速度
	ROS_INFO("I heard z angular_velocity: [%f]", imu_data.angular_velocity.z);   //z轴角速度
	std::cout << "IMUdata Received" << std::endl;	
}



//订阅/GPS_data话题的回调函数，采集GPS检测的位置信息
void DecisionNodeTest::GPSdata_callback(const sensor_msgs::NavSatFix& gps_data)
{
	ROS_INFO("I heard latitude: [%f]",gps_data.latitude);   //维度
	ROS_INFO("I heard longitude: [%f]",gps_data.longitude); //经度
	std::cout << "GPSdata Received" << std::endl;	
    
    
    //根据GPS经纬度信息控制车自动架势巡检程序
     



     

     DecisionNodeTest::cmd_vel_publish();

}



//订阅/GPS_data话题的回调函数，采集GPS检测的位置信息
void DecisionNodeTest::CmdVeldata_callback(const geometry_msgs::Twist& cmdvel_data)
{
	ROS_INFO("I heard north velocity: [%f]",cmdvel_data.linear.x);   //N前向速度
	ROS_INFO("I heard earth velocity: [%f]",cmdvel_data.linear.y); //E右向速度
	std::cout << "CmdVel_data Received" << std::endl;	

}








void DecisionNodeTest::cmd_vel_publish()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x=1.0;
    cmd_vel.linear.y=0;
    cmd_vel.angular.z=0;
	//ROS_INFO("I heard linear velocity: x-[%f],y-[%f],",cmd_vel.linear.x,cmd_vel.linear.y);
	//ROS_INFO("I heard angular velocity: [%f]",cmd_vel.angular.z);

	cmd_vel_pub.publish(cmd_vel);
    std::cout << "Twist Cmd_Vel Send!" << std::endl;	
}



}




    

