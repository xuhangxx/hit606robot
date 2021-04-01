/*
 * Translates sensor_msgs/NavSat{Fix,Status} into nav_msgs/Odometry using UTM
 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/Imu.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <indoor_navigation/soc.h>

#include <sys/stat.h>
#include <std_msgs/String.h>  
#include <iostream>
#include <fstream>

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


using namespace gps_common;
using namespace std;

static ros::Publisher gps_pub;
static ros::Publisher pathgps_pub;
std::string frame_id, child_frame_id;
double rot_cov;
bool append_zone = false;
int flag=1;
int flag2=0;
double deltax,deltay;
double deltanorthing,deltaeasting;
double x,y;
double yaw1,yaw2,pitch,roll,pitch1,roll1;
double deltayaw;
const double pi = 3.1415926535898;
nav_msgs::Odometry odom;
std::string mulu0;
static tf::TransformBroadcaster odom_broadcaster;
geometry_msgs::TransformStamped odom_trans;
nav_msgs::Path path;
inline bool exist(const std::string& name){
    struct stat buffer;
    return ( stat( name.c_str(), &buffer ) == 0);
}

void callback(const sensor_msgs::NavSatFixConstPtr& fix) {

  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_DEBUG_THROTTLE(60,"No fix.");
    odom.header.stamp = fix->header.stamp;
    odom.header.frame_id = fix->header.frame_id;
    odom.child_frame_id = child_frame_id;
    odom.twist.twist.linear.x=0;
    gps_pub.publish(odom);
    return;
  }
  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
  if (gps_pub) {
    odom.header.stamp = fix->header.stamp;

    if (frame_id.empty()) {
      if(append_zone) {
        odom.header.frame_id = fix->header.frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = fix->header.frame_id;
      }
    } else {
      if(append_zone) {
        odom.header.frame_id = frame_id + "/utm_" + zone;
      } else {
        odom.header.frame_id = frame_id;
      }
    }
    
    if(fix->status.status==1 && flag == 1)
    {
      flag=0;
      deltaeasting=easting;
      deltanorthing=northing;
      deltax=x;
      deltay=y;
      deltayaw=yaw1-yaw2;
      std::ofstream infile("/home/x805/catkin_ws/src/indoor_navigation/maps/gps.txt");
      infile.flush();
      infile<<deltaeasting<<" "<<deltanorthing<<" "<<deltax<<" "<<deltay<<" "<<deltayaw;
      infile.close();
    }
    else if(flag2)
    {
      flag2=0;
      deltaeasting=easting;
      deltanorthing=northing;
      deltax=x;
      deltay=y;
    }
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = deltax+(northing-deltanorthing)*cos(deltayaw)-(easting-deltaeasting)*sin(deltayaw);
    odom.pose.pose.position.y = deltay-(northing-deltanorthing)*sin(deltayaw)-(easting-deltaeasting)*cos(deltayaw);
    odom.pose.pose.position.z = fix->altitude;
    
    odom.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw2);
    
    //odom.pose.pose.orientation.x = 0;
    //odom.pose.pose.orientation.y = 0;
    //odom.pose.pose.orientation.z = 0;
    //odom.pose.pose.orientation.w = 1;
    
    // Use ENU covariance to build XYZRPY covariance
    boost::array<double, 36> covariance = {{
      fix->position_covariance[0],
      fix->position_covariance[1],
      fix->position_covariance[2],
      0, 0, 0,
      fix->position_covariance[3],
      fix->position_covariance[4],
      fix->position_covariance[5],
      0, 0, 0,
      fix->position_covariance[6],
      fix->position_covariance[7],
      fix->position_covariance[8],
      0, 0, 0,
      0, 0, 0, rot_cov, 0, 0,
      0, 0, 0, 0, rot_cov, 0,
      0, 0, 0, 0, 0, rot_cov
    }};
    odom.twist.twist.linear.x=1;
    odom.pose.covariance = covariance;
    
    gps_pub.publish(odom);



    path.header.stamp=ros::Time::now();
    path.header.frame_id="odom";

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x=odom.pose.pose.position.x;
    this_pose_stamped.pose.position.y=odom.pose.pose.position.y;
    this_pose_stamped.pose.orientation=odom.pose.pose.orientation;

    this_pose_stamped.header.stamp=ros::Time::now();
    this_pose_stamped.header.frame_id="odom";
    path.poses.push_back(this_pose_stamped);
    pathgps_pub.publish(path);
   // odom_trans.header.stamp = ros::Time::now();
    //odom_trans.header.frame_id = "/odom";
    //odom_trans.child_frame_id = "/base_footprint";
    //odom_trans.transform.translation.x = odom.pose.pose.position.x;
    //odom_trans.transform.translation.y = odom.pose.pose.position.y;
    //odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation=odom.pose.pose.orientation;
   // odom_broadcaster.sendTransform(odom_trans);
  }
}

void imucallback (const sensor_msgs::Imu& msg)
{
    tf::Transform transform;
	  transform.setRotation( tf::Quaternion(msg.orientation.x,
                                          msg.orientation.y,
                                          msg.orientation.z,
                                          msg.orientation.w) );
    transform.getBasis().getEulerYPR(yaw1, pitch1, roll1);	
    yaw1=-yaw1;
}

void odomcallback (const nav_msgs::Odometry& msg)
{
    x=msg.pose.pose.position.x;
    y=msg.pose.pose.position.y;
    tf::Transform transform;
	  transform.setRotation( tf::Quaternion(msg.pose.pose.orientation.x,
                                          msg.pose.pose.orientation.y,
                                          msg.pose.pose.orientation.z,
                                          msg.pose.pose.orientation.w) );
    transform.getBasis().getEulerYPR(yaw2, pitch, roll);	
}
void soc_callback(const indoor_navigation::soc& soc)
{
    if(soc.code == 5 && soc.action ==4)
    {
        ros::Duration(0.09).sleep();
        flag2=1;
    }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "utm_odometry_nodegps");
  ros::NodeHandle node;
  ros::NodeHandle priv_node("~");

  priv_node.param<std::string>("frame_id", frame_id, "");
  priv_node.param<std::string>("child_frame_id", child_frame_id, "");
  priv_node.param<double>("rot_covariance", rot_cov, 99999.0);
  priv_node.param<bool>("append_zone", append_zone, false);

  gps_pub = node.advertise<nav_msgs::Odometry>("/gps", 1);
  ros::Subscriber fix_sub = node.subscribe("/GPS_data", 1, callback);
  ros::Subscriber odom_sub = node.subscribe("/odom", 1, odomcallback);
  ros::Subscriber IMU_sub = node.subscribe("/IMU_data", 1, imucallback);
  pathgps_pub= node.advertise<nav_msgs::Path>("/trajectorygps",1);
  ros::Subscriber soc_sub = node.subscribe("/soc",1,soc_callback);
  mulu0="/home/x805/catkin_ws/src/indoor_navigation/maps/gps.txt";
  std::stringstream mulu2;
  mulu2<<mulu0;
  if(exist(mulu2.str()))
  {
     std::ifstream myfile("/home/x805/catkin_ws/src/indoor_navigation/maps/gps.txt");
     myfile>>deltaeasting>>deltanorthing>>deltax>>deltay>>deltayaw;
     myfile.close();
     flag=0;
  }

  boost::array<double, 36> covariance = {{
      1000000,
      1000000,
      1000000,
      0, 0, 0,
      1000000,
      1000000,
      1000000,
      0, 0, 0,
      1000000,
      1000000,
      1000000,
      0, 0, 0,
      0, 0, 0, 1000000, 0, 0,
      0, 0, 0, 0, 1000000, 0,
      0, 0, 0, 0, 0, 1000000
    }};
  odom.pose.covariance = covariance;

  ros::spin();
}

