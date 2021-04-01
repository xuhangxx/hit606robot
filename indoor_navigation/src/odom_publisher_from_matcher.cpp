

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <geometry_msgs/Twist.h>
#include <list>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdlib.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>  //轨迹
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <vector>
#include <tf/transform_broadcaster.h>


using namespace std;


ros::Time current_time, last_time;
double last_x, last_y, last_theta;
double last_vel_x, last_vel_y, last_angular_z;
int flag_first_time = 1;


class SubsPose2dPubOdom  
{  
public:  
    SubsPose2dPubOdom ()  
    {       
        sub_ = n_.subscribe("/pose2D", 1, &SubsPose2dPubOdom::callback_, this);
        pub_ = n_.advertise<nav_msgs::Odometry>("odom", 1);
    }


    void callback_ (const geometry_msgs::Pose2D::ConstPtr& msg)
    {


        if (!flag_first_time)
        {
            current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            nav_msgs::Odometry odom_;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(msg->theta);

            odom_.pose.pose.position.x = msg->x;
            odom_.pose.pose.position.y = msg->y;
            odom_.pose.pose.position.z = 0.0;
            odom_.pose.pose.orientation = odom_quat;



            odom_.child_frame_id = "base_link";
            odom_.twist.twist.linear.x = 0.5 * ((msg->x - last_x)/dt + last_vel_x);
            odom_.twist.twist.linear.y = 0.5 * ((msg->y - last_y)/dt + last_vel_y);
            odom_.twist.twist.angular.z = 0.5 * ((msg->theta - last_theta)/dt + last_angular_z);

            pub_.publish(odom_);
        }
        last_time = ros::Time::now();
        flag_first_time = 0;
        last_x = msg->x;
        last_y = msg->y;
        last_theta = msg->theta;
        last_vel_x = last_vel_y = last_angular_z = 0;
    }


    private:  
      ros::NodeHandle n_;
      ros::Subscriber sub_;
      ros::Publisher pub_;
}; 
      


int main( int argc, char** argv)
{
    ros::init(argc, argv, "SubsPose2dPubOdom"); 

    SubsPose2dPubOdom subnpub;  
    ros::spin();
    
    return 0;
}
