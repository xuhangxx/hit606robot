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


double pi=3.1415926535898;
int flag_first_time = 1;
double initial_yaw;


class SubsIMUdataPubImu  
{  
public:  
    SubsIMUdataPubImu ()  
    {       
        sub_ = n_.subscribe("/IMU_data", 10, &SubsIMUdataPubImu::callback_, this);
        pub_ = n_.advertise<sensor_msgs::Imu>("imu/data", 1);
    }


    void callback_ (const sensor_msgs::Imu::ConstPtr& msg)
    {
        if (flag_first_time)
        {
            flag_first_time = 0;
            initial_yaw = msg->orientation.z;
        }

        sensor_msgs::Imu imu_publish;
        imu_publish = *msg;

        double yaw_ = -(msg->orientation.z - initial_yaw) * pi / 180;
        
        imu_publish.orientation.x = 0;
        imu_publish.orientation.y = 0;
        imu_publish.orientation.z = sin (0.5 * yaw_);
        imu_publish.orientation.w = cos (0.5 * yaw_);

        pub_.publish(imu_publish);

    }


    private:  
      ros::NodeHandle n_;
      ros::Subscriber sub_;
      ros::Publisher pub_;
}; 
      


int main( int argc, char** argv)
{
    ros::init(argc, argv, "SubsIMUdataPubImu"); 

    SubsIMUdataPubImu subnpub;  
    ros::spin();
    
    return 0;
}
