

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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>  //轨迹
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <vector>



using namespace std;


class SubsCmdvel  
{  
public:  
    SubsCmdvel ()  
    {       
        sub_cmdvel_ = n_.subscribe("/cmd_vel", 1, &SubsCmdvel::callback_, this);
    }


    void callback_ (const geometry_msgs::Twist::ConstPtr& msg)
    {
        cout << ".x = " << msg->linear.x << "  .z = " << msg->angular.z << endl;
        if (msg->linear.x > 0 )
        {
            if (msg->angular.z > 0)
            {
                cout << "前进 + 左转" << endl;
            }else if (msg->angular.z == 0)
            {
                cout << "前进" << endl;
            }else
            {
                cout << "前进 + 右转" << endl;
            }
        }else if (msg->linear.x == 0)
        {
            if (msg->angular.z > 0)
            {
                cout << "左转" << endl;
            }else if (msg->angular.z == 0)
            {
                cout << "不动" << endl;
            }else
            {
                cout << "右转" << endl;
            }
        }
    }


    private:  
      ros::NodeHandle n_;
      ros::Subscriber sub_cmdvel_;
}; 
      


int main( int argc, char** argv)
{
    ros::init(argc, argv, "SubsCmdvel"); 
    SubsCmdvel sub_;  
    ros::spin();
    
    return 0;
}
