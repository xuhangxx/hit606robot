/* 该节点用来发送户外里程计
 * @XH
 * *******************************/
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>  //轨迹
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <tf/transform_broadcaster.h>
#include <math.h>

using namespace std;
int flag=1;
int first=0;
int ct=0;
const double pi = 3.1415926535898;
double a=6378160;   
double b=6356775;  
double c=(a-b)/a; 
vector<double> current_utm_x_y;
double origin_x;
double origin_y;
geometry_msgs::Quaternion quat;
int p;
vector<double> LL2UTM_respective(double lon, double lat)
{
    double e=2*c-c*c;
    double ee=e/(1-e);
    double lonRad=(lon+180)-int((lon+180)/360)*360-180;
    double latRad = lat*pi/180;
    lonRad = lonRad*pi/180;

    double V = a/sqrt(1-e*pow(sin(latRad),2));
    double T = pow(tan(latRad),2);
    double C = ee*pow(cos(latRad),2);
    double A = cos(latRad)*lonRad;
    double M = a*((1-e/4-3*e*e/64-5*e*e*e/256)*latRad-(3*e/8+3*e*e/32+45*e*e*e/1024)*sin(2*latRad)
    +(15*e*e/256+45*e*e*e/1024)*sin(4*latRad)-(35*e*e*e/3072)*sin(6*latRad));
 
    vector<double> UTM;
    UTM[0] = 0.9996*V*(A+(1-T+C)*A*A*A/6 + (5-18*T+T*T+72*C-58*ee)*pow(A,5)/120);//+ 500000.0
    UTM[1] = 0.9996*(M+V*tan(latRad)*(A*A/2+(5-T+9*C+4*C*C)*pow(A,4)/24 +(61-58*T+T*T+600*C-330*ee)*pow(A,6)/720));


    return UTM;
}

void imu_callback2 (const sensor_msgs::ImuConstPtr& msg)
{
        quat = tf::createQuaternionMsgFromRollPitchYaw(msg->orientation.x, msg->orientation.y, msg->orientation.z);
        p=msg->orientation.w;
        if (p>20)
        {
            flag=0;
            if(!first)
            {
                first=1;
            }
        }
        else
        {
            flag=1;
        }
}

void gps_callback (const sensor_msgs::NavSatFixConstPtr& msg)
{
    if(first==1)
    {
        origin_x=msg->longitude;
        origin_y=msg->latitude;
        first=2;
    }
    if(first!=0)
    {
        current_utm_x_y = LL2UTM_respective(msg->longitude-origin_x, msg->latitude-origin_y);
    }
}



int main (int argc, char** argv)
{
    ros::init(argc, argv, "open_node");
    ros::NodeHandle nh;

    ros::Subscriber GPS_sub = nh.subscribe("GPS_data",1,gps_callback);
	ros::Subscriber imu_sub = nh.subscribe("IMU_data",1,imu_callback);
	
    ros::Rate loop_rate(10);

    while(ros::ok())
	{
        
        static tf::TransformBroadcaster odom_broadcaster;
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.seq = ct;
        odom_trans.header.frame_id = "/odom";
        odom_trans.child_frame_id = "/base_footprint";
        odom_trans.transform.translation.x = current_utm_x_y[0];
        odom_trans.transform.translation.y = current_utm_x_y[1];
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation=quat;
        if (p>20)
        {   
            odom_broadcaster.sendTransform(odom_trans);
            ct++;
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
}


