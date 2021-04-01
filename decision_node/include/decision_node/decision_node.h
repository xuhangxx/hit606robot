#ifndef DECISION_NODE_H
#define DECISION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>  
#include <sensor_msgs/NavSatFix.h>    //GPS
#include <geometry_msgs/Twist.h>

namespace  decision_node
{

   class DecisionNodeTest
  {
   public:

      DecisionNodeTest(ros::NodeHandle &nh);    

    private:
      void Velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
      void IMUdata_callback(const sensor_msgs::Imu& imu_data);
      void GPSdata_callback(const sensor_msgs::NavSatFix& gps_data);  //回调函数声明
      void CmdVeldata_callback(const geometry_msgs::Twist& cmdvel_data);  //回调函数声明

      void cmd_vel_publish();

      ros::NodeHandle nh_;
      ros::Subscriber Velodyne_sub;
      ros::Subscriber IMU_sub;
      ros::Subscriber GPS_sub;
      ros::Subscriber CmdVel_sub;

      ros::Publisher cmd_vel_pub;
     

  };

}  // namespace decision_node

#endif  // DECISION_NODE_H