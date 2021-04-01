#ifndef VELODYNE_DATATEST_H
#define VELODYNE_DATATEST_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>


namespace  velodyne_datatest
{

   class VelodyneDataTest
  {
    public:
      VelodyneDataTest(ros::NodeHandle &nh);    

    private:
      void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);  //回调函数声明

      ros::NodeHandle nh_;
      ros::Subscriber subLaserCloud;
      ros::Publisher pubLaserScan;
      ros::Publisher pubLaserCloud;
      ros::Publisher  pubVelodyneObstacle;

  };

}  // namespace velodyne_datatest

#endif  // VELODYNE_DATATEST_H