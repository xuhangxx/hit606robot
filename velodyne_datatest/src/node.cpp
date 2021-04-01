#include<ros/ros.h>
#include<velodyne_datatest/velodyne_datatest.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velodyne_datatest_node");
    ros::NodeHandle nh;

    velodyne_datatest::VelodyneDataTest n(nh);
    


    ros::spin();
    return 0;
}



   

