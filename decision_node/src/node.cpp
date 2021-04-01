#include <ros/ros.h>
#include "decision_node/decision_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision_node");

    ros::NodeHandle nh;

    decision_node::DecisionNodeTest n(nh);
   
    ros::spin();//不用循环直接加载回调函数
    return 0;
}

