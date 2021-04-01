#!/usr/bin/env python
 
import roslaunch
import rospy
import time
import os 
import math
import tf
from indoor_navigation.msg import soc


def callback(soc):
    if (soc.code==-1 and soc.code2==100 and soc.action==1):
	os.system("echo y|rosclean purge")  #清理ROS内存
	os.system("shutdown")

if __name__ == '__main__':
    rospy.init_node('guanji', anonymous=True)
    rospy.Subscriber('soc', soc, callback)
    rospy.spin()
