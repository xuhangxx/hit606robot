#!/usr/bin/env python

import rospy
from indoor_navigation.msg import vision
from geometry_msgs.msg import Twist 
from indoor_navigation.msg import soc

deltax=0
deltay=0
dis=150
flag=1	

def visionCallBack(vision_data_):
    global deltax,deltay,dis
    deltax=vision_data_.x
    deltay=vision_data_.y
    dis=vision_data_.d
    print deltax
    print 'dd ddd'
    print deltay
    print 'sddsdsd'
    print dis

def callback(soc):
    global flag
    if (soc.code==7 and soc.action==3):#视觉跟随模式下执行下述控制指令
        flag=1
    elif (soc.code==1 or soc.code==2 or soc.code==3 or soc.code==4):
        flag=0	


if __name__ == '__main__':
    rospy.init_node('visionvel', anonymous=True)
    rospy.Subscriber("/vision", vision, visionCallBack)
    rospy.Subscriber('soc', soc, callback)
    cmdvel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) 
    try:
        rate = rospy.Rate(50)
	while not rospy.is_shutdown():
            if flag==1:
		cmd_vel=Twist()
	        if deltax<=-50:
                    cmd_vel.linear.y=0
                    cmd_vel.angular.z=0.5
	        elif deltax>=50:
                    cmd_vel.linear.y=0
                    cmd_vel.angular.z=-0.5
	        elif deltax<=-20:
                    cmd_vel.linear.y=0
                    cmd_vel.angular.z=0.01*(-20-deltax)
	        elif deltax>=20:
                    cmd_vel.linear.y=0
                    cmd_vel.angular.z=-0.01*(deltax-20)
	        else:
                    cmd_vel.linear.y=0
                    cmd_vel.angular.z=0

	        if dis >=115:
                    cmd_vel.linear.x=-0.2
	        elif dis >=95:
                    cmd_vel.linear.x=-0.01*(dis-95)
	        elif dis <=65:
                    cmd_vel.linear.x=0.2
	        elif dis <=85:
                    cmd_vel.linear.x=0.01*(85-dis)
	        else:
                    cmd_vel.linear.x=0
                cmdvel_pub.publish(cmd_vel)
            rate.sleep()
           
    except rospy.ROSInterruptException:
        pass

