#!/usr/bin/env python

import rospy
import socket
import time
import threading

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from indoor_navigation.msg import vision
from sensor_msgs.msg import Imu

new_socket = socket.socket()        
ip = "192.168.2.220"         
port = 52052               
new_socket.bind((ip, port))       
new_socket.listen(5)


if __name__ == '__main__':
    rospy.init_node('visionpub', anonymous=True)
    vision_pub = rospy.Publisher("vision", vision, queue_size=10) 
    pub_string = vision()
    try:
        rate = rospy.Rate(50)
	while not rospy.is_shutdown():
            print('connecting')
            new_cil, addr = new_socket.accept() 
            print('connect:', addr)
            res = new_cil.recv(64).decode()
            if len(res)==0 or res.isspace(): 
                continue
            res_list = res.split(' ')
            pub_string.x = float(res_list[0])
    	    pub_string.y = float(res_list[1])
            pub_string.d = float(res_list[2])
            vision_pub.publish(pub_string)
            rate.sleep()
           
    except rospy.ROSInterruptException:
        pass
