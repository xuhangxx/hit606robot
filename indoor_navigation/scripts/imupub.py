#!/usr/bin/env python
#通过视觉工控机传输云台imu，在此脚本接收imu信息后打包为imu2话题发布出去
import rospy
import socket
import time
import threading

from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist  
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from indoor_navigation.msg import panimu
from sensor_msgs.msg import Imu

new_socket = socket.socket()        
ip = "192.168.2.220"         
port = 52053                
new_socket.bind((ip, port))       
new_socket.listen(5)


if __name__ == '__main__':
    rospy.init_node('imupub', anonymous=True)
    imu_pub = rospy.Publisher("imu2", panimu, queue_size=10) #消息类型panimu话题名imu2
    pub_string = panimu()#消息内容初始化为pub_string
    try:
        rate = rospy.Rate(50)
	while not rospy.is_shutdown():
            print('connecting')
            new_cil, addr = new_socket.accept() 
            print('connect:', addr)
            res = new_cil.recv(64).decode()
            res_list = res.split(' ')
            if len(res_list) == 0:
                new_cil.close()
                print('disconnect', addr)
                continue
            pub_string.pitch = float(res_list[0])
    	    pub_string.roll = float(res_list[1])
            pub_string.yaw = float(res_list[2])
            imu_pub.publish(pub_string)#发布imu2的消息
            rate.sleep()
           
    except rospy.ROSInterruptException:
        pass
