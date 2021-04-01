#!/usr/bin/env python
#  -*- coding:utf-8 -*-
import os  
import yaml
import time
import rospy
import re
import json
import requests
import io
from rospy_message_converter import json_message_converter
#from c import *
#192.168.2.227:21567/trajectory'      #129.211.44.129:10021/trajectory'
pathguiji="/home/x805/catkin_ws/src/indoor_navigation/scripts/guiji"
y6={}
str2=''

def listdir(path, list_name):  
    for file in os.listdir(path):
        if os.path.splitext(file)[1] == '.txt':  
            file_path = os.path.join(path, file) 
     	    if os.path.isdir(file_path):  
            	listdir(file_path, list_name)  
            else:
            	list_name.append(file_path[62:len(file_path)])
    str=''
    a=len(list_name)
    for i in range(a):
        if i == a-1:
            str=str+list_name[i]
        else:
            str=str+list_name[i]+','
    global str2,y6
    if str2!=str:
    	str2=str
	y6 = {
            "filename": str
    	}
	f = open('guiji.json', 'w')
        e = {
	     "code": 9001,
	     "data": y6
        }
	data=json.dumps(e)
	#socketSend(json.dumps(e))
        #socketSend("trajectory---")
        url = 'http://192.168.2.190:10021/trajectory' 
        r = requests.post(url, data=data, headers={'content-type': 'application/json'})
        f.write(data)
        f.close()


if __name__=="__main__":
     rospy.init_node('ros2jsonguiji', anonymous=True)
     try:
         rate = rospy.Rate(5)
         while not rospy.is_shutdown():
             result = []
             listdir(pathguiji,result)
             
             rate.sleep()

     except rospy.ROSInterruptException:
        pass
     #rospy.spin()

