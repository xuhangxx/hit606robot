#!/usr/bin/env python
import os  
import yaml
import time
import rospy
import re
import json
import io
from rospy_message_converter import json_message_converter
#from sensor_msgs.msg import Imu
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
#from geometry_msgs.msg import PoseStamped
from indoor_navigation.msg import soc
from indoor_navigation.msg import ctrmode
from indoor_navigation.msg import status
from indoor_navigation.msg import panangle
from indoor_navigation.msg import panctrmode
from indoor_navigation.msg import path
from indoor_navigation.msg import pose
from gmapping.msg import map2
from c import *

pathguiji="/home/x805/catkin_ws/src/indoor_navigation/scripts/guiji"
y1={}
y2={}
y3={}
y4={}
y5={}
y6={}
filename=""
socflag=0
flag=0
y7={}
y8={}
y9={}
y10={}
#y11={}
y12={}

def globalJsonCallBack(global_data_):
     global flag
     #if flag==1:
     global y5
     ite = path() 
     y5={}
     a=len(global_data_.poses)
     for i in range(a):
         p= pose()
         ite.poses.append(p)
         ite.poses[i].x = global_data_.poses[i].pose.position.x
         ite.poses[i].y = global_data_.poses[i].pose.position.y
     path_json_ = json_message_converter.convert_ros_message_to_json(ite)
     y5=path_json_
     print y5

def socJsonCallBack(soc_data_):
      global flag,socflag,filename
     
      if soc_data_.code == 5 and soc_data_.action ==1:
	filename="guiji/guiji"+soc_data_.filename
        socflag=1
      elif soc_data_.code == 3 or soc_data_.code == 4:
        socflag=0
        flag=0
      elif soc_data_.code == 2:
        print socflag
        flag=0
        if socflag==1:
      #if soc_data_.status.status == 0:
            fs= open(filename,"r")   #+soc_data_.filename,"r")
            a=fs.readlines()
            global y5
            y5={}
            i=0
            ite = path()
            for line in a[0:]:
	        seq = re.compile(" ")
                lst = seq.split(line.strip())
                p= pose()
                ite.poses.append(p)
                ite.poses[i].x = float(lst[0])
                ite.poses[i].y = float(lst[1])
                i=i+1
            path_json_ = json_message_converter.convert_ros_message_to_json(ite)
            y5=path_json_
            #print y5
            fs.close()

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
    global y6
    y6 = {
            "filename": str
    }
    #print y6


if __name__=="__main__":
     rospy.init_node('ros2jsonpath', anonymous=True)
     rospy.Subscriber("/move_base/TrajectoryPlannerROS/global_plan", Path, globalJsonCallBack) 
     rospy.Subscriber("/soc", soc, socJsonCallBack)
     try:
         rate = rospy.Rate(10)
         while not rospy.is_shutdown():
             result = []
             listdir(pathguiji,result)
             f = open('path.json', 'w')
             e = {
	         "code": 9002,
		 "data": y5
	     }
             d=json.dumps(e)
	     socketSend(json.dumps(e))
             socketSend("end---")
             f.write(d)
             f.close()
             
             rate.sleep()

     except rospy.ROSInterruptException:
        pass
     #rospy.spin()

