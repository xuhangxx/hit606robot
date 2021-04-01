#!/usr/bin/env python
#信息发送至显控服务器
import os  
import yaml
import time
import rospy
import re
import json
import io
import requests
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
#from c import *
url = 'http://192.168.2.190:10021/main'

pathguiji="/home/x805/catkin_ws/src/indoor_navigation/scripts/guiji"
y1={}
y2={}
y3={}
y4={}
#y5="{\"poses\":[{\"y\":0,\"x\":0}]}"
y5="{\"poses\":[]}"
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
def MapdataJsonCallBack(mapdata_data_):
      mapdata_json_ = json_message_converter.convert_ros_message_to_json(mapdata_data_)#地图转换为json格式
      global y1
      y1 = mapdata_json_

def gpsJsonCallBack(gps_data_):
      gps_json_ = json_message_converter.convert_ros_message_to_json(gps_data_)#gps数据转换为json格式
      global y2
      y2 = gps_json_

def velJsonCallBack(vel_data_):
      vel_json_ = json_message_converter.convert_ros_message_to_json(vel_data_)#cmdvel2速小车速度信息转换为json格式
      global y3
      y3 = vel_json_

def poseJsonCallBack(pose_data_):
      pose_json_ = json_message_converter.convert_ros_message_to_json(pose_data_)#位姿信息转换为json格式
      global y4
      y4 = pose_json_

def globalJsonCallBack(global_data_):#路径规划得到的路径信息转换为json格式
     global flag
     if flag==1:
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
      #if soc_data_.code == 1:
        #flag=1
        #socflag=0
        #time.sleep(2)
        #f = open('map.json', 'w')
        #item = {
        #   "map3d": y12,
        #}
        #e = {
	#   "code": 2,
        #   "data": item
        #}
        #d=json.dumps(e)
	#socketSend(json.dumps(e))
        #socketSend("end---")
        #time.sleep(3)
        #f.write(d)
        #f.write('ctrmodeback : '+y7+' / '+'status : '+y8+' / '+'map3d : '+y1+' / '+'gps3d : '+y2+' / '+'v : '+y3+' / '+'pos : '+y4+' / '+'list : '+y6+' / '+'pathplan : '+y5+' / '+'panctrmodeback : '+y10+' / '+'panangle : '+y9+' / ')
        #y6=''
        #f.close()
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


def ctrmodeJsonCallBack(ctrmode_data_):#平台控制模式信息（示教模式/循迹模式/自主模式）转换为json格式
      ctrmode_json_ = json_message_converter.convert_ros_message_to_json(ctrmode_data_)
      global y7
      y7 = ctrmode_json_

def statusJsonCallBack(status_data_):#平台状态模式信息（01234）
      status_json_ = json_message_converter.convert_ros_message_to_json(status_data_)
      global y8
      y8 = status_json_

def panangleJsonCallBack(panangle_data_):
      panangle_json_ = json_message_converter.convert_ros_message_to_json(panangle_data_)
      global y9
      y9 = panangle_json_

def panctrmodeJsonCallBack(panctrmode_data_):
      panctrmode_json_ = json_message_converter.convert_ros_message_to_json(panctrmode_data_)
      global y10
      y10 = panctrmode_json_



if __name__=="__main__":
     rospy.init_node('ros2json', anonymous=True)
     rospy.Subscriber("/map_metadata", MapMetaData, MapdataJsonCallBack)
     rospy.Subscriber("/GPS_data", NavSatFix, gpsJsonCallBack)
     rospy.Subscriber("/pose", Odometry, poseJsonCallBack)
     rospy.Subscriber("/cmd_vel2", Twist, velJsonCallBack)
     rospy.Subscriber("/move_base/TrajectoryPlannerROS/global_plan", Path, globalJsonCallBack) 
     rospy.Subscriber("/ctrmode", ctrmode, ctrmodeJsonCallBack)
     rospy.Subscriber("/status", status, statusJsonCallBack)
     rospy.Subscriber("/panangle", panangle, panangleJsonCallBack)
     rospy.Subscriber("/panctrmode", panctrmode, panctrmodeJsonCallBack)
     rospy.Subscriber("/soc", soc, socJsonCallBack)
     try:
         rate = rospy.Rate(10)
         while not rospy.is_shutdown():
             #fs= open("guiji/guiji1.txt","r")   #+soc_data_.filename,"r")
             #a=fs.readlines()
             #global y5
             #str=''
             #i=0
             #ite = Path()
             #for line in a[0:]:
	         #seq = re.compile(" ")
                 ##p= PoseStamped()
                 #ite.poses.append(p)
                 #ite.poses[i].pose.position.x = float(lst[0])
                 ##i=i+1
             #path_json_ = json_message_converter.convert_ros_message_to_json(ite)
             #y5=path_json_
             #fs.close()
             f = open('main.json', 'w')
             item = {
            	"ctrmodeback": y7,
            	"status": y8,
            	"gps3d": y2,
		"map_metadata":y1,
            	"v": y3,
            	"pos": y4,
            	"pathplan": y5,
            	"panctrmodeback": y10,
            	"panangle": y9,
                #"scan":y11,
             }
             e = {
	         "code": 1,
		 "data": item
                  
	     }
             data=json.dumps(e)
             r = requests.post(url, data=data, headers={'content-type': 'application/json'})
	     #socketSend(json.dumps(e))
             #socketSend("end---")
             #time.sleep(3)
             f.write(data)
             #f.write('ctrmodeback : '+y7+' / '+'status : '+y8+' / '+'map3d : '+y1+' / '+'gps3d : '+y2+' / '+'v : '+y3+' / '+'pos : '+y4+' / '+'list : '+y6+' / '+'pathplan : '+y5+' / '+'panctrmodeback : '+y10+' / '+'panangle : '+y9+' / ')
             #y6=''
             f.close()
             
             rate.sleep()

     except rospy.ROSInterruptException:
        pass
     #rospy.spin()

