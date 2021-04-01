#!/usr/bin/env python
#  -*- coding:utf-8 -*-
import os  
import yaml
import time
import rospy
import re
import json
import io
import requests
from rospy_message_converter import json_message_converter
from indoor_navigation.msg import soc
from nav_msgs.msg import OccupancyGrid

url = 'http://192.168.2.190:10021/map'
y1={}
def MapJsonCallBack(map2_data_):
      map2_json_ = json_message_converter.convert_ros_message_to_json(map2_data_)
      global y1
      y1 = map2_json_



if __name__=="__main__":
     rospy.init_node('ros2jsonmap', anonymous=True)
     rospy.Subscriber("/map", OccupancyGrid, MapJsonCallBack)
     try:
         rate = rospy.Rate(1)
         while not rospy.is_shutdown():
            f = open('map.json', 'w')
            item = {
               "map":y1,
            }
            e = {
	       "code": 2,
	       "data": item
                  
	    }
            data=json.dumps(e)
            r = requests.post(url, data=data, headers={'content-type': 'application/json'})
            f.write(data)
            f.close()
             
            rate.sleep()

     except rospy.ROSInterruptException:
        pass
     #rospy.spin()

