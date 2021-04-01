#!/usr/bin/env python
# -*- coding: utf-8 -*-
#接收服务器json数据，转化后发布话题soc 

# ----------------------------------------
# 语言：Python2.7
# 功能：socket客户端
# 日期：2020-07-23
# ----------------------------------------
import threading
import time, signal
import random
import rospy
import sys, select, termios, tty
from indoor_navigation.msg import soc

defaultencoding = 'utf-8'
if sys.getdefaultencoding() != defaultencoding:
    reload(sys)
    sys.setdefaultencoding(defaultencoding)

from socket import *
import json

HOST = '192.168.2.190'#'192.168.2.227' #HOST = 'juyingtech.tpddns.cn'  # 连接地址xh
PORT = 21567          # 端口xh
BUFSIZ = 1024         # 接收数据大小xh
ADDR = (HOST,PORT)

tcpCliSock = socket(AF_INET,SOCK_STREAM)
tcpCliSock.connect(ADDR)

flag=0

def quit(signum, frame):
    print('所有进程已关闭')
    sys.exit()

def is_json(data):
    try:
        json.loads(data)
    except ValueError:
        return False
    return True

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('soc', soc, queue_size=10) #soc消息类型
    pub_string = soc()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = tcpCliSock.recv(BUFSIZ).decode('utf-8')    #获取服务器返回数据xh
        #if not data:
        print data
            #break
        #jsonData = json.loads(data)
        #code = jsonData["code"]
        #action = jsonData["action"] 
        #code2 = jsonData["code2"]  
        #filename = jsonData["filename"] 
        #yaw = jsonData["yaw"] 
        #pitch = jsonData["pitch"]
        #roll = jsonData["roll"]
        #x1 = jsonData["grid.start.x"]
        #y1 = jsonData["grid.start.y"]
        #x2 = jsonData["grid.end.x"]
        #y2 = jsonData["grid.end.y"]
        #print code
        if is_json(data):
            global flag
            if flag==0:       #跳过第一个无意义指令数据xh
		flag=1
	    else:
            	jsonData = json.loads(data)  #json字符串解码 
            	pub_string.code = int(jsonData["code"])
            	pub_string.action = int(jsonData["action"])
            	pub_string.code2 = int(jsonData["code2"])  
            	pub_string.key = jsonData["key"]
            	pub_string.filename = jsonData["filename"] 
            	pub_string.yaw = int(jsonData["yaw"]) 
            	pub_string.pitch = int(jsonData["pitch"])
                pub_string.roll = int(jsonData["roll"])
                pub_string.x1=int(jsonData["grid"]["start"]["x"])
                pub_string.y1=int(jsonData["grid"]["start"]["y"])
                pub_string.x2=int(jsonData["grid"]["end"]["x"])
                pub_string.y2=int(jsonData["grid"]["end"]["y"])
            	pub.publish(pub_string) #打包发布消息xh
    	rate.sleep()
        
if __name__=="__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


