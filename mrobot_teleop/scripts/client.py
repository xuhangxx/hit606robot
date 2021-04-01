#!/usr/bin/env python
# -*- coding: utf-8 -*-


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
from geometry_msgs.msg import Twist
msg = """
Control mrobot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop

CTRL-C to quit
"""

#defaultencoding = 'utf-8'
#if sys.getdefaultencoding() != defaultencoding:
    #reload(sys)
    #sys.setdefaultencoding(defaultencoding)

from socket import *
import json
from mrobot_teleop2 import *
from indoor_navigation.msg import soc

HOST = '192.168.2.190'  # 连接地址
PORT = 21567        # 端口
BUFSIZ = 1024        # 接收数据大小
ADDR = (HOST,PORT)

tcpCliSock = socket(AF_INET,SOCK_STREAM)
tcpCliSock.connect(ADDR)

def quit(signum, frame):
    print('所有进程已关闭')
    sys.exit()

#def is_json(data):
#    try:
 #       json.loads(data)
 #   except ValueError:
 #       return False
  #  return True

def run1(x,th,status,count,control_speed,control_turn):
    #try:
    while(1):
        global flag
        if flag==1:
            #data = tcpCliSock.recv(BUFSIZ).decode('utf-8')    #获取服务器返回数据
            #if not data:
                #break
            #if is_json(data):
                #jsonData = json.loads(data)
                #code = jsonData["code"]
                #key = jsonData["key"]
                global code
                if code == 8:
		    global twist,target_speed,target_turn,key,speed,turn
                    twist,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn = controlcar(key,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn)
	            pub.publish(twist)
                    code=0
		    #print twist.linear.x
                #if code == -1:
	            #twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                    #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
	            #pub.publish(twist)
                    #print("socket连接成功...")

    #except Exception as e:
      #  print repr(e)

    #finally:
     #   twist = Twist()
      #  twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
       # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
       # pub.publish(twist)


def run2(x,th,status,count,control_speed,control_turn):
    #print 'sss'
    #try:
    while(1):
        global twist,target_speed,target_turn,key,flag,speed,turn
        #print 'sss2'
        if flag==1:
            #print 'sss2'
	    time.sleep(0.1)
	    global twist,target_speed,target_turn,key
	    twist,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn = controlcar2(key,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn)
    	    pub.publish(twist)
        else:
            target_speed = 0
            target_turn = 0
            key='k'
            speed = .2
            turn = 1
	    #print twist.linear.x
    #except Exception as e:
      #  print repr(e)

    #finally:
       # twist = Twist()
       # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
       # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        #pub.publish(twist)

def callback(soc):
    global flag,key,code
    if (soc.code==3 or soc.code==4):
        flag=1
    elif (soc.code==1 or soc.code==2):
        flag=0
    elif (soc.code==8 and soc.action==1):
	key=soc.key
        code=8
    elif (soc.code==7 and soc.action==3):
        flag=0
	

if __name__=="__main__":
    #settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('mrobot_teleop')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
    rospy.Subscriber('soc', soc, callback)
    speed = .2
    turn = 1
    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    twist = Twist()
    flag = 1	
    code=0
    key = 'k'
    #print msg
    #print vels(speed,turn)
    try:
	signal.signal(signal.SIGINT, quit)
    	signal.signal(signal.SIGTERM, quit)
    	t1 = threading.Thread(target=run1, args=(x,th,status,count,target_speed,control_turn,))
    	t2 = threading.Thread(target=run2, args=(x,th,status,count,target_speed,control_turn,))
    	t1.setDaemon(True)
	t2.setDaemon(True)
    	t1.start()
    	t2.start()
        rate = rospy.Rate(10)
        #print 'ssssdsdsdsdsd'
        while not rospy.is_shutdown():
    	#while True:
            #print 'sss'
            rate.sleep()
            #pass
    except Exception as exc:
        print(exc)
    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
# tcpCliSock.close()


