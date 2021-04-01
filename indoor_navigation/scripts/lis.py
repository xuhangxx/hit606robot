#!/usr/bin/env python
#读取地图并/执行保存地图的命令/删除所选轨迹/将显控界面点选的目标点发送成movebase的目标点/将小车真实速度转换为显控显示速度
import rospy
import os 
import math
import tf
from nav_msgs.msg import MapMetaData
from indoor_navigation.msg import soc
from move_base_msgs.msg import MoveBaseActionGoal
from indoor_navigation.msg import ctrmode
from indoor_navigation.msg import status
from geometry_msgs.msg import Twist

r=0.01
dx=-3.0
dy=-3.0
width=100
height=100
ctrmodeback=7
def mapcallback(mapinfo):
    global r,dx,dy,width,height
    r=mapinfo.resolution
    dx=mapinfo.origin.position.x
    dy=mapinfo.origin.position.y
    width=mapinfo.width
    height=mapinfo.height

def callback(soc):
    global ctrmodeback
    if (soc.code==5 and soc.action==2):#接收到显控保存地图指令，
        os.system("python /home/x805/catkin_ws/src/indoor_navigation/scripts/savemap.py")#执行保存地图的python脚本
    elif (soc.code==5 and soc.action==3):#接收到显控删除指定循迹列表的指令
        path = '/home/x805/catkin_ws/src/indoor_navigation/scripts/guiji/guiji' + soc.filename#所删除轨迹的路径
        if os.path.exists(path):
            os.remove(path)#执行删除轨迹
    #elif (soc.code==5 and soc.action==4):
        #path = '/home/kaanh/catkin_ws805/src/indoor_navigation/maps/chushi.txt'
        #if os.path.exists(path):
            #os.remove(path)
    elif (soc.code==6 and soc.action==1):#接收到发送目标点的指令，得到目标点信息用于路径规划
        global r,dx,dy
        x1=soc.x1*r+dx
        y1=soc.y1*r+dy
        angle=math.atan2((soc.x1-soc.x2),(soc.y1-soc.y2))
        q = tf.transformations.quaternion_from_euler(0, 0, angle)#欧拉角转四元数
        print angle
        pub_string.goal.target_pose.header.frame_id="map"
        pub_string.goal.target_pose.pose.position.x=(width-soc.y1)*r+dx
        pub_string.goal.target_pose.pose.position.y=(height-soc.x1)*r+dy
        pub_string.goal.target_pose.pose.position.z=0
        pub_string.goal.target_pose.pose.orientation.x=q[0]
        pub_string.goal.target_pose.pose.orientation.y=q[1]
        pub_string.goal.target_pose.pose.orientation.z=q[2]
        pub_string.goal.target_pose.pose.orientation.w=q[3]
        pub.publish(pub_string)

def velCallBack(vel_data_):#将小车真实速度转换为显控显示速度并进行发布
      global pub_vel
      pub_vel=vel_data_

if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('map_metadata',MapMetaData, mapcallback)
    rospy.Subscriber('soc', soc, callback)
    rospy.Subscriber("/cmd_vel", Twist, velCallBack)
    pub = rospy.Publisher('move_base/goal', MoveBaseActionGoal, queue_size=10)
    pubctr = rospy.Publisher('/cmd_vel2', Twist, queue_size=10)
    pub_vel=Twist()
    pub_string = MoveBaseActionGoal()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pubctr.publish(pub_vel)
	pub_vel=Twist()
    	rate.sleep()
