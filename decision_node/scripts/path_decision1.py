#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
import re
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from velodyne_datatest.msg import velodyne_obstacle  #自定义话题类型 功能包+消息文件名

k = 0.1  # look forward gain
Lfc = 2.0  # look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s]
L = 1.0  # [m] wheel base of vehicle

a=6378160   #earth long axis
b=6356775   #short
c=(a-b)/a   # e ratio

deltayaw=0.0  #初始标定量，用经纬度计算正北偏航量
v0=0.5      #底盘速度直行控制量
v1=0.2      #底盘速度转弯控制量
w0=0.5       #底盘角速度控制量  
#v1=10      #GPS速度反馈量
class VehicleState:
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v0
        #惯导设备采集x，y，安装在前后轮正中心
        #self.rear_x = self.x- ((L / 2) * math.sin(self.yaw))#后轮x坐标                                            
        #self.rear_y = self.y- ((L / 2) * math.cos(self.yaw)) #后轮y坐标

def update(state,xr, yr, yawr):
 
        state.x = xr
        state.y = yr
        state.yaw = yawr
        #state.v = v1
        #state.rear_x = state.x- ((L / 2) * math.sin(state.yaw))#实时后轮x坐标
        #state.rear_y = state.y- ((L / 2) * math.cos(state.yaw))#实时后轮y坐标
        return state
#ping上一个时刻目标点的序列号 
def pure_pursuit_control(state, cx, cy, pind):  
        ind = calc_target_index(state, cx, cy)
        if pind >= ind:  #ind是下一个去目标点的序号
            ind = pind

        if ind < len(cx):
            tx = cx[ind]
            ty = cy[ind]   #赋予新的点经纬读值（目标点的经纬度）
        else:
            tx = cx[-1]
            ty = cy[-1]        #最后一个轨迹点
            ind = len(cx) - 1  #轨迹到头了
 
        #alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) + (state.yaw*math.pi)/180-0.5*math.pi
        #alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw
        #if state.v < 0: 
            #alpha = math.pi - alpha
        #Lf = k * state.v + Lfc
        #delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
        delta = ((math.pi/2 - math.atan2(ty - state.y, tx - state.x))/math.pi)*180 
        return delta, ind

#计算下一个目标点的序号
def calc_target_index(state, cx, cy):
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))  #当前点与目标点之间最小距离的序号
        L = 0.0
        Lf = k * state.v + Lfc  # aim distance 
        while Lf > L and (ind + 1) < len(cx):  #在轨迹范围和数量内
            dx = cx[ind + 1] - cx[ind]
            dy = cx[ind + 1] - cx[ind]
            L += math.sqrt(dx ** 2 + dy ** 2)  
            ind += 1
        return ind

def LL2UTM(a,f,lat,lon,lonOrigin,FN):
        eSquare=2*f-f*f
        k0=0.9996
        lonTemp=(lon+180)-int((lon+180)/360)*360-180
        latRad = math.radians(lat)
        lonRad = math.radians(lonTemp)
        lonOriginRad = math.radians(lonOrigin)
        e2Square = (eSquare)/(1-eSquare)

        V = a/math.sqrt(1-eSquare*math.sin(latRad)**2)
        T = math.tan(latRad)**2
        C = e2Square*math.cos(latRad)**2
        A = math.cos(latRad)*(lonRad-lonOriginRad)
        M = a*((1-eSquare/4-3*eSquare**2/64-5*eSquare**3/256)*latRad
        -(3*eSquare/8+3*eSquare**2/32+45*eSquare**3/1024)*math.sin(2*latRad)
        +(15*eSquare**2/256+45*eSquare**3/1024)*math.sin(4*latRad)
        -(35*eSquare**3/3072)*math.sin(6*latRad))

        UTMEasting = k0*V*(A+(1-T+C)*A**3/6 + (5-18*T+T**2+72*C-58*e2Square)*A**5/120)#+ 500000.0
        UTMNorthing = k0*(M+V*math.tan(latRad)*(A**2/2+(5-T+9*C+4*C**2)*A**4/24 +(61-58*T+T**2+600*C-330*e2Square)*A**6/720))
        UTMNorthing += FN 
        return UTMEasting,UTMNorthing

 

def callback1(gps_data):
           #rospy.loginfo( "I heard latitude %s", gps_data.latitude)
           #rospy.loginfo( "I heard longitude %s", gps_data.longitude)
           latitude=gps_data.latitude-31.0512537
           longitude=gps_data.longitude-121.3851663
           global xr,yr
           xr,yr=LL2UTM(a,c,latitude,longitude,0,0)
           #rospy.loginfo("xr: %f, yr %f" % (xr, yr)) 
      
def callback2(imu_data):
           global  yawr
           yawr=imu_data.orientation.z
           #rospy.loginfo("yawr: %f" % (yawr)) 

def callback3(obs_):
           global distance,yaw_angle
           distance=obs_.distance
           yaw_angle=obs_.yaw_angle
           #rospy.loginfo("distance: %f,yaw_angle: %f" % (distance,yaw_angle)) 

def path_decison_test():
    rospy.init_node("path_decison_test", anonymous=True)
    #filename='/home/robot/catkin_805/src/decision_node/data/805gps_lat&lon'
    filename='/home/kaanh2019/catkin_ws805/src/decision_node/data/1123gps_lat&lon'
    #filename='/home/zsj/ROS/catkin_apollo805/src/decision_node/data/805gps_lat&lon'
    cx,cy=[],[]
    with open(filename,'r') as f:
     	lines=f.readlines() 
    for line in lines:
	value=[float(s) for s in line.split( )]
        Easting,Northing=LL2UTM(a,c,value[0]-31.0512537,value[1]-121.3851663,0,0)
        #Easting,Northing=LL2UTM(a,c,value[0]-45.7458007,value[1]-126.6261953,0,0)
        #Easting,Northing=LL2UTM(a,c,value[0],value[1],0,0)
   	cx.append(Easting)  #东经变化表示横坐标变化
 	cy.append(Northing) #北纬变化表述纵坐标变化

    global xr,yr,yawr
    xr,yr=LL2UTM(a,c,0,0,0,0)
    yawr=deltayaw  #待标定
    state = VehicleState(x=xr, y= yr, yaw=yawr, v=v0)
    lastIndex = len(cx) - 1
    #x = [state.rear_x]
    #y = [state.rear_y]
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    target_ind = calc_target_index(state, cx, cy)   #目标点序号

    cmdvel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    while lastIndex > target_ind and not rospy.is_shutdown():
        dilta, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        rospy.loginfo("(0~360)dilta: %f" % dilta)
   
        rospy.Subscriber("GPS_data", NavSatFix, callback1)
        rospy.loginfo("xr: %f, yr %f" % (xr, yr)) 

        rospy.Subscriber("IMU_data", Imu, callback2)

        state = update(state,xr,yr,yawr)  #更新值显示和处理求

        if (state.yaw>=0 and state.yaw<=180):
            rospy.loginfo("(0~180)yawr: %f" % (state.yaw)) 
            if dilta-state.yaw>=15.0 :      #目标点在当前点的右侧 执行右转程序
                cmd_vel=Twist()
                cmd_vel.linear.x=v1
                cmd_vel.linear.y=0
                cmd_vel.angular.z=-w0
                rospy.loginfo("turn right  ----------> vx: %f, wz %f" % (v1, -w0)) 
                cmdvel_pub.publish(cmd_vel)
            elif dilta-state.yaw<=-15.0 :     
                cmd_vel=Twist()
                cmd_vel.linear.x=v1
                cmd_vel.linear.y=0
                cmd_vel.angular.z=w0
                rospy.loginfo("turn lift <----------- vx: %f, wz %f" % (v1, w0))
                cmdvel_pub.publish(cmd_vel)
            else :
                cmd_vel=Twist()
                cmd_vel.linear.x=v0
                cmd_vel.linear.y=0
                cmd_vel.angular.z=0
                rospy.loginfo("go straight |||||||||| vx: %f, wz %f" % (v0, 0))
                cmdvel_pub.publish(cmd_vel)
        else:
            plus_yawr = state.yaw+360
            rospy.loginfo("(180~360)yawr: %f" % (plus_yawr)) 
            if dilta-plus_yawr>=15.0 :      #目标点在当前点的右侧 执行右转程序
                cmd_vel=Twist()
                cmd_vel.linear.x=v1
                cmd_vel.linear.y=0
                cmd_vel.angular.z=-w0
                rospy.loginfo("turn right  ----------> vx: %f, wz %f" % (v1, -w0)) 
                cmdvel_pub.publish(cmd_vel)
            elif dilta-plus_yawr<=-15.0 :     
                cmd_vel=Twist()
                cmd_vel.linear.x=v1
                cmd_vel.linear.y=0
                cmd_vel.angular.z=w0
                rospy.loginfo("turn lift <----------- vx: %f, wz %f" % (v1, w0))
                cmdvel_pub.publish(cmd_vel)
            else :
                cmd_vel=Twist()
                cmd_vel.linear.x=v0
                cmd_vel.linear.y=0
                cmd_vel.angular.z=0
                rospy.loginfo("go straight |||||||||| vx: %f, wz %f" % (v0, 0))
                cmdvel_pub.publish(cmd_vel)




        #rospy.Subscriber("Velodyne_obstacle", velodyne_obstacle, callback3)
        #rospy.loginfo("distance: %f,yaw_angle: %f" % (distance,yaw_angle)) 
        
        #state = update(state,xr,yr,yawr)  #更新值显示和处理求偏转角度

        #新增 目标相对当前位置的角度 delta_xx 注意：角度值正 目标在右
       
        

        
            

        


        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(cx[target_ind], cy[target_ind], "go", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
        plt.pause(0.001)
    rospy.spin()


if __name__ == '__main__':
    try:  
        print("Starting path_decison_test node.")
        path_decison_test()
       
    except KeyboardInterrupt:
        print("Shutting down path_decisone_test node.")