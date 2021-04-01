#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
           }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def controlcar(key,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn):
        # 运动控制方向键（1：正方向，-1负方向）
    if key in moveBindings.keys():
        x = moveBindings[key][0]
        th = moveBindings[key][1]
        count = 0
        # 速度修改键
    elif key in speedBindings.keys():
        speed = speed * speedBindings[key][0]  # 线速度增加0.1倍
        turn = turn * speedBindings[key][1]    # 角速度增加0.1倍
        count = 0

        print vels(speed,turn)
        if (status == 14):
            print msg
        status = (status + 1) % 15
        # 停止键
    elif key == ' ' or key == 'k' :
        x = 0
        th = 0
        control_speed = 0
        control_turn = 0
    else:
        count = count + 1
        if count > 4:
            x = 0
            th = 0
        if (key == '\x03'):
            return

        # 目标速度=速度值*方向值
    target_speed = speed * x
    target_turn = turn * th
    #print target_speed
    # 速度限位，防止速度增减过快
    if target_speed > control_speed:
        control_speed = min( target_speed, control_speed + 0.02 )
    elif target_speed < control_speed:
        control_speed = max( target_speed, control_speed - 0.02 )
    else:
        control_speed = target_speed

    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.1 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.1 )
    else:
        control_turn = target_turn

        # 创建并发布twist消息
    twist = Twist()
    twist.linear.x = control_speed; 
    twist.linear.y = 0; 
    twist.linear.z = 0
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = control_turn
    return twist,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn

def controlcar2(key,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn):
        # 运动控制方向键（1：正方向，-1负方向）
    if key in moveBindings.keys():
        x = moveBindings[key][0]
        th = moveBindings[key][1]
        count = 0
        # 速度修改键
    elif key == ' ' or key == 'k' :
        x = 0
        th = 0
        control_speed = 0
        control_turn = 0
    else:
        count = count + 1
        if count > 4:
            x = 0
            th = 0
        if (key == '\x03'):
            return

        # 目标速度=速度值*方向值
    # 速度限位，防止速度增减过快
    if target_speed > control_speed:
        control_speed = min( target_speed, control_speed + 0.02 )
    elif target_speed < control_speed:
        control_speed = max( target_speed, control_speed - 0.02 )
    else:
        control_speed = target_speed

    if target_turn > control_turn:
        control_turn = min( target_turn, control_turn + 0.1 )
    elif target_turn < control_turn:
        control_turn = max( target_turn, control_turn - 0.1 )
    else:
        control_turn = target_turn

        # 创建并发布twist消息
    twist = Twist()
    twist.linear.x = control_speed; 
    #print target_speed
    twist.linear.y = 0; 
    twist.linear.z = 0
    twist.angular.x = 0; 
    twist.angular.y = 0; 
    twist.angular.z = control_turn
    return twist,speed,turn,x,th,status,count,target_speed,target_turn,control_speed,control_turn
