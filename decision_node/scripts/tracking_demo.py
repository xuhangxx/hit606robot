#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
 
k = 0.1  # look forward gain
Lfc = 2.0  # look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s]
L = 1.0  # [m] wheel base of vehicle
a=6378160   #earth long axis
b=6356775   #short
c=(a-b)/a   # e ratio
 
 
old_nearest_point_index = None
show_animation = True
 
 
class State:
 
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))
 
 
def update(state, a, delta):
 
    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.v = state.v + a * dt
    state.rear_x = state.x - ((L / 2) * math.cos(state.yaw))
    state.rear_y = state.y - ((L / 2) * math.sin(state.yaw))
 
    return state
 
 
def PIDControl(target, current):
    a = Kp * (target - current)
 
    return a
 
 
def pure_pursuit_control(state, cx, cy, pind):
 
    ind = calc_target_index(state, cx, cy)
 
    if pind >= ind:
        ind = pind
 
    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1
 
    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
 
    Lf = k * state.v + Lfc
 
    delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
 
    return delta, ind
 
def calc_distance(state, point_x, point_y):
 
    dx = state.rear_x - point_x
    dy = state.rear_y - point_y
    return math.sqrt(dx ** 2 + dy ** 2)

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
 
def calc_target_index(state, cx, cy):
 
    global old_nearest_point_index
 
    if old_nearest_point_index is None:
        # search nearest point index
        dx = [state.rear_x - icx for icx in cx]
        dy = [state.rear_y - icy for icy in cy]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        ind = d.index(min(d))
        old_nearest_point_index = ind
 
    else:
        ind = old_nearest_point_index
        distance_this_index = calc_distance(state, cx[ind], cy[ind])
 
        while True:
            ind = ind + 1 if (ind + 1) < len(cx) else ind
            print("ind=%d",ind);
            distance_next_index = calc_distance(state, cx[ind], cy[ind])
            if distance_this_index < distance_next_index:
                break
            distance_this_index = distance_next_index
        old_nearest_point_index = ind
 
 
    L = 0.0
 
    Lf = k * state.v + Lfc
    Lf=Lfc;
 
    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind] - state.rear_x
        dy = cy[ind] - state.rear_y
        L = math.sqrt(dx ** 2 + dy ** 2)
        ind += 1
 
    return ind
 
 
def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """
 
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)
 
 
def main():
    #filename = '/home/robot/catkin_805/src/decision_node/data/805gps_lat&lon'
    filename='/home/kaanh2019/catkin_ws805/src/decision_node/data/20191208gps_data'
    cx,cy= [],[]
    with open(filename, 'r') as f:
        lines = f.readlines()
        for line in lines:
            value = [float(s) for s in line.split(" ")]
            #Easting,Northing=LL2UTM(a,c,value[0]-45.7458007,value[1]-126.6261953,0,0)
            Easting,Northing=LL2UTM(a,c,value[0]-31.0512537,value[1]-121.3851663,0,0)
            cx.append(Easting)
            cy.append(Northing) 

 
    target_speed = 10.0 /14.4 # [m/s]
 
    T = 100.0  # max simulation time
 
    # initial state
    state = State(x=0, y=0, yaw=0.0, v=0.0)
 
    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_ind = calc_target_index(state, cx, cy)
 
    while T >= time and lastIndex > target_ind:
        ai = PIDControl(target_speed, state.v)
        di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
        state = update(state, ai, di)
 
        time = time + dt
 
        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
 
        if show_animation:  # pragma: no cover
            plt.cla()
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)
 
    # Test
    assert lastIndex >= target_ind, "Cannot goal"
 
    if show_animation:  # pragma: no cover
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
 
        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()
 
 
if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()

