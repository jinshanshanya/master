#!/usr/bin/env python
# rosrun DecisionControl PathPlanningNode.py
# license @Global Carrier Strike Group
import rospy
from InsMsg.msg import ins_p2
from VehicleMsg.msg import adm_cmd
from VehicleMsg.msg import cdm_cmd
from functionalV1 import *
from math import *
from sympy import *
#from statemachine import StateMachine, State
# import time
from std_msgs.msg import String
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
 

pi = np.pi
N = len(path_waypoint)
c_x = list()
c_y = list()
delta_head = list()
B0 = way2[0, 0]
L0 = way2[0, 1]
t = list()

# pub = rospy.Publisher('PathPlaningNoddPyTopic', vehicle_download, queue_size=100)
pub = rospy.Publisher('AdmCmdMsg', adm_cmd, queue_size=100)

# avoid_point = np.empty(shape=[0, 3])  # Obstacle avoidance point


def PathPlaningNode():
    global waypoint
    rospy.Subscriber("insMsg", ins_p2, InsCallback)
    rospy.Subscriber("CdmCmdMsg",cdm_cmd , VehCallback)

    rospy.init_node('PathPlanningNodeStaticV1Py', anonymous=True)
    r = rospy.Rate(100)  # 100hz

    while not rospy.is_shutdown():
        r.sleep()
    way_path_x = waypoint[:, 0]
    way_path_y = waypoint[:, 1]
    #print("waypoint_x:",waypoint[:, 0])
    plt.plot(way_path_x, way_path_y, 'b*')
    plt.plot(c_x, c_y, 'r')
    # plt.plot(delta_head, t, 'g')
    plt.show()


# the following is DisObjCallback signals
# ======================================
d =100
v_front = 0
# ======================================
# the following is VehCallback
# ======================================
Veh_Spd = 0.0
drive_mode = 0
vehicle_length = 4.5
Veh_Mass=0

# ======================================
# the following is insCallback
# ======================================
T = 0
L = 2.0
earth_current_x = 0
earth_current_y = 0
heading = 0

#================================
#the following is main
#=================================
current_ind = None
# ======================================
# end of initialization
# ======================================
# def PathCallback(data):
#    global avoid_point
#    for i in range(20):
#        x = data.path[i].x
#        y = data.path[i].y
#        heading = data.path[i].heading
#       avoid_point = np.append(avoid_point, [[x, y, heading]], axis=0)
#   print('avoid_point',avoid_point)

def VehCallback(data):
    global Veh_Spd
    global Veh_Mass
    global DriveModeFd

    Veh_Spd = data.Veh_Spd
    Veh_Mass =data.Veh_Mass
    drive_mode = data.DriveModeFd



def InsCallback(data):
    global earth_current_x
    global earth_current_y
    global heading
    global current_ind
    global waypoint
    global B0
    global L0
    global d
    global T
    global L
    earth_current_x = data.Lat
    earth_current_y = data.Lon
    heading = data.Heading


    if T >= 0 :
        v_des, control_steering_angle,current_x, current_y ,waypoint= main()
        
    c_x.append(current_x)
    c_y.append(current_y)
    print('control_steering_angle is', control_steering_angle)
   

    msg = adm_cmd()

    # ------------------------------
    if T < 100:
        msg.Amble_Brake = 0  # 0
        msg.Gear = 0
        msg.GPS1_Curvature_cmd = 32128  #no Define signal
        msg.AccPed_enable = 0
        msg.AccPed = 0
    if T >= 100 and T < 200:
        msg.Amble_Brake = 0  # 0
        msg.Gear = 1             #no Define signal
        msg.GPS1_Curvature_cmd = 32128  #no Define signal
        msg.AccPed_enable = 1
        msg.AccPed = 0
    if T >= 200 and T < 250:  # gear = 4
        msg.Amble_Brake = 0  # 0
        msg.Gear = 1             #no Define signal
        msg.GPS1_Curvature_cmd = 32128  #no Define signal
        msg.AccPed_enable = 1
        msg.AccPed = 0
    if T >= 250:
        msg.Amble_Brake = 0  # 0
        msg.Gear = 1             #no Define signal
        msg.GPS1_Curvature_cmd = 32128 + 12.82*control_steering_angle  #no Define signal
        msg.AccPed_enable = 1
        msg.AccPed = 24
        print("AccPed",msg.AccPed)

	
        if current_ind >= len(waypoint) - 50 and current_ind < len(waypoint):
            msg.AccPed_enable = 0
            msg.AccPed = 0
            msg.Amble_Brake = 50  # 0
            msg.Gear = 0      #no Define signal

        if current_ind >= len(waypoint) - 20 and v == 0:
            msg.AccPed_enable = 0
            msg.AccPed = 0
            msg.Amble_Brake = 50  # 0
            msg.Gear = 0      #no Define signal

    # rospy.loginfo(msg)
    pub.publish(msg)
    print("T:", T)
    T = T + 1
	
	
def main():
    
    global earth_current_x
    global earth_current_y
    global heading
    global current_ind
    global waypoint
    global B0
    global L0
    global d
    global vehicle_length
    global L

    if drive_mode == 0:
        print('please switch the drive mode')
    if drive_mode == 1:
        # ===================================
        #starting of each loop
        # 1, you should first find your Cartesian location current_x, and current_y in unit 'm'
        current_x, current_y = transformation(earth_current_x, earth_current_y, B0, L0)

        # 2, then find you current index, you current offset, and a set of look forward points
        look_forward_point, current_ind, lateral_offset = ind_route_generation(current_x, current_y, current_ind)

        # 3, check whether you should make bezier curve avoidance
        waypoint = avoidance_judgment(current_ind,waypoint_x,waypoint_y,waypoint_heading,v_front,d,L)
	
        # 4, update new current index, lateral_offset, and look forward points if the original reference line 		is updated
        look_forward_point, current_ind, lateral_offset = ind_route_generation(current_x, current_y, 		current_ind)

        # 5, transform loo_forward_point from   global  to Frenet
        look_forward_point = global_to_frenet(current_x, current_y, heading, look_forward_point)

        # 6, find your prediction point, which is denoted by x_ref, and y_ref
        x_ref, y_ref = breakthrough(current_x, current_y, look_forward_point, lateral_offset)

        # 7, calculate longitudinal desired velocity
        v_des = control_v(d)

        # 8, calculate lateral desired steering angle
        control_steering_angle = control_steering(x_ref, y_ref,vehicle_length)
        # ending of each loop
        # ===================================
<<<<<<< HEAD
        return v_des, control_steering_angle,current_x, current_y ,waypoint
=======
    	return v_des, control_steering_angle,current_x, current_y ,waypoint
>>>>>>> d81d4ed221e30437b857fdcd6b8761c2b0cb6ab0
       
    #end if
#end main

if __name__ == '__main__':
    try:
        PathPlaningNode()
    except rospy.ROSInterruptException:
        pass
    #end try
#end if
