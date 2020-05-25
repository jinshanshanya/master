#!/usr/bin/env python
# rosrun DecisionControl PathPlanningNode.py
# license @Global Carrier Strike Group
import rospy
from InsMsg.msg import ins_p2
from VehicleMsg.msg import adm_cmd
from VehicleMsg.msg import cdm_cmd
from math import *
from sympy import *
from std_msgs.msg import String
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from functionalV1_8 import *
 

N = len(path_waypoint)
c_x = list()
c_y = list()
c_x_F = list()
c_y_F= list()
steering_angle = list()  
lateral_dis = list()

B0 = way2[0, 0]
L0 = way2[0, 1]
t = list()

pub = rospy.Publisher('AdmCmdMsg', adm_cmd, queue_size=100)

def PathPlaningNode():
    global waypoint
    rospy.Subscriber("insMsg", ins_p2, InsCallback)
    rospy.Subscriber("CdmCmdMsg",cdm_cmd , VehCallback)
    rospy.init_node('PathPlanningNodeStaticV1_8Py', anonymous=True)
    r = rospy.Rate(50)  # 100hz

    while not rospy.is_shutdown():
        r.sleep()
    way_path_x = waypoint[:, 0]
    way_path_y = waypoint[:, 1]

    file = os.getcwd() + '\\Steer_angle.csv'
    data = pd.DataFrame({"steer_angle":steering_angle ,"lateral_off":lateral_dis ,"c_y":c_y ,"c_x":c_x})
    data.to_csv(file,index=False)

    plt.figure(1)
    plt.plot(way_path_x, way_path_y, "b",label='waypoint_path')
    plt.plot(c_x, c_y, 'r',label='current_path_rear_axis')
    plt.plot(c_x_F, c_y_F, 'g',label='current_path_front_axis')
    plt.legend() 
    plt.grid(True) 

    plt.figure(2)
    plt.plot(steering_angle,'g',label='steer_angle')
    plt.legend() 
    plt.grid(True) 

    plt.figure(3)
    plt.plot(lateral_dis,'g',label='lateral_offset_front_axis')
    plt.legend() 
    plt.grid(True)

    plt.show()

# the following is DisObjCallback signals
# ======================================
d = 100
distance_lat = 0
v_front = 0
target_id = 0
obstacle_point = np.empty(shape=[0, 1])
# ======================================
# the following is VehCallback
# ======================================
v = 0.0
gear = 0
drive_mode = 0
vehicle_length = 4.5
# ======================================
# the following is insCallback
# ======================================
T = 0
L = 2.0
earth_current_x = 0
earth_current_y = 0
heading = 0
id = 1999999
complete = 1
#================================
#the following is main
#=================================
current_ind = None
current_ind_F = None
# ======================================
# end of initialization
# ======================================
    

def VehCallback(data):
    global v
    global Veh_Mass
    global DriveModeFd
    global TCM_GearFd
    v = data.Veh_Spd
    Veh_Mass =data.Veh_Mass
    drive_mode = data.DriveModeFd
    TCM_GearFd = data.TCM_GearFd

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
    global v
    earth_current_x = data.Lat
    earth_current_y = data.Lon
    heading = data.Heading
def InsCallback(data):
    global earth_current_x
    global earth_current_y
    global heading
    global Time
    global current_ind
    global current_ind_F
    global waypoint
    global B0
    global L0
    global d
    global T
    global L
    earth_current_x = data.Lat
    earth_current_y = data.Lon
    heading = data.Heading
    Time = data.Time

    if T >= 0 :
        v_des, control_steering_angle,current_x, current_y ,waypoint ,lateral_offset,current_x_F ,current_y_F= main()
        
    c_x.append(current_x)
    c_y.append(current_y)
    c_x_F.append(current_x_F)
    c_y_F.append(current_y_F)
    steering_angle.append(control_steering_angle)   
    #lateral_offset= lateral_offset.astype(type('float', (float,), {}))  
    lateral_dis.append(lateral_offset)
    #print('obstacle distance is:', d)
    print('control_steering_angle is', control_steering_angle)
   

    msg = adm_cmd()

    controller = PositionPID()
    target_v = 5
    kp = 1.5
    ki = 0.75
    kd = 0.12
    controller.setTarget(target_v,kp,ki,kd)
    throat = max(0,min(100,controller.updata(v)))

    print("throat",throat)

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
    global current_ind_F
    global waypoint
    global B0
    global L0
    global d
    global vehicle_length
    global L
    global complete

    #if drive_mode == 0:
        #print('please switch the drive mode')
    if drive_mode == 0:
        # ===================================
        #starting of each loop
        # 1, you should first find your Cartesian location current_x, and current_y in unit 'm'
        current_x, current_y = transformation(earth_current_x, earth_current_y, B0, L0)

        # 2, then find you current index, you current offset, and a set of look forward points
        look_forward_point, current_ind, lateral_offset ,current_x_F ,current_y_F= ind_route_generation(current_x, current_y, current_ind ,heading ,current_ind_F)

        # 3, check whether you should make bezier curve avoidance
        waypoint, end_point = avoidance_judgment(current_ind,waypoint_x,waypoint_y,waypoint_heading,v_front,d,L)
        # 3.1, update complete, which is a trigger that show whether the changing lane mission is completed. initial value =1(complete), when in mission, value = 0
        complete = update_complete(current_ind, end_point)
        # 4, update new current index, lateral_offset, and look forward points if the original reference line 		is updated
        look_forward_point, current_ind, lateral_offset ,current_x_F ,current_y_F= ind_route_generation(current_x, current_y, current_ind ,heading ,current_ind_F)

        # 5, transform look_forward_point from  global to Frenet
        look_forward_point = global_to_frenet(current_x, current_y, heading, look_forward_point)

        # 6, find your prediction point, which is denoted by x_ref, and y_ref
        x_ref, y_ref = breakthrough(current_x, current_y, look_forward_point, lateral_offset)

        # 7, calculate longitudinal desired velocity
        v_des = control_v(d)

	# 8 calculate delta_heading
	delta_heading = CalHeading(waypoint_heading, heading,current_ind)

        # 9, calculate lateral desired steering angle
        control_steering_angle = control_steering(x_ref, y_ref,vehicle_length,lateral_offset,delta_heading)
        # ending of each loop
        # ===================================
    	return v_des, control_steering_angle,current_x, current_y ,waypoint ,lateral_offset,current_x_F ,current_y_F
       
    #end if
#end main

if __name__ == '__main__':
    try:
        PathPlaningNode()
    except rospy.ROSInterruptException:
        pass
    #end try
#end if
