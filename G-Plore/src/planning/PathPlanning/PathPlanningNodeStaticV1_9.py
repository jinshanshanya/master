#!/usr/bin/env python
# rosrun DecisionControl PathPlanningNode.py
# license @Global Carrier Strike Group
import rospy
from InsMsg.msg import ins_p2
from VehicleMsg.msg import vehicle_download
from VehicleMsg.msg import vehicle_upload
from PlanningMsg.msg import path_data
from CameraMsg.msg import VisObjs
from DisObjMsg.msg import DisObj
# from vehicle_logging.msg import logging_msg
from functionalV1_8 import *
from math import *
#from statemachine import StateMachine, State
# import time
from std_msgs.msg import String
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
 
# from scipy import signal
# the TarAcceler Kp should be tuned.
# setsize should be tuned.
# pure pursuit equation k should be tuned.
# ratio from wheel angle to compute angle should be tuned.
pi = np.pi
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

# pub = rospy.Publisher('PathPlaningNoddPyTopic', vehicle_download, queue_size=100)
pub = rospy.Publisher('vehcileDownloadMsg', vehicle_download, queue_size=100)

# avoid_point = np.empty(shape=[0, 3])  # Obstacle avoidance point


def PathPlaningNode():
    global waypoint
    rospy.Subscriber("insMsg", ins_p2, InsCallback)
    rospy.Subscriber("vehicleUploadMsg", vehicle_upload, VehCallback)
    rospy.Subscriber("DisObjMsg", DisObj, DisObjCallback)
    # rospy.Subscriber("PointMsg", path_data, PathCallback)  # avoid point
    # rospy.Subscriber("logging_msg", logging_msg, LogCallback)
    rospy.init_node('PathPlanningNodeStaticV1_8Py', anonymous=True)
    r = rospy.Rate(100)  # 100hz

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
vehicle_length = 2.85
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
# def PathCallback(data):
#    global avoid_point
#    for i in range(20):
#        x = data.path[i].x
#        y = data.path[i].y
#        heading = data.path[i].heading
#       avoid_point = np.append(avoid_point, [[x, y, heading]], axis=0)
#   print('avoid_point',avoid_point)
def DisObjCallback(data):
    global d
    global distance_lat
    global v_front
    global target_id
    global obstacle_point
    d = data.Long_Pos
    v_front = data.Long_Vel
    target_id = data.ID
    obstacle_x = data.Long_Pos
    obstacle_y = 2 #data.Lat_Pos
    obstacle_point = np.array([[obstacle_x], [obstacle_y]]).reshape(2, 1)
    print('v_front', v_front)
    

def VehCallback(data):
    global v
    global gear
    global drive_mode
    global Act_steering
    v = data.Vehicle_speed
    gear = data.Tar_gearFeed
    drive_mode = data.Drive_Mode
    Act_steering=data.EPS_SteerAng
    print("Act_steering_angle:", Act_steering)

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
    #global current_x
    #global current_y

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
   

    msg = vehicle_download()

    # ------------------------------
    if T < 50:
        msg.Stoprequest = 0
        msg.Tar_gear = 1
        msg.TarAccler = 0  # 0
        msg.Tar_steerangle = 25
    if T >= 50 and T < 100:
        msg.Stoprequest = 0  # 1
        msg.Tar_gear = 1
        msg.TarAccler = 0  # 0
        msg.Tar_steerangle = 25
    if T >= 100 and T < 150:  # gear = 4
        msg.DiveoffReq = 0
        msg.Tar_gear = 4  # 4
        msg.TarAccler = 0  # 0
        msg.Tar_steerangle = 25
        msg.Stoprequest = 0  # 1
    if T >= 150:
        msg.Stoprequest = 0
        msg.DiveoffReq = 1
        msg.Tar_gear = 4  # 4
        #if T > 170 and v > 0:
        if T >180 :
            msg.DiveoffReq = 0
        msg.Tar_steerangle = round(control_steering_angle)  # for reduce error
        # v_des = control_v(v, )
        msg.TarAccler = 0.06 * (v_des - v)  # right # the TarAcceler Kp should be tuned.
        msg.ADS_mode = 3
        msg.Slope = 0

        if current_ind >= 835 and current_ind <= 850:
            msg.TarAccler = -0.4
        # rostime = rospy.get_time()
            rospy.sleep(3.)
            msg.DiveoffReq = 1
            msg.TarAccler = 0.1 * (v_des - v)  # here we use pure P control to calculate desired acceleration
            msg.Tar_steerangle = round(control_steering_angle)
            msg.ADS_mode = 3
        # dc = dc + 1
        # print(dc)
        if current_ind >= 850:
            msg.DiveoffReq = 0
	
        if current_ind >= len(waypoint) - 30 and current_ind < len(waypoint):
            msg.TarAccler = -0.4  # -0.2
            msg.ADS_mode = 3

        if current_ind >= len(waypoint) - 30 and v == 0:
            msg.TarAccler = 0  # 0
            msg.DiveoffReq = 0
            msg.Stoprequest = 1

            msg.ADS_mode = 2
            msg.Tar_gear = 1

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
